#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "georef_tools.h"
#include "wbms_georef.h"
#include <math.h>
#include "cmath.h"


#define OUTPUT_ANGLES_IN_DEGREES


int sprint_output_format_sbf(char* str,output_format_e* format){
    char* sptr = str;
    int cnt;
    int sfcnt=1;
    output_format_e* fptr = format;
    for(cnt = 0;cnt < MAX_OUTPUT_FIELDS;cnt++){
        if(*fptr == none){
            return sptr-str;
        }
        if(*fptr == x) {fptr++;continue;} //x,y,and z is mandatory in SBF, and not written to metafile
        if(*fptr == y) {fptr++;continue;} //x,y,and z is mandatory in SBF, and not written to metafile
        if(*fptr == z) {fptr++;continue;} //x,y,and z is mandatory in SBF, and not written to metafile
        switch (*fptr){
#define foo(x) case x: sptr+=sprintf(sptr,"SF%d=%s\n",sfcnt++,STRINGIFY(x));break
            iterate_output_format(foo);
#undef foo
            default: break;
        }
        fptr++;
    }
    return sptr-str;
}

uint16_t output_format_count_fields(output_format_e format[MAX_OUTPUT_FIELDS]){
    uint16_t cnt = 0; 
    for(uint16_t jj=0; jj<MAX_OUTPUT_FIELDS; jj++){
        if (format[jj]==none) break;
        cnt++;
    }
    return cnt;
}

int write_sbf_meta_to_buffer(output_format_e format[MAX_OUTPUT_FIELDS], uint64_t point_count, double x_offset, double y_offset, double z_offset,  /*OUTPUT*/char* outbuf){
	uint32_t len;
    uint16_t field_count = output_format_count_fields(format);
	len = 0;
    len += sprintf(&(outbuf[len]),"[SBF]\n");
    len += sprintf(&(outbuf[len]),"Points=%ld\n", point_count);
    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up
    len += sprintf(&(outbuf[len]),"GlobalShift=%f, %f, %f\n",-y_offset,-x_offset,z_offset);
    len += sprintf(&(outbuf[len]),"SFCount=%d\n",field_count-3);
    len += sprint_output_format_sbf(&(outbuf[len]),format);
    return len;
}


//CloudCompare SBF-format seems to be big-endian
#ifndef __unix__
#include <stdlib.h>
#define __border_16(x) _byteswap_ushort(x)
#define __border_32(x) _byteswap_ulong(x)
#define __border_64(x) _byteswap_uint64(x)
#else
#include <byteswap.h>
#define __border_64(x) __bswap_64(x)
#define __border_32(x) __bswap_32(x)
#define __border_16(x) __bswap_16(x)
#endif

inline static void write_f64_unaligned_bswap(uint8_t* dst, double x){
    uint8_t* src = (uint8_t*) &x;
    for (uint16_t ix = 0;ix<sizeof(double);ix++){
        dst[sizeof(double)-ix-1] = src[ix];
    }
}
inline static void write_f32_unaligned_bswap(uint8_t* dst, float x){
    uint8_t* src = (uint8_t*) &x;
    for (uint16_t ix = 0;ix<sizeof(float);ix++){
        dst[sizeof(float)-ix-1] = src[ix];
    }
}

int write_sbf_header_to_buffer(output_format_e format[MAX_OUTPUT_FIELDS], uint64_t point_count, double x_offset, double y_offset, double z_offset,  /*OUTPUT*/char* outbuf){
	char * dp = outbuf;
    uint16_t field_count = output_format_count_fields(format);
    //SBF header flag 
    *((uint8_t*)dp) = 42; dp+=1;
    *((uint8_t*)dp) = 42; dp+=1;
    //Point count (Np) 
    *((uint64_t*)dp) = __border_64(point_count); dp+=8;
    //Scalar field count (Ns) 
    *((uint16_t*)dp) = __border_16(field_count-3); dp+=2;        //x,y,z is implicit, and does not count in the field count
    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up
    //X coordinate shift (should be added to all 32bit coordinates stored in the file)
    write_f64_unaligned_bswap((uint8_t*)dp,y_offset); dp+=8;
    //Y coordinate shift (should be added to all 32bit coordinates stored in the file)
    write_f64_unaligned_bswap((uint8_t*)dp,x_offset); dp+=8;
    //Z coordinate shift (should be added to all 32bit coordinates stored in the file)
    write_f64_unaligned_bswap((uint8_t*)dp,-z_offset); dp+=8;
    //Reserved for later 
    memset(dp,0,28); dp+= 28;

    return dp-outbuf;
}

int write_sbf_to_buffer(double x_offset, double y_offset, double z_offset,double ts_offset, double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf){
    double* x_val = &(data->x[0]);
    double* y_val = &(data->y[0]);
    double* z_val = &(data->z[0]);
    float* z_var_val = &(data->z_var[0]);
    float* intensity_val = &(data->i[0]);
    float* quality_val = &(data->quality[0]);
    float* strength_val = &(data->strength[0]);
    float* range_val = &(data->range[0]);
    float* teta_val = &(data->teta[0]);
    float* steer_val = &(data->steer[0]);
    int * beam_number = &(data->beam[0]);
    float* swath_y_val = &(data->swath_y[0]);
    float* aoi_val = &(data->aoi[0]);
    float* up_gate = &(data->up_gate[0]);
    float* low_gate = &(data->low_gate[0]);
    float* tx_angle = &(data->tx_angle);
    float* sv = &(data->sv);
    float* tx_freq = &(data->tx_freq);
    int*   multiping_index = &(data->multiping_index);
    int*   multifreq_index = &(data->multifreq_index);

	char * dp = outbuf;
	uint32_t ii,jj;
    pos_ix = find_closest_index_in_posdata(posdata,pos_ix, ts);
    navdata_t* pos = &(posdata[pos_ix]);
    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up
	for (ii = 0;ii<n;ii++){
        for(jj=0; jj<MAX_OUTPUT_FIELDS; jj++){
            if (format[jj]==none) break;
            switch(format[jj]){
                case none: break;
                case x: write_f32_unaligned_bswap((uint8_t*)dp,(float)(y_val[ii]-y_offset)); dp+=4;break;
                case y: write_f32_unaligned_bswap((uint8_t*)dp,(float)(x_val[ii]-x_offset)); dp+=4;break;
                case z: write_f32_unaligned_bswap((uint8_t*)dp,(float)(-(z_val[ii]-z_offset))); dp+=4;break;
                
                case z_var: write_f32_unaligned_bswap((uint8_t*)dp,(float)(z_var_val[ii])); dp+=4;break;
                case z_stddev: write_f32_unaligned_bswap((uint8_t*)dp,(float)(sqrtf(z_var_val[ii]))); dp+=4;break;
                #ifdef OUTPUT_ANGLES_IN_DEGREES
                case teta: write_f32_unaligned_bswap((uint8_t*)dp,(float)(teta_val[ii]*180/M_PI)); dp+=4;break;
                case steer: write_f32_unaligned_bswap((uint8_t*)dp,(float)(steer_val[ii]*180/M_PI)); dp+=4;break;
                case el: write_f32_unaligned_bswap((uint8_t*)dp,(float)((*tx_angle)*180/M_PI)); dp+=4;break;
                case aoi: write_f32_unaligned_bswap((uint8_t*)dp,(float)(aoi_val[ii]*180/M_PI)); dp+=4;break;
                #else
                case teta: write_f32_unaligned_bswap((uint8_t*)dp,(float)(teta_val[ii])); dp+=4;break;
                case steer: write_f32_unaligned_bswap((uint8_t*)dp,(float)(steer_val[ii])); dp+=4;break;
                case el: write_f32_unaligned_bswap((uint8_t*)dp,(float)(*tx_angle)); dp+=4;break;
                case aoi: write_f32_unaligned_bswap((uint8_t*)dp,(float)(aoi_val[ii])); dp+=4;break;
                #endif
                case beam: write_f32_unaligned_bswap((uint8_t*)dp,(float)(beam_number[ii])); dp+=4;break; //TODO remove this modulo 4 when no more needed
                case range: write_f32_unaligned_bswap((uint8_t*)dp,(float)(range_val[ii])); dp+=4;break;
                case val: write_f32_unaligned_bswap((uint8_t*)dp,(float)(intensity_val[ii])); dp+=4;break;
                case swath_y: write_f32_unaligned_bswap((uint8_t*)dp,(float)(swath_y_val[ii])); dp+=4;break;
                case quality: write_f32_unaligned_bswap((uint8_t*)dp,(float)(quality_val[ii])); dp+=4;break;
                case strength: write_f32_unaligned_bswap((uint8_t*)dp,(float)(strength_val[ii])); dp+=4;break;
                
                case cgate: write_f32_unaligned_bswap((uint8_t*)dp,(float)((range_val[ii]-0.5*(low_gate[ii]+up_gate[ii]))*200/(low_gate[ii]+up_gate[ii]))); dp+=4;break;
                //case ugate: write_f32_unaligned_bswap((uint8_t*)dp,(float)((range_val[ii]-up_gate[ii])*100/range_val[ii])); dp+=4;break;
                //case lgate: write_f32_unaligned_bswap((uint8_t*)dp,(float)((low_gate[ii]-range_val[ii])*100/range_val[ii])); dp+=4;break;
                //case gatew: write_f32_unaligned_bswap((uint8_t*)dp,(float)((low_gate[ii]-up_gate[ii])*200/(low_gate[ii]+up_gate[ii]))); dp+=4;break;
                case ugate: write_f32_unaligned_bswap((uint8_t*)dp,(float)up_gate[ii]); dp+=4;break;
                case lgate: write_f32_unaligned_bswap((uint8_t*)dp,(float)low_gate[ii]); dp+=4;break;
                case gatew: write_f32_unaligned_bswap((uint8_t*)dp,(float)(low_gate[ii]-up_gate[ii])); dp+=4;break;
                
                case t: write_f32_unaligned_bswap((uint8_t*)dp,(float)(ts-ts_offset)); dp+=4;break;
                case c: write_f32_unaligned_bswap((uint8_t*)dp,(float)(*sv)); dp+=4;break;
                
                case freq: write_f32_unaligned_bswap((uint8_t*)dp,(float)(*tx_freq)); dp+=4;break;
                case multiping: write_f32_unaligned_bswap((uint8_t*)dp,(float)(*multiping_index)); dp+=4;break;
                case multifreq: write_f32_unaligned_bswap((uint8_t*)dp,(float)(*multifreq_index)); dp+=4;break;

                case X: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->y-y_offset)); dp+=4;break;
                case Y: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->x-x_offset)); dp+=4;break;
                case Z: write_f32_unaligned_bswap((uint8_t*)dp,(float)(-(pos->z-z_offset))); dp+=4;break;
                
                case HOR_ACC: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->hor_accuracy)); dp+=4;break;
                case VERT_ACC: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->vert_accuracy)); dp+=4;break;
                
                #ifdef OUTPUT_ANGLES_IN_DEGREES
                case LAT: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->lat)*180/M_PI); dp+=4;break;
                case LON: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->lon)*180/M_PI); dp+=4;break;
                case YAW: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->yaw)*180/M_PI); dp+=4;break;
                case PITCH: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->pitch)*180/M_PI); dp+=4;break;
                case ROLL: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->roll)*180/M_PI); dp+=4;break;
                #else
                case LAT: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->lat)); dp+=4;break;
                case LON: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->lon)); dp+=4;break;
                case YAW: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->yaw)); dp+=4;break;
                case PITCH: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->pitch)); dp+=4;break;
                case ROLL: write_f32_unaligned_bswap((uint8_t*)dp,(float)(pos->roll)); dp+=4;break;
                #endif
            }
        }
    }
    return dp-outbuf;    
}
