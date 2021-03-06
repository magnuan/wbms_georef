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


int sprint_output_format(char* str,output_format_e* format){
    char* sptr = str;
    int cnt;
    output_format_e* fptr = format;
    for(cnt = 0;cnt < MAX_OUTPUT_FIELDS;cnt++){
        if(*fptr == none){
            sptr+=sprintf(sptr,"\n");
            return sptr-str;
        }
        if(cnt) sptr += sprintf(sptr,",");
        switch (*fptr){
#define foo(x) case x: sptr+=sprintf(sptr,STRINGIFY(x));break
            iterate_output_format(foo);
#undef foo
            default: break;
        }
        fptr++;
    }
    sptr+=sprintf(sptr,"\n");
    return sptr-str;
}

int write_csv_header_to_buffer(output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf){
    return sprint_output_format(outbuf,format);
}

int write_csv_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf){
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

	uint32_t ii,jj,len;
	len = 0;
    pos_ix = find_closest_index_in_posdata(posdata,pos_ix, ts);
    navdata_t* pos = &(posdata[pos_ix]);
    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up
	for (ii = 0;ii<n;ii++){
        for(jj=0; jj<MAX_OUTPUT_FIELDS; jj++){
            if (format[jj]==none) break;
            if(jj)  len += sprintf(&(outbuf[len]),", ");
            switch(format[jj]){
                case none: break;
                case x: len += sprintf(&(outbuf[len]),"%11.3f",y_val[ii]);break;
                case y: len += sprintf(&(outbuf[len]),"%11.3f",x_val[ii]);break;
                case z: len += sprintf(&(outbuf[len]),"%11.3f",-z_val[ii]);break;
                case z_var: len += sprintf(&(outbuf[len]),"%11.3f",z_var_val[ii]);break;
                case z_stddev: len += sprintf(&(outbuf[len]),"%11.3f",sqrtf(z_var_val[ii]));break;
                case teta: len += sprintf(&(outbuf[len]),"%11.3f",teta_val[ii]*180/M_PI);break;
                case beam: len += sprintf(&(outbuf[len]),"%11d",beam_number[ii]);break;
                case range: len += sprintf(&(outbuf[len]),"%11.3f",range_val[ii]);break;
                case steer: len += sprintf(&(outbuf[len]),"%11.3f",steer_val[ii]*180/M_PI);break;
                case el: len += sprintf(&(outbuf[len]),"%11.3f",(*tx_angle)*180/M_PI);break;
                case val: len += sprintf(&(outbuf[len]),"%11.3f",intensity_val[ii]);break;
                case quality: len += sprintf(&(outbuf[len]),"%11.3f",quality_val[ii]);break;
                case strength: len += sprintf(&(outbuf[len]),"%11.3f",strength_val[ii]);break;
                case swath_y: len += sprintf(&(outbuf[len]),"%11.3f",swath_y_val[ii]);break;
                case aoi: len += sprintf(&(outbuf[len]),"%11.3f",aoi_val[ii]*180/M_PI);break;
                case cgate: len += sprintf(&(outbuf[len]),"%11.3f",(range_val[ii]-0.5*(low_gate[ii]+up_gate[ii]))*200/(low_gate[ii]+up_gate[ii]));break;
                case ugate: len += sprintf(&(outbuf[len]),"%11.3f",(range_val[ii]-up_gate[ii])*100/range_val[ii]);break;
                case lgate: len += sprintf(&(outbuf[len]),"%11.3f",(low_gate[ii]-range_val[ii])*100/range_val[ii]);break;
                case gatew: len += sprintf(&(outbuf[len]),"%11.3f",(low_gate[ii]-up_gate[ii])*200/(low_gate[ii]+up_gate[ii]));break;
                case t: len += sprintf(&(outbuf[len]),"%12.5f",ts);break;
                case c: len += sprintf(&(outbuf[len]),"%8.3f",*sv);break;
                
                case freq: sprintf(&(outbuf[len]),"%11.3f",*tx_freq);break;
                case multiping: sprintf(&(outbuf[len]),"%11d",*multiping_index);break;
                case multifreq: sprintf(&(outbuf[len]),"%11d",*multifreq_index);break;
                
                case X: len += sprintf(&(outbuf[len]),"%11.3f",pos->y);break;
                case Y: len += sprintf(&(outbuf[len]),"%11.3f",pos->x);break;
                case Z: len += sprintf(&(outbuf[len]),"%11.3f",-pos->z);break;
                case HOR_ACC: len += sprintf(&(outbuf[len]),"%11.3f",pos->hor_accuracy);break;
                case VERT_ACC: len += sprintf(&(outbuf[len]),"%11.3f",pos->vert_accuracy);break;
                case YAW: len += sprintf(&(outbuf[len]),"%11.3f",pos->yaw*180/M_PI);break;
                case PITCH: len += sprintf(&(outbuf[len]),"%11.3f",pos->pitch*180/M_PI);break;
                case ROLL: len += sprintf(&(outbuf[len]),"%11.3f",pos->roll*180/M_PI);break;

            }
        }
        len += sprintf(&(outbuf[len]),"\n");
    }
        
	return len;
}

int write_csv_proj_string_to_buffer(char* str, /*OUTPUT*/char* outbuf){
	char * dp;
	dp = outbuf;
    dp += sprintf(dp,"Projection string: %s\n",str);
    return dp-outbuf;
}
