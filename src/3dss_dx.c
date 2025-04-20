#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
#ifdef __unix__
#include <unistd.h>
#endif
#include <time.h>

#include <string.h>
#include <stdint.h>
#include "3dss_dx.h"
#include "time_functions.h"
#include "nmea_nav.h"

#include <math.h>
#include "wbms_georef.h"
#include "georef_tools.h"
#include "intensity_scaling.h"
#include "cmath.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif
#define ROLL_VECTOR_LEN 512
#define ROLL_VECTOR_RATE 500.

static uint8_t verbose = 1;

static offset_t* sensor_offset;


void p3dss_init(void){
}

void p3dss_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}



/* For now a really simple test to see if we can find the magic preamble in the datastream
 * No check that the data packet acctually contains sensor and/or navigation data*/ 
uint8_t p3dss_test_nav_file(int fd){
	if(p3dss_seek_next_header(fd)) return 0;
    else return 1;
}
uint8_t p3dss_test_bathy_file(int fd){
	if(p3dss_seek_next_header(fd)) return 0;
    else return 1;
}

//Copy-paste from wbms header seek, could probably be made both faster and simpler
int p3dss_seek_next_header(int fd){
    //printf("p3dss_seek_next_header\n");
	char state = 0;
	uint8_t v=0;
	int n;
	int dump= 0;
	//int position = lseek(fd, 0, SEEK_CUR);
	//fprintf(stderr,"\nseek header start at 0x%08x\n",position);
    int read_bytes = 0;
	while (read_bytes<(MAX_3DSS_PACKET_SIZE+16)){
		n = read(fd,&v,1);
        read_bytes++;
		if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
		if(n==0){ /*fprintf(stderr,"End of 3DSS stream\n");*/return -1;}
		if(n>0){
			//fprintf(stderr,"%02x ",v);
			dump += 1;
            
			switch (state){
				case 0: state = (v==0x50)?1:0;break;
				case 1: state = (v==0x49)?2:0;break;
				case 2: state = (v==0x4e)?3:0;break;
				case 3: state = (v==0x47)?4:0;break;
				case 4: state = (v==0x27)?5:0;break;
				case 5: state = (v==0x2b)?6:0;break;
				case 6: state = (v==0x3a)?7:0;break;
				case 7: state = (v==0xd8)?8:0;break;
				case 8: state = (v==0x74)?9:0;break;
				case 9: state = (v==0x2a)?10:0;break;
				case 10: state = (v==0x1c)?11:0;break;
				case 11: state = (v==0x33)?12:0;break;
				case 12: state = (v==0xe9)?13:0;break;
				case 13: state = (v==0xb0)?14:0;break;
				case 14: state = (v==0x73)?15:0;break;
				case 15: state = (v==0xb1)?16:0;break;
			}
			if (state==16){
				dump-=16;
				if(dump>0)  {
					fprintf(stderr,"3DSS seek dump %d bytes\n",dump);
				}
				return 0;	
			}
		}
	}
    //printf("p3dss_seek_next_header end -1\n");
	return -1;
}

#include <errno.h>

int p3dss_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;
	//fprintf(stderr,"p3dss_fetch_next_packet\n");
	if(p3dss_seek_next_header(fd)) return 0;
	//Fetch main header
    uint32_t data_count;
	rem = sizeof(data_count); //Fetch remainder of DxHeader, that is the data_count field
	dp = (char*) &(data_count);
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types 
	//fprintf(stderr,"Reading %d byte 3DSS packet\n",data_count);
	//Fetch sub header and  payload
	rem = data_count; 
    dp = data;
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types
	//fprintf(stderr,"%d bytes read from %d\n",data_count,fd);
	return data_count;
}


int p3dss_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out){
    p3dss_DxData_t * data = (p3dss_DxData_t*) databuffer;
 	
    double ts = (double)(data->time.seconds) + (double)(data->time.nanoseconds)/1e9;
    *ts_out = ts;
    /*fprintf(stderr,"3DSS data: ts=%ld.%03d id=%s  size=%d\n", \
            data->time.seconds, data->time.nanoseconds/1000000,\
            data->system_info.id,\
            len\
            );
    */
    return 1;
}


/**
*
* This function is assuming that position and attitude data comes synched, with same time stamp. As from an integrated navigation system
* For more generic streams, where attitude and position messages might come from different sources, at different time and rate. This will not work
*/
int p3dss_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
    p3dss_DxData_t * data = (p3dss_DxData_t*) databuffer;

 	double ts = (double)(data->time.seconds) + (double)(data->time.nanoseconds)/1e9;
    /*fprintf(stderr,"p3dss_georef_data: ascii_cnt=%d prt_ss_cnt=%d stb_ss_cnt=%d prt_ss3d_cnt=%d stb_ss3d_cnt=%d prt_bath_cnt=%d stb_bath_cnt=%d sbg_cnt=%d\n", \
        data->ascii_sentence_count, \
        data->port_sidescan_count, \
        data->starboard_sidescan_count, \
        data->port_sidescan3d_count, \
        data->starboard_sidescan3d_count, \
        data->port_bathymetry_count, \
        data->starboard_bathymetry_count, \
        data->sbg_message_count \
        );*/

    int ret = NO_NAV_DATA;
    for (uint32_t n=0; n<data->ascii_sentence_count;n++){
            p3dss_Ascii_t *asc = (p3dss_Ascii_t *)(databuffer + data->ascii_sentence_offset + n*sizeof(p3dss_Ascii_t));
            if (asc->sentence[0]=='$'){  //Process NMEA messages
                //fprintf(stderr,"%f - %f  %s\n",ts,(double)(asc->time.seconds)+(double)(asc->time.nanoseconds)/1e9,asc->sentence);
                double temp_ts_out;
                int r = nmea_nav_process_nav_packet(asc->sentence, strlen(asc->sentence),ts, &temp_ts_out, z_offset, alt_mode, proj, navdata, aux_navdata);
                if (r != NO_NAV_DATA){
                    ret = r;
                    *ts_out = temp_ts_out;
                }
            }
        
    }
    return ret;
}




uint32_t p3dss_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,  /*OUTPUT*/ output_data_t* outbuf){

    double* x = &(outbuf->x[0]);
    double* y = &(outbuf->y[0]);
    double* z = &(outbuf->z[0]);
    //float* z_var = &(outbuf->z_var[0]);
    float* intensity = &(outbuf->i[0]);
    float* beam_range = &(outbuf->range[0]);
    float* beam_angle = &(outbuf->teta[0]);
    //float* beam_steer = &(outbuf->steer[0]);
    int * beam_number = &(outbuf->beam[0]);
    //float* swath_y = &(outbuf->swath_y[0]);
    //float* aoi = &(outbuf->aoi[0]);
    //float* upper_gate_range = &(outbuf->up_gate[0]);
    //float* lower_gate_range = &(outbuf->low_gate[0]);
    //float* quality = &(outbuf->quality[0]);
    //int*   quality_flags = &(outbuf->quality_flags[0]);
    //float* priority = &(outbuf->priority[0]);
    //float* tx_angle_out = &(outbuf->tx_angle);
    //float* fs_out = &(outbuf->sample_rate);
    //float* ping_rate_out = &(outbuf->ping_rate);
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    float* tx_plen_out = &(outbuf->tx_plen);
    float* tx_bw_out = &(outbuf->tx_bw);
    float* tx_voltage_out = &(outbuf->tx_voltage);
    int* ping_number_out = &(outbuf->ping_number);
    //int* multiping_index_out = &(outbuf->multiping_index);
    //int* multifreq_index_out = &(outbuf->multifreq_index);
    //int* classification_val = &(outbuf->classification[0]);
	uint32_t ix_out;
    
    p3dss_DxData_t * data = (p3dss_DxData_t*) databuffer;

 	double ts = (double)(data->time.seconds) + (double)(data->time.nanoseconds)/1e9;
    ts += sensor_offset->time_offset;
    
 	float sv_face = data->parameters.sound_velocity.face;
    float sv_bulk = data->parameters.sound_velocity.bulk;
    
    float sv = sv_face;
    if (sensor_params->force_sv > 0){
        sv = sensor_params->force_sv;
    }

    *sv_out = sv;

    /*fprintf(stderr,"p3dss_georef_data: ascii_cnt=%d prt_ss_cnt=%d stb_ss_cnt=%d prt_ss3d_cnt=%d stb_ss3d_cnt=%d prt_bath_cnt=%d stb_bath_cnt=%d sbg_cnt=%d\n", \
        data->ascii_sentence_count, \
        data->port_sidescan_count, \
        data->starboard_sidescan_count, \
        data->port_sidescan3d_count, \
        data->starboard_sidescan3d_count, \
        data->port_bathymetry_count, \
        data->starboard_bathymetry_count, \
        data->sbg_message_count \
        );*/
    
    //Calculate interpolated navigation data 
	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    if (calc_interpolated_nav_data( posdata, pos_ix, ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
        if(verbose) fprintf(stderr, "Could not find navigation data for 3DSS bathy record at time %f\n",ts);
        return 0;
    }

    float max_two_way_time=0.5f; //TODO calculate this from data
    float roll_vector[ROLL_VECTOR_LEN];
    float z_vector[ROLL_VECTOR_LEN];
    calc_interpolated_roll_and_z_vector(posdata, pos_ix, ts, max_two_way_time, ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
   
    
    float *xs;
    float *ys;
    float *zs;
    
    ix_out = 0;
    #define MAX_RESPONSE_ANGLE (70*M_PI/180)
    if(0){ // Process Bathy data record
        size_t Nn = data->port_bathymetry_count + data->starboard_bathymetry_count;
        xs = (float*) malloc(Nn*sizeof(float));
        ys = (float*) malloc(Nn*sizeof(float));
        zs = (float*) malloc(Nn*sizeof(float));

        for (uint32_t n=0; n<data->port_bathymetry_count;n++){
            p3dss_BathymetryPoint_t *bp = (p3dss_BathymetryPoint_t *)(databuffer + data->port_bathymetry_offset + n*sizeof(p3dss_BathymetryPoint_t));
            beam_range[ix_out] = bp->range;
            beam_angle[ix_out] = -(MAX_RESPONSE_ANGLE) - bp->angle;  //Convert from 0deg=port -90deg=down to -90deg=port 0deg=down  
            intensity[ix_out] = bp->amplitude;
            ix_out++;
        }
        for (uint32_t n=0; n<data->starboard_bathymetry_count;n++){
            p3dss_BathymetryPoint_t *bp = (p3dss_BathymetryPoint_t *)(databuffer + data->starboard_bathymetry_offset + n*sizeof(p3dss_BathymetryPoint_t));
            beam_range[ix_out] = bp->range;
            beam_angle[ix_out] = bp->angle;
            beam_angle[ix_out] = (MAX_RESPONSE_ANGLE) + bp->angle;  //Convert from 0deg=stb. -90deg=down to 90deg=stb 0deg=down  
            intensity[ix_out] = bp->amplitude;
            ix_out++;
        }
    }
    else{ // Process 3D sidescan data record
        size_t Nn = data->port_sidescan3d_count + data->starboard_sidescan3d_count;
        xs = (float*) malloc(Nn*sizeof(float));
        ys = (float*) malloc(Nn*sizeof(float));
        zs = (float*) malloc(Nn*sizeof(float));

        for (uint32_t n=0; n<data->port_sidescan3d_count;n++){
            p3dss_Sidescan3DPoint_t *bp = (p3dss_Sidescan3DPoint_t *)(databuffer + data->port_sidescan3d_offset + n*sizeof(p3dss_Sidescan3DPoint_t));
            beam_range[ix_out] = bp->range;
            beam_angle[ix_out] = -(MAX_RESPONSE_ANGLE) - bp->angle;  //Convert from 0deg=port -90deg=down to -90deg=port 0deg=down  
            intensity[ix_out] = bp->amplitude;
            ix_out++;
        }
        for (uint32_t n=0; n<data->starboard_sidescan3d_count;n++){
            p3dss_Sidescan3DPoint_t *bp = (p3dss_Sidescan3DPoint_t *)(databuffer + data->starboard_sidescan3d_offset + n*sizeof(p3dss_Sidescan3DPoint_t));
            beam_range[ix_out] = bp->range;
            beam_angle[ix_out] = bp->angle;
            beam_angle[ix_out] = (MAX_RESPONSE_ANGLE) + bp->angle;  //Convert from 0deg=stb. -90deg=down to 90deg=stb 0deg=down  
            intensity[ix_out] = bp->amplitude;
            ix_out++;
        }

    }
    uint32_t Nout = ix_out;
    for (uint32_t n=0; n<Nout;n++){
        xs[n] = 0;
        ys[n] = beam_range[n] * sin(beam_angle[n]);
        zs[n] = beam_range[n] * cos(beam_angle[n]);
        beam_number[n] = n;
    }

    georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,sv_face, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
    

    free(xs);free(ys);free(zs);
    outbuf->N = Nout;
	return Nout;

}

