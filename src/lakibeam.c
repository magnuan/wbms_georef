#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <time.h>
#include "crc32.h"

#include <string.h>
#include <stdint.h>
#include <math.h>

#include "cmath.h"
#include "wbms_georef.h"
#include "georef_tools.h"
//#include "proj_wrapper.h"
#if defined(_MSC_VER)
#include <io.h>
#endif
#include "lakibeam.h"

static uint8_t verbose = 0;

static offset_t* sensor_offset;
static double lakibeam_epoch;

void lakibeam_set_epoch(double epoch){
    lakibeam_epoch = epoch;
}

void lakibeam_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}

uint8_t lakibeam_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(LAKIBEAM_LIDAR_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<10;test++){     //Test the first 10 packets, if none of them contains bathy data it is pobably not a valid data file
        int len; 
        len = lakibeam_fetch_next_packet(data, fd);
        if (len > 0 ){
            double ts;
            if (lakibeam_identify_packet(data, len, &ts,0.)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

int lakibeam_seek_next_header(int fd){
	char state = 0;
	uint8_t v;
	int n;
	int dump= 0;
    int read_bytes = 0;
	while (read_bytes<(2*LAKIBEAM_LIDAR_PACKET_SIZE)){
		n = read(fd,&v,1);
        read_bytes++;
		if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
		if(n==0){ /*fprintf(stderr,"End of LIDAR stream\n");*/return -1;}
		if(n>0){
			dump += 1;
			switch (state){
				case 0: state = (v==0x37)?1:0;break;    // Factory bytes 0x37,0x40
				case 1: state = (v==0x40)?2:0;break;
				case 2: state = (v==0xFF)?3:0;break;    // Followed by sync signal 0xFF,0xEE
				case 3: state = (v==0xEE)?4:0;break;
			}
			if (state==4){
				dump-=4;
				if(dump) fprintf(stderr,"Lakibeam seek dump %d bytes\n",dump);
				return 0;	
			}
		}
	}
	return -1;
}

int lakibeam_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;

	if(lakibeam_seek_next_header(fd)) return 0;
	data[0] = 0xFF;data[1] = 0xEE;
	rem = 1206-2-2; //Fetch packet header (minus the sync we allready have, and the 2 factory bytes)
	dp = &(data[2]);
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types 
	if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
	if(n==0){ /*fprintf(stderr,"End of stream\n");*/return 0;}
	return 1206;;
}

int lakibeam_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in){
	uint16_t * vdata = (uint16_t*) databuffer;
	uint32_t ii;
	
	double lidar_ts = 1e-6*(((uint32_t*)databuffer)[1200/4]); /* Time in seconds since startup */
    lidar_ts += lakibeam_epoch;
	*ts_out = lidar_ts; 

	if (len != 1206){
		fprintf(stderr,"odd sized lakibeam packet received, discarding, len=%d\n",len);
		return 0;
	}
	for (ii=0;ii<12;ii++){              //Each packet has 12 datablocks of 100 bytes, each starting with 0xeeff of 0xffff
		if ((vdata[ii*50])!=(0xeeff) && (vdata[ii*50])!=(0xffff)){
			if(verbose) fprintf(stderr,"malformed lakibeam packet received, discarding\n");
			fprintf(stderr,"malformed lakibeam packet received, discarding\n");
			return 0;
		}
	}
	return 1;
}


#define LIDAR_DP (16*2*12)

uint32_t  lakibeam_count_data( uint16_t * data,double *ts){
    uint32_t ix_out = 0;
	*ts = 1e-6*(((uint32_t*)data)[1200]); /* Time in seconds since startup */
    
    for (uint16_t block=0;block<12;block++){
		for (uint16_t set = 0;set<2;set++){ // [Strongest, Last]
		    uint8_t* dp = (uint8_t*)(&(data[block*50+2])) + (set*3); //Set 1 os offset 3 bytes wrt set 0

			for (uint16_t ch = 0; ch<16;ch++){
				float sensor_r = (float)(((uint16_t)dp[0])+((uint16_t)dp[1])*256)*1e-3f;	//Lakibeam measures distance in units of 1mm 
                if (sensor_r>0){       
                    ix_out++;
                }
				dp+=6; // Each measuring result is 6 Bytes 
			}
		}
	}
	return ix_out;
}

uint32_t lakibeam_georef_data( uint16_t* data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/  output_data_t* outbuf){
    double* x = &(outbuf->x[0]);
    double* y = &(outbuf->y[0]);
    double* z = &(outbuf->z[0]);
    float* intensity = &(outbuf->i[0]);
    float* range = &(outbuf->range[0]);
    float* az_out = &(outbuf->teta[0]);
    float* el_out = &(outbuf->steer[0]);
    
    static uint32_t ping_number = 0;

    uint16_t ix_in_stride = MAX(1,sensor_params->beam_decimate); 
    const uint32_t ping_number_stride = sensor_params->ping_decimate; 

    ping_number++;
    
    if ((ping_number%ping_number_stride) != 0) return 0;

	float sensor_r,sensor_az;
	float xs[LIDAR_DP];
	float ys[LIDAR_DP];
	float zs[LIDAR_DP];

	double nav_x, nav_y, nav_z; 			/*Position in global coordinates (east,north,up)*/
	float nav_yaw,  nav_pitch,  nav_roll; /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;

	uint16_t Nout;
	uint16_t ix_out;

	double lidar_ts = 1e-6*(((uint32_t*)data)[1200/4]); /* Time in seconds since startup */
    lidar_ts += lakibeam_epoch;

    /*for (uint16_t ix=0;ix<128;ix++){
        fprintf(stderr,"0x%02x, ", ((uint8_t*)(data))[ix]);
    }
    fprintf(stderr,"\n");*/

    //fprintf(stderr,"Lakibeam ts=%f\n",lidar_ts); 
    if (calc_interpolated_nav_data( posdata, pos_ix, lidar_ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt )) return 0;
    if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }
	
	float d_azimuth;
	if (data[51] < data[1])
		d_azimuth = ((float)(36000+data[51]-data[1]))*(100*M_PI/(16*100*180)); // Convert to radians from centidegrees
	else
		d_azimuth = ((float)(data[51]-data[1]))*(M_PI/(16*100*180)); // Convert to radians from centidegrees

	ix_out = 0;
    uint32_t ix_in = 0;
	// Populate r,az,el with data from lidar
	for (uint16_t block=0;block<12;block++){
        if(data[block*50]!=(0xeeff)) continue;
		float azimuth = (float)data[block*50+1]*(M_PI/(100*180)); //0 to 2pi (Angle for first data block)
		//fprintf(stderr,"Az = %f deg\n ",azimuth*180/M_PI);
		
		for (uint16_t set = 0;set<2;set++){ // [Strongest, Last]
		    uint8_t* dp = (uint8_t*)(&(data[block*50+2])) + (set*3); //Set 1 os offset 3 bytes wrt set 0

			for (uint16_t ch = 0; ch<16;ch++){
				sensor_r = (float)(((uint16_t)dp[0])+((uint16_t)dp[1])*256)*1e-3f;	//Lakibeam measures distance in units of 1mm 
		        //fprintf(stderr,"Range = %f m\n ",sensor_r);
				sensor_az = azimuth + d_azimuth*ch;
				if (sensor_r>sensor_params->min_range && sensor_r<sensor_params->max_range){
					if ( (sensor_az>sensor_params->min_azimuth && sensor_az<sensor_params->max_azimuth )){
                        ix_in++;
                        if ((ix_in % ix_in_stride)==0){
                            xs[ix_out] = 0;
                            ys[ix_out] = -sensor_r * sinf(sensor_az); //Sign flipped compared to standard right hand system
                            zs[ix_out] = sensor_r * cosf(sensor_az);
                            intensity[ix_out] = dp[2];
                            range[ix_out] = sensor_r;
                            az_out[ix_out] = sensor_az;
                            el_out[ix_out] = 0;
                            ix_out++;
                        }
					}
				}
				dp+=6; // Each measuring result is 6 Bytes 
			}
		}
	}
	//fprintf(stderr,"\n");
	Nout = ix_out;

	georef_to_global_frame(sensor_offset, xs, ys, zs, Nout,0, nav_x, nav_y, nav_z, nav_yaw, nav_pitch,  nav_roll, ray_trace_none,  sensor_params->mounting_depth,/*OUTPUT*/ x,y,z);
	//fprintf(stderr,"Georef lidar data\n");	
    outbuf->N = Nout;
	return Nout;
}
