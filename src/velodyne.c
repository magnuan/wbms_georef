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
#include "velodyne.h"

static uint8_t verbose = 0;

uint8_t velodyne_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(VELODYNE_LIDAR_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<10;test++){     //Test the first 10 packets, if none of them contains bathy data it is pobably not a valid data file
        int len; 
        len = velodyne_fetch_next_packet(data, fd);
        if (len > 0 ){
            double ts;
            if (velodyne_identify_packet(data, len, &ts,0.)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

int velodyne_seek_next_header(int fd){
	char state = 0;
	uint8_t v;
	int n;
	int dump= 0;
    int read_bytes = 0;
	while (read_bytes<(2*VELODYNE_LIDAR_PACKET_SIZE)){
		n = read(fd,&v,1);
        read_bytes++;
		if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
		if(n==0){ fprintf(stderr,"End of LIDAR stream\n");return -1;}
		if(n>0){
			dump += 1;
			switch (state){
				case 0: state = (v==0xFF)?1:0;break;
				case 1: state = (v==0xEE)?2:0;break;
			}
			if (state==2){
				dump-=2;
				if(dump) fprintf(stderr,"Lidar seek dump %d bytes\n",dump);
				return 0;	
			}
		}
	}
	return -1;
}

int velodyne_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;

	//Find $GRP preamble
	if(velodyne_seek_next_header(fd)) return 0;
	data[0] = 0xFF;data[1] = 0xEE;
	rem = 1206-2; //Fetch packet header (minus the preamble we allready have)
	dp = &(data[2]);
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types 
	if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
	if(n==0){ fprintf(stderr,"End of stream\n");return 0;}
	return 1206;;
}

int velodyne_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in){
	uint16_t * vdata = (uint16_t*) databuffer;
	uint32_t ii;
	//TODO Check that the ts readout is correct
	
	double lidar_ts = 1e-6*(((uint32_t*)databuffer)[25*12]); /* Time in seconds since last hour mark */
	if ((lidar_ts<-1.) || (lidar_ts>3601.)) return 0;
    lidar_ts += (floor((ts_in)/(3600)))*3600;			/* Adding number of hours since UNIX epoch */
	if((lidar_ts-(ts_in))>(1800.)) lidar_ts -= 3600.;	/* If lidar_ts now is more than 30 min ahead of pos_ts, we probably had lidar right before hour mark when pos passed, subtract one hour*/
	*ts_out = lidar_ts; 

	if (len != 1206){
		fprintf(stderr,"odd sized velodyne packet received, discarding, len=%d\n",len);
		return 0;
	}
	for (ii=0;ii<12;ii++){
		if ((vdata[ii*50])!=(0xeeff)){
			if(verbose) fprintf(stderr,"malformed velodyne packet received, discarding\n");
			fprintf(stderr,"malformed velodyne packet received, discarding\n");
			return 0;
		}
	}
	return 1;
}


#define LIDAR_DP (16*2*12)
uint32_t velodyne_georef_data( uint16_t* data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, offset_t* sensor_offset,/*OUTPUT*/  output_data_t* outbuf){
     double* x = &(outbuf->x[0]);
     double* y = &(outbuf->y[0]);
     double* z = &(outbuf->z[0]);
     float* intensity = &(outbuf->i[0]);
     float* range = &(outbuf->range[0]);
     float* az_out = &(outbuf->teta[0]);
     float* el_out = &(outbuf->steer[0]);

	float sensor_r,sensor_az,sensor_el;
	float xs[LIDAR_DP];
	float ys[LIDAR_DP];
	float zs[LIDAR_DP];
    navdata_t *pos; 
    pos = &(posdata[pos_ix]); //TODO we need to find correct pos_ix here based on which is closest in posdata vector
	//This is how the Velodyne datasheet defines elevation angles by channel
	float eltable[16] = {-0.26179939f,0.01745329f,-0.2268928f,0.05235988f,-0.19198622f,0.08726646f,-0.15707963f,0.12217305f,-0.12217305f,0.15707963f,-0.08726646f,0.19198622f,-0.05235988f,0.2268928f,-0.01745329f,0.26179939f};
	//float eltable[16] = {0.26179939,-0.01745329,0.2268928,-0.05235988,0.19198622,-0.08726646,0.15707963,-0.12217305,0.12217305,-0.15707963,0.08726646,-0.19198622,0.05235988,-0.2268928,0.01745329,-0.26179939};

	uint8_t* dp;
	double nav_x, nav_y, nav_z; 			/*Position in global coordinates (east,north,up)*/
	float nav_yaw,  nav_pitch,  nav_roll; /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;

	uint16_t Nout;
	uint16_t ix_out;

	double lidar_ts = 1e-6*(((uint32_t*)data)[25*12]); /* Time in seconds since last hour mark */
	lidar_ts += (floor((pos->ts)/(3600)))*3600;			/* Adding number of hours since UNIX epoch */
	if((lidar_ts-(pos->ts))>(1800.)) lidar_ts -= 3600.;	/* If lidar_ts now is more than 30 min ahead of pos_ts, we probably had lidar right before hour mark when pos passed, subtract one hour*/
    lidar_ts += sensor_offset->time_offset;
    
    if (calc_interpolated_nav_data( posdata, pos_ix, lidar_ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt )) return 0;
	
	uint16_t block,ch,set;
	float azimuth;
	float d_azimuth;
	if (data[51] < data[1])
		d_azimuth = ((float)(36000+data[51]-data[1]))*(100*M_PI/(2*100*180)); // Convert to radians from centidegrees
	else
		d_azimuth = ((float)(data[51]-data[1]))*(M_PI/(2*100*180)); // Convert to radians from centidegrees

	ix_out = 0;
	// Populate r,az,el with data from lidar
	for (block=0;block<12;block++){
		azimuth = (float)data[block*50+1]*(M_PI/(100*180)); //0 to 2pi
		dp = (uint8_t*)(&(data[block*50+2]));
		//fprintf(stderr,"%6d ",data[block*50+1]);
		
		for (set = 0;set<2;set++){
			for (ch = 0; ch<16;ch++){
				sensor_r = (float)(((uint16_t)dp[0])+((uint16_t)dp[1])*256)*2e-3f;	//Velodyne measures distance in units of 2mm 
				sensor_az = azimuth;
				sensor_el = eltable[ch];
				//fprintf(stderr,"r=%6.2f az=%6.2f el=%6.2f\n",sensor_r, sensor_az*180/M_PI, sensor_el*180/M_PI);
				if (sensor_r>sensor_params->min_range && sensor_r<sensor_params->max_range){
					if ( (sensor_az>sensor_params->min_azimuth && sensor_az<sensor_params->max_azimuth )){
						/***** Converting sensor data from spherical to kartesian coordinates *********/
						// Standard lidar mounting, Lidar top pointing forward, Mounting bracket pointing downwards, (VLP-16 manual cable pointing up)
						//az=0 el=0 => Up(0,0,-1), az=90 el=0 => Port(0,-1,0), az=0 el=90 => Forward(1,0,0) 
						xs[ix_out] = sensor_r * sinf(sensor_el);
						ys[ix_out] = -sensor_r * sinf(sensor_az)*cosf(sensor_el); //Sign flipped compared to standard right hand system
						zs[ix_out] = -sensor_r * cosf(sensor_az)*cosf(sensor_el);
						intensity[ix_out] = dp[2];
                        range[ix_out] = sensor_r;
                        az_out[ix_out] = sensor_az;
                        el_out[ix_out] = sensor_el;
						ix_out++;
					}
				}
				dp+=3;
			}
			azimuth += d_azimuth;
		}
	}
	//fprintf(stderr,"\n");
	Nout = ix_out;

	georef_to_global_frame(sensor_offset, xs, ys, zs, Nout,0, nav_x, nav_y, nav_z, nav_yaw, nav_pitch,  nav_roll, ray_trace_none,  sensor_params->mounting_depth,/*OUTPUT*/ x,y,z);
	//fprintf(stderr,"Georef lidar data\n");	
	return Nout;
}
