#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "cmath.h"
#include <math.h>
#include "wbms_georef.h"
#include "wbm_tool_nav.h"


uint8_t wbm_tool_nav_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_WBM_TOOL_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = wbm_tool_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            if (wbm_tool_nav_identify_packet(data, len, &ts)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}
//WBM tool output is text based. Just look for newline to find sync.
int wbm_tool_nav_seek_next_header(int fd){
	uint8_t v=0;
	int n;
    while (1){
        n = read(fd,&v,1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ fprintf(stderr,"End of wbm_tool NAV stream\n");return -1;}
        if(n>0){
            if (v=='\n'){
                return 0;	
            }
        }
    }
}

//Read one line of data from fd into data buffer (limited to 1kB)
int wbm_tool_nav_fetch_next_packet(char * data, int fd){
	int rem,n;
    for (rem=0;rem<MAX_WBM_TOOL_PACKET_SIZE;rem++){
        n = read(fd,&(data[rem]),1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ fprintf(stderr,"End of wbm_tool NAV stream\n");return 0;}
        if (data[rem]=='\n') break;
    }
    return rem;
}

int wbm_tool_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
	double ts,ts_sys;
	float altitude,heading,roll,pitch;
    double latitude,longitude;
    len = MIN(len,200); //WBM_TOOL line should be around 160 bytes, accept up to 200
    databuffer[len]=0;
    int ii = sscanf(databuffer,"%lf %lf %f %f %f %lf %lf %f"
            ,&ts
            ,&ts_sys
            ,&roll
            ,&pitch
            ,&heading
            ,&latitude
            ,&longitude
            ,&altitude
            );
    //Sanity tests
    if (ii<8) return 0; //Not enough fields in csv string
    if ((ts<(60*60*24*365.25*(2000-1970))) || (ts>(60*60*24*365.25*(2100-1970)))) return 0; //Only time stamps in 21st century are considered ok
    if ((ts_sys<(60*60*24*365.25*(2000-1970))) || (ts_sys>(60*60*24*365.25*(2100-1970)))) return 0; 
    if ((roll<-M_PI) ||(roll>M_PI)) return 0;
    if ((pitch<-M_PI) ||(pitch>M_PI)) return 0;
    if ((heading<-(2*M_PI)) ||(heading>(2*M_PI))) return 0;
    if ((latitude<-(M_PI/2)) ||(latitude>(M_PI/2))) return 0;
    if ((longitude<-(2*M_PI)) ||(longitude>(2*M_PI))) return 0;
    if ((altitude<-(10000.f)) ||(altitude>(10000.f))) return 0;
    return 1;
}


int wbm_tool_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata){
	double ts,ts_sys;
	float altitude,heading,roll,pitch;
    double latitude,longitude;
    len = MIN(len,200); //WBM_TOOL line should be around 160 bytes, accept up to 200
    databuffer[len]=0;
    int ii = sscanf(databuffer,"%lf %lf %f %f %f %lf %lf %f"
            ,&ts
            ,&ts_sys
            ,&roll
            ,&pitch
            ,&heading
            ,&latitude
            ,&longitude
            ,&altitude
            );

    if (alt_mode==0){ //Nav data from WBM tool does not output heave, but if alt mode is "none" set altitude to 0 
        altitude = 0;
    }
    if(ii>=8){
        *ts_out = ts;
        navdata->ts = ts ;
        navdata->lon = longitude;
        navdata->lat = latitude;
        navdata->roll = roll;
        navdata->pitch = pitch;
        navdata->yaw = heading;
        navdata->alt = altitude; //WBMtool has positive up altitude

        if (proj){
            navdata->yaw += north_to_northing_offset(navdata->lon, navdata->lat, proj);
            latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
        }

        navdata->course = navdata->yaw;
        //fprintf(stderr,"lat=%12.7f\n",navdata->lat*180/M_PI);
        navdata->heave = 0.;
        navdata->z += z_offset;

        return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    else{
        fprintf(stderr, "Invalid line in WBM_TOOL file (only %d of 8 read): %s\n",ii,databuffer);
    }
    return NO_NAV_DATA;
}
