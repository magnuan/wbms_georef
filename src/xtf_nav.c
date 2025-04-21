#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "cmath.h"
#include "time_functions.h"
#include <math.h>
#include "wbms_georef.h"
#include "xtf_nav.h"
#if defined(_MSC_VER)
#include <io.h>
#endif

//I dont think this is XTF. It was just some csv based navigation file I got for a dataset, that was named XTF
//It reads in position in projected coordinates, so wn need to transform the coordinate system to be able to output to other projections

static uint8_t verbose = 0;
static PJ *proj_latlon_to_input_utm;
static float input_timezone = 0.0;

uint8_t xtf_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_XTF_NAV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = xtf_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            if (xtf_nav_identify_packet(data, len, &ts)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

//XTF is text based. Just look for newline to find sync.
int xtf_nav_seek_next_header(int fd){
	uint8_t v;
	int n;
    while (1){
        n = read(fd,&v,1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ /*fprintf(stderr,"End of XTF NAV stream\n");*/return -1;}
        if(n>0){
            if (v=='\n'){
                return 0;	
            }
        }
    }
}

int xtf_nav_fetch_next_packet(char * data, int fd){
	int rem,n;
    //Fetch POS_MODE_XTF_NAV data here
    //Read one line of data from fd into data buffer
    for (rem=0;rem<MAX_XTF_NAV_PACKET_SIZE;rem++){
        n = read(fd,&(data[rem]),1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ /*fprintf(stderr,"End of XTF NAV stream\n");*/return 0;}
        if (data[rem]=='\n') break;
    }
    return rem;
}

int xtf_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
	int year,month,date,hour,min;
	float sec,kp,depth,altitude,heading;
	double northing, easting;
    int ii = sscanf(databuffer,	"%4d%2d%d,%d,%d,%f,%f,%lf,%lf,%f,%f,%f", 
                &year,&month,&date,&hour,&min,&sec,&kp,&easting,&northing,&depth,&altitude,&heading);
    //Sanity tests
    if (ii<12) return 0; //Not enough fields in csv string
    if ((year<2000) ||(year>2100)) return 0;
    if ((month<1) ||(month>12)) return 0;
    if ((date<1) ||(date>31)) return 0;
    if ((hour<0) ||(hour>24)) return 0;
    if ((min<0) ||(min>60)) return 0;
    if ((sec<0) ||(sec>61)) return 0;
    if ((heading<-(360.f)) ||(heading>(360.f))) return 0;
    if ((northing<-(1e6f)) ||(northing>(1e6f))) return 0;
    if ((easting<-(1e6f)) ||(easting>(1e6f))) return 0;
    if ((altitude<-(10000.f)) ||(altitude>(10000.f))) return 0;
    return 1;
}

void xtf_nav_set_params(float timezone, char* input_projection_string){
    input_timezone = timezone;
    
    if(latlon_to_proj_from_string(input_projection_string,&proj_latlon_to_input_utm)<0){
		fprintf(stderr, "Error configuring XTF nav georef parameters\n");
	}
}

int xtf_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
	double ts;
	int year,month,date,doy,hour,min;
	float sec,kp,depth,altitude,heading,dol,fix,roll,pitch;
	double northing, easting;
	#define XTF_NAV_VERT_ADJ (-7)

    len = MIN(len,100); //XTF NAV line should be around 80 bytes, accept up to 100
    databuffer[len]=0;
    int ii = sscanf(databuffer,	"%4d%2d%d,%d,%d,%f,%f,%lf,%lf,%f,%f,%f,%f,%f,%f,%f", &year,&month,&date,&hour,&min,&sec,&kp,&easting,&northing,&depth,&altitude,&heading,&dol,&fix,&roll,&pitch);
    if (ii!=16) {ii = sscanf(databuffer,	"%4d%2d%d,%d,%d,%f,%f,%lf,%lf,%f,%f,%f,%f,,%f,%f", &year,&month,&date,&hour,&min,&sec,&kp,&easting,&northing,&depth,&altitude,&heading,&dol,&roll,&pitch);}
    if(ii>=15){
        doy = date_to_doy( (uint16_t)year, (uint16_t)month, (uint16_t) date);
        ts = irigb_to_gm( year, doy, hour, min, sec)- input_timezone*3600.;
        *ts_out = ts;

        if (proj){
            latlon_from_kart(northing, easting , -altitude, proj_latlon_to_input_utm, /*output*/ &(navdata->lon), &(navdata->lat) , &(navdata->alt));
            latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
        }
        //Fill out navdata from XTF-NAV data
        navdata->ts = ts ;
        navdata->roll = roll*M_PI/180;
        navdata->pitch = pitch * M_PI/180;
        navdata->yaw = heading * M_PI/180;
        navdata->course = navdata->yaw;
        navdata->heave = 0.;
        navdata->z += z_offset;

        if(verbose) fprintf(stderr, "%d read  doy=%d ts = %f, lat=%f lon=%f, %4d/%2d/%d %d:%d:%f KP=%f %fE %fN depth=%f alt=%f heading=%f dol=%f fix=%f roll=%f pitch=%f\n", 
                ii,doy,ts,navdata->lat*180/M_PI,navdata->lon*180/M_PI,year,month,date,hour,min,sec,kp,easting,northing,depth,altitude,heading,dol,fix,roll,pitch);
        return 1;
    }
    else{
        fprintf(stderr, "Invalid line in XTF NAV (only %d of 16 read): %s\n",ii,databuffer);
    }
    return 0;
}
