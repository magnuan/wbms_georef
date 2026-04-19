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
#include "csv_nav.h"
#if defined(_MSC_VER)
#include <io.h>
#endif

static uint8_t verbose = 0;
static PJ *proj_latlon_to_input_utm;
static float input_timezone = 0.0;

uint8_t csv_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_CSV_NAV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = csv_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            if (csv_nav_identify_packet(data, len, &ts)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

//CSV is text based. Just look for newline to find sync.
int csv_nav_seek_next_header(int fd){
	uint8_t v;
	int n;
    while (1){
        n = read(fd,&v,1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ /*fprintf(stderr,"End of CSV NAV stream\n");*/return -1;}
        if(n>0){
            if (v=='\n'){
                return 0;	
            }
        }
    }
}

int csv_nav_fetch_next_packet(char * data, int fd){
	int rem,n;
    //Read one line of data from fd into data buffer
    for (rem=0;rem<MAX_CSV_NAV_PACKET_SIZE;rem++){
        n = read(fd,&(data[rem]),1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ /*fprintf(stderr,"End of CSV NAV stream\n");*/return 0;}
        if (data[rem]=='\n') break;
    }
    return rem;
}

int csv_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
	double ts, lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, yaw_deg;
    int ii = sscanf(databuffer,"%f,%f,%f,%f,%f,%f,%f",
                &ts, &lat_deg, &lon_deg, &alt_m, &roll_deg, &pitch_deg, &yaw_deg);
    //Sanity tests
    if (ii<7) return 0; //Not enough fields in csv string
    if ((ts < 946681229.0) || (ts > 4102441229)) return 0; // ts between Jan 1st 2000 and Jan 1st 2100
    if ((lat_deg<-90) || (lat_deg>90)) return 0;
    if ((lon_deg<-360) || (lon_deg>360)) return 0;
    if ((alt_m<-10000) || (alt_m>10000)) return 0;
    if ((roll_deg<-360) || (roll_deg>360)) return 0;
    if ((pitch_deg<-360) || (pitch_deg>360)) return 0;
    if ((yaw_deg<-360) || (yaw_deg>360)) return 0;
    return 1;
}

void csv_nav_set_params(float timezone, char* input_projection_string){
    input_timezone = timezone;
    
    if(latlon_to_proj_from_string(input_projection_string,&proj_latlon_to_input_utm)<0){
		fprintf(stderr, "Error configuring CSV nav georef parameters\n");
	}
}

int csv_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
	double ts, lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, yaw_deg;
    len = MIN(len,100); //CSV NAV line should be around 80 bytes, accept up to 100
    databuffer[len]=0;
    int ii = sscanf(databuffer,"%f,%f,%f,%f,%f,%f,%f",
                &ts, &lat_deg, &lon_deg, &alt_m, &roll_deg, &pitch_deg, &yaw_deg);

    if(ii>=7){
        navdata->ts = ts ;
        navdata->lon = lon_deg*M_PI/180;
        navdata->lat = lat_deg*M_PI/180;
        navdata->alt = alt_m;
        if (proj){
            latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
        }
        navdata->roll = roll_deg*M_PI/180;
        navdata->pitch = pitch_deg * M_PI/180;
        navdata->yaw = yaw_deg * M_PI/180;
        navdata->course = navdata->yaw;
        navdata->heave = 0.;
        navdata->z += z_offset;
        return 1;
    }
    else{
        fprintf(stderr, "Invalid line in CSV NAV (only %d of 7 read): %s\n",ii,databuffer);
    }
    return 0;
}
