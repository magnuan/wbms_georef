#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "cmath.h"
#include <math.h>
#include "wbms_georef.h"
#include "sbet_nav.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#endif


/* Parser for Applanix, Smoothened best estimate trajectory  SBET file */

static double sbet_epoch = 0;

uint8_t sbet_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(SBET_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = sbet_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            if (sbet_nav_identify_packet(data, len, &ts)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

//Sbet files has no synchronizaton tag, so we just need to assume we are in sync 
int sbet_nav_seek_next_header(int fd){
    return 0;
}

int sbet_nav_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;
	uint16_t count;
    count = SBET_PACKET_SIZE;       //SBET has a fixed set of 17 double values
    rem = count; 
    dp = &(data[0]);
    while (rem>0){ 
        n= read(fd,dp,rem);rem -= n; dp+=n;
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ fprintf(stderr,"End of SBET stream\n");return 0;}
    }
    return count;
}

//Chack that there is sort of sane values in data
int sbet_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
    if (len<(14*8)) return 0;
    char* dp = databuffer;
    double ts = *(((double*)(dp)));dp+=8;
    double lat = *(((double*)(dp)));dp+=8;
    double lon = *(((double*)(dp)));dp+=8;
    double alt = *(((double*)(dp)));dp+=8;
    dp+=24;  //Skip time derivates
    double roll = *(((double*)(dp)));dp+=8;
    double pitch = *(((double*)(dp)));dp+=8;
    double yaw = *(((double*)(dp)));dp+=8;
    double course = *(((double*)(dp)));dp+=8;
    #if 0
    fprintf(stderr, "SBET indentify ts=%f,lat=%f,lon=%f,alt=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,course=%5.2f\n",
            ts, lat*180/M_PI,lon*180/M_PI,alt,roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI,course*180/M_PI);
    #endif
    if ((ts<1.) || (ts>(60.*60.*24.*32.))) return 0; //Time stamp seems to be relative to meginning of month, so if it is more than 32 days, its bad, also do not allow 0, to prevent all 0 data from passing as good
    if ((roll<-M_PI) ||(roll>M_PI)) return 0;
    if ((pitch<-M_PI) ||(pitch>M_PI)) return 0;
    if ((yaw<-(2*M_PI)) ||(yaw>(2*M_PI))) return 0;
    if ((course<-(2*M_PI)) ||(course>(2*M_PI))) return 0;
    if ((lat<-(M_PI/2)) ||(lat>(M_PI/2))) return 0;
    if ((lon<-(2*M_PI)) ||(lon>(2*M_PI))) return 0;
    if ((alt<-(10000.f)) ||(alt>(10000.f))) return 0;
    if ((ABS(lat)<1.f) || (ABS(lon)<1.f)) return 0;   //Another rule to prevent false detection on all 0. 
    //fprintf(stderr,"Good SBET\n");
    return 1;
}

double calc_sbet_epoch(double ts){
    //TODO I have no idea if this is right in general, it worked for one dataset I got.
    // It sets epoch ot UTC midnight last day in previous month
    time_t t;
    t = ts; //Ignore fract seconds
	struct tm* buf = gmtime(&t);
    //fprintf(stderr,"ts = %d tm = [%d-%d-%d %02d:%02d:%02d  %d]\n",t,1900+buf.tm_year,buf.tm_mon,buf.tm_mday,buf.tm_hour,buf.tm_min,buf.tm_sec,buf.tm_isdst);
    buf->tm_sec = 0;
    buf->tm_min = 0;
    buf->tm_hour = 0;
    buf->tm_mday = 0;
    buf->tm_isdst = 0;
    t = mktime(buf);
    if (buf->tm_isdst) t+=3600;
    //fprintf(stderr,"ts = %d tm = [%d-%d-%d %02d:%02d:%02d  %d]\n",t,1900+buf.tm_year,buf.tm_mon,buf.tm_mday,buf.tm_hour,buf.tm_min,buf.tm_sec,buf.tm_isdst);
    return (double) t;
}

void set_sbet_epoch(double ts){
    if (sbet_epoch==0){
        sbet_epoch = calc_sbet_epoch(ts);
        time_t raw_time = (time_t) sbet_epoch;
        fprintf(stderr,"Setting SBET epoch to: ts=%0.3f %s  ",sbet_epoch,ctime(&raw_time));
    }
}

int sbet_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata){
    if (sbet_epoch == 0)    //Cant decode this data unless we know SBET epoch
        return NO_NAV_DATA;
    char* dp = databuffer;
    double ts = *(((double*)(dp)));dp+=8;
    ts += sbet_epoch;
    *ts_out = ts;
	//char str_buf[256];
    //sprintf_unix_time(str_buf, ts);
    //if(verbose) fprintf(stderr,"SBET data %s\n",str_buf);
    //Put new data in next block of circular buffer
    navdata->ts = ts;
    navdata->lat = *(((double*)(dp)));dp+=8;
    navdata->lon = *(((double*)(dp)));dp+=8;
    navdata->alt = *(((double*)(dp)));dp+=8;
    dp+=24;  //Skip time derivates
    navdata->roll = *(((double*)(dp)));dp+=8;
    navdata->pitch = *(((double*)(dp)));dp+=8;
    navdata->yaw = *(((double*)(dp)));dp+=8;
    navdata->course = *(((double*)(dp)));dp+=8;
    navdata->heave = 0;
    dp+=24;  //Skip time derivates
    //if(verbose) fprintf(stderr, "SBET lat=%5.2f,lon=%5.2f,alt=%5.2f,dx=%5.2f,dy=%5.2f,dz=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,course=%5.2f,heave=%5.2f,droll=%5.2f,dpitch=%5.2f,dyaw=%5.2f,ddx=%5.2f,ddy=%5.2f,ddz=%5.2f\n",navdata->lat*180/M_PI,navdata->lon*180/M_PI,navdata->alt,navdata->dx,navdata->dy,navdata->dz,navdata->roll*180/M_PI,navdata->pitch*180/M_PI,navdata->yaw*180/M_PI,navdata->course*180/M_PI,navdata->heave,navdata->droll*180/M_PI,navdata->dpitch*180/M_PI,navdata->dyaw*180/M_PI,navdata->ddx,navdata->ddy,navdata->ddz);
    if (proj){
        navdata->yaw += north_to_northing_offset(navdata->lon, navdata->lat, proj);
        latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
    }
    navdata->z += z_offset;
    return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
}
