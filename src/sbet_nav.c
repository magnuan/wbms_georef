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
#include "time_functions.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif


/* Parser for Applanix, Smoothened best estimate trajectory  SBET file */

static double sbet_epoch = 0;

// Set epoch to UTC midnight last sunday SBET gives time in GPS time (seconds of week) 
double calc_sbet_epoch(double ts){
    time_t t;
    t = ts; //Ignore fract seconds
	struct tm* buf = gmtime(&t);
    time_t seconds_since_sunday = (buf->tm_sec + 60*(buf->tm_min + 60*(buf->tm_hour + 24*buf->tm_wday)));
    t -= seconds_since_sunday;
    //TODO should GPS UTC leap seconds differnce be added here?
    // Tested, but it seems very wrong when I do. Perhaps this is added somewhere else
    return (double) t;
}

void set_sbet_epoch(double ts){
    if (sbet_epoch==0){
        sbet_epoch = calc_sbet_epoch(ts);
        time_t raw_time = (time_t) sbet_epoch;
        fprintf(stderr,"Setting SBET epoch to: ts=%0.3f %s  ",sbet_epoch,ctime(&raw_time));
    }
}


/***************** SECTION FOR BINARY FORMATED SBET DATA ***************************************/
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
        if(n==0){ /*fprintf(stderr,"End of SBET stream\n");*/return 0;}
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
    if ((ts<0.001) || (ts>(60.*60.*24.*7.))) return 0; //Time stamp is relative to beginning of week, so if it is more than 7 days, its bad, also do not allow 0, to prevent all 0 data from passing as good
    if ((roll<-M_PI) ||(roll>M_PI)) return 0;
    if ((pitch<-M_PI) ||(pitch>M_PI)) return 0;
    if ((yaw<-(2*M_PI)) ||(yaw>(2*M_PI))) return 0;
    if ((course<-(2*M_PI)) ||(course>(2*M_PI))) return 0;
    if ((lat<-(M_PI/2)) ||(lat>(M_PI/2))) return 0;
    if ((lon<-(2*M_PI)) ||(lon>(2*M_PI))) return 0;
    if ((alt<-(10000.f)) ||(alt>(10000.f))) return 0;
    if ((ABS(lat)<0.001f) || (ABS(lon)<0.001f)) return 0;   //Another rule to prevent false detection on all 0. 
    fprintf(stderr,"Good SBET\n");
    return 1;
}

int sbet_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
    static int rcnt=0;
    if (sbet_epoch == 0)    //Cant decode this data unless we know SBET epoch
        return NO_NAV_DATA;
    char* dp = databuffer;
    double ts = *(((double*)(dp)));dp+=8;
    ts += sbet_epoch;
    *ts_out = ts;
	
    if(0){//rcnt%10000 ==0){
        char str_buf[256];
        sprintf_unix_time(str_buf, ts);
        fprintf(stderr,"SBET data %d %s\n",rcnt, str_buf);
    }
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
    
    #if 0
    if (rcnt <100){
        fprintf(stderr, "SBET wtime=%13.5f lat=%10.4f,lon=%10.4f,alt=%6.3f,roll=%7.4f,pitch=%7.4f,yaw=%7.4f,course=%5.2f\n",
                                    navdata->ts-sbet_epoch, navdata->lat*180/M_PI,navdata->lon*180/M_PI,navdata->alt,
                                    navdata->roll*180/M_PI,navdata->pitch*180/M_PI,navdata->yaw*180/M_PI,navdata->course*180/M_PI);
    }
    #endif
    if (proj){
        navdata->yaw += north_to_northing_offset(navdata->lon, navdata->lat, proj);
        latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
    }
    navdata->z += z_offset;
    rcnt++;
    return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
}


/***************** SECTION FOR CSV FORMATED SBET DATA ***************************************/

uint8_t sbet_csv_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_SBET_CSV_NAV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<250;test++){     //Test the first 50 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = sbet_csv_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            if (sbet_csv_nav_identify_packet(data, len, &ts)){
                pass=1;
                break;
            }
        }
    }
    free(data);
    return pass;
}

//SBET_CSV is text based. Just look for newline to find sync.
int sbet_csv_nav_seek_next_header(int fd){
	uint8_t v;
	int n;
    while (1){
        n = read(fd,&v,1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ /*fprintf(stderr,"End of SBET_CSV NAV stream\n");*/return -1;}
        if(n>0){
            if (v=='\n'){
                return 0;	
            }
        }
    }
}

int sbet_csv_nav_fetch_next_packet(char * data, int fd){
    //TODO This is insanely much faster than reading byte by byte with read
    //But in probably only works with file input.
    //Ideally this should be chosen run-time based on wether input is file or stream
    #if 1
    ssize_t read;
    static FILE *file;
    if (file==NULL){
        file = fdopen(fd, "r");
    }
    if (file ==NULL) return -1;
    //size_t len;
    //read = getline(&data, &len,file);
    char* ret = fgets(data, MAX_SBET_CSV_NAV_PACKET_SIZE,file);
    if (ret==NULL) return 0;
    read = strlen(data);
    if (read>=0){
        return read;
    }
    else return 0;
    #else
	int rem,n;
    //Fetch POS_MODE_SBET_CSV_NAV data here
    //Read one line of data from fd into data buffer
    for (rem=0;rem<MAX_SBET_CSV_NAV_PACKET_SIZE;rem++){
        n = read(fd,&(data[rem]),1);
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ fprintf(stderr,"End of SBET_CSV NAV stream\n");return 0;}
        if (data[rem]=='\n') break;
    }
    return rem;
    #endif
}

int sbet_csv_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
	double ts,lat,lon,east,north;
    float dist,alt1,alt2,roll,pitch,yaw;
    float d_east,d_north,d_alt;
    float sd_east,sd_north,sd_alt,sd_roll,sd_pitch,sd_yaw;
    //TIME, DISTANCE, EASTING, NORTHING, ELLIPSOID HEIGHT, 
    //LATITUDE, LONGITUDE, ELLIPSOID HEIGHT, ROLL, PITCH, HEADING, 
    //EAST VELOCITY, NORTH VELOCITY, UP VELOCITY, 
    //EAST SD, NORTH SD, HEIGHT SD, ROLL SD, PITCH SD, HEADING SD
    int ii = sscanf(databuffer, 
        "%lf %f %lf %lf %f"  
        "%lf %lf %f %f %f %f" 
        "%f %f %f" 
        "%f %f %f %f %f %f", 
        &ts, &dist, &east, &north, &alt1,
        &lat, &lon,&alt2, &roll, &pitch, &yaw,
        &d_east, &d_north, &d_alt,
        &sd_east, &sd_north, &sd_alt, &sd_roll, &sd_pitch, &sd_yaw);
    

    //Sanity tests
    if (ii<11) return 0; //Not enough fields in csv string
    #if 0
    fprintf(stderr, "SBET CSV indentify ts=%f,lat=%f,lon=%f,alt=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,north=%7.3f,east=%7.3f\n",
            ts, lat,lon,alt2,roll,pitch,yaw,north,east);
    #endif
    if ((ts<0) ||(ts>(60*60*24*7))) return 0;
    if ((alt2<-11034.) ||(alt2>8848.)) return 0;
    if ((lat<-(90.f)) ||(lat>(90.f))) return 0;
    if ((lon<-(360.f)) ||(lon>(360.f))) return 0;
    if ((roll<-(360.f)) ||(roll>(360.f))) return 0;
    if ((pitch<-(360.f)) ||(pitch>(360.f))) return 0;
    if ((yaw<-(360.f)) ||(yaw>(360.f))) return 0;
    if ((north<-(10e6f)) ||(north>(10e6f))) return 0;
    if ((east<-(1e6f)) ||(east>(1e6f))) return 0;
    return 1;
}


int sbet_csv_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
    static int rcnt=0;
	double ts,lat,lon,east,north;
    float dist,alt1,alt2,roll,pitch,yaw;
    //float d_east,d_north,d_alt;
    //float sd_east,sd_north,sd_alt,sd_roll,sd_pitch,sd_yaw;

    len = MIN(len,500); //SBET_CSV NAV line should be around 203 bytes, accept up to 500
    databuffer[len]=0;
    int ii = sscanf(databuffer, 
        "%lf %f %lf %lf %f"  
        "%lf %lf %f %f %f %f", 
        &ts, &dist, &east, &north, &alt1,
        &lat, &lon,&alt2, &roll, &pitch, &yaw);
    /*int ii = sscanf(databuffer, 
        "%lf %f %lf %lf %f"  
        "%lf %lf %f %f %f %f" 
        "%f %f %f" 
        "%f %f %f %f %f %f", 
        &ts, &dist, &east, &north, &alt1,
        &lat, &lon,&alt2, &roll, &pitch, &yaw,
        &d_east, &d_north, &d_alt,
        &sd_east, &sd_north, &sd_alt, &sd_roll, &sd_pitch, &sd_yaw);*/
    /*fprintf(stderr, "SBET CSV process %10d : ts=%f,lat=%f,lon=%f,alt=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,north=%7.3f,east=%7.3f\n",
            rcnt, ts, lat,lon,alt2,roll,pitch,yaw,north,east);*/
    
    if(ii>=11){
        rcnt++;
        ts += sbet_epoch;
        *ts_out = ts;

        //Fill out navdata from SBET_CSV-NAV data
        navdata->ts = ts ;
        navdata->lat = lat*M_PI/180;
        navdata->lon = lon*M_PI/180;
        navdata->alt = alt2;
        navdata->roll = roll*M_PI/180;
        navdata->pitch = pitch * M_PI/180;
        navdata->yaw = yaw * M_PI/180;
        navdata->course = navdata->yaw;
        navdata->heave = 0.;
        #if 0
        if (rcnt <100){
            fprintf(stderr, "SBET wtime=%13.5f lat=%10.4f,lon=%10.4f,alt=%6.3f,roll=%7.4f,pitch=%7.4f,yaw=%7.4f,course=%5.2f\n",
                                        navdata->ts-sbet_epoch, navdata->lat*180/M_PI,navdata->lon*180/M_PI,navdata->alt,
                                        navdata->roll*180/M_PI,navdata->pitch*180/M_PI,navdata->yaw*180/M_PI,navdata->course*180/M_PI);
        }
        #endif
        if (proj){
            navdata->yaw += north_to_northing_offset(navdata->lon, navdata->lat, proj);
            latlon_to_kart(navdata->lon, navdata->lat , navdata->alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
        }
        navdata->z += z_offset;

        return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    else{
        fprintf(stderr, "Invalid line in SBET_CSV NAV (only %d of minimum 11 read): %s",ii,databuffer);
    }
    return 0;
}
