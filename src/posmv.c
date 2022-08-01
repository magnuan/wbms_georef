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
#include "time_functions.h"
#include "posmv.h"

#include <math.h>
#include "cmath.h"
#include "wbms_georef.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#endif


#define POS_MODE_POSMV_GPS_EPOC  (315964800)

posmv3_t posmv3;
static uint8_t verbose = 0;

uint8_t posmv_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_POSMV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = posmv_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            int type = posmv_identify_packet(data, len, &ts);
            int req_types[] = {102};    //So far only support posmv type 102 
            size_t n_req_types = 1;
            for (size_t req_type_ix = 0; req_type_ix<n_req_types;req_type_ix++){
                if (type==req_types[req_type_ix]){
                    pass=1;
                    break;
                }
            }
            if (pass) break;
        }
        else{
            break;
        }
    }
    free(data);
    return pass;
}

double posmv_time_to_unix_time(double time1,double time2,uint8_t timetype){
	double gps;
	switch(timetype){
		case 0: return -1.0;		//1=POS;2=POS
		case 1: gps = time1;break;	//1=GPS;2=POS
		case 2: gps = time1+posmv3.gps_utc_diff;break;	//1=UTC;2=POS
		case 16: gps = time2;break;	//1=POS;2=GPS
		case 17: gps = time1;break;	//1=GPS;2=GPS
		case 18: gps = time2;break;	//1=UTC;2=GPS
		case 32: gps = time2+posmv3.gps_utc_diff;break;	//1=POS;2=UTC
		case 33: gps = time1;break;	//1=GPS;2=UTC
		case 34: gps = time1+posmv3.gps_utc_diff;break;	//1=UTC;2=UTC
		case 48: return -1.0;;	//1=POS;2=USR
		case 49: gps = time1;break;	//1=GPS;2=USR
		case 50: gps = time1+posmv3.gps_utc_diff;break;	//1=UTC;2=USR
		default: return -1.0;
	};
	return POS_MODE_POSMV_GPS_EPOC+(60*60*24*7*posmv3.gps_week) + gps - posmv3.gps_utc_diff;
}


#define POSMV_BUFFER_SIZE (MAX_POSMV_PACKET_SIZE+4)
int buffered_read(int fd, void* data, int len){
    static uint8_t buffer[POSMV_BUFFER_SIZE];
    static uint8_t* buf_ptr;
    static int available = 0;
    while (1){
        if (len>0){
            if (len<=available){
                memcpy(data,buf_ptr,len);
                available -= len;
                buf_ptr += len;
                return len;
            }
            else if(available>0){
                len = available;
                memcpy(data,buf_ptr,len);
                available = 0;
                return len;
            }
            else{
                buf_ptr = buffer;
                available = read(fd,buffer,POSMV_BUFFER_SIZE);
                if (available==0){
                    return 0;
                }
            }
        }
    }
}


int posmv_seek_next_header(int fd){
	char state = 0;
	uint8_t v;
	int n;
	int dump= 0;
    int read_bytes = 0;
	while (read_bytes<(MAX_POSMV_PACKET_SIZE+4)){
        n = buffered_read(fd,&v,1);
        read_bytes++;
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ fprintf(stderr,"End of POS_MODE_POSMV stream\n");return -1;}
        if(n>0){
            dump += 1;
            switch (state){
                case 0: state = (v=='$')?1:0;break;
                case 1: state = (v=='G')?2:0;break;
                case 2: state = (v=='R')?3:0;break;
                case 3: state = (v=='P')?4:0;break;
            }
            if (state==4){
                dump-=4;
                //if(dump>2) fprintf(stderr,"PosMV seek dump %d bytes\n",dump);
                return 0;	
            }
        }
    }
	return -1;
}

int posmv_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;
	uint16_t gid;
	uint16_t count;
    if(posmv_seek_next_header(fd)) return 0;
    data[0] = '$';data[1] = 'G';data[2] = 'R';data[3] = 'P';
    n = buffered_read(fd,&gid,2);
    n = buffered_read(fd,&count,2);
    *((uint16_t*)(&(data[4]))) = gid;
    *((uint16_t*)(&(data[6]))) = count;
    rem = count; //Fetch packet header (minus the preamble we allready have)
    dp = &(data[8]);
    while (rem>0){ 
        n= buffered_read(fd,dp,rem);rem -= n; dp+=n;
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ fprintf(stderr,"End of POS_MODE_POSMV stream\n");return 0;}
    }
    return count+8;
}

int posmv_identify_packet(char* databuffer, uint32_t len, double* ts_out){
    if (strncmp(databuffer,"$GRP",4) != 0 ){
        if(verbose) fprintf(stderr,"Malformed pos packet received, discarding\n");
        return 0;
    }
    char * dp = databuffer + 4;
    uint16_t gid = *(((uint16_t*)(dp)));dp+=2;
    uint16_t count = *(((uint16_t*)(dp)));dp+=2;
    double time1 = *(((double*)(dp)));dp+=8;
    double time2 = *(((double*)(dp)));dp+=8;
    /*double dist = *(((double*)(dp)));*/ dp+=8;
    uint8_t timetype = *(((uint8_t*)(dp)));dp+=1;
    /*uint8_t disttype = *(((uint8_t*)(dp)));*/ dp+=1;

	uint16_t cs=0;
    for (int ii = 0; ii<((8+count)/2);ii++) cs += ((uint16_t*)databuffer)[ii]; 
    if (cs){
        if(verbose) fprintf(stderr,"POS_MODE_POSMV packet CS error, discarding\n");
        return 0;
    }

    double ts = posmv_time_to_unix_time(time1,time2,timetype);
    *ts_out = ts;
    return gid; 
}


int posmv_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata){
    static uint32_t group3_cnt = 0;
    static uint32_t group102_cnt = 0;
    if (strncmp(databuffer,"$GRP",4) != 0 ){
        if(verbose) fprintf(stderr,"Malformed pos packet received, discarding\n");
        return NO_NAV_DATA;
    }

    char * dp = databuffer + 4;
    uint16_t gid = *(((uint16_t*)(dp)));dp+=2;
    uint16_t count = *(((uint16_t*)(dp)));dp+=2;
    double time1 = *(((double*)(dp)));dp+=8;
    double time2 = *(((double*)(dp)));dp+=8;
    double dist = *(((double*)(dp)));dp+=8;
    uint8_t timetype = *(((uint8_t*)(dp)));dp+=1;
    uint8_t disttype = *(((uint8_t*)(dp)));dp+=1;

	uint16_t cs=0;
    for (int ii = 0; ii<((8+count)/2);ii++) cs += ((uint16_t*)databuffer)[ii]; 
    if (cs){
        if(verbose) fprintf(stderr,"POS_MODE_POSMV packet CS error, discarding\n");
        return NO_NAV_DATA;
    }

    double ts = posmv_time_to_unix_time(time1,time2,timetype);
    *ts_out = ts;
	
    char str_buf[256];
    sprintf_unix_time(str_buf, ts);
    if(verbose) fprintf(stderr,"POS_MODE_POSMVdata gid=%d %s count=%d time1=%f time2=%f dist=%f timetype=%d disttype=%d cs=%d\n",gid,str_buf,count,time1,time2,dist,timetype,disttype,cs);

    switch (gid){
        default:
            return NO_NAV_DATA;
        case 3: //#Primary GPS, Info about primary GPS receiver status (No positioning data)
            group3_cnt++;
            posmv3.mode = *(((uint8_t*)(dp)));dp+=1;
            posmv3.sv_n = *(((uint8_t*)(dp)));dp+=1;
            posmv3.count2 = *(((uint16_t*)(dp)));dp+=2;
            //Next count2 bytes is SV data
            dp += posmv3.count2;
            posmv3.hdop = *(((float*)(dp)));dp+=4;
            posmv3.vdop = *(((float*)(dp)));dp+=4;
            posmv3.dgps_latency = *(((float*)(dp)));dp+=4;
            posmv3.dgps_statid = *(((uint16_t*)(dp)));dp+=2;
            posmv3.gps_week = *(((uint32_t*)(dp)));dp+=4;
            posmv3.gps_utc_diff = *(((double*)(dp)));dp+=8;
            posmv3.gps_nav_latency = *(((float*)(dp)));dp+=4;
            posmv3.geoid_separation = *(((float*)(dp)));dp+=4;
            posmv3.gps_type = *(((uint8_t*)(dp)));dp+=1;
            posmv3.gps_status = *(((uint32_t*)(dp)));dp+=4;
            //Recalculate time as this depends on the new received gps_utc_diff
            ts = posmv_time_to_unix_time(time1,time2,timetype);
            *ts_out = ts;
            posmv3.ts = ts;
            if(group3_cnt<3){ 
                fprintf(stderr, "POS_MODE_POSMV GRP3 ts=%f,mode=%d,nSV=%d,count2=%d,hdop=%f,vdop=%f,dgps_latency=%f,dgps_statid=%d,WeekNr=%d,GPS_UTC_diff=%f,gps_nav_latency=%f,Geoid_sep=%f,gps_type=%d,gps_status=%d\n",ts,posmv3.mode,posmv3.sv_n,posmv3.count2,posmv3.hdop,posmv3.vdop,posmv3.dgps_latency,posmv3.dgps_statid,posmv3.gps_week,posmv3.gps_utc_diff,posmv3.gps_nav_latency,posmv3.geoid_separation,posmv3.gps_type,posmv3.gps_status);
            }
            return NO_NAV_DATA;
        case 102:
            group102_cnt++;
            //Fill out navdata from posmv 102 data
            navdata->ts = ts;
            navdata->lat = *(((double*)(dp)));dp+=8;
            navdata->lon = *(((double*)(dp)));dp+=8;
            navdata->alt = *(((double*)(dp)));dp+=8;
            dp+=12;  //Skip time derivates
            navdata->roll = *(((double*)(dp)));dp+=8;
            navdata->pitch = *(((double*)(dp)));dp+=8;
            navdata->yaw = *(((double*)(dp)));dp+=8;
            navdata->course = *(((double*)(dp)));dp+=8;
            navdata->heave = *(((float*)(dp)));dp+=4;
            dp+=24;  //Skip time derivates
            //pad = *(((uint16_t*)(dp)));dp+=2;
            cs = *(((uint16_t*)(dp)));dp+=2;
            //end = *(((uint16_t*)(dp)));dp+=2;

            if(verbose) fprintf(stderr, "POS_MODE_POSMV GRP102 lat=%5.2f,lon=%5.2f,alt=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,course=%5.2f,heave=%5.2f\n",navdata->lat,navdata->lon,navdata->alt,navdata->roll,navdata->pitch,navdata->yaw,navdata->course,navdata->heave);
            navdata->lat *=(M_PI/180);
            navdata->lon *=(M_PI/180);
            navdata->roll *=(M_PI/180);
            navdata->pitch *=(M_PI/180);
            navdata->yaw *=(M_PI/180);
            navdata->alt -= posmv3.geoid_separation;
            navdata->hor_accuracy = posmv3.hdop;
            navdata->vert_accuracy = posmv3.vdop;

            if (proj){
                navdata->yaw += north_to_northing_offset(navdata->lon, navdata->lat, proj);
                //fprintf(stderr, "NORTH TO NORTHING OFFSET = %6.3f deg\n",north_to_northing_offset(navdata->lon, navdata->lat, proj)*180/M_PI);
                double alt;
                switch (alt_mode){ 
                    case 1:  alt = navdata->alt; break; //GPS altitude from posmv is positive up (When having RTK, we typically want to use GPS altitude instread of heave+tide)
                    case 2:  alt  = -navdata->heave; break; //heave from posmv is positive down ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
                    default: alt = 0;
                }

                latlon_to_kart(navdata->lon, navdata->lat , alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &(navdata->z));
            }

            navdata->z += z_offset;
            if (group3_cnt==0){
                fprintf(stderr, "No Posmv group 3 data for group102 %d\n",group102_cnt);
                return NO_NAV_DATA;
            }
            if ((posmv3.ts-navdata->ts)>3600. || (navdata->ts-posmv3.ts)>3600.) {
                fprintf(stderr, "Posmv group 3 data out of sync for group102_cnt=%d group3_cnt=%d (ts3=%f ts102=%f)\n",group102_cnt,group3_cnt,posmv3.ts,navdata->ts);
                return NO_NAV_DATA;
            }

            return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    return NO_NAV_DATA;
}
