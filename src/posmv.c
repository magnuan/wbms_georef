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

static uint32_t group3_cnt = 0;
static uint32_t group2_cnt = 0;
static uint32_t group1_cnt = 0;
static uint32_t group102_cnt = 0;
posmv2_t posmv2;
posmv3_t posmv3;
static uint8_t verbose = 0;

#ifdef COLLECT_POSMV_STATS
static uint32_t posmv_stat_packet_count[POSMV_ID_MAX+1];
#endif


/* Alternative GPS epoch to use if we dont have GPS Week and Leap second info from group 3*/
static double alt_gps_epoch = 0;

static double calc_epoch(double ts){
    time_t t;
    t = ts; //Ignore fract seconds
	struct tm* buf = gmtime(&t);
    time_t seconds_since_sunday = (buf->tm_sec + 60*(buf->tm_min + 60*(buf->tm_hour + 24*buf->tm_wday)));
    t -= seconds_since_sunday;
    //TODO should GPS UTC leap seconds differnce be added here?
    // Tested, but it seems very wrong when I do. Perhaps this is added somewhere else
    return (double) t;
}


void set_posmv_alt_gps_epoch(double ts){
    if (alt_gps_epoch==0){
        alt_gps_epoch = calc_epoch(ts);
        time_t raw_time = (time_t) alt_gps_epoch;
        fprintf(stderr,"Setting POSMV alt epoch to: ts=%0.3f %s  ",alt_gps_epoch,ctime(&raw_time));
    }
}


void posmv_init(void){
    #ifdef COLLECT_POSMV_STATS
    for (uint32_t id=0;id<POSMV_ID_MAX;id++){
        posmv_stat_packet_count[id]=0;
    }
    #endif
}
void posmv_print_stats(void){
    #ifdef COLLECT_POSMV_STATS
    uint32_t total_data = 0;
    for (uint32_t id=0;id<POSMV_ID_MAX;id++){
        total_data+=posmv_stat_packet_count[id];
    }
    if (total_data){
        fprintf(stderr,"------- Reson POSMV data packet count -----\n");
    }
    for (uint32_t id=0;id<POSMV_ID_MAX;id++){
        if(posmv_stat_packet_count[id]>0){
            fprintf(stderr, "POSMV record type %5d: %8d\n",id,posmv_stat_packet_count[id]);
        }
    }
    if (total_data){
        fprintf(stderr,"\n");
    }
    #endif
}

uint8_t posmv_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_POSMV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<200;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = posmv_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            int type = posmv_identify_packet(data, len, &ts);
            int req_types[] = {1,102};    //So far only support posmv type 1 or 102 
            size_t n_req_types = 2;
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
    if (group3_cnt==0)
        return alt_gps_epoch + gps;  //If we dont have any group 3 data GPS week number info, use alternative epoch
    else
        return posmv3.gps_epoch + gps;
}


#define POSMV_BUFFER_SIZE (MAX_POSMV_PACKET_SIZE+4)
static int buffered_read(int fd, void* data, int len){
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
    #ifdef COLLECT_POSMV_STATS
    if(gid<POSMV_ID_MAX){
        posmv_stat_packet_count[gid]++;
    }
    #endif
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
    if(verbose) fprintf(stderr,"GID=%d, Count=%d, Time1=%f, Time2=%f, timetype=%d\n",gid,count,time1,time2,timetype);

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


int posmv_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
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
	
    char str_buf[256];
    sprintf_unix_time(str_buf, ts);
    if(verbose) fprintf(stderr,"POS_MODE_POSMVdata gid=%d %s count=%d time1=%f time2=%f dist=%f timetype=%d disttype=%d cs=%d\n",gid,str_buf,count,time1,time2,dist,timetype,disttype,cs);
    //if(gid==1 || gid==102) fprintf(stderr,"POS_MODE_POSMVdata gid=%d %s count=%d time1=%f time2=%f dist=%f timetype=%d disttype=%d cs=%d\n",gid,str_buf,count,time1,time2,dist,timetype,disttype,cs);

    switch (gid){
        default:
            return NO_NAV_DATA;
        case 2: //#Vessel Navigation Performance Metrics
            group2_cnt++;
            
            posmv2.north_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.east_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.down_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.north_vel_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.east_vel_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.down_vel_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.roll_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.pitch_rms_error = *(((float*)(dp)));dp+=4;
            posmv2.heading_rms_error = *(((float*)(dp)));dp+=4;

            aux_navdata->hor_accuracy = sqrtf((posmv2.north_rms_error*posmv2.north_rms_error) + (posmv2.east_rms_error*posmv2.east_rms_error));
            aux_navdata->vert_accuracy = posmv2.down_rms_error;

            //Recalculate time as this depends on the new received gps_utc_diff
            ts = posmv_time_to_unix_time(time1,time2,timetype);
            //*ts_out = ts;
            posmv2.ts = ts;
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
	
            posmv3.gps_epoch = POS_MODE_POSMV_GPS_EPOC+(60*60*24*7*posmv3.gps_week) - posmv3.gps_utc_diff;
            
            aux_navdata->mode = posmv3.mode;
            aux_navdata->sv_n = posmv3.sv_n;
            if (ts > (posmv2.ts + 10.)){ //Replace hdop and vdop with data from group3 if last group2 data is older than 10 sec
                aux_navdata->hor_accuracy = posmv3.hdop;
                aux_navdata->vert_accuracy = posmv3.vdop;
            }
            aux_navdata->dgps_latency = posmv3.dgps_latency;
            
            /* The following two values are used to calculate absolute time from POSMV time. If we dont have Group3 data, we use alt_gps_epoch instead*/
            aux_navdata->gps_week = posmv3.gps_week;                    //This is used to derive absolute time from seconds in week
            aux_navdata->gps_utc_diff = posmv3.gps_utc_diff;            //This is used to derive time from GPS time

            aux_navdata->gps_nav_latency = posmv3.gps_nav_latency;
            aux_navdata->geoid_separation = posmv3.geoid_separation;    //This is used to derive altitude from height above geoid
            aux_navdata->gps_type = posmv3.gps_type;
            aux_navdata->gps_status = posmv3.gps_status;

            //Recalculate time as this depends on the new received gps_utc_diff
            ts = posmv_time_to_unix_time(time1,time2,timetype);
            //*ts_out = ts;
            posmv3.ts = ts;
            if(group3_cnt<3){ 
                fprintf(stderr, "POS_MODE_POSMV GRP3 ts=%f,mode=%d,nSV=%d,count2=%d,hdop=%f,vdop=%f,dgps_latency=%f,dgps_statid=%d,WeekNr=%d,GPS_UTC_diff=%f,gps_nav_latency=%f,Geoid_sep=%f,gps_type=%d,gps_status=%d\n",ts,posmv3.mode,posmv3.sv_n,posmv3.count2,posmv3.hdop,posmv3.vdop,posmv3.dgps_latency,posmv3.dgps_statid,posmv3.gps_week,posmv3.gps_utc_diff,posmv3.gps_nav_latency,posmv3.geoid_separation,posmv3.gps_type,posmv3.gps_status);
            }
            return NO_NAV_DATA;
        case 1:
        case 102:
            //If group 1 data has been found in data steam, ignore all other navigation groups to prevent duplicate navigation
            if ((group1_cnt>0) && (gid != 1)){
                return NO_NAV_DATA;     
            }
            if (gid==1){
                group1_cnt++;
                //Fill out navdata from posmv 1 data
                navdata->ts = ts;
                fprintf(stderr,"PosMV Grp1 Ts= %f\n",ts);
                navdata->lat = *(((double*)(dp)));dp+=8;
                navdata->lon = *(((double*)(dp)));dp+=8;
                navdata->alt = *(((double*)(dp)));dp+=8;
                dp+=12;  //Skip time derivates (North, East, Down)
                navdata->roll = *(((double*)(dp)));dp+=8;
                navdata->pitch = *(((double*)(dp)));dp+=8;
                navdata->yaw = *(((double*)(dp)));dp+=8;
                
                navdata->course = *(((double*)(dp)));dp+=8;
                /*navdata->track_angle = *(((float*)(dp)));*/dp+=4;
               

                dp+=28;  //Skip time derivates
                dp+=1;   //Skip status flag
                /*pad = *(((uint16_t*)(dp)));*/ dp+=1;
                cs = *(((uint16_t*)(dp)));dp+=2;
                //end = *(((uint16_t*)(dp)));dp+=2;
            }
            else if (gid==102){
                group102_cnt++;
                //Fill out navdata from posmv 102 data
                navdata->ts = ts;
                navdata->lat = *(((double*)(dp)));dp+=8;
                navdata->lon = *(((double*)(dp)));dp+=8;
                navdata->alt = *(((double*)(dp)));dp+=8;
                dp+=12;  //Skip time derivates (Along track, across track, down)
                navdata->roll = *(((double*)(dp)));dp+=8;
                navdata->pitch = *(((double*)(dp)));dp+=8;
                navdata->yaw = *(((double*)(dp)));dp+=8;
                navdata->course = *(((double*)(dp)));dp+=8;
                navdata->heave = *(((float*)(dp)));dp+=4;
                dp+=24;  //Skip time derivates
                //pad = *(((uint16_t*)(dp)));dp+=2;
                cs = *(((uint16_t*)(dp)));dp+=2;
                //end = *(((uint16_t*)(dp)));dp+=2;
            }

            if(verbose) fprintf(stderr, "POS_MODE_POSMV GRP%d lat=%5.2f,lon=%5.2f,alt=%5.2f,roll=%5.2f,pitch=%5.2f,yaw=%5.2f,course=%5.2f,heave=%5.2f\n",gid,navdata->lat,navdata->lon,navdata->alt,navdata->roll,navdata->pitch,navdata->yaw,navdata->course,navdata->heave);
            navdata->lat *=(M_PI/180);
            navdata->lon *=(M_PI/180);
            navdata->roll *=(M_PI/180);
            navdata->pitch *=(M_PI/180);
            navdata->yaw *=(M_PI/180);
            navdata->course *=(M_PI/180);
            navdata->alt -= aux_navdata->geoid_separation;

            //fprintf(stderr,"PROJ %s\n",proj?"true":"false");
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
            else{
                fprintf(stderr,"No proj defined\n");
            }

            navdata->z += z_offset;
            #if 0
            //By having an alternative gps epoch derived from data, we can go without group3 data
            if (group3_cnt==0){
                fprintf(stderr, "No Posmv group 3 data for group %d %d\n",group102_cnt);
                return NO_NAV_DATA;
            }
            if ((posmv3.ts-navdata->ts)>3600. || (navdata->ts-posmv3.ts)>3600.) {
                fprintf(stderr, "Posmv group 3 data out of sync for group1_cnt=%d group102_cnt=%d group3_cnt=%d (ts3=%f ts102=%f)\n",group1_cnt, group102_cnt,group3_cnt,posmv3.ts,navdata->ts);
                return NO_NAV_DATA;
            }
            #endif
            //Only update time with new nav data
            *ts_out = ts;
            return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    return NO_NAV_DATA;
}
