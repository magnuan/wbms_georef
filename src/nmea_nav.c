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

#include <math.h>
#include "wbms_georef.h"
#include "nmea_nav.h"
#include "georef_tools.h"
#include "cmath.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif




uint8_t nmea_nav_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_NMEA_NAV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = nmea_nav_fetch_next_packet(data, fd);
        if (len > 0 ){
            double ts;
            pass=  nmea_nav_identify_sensor_packet(data, len, &ts);
            if (pass) break;
        }
        else{
            break;
        }
    }
    free(data);
    return pass;
}

unsigned char nmea_calc_checksum(char* s, size_t len){
    unsigned char cs = 0;
    char * c;
    c = s;
    for (size_t n = 0; n<len;n++){
    //while(*c != 0){
        cs = cs^(*c);
        c++;
    }
    return cs;
}



#include <errno.h>
int nmea_nav_fetch_next_packet(char * data, int fd){
    ssize_t K=0;        //Total number of bytes read
    
    while ( K++ < MAX_NMEA_SEARCH_LEN){
        ssize_t N=0;        //Number of bytes in packet
        uint8_t start = 0;
        uint8_t end = 0;
        while ( (N < (MAX_NMEA_NAV_PACKET_SIZE-1)) && (K++ < MAX_NMEA_SEARCH_LEN) ){
            ssize_t n = read(fd, &(data[N]),1);
            if (n != 1) return 0;               //No data read from source, abort
            if (data[N] == '$'){                //Start delimiter found, restart if allready started
                start=1;
                N=0;
            }
            if (start){                         //If started, look for asterix marking end of data
                if (N>=2){
                    if (data[N-2] == '*'){      //NMEA end char (+ two bytes checksum)
                        end = 1;
                        break;
                    }
                }
                N++;
            }
        }
        if (end==0) continue;
        if (N<8) continue;
        uint8_t cs = nmea_calc_checksum(&(data[1]),N-3); //Checksum, XOR bytes between $ and *
        char cs_str[3];
        sprintf(cs_str, "%02X", cs);
        if (strncmp(&(data[N-1]),cs_str,2)) continue;

        data[N+1] = 0; //Null terminate string
        //fprintf(stderr,"NMEA message read: %s\n",data);
        return N;
    }
    return 0;

}


int nmea_nav_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out){
    
    char talkerID[3];
    memcpy(&talkerID,&(databuffer[1]),2);
    talkerID[2] = 0;
    char msgID[4];
    memcpy(&msgID,&(databuffer[3]),3);
    msgID[3] = 0;
    

    nmea_sentence_id_e id;
    if      (strncmp(msgID,"HEA",3)==0) id = nmea_id_HEA;
    else if (strncmp(msgID,"ORI",3)==0) id = nmea_id_ORI;
    else if (strncmp(msgID,"DEP",3)==0) id = nmea_id_DEP;
    else if (strncmp(msgID,"POS",3)==0) id = nmea_id_POS;
    else id = nmea_id_unknown;
 
    double ts; 
    long int ts_ms;
    switch(id){
        case nmea_id_HEA:
        case nmea_id_ORI:
        case nmea_id_DEP:
        case nmea_id_POS:
            sscanf(databuffer+7,"%*d,%*f,%ld", &ts_ms);
            ts = (double)(ts_ms) * 1e-3;
            break;
	    default:
            ts = 0.0;
    }
    *ts_out = ts;

    //fprintf(stderr, "Talker ID = %s,  Message ID = %s ID=%d ts=%f\n", talkerID, msgID, (int)id, ts);
	
	return (int) id;
}


/**
*
* This function is assuming that position and attitude data comes synched, with same time stamp. As from an integrated navigation system
* For more generic nmea_nav streams, where attitude and position messages might come from different sources, at different time and rate. This will not work
*/
int nmea_nav_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
	static float last_alt;
    
    static navdata_t navdata_collector; 
	static uint8_t have_pos; 
	static uint8_t have_attitude; 
	static uint8_t have_heading; 
	static uint8_t have_depth; 

    double ts;
	if (len <=0 )
		return NO_NAV_DATA;	

    nmea_sentence_id_e id = (nmea_sentence_id_e) nmea_nav_identify_sensor_packet(databuffer,len, &ts);
	
    *ts_out = ts;

	//So far we only process nmea_nav record 1015 and 1016 for navigation
	if (id == nmea_id_unknown)
		return NO_NAV_DATA;	

    // The data collector needs to collect one set of posision and one set of attitude data with same timestamp
    // to generate a navigation out entry. 
    if ( (id==nmea_id_HEA) || (id==nmea_id_ORI) || (id==nmea_id_POS) ){  //Timestamp taken from these three sentences have to agree
        if (ts != navdata_collector.ts){ 	// If this data has a new timestamp
            navdata_collector.ts = ts;
            //New entry, still no navigation data 
            if (have_pos || have_attitude || have_heading){
                if (! (have_pos && have_attitude && have_heading) ){
                    //fprintf(stderr,"Dumping incomplete nav dataset\n");
                }
            }
            have_pos=0;
            have_attitude=0;
            have_heading=0;
        }
    }

    
    double lon, lat;
    char latNS, lonEW;
    float depth, alt;
    char depth_unit, alt_unit;
    float heading, roll, pitch;

	switch (id){
		case nmea_id_POS: // Position
            // $EIPOS,064,102150.677,1648722110677,63.461563,N,10.546222,E*72
            if(sscanf(databuffer+7,"%*d,%*f,%*d,%lf,%c,%lf,%c", &lat,&latNS,&lon,&lonEW) >=4){
                if(latNS=='S') lat *=-1;
                if(lonEW=='W') lon *=-1;
                navdata_collector.lon = lon*M_PI/180;
                navdata_collector.lat = lat*M_PI/180;
                have_pos = 1;
                //fprintf(stderr,"POS Lat=%f  Lon=%f\n ", lat,lon);
                
                if (proj){
                    latlon_to_kart(navdata_collector.lon, navdata_collector.lat , last_alt, proj,     &(navdata_collector.x), &(navdata_collector.y) , &(navdata_collector.z));
                }
            }
			break;
		case nmea_id_DEP: // Depth
            // $EIDEP,058,102150.285,1648722110285,232.19,m,006.05,m*61
            if(sscanf(databuffer+7,"%*d,%*f,%*d,%f,%c,%f,%c", &depth,&depth_unit,&alt,&alt_unit) >=4){
                navdata_collector.alt = -depth;
                last_alt = -depth;
                have_depth = 1;
                //fprintf(stderr,"DEPTH depth=%f %c  Alt=%f %c\n ", depth, depth_unit, alt, alt_unit);
            }
			break;
		case nmea_id_HEA: // Heading
            // $EIHEA,047,102150.677,1648722110677,301.07*4D
            if(sscanf(databuffer+7,"%*d,%*f,%*d,%f", &heading) >=1){
                navdata_collector.yaw = heading*M_PI/180.;
                have_heading = 1;
                //fprintf(stderr,"HEADING heading=%f\n ", heading);
            }
			break;
		case nmea_id_ORI: // Orientation
            // $EIORI,054,102150.677,1648722110677,013.88,000.11*62
            if(sscanf(databuffer+7,"%*d,%*f,%*d,%f,%f", &roll, &pitch) >=2){
                navdata_collector.roll = roll*M_PI/180;
                navdata_collector.pitch = pitch*M_PI/180;
			    navdata_collector.heave = 0.0;
                have_attitude = 1;
                //fprintf(stderr,"ORI pitch=%f Roll=%f \n ", pitch, roll);
            }
			break;

		default:
            break;
	}

	if (have_pos && have_attitude && have_depth && have_heading){
        memcpy(navdata, &navdata_collector, sizeof(navdata_t));
        have_pos = 0;
        have_attitude = 0;	
        have_heading = 0;
        have_depth = 0;

        return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    return NO_NAV_DATA;
}



