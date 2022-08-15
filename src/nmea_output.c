#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include "georef_tools.h"
#include "time_functions.h"
#include "wbms_georef.h"
#include <math.h>
#include "cmath.h"
#include "posmv.h"



int write_nmea_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, /*OUTPUT*/char* outbuf){
	uint32_t len=0;
    uint32_t sstart;

    pos_ix = find_closest_index_in_posdata(posdata,pos_ix, ts);
    navdata_t* pos = &(posdata[pos_ix]);

	uint16_t t_year; uint16_t t_doy; uint8_t t_hour; uint8_t t_min; float t_sec;
    gm_to_irigb(ts, &t_year, &t_doy, &t_hour, &t_min, &t_sec);

        
    time_t ts_time_t = ts;
    struct tm  ts_tm;
    ts_tm = *localtime(&ts_time_t);
    char       ts_str[80];

	double lat = pos->lat*180/M_PI;
	char lat_dir;
	if (lat>=0){
		lat_dir = 'N';
	}
	else{
		lat = -lat;
		lat_dir = 'S';
	}
	uint16_t lat_deg = floorf(lat);
	double	 lat_min = (lat-lat_deg)*60;

    float heading_deg = pos->yaw*180/M_PI;
    float course_deg = pos->course*180/M_PI;

	double lon = pos->lon*180/M_PI;
	char lon_dir;
	if (lon>=0){
		lon_dir = 'E';
	}
	else{
		lon = -lon;
		lon_dir = 'W';
	}
	uint16_t lon_deg = floorf(lon);
	double	 lon_min = (lon-lon_deg)*60;
	float  hdop = pos->hor_accuracy;
    
    //Calculate vessel horizontal speed
    navdata_t* pos_old = &(posdata[(pos_ix+NAVDATA_BUFFER_LEN-10)%NAVDATA_BUFFER_LEN]); //Navdata 10 samples back in time TODO there is a slight risk of posdata log wrapping when doing this
    float dx = pos->x - pos_old->x;
    float dy = pos->y - pos_old->y;
    float dt = pos->ts - pos_old->ts;
    float speed = sqrtf(dx*dx+dy*dy)/dt;
	
	//TODO These values are just hard coded for now. Need to get them from input data when available
	uint8_t gps_status = 2;
	uint16_t sv_n = 10;
	float geoid_sep = 0.0;
	float dgps_latency = 0.0;
	uint16_t dgps_statid = 0; 

	posmv3_t* posmv3 = get_posmv3_ptr();
	if(posmv3){
		/*switch (posmv3->mode){
			case 1: case 2:
				gps_status = 1;break;
			case 3: case 4:
				gps_status = 2;break;
			default:
				gps_status = 0;
		}*/
		gps_status = posmv3->gps_status;
		sv_n = posmv3->sv_n;
		//hdop = posmv3->hdop;
		geoid_sep = posmv3->geoid_separation;
		dgps_latency = posmv3->dgps_latency;
		dgps_statid = posmv3->dgps_statid;
	}
	 

    // $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
    sstart=len;
    len += sprintf(&(outbuf[len]),"$GPGGA,"); // Message ID
    len += sprintf(&(outbuf[len]),"%02d%02d%02d.%1d,",t_hour, t_min, (int)(floorf(t_sec)), ((int)floorf((t_sec*10)))%10); // UTC of position fix
    len += sprintf(&(outbuf[len]),"%02d%011.8f,%c,",lat_deg, lat_min,lat_dir); // Latitude
    len += sprintf(&(outbuf[len]),"%03d%011.8f,%c,",lon_deg, lon_min,lon_dir); // Longitude
    len += sprintf(&(outbuf[len]),"%1d,",gps_status); 		// GPS Quality indicator
    len += sprintf(&(outbuf[len]),"%02d,",sv_n); 	  		// Number of SVs in use, range from 00 through to 24+ 
    len += sprintf(&(outbuf[len]),"%3.1f,",hdop); 			// HDOP 
    len += sprintf(&(outbuf[len]),"%0.3f,M,", -pos->z); 	// Orthometric height (MSL reference) 
    len += sprintf(&(outbuf[len]),"%0.3f,M,", geoid_sep); 	// Geoid separation 
    if (dgps_statid)
		len += sprintf(&(outbuf[len]),"%3.1f %04d",dgps_latency,dgps_statid); 					// Time since last DGPS update and DGPS reference station id, both blank
	else
		len += sprintf(&(outbuf[len]),","); 					// Time since last DGPS update and DGPS reference station id, both blank
	
	//Checksum does not include $-sign
	char cs = 0;
	for (size_t ii=sstart+1;ii<len;ii++){
		cs ^= outbuf[ii];
	}
    len += sprintf(&(outbuf[len]),"*%02X",cs); 					// Checksum
	len += sprintf(&(outbuf[len]),"\r\n"); // <CR><LF>


    // $GPHDT
    sstart=len;
    len += sprintf(&(outbuf[len]),"$GPHDT,"); // Message ID
    len += sprintf(&(outbuf[len]),"%06.2f,T",heading_deg); // True heading
	//Checksum does not include $-sign
	cs = 0;
	for (size_t ii=sstart+1;ii<len;ii++){
		cs ^= outbuf[ii];
	}
    len += sprintf(&(outbuf[len]),"*%02X",cs); 					// Checksum
	len += sprintf(&(outbuf[len]),"\r\n"); // <CR><LF>

    // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4, 084.4, 230394,003.1,W*6A    
    sstart=len;
    len += sprintf(&(outbuf[len]),"$GPRMC,"); // Message ID
    len += sprintf(&(outbuf[len]),"%02d%02d%02d,",t_hour, t_min, (int)(floorf(t_sec))); // UTC of position fix
    len += sprintf(&(outbuf[len]),"A,");                                        // A for Active (TODO when to set to V for Void)
    len += sprintf(&(outbuf[len]),"%02d%011.8f,%c,",lat_deg, lat_min,lat_dir);  // Latitude
    len += sprintf(&(outbuf[len]),"%03d%011.8f,%c,",lon_deg, lon_min,lon_dir);  // Longitude
    len += sprintf(&(outbuf[len]),"%05.1f,",speed*3600./1852.); 			    // Speed in knots 
    len += sprintf(&(outbuf[len]),"%05.1f,",course_deg);                        // True heading
    strftime(ts_str, sizeof(ts_str), "%d%m%y", &ts_tm);
    len += sprintf(&(outbuf[len]),"%s",ts_str);                                // UTC date
    //len += sprintf(&(outbuf[len]),",,R");                                     // Magnetic variation, and direction not given, Mode set to R for RTK (TODO should be set based on nav data)
	//Checksum does not include $-sign
	cs = 0;
	for (size_t ii=sstart+1;ii<len;ii++){
		cs ^= outbuf[ii];
	}
    len += sprintf(&(outbuf[len]),"*%02X",cs); 					// Checksum
	len += sprintf(&(outbuf[len]),"\r\n"); // <CR><LF>

               
/* 
                case LAT: len += sprintf(&(outbuf[len]),"%11.7f",pos->lat*180/M_PI);break;
                case LON: len += sprintf(&(outbuf[len]),"%11.7f",pos->lon*180/M_PI);break;
                case X: len += sprintf(&(outbuf[len]),"%11.3f",pos->y);break;
                case Y: len += sprintf(&(outbuf[len]),"%11.3f",pos->x);break;
                case Z: len += sprintf(&(outbuf[len]),"%11.3f",-pos->z);break;
                case HOR_ACC: len += sprintf(&(outbuf[len]),"%11.3f",pos->hor_accuracy);break;
                case VERT_ACC: len += sprintf(&(outbuf[len]),"%11.3f",pos->vert_accuracy);break;
                case YAW: len += sprintf(&(outbuf[len]),"%11.3f",pos->yaw*180/M_PI);break;
                case PITCH: len += sprintf(&(outbuf[len]),"%11.3f",pos->pitch*180/M_PI);break;
                case ROLL: len += sprintf(&(outbuf[len]),"%11.3f",pos->roll*180/M_PI);break;
*/
        
	return len;
}

