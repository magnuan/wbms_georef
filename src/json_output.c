#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "georef_tools.h"
#include "time_functions.h"
#include "wbms_georef.h"
#include <math.h>
#include "cmath.h"
#include "posmv.h"



int write_json_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, /*OUTPUT*/char* outbuf){
	uint32_t len=0;

    pos_ix = find_closest_index_in_posdata(posdata,pos_ix, ts);
    navdata_t* pos = &(posdata[pos_ix]);

	double lat = pos->lat*180/M_PI;
	double lon = pos->lon*180/M_PI;
	
	//TODO These values are just hard coded for now. Need to get them from input data when available
	uint8_t gps_status = 0;
	uint16_t sv_n = 0;
	float  hdop = 0.0;
	float  vdop = 0.0;
	//float geoid_sep = 0.0;
	//float dgps_latency = 0.0;
	//uint16_t dgps_statid = 0; 

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
		hdop = posmv3->hdop;
		vdop = posmv3->vdop;
		//geoid_sep = posmv3->geoid_separation;
		//dgps_latency = posmv3->dgps_latency;
		//dgps_statid = posmv3->dgps_statid;
	}
    len += sprintf(&(outbuf[len]),"{"); 
    len += sprintf(&(outbuf[len]),"\"latitude\"=%11.7f",lat); 
    len += sprintf(&(outbuf[len]),",\"longitude\"=%11.7f",lon); 
    len += sprintf(&(outbuf[len]),",\"altitude\"=%0.3f",-pos->z);
    len += sprintf(&(outbuf[len]),",\"ts\"=%11.3f",ts);
    len += sprintf(&(outbuf[len]),",\"gps_status\"=%1d",gps_status);
    len += sprintf(&(outbuf[len]),",\"sv_n\"=%2d",sv_n);
    len += sprintf(&(outbuf[len]),",\"hdop\"=%0.3f",hdop);
    len += sprintf(&(outbuf[len]),",\"vdop\"=%0.3f",vdop);
    len += sprintf(&(outbuf[len]),"}\r\n"); 


    

               
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

