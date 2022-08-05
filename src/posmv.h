#ifndef __POSMV_H__
#define __POSMV_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"


//Just a guess, needs to be figuread out from proticol definition
#define MAX_POSMV_PACKET_SIZE 32768

//GRP3 params		
typedef struct{
	double ts;
	uint8_t mode ;
	uint8_t sv_n ;
	uint16_t count2 ;
	float hdop ;
	float vdop ;
	float dgps_latency ;
	uint16_t dgps_statid ;
	uint32_t gps_week;
	double gps_utc_diff; //Guessing on leap seconds (last adjust to 17 Dec 31th 2016)
	float gps_nav_latency ;
	float geoid_separation;
	uint8_t gps_type ;
	uint32_t gps_status ;
}posmv3_t;

posmv3_t* get_posmv3_ptr(void);
uint8_t posmv_test_file(int fd);
double posmv_time_to_unix_time(double time1,double time2,uint8_t timetype);
int posmv_seek_next_header(int fd);
int posmv_fetch_next_packet(char * data, int fd);
int posmv_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int posmv_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata);

#endif
