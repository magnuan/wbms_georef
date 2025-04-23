#ifndef __POSMV_H__
#define __POSMV_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"


//Just a guess, needs to be figuread out from proticol definition
#define MAX_POSMV_PACKET_SIZE 32768

#define POSMV_ID_MAX 32768

//GRP2 params		
typedef struct{
	double ts;
    float north_rms_error;
    float east_rms_error;
    float down_rms_error;
    float north_vel_rms_error;
    float east_vel_rms_error;
    float down_vel_rms_error;
    float roll_rms_error;
    float pitch_rms_error;
    float heading_rms_error;
}posmv2_t;

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
    double gps_epoch;
}posmv3_t;

uint8_t posmv_test_file(int fd);
double posmv_time_to_unix_time(double time1,double time2,uint8_t timetype);
int posmv_seek_next_header(int fd);
int posmv_fetch_next_packet(char * data, int fd);
int posmv_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int posmv_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

void posmv_init(void);
void posmv_print_stats(void);
void set_posmv_alt_gps_epoch(double ts);

uint32_t posmv_num_record_types(void);
uint32_t posmv_get_record_count(record_count_t* records);
#endif
