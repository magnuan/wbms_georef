#ifndef __SBET_NAV_H__
#define __SBET_NAV_H__
#include "proj_wrapper.h"


void set_sbet_epoch(double ts);

/***************** SECTION FOR BINARY FORMATED SBET DATA ***************************************/
#define SBET_PACKET_SIZE (8*17)
uint8_t sbet_test_file(int fd);
int sbet_nav_seek_next_header(int fd);
int sbet_nav_fetch_next_packet(char * data, int fd);
int sbet_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int sbet_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);


/***************** SECTION FOR CSV FORMATED SBET DATA ***************************************/
#define MAX_SBET_CSV_NAV_PACKET_SIZE 1024
uint8_t sbet_csv_test_file(int fd);
int sbet_csv_nav_seek_next_header(int fd);
int sbet_csv_nav_fetch_next_packet(char * data, int fd);
int sbet_csv_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int sbet_csv_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#endif
