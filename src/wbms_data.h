#ifndef __WBMS_DATA_H__
#define __WBMS_DATA_H__
#include "bathy_packet.h"

void wbms_set_sensor_offset(offset_t* s);
uint8_t wbms_test_file(int fd,int* version);
int wbms_seek_next_header(int fd);
int wbms_fetch_next_packet(char * data, int fd);
int wbms_identify_packet(char* databuffer, uint32_t len, double* ts_out, int* version);

uint32_t wbms_georef_data( bath_data_packet_t* bath, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/ output_data_t* outbuf,/*INPUT*/ uint32_t force_bath_version);


#endif
