#ifndef __NMEA_NAV_H__
#define __NMEA_NAV_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"

#define MAX_NMEA_SEARCH_LEN (1024*1024)
#define MAX_NMEA_NAV_PACKET_SIZE (1024*1024)

typedef enum  { nmea_id_unknown=0, nmea_id_HEA, nmea_id_ORI, nmea_id_DEP, nmea_id_POS} nmea_sentence_id_e; 

uint8_t nmea_nav_test_file(int fd);
int nmea_nav_fetch_next_packet(char * data, int fd);
int nmea_nav_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out);
int nmea_nav_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#endif
