#ifndef __SBET_NAV_H__
#define __SBET_NAV_H__
#include "proj_wrapper.h"

#define SBET_PACKET_SIZE (8*17)

uint8_t sbet_test_file(int fd);
int sbet_nav_seek_next_header(int fd);
int sbet_nav_fetch_next_packet(char * data, int fd);
int sbet_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);

void set_sbet_epoch(double ts);
int sbet_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata);

#endif
