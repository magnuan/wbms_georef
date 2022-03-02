#ifndef __XTF_NAV_H__
#define __XTF_NAV_H__
#include "proj_wrapper.h"
#define MAX_XTF_NAV_PACKET_SIZE (1024)

uint8_t xtf_test_file(int fd);
int xtf_nav_seek_next_header(int fd);
int xtf_nav_fetch_next_packet(char * data, int fd);
int xtf_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int xtf_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata);

void xtf_nav_set_params(float timezone, char* input_projection_string);
#endif
