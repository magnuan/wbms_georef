#ifndef __WBM_TOOL_NAV_H__
#define __WBM_TOOL_NAV_H__
#include "proj_wrapper.h"

#define MAX_WBM_TOOL_PACKET_SIZE (1024)

uint8_t wbm_tool_nav_test_file(int fd);
int wbm_tool_nav_seek_next_header(int fd);
int wbm_tool_nav_fetch_next_packet(char * data, int fd);
int wbm_tool_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);

int wbm_tool_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#endif
