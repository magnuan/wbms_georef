#ifndef __EELUME_SBD_NAV_H__
#define __EELUME_SBD_NAV_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"

#define MAX_EELUME_SBD_NAV_PACKET_SIZE (1024*1024)


uint8_t eelume_sbd_nav_test_file(int fd);
int eelume_sbd_nav_fetch_next_packet(char * data, int fd);
int eelume_sbd_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
int eelume_sbd_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#endif

