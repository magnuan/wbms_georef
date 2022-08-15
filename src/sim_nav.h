#ifndef __SIM_NAV_H__
#define __SIM_NAV_H__
#include "proj_wrapper.h"

int sim_nav_seek_next_header(int fd);
int sim_nav_fetch_next_packet(char * data, int fd);
int sim_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
void sim_nav_set_params(float speed);

int sim_nav_process_packet(double ts_in, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#endif
