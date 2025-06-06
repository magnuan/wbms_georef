#ifndef __GSF_WRAPPER_H__
#define __GSF_WRAPPER_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"

void gsf_init(void);
void gsf_print_stats(void);

void gsf_set_sensor_offset(offset_t* s);
uint8_t gsf_test_nav_file(int fd);
uint8_t gsf_test_bathy_file(int fd);
int gsf_fetch_next_packet(char * data, int fd);
int gsf_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out);
int gsf_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);
int32_t gsf_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, uint16_t sector, /*OUTPUT*/ output_data_t* outbuf);
uint32_t gsf_count_data( char* databuffer,uint32_t databuffer_len, double* ts);
sensor_count_stats_t* gsf_get_count_stats(void);
uint32_t gsf_num_record_types(void);
uint32_t gsf_get_record_count(record_count_t* records);
const char * gsf_get_data_type(void);
uint8_t gsf_rewind(int fd);
uint8_t gsf_close(int fd);
            
void gsf_set_sensor_filename(const char* fname);
void gsf_set_nav_filename(const char* fname);
#endif
