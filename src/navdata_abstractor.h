#ifndef _NAVDATA_ABSTRACTOR_H_
#define _NAVDATA_ABSTRACTOR_H_

typedef enum  { pos_mode_posmv=0, pos_mode_xtf=1,pos_mode_wbm_tool=2, pos_mode_sbet=3, pos_mode_sim=4, pos_mode_s7k=5, pos_mode_3dss_stream=6, pos_mode_eelume=7,pos_mode_nmea=8,pos_mode_sbet_csv=9, pos_mode_autodetect=10,pos_mode_unknown=11} pos_mode_e; 

extern const char *pos_mode_names[];
extern const char *pos_mode_short_names[];

extern navdata_t navdata[NAVDATA_BUFFER_LEN];
extern size_t navdata_ix;
extern uint32_t navdata_count;
extern aux_navdata_t aux_navdata;
extern uint16_t navdata_alt_mode;
extern PJ *proj_latlon_to_output_utm;


uint8_t navigation_test_file(int fd, pos_mode_e mode);
pos_mode_e navigation_autodetect_file(FILE* fp);
int navigation_fetch_next_packet(char * data, int fd, pos_mode_e mode);
int process_nav_data_packet(char* databuffer, uint32_t len, double ts_in, double* ts_out, pos_mode_e mode, double z_offset);


int navigation_num_record_types(pos_mode_e mode);
int navigation_get_record_count(pos_mode_e mode, record_count_t* records);

#endif
