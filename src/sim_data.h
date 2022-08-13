#ifndef __SIM_DATA_H__
#define __SIM_DATA_H__

void set_min_sim_data_period(float val);
int sim_fetch_next_packet(char * data, int fd);
int sim_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in);

uint32_t sim_georef_data( navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, offset_t* sensor_offset,/*OUTPUT*/ output_data_t* outbuf);


#endif
