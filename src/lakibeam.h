#ifndef __LAKIBEAM_H__
#define __LAKIBEAM_H__

#define LAKIBEAM_LIDAR_PACKET_SIZE (1206)
#define MAX_LAKIBEAM_LIDAR_PACKETS (8)


void lakibeam_set_epoch(double epoch);
void lakibeam_set_sensor_offset(offset_t* s);
uint8_t lakibeam_test_file(int fd);
int lakibeam_seek_next_header(int fd);
int lakibeam_fetch_next_packet(char * data, int fd);
int lakibeam_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in);
uint32_t lakibeam_georef_data( uint16_t* data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/  output_data_t* outbuf);
uint32_t  lakibeam_count_data( uint16_t * sensor_data_buffer,double *ts);

#endif
