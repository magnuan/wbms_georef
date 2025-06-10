#ifndef __VELODYNE_H__
#define __VELODYNE_H__

#define VELODYNE_LIDAR_PACKET_SIZE (1206)
#define MAX_VELODYNE_LIDAR_PACKETS (8)


void velodyne_set_sensor_offset(offset_t* s);
uint8_t velodyne_test_file(int fd);
int velodyne_seek_next_header(int fd);
int velodyne_fetch_next_packet(char * data, int fd);
int velodyne_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in);
uint32_t velodyne_georef_data( uint16_t* data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/  output_data_t* outbuf);
uint32_t  velodyne_count_data( uint16_t * sensor_data_buffer,double *ts);

#endif
