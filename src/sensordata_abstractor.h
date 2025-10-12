#ifndef _SENSORDATA_ABSTRACTOR_H_
#define _SENSORDATA_ABSTRACTOR_H_

typedef enum  {sensor_mode_wbms=1,sensor_mode_wbms_v5=2, sensor_mode_velodyne=3, sensor_mode_sim=4, sensor_mode_s7k=5, sensor_mode_3dss_stream=6, sensor_mode_gsf=7,sensor_mode_lakibeam=8,sensor_mode_end=9, sensor_mode_autodetect=20,sensor_mode_unknown=21} sensor_mode_e; 
extern const char *sensor_mode_names[];
extern char *sensor_mode_short_names[];
uint8_t sensor_test_file(int fd, sensor_mode_e mode, int* version);
sensor_mode_e sensor_autodetect_file(FILE* fp);
int sensor_fetch_next_packet(char * data, int fd, sensor_mode_e mode);
int sensor_identify_packet(char* databuffer, uint32_t len, double ts_in, double* ts_out, sensor_mode_e mode);
void sensor_get_sv_range(int fd, sensor_mode_e mode, float* min_sv, float* max_sv);

int sensor_num_record_types(sensor_mode_e mode);
int sensor_get_record_count(sensor_mode_e mode, record_count_t* records);

const char *  sensor_get_data_type(sensor_mode_e mode);
sensor_count_stats_t* sensor_get_count_stats(sensor_mode_e mode);
#endif
