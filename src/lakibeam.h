#ifndef __LAKIBEAM_H__
#define __LAKIBEAM_H__



/* Richbeam Lakibem lidar via IP/UDP */
#define LAKIBEAM_POINT_PER_BLOCK 16
#define LAKIBEAM_BLOCKS 12
#define LAKIBEAM_POINT_PER_PACK (LAKIBEAM_BLOCKS*LAKIBEAM_POINT_PER_BLOCK)


/* Richbeam Lakibem lidar via IP/UDP */
typedef struct __attribute__((packed)) {
    uint16_t distance1;
    uint8_t intensity1;
    uint16_t distance2;
    uint8_t intensity2;
} LakibeamPoint_t;

typedef struct __attribute__((packed)) {
    uint16_t header;
    uint16_t angle;
    LakibeamPoint_t point[LAKIBEAM_POINT_PER_BLOCK];
} LakibeamBlock_t;

typedef struct __attribute__((packed)) {
    LakibeamBlock_t block[LAKIBEAM_BLOCKS];
    uint32_t timestamp_us;
    uint16_t factory;
}LakibeamFrame_t;

typedef struct __attribute__((packed)) {
    LakibeamBlock_t block[LAKIBEAM_BLOCKS];
    uint64_t timestamp_us;
    uint16_t factory;
}ExtendedLakibeamFrame_t;


#define LAKIBEAM_LIDAR_PACKET_SIZE (sizeof(LakibeamFrame_t))
#define LAKIBEAM_EXTENDED_LIDAR_PACKET_SIZE (sizeof(ExtendedLakibeamFrame_t)) 


void lakibeam_set_epoch(double epoch);
void lakibeam_set_sensor_offset(offset_t* s);
uint8_t lakibeam_test_file(int fd,int* version);
int lakibeam_seek_next_header(int fd);
int lakibeam_fetch_next_packet(char * data, int fd);
int lakibeam_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in);
uint32_t lakibeam_georef_data( uint16_t* data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/  output_data_t* outbuf);
uint32_t  lakibeam_count_data( uint16_t * sensor_data_buffer,double *ts);

#endif
