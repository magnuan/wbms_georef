#ifndef __JSON_OUTPUT_H__
#define __JSON_OUTPUT_H__

#include "wbms_georef.h"

int write_json_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, aux_navdata_t *aux_navdata,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf);



typedef struct{
    char* file_type;        //s7k, wbms, posmv, etc
    char* file_version;     //"2.0"
    char* data_type;        //mbes, sbp, lidar, svp
    char* sensor_type;      //i77h, etc
    uint8_t  has_navigation;
    uint8_t  has_sensor;
    uint8_t  has_aux;          //SV-profile, etc
    uint32_t datapoints;
    uint32_t navigation_points;
    double start_time;
    float duration;
    double latitude;        //Ceter of nav line
    double longitude;       
    float altitude;
    float line_length;
    float start_stop_distance;
    float mean_depth;       // For sonars, this is defined as depth under sonar
    float coverage_area;
    uint32_t num_record_types;
    record_count_t records[];
}file_stats_t;

int write_stats_json_to_buffer(file_stats_t* stats, /*OUTPUT*/char* outbuf);
#endif
