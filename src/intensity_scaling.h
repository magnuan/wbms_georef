#ifndef __INTENSITY_SCALING_H__
#define __INTENSITY_SCALING_H__
#include "wbms_georef.h"

#define INTENSITY_ANGLE_MAX_VALUES 128
#define INTENSITY_ANGLE_STEP (0.5*M_PI/INTENSITY_ANGLE_MAX_VALUES)

typedef struct{
	float angle_rad;
	float intensity_scale;
}intensity_angle_corr_t;

extern intensity_angle_corr_t intenity_angle_corr_table[INTENSITY_ANGLE_MAX_VALUES];

int read_intensity_angle_corr_from_file(char* fname, const float d_angle, const size_t count_out, /*output*/ intensity_angle_corr_t* corr_out);
float calc_intensity_range_scaling(float range,  sensor_params_t* sensor_params);
float calc_footprint_scaling(float range, float aoi, float beam_angle, float eff_plen, sensor_params_t* sensor_params, /*OUTPUT*/ float* footprint);
float calc_beam_time_response(float range,float aoi, float beam_angle, float plen, sensor_params_t* sensor_params);
float calc_beam_footprint(float range,float aoi, float beam_angle, float plen, sensor_params_t* sensor_params);
float calc_ara_scaling(float aoi, sensor_params_t* sensor_params);

#endif
