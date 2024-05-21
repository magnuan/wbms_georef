#ifndef __GEOREF_TOOLS_H__
#define __GEOREF_TOOLS_H__
#include "wbms_georef.h"

int georef_to_global_frame(
					/* Sensor offset*/ 		offset_t* offset, 
					/*sensor data arrays*/ 	float* xs, float* ys, float* zs, uint32_t n, /* Detection points in SONAR frame */
					/*sonar sv */			float c,												/* Sound velocity for sonar data at sonar head, to apply raybending corrections. 0 for no correction*/
					/*Nav pos*/ 			double nav_x, double nav_y, double nav_z, 			/*Position in global coordinates (east,north,up)*/
					/*Nav attitude*/		float nav_yaw, float nav_pitch, float nav_roll, /*Rotations of posmv coordinates*/
                    /*Ray tracing mode*/    int ray_tracing_mode, float sonar_mounting_depth,
					/*OUTPUT*/
					/*sensor data arrays*/	double* x, double* y, double* z
					);

float apply_beam_correction_poly(float angle, float* poly, uint32_t order);
int attitude_test(sensor_params_t* sensor_params, float yaw,  float pitch,  float roll, float droll_dt, float dpitch_dt, float dyaw_dt); 
size_t find_closest_index_in_posdata(navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, double ts);
int calc_interpolated_nav_data( navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, double ts,/*OUTPUT*/ double* nav_x, double* nav_y, double* nav_z, float* nav_yaw, float* nav_pitch, float* nav_roll, float* nav_dyaw_dt, float* nav_dpitch_dt, float* nav_droll_dt);
int calc_interpolated_roll_and_z_vector(navdata_t posdata[NAVDATA_BUFFER_LEN], size_t  pos_ix, double ts, float t_dur, float fs, size_t N, /*output*/ float* roll_vector, float* z_vector);
void set_time_diff_limit(float t);
void set_use_sonar_sv_for_initial_ray_parameter(uint8_t val);
#endif
