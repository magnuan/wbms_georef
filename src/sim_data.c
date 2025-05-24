#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <time.h>
#include "crc32.h"

#include <string.h>
#include <stdint.h>
#include "bathy_packet.h"
#include <math.h>
#include "time_functions.h"

#include "cmath.h"
#include "wbms_georef.h"
#include "sim_data.h"
#include "georef_tools.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <corecrt_math_defines.h>
#endif


static float min_sim_data_period = 0.5;


void set_min_sim_data_period(float val){
    min_sim_data_period = val;
}

int sim_fetch_next_packet(char * data, int fd){
	return 1;
}

int sim_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in){
    static double ts;
    if((ts<(ts_in-2-POS_PREFETCH_SEC)) || (ts>(ts_in+2))){ //If we get toom much out of sync with input time source (navigation time) jump
        ts = ts_in;
        *ts_out = ts ;
        return 1;
    }
    else if ( ts_in > ts + min_sim_data_period){
        ts += min_sim_data_period;        //Otherwise just generate data at a fixed interval
        *ts_out = ts ;
        return 1;
    }
    *ts_out = ts ;
    return 0;
}




uint32_t sim_georef_data( navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/ output_data_t* outbuf){
    double* x = &(outbuf->x[0]);
    double* y = &(outbuf->y[0]);
    double* z = &(outbuf->z[0]);
    float* z_var = &(outbuf->z_var[0]);
    float* intensity = &(outbuf->i[0]);
    float* beam_range = &(outbuf->range[0]);
    float* beam_angle = &(outbuf->teta[0]);
    float* beam_steer = &(outbuf->steer[0]);
    int * beam_number = &(outbuf->beam[0]);
    float* swath_y = &(outbuf->swath_y[0]);
    float* aoi = &(outbuf->aoi[0]);
    float* upper_gate_range = &(outbuf->up_gate[0]);
    float* lower_gate_range = &(outbuf->low_gate[0]);
    float* quality = &(outbuf->quality[0]);
    float* strength = &(outbuf->strength[0]);
    float* tx_angle_out = &(outbuf->tx_angle);
    float* fs_out = &(outbuf->sample_rate);
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);
	
	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;

    //Use timestamp from last nav data
    double ts = posdata[pos_ix].ts;

    if (calc_interpolated_nav_data( posdata, pos_ix, ts, /*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
        return 0;
    }

	if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }

    x[0] = nav_x;
    y[0] = nav_y;
    z[0] = nav_z;
    z_var[0] = 0.f;
    intensity[0] = 0.f;
    beam_range[0] = 0.f;
    beam_angle[0] = 0.f;
    beam_steer[0] = 0.f;
    beam_number[0] = 0;
    swath_y[0] = 0.f;
    aoi[0] = 0.f;
    upper_gate_range[0] = 0.f;
    lower_gate_range[0] = 0.f;
    quality[0] = 3;
    strength[0] = 0.f;
    tx_angle_out[0] = 0.f;
    fs_out[0] = 78125.f;
    sv_out[0] = 0.f;
    tx_freq_out[0] = 0.f;
    multiping_index_out[0] = 0;
    multifreq_index_out[0] = 0;

    return 1;
}
