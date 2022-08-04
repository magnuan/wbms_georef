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


#define MIN_SIM_DATA_PERIOD (0.5)

int sim_fetch_next_packet(char * data, int fd){
	return 1;
}

int sim_identify_packet(char* databuffer, uint32_t len, double* ts_out, double ts_in){
    //If last navigation data has not changed, return no new data
    if (*ts_out <= (ts_in+MIN_SIM_DATA_PERIOD))
        return 0;
    //Otherwise return a new dataset with same time as the navigation data    
	*ts_out = ts_in ; 
	return 1;
}




uint32_t sim_georef_data( navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, offset_t* sensor_offset,/*OUTPUT*/ output_data_t* outbuf){
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
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);

    x[0] = posdata[pos_ix].x;
    y[0] = posdata[pos_ix].y;
    z[0] = posdata[pos_ix].z;
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
    sv_out[0] = 0.f;
    tx_freq_out[0] = 0.f;
    multiping_index_out[0] = 0;
    multifreq_index_out[0] = 0;

    return 1;
}
