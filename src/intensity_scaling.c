#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "linalg.h"
#include "non_posix.h"

#include "cmath.h"
#include <math.h>
#include "wbms_georef.h"
#include "raytracing.h"
#include "time_functions.h"
#include "intensity_scaling.h"
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

//static uint8_t verbose = 0;

intensity_angle_corr_t intenity_angle_corr_table[INTENSITY_ANGLE_MAX_VALUES];

static int comp_intensity_angle_corr_on_angle_func(const void *a, const void *b){
	intensity_angle_corr_t *x = (intensity_angle_corr_t *)a;
	intensity_angle_corr_t *y = (intensity_angle_corr_t *)b;
	return ((x->angle_rad - y->angle_rad)>0);
}

int read_intensity_angle_corr_from_file(char* fname, const float d_angle_rad, const size_t count_out, /*output*/ intensity_angle_corr_t* intensity_angle_corr_out){
	FILE * fp = fopen(fname,"r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	char * c;
	size_t count_in = 0;
    const size_t max_count = 256;
	
    
    intensity_angle_corr_t* intensity_angle_corr_in;
	intensity_angle_corr_in = malloc(max_count*sizeof(intensity_angle_corr_t));
	
    if (fp==NULL){
		fprintf(stderr,"Could not open file %s for reading intensity angle corr\n",fname);
		return -1;
	}


	//Read in all angle correction measurement entries
	fprintf(stderr,"Reading in intensity corr from file %s\n",fname);
	while ((read = getline(&line, &len, fp)) != -1) {
		if (read>0){
			c = line;
			while (*c==' ' || *c=='\t' || *c=='\n' || *c=='\r') c++; 	//Skip leading white spaces
			if(*c==0) continue;											//End of line
			if(*c=='#') continue;										//Comment
	        float intensity_meas, angle_deg;
            if (sscanf(c,"%f,%f",&angle_deg, &intensity_meas)==2){
                intensity_angle_corr_in[count_in].intensity_scale = 1000./intensity_meas;
                intensity_angle_corr_in[count_in].angle_rad = angle_deg*M_PI/180.;
                count_in++;
            }
		}
        if( count_in >= max_count-1 ) break;
	}
    // When resample use  a filter of this length (in radians)
    const float angle_rad_avg = 5.f*M_PI/180;
    
    fprintf(stderr,"# Sort measurements on angle\n");
	qsort(intensity_angle_corr_in,count_in,sizeof(intensity_angle_corr_t),comp_intensity_angle_corr_on_angle_func);
    fprintf(stderr,"# Count in = %d, max_angle = %0.2f deg  intensity_scale=%7.2f\n", (int) count_in, intensity_angle_corr_in[count_in-1].angle_rad*180/M_PI, intensity_angle_corr_in[count_in-1].intensity_scale);

    // Re-arranging in data in to two separate arrays for filtering
    float* d_in = malloc(count_in * sizeof(float));
    float* s_in = malloc(count_in * sizeof(float));
    for(size_t ii = 0; ii < count_in; ii++){
        d_in[ii] = intensity_angle_corr_in[ii].angle_rad;
        s_in[ii] = intensity_angle_corr_in[ii].intensity_scale;
    }

    // Creating a uniformly sampled output arrays
    //size_t count_out = intensity_angle_corr_in[count_in-1].angle_rad / d_angle_rad;
    float* d_out = malloc(count_out * sizeof(float));
    float* s_out = malloc(count_out * sizeof(float));
    for(size_t ii = 0; ii < count_out; ii++){
        d_out[ii] = (float)ii * d_angle_rad;
    }
    //Run 1st order Savitzky–Golay like filter, with non-uniform input, and resampled output
    non_uniform_1order_savgol(d_in, s_in, count_in, d_out, s_out, count_out, angle_rad_avg);


    // Write back to output data struct
    for(size_t ii = 0; ii < count_out; ii++){
        intensity_angle_corr_out[ii].angle_rad = d_out[ii];
        intensity_angle_corr_out[ii].intensity_scale = s_out[ii];
    }
    //Extrapolate with closest value
    for(size_t ix=0; ix < count_out;ix++){
        if( intensity_angle_corr_out[ix].angle_rad <= intensity_angle_corr_in[0].angle_rad ){
            intensity_angle_corr_out[ix].intensity_scale = intensity_angle_corr_in[0].intensity_scale;
        }
        if( intensity_angle_corr_out[ix].angle_rad >= intensity_angle_corr_in[count_in-1].angle_rad ){
            intensity_angle_corr_out[ix].intensity_scale = intensity_angle_corr_in[count_in-1].intensity_scale;
        }
    }
	// ASCII plot filtered intensity_scale table
	
	for(size_t ix=0; ix < count_out;ix++){ 
		fprintf(stderr,"%3d %6.2f %7.2f",(int) ix,intensity_angle_corr_out[ix].angle_rad*180/M_PI, intensity_angle_corr_out[ix].intensity_scale);
		for (int ii=0;ii<(20*log10(intensity_angle_corr_out[ix].intensity_scale));ii++) fprintf(stderr,"*");
		fprintf(stderr,"\n");
	}

	fprintf(stderr,"%d Sound velocity / angle pairs read from file, resampled to %d:\n",(int) count_in, (int) count_out );
	fclose(fp);
    free(intensity_angle_corr_in);
    free(d_in);
    free(s_in);
    free(d_out);
    free(s_out);
	if (line) free(line);
	return (int)count_out;
}
