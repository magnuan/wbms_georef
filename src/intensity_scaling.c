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

//Simple typical rx sensitivity curve for 400kHz, taken from http://magtank.norbit.no/database/data/24115/2334507
inline float rx_sensitivity_model_dB(float beam_angle){
    return 2.98*powf(beam_angle,2) - 2.52*powf(beam_angle,4) + 3.69*powf(beam_angle,6) - 1.32*powf(beam_angle,8);
}


/* Calculate the beam time response in seconds */
float calc_beam_time_response(float range,float aoi, float beam_angle, float plen, sensor_params_t* sensor_params){
    beam_angle = ABS(beam_angle);
    beam_angle = MIN(beam_angle, 80*M_PI/180);
    const float c = 1500; //We jsut assume 1500m/s SV here, it is not a very precise value anyways in this case
    float beamwidth = sensor_params->rx_nadir_beamwidth / cosf(beam_angle);
    float T1 = plen;                            // Pulse length limited 
    float T2 = (2/c)*range*beamwidth*tanf(ABS(aoi)); //Beam width limited
    float T = MAX(T1,T2);
    return T;
}


/* Calculate the beam footprint in square meters*/
float calc_beam_footprint(float range,float aoi, float beam_angle, float plen, sensor_params_t* sensor_params){
    beam_angle = ABS(beam_angle);
    beam_angle = MIN(beam_angle, 80*M_PI/180);
    const float c = 1500; //We jsut assume 1500m/s SV here, it is not a very precise value anyways in this case
    float beamwidth = sensor_params->rx_nadir_beamwidth / cosf(beam_angle);
    float Ax1 = c*plen/2 * 1./(sinf(ABS(aoi)+1e-2f));   // Pulse length limited 
    float Ax2 = range * beamwidth;                      //Beamwidth limited
    #if 0
    //Simple minimum of bandwidth and beamwidth limited footprint
    float Ax = MIN(Ax1, Ax2);
    #else
    //Smoother blending of bandwidth / beamwidth limited footprint  
    const float k = 3;      //Higher number gives less smooth transition >8 practically the same as MIN()-funtion
    float Ax = powf(powf(Ax1,-k) + powf(Ax2,-k),-1/k); 
    #endif
    
    float Ay = range * sensor_params->tx_nadir_beamwidth;
    return Ax*Ay;
}

float calc_intensity_scaling(float range, float aoi, float beam_angle, float eff_plen, sensor_params_t* sensor_params, /*OUTPUT*/ float* footprint){
    float gain = 1.0f;

    if (sensor_params->intensity_correction){
        //Compensate for rx sensitivity  (This is specific for the 400kHz norbit, so we leave it out for now)
        //  float rx_sens_dB = rx_sensitivity_model_dB(beam_angle);
        //  gain *= powf(10.f,-rx_sens_dB/20);
        //Compensate for spreading loss: Add 40dB/log10(r)
        gain *= range*range;              //Comp two-way spreading loss, 40dB/log10(r)     
                                          // Compensate for attenuation
        float damping_dB = sensor_params->intensity_range_attenuation * (2*range/1000); 
        gain *= powf(10.f,damping_dB/20);
        //Compensate for range and angle dependent footprint
        *footprint = calc_beam_footprint(range,aoi,beam_angle,eff_plen,sensor_params);
        gain /= sqrtf(*footprint); //Subtract 10log10(A) Retured power scales with the footprint area, so divide the signal amplitude by the sqrt of the area
    }


    switch(sensor_params->ara_model){
        default:
        case ara_model_none:
            break;
        case ara_model_table:
            {
                int ix = ABS(aoi)/INTENSITY_ANGLE_STEP;
                ix = LIMIT(ix,0,INTENSITY_ANGLE_MAX_VALUES-1);
                gain *= intenity_angle_corr_table[ix].intensity_scale;
            }
            break;
        case ara_model_cos_aoi:
            //Compensate for source incidence
            gain /= sqrtf(cosf(aoi));        //Subtract 10*log10(cos(aoi)) 
            break;

        case ara_model_sandy_gravel:
            {
                //10*np.log10(np.cos(teta)) + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  1.5*np.exp(-(teta**2)/(np.deg2rad(30)**2)) -15
                float reflectivity_model_dB =    10*log10f(cosf(aoi)) \
                                                + 20*log10f(cosf((M_PI/2)*(powf(aoi/(M_PI/2),8)))) \
                                                + 1.5*(expf(-powf(aoi,2) / powf(30*M_PI/180,2) )) \
                                                - 15; 
                gain *= powf(10.f, -reflectivity_model_dB/20);
            }
            break;
        
        case ara_model_gravelly_muddy_sand:
            {
                //10*np.log10(np.cos(teta))  + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  8.5*np.exp(-(teta**2)/(np.deg2rad(20)**2)) -18
                float reflectivity_model_dB =    10*log10f(cosf(aoi)) \
                                                + 20*log10f(cosf((M_PI/2)*(powf(aoi/(M_PI/2),8)))) \
                                                + 8.5*(expf(-powf(aoi,2) / powf(20*M_PI/180,2) )) \
                                                - 18; 
                gain *= powf(10.f, -reflectivity_model_dB/20);
            }
            break;

        case ara_model_muddy_sand:
            {
                //10*np.log10(np.cos(teta)) + 30*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**6))) +  12.5*np.exp(-(teta**2)/(np.deg2rad(13)**2)) -22 
                float reflectivity_model_dB =    10*log10f(cosf(aoi)) \
                                                + 30*log10f(cosf((M_PI/2)*(powf(aoi/(M_PI/2),6)))) \
                                                + 12.5*(expf(-powf(aoi,2) / powf(13*M_PI/180,2) )) \
                                                - 22; 
                gain *= powf(10.f, -reflectivity_model_dB/20);
            }
            break;

        case ara_model_gravelly_mud:
            {
                //10*np.log10(np.cos(teta)) + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  16.*np.exp(-(teta**2)/(np.deg2rad(5)**2)) -23 
                float reflectivity_model_dB =    10*log10f(cosf(aoi)) \
                                                + 20*log10f(cosf((M_PI/2)*(powf(aoi/(M_PI/2),8)))) \
                                                + 16*(expf(-powf(aoi,2) / powf(5*M_PI/180,2) )) \
                                                - 23; 
                gain *= powf(10.f, -reflectivity_model_dB/20);
            }
            break;

        case ara_model_clay:
            {
                //10*np.log10(np.cos(teta))  + 20*np.log10(np.cos(np.pi/2*((teta/np.pi*2)**8))) +  15.5*np.exp(-(teta**2)/(np.deg2rad(5)**2)) -26 
                float reflectivity_model_dB =    10*log10f(cosf(aoi)) \
                                                + 20*log10f(cosf((M_PI/2)*(powf(aoi/(M_PI/2),8)))) \
                                                + 15.5*(expf(-powf(aoi,2) / powf(5*M_PI/180,2) )) \
                                                - 26; 
                gain *= powf(10.f, -reflectivity_model_dB/20);
            }
            break;
    }

    return gain;
}
