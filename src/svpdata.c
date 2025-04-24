#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include "cmath.h"
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif
#include "svpdata.h"


// Sometimes the SV cast might not contain data deep enough to cover the entire depth of the data.
// The workaround is to just extrapolate the sv profile to the maximum depth of the data 
// Because we do not know the maximum data depth when the sv data is read in, we do not know exactly how deep we need to extrapolate.
// Too short, and we might have to discard data due to lacking sv profile, too long means unneccessary run-time / memory use.
// As a initial compromise we set it to 2, assuming the SV cast is at least half the maximum data depth
#define EXTRAPOLATE_SV (2.0)
#define PYTHON_PRINTOUT



static int comp_sv_meas_on_depth_func(const void *a, const void *b){
	sv_meas_t *x = (sv_meas_t *)a;
	sv_meas_t *y = (sv_meas_t *)b;
	return ((x->depth - y->depth)>0);
}

uint8_t svp_auto_swap_sv_depth(sv_meas_t* sv_meas_in, size_t count_in){
    //Swap sv and depth if this seems more likely
    int sv_score = 0;
    int depth_score = 0;
    //Count which column has most values within sane sv range
    for (size_t ix=0;ix<count_in;ix++){
        if ((sv_meas_in[ix].sv>1400.f) && (sv_meas_in[ix].sv<1600.f)) {sv_score+=1;} 
        if ((sv_meas_in[ix].depth>1400.f) && (sv_meas_in[ix].depth<1600.f)) {depth_score+=1;} 
    }
    uint8_t swap = (depth_score>sv_score);
    if (swap){
        fprintf(stderr, "Assuming SV file order: depth,sv\n");
        for (size_t ix=0;ix<count_in;ix++){
            float tmp = sv_meas_in[ix].sv;
            sv_meas_in[ix].sv = sv_meas_in[ix].depth;
            sv_meas_in[ix].depth = tmp;
        }
    }
    else{
        fprintf(stderr, "Assuming SV file order: sv,depth\n");
    }
    return swap;
}

size_t  svp_extrapolate_sv_table(sv_meas_t* sv_meas_in, size_t count_in, const size_t max_count){
    
    float min_depth = 1e9;
    float max_depth = -1e9;
    for (size_t ix=0;ix<count_in;ix++){
        min_depth=MIN(min_depth,sv_meas_in[ix].depth);
        max_depth=MAX(max_depth,sv_meas_in[ix].depth);
    }
    fprintf(stderr,"SV profile %ld values from %f to %f m depth\n",count_in,min_depth,max_depth);
        
    float max_depth_sv = 0;
    for (size_t ix=0;ix<count_in;ix++){
        if (sv_meas_in[ix].depth == max_depth){
            max_depth_sv = sv_meas_in[ix].sv;
        }
    }
    size_t ix = count_in;
    float d = max_depth*1.1;
    for (; (d<max_depth*EXTRAPOLATE_SV) && (ix<max_count); d*=1.1){
         sv_meas_in[ix].depth = d;
         sv_meas_in[ix].sv = max_depth_sv + (0.017*(d-max_depth)); // Assume that SV only increases with pressure (0.17m/s per Bar)
         ix++;
    }
    count_in = ix;
    fprintf(stderr,"Extrapolated SV profile %ld values, extrapolate %f to %f m depth to %fm/s\n",count_in,max_depth,d,max_depth_sv);
    max_depth = d;
    
    return count_in;
}


int svp_read_from_file(char* fname, sv_meas_t* sv_meas, const size_t max_count){
	FILE * fp = fopen(fname,"r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	char * c;
	size_t count_in = 0;
	float sv, depth;


	if (fp==NULL){
		fprintf(stderr,"Could not open file %s for reading sound velocity\n",fname);
		return -1;
	}

	//Read in all SV measurement entries
	fprintf(stderr,"Reading in SV measurements from file %s\n",fname);
	while ((read = getline(&line, &len, fp)) != -1) {
		if (read>0){
			c = line;
			while (*c==' ' || *c=='\t' || *c=='\n' || *c=='\r') c++; 	//Skip leading white spaces
			if(*c==0) continue;											//End of line
			if(*c=='#') continue;										//Comment
            if (sscanf(c,"%f,%f",&sv, &depth)==2){
                sv_meas[count_in].sv = sv;
                sv_meas[count_in].depth = depth;
                count_in++;
            }
            if (sscanf(c,"%f\t%f",&sv, &depth)==2){
                sv_meas[count_in].sv = sv;
                sv_meas[count_in].depth = depth;
                count_in++;
            }
            if (sscanf(c,"%f %f",&sv, &depth)==2){
                sv_meas[count_in].sv = sv;
                sv_meas[count_in].depth = depth;
                count_in++;
            }
		}
        if( count_in >= max_count ) break;
	}
	if (line) free(line);
	fclose(fp);

    return count_in;
}
    

size_t svp_filter_and_resample(sv_meas_t* sv_meas_in, size_t count_in, sv_meas_t** sv_meas_out){

    // Resample SV data with this depth resolution (in meters)
    const float d_depth = 1.;
    // When resample use  a filter of this length (in meters) 
    const float depth_avg = 3.;
	
    fprintf(stderr,"# Sort measurements on depth\n");
	qsort(sv_meas_in,count_in,sizeof(sv_meas_t),comp_sv_meas_on_depth_func);
    fprintf(stderr,"# Count in = %d, max_depth = %0.2fm  sv=%7.2fm/s\n", (int) count_in, sv_meas_in[count_in-1].depth, sv_meas_in[count_in-1].sv);

    // Re-arranging in data in to two separate arrays for filtering
    float* d_in = malloc(count_in * sizeof(float));
    float* s_in = malloc(count_in * sizeof(float));
    for(size_t ii = 0; ii < count_in; ii++){
        d_in[ii] = sv_meas_in[ii].depth;
        s_in[ii] = sv_meas_in[ii].sv;
    }
    float* s_tmp = malloc(count_in * sizeof(float));
    //Run median filter on sv_data
    med_filter(s_in, s_tmp, 11, count_in);
   
    // Creating a uniformly sampled output arrays
    size_t count_out = (size_t) (sv_meas_in[count_in-1].depth / d_depth);
    float* d_out = malloc(count_out * sizeof(float));
    float* s_out = malloc(count_out * sizeof(float));
    for(size_t ii = 0; ii < count_out; ii++){
        d_out[ii] = (float)ii * d_depth;
    }
    
    //Run 1st order Savitzky–Golay like filter, with non-uniform input, and resampled output
    non_uniform_1order_savgol(d_in, s_tmp, count_in, d_out, s_out, count_out, depth_avg);

	sv_meas_t* sv_meas_resamp = malloc(count_out*sizeof(sv_meas_t));
    
    // Write back to output data struct
    for(size_t ii = 0; ii < count_out; ii++){
        sv_meas_resamp[ii].depth = d_out[ii];
        sv_meas_resamp[ii].sv = s_out[ii];
    }
	// ASCII plot filtered sv table
	
    /*
    fprintf(stderr,"d_out = np.array([");
    for(size_t ix=0; ix < count_in;ix++) 
		fprintf(stderr,"[%0.2f, %0.2f],",d_in[ix], s_in[ix]);
    fprintf(stderr,"])\n");
    */
    
	for(size_t ix=0; ix < count_out;ix++){ 
		fprintf(stderr,"%3d %6.2f %7.2f",(int) ix,sv_meas_resamp[ix].depth, sv_meas_resamp[ix].sv);
		for (int ii=0;ii<(sv_meas_resamp[ix].sv-1400);ii++) fprintf(stderr,"*");
		fprintf(stderr,"\n");
	}
    

	fprintf(stderr,"%d Sound velocity / depth pairs read from file, resampled to %d:\n",(int) count_in, (int) count_out );
    free(d_in);
    free(s_in);
    free(s_tmp);
    free(d_out);
    free(s_out);
    *sv_meas_out = sv_meas_resamp;
	return (int) count_out;
}
