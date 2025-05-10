#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "cmath.h"
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif
#include "time_functions.h"
#include "svpdata.h"


#define MIN_SANE_SV 1350.f
#define MAX_SANE_SV 1650.f
#define MIN_SANE_DEPTH 0.f
#define MAX_SANE_DEPTH 1000.f

char *svp_mode_short_names[] = {
	"-",
	"caris v2",
	"ascii tuple",
    "swift_svp",
    "unknown"
};


static int comp_sv_meas_on_depth_func(const void *a, const void *b){
	sv_meas_t *x = (sv_meas_t *)a;
	sv_meas_t *y = (sv_meas_t *)b;
	return ((x->depth - y->depth)>0);
}

size_t svp_discard_insane(sv_meas_t* sv_meas_in, size_t count_in){
    size_t ix_out = 0;
    for (size_t ix_in=0;ix_in<count_in;ix_in++){
        uint8_t sane = ((sv_meas_in[ix_in].sv>MIN_SANE_SV) && (sv_meas_in[ix_in].sv<MAX_SANE_SV));
        sane        &= ((sv_meas_in[ix_in].depth>MIN_SANE_DEPTH) && (sv_meas_in[ix_in].depth<MAX_SANE_DEPTH));
        if (sane){
            sv_meas_in[ix_out].sv = sv_meas_in[ix_in].sv;
            sv_meas_in[ix_out].depth = sv_meas_in[ix_in].depth;
            ix_out++;
        }
    }
    return ix_out;
}




uint8_t svp_auto_swap_sv_depth(sv_meas_t* sv_meas_in, size_t count_in){
    //Swap sv and depth if this seems more likely
    int sv_score = 0;
    int depth_score = 0;
    //Count which column has most values within sane sv range
    for (size_t ix=0;ix<count_in;ix++){
        if ((sv_meas_in[ix].sv>MIN_SANE_SV) && (sv_meas_in[ix].sv<MAX_SANE_SV)) {sv_score+=1;} 
        if ((sv_meas_in[ix].depth>MIN_SANE_SV) && (sv_meas_in[ix].depth<MAX_SANE_SV)) {depth_score+=1;} 
    }
    uint8_t swap = (depth_score>sv_score);
    if (swap){
        fprintf(stderr, "Assuming SV file order: sv,depth\n");
        for (size_t ix=0;ix<count_in;ix++){
            float tmp = sv_meas_in[ix].sv;
            sv_meas_in[ix].sv = sv_meas_in[ix].depth;
            sv_meas_in[ix].depth = tmp;
        }
    }
    else{
        fprintf(stderr, "Assuming SV file order: depth,sv\n");
    }
    return swap;
}

size_t  svp_extrapolate_sv_table(sv_meas_t* sv_meas_in, float extrapolate, size_t count_in, const size_t max_count){
    
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
    for (; (d<max_depth*extrapolate) && (ix<max_count); d*=1.1){
         sv_meas_in[ix].depth = d;
         sv_meas_in[ix].sv = max_depth_sv + (0.017*(d-max_depth)); // Assume that SV only increases with pressure (0.17m/s per Bar)
         ix++;
    }
    count_in = ix;
    fprintf(stderr,"Extrapolated SV profile %ld values, extrapolate %f to %f m depth to %fm/s\n",count_in,max_depth,d,max_depth_sv);
    max_depth = d;
    
    return count_in;
}

svp_mode_e svp_test_file(char* fname, double* ts, size_t* cnt){
	FILE * fp = fopen(fname,"r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	char * c;
	size_t data_count = 0;
	float x,y,z;
    uint8_t caris_svp_v2 = 0;
    uint8_t swift_svp = 0;

    //Start time
    int year,month,day,doy,HH,MM,SS;
    float fSS;
    //End time
    int year1,month1,day1,HH1,MM1;
    float fSS1;


    int bogus_count = 0;
    
    const int max_bogus = 10;

	if (fp==NULL){
		return svp_mode_none;
	}

	while ((read = getline(&line, &len, fp)) != -1) {
		if (read>0){
			c = line;
			while (*c==' ' || *c=='\t' || *c=='\n' || *c=='\r') c++; 	//Skip leading white spaces
			if(*c==0) continue;											//End of line
			if(*c=='#') continue;										//Comment
           
            if (caris_svp_v2==1){ //Try to read out Caris SVP Section header
                if (sscanf(c,"Section, %d-%d, %d:%d:%d",&year,&doy,&HH,&MM,&SS)==5){
                    caris_svp_v2=2;
                    continue;
                }
                if (sscanf(c,"Section %d-%d %d:%d:%d",&year,&doy,&HH,&MM,&SS)==5){
                    caris_svp_v2=2;
                    continue;
                }
            }

            if ( strncmp(c,"[SVP_VERSION_2]",14)==0) {caris_svp_v2 = 1;continue;};
            if ( strncmp(c,"SWIFT SVP",9)==0) {swift_svp = 1;continue;};
            if (swift_svp){
                if (data_count==0){
                    if (sscanf(c,"%d/%d/%d %d:%d:%f ,%f,%f,%f",&year,&month,&day,&HH,&MM,&fSS, &x, &y, &z)==9) {data_count++;continue;};
                }
                else{
                    if (sscanf(c,"%d/%d/%d %d:%d:%f ,%f,%f,%f",&year1,&month1,&day1,&HH1,&MM1,&fSS1, &x, &y, &z)==9) {data_count++;continue;};
                }
            }
            else{
                if (sscanf(c,"%f,%f",&x, &y)==2) {data_count++;continue;};
                if (sscanf(c,"%f\t%f",&x, &y)==2) {data_count++;continue;};
                if (sscanf(c,"%f %f",&x, &y)==2) {data_count++;continue;};
            }
            bogus_count ++;
		}
        if (bogus_count > max_bogus) break;
	}
    *cnt = data_count;

	if (line) free(line);
	fclose(fp);

    if (bogus_count > max_bogus){
        fprintf(stderr, "SVP test file: Bogus file\n");
        return svp_mode_none;
    }

    *ts = 0;
    if (data_count ==0){
        //fprintf(stderr, "SVP test file: Unknown file\n");
        return svp_mode_none;
    }
    if (swift_svp){
        *ts = irigb_to_gm(year,date_to_doy(year,month,day),HH,MM,fSS);
        //fprintf(stderr, "SVP test file: Swift SVP: ts=%f %d-%02d-%02d, %02d:%02d:%05.2f\n", *ts,year,month,day,HH,MM,fSS);
        return svp_mode_swift_svp;
    }
    if (caris_svp_v2==2){
        *ts = irigb_to_gm(year,doy,HH,MM,SS);
        //fprintf(stderr, "SVP test file: Carris SVP version 2: ts=%f year=%d, doy=%d, %02d:%02d:%02d\n", *ts,year,doy,HH,MM,SS);
        return svp_mode_caris_v2;
    }
    
    //fprintf(stderr, "SVP test file: ASCII tuple file\n");
    return svp_mode_ascii_tuple;
    
}



int svp_read_from_file(char* fname, sv_meas_t* sv_meas, const size_t max_count){
    double ts;
    size_t cnt;
    switch(svp_test_file(fname,&ts,&cnt)){
        case svp_mode_caris_v2: 
        case svp_mode_ascii_tuple:
            return svp_read_from_ascii_tuplet_file(fname, sv_meas, max_count);
        case svp_mode_swift_svp: 
            return svp_read_from_swift_svp_file(fname, sv_meas, max_count);
        case svp_mode_none:
            return 0;
    }
    return 0;
}

int svp_read_from_ascii_tuplet_file(char* fname, sv_meas_t* sv_meas, const size_t max_count){
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
            if (sscanf(c,"%f,%f",&depth, &sv)==2){
                sv_meas[count_in].sv = sv;
                sv_meas[count_in].depth = depth;
                count_in++;
            }
            else if (sscanf(c,"%f\t%f",&depth, &sv)==2){
                sv_meas[count_in].sv = sv;
                sv_meas[count_in].depth = depth;
                count_in++;
            }
            else if (sscanf(c,"%f %f",&depth, &sv)==2){
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

int svp_read_from_swift_svp_file(char* fname, sv_meas_t* sv_meas, const size_t max_count){
	FILE * fp = fopen(fname,"r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	char * c;
	size_t count_in = 0;
	float sv, depth,temp;
    int year,month,day,HH,MM;
    float fSS;


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
            if (sscanf(c,"%d/%d/%d %d:%d:%f ,%f,%f,%f",&year,&month,&day,&HH,&MM,&fSS, &depth, &temp, &sv)==9) {
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
