#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#ifdef __unix__
#include <unistd.h>
#endif
#include <time.h>

#include <string.h>
#include <stdint.h>
#include <math.h>
#include "cmath.h"
#include "gsf_wrapper.h"
#include "georef_tools.h"
#include "intensity_scaling.h"
#include "gsf.h"

#define ROLL_VECTOR_LEN 512
#define ROLL_VECTOR_RATE 500.

static const char* sensor_fname = NULL;
static const char* nav_fname = NULL;

static int sensor_handle;
static int nav_handle;

static int sensor_fd;
static int nav_fd;

gsfDataID *nav_dataID;
gsfRecords *nav_records;
gsfDataID *sensor_dataID;
gsfRecords *sensor_records;

extern int gsfError;

static uint32_t gsf_nav_stat_packet_count[NUM_REC_TYPES];
static uint32_t gsf_sensor_stat_packet_count[NUM_REC_TYPES];

static uint8_t verbose = 0;
static offset_t* sensor_offset;

void gsf_init(void){
    nav_dataID = malloc(sizeof(gsfDataID));
    nav_records = malloc(sizeof(gsfRecords));
    sensor_dataID = malloc(sizeof(gsfDataID));
    sensor_records = malloc(sizeof(gsfRecords));

    for(int ix=0; ix<NUM_REC_TYPES; ix++){
        gsf_nav_stat_packet_count[ix]=0;
        gsf_sensor_stat_packet_count[ix]=0;
    }
	return;
}

void gsf_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}

void gsf_print_stats(void){
    uint32_t total_data = 0;
    for (uint32_t id=0;id<NUM_REC_TYPES;id++){
        total_data+=gsf_nav_stat_packet_count[id];
    }
    if (total_data){
        fprintf(stderr,"------- GSF data packet count -----\n");
    }
    for (uint32_t id=0;id<NUM_REC_TYPES;id++){
        if(gsf_nav_stat_packet_count[id]>0){
            fprintf(stderr, "GSF record type %5d: %8d\n",id,gsf_nav_stat_packet_count[id]);
        }
    }
    if (total_data){
        fprintf(stderr,"\n");
    }
}

uint32_t gsf_num_record_types(void){
    uint32_t ret = 0;
    for (uint32_t id=0;id<NUM_REC_TYPES;id++){
        ret+= (gsf_nav_stat_packet_count[id]>0);
    }
    return ret;
}

uint32_t gsf_get_record_count(record_count_t* records){
    size_t ix = 0;
    for (uint32_t id=0;id<NUM_REC_TYPES;id++){
        if(gsf_nav_stat_packet_count[id] > 0){
            (records+ix)->type = id;
            (records+ix)->count = gsf_nav_stat_packet_count[id];
            ix++;
        }
    }
    return (uint32_t) ix;
}


void gsf_set_sensor_filename(const char* fname){
    sensor_fname = fname;
}

void gsf_set_nav_filename(const char* fname){
    nav_fname = fname;
}


uint8_t gsf_test_nav_file(int fd){
    if (nav_fname==NULL) return 0;
    fprintf(stderr, "Testing file %s as GSF navigation file",nav_fname);
    int ret = gsfOpen(nav_fname, GSF_READONLY, &nav_handle);
    fprintf(stderr, " %s, ret=%d\n",(ret==0)?"SUCCESS":"FAIL", ret);
    if (ret) return 0;
    nav_fd = fd;
	return 1;
}

uint8_t gsf_test_bathy_file(int fd){
    if (sensor_fname==NULL) return 0;
    fprintf(stderr, "Testing file %s as GSF sensor file",sensor_fname);
    int ret = gsfOpen(sensor_fname, GSF_READONLY, &sensor_handle);
    fprintf(stderr, " %s, ret=%d\n",(ret==0)?"SUCCESS":"FAIL", ret);
    if (ret) return 0;
    sensor_fd = fd;
	return 1;
}

int gsf_fetch_next_packet(char * data, int fd){
    int ret;
    if (fd == nav_fd){
        ret =  gsfRead(nav_handle, GSF_NEXT_RECORD, nav_dataID, nav_records,NULL,0);
        int data_type = nav_dataID->recordID & 0x0FFF;
        gsf_nav_stat_packet_count[data_type]++;
        //fprintf(stderr, "gsf_fetch_next_nav_packet ret=%d, data_type=%d, registry=%d, cs_flag=%d, record_number=%d, err=%d\n",ret,data_type,(nav_dataID->recordID >>12) & 0x0FFF,nav_dataID->checksumFlag, nav_dataID->record_number, gsfError); 
        return ret>0?ret:0;
    }
    if (fd == sensor_fd){
        ret =  gsfRead(sensor_handle, GSF_NEXT_RECORD, sensor_dataID, sensor_records,NULL,0);
        int data_type = sensor_dataID->recordID & 0x0FFF;
        gsf_sensor_stat_packet_count[data_type]++;
        //fprintf(stderr, "gsf_fetch_next_sensor_packet ret=%d, data_type=%d, registry=%d, cs_flag=%d, record_number=%d, err=%d\n",ret,data_type,(sensor_dataID->recordID >>12) & 0x0FFF,sensor_dataID->checksumFlag, sensor_dataID->record_number, gsfError); 
        return ret>0?ret:0;
    }
    return 0;
}

int gsf_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out){
    int data_type = sensor_dataID->recordID & 0x0FFF;
    if  (data_type != GSF_RECORD_SWATH_BATHYMETRY_PING) return 0;  // So far GSF_RECORD_SWATH_BATHYMETRY_PING is the only record we know contains navigation data
    //int registry = (sensor_dataID->recordID >>12) & 0x0FFF;
    //fprintf(stderr, "gsf_identify_sensor_packet data_type=%d, registry=%d, cs_flag=%d, record_number=%d\n",data_type,registry,sensor_dataID->checksumFlag, sensor_dataID->record_number); 
	
    *ts_out = sensor_records->mb_ping.ping_time.tv_sec;
    *ts_out += (sensor_records->mb_ping.ping_time.tv_nsec)*1e-9;
    return data_type;
}

int gsf_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
    int data_type = nav_dataID->recordID & 0x0FFF;
    if      (data_type == GSF_RECORD_SWATH_BATHYMETRY_PING) {}  // So far GSF_RECORD_SWATH_BATHYMETRY_PING is the only record we know contains navigation data
    //else if (data_type == GSF_RECORD_ATTITUDE) {}             // TODO HOW TO DEAL WITH THE HIGH RATE ATITUDE DATA HERE
    else return NO_NAV_DATA; 
    

    *ts_out = nav_records->mb_ping.ping_time.tv_sec;
    *ts_out += (nav_records->mb_ping.ping_time.tv_nsec)*1e-9;
    navdata->ts = *ts_out;
    navdata->lon = nav_records->mb_ping.longitude*M_PI/180.;
    navdata->lat = nav_records->mb_ping.latitude*M_PI/180.;
    navdata->alt = nav_records->mb_ping.height;
    navdata->heave = nav_records->mb_ping.heave;
    navdata->roll = nav_records->mb_ping.roll*M_PI/180.;
    navdata->pitch = nav_records->mb_ping.pitch*M_PI/180.;
    navdata->yaw = nav_records->mb_ping.heading*M_PI/180.;
    aux_navdata->geoid_separation = nav_records->mb_ping.sep;
    //fprintf(stderr, "gsf_process_nav_packet ts=%lf  Lat=%f Lon=%f Height=%f Yaw=%f Pitch=%f Roll=%f Heave=%f\n",
    //        *ts_out,navdata->lat*180/M_PI, navdata->lon*180/M_PI, navdata->alt, navdata->yaw*180/M_PI, navdata->pitch*180/M_PI, navdata->roll*180/M_PI, navdata->heave );

    if (proj){
        double alt;
        switch (alt_mode){ 
            case 1:  alt = navdata->alt; break;     //GPS altitude from posmv is positive up (When having RTK, we typically want to use GPS altitude instread of heave+tide)
            case 2:  alt = navdata->heave; break;   //heave from s7k is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
            default: alt = 0;
        }
        double z;
        latlon_to_kart(navdata->lon, navdata->lat , alt, proj, /*output*/ &(navdata->x), &(navdata->y) , &z);
        if (alt_mode ==1){ 
            navdata->z = z; 
        }
	    else if (alt_mode ==2){ 
				navdata->z = -navdata->heave;  //heave from posmv is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
	    }
        navdata->z += z_offset;
    }

    
    return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
}

uint32_t gsf_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/ output_data_t* outbuf){
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
    float* quality = &(outbuf->quality[0]);
    int*   quality_flags = &(outbuf->quality_flags[0]);
    float* priority = &(outbuf->priority[0]);
    float* tx_angle_out = &(outbuf->tx_angle);
    float* fs_out = &(outbuf->sample_rate);
    float* ping_rate_out = &(outbuf->ping_rate);
    float* sv_out = &(outbuf->sv);
    float* intensity_noise_ref_out = &(outbuf->intensity_noise_ref);   
    float* strength_noise_ref_out = &(outbuf->strength_noise_ref);   
    float* tx_freq_out = &(outbuf->tx_freq);
    float* tx_bw_out = &(outbuf->tx_bw);
    float* tx_plen_out = &(outbuf->tx_plen);
    float* tx_voltage_out = &(outbuf->tx_voltage);
    int* ping_number_out = &(outbuf->ping_number);
    int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);
    int* classification_val = &(outbuf->classification[0]);
    float* footprint_area = &(outbuf->footprint[0]);  
    float * footprint_time = &(outbuf->footprint_time[0]);  

    
	float sensor_r;
	float sensor_az;
	float sensor_el;
	float sensor_t;
    float sensor_elec_steer;
	//float xs[MAX_DP], ys[MAX_DP], zs[MAX_DP];	/*Coordinate with respect to sensor (forward,starboard, down)*/																					 

	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    float tx_angle; 
	float Fs;
    float ping_rate;
	float c;
	uint16_t Nin;
    uint32_t ping_number;
    uint16_t multiping_index;
    uint16_t multifreq_index;
	uint16_t Nout;
	uint16_t ix_in,ix_out;
    const uint16_t ix_in_stride = MAX(1,sensor_params->beam_decimate); 
    const uint32_t ping_number_stride = sensor_params->ping_decimate; 
    float roll_vector[ROLL_VECTOR_LEN];
    float z_vector[ROLL_VECTOR_LEN];
	float sensor_az_tx2rx_corr;
	float sensor_z_tx2rx_corr;
    float tx_freq;
    float tx_bw;
    float tx_plen;
    float tx_voltage;

    int data_type = sensor_dataID->recordID & 0x0FFF;
    if      (data_type == GSF_RECORD_SWATH_BATHYMETRY_PING) {}  // So far GSF_RECORD_SWATH_BATHYMETRY_PING is the only record we process
    else return 0; 
   
    static double prev_ts = 0;
    double ts = sensor_records->mb_ping.ping_time.tv_sec;
    ts += (sensor_records->mb_ping.ping_time.tv_nsec)*1e-9;
    
    static uint32_t pingcounter=0;
    
    if (pingcounter++<10){
        fprintf(stderr,"Sonar type %s  id=%d \n", gsfGetSonarTextName(&(sensor_records->mb_ping)), sensor_records->mb_ping.sensor_id );

    }

    switch(sensor_records->mb_ping.sensor_id){
        case 156: // Kongsberg EM712
        default:
            //TX parameters, 
            //TODO fint these values in some meta data
            tx_freq = 400e3;
            tx_bw = 80e3; 
            tx_plen = 100e-6;
            tx_angle = 0 ; //TODO This needs to be resolved for STX systems, is it based on the nav_records->mb_ping.sector_number array
            tx_voltage = 0;
            break;
        
    }

    //TODO Check out this one:
    // sensor_records->mb_ping.incident_beam_adj vector
    
    Fs = 0; //TODO do we need this?


    c = sensor_records->svp.sound_speed[0];             //TODO Here we just assume that the first entry is the surface SV 
    //float c_depth = sensor_records->svp.depth[0];
    //fprintf(stderr, "Sound speed %f m/s  @ %f m\n",c, c_depth);

    float attenuation = calc_attenuation(tx_freq, sensor_params);
    outbuf->gain = 1.0; //TODO
       
    //For CW pulses, bandwidth is given by pulse length
    tx_bw = MAX(tx_bw,1/tx_plen);
    
    if (sensor_params->force_sv > 0){
        c = sensor_params->force_sv;
    }
    if (c != c){
        fprintf(stderr, "NaN sound velocity encountered in data");
    }
    Nin = sensor_records->mb_ping.number_beams;

    multifreq_index = 0;                    //TODO
    ping_number = pingcounter;              // This nowhere in data ?
    multiping_index =  0;                   //TODO
    ping_rate =  1./(ts-prev_ts);
    ping_rate = LIMIT(ping_rate,0.1,100);
    prev_ts = ts;

    *sv_out = c; 
    

    *multiping_index_out = multiping_index;
    *multifreq_index_out = multifreq_index;
    *tx_freq_out = tx_freq;
    *tx_bw_out = tx_bw;
    *tx_plen_out = tx_plen;
    *tx_voltage_out = tx_voltage;
    *ping_number_out = ping_number;
    *fs_out = Fs;
    *ping_rate_out = ping_rate;
    tx_angle *= sensor_params->scale_tx_angle;

    //Skip whole dataset condition
    sensor_el  = tx_angle;								 
    if (    ((sensor_params->multifreq_index>=0) && (sensor_params->multifreq_index!=multifreq_index)) ||
	        ((sensor_el < sensor_params->min_elevation) || (sensor_el > sensor_params->max_elevation)) ||
	        ((ping_number < sensor_params->min_ping_number) || (sensor_params->max_ping_number && (ping_number > sensor_params->max_ping_number))) ||
            ((ping_number%ping_number_stride) != 0)
        ){
        return(0);
    }


    
    //Find nav data for tx instant
    //Generate a vector with interpolated roll starting at tx time, with 1ms samplerate, for 1/ping_rate duration
    // This is tobe used to correct azimuth and heave data, as this is defined at rx (not tx) time
    //To calculate angle-of insidence, we must sort beams on angle (Strictly only neccessary for ISS data)
    //Sort bath->dp[n] based on bath->dp[n].angle for n in 0-Nin
    
    if (calc_interpolated_nav_data( posdata, pos_ix, ts+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
        if(verbose) fprintf(stderr, "Could not find navigation data for GSF record at time %f\n",ts+sensor_offset->time_offset);
        return 0;
    }
    calc_interpolated_roll_and_z_vector(posdata, pos_ix, ts+sensor_offset->time_offset, (1.f/ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
    
    //TODO add this if the data could be unsorted
    //if(sensor_params->calc_aoi){
    //    qsort(bath_vX->dp, Nin, sizeof(detectionpoint_vX_t), cmp_wbms_vX_dp_on_angle_func);
    //}

	if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }

    
    size_t Nn = (Nin/ix_in_stride)+1;
    float *xs = malloc(Nn*sizeof(float));
    float *ys = malloc(Nn*sizeof(float));
    float *zs = malloc(Nn*sizeof(float));


    uint8_t sensor_quality_flags;
    uint8_t priority_flags;
    uint16_t flags;

    //Calculate sounding positions in sonar reference frame at tx instant
	ix_out = 0;
    //printf("Nin=%d\n",Nin);

	for (ix_in=0;ix_in<Nin;ix_in+=ix_in_stride){
    
        sensor_t =  *(sensor_records->mb_ping.travel_time +ix_in);		//Calculate tx to rx time for each point 
        sensor_r   = (sensor_t*c)/2;	                            //Calculate range to each point en meters
        sensor_az  = *(sensor_records->mb_ping.beam_angle+ ix_in) * -M_PI/180.;
        sensor_elec_steer = sensor_az;
        if(sensor_records->mb_ping.quality_flags){
            sensor_quality_flags = *(sensor_records->mb_ping.quality_flags +ix_in);
        }
        else{
            sensor_quality_flags = 3;
        }
        flags = *(sensor_records->mb_ping.beam_flags + ix_in);

        sensor_r  += sensor_offset->r_err;


        // Apply correctiom from beam corection polynom if defined
        if (sensor_params->beam_corr_poly_order){
            sensor_az = apply_beam_correction_poly(sensor_az, sensor_params->beam_corr_poly, sensor_params->beam_corr_poly_order);
        }
    
            
        
        // Add correction for roll during tx2rx period for each beam individually
        sensor_az_tx2rx_corr = -roll_vector[(size_t) round(sensor_t*ROLL_VECTOR_RATE)]; //Roll is given in opposite angles than sonar azimuth
        sensor_z_tx2rx_corr = z_vector[(size_t) round((sensor_t/2)*ROLL_VECTOR_RATE)]; // Z correction is for half tx to rx time
		if (	(sensor_quality_flags >= sensor_params->min_quality_flag) && 
				(sensor_quality_flags <= sensor_params->max_quality_flag) &&
				(sensor_az > sensor_params->min_azimuth) && (sensor_az < sensor_params->max_azimuth) &&
				(sensor_r > sensor_params->min_range) && (sensor_r < sensor_params->max_range) 
			){
            beam_number[ix_out] = ix_in;
            beam_angle[ix_out] =  sensor_az;  //Store raw beam angle from sonar for data analysis
            beam_steer[ix_out] = sensor_elec_steer;
            beam_range[ix_out] = sensor_r;
            quality[ix_out] = (float) sensor_quality_flags;
            quality_flags[ix_out] = sensor_quality_flags;
            classification_val[ix_out] = (sensor_quality_flags==3);  //Just calssify as Seafloor (=1) if Q=3 and Noise(=0) otherwise
            priority[ix_out] = (float) priority_flags;
            *tx_angle_out = sensor_el;
			
            intensity[ix_out] = sensor_records->mb_ping.mr_amplitude[ix_in];
            intensity[ix_out] = sensor_records->mb_ping.sector_number[ix_in];

			#if 1       //TODO select cone-cone / cone-plane based on system
			xs[ix_out] = sensor_r * sin(sensor_el);
			ys[ix_out] = sensor_r * sin(sensor_az); //Sign flipped compared to standard right hand system
			zs[ix_out] = sensor_r * sqrt(1. - (sin((sensor_az+sensor_az_tx2rx_corr))*sin((sensor_az+sensor_az_tx2rx_corr))) - (sin(sensor_el)*sin(sensor_el)));
			#else
			// ***** Converting sensor data from spherical to kartesian coordinates ********
			// Projection assumes cone-plane coordinates (See assembla ticket #1725)								 
			// Standard sonar mounting az=0 el=0 => Nadir(0,0,1), az=90 el=0 => Starbord(0,1,0), az=0 el=90 => Forward(1,0,0) 
			// Sign of y axis is flipped (compared to #1725) because az rotation is left to right, and not right to left which it should be if it was a positive rotation around x-axis
			xs[ix_out] = sensor_r * sinf(sensor_el);
			ys[ix_out] = sensor_r * sinf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el); //Sign flipped compared to standard right hand system
			zs[ix_out] = sensor_r * cosf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el);
			#endif
            zs[ix_out] += sensor_z_tx2rx_corr;
			ix_out++;
			
		}
    }
    Nout = ix_out;

    georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
     
    //Calculate acrosstrack pos rel sonar, for analyzis 
    float sin_nav_yaw = sinf(nav_yaw);
    float cos_nav_yaw = cosf(nav_yaw);
	for (ix_out=0;ix_out<Nout;ix_out++){
        swath_y[ix_out] = -(float)(x[ix_out]-nav_x)*sin_nav_yaw + (float)((y[ix_out]-nav_y))*cos_nav_yaw;   //Offset of sounding accross-track   
    }
    // printf("Nout1 = %d\n",Nout);
	
	#if 1
	//Post GEO-REF filtering
	Nin = Nout;
	ix_out = 0;
	for (ix_in=0;ix_in<Nin;ix_in++){
		x[ix_out] = x[ix_in];
		y[ix_out] = y[ix_in];
		z[ix_out] = z[ix_in];
        z_var[ix_out] = z_var[ix_in];
		intensity[ix_out] = intensity[ix_in];
		beam_angle[ix_out] = beam_angle[ix_in];
		swath_y[ix_out] = swath_y[ix_in];
        beam_number[ix_out] = beam_number[ix_in];
        beam_steer[ix_out] = beam_steer[ix_in];
        beam_range[ix_out] = beam_range[ix_in];

		if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;
        if (swath_y[ix_in]>sensor_params->swath_max_y || swath_y[ix_in]<sensor_params->swath_min_y) continue;

		ix_out++;
	}
	Nout = ix_out;
	#endif 
    
    //Calculate AOI on  Post-filtered data
    if (sensor_params->calc_aoi){
        calc_aoi(beam_range, beam_angle, Nout, /*output*/ aoi);
    }
    else{
        for (ix_out=0;ix_out<Nout;ix_out++){
            aoi[ix_out] = beam_angle[ix_out];
        }
    }
    //Calculate corrected intensity and uncertainty model on  Post-filtered data
	// Populate r,az,el and t with data from bath data
    float eff_plen = MIN(tx_plen, 2./tx_bw);
	for (size_t ix=0;ix<Nout;ix++){
        //Compensate intensity for range and AOI
        intensity[ix]  *= calc_intensity_range_scaling(beam_range[ix],attenuation, sensor_params);
        intensity[ix]  *= calc_footprint_scaling(beam_range[ix], aoi[ix], beam_angle[ix],eff_plen , sensor_params, /*OUTPUT*/ &(footprint_area[ix]));
        intensity[ix]  *= calc_ara_scaling(aoi[ix_out], sensor_params);
        footprint_time[ix] = calc_beam_time_response(beam_range[ix], aoi[ix], beam_angle[ix],eff_plen , sensor_params);
	}
    variance_model(beam_range, beam_angle,aoi,Nout,nav_droll_dt,nav_dpitch_dt,/*output*/ z_var);

    //printf("Nout2 = %d\n",Nout);

    free(xs);free(ys);free(zs);
    outbuf->N = Nout;
	return Nout;
}

static sensor_count_stats_t gsf_count_stats;

sensor_count_stats_t* gsf_get_count_stats(void){
    return &gsf_count_stats;
}

uint32_t gsf_count_data( char* databuffer,uint32_t databuffer_len, double* ts){
	return 0;
}


const char * gsf_get_data_type(void){
	return "mbes";
}

