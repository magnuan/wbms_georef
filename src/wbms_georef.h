#ifndef __WBMS_GEOREF_H__
#define __WBMS_GEOREF_H__

#if defined(_MSC_VER)
#define NANF (nanf(""))
#define ATT_UNUSED
#else
#define NANF (0.f/0.f)
#define ATT_UNUSED __attribute__((unused))
#endif

#define NAVDATA_BUFFER_LEN (8*256)
#define POS_PREFETCH_SEC (8.0)
//Use input sound velocity profile to correct for ray bending 

#define MAX_DP 16*1024					//Maximum number of detection points in one dataset
#define MAX_LINELEN 128			//Maximum number of characters to represent one datapoint in output data CSV or BIN (11+1+11+1+8+1+3+1+8+1+8+2=56 +fudge = 128)


#define STRINGIFY0(v) #v
#define STRINGIFY(v) STRINGIFY0(v)

#define MAX_OUTPUT_FIELDS 128
typedef enum  {none=0, t,c, x,y,z,z_var, z_stddev,range, teta,steer,beam,el,val,val_dB,swath_y,aoi,ugate,lgate,cgate,gatew,quality,priority,strength,freq,bw,plen,voltage,multiping,multifreq,pingrate,pingnumber,pingdiff, LAT,LON,X,Y,Z,YAW,PITCH,ROLL, HOR_ACC,VERT_ACC,COURSE,SPEED,MOVEMENT,ALTITUDE,ALTIMETER,DEPTH,GPS_ACCURACY,GPS_STATUS,SATELLITES,line_of_sight,classification,iref, sref,snp_len,footprint_time,footprint,gain} output_format_e;
#define iterate_output_format(_F)  _F(x); _F(y); _F(z); _F(z_var); _F(z_stddev);   _F(teta);   _F(range); _F(steer);_F(beam); _F(el);  _F(val); _F(val_dB); _F(swath_y); _F(aoi); _F(ugate); _F(lgate); _F(cgate); _F(gatew);_F(quality);_F(priority);_F(strength);_F(freq);_F(bw);_F(plen);_F(voltage);_F(pingrate);_F(pingnumber);_F(pingdiff);_F(multiping);_F(multifreq);   _F(LAT); _F(LON); _F(X); _F(Y); _F(Z);   _F(t); _F(c);   _F(YAW); _F(PITCH); _F(ROLL); _F(HOR_ACC); _F(VERT_ACC); _F(COURSE); _F(SPEED);_F(MOVEMENT); _F(ALTITUDE); _F(ALTIMETER); _F(DEPTH); _F(GPS_ACCURACY); _F(GPS_STATUS); _F(SATELLITES); _F(line_of_sight); _F(classification); _F(iref); _F(sref); _F(snp_len); _F(footprint_time); _F(footprint); _F(gain)

#define PROJ_STR_MAGIC_NUMBER (0x207274736a6f7270)

#define MAX_BEAM_ANGLE_MODEL_ORDER 12
#define MAX_SENSOR_PACKET_SIZE (8*1024*1024)
#define MAX_NAVIGATION_PACKET_SIZE (8*1024*1024)

#define FORCE_MULTIDETECT_TO_QUALITY3 


#define COUNT_S7K_SNIPPET_SATURATION
#define COLLECT_S7K_STATS
#define COLLECT_POSMV_STATS

#define TX_POWER_REF 220


typedef enum {detection=0, upper_gate=1, lower_gate=2, center_gate=3} sonar_sample_mode_e;
typedef enum {ray_trace_none=0, ray_trace_fixed_depth_lut=1, ray_trace_fixed_depth_direct=2, ray_trace_var_depth=3} ray_tracing_mode_e;
typedef enum {s7k_backscatter_bathy=0, s7k_backscatter_snippets=1, s7k_backscatter_norm_snippets=2} s7k_backscatter_source_e;
typedef enum {snippet_mean_pow=0, snippet_sum_pow=1,snippet_detection_value=2, snippet_3dB_footprint_mean_pow=3,snippet_10dB_footprint_sum_pow=4, snippet_max=5} snippet_processing_mode_e;
typedef enum {ara_model_table=-1, ara_model_none=0, ara_model_cos_aoi=1, ara_model_sandy_gravel=2,ara_model_sandy_gravel_some_mud=3, ara_model_gravelly_muddy_sand=4, ara_model_muddy_sand=5, ara_model_gravelly_mud=6, ara_model_clay=7} ara_model_e;

//s7k_backscatter_bathy
//s7k_backscatter_snippets          From 7028 record
//s7k_backscatter_norm_snippets     From 7058 record

typedef struct{
	double ts;
	double lat,lon,alt;
	double x,y,z;   //x:easting, y:northing, z:down
	double roll,pitch,yaw,course;
	float heave;
    float depth;
    float altimeter;
    float speed;
}navdata_t;



typedef struct{
    float hor_accuracy,vert_accuracy;
	uint8_t mode ;
	uint8_t sv_n ;
	float dgps_latency ;
	uint16_t dgps_statid ;
	uint32_t gps_week;
	double gps_utc_diff; //Guessing on leap seconds (last adjust to 17 Dec 31th 2016)
	float gps_nav_latency ;
	float geoid_separation;
	uint8_t gps_type ;
	uint32_t gps_status ;
}aux_navdata_t;

#define NO_NAV_DATA         0
#define NAV_DATA_GEO        1
#define NAV_DATA_PROJECTED  2

typedef struct{
	float x;	//Forward [m]
	float y;	//Starbord [m]
	float z;	 //Down [m]
	float yaw;  //Right-hand pos around z (pos turning right) [rad]
	float pitch; //Right-hand pos around y (pos pitching up) [rad]
	float roll; //Right-hand positive arount x (pos rolling clockwise) [rad]
	float r_err; //Range error adjustment
    double time_offset; //Time offset in s (time ADDED to sensor time when aligning wiht navigation data)
}offset_t;


typedef struct{
    uint8_t min_quality_flag;
    uint8_t max_quality_flag;
    uint8_t min_priority_flag;
    uint8_t max_priority_flag;
    int8_t multifreq_index;
    uint32_t data_skip ;     //Number of datasets to skip 
    uint32_t data_length ;   //Number of datasets to process (0 for all)
    float min_depth;
    float max_depth;
    float min_range;
    float max_range;
    float min_azimuth;
    float max_azimuth;
    float min_elevation;
    float max_elevation;
    uint32_t min_ping_number ;
    uint32_t max_ping_number ;
    float swath_min_y ;
    float swath_max_y ;

    float max_abs_roll;
    float max_abs_pitch;
    float max_abs_yaw;
    float max_abs_droll_dt;
    float max_abs_dpitch_dt;
    float max_abs_dyaw_dt;

    float sv_offset;
    float force_sv;
    float mounting_depth;
    char intensity_correction; 
    float intensity_range_attenuation; 
    ara_model_e ara_model;
    float ara_model_multiplier;

    
    float rx_nadir_beamwidth;
    float tx_nadir_beamwidth;

    ray_tracing_mode_e ray_tracing_mode;
    uint8_t calc_aoi;
    sonar_sample_mode_e sonar_sample_mode; 
    uint32_t beam_decimate;
    uint32_t ping_decimate;
    uint8_t sbp_motion_stab;
    uint8_t sbp_raw_data;
    float sbp_bp_filter_start_freq;
    float sbp_bp_filter_stop_freq;
    float scale_tx_angle;
    uint32_t beam_corr_poly_order;
    float beam_corr_poly[MAX_BEAM_ANGLE_MODEL_ORDER];
    s7k_backscatter_source_e s7k_backscatter_source; //0=use from  
    uint8_t keep_s7k_tvg;
    uint8_t keep_s7k_footprint_comp;
    snippet_processing_mode_e snippet_processing_mode;
    float  tx_sensitivity;                          //In Pa @1m / V
    float  rx_sensitivity;                          //In V / Pa
    float  rx_processing_gain;                      //In LSB / V
}sensor_params_t;


typedef struct{
    double  x[MAX_DP];
	double  y[MAX_DP];
	double  z[MAX_DP];
    float   z_var[MAX_DP];
	float   i[MAX_DP];
	float   range[MAX_DP];
	float   teta[MAX_DP];
	float   steer[MAX_DP];
	int     beam[MAX_DP];
	float   swath_y[MAX_DP];
	float   aoi[MAX_DP];
	float   up_gate[MAX_DP];
	float   low_gate[MAX_DP];
    float   quality[MAX_DP];
    int     quality_flags[MAX_DP];
    float   priority[MAX_DP];
    float   strength[MAX_DP];
    int     classification[MAX_DP];
    int     snp_len[MAX_DP];
    float   footprint_time[MAX_DP];
    float   footprint[MAX_DP];
    float   tx_angle;
    float   sv;
    float   tx_freq;
    float   tx_bw;
    float   tx_plen;
    float   tx_voltage;
    int     ping_number;
    int     multiping_index;
    int     multifreq_index;
    float   sample_rate;
    float   ping_rate;
    float   intensity_noise_ref;
    float   strength_noise_ref;
    int     N;
    float   gain;
}output_data_t;

typedef struct{
    uint32_t type;
    uint32_t count;
}record_count_t;

typedef struct{
    float freq;
    float bw;
}sensor_count_stats_t;
#endif
