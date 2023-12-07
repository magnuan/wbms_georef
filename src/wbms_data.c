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
#include "loki_packet.h"
#include <math.h>
#include "time_functions.h"

#include "cmath.h"
#include "wbms_georef.h"
#include "wbms_data.h"
#include "georef_tools.h"
#include "intensity_scaling.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <corecrt_math_defines.h>
#endif


//#define FAKE_SBP_TIMESTAMP
#define IGNORE_WBMS_CRC_FOR_SBP_DATA

static uint8_t verbose = 1;

static offset_t* sensor_offset;

void wbms_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}

/* Read some data from file, and verify if it is a valid file type.
   return 1 if file is a valid wbms bathy file
   return 0 if file is not a valid wbms bathy file
*/
uint8_t wbms_test_file(int fd, int* version){
    uint8_t pass=0;
    //printf("MAX_WBMS_PACKET_SIZE=%d\n",MAX_WBMS_PACKET_SIZE);
    char* data = malloc(MAX_WBMS_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<10;test++){     //Test the first 10 packets, if none of them contains bathy data it is pobably not a valid data file
        int len; 
        len = wbms_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            int type = wbms_identify_packet(data, len, &ts, version);
            if (type==PACKET_TYPE_BATH_DATA){
                pass=1;
                break;
            }
            else if (type==PACKET_TYPE_SBP_DATA){
                pass=1;
                break;
            }
        }
        else{
            break;
        }
    }
    free(data);
    return pass;
}


//#define PRINT_DROPPED_DATA

int wbms_seek_next_header(int fd){
    //printf("wbms_seek_next_header\n");
	char state = 0;
	uint8_t v=0;
	int n;
	int dump= 0;
	//int position = lseek(fd, 0, SEEK_CUR);
	//fprintf(stderr,"\nseek headed start at 0x%08x\n",position);
    int read_bytes = 0;
    #ifdef PRINT_DROPPED_DATA
    char print_mode = 0;
    #endif
	while (read_bytes<(MAX_WBMS_PACKET_SIZE+4)){
		n = read(fd,&v,1);
        read_bytes++;
		if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
		if(n==0){ fprintf(stderr,"End of WBMS stream\n");return -1;}
		if(n>0){
			//fprintf(stderr,"%02x ",v);
			dump += 1;
            
            #ifdef PRINT_DROPPED_DATA
            // Debug: dump all data that is not wbms-data
            if ( ( (state==0) && ((v!=0xef) )) || ( (state==1) && ((v!=0xbe) )) || ( (state==2) && ((v!=0xad) )) || ( (state==3) && ((v!=0xde) ))){
			    if (v=='$'){
                    fprintf(stderr,"\n");
                    print_mode=1;
                }
			    if (v=='\r'){ 
                    fprintf(stderr,"\n");
                    print_mode=0;
                }
                if (print_mode==0) fprintf(stderr,"%02x ",v);
			    if (print_mode==1) fprintf(stderr,"%c",v);
        
            }
            #endif
			switch (state){
				case 0: state = (v==0xef)?1:0;break;
				case 1: state = (v==0xbe)?2:0;break;
				case 2: state = (v==0xad)?3:0;break;
				case 3: state = (v==0xde)?4:0;break;
			}
			if (state==4){
				dump-=4;
				if(dump>0)  {
					//fprintf(stderr,"WBMS seek dump %d bytes\n",dump);
					//sleep(1);
				}
                //printf("wbms_seek_next_header end 0\n");
				return 0;	
			}
		}
	}
    //printf("wbms_seek_next_header end -1\n");
	return -1;
}

int wbms_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;
	uint32_t size;
	//fprintf(stderr,"wbms_fetch_next_packet\n");
	if(wbms_seek_next_header(fd)) return 0;
	((uint32_t*)data)[0] = 0xDEADBEEF;
	//Fetch main header
	rem = 20; //Fetch packet header (minus the preamble we allready have)
	dp = &(data[4]);
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types 
	size = ((uint32_t*)data)[2];
	//fprintf(stderr,"Reading %d byte WBMS packet\n",size);
	//Fetch sub header and  payload
	rem = size - 24; 
	//while (rem>0){ n= read(fd,dp,rem);rem -= n; dp+=n;}
	while (rem>0){ n= read(fd,dp,rem);if (n<=0) return 0;rem -= n; dp+=n;} //Read neccessary data, abort if no data or failure occurs TODO: verify that this works with all stream types
	//fprintf(stderr,"%d bytes read from %d\n",size,fd);
	return size;
}

int wbms_identify_packet(char* databuffer, uint32_t len, double* ts_out, int* version){
    static int rcnt=0;
	packet_header_t* wbms_packet_header;
	bath_data_packet_t* wbms_bath_packet;
    sbp_data_packet_t* wbms_sbp_packet;
	char str_buf[256];
    #ifdef FAKE_SBP_TIMESTAMP
    static double fake_time = 0;
    #endif

	wbms_packet_header = (packet_header_t*) databuffer;

	if (wbms_packet_header->size != len){
		if(verbose) fprintf(stderr,"malformed wbms packet received, discarding\n");
		return 0;
	}
    #ifdef IGNORE_WBMS_CRC_FOR_SBP_DATA
    if (wbms_packet_header->type != PACKET_TYPE_SBP_DATA){
        uint32_t crc;
        crc = crc32_le(0,(uint8_t*)databuffer+24,len-24);
        if (wbms_packet_header->crc != crc){
            if(verbose) fprintf(stderr,"WBMS packet CRC error, discarding\n");
            return 0;
        }
    }
    #else
    uint32_t crc;
    crc = crc32_le(0,(uint8_t*)databuffer+24,len-24);
    if (wbms_packet_header->crc != crc){
        if(verbose) fprintf(stderr,"WBMS packet CRC error, discarding\n");
        return 0;
    }
    #endif
	//if (verbose) fprintf(stderr, "Received WBMS packet type %d size %d  CRC = 0x%08x\n",wbms_packet_header->type, wbms_packet_header->size, wbms_packet_header->crc);	
    rcnt++;
    
    //fprintf(stderr,"WBMS %d : type = %d ver=%d\n",rcnt,wbms_packet_header->type,wbms_packet_header->version);
	switch (wbms_packet_header->type){
		case PACKET_TYPE_BATH_DATA: 
			wbms_bath_packet = (bath_data_packet_t*) databuffer;
            if (version){
                *version = wbms_bath_packet->header.version;
            }
			if (0){//rcnt%100==0){
				sprintf_unix_time(str_buf, wbms_bath_packet->sub_header.time);
				fprintf(stderr,"WBMS bathy %d : ver=%d ping=%7d  multi ping=%2d/%2d tx=%5.1fdeg %s\n",
                        rcnt,
                        wbms_bath_packet->header.version,
                        wbms_bath_packet->sub_header.ping_number,
                        wbms_bath_packet->sub_header.multiping_seq_nr,
                        wbms_bath_packet->sub_header.multiping,
                        wbms_bath_packet->sub_header.tx_angle*180/M_PI, 
                        str_buf);
			}
			#define GUESS_NEXT_WBMS_TS
			#ifdef GUESS_NEXT_WBMS_TS
			*ts_out = wbms_bath_packet->sub_header.time+sensor_offset->time_offset + (1./(wbms_bath_packet->sub_header.ping_rate));
			#else
			*ts_out = wbms_bath_packet->sub_header.time+sensor_offset->time_offset;
			#endif
			return PACKET_TYPE_BATH_DATA;
		case PACKET_TYPE_WATERCOL_DATA: 
			if(verbose>2) fprintf(stderr,"Received WBMS watercol data, Discarding!\n");
			return 0;
		case PACKET_TYPE_SNIPPET_DATA: 
			if(verbose>2) fprintf(stderr,"Received WBMS snippet data, Discarding!\n");
			return 0;
		case PACKET_TYPE_SIDESCAN_DATA: 
			if(verbose>2) fprintf(stderr,"Received WBMS sidescan data, Discarding!\n");
			return 0;
		case PACKET_TYPE_GEOREF_SIDESCAN_DATA: 
			if(verbose>2) fprintf(stderr,"Received WBMS georef sidescan data, Discarding!\n");
			return 0;
		case PACKET_TYPE_SBP_DATA: 
			wbms_sbp_packet = (sbp_data_packet_t*) databuffer;
            if (version){
                *version = wbms_sbp_packet->header.version;
            }
            
            //TODO remove this hack. ping_rate is not reported correctly, so to avoid div by zero error we set it fixed to 5Hz
            wbms_sbp_packet->sub_header.ping_rate = 5.; 

            #ifdef FAKE_SBP_TIMESTAMP
            wbms_sbp_packet->sub_header.time = fake_time;
            fake_time += 1./(wbms_sbp_packet->sub_header.ping_rate);
            #endif
				
			if (0){//rcnt%100==0){
                sprintf_unix_time(str_buf, wbms_sbp_packet->sub_header.time);
                fprintf(stderr,"WBMS SBP %d : ver=%d ping=%7d  %s\n",
                        rcnt,
                        wbms_sbp_packet->header.version,
                        wbms_sbp_packet->sub_header.ping_number,
                        str_buf);
            }

			#ifdef GUESS_NEXT_WBMS_TS
			*ts_out = wbms_sbp_packet->sub_header.time+sensor_offset->time_offset + (1./(wbms_sbp_packet->sub_header.ping_rate));
			#else
			*ts_out = wbms_sbp_packet->sub_header.time+sensor_offset->time_offset;
			#endif
			return PACKET_TYPE_SBP_DATA;
		default:
			if(verbose>2) fprintf(stderr,"Received unknown WBMS data, Discarding!\n");
			return 0;
		}
	return 0;
}

//Sorting functions
int cmp_wbms_dp_on_angle_func (const void * a, const void * b) {
	detectionpoint_t *dp_a, *dp_b;
	dp_a = (detectionpoint_t*) a;
	dp_b = (detectionpoint_t*) b;
	
	return ( dp_a->angle - dp_b->angle )>0?1:-1;
}
int cmp_wbms_v5_dp_on_angle_func (const void * a, const void * b) {
	detectionpoint_v5_t *dp_a, *dp_b;
	dp_a = (detectionpoint_v5_t*) a;
	dp_b = (detectionpoint_v5_t*) b;
	
	return ( dp_a->angle - dp_b->angle )>0?1:-1;
}
int cmp_wbms_v104_dp_on_angle_func (const void * a, const void * b) {
	detectionpoint_v104_t *dp_a, *dp_b;
	dp_a = (detectionpoint_v104_t*) a;
	dp_b = (detectionpoint_v104_t*) b;
	
	return ( dp_a->angle - dp_b->angle )>0?1:-1;
}

/***************************************************************************//**
* @brief        Calculate rx angle correction to compensate for sonar tx and rx beam intersecting as two cones, while we wnat to output angles in polar coordinates
* @param[in]    azimuth             rx angle in radians
* @param[in]    elevation           tx angle in radians
* @param[in]    steer               rx angle (phased array steered part) in radians
*
* @return difference from corrected azimuth angle
*
******************************************************************************/
static float calc_sonar_to_cp_corrections(float azimuth, float elevation, float steer){
    //See assembla ticket #1725
    float az,az1,az2,el,cos_el,sin2_el, sin_az2, sin2_az2;
    el = elevation;
    cos_el = cos(el);
    sin2_el = sin(el); sin2_el*=sin2_el;

    az = azimuth;                            //Total azimuth steering
    az2 = steer;                            //Steering done by phase shifting array
    az1 = az-az2;                                //Steering done by physically shifting array
    sin_az2= sin(az2);
    sin2_az2 = sin_az2*sin_az2;
    return  asin ( ( cos(az1)*sin_az2 + sin(az1)*sqrt(1-sin2_az2-sin2_el) )/cos_el )-az;
}

static float calc_shallow_angle_skew_corrections(float angle, float range, float att){
    angle = LIMIT(angle,-80.f*M_PI/180.f, 80.f*M_PI/180.f);
    //Opening angle model
    const float psi0 = 0.5*M_PI/180;        //Nadir opening angle for WH
    float psi = psi0/cosf(angle);         //Opening angle at specific angle
    //Convert opening angle to k-value for exponential function
    // 1./(sqrt(2*np.log(2))) = 0.849321   
    float k = psi*0.84932f;

    //Convert attenuation in dB/km (one way) to damping factor alpha
    //  np.log(10)/20e3 = 0.0001151292
    float alpha = att * 0.0001151292f;
    return -(k*k)/2 * (1+alpha*range) * tanf(angle);
}

uint32_t wbms_georef_data( bath_data_packet_t* bath, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,/*OUTPUT*/ output_data_t* outbuf,/*INPUT*/ uint32_t force_bath_version){
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
     int*   quality_flags = &(outbuf->quality_flags[0]);
     float* priority = &(outbuf->priority[0]);
     float* strength = &(outbuf->strength[0]);
     float* tx_angle_out = &(outbuf->tx_angle);
     float* sv_out = &(outbuf->sv);
     float* tx_freq_out = &(outbuf->tx_freq);
     float* tx_bw_out = &(outbuf->tx_bw);
     float* tx_plen_out = &(outbuf->tx_plen);
     float* tx_voltage_out = &(outbuf->tx_voltage);
     int* ping_number_out = &(outbuf->ping_number);
     int* multiping_index_out = &(outbuf->multiping_index);
     int* multifreq_index_out = &(outbuf->multifreq_index);

	float sensor_strength;
    #ifdef OUTPUT_QUALITY_VAL
	float sensor_quality;
	#endif
    float sensor_lg;
	float sensor_ug;
	float sensor_r;
	float sensor_az;
	float sensor_el;
	float sensor_t;
    float sensor_elec_steer;
	//float xs[MAX_DP], ys[MAX_DP], zs[MAX_DP];	/*Coordinate with respect to sensor (forward,starboard, down)*/																					 

	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    uint32_t bath_version =  bath->header.version;
    bath_data_packet_v5_t* bath_v5 = (bath_data_packet_v5_t*) bath; /* Cast to v5/v6 packet, could use an union type here as well */
    bath_data_packet_v7_t* bath_v7 = (bath_data_packet_v7_t*) bath; /* Cast to v7 packet, could use an union type here as well */
    bath_data_packet_v8_t* bath_v8 = (bath_data_packet_v8_t*) bath; /* Cast to v8 packet, could use an union type here as well */
    bath_data_packet_v104_t* bath_v104 = (bath_data_packet_v104_t*) bath; /* Cast to v5 packet, could use an union type here as well */
    float tx_angle; 
	float Fs;
	float c;
	uint16_t Nin;
    uint32_t ping_number;
    uint16_t multiping_index;
    uint16_t multifreq_index;
	uint16_t Nout;
	uint16_t ix_in,ix_out;
    const uint16_t ix_in_stride = sensor_params->beam_decimate; 
    const uint32_t ping_number_stride = sensor_params->ping_decimate; 
    #define ROLL_VECTOR_LEN 512
    #define ROLL_VECTOR_RATE 500.
    float roll_vector[ROLL_VECTOR_LEN];
    float z_vector[ROLL_VECTOR_LEN];
	float sensor_az_tx2rx_corr;
	float sensor_z_tx2rx_corr;
    float tx_freq;
    float tx_bw;
    float tx_plen;
    float tx_voltage;
    
    if (force_bath_version>0){
        bath_version =  force_bath_version;
    }

    //fprintf(stderr,"Bathy packet version = %d\n",bath_version);
    if (bath_version < 5){
        tx_freq = bath->sub_header.tx_freq;
        tx_bw = bath->sub_header.tx_bw;
        tx_plen = bath->sub_header.tx_len;
        tx_voltage = bath->sub_header.tx_voltage;
        tx_angle = bath->sub_header.tx_angle;
        Fs = bath->sub_header.sample_rate;
        c = bath->sub_header.snd_velocity+sensor_params->sv_offset;
        Nin = bath->sub_header.N;
        multifreq_index = ((bath->sub_header.flags)>>4) & 0x03;
        ping_number =  bath->sub_header.ping_number;
        multiping_index =  bath->sub_header.multiping_seq_nr;
    }
    else if (bath_version == 104){
        tx_freq = bath_v104->sub_header.tx_freq;
        tx_bw = bath_v104->sub_header.tx_bw;
        tx_plen = bath_v104->sub_header.tx_len;
        tx_voltage = bath_v104->sub_header.tx_voltage;
        tx_angle = bath_v104->sub_header.tx_angle;
        Fs = bath_v104->sub_header.sample_rate;
        c = bath_v104->sub_header.snd_velocity+sensor_params->sv_offset;
        Nin = bath_v104->sub_header.N;
        multifreq_index = ((bath_v104->sub_header.flags)>>4) & 0x03;
        ping_number =  bath_v104->sub_header.ping_number;
        multiping_index =  bath->sub_header.multiping_seq_nr;
    }
    else if ((bath_version == 5) || (bath_version == 6)){
        tx_freq = bath_v5->sub_header.tx_freq;
        tx_bw = bath_v5->sub_header.tx_bw;
        tx_plen = bath_v5->sub_header.tx_len;
        tx_angle = bath_v5->sub_header.tx_angle;
        tx_voltage = bath_v5->sub_header.tx_voltage;
        Fs = bath_v5->sub_header.sample_rate;
        c = bath_v5->sub_header.snd_velocity+sensor_params->sv_offset;
        Nin = bath_v5->sub_header.N;
        multifreq_index = ((bath_v5->sub_header.flags)>>4) & 0x03;
        ping_number =  bath_v5->sub_header.ping_number;
        multiping_index =  bath->sub_header.multiping_seq_nr;
    }
    else if (bath_version == 7){
        tx_freq = bath_v7->sub_header.tx_freq;
        tx_bw = bath_v7->sub_header.tx_bw;
        tx_plen = bath_v7->sub_header.tx_len;
        tx_angle = bath_v7->sub_header.tx_angle;
        tx_voltage = bath_v7->sub_header.tx_voltage;
        Fs = bath_v7->sub_header.sample_rate;
        c = bath_v7->sub_header.snd_velocity+sensor_params->sv_offset;
        Nin = bath_v7->sub_header.N;
        multifreq_index =bath_v7->sub_header.multifreq_band_index;
        ping_number =  bath_v7->sub_header.ping_number;
        multiping_index =  bath_v7->sub_header.multiping_scan_index;
    }
    else {//if (bath_version == 8){
        tx_freq = bath_v8->sub_header.tx_freq;
        tx_bw = bath_v8->sub_header.tx_bw;
        tx_plen = bath_v8->sub_header.tx_len;
        tx_angle = bath_v8->sub_header.tx_angle;
        tx_voltage = bath_v8->sub_header.tx_voltage;
        Fs = bath_v8->sub_header.sample_rate;
        c = bath_v8->sub_header.snd_velocity+sensor_params->sv_offset;
        Nin = bath_v8->sub_header.N;
        multifreq_index =bath_v8->sub_header.multifreq_band_index;
        ping_number =  bath_v8->sub_header.ping_number;
        multiping_index =  bath_v8->sub_header.multiping_scan_index;
    }
    *sv_out = c; 
    *multiping_index_out = multiping_index;
    *multifreq_index_out = multifreq_index;
    *tx_freq_out = tx_freq;
    *tx_bw_out = tx_bw;
    *tx_plen_out = tx_plen;
    *tx_voltage_out = tx_voltage;
    *ping_number_out = ping_number;
	float div_Fs = 1.f/Fs;
	float c_div_2Fs = c/(2*Fs);
    /*printf("ping_number=%9d, multiping= %d tx_angle=%5.1f multifreq_index=%d\n", 
    ping_number, multiping_index,tx_angle*180/M_PI, multifreq_index);*/
    //printf("tx_angle=%f, Fs=%f, c=%f, Nin=%d, multifreq_index=%d\n", tx_angle,  Fs,  c,  Nin,  multifreq_index);

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
    if (bath_version < 5){
        if (calc_interpolated_nav_data( posdata, pos_ix, bath->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
            if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath->sub_header.time+sensor_offset->time_offset);
            return 0;
        }
        calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath->sub_header.time+sensor_offset->time_offset, (1.f/bath->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        if(sensor_params->calc_aoi){
            qsort(bath->dp, Nin, sizeof(detectionpoint_t), cmp_wbms_dp_on_angle_func);
        }
    }
    else if(bath_version==104){ //Same, but for v104 packets
        if (calc_interpolated_nav_data( posdata, pos_ix, bath_v104->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
            if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath->sub_header.time+sensor_offset->time_offset);
            return 0;
        }
        calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath_v104->sub_header.time+sensor_offset->time_offset, (1.f/bath_v104->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        if(sensor_params->calc_aoi){
            qsort(bath_v104->dp, Nin, sizeof(detectionpoint_v104_t), cmp_wbms_v104_dp_on_angle_func);
        }
    }
    else if ((bath_version==5)||(bath_version==6)){ //Same, but for v5 packets
        if (calc_interpolated_nav_data( posdata, pos_ix, bath_v5->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
            if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath->sub_header.time+sensor_offset->time_offset);
            return 0;
        }
        calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath_v5->sub_header.time+sensor_offset->time_offset, (1.f/bath_v5->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        if(sensor_params->calc_aoi){
            qsort(bath_v5->dp, Nin, sizeof(detectionpoint_v5_t), cmp_wbms_v5_dp_on_angle_func);
        }
    }
    else if (bath_version==7){ //Same, but for v7 packets
        if (calc_interpolated_nav_data( posdata, pos_ix, bath_v7->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
            if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath->sub_header.time+sensor_offset->time_offset);
            return 0;
        }
        calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath_v7->sub_header.time+sensor_offset->time_offset, (1.f/bath_v5->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        if(sensor_params->calc_aoi){
            qsort(bath_v7->dp, Nin, sizeof(detectionpoint_v5_t), cmp_wbms_v5_dp_on_angle_func);
        }
    }
    else {//if (bath_version==8){ //Same, but for v8 packets
        if (calc_interpolated_nav_data( posdata, pos_ix, bath_v8->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
            if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath->sub_header.time+sensor_offset->time_offset);
            return 0;
        }
        calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath_v8->sub_header.time+sensor_offset->time_offset, (1.f/bath_v5->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        if(sensor_params->calc_aoi){
            qsort(bath_v8->dp, Nin, sizeof(detectionpoint_v5_t), cmp_wbms_v5_dp_on_angle_func);
        }
    }

	if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }

    
    float *xs = malloc(MAX_DP*sizeof(float));
    float *ys = malloc(MAX_DP*sizeof(float));
    float *zs = malloc(MAX_DP*sizeof(float));

	// Populate r,az,el and t with data from bath data
	float prev_sensor_r = 0.;
	float prev_sensor_az = 0.;

	float inten;
    uint8_t sensor_quality_flags;
    uint8_t priority_flags;
    uint16_t flags;

    //Calculate sounding positions in sonar reference frame at tx instant
	ix_out = 0;
    //printf("Nin=%d\n",Nin);

	for (ix_in=0;ix_in<Nin;ix_in+=ix_in_stride){
        if (bath_version < 5){
            uint32_t sample_number;
            if (sensor_params->sonar_sample_mode == upper_gate) 
                sample_number = bath->dp[ix_in].upper_gate;
            else if (sensor_params->sonar_sample_mode == lower_gate) 
                sample_number = bath->dp[ix_in].lower_gate;
            else if (sensor_params->sonar_sample_mode == center_gate) 
                sample_number = bath->dp[ix_in].lower_gate/2 + bath->dp[ix_in].upper_gate/2;
            else
                sample_number = bath->dp[ix_in].sample_number;
            sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
            sensor_ug = bath->dp[ix_in].upper_gate*c_div_2Fs;
            sensor_lg = bath->dp[ix_in].lower_gate*c_div_2Fs;
            #ifdef OUTPUT_QUALITY_VAL
            sensor_quality = (float)(bath->dp[ix_in].quality_val);
            #endif
            sensor_strength = 0.0f;
            sensor_t = sample_number*div_Fs;		//Calculate tx to rx time for each point 
            sensor_az  = bath->dp[ix_in].angle;
            sensor_elec_steer = 0;                                  // Batt version prev 5 does not contain this information
            sensor_quality_flags = bath->dp[ix_in].quality_flags;
            flags = bath->dp[ix_in].flags;
            inten = bath->dp[ix_in].intensity;
        }
        else if(bath_version ==104){
            float sample_number;
            if (sensor_params->sonar_sample_mode == upper_gate) 
                sample_number = (float) bath_v104->dp[ix_in].upscaled_upper_gate;
            else if (sensor_params->sonar_sample_mode == lower_gate) 
                sample_number = (float) bath_v104->dp[ix_in].upscaled_lower_gate;
            else
                sample_number = (float) bath_v104->dp[ix_in].upscaled_sample_number;

            sensor_r   = sample_number*c_div_2Fs/SAMPLE_NUMBER_V104_UPSCALE;	//Calculate range to each point en meters
            sensor_ug = (float) bath_v104->dp[ix_in].upscaled_upper_gate*c_div_2Fs/SAMPLE_NUMBER_V104_UPSCALE;
            sensor_lg = (float) bath_v104->dp[ix_in].upscaled_lower_gate*c_div_2Fs/SAMPLE_NUMBER_V104_UPSCALE;
            #ifdef OUTPUT_QUALITY_VAL
            sensor_quality = (float)(bath_v104->dp[ix_in].quality_val);
            #endif
            sensor_strength = 0.0f;
            sensor_t =  sample_number*div_Fs;		//Calculate tx to rx time for each point 
            sensor_az  = (float) bath_v104->dp[ix_in].angle;
            sensor_elec_steer = 0;
            sensor_quality_flags = bath_v104->dp[ix_in].quality_flags;
            flags = bath_v104->dp[ix_in].flags;
            inten = (float) bath_v104->dp[ix_in].intensity;
        }
        else if ((bath_version==5)||(bath_version==6)){
            float sample_number;
            if (sensor_params->sonar_sample_mode == upper_gate) 
                sample_number = bath_v5->dp[ix_in].upper_gate;
            else if (sensor_params->sonar_sample_mode == lower_gate) 
                sample_number = bath_v5->dp[ix_in].lower_gate;
            else
                sample_number = bath_v5->dp[ix_in].sample_number;

            sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
            sensor_ug = bath_v5->dp[ix_in].upper_gate*c_div_2Fs;
            sensor_lg = bath_v5->dp[ix_in].lower_gate*c_div_2Fs;
            #ifdef OUTPUT_QUALITY_VAL
            sensor_quality = (float)(bath_v5->dp[ix_in].quality_val);
            #endif
            sensor_strength = (float)(bath_v5->dp[ix_in].strength);
            sensor_t =  sample_number*div_Fs;		//Calculate tx to rx time for each point 
            sensor_az  = bath_v5->dp[ix_in].angle;
            sensor_elec_steer = bath_v5->dp[ix_in].steer_angle;
            sensor_quality_flags = bath_v5->dp[ix_in].quality_flags;
            flags = bath_v5->dp[ix_in].flags;
            inten = bath_v5->dp[ix_in].intensity;
        }
        else if (bath_version==7){
            uint32_t sample_number;
            if (sensor_params->sonar_sample_mode == upper_gate) 
                sample_number = bath_v7->dp[ix_in].upper_gate;
            else if (sensor_params->sonar_sample_mode == lower_gate) 
                sample_number = bath_v7->dp[ix_in].lower_gate;
            else if (sensor_params->sonar_sample_mode == center_gate) 
                sample_number = bath_v7->dp[ix_in].lower_gate/2 + bath_v7->dp[ix_in].upper_gate/2;
            else
                sample_number = bath_v7->dp[ix_in].sample_number;
            sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
            sensor_ug = (float)(bath_v7->dp[ix_in].upper_gate)*c_div_2Fs;
            sensor_lg = (float)(bath_v7->dp[ix_in].lower_gate)*c_div_2Fs;
            #ifdef OUTPUT_QUALITY_VAL
            sensor_quality = (float)(bath_v7->dp[ix_in].quality_val);
            #endif
            sensor_strength = 0.0f;
            sensor_t = sample_number*div_Fs;		//Calculate tx to rx time for each point 
            sensor_az  = bath_v7->dp[ix_in].angle;
            sensor_elec_steer = 0;                                  // Batt version prev 5 does not contain this information
            sensor_quality_flags = bath_v7->dp[ix_in].quality_flags;
            flags = bath_v7->dp[ix_in].flags;
            inten = bath_v7->dp[ix_in].intensity;
        }
        else {//if (bath_version==8){
            float sample_number;
            if (sensor_params->sonar_sample_mode == upper_gate) 
                sample_number = bath_v8->dp[ix_in].upper_gate;
            else if (sensor_params->sonar_sample_mode == lower_gate) 
                sample_number = bath_v8->dp[ix_in].lower_gate;
            else
                sample_number = bath_v8->dp[ix_in].sample_number;

            sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
            sensor_ug = bath_v8->dp[ix_in].upper_gate*c_div_2Fs;
            sensor_lg = bath_v8->dp[ix_in].lower_gate*c_div_2Fs;
            #ifdef OUTPUT_QUALITY_VAL
            sensor_quality = (float)(bath_v8->dp[ix_in].quality_val);
            #endif
            sensor_strength = (float)(bath_v8->dp[ix_in].strength);
            sensor_t =  sample_number*div_Fs;		//Calculate tx to rx time for each point 
            sensor_az  = bath_v8->dp[ix_in].angle;
            sensor_elec_steer = bath_v8->dp[ix_in].steer_angle;
            sensor_quality_flags = bath_v8->dp[ix_in].quality_flags;
            flags = bath_v8->dp[ix_in].flags;
            inten = bath_v8->dp[ix_in].intensity;
        }
        priority_flags = ((flags)>>9) & (0x0F);
        sensor_r  += sensor_offset->r_err;
        sensor_ug  += sensor_offset->r_err;
        sensor_lg  += sensor_offset->r_err;


        //#define STX_GEOM_COR
        #ifdef STX_GEOM_COR
        sensor_az += calc_sonar_to_cp_corrections(sensor_az,sensor_el,sensor_elec_steer);
        #endif


    
        //#define SHALLOW_ANGLE_SKEW_COR
        #ifdef SHALLOW_ANGLE_SKEW_COR
        sensor_az += calc_shallow_angle_skew_corrections(sensor_az,sensor_r, sensor_params->intensity_range_attenuation);
        #endif

            
        #ifdef FORCE_MULTIDETECT_TO_QUALITY3
        if (priority_flags==1 || priority_flags ==2){
            sensor_quality_flags = 3;
        }
        #endif
        
        // Add correction for roll during tx2rx period for each beam individually
        sensor_az_tx2rx_corr = -roll_vector[(size_t) round(sensor_t*ROLL_VECTOR_RATE)]; //Roll is given in opposite angles than sonar azimuth
        sensor_z_tx2rx_corr = z_vector[(size_t) round((sensor_t/2)*ROLL_VECTOR_RATE)]; // Z correction is for half tx to rx time
        //printf("sensor_quality_flags=%d,  sensor_params->min_priority_flag=%d, sensor_az=%fdeg, sensor_el=%fdeg\n",sensor_quality_flags,((flags)>>9) & (0x0F),sensor_az*180/M_PI, sensor_el*180/M_PI);
        //printf("sonar_min_quality_flag=%d, sensor_params->max_quality_flag=%d, sensor_params->min_priority_flag=%d, sensor_params->max_priority_flag=%d\n",sensor_params->min_quality_flag,sensor_params->max_quality_flag,sensor_params->min_priority_flag,sensor_params->max_priority_flag);
		if (	(sensor_quality_flags >= sensor_params->min_quality_flag) && 
				(sensor_quality_flags <= sensor_params->max_quality_flag) &&
		        (priority_flags >= sensor_params->min_priority_flag) && 
				(priority_flags <= sensor_params->max_priority_flag) &&
				(sensor_az > sensor_params->min_azimuth) && (sensor_az < sensor_params->max_azimuth) &&
				(sensor_r > sensor_params->min_range) && (sensor_r < sensor_params->max_range) 
			){
            
            beam_number[ix_out] = ix_in;
            beam_angle[ix_out] =  sensor_az;  //Store raw beam angle from sonar for data analysis
            beam_steer[ix_out] = sensor_elec_steer;
            beam_range[ix_out] = sensor_r;
            upper_gate_range[ix_out] = sensor_ug;
            lower_gate_range[ix_out] = sensor_lg;
            strength[ix_out] = sensor_strength;
            #ifdef OUTPUT_QUALITY_VAL
                quality[ix_out] = sensor_quality;
            #else
                quality[ix_out] = (float) sensor_quality_flags;
            #endif
            quality_flags[ix_out] = (float) sensor_quality_flags;
            priority[ix_out] = (float) priority_flags;
            *tx_angle_out = sensor_el;
			
			//Compensate intensity for range and AOI
            if (sensor_params->calc_aoi){
                aoi[ix_out] = atan2f((sensor_r-prev_sensor_r), (sensor_az-prev_sensor_az)*sensor_r);         //AOI defined as angle between seafloor normal and beam (not seafloor and beam)
            }
            else{
                aoi[ix_out] = sensor_az;        //Just asume that AOI is equal to beam angle (flat seafloor assumption)
            }
            aoi[ix_out] = (aoi[ix_out]);
            if (sensor_params->intensity_range_comp){
                inten *= sensor_r;                  //Only comp one-way spreading loss     
                float damping_dB = sensor_params->intensity_range_attenuation * (2*sensor_r/1000); 
                inten *= powf(10.f,damping_dB/20); 
            }
            if (sensor_params->intensity_aoi_comp){
                if(sensor_params->use_intensity_angle_corr_table){
                    int ix = ABS(aoi[ix_out])/INTENSITY_ANGLE_STEP;
                    ix = LIMIT(ix,0,INTENSITY_ANGLE_MAX_VALUES-1);
                    inten *= intenity_angle_corr_table[ix].intensity_scale;
                }
                else{
                    inten = inten/((MAX(cosf(aoi[ix_out]),0.1f)));
                }
            }
            intensity[ix_out] = inten;

			#ifdef CONE_CONE_COORD
			xs[ix_out] = sensor_r * sin(sensor_el);
			ys[ix_out] = sensor_r * sin(sensor_az); //Sign flipped compared to standard right hand system
			zs[ix_out] = sensor_r * sqrt(1. - (sin((sensor_az+sensor_az_tx2rx_corr))*sin((sensor_az+sensor_az_tx2rx_corr))) - (sin(sensor_el)*sin(sensor_el)));
			#else
			/***** Converting sensor data from spherical to kartesian coordinates *********/
			// Projection assumes cone-plane coordinates (See assembla ticket #1725)								 
			// Standard sonar mounting az=0 el=0 => Nadir(0,0,1), az=90 el=0 => Starbord(0,1,0), az=0 el=90 => Forward(1,0,0) 
			// Sign of y axis is flipped (compared to #1725) because az rotation is left to right, and not right to left which it should be if it was a positive rotation around x-axis
			xs[ix_out] = sensor_r * sinf(sensor_el);
			ys[ix_out] = sensor_r * sinf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el); //Sign flipped compared to standard right hand system
			zs[ix_out] = sensor_r * cosf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el);
			#endif
            zs[ix_out] += sensor_z_tx2rx_corr;


            //TODO insert uncertainty model here
            float beam_width = (0.1*M_PI/180.) / cosf(0.85*sensor_az);
            float sigma_teta =  M_SQRT2 * beam_width;
            const float sigma_range = M_SQRT2 * 1450./80e3;
            const float sigma_t = M_SQRT2 * 0.005;
            float sigma_z_teta  = sigma_teta*sensor_r*sinf(sensor_az);
            float sigma_z_range = sigma_range*cosf(sensor_az);
            float sigma_z_roll = nav_droll_dt * sigma_t *sensor_r*cosf(sensor_az);
            float sigma_z_pitch = nav_dpitch_dt * sigma_t *sensor_r*cosf(sensor_az);
            float sigma_aoi = 5e-4f * sensor_r / powf(cosf(aoi[ix_out]),2); 
            sigma_aoi = MIN(sigma_aoi, sensor_r/200);  //Limit aoi dependent std dev to max 2% range
            z_var[ix_out] =  sigma_z_teta*sigma_z_teta + sigma_z_range*sigma_z_range + sigma_z_roll*sigma_z_roll + sigma_z_pitch*sigma_z_pitch + sigma_aoi*sigma_aoi;
            //z_var[ix_out] =   sigma_aoi*sigma_aoi;
            //z_var[ix_out] =  sigma_z_pitch*sigma_z_pitch;

			ix_out++;
			
			prev_sensor_r = sensor_r;
			prev_sensor_az = sensor_az;
		}
        //else{
            //printf("sensor_quality_flags=%d,  sensor_params->min_priority_flag=%d, sensor_r=%f, sensor_az=%fdeg, sensor_el=%fdeg\n",sensor_quality_flags,((flags)>>9) & (0x0F),sensor_r,sensor_az*180/M_PI, sensor_el*180/M_PI);
            //printf("sensor_params->min_quality_flag=%d, sensor_params->max_quality_flag=%d, sensor_params->min_priority_flag=%d, sensor_params->max_priority_flag=%d\n",sensor_params->min_quality_flag,sensor_params->max_quality_flag,sensor_params->min_priority_flag,sensor_params->max_priority_flag);
        //}
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
        aoi[ix_out] = aoi[ix_in];
        beam_number[ix_out] = beam_number[ix_in];
        beam_steer[ix_out] = beam_steer[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
        upper_gate_range[ix_out] = upper_gate_range[ix_in];
        lower_gate_range[ix_out] = lower_gate_range[ix_in];

		if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;
        if (swath_y[ix_in]>sensor_params->swath_max_y || swath_y[ix_in]<sensor_params->swath_min_y) continue;

		ix_out++;
	}
	Nout = ix_out;
	#endif 
    //printf("Nout2 = %d\n",Nout);
	
    free(xs);free(ys);free(zs);
	return Nout;
}

static uint32_t calc_match_filter(float fc,float bw,float len, float Fs, /*output*/ float* mfir){
    float t,df,dfdn,dtdn;
    uint32_t N,n;
    N = (uint32_t) (len*Fs);

    dtdn = (1./Fs);
    t=-len/(2.*Fs);

    df = -bw/2;
    dfdn =  bw/(2.*len*Fs);
    
    for (n=0;n<N;n++){
        float apz_val = 1.f;//(0.54+0.46*Q_cos((2.*M_PI*(n+0.5))/N-M_PI));
        mfir[n] = sinf(2.*M_PI*(fc+df)*t) * apz_val;
        df += dfdn;
        t += dtdn; 
    }
    //Remove DC in filter
    float dc = 0;
    for (n=0;n<N;n++)
        dc+=mfir[n];
    dc/=N;
    for (n=0;n<N;n++)
        mfir[n] -= dc;
    //printf("Calc match filter: freq %1.2fkHz bw %1.2fkHz  len %1.0f us Fs = %1.2fkHz %d taps\n",fc/1000,bw/1000,len*1e6,Fs/1000,N);
    return N;
}

static void match_filter_data(/*Input*/ float* sig_in, float freq, float bw, float plen, float Fs, int32_t Nin, /*Output*/ float* sig_out){
    uint32_t mfir_len = (uint32_t) (plen*Fs);
    float* mfir = malloc(mfir_len * sizeof(float));
    calc_match_filter(freq,bw,plen, Fs, /*output*/ mfir);

    for (uint32_t ix = 0; ix < (mfir_len/2); ix++)
        sig_out[ix] = 0;
    for (uint32_t ix = mfir_len/2; ix < (Nin-mfir_len/2); ix++){
        sig_out[ix] = 0;
        for (uint32_t mix = 0; mix < mfir_len; mix++){
            sig_out[ix] += sig_in[ix - mfir_len/2 + mix]  *  mfir[mix];
        }
    }
    for (uint32_t ix = Nin-(mfir_len/2); ix < Nin; ix++)
        sig_out[ix] = 0;

    free(mfir);
}

static void bp_filter_data(/*Input*/ float* sig_in, float freq, float bw, float Fs, int32_t Nin, /*Output*/ float* sig_out){
    bw = LIMIT(bw,0,Fs);
    uint32_t bpfir_len = (uint32_t) (2*Fs/bw);
    float* bpfir = malloc(bpfir_len * sizeof(float));
    
    /*Windowed sinc with hamming window*/
    float fcn = freq/Fs;     //Normalized center freq 
    float bwn = bw/2/Fs;     //Normalized cutoff freq
    //Sinc function
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        float phi = 2*M_PI*bwn*ii;
        bpfir[ix] = phi==0?1.0f:sinf(phi)/phi;
    }
    //Mixing up to center freq
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        float phi = 2*M_PI*fcn*ii;
        bpfir[ix] *= cosf(phi);
    }
    //Adding window function (hamming)
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        bpfir[ix] *= 0.54 + 0.46*cos((2.*M_PI*ii)/bpfir_len);
    }

    
    for (uint32_t ix = 0; ix < (bpfir_len/2); ix++)
        sig_out[ix] = 0;
    for (uint32_t ix = bpfir_len/2; ix < (Nin-bpfir_len/2); ix++){
        sig_out[ix] = 0;
        for (uint32_t mix = 0; mix < bpfir_len; mix++){
            sig_out[ix] += sig_in[ix - bpfir_len/2 + mix]  *  bpfir[mix];
        }
    }
    for (uint32_t ix = Nin-(bpfir_len/2); ix < Nin; ix++)
        sig_out[ix] = 0;

    free(bpfir);
}
    


static void hilbert_envelope_data(/*Input*/ float* sig_in, int32_t Nin, /*Output*/ float* sig_out){
    static const uint32_t hlen = 63;
    static const float hcoef[] = {    -0.0162738 ,  0.01472801, -0.01798631,  0.01331904, -0.01990413,
                                0.01202334, -0.02207802,  0.01082204, -0.02457624,  0.0096998 ,
                               -0.02749287,  0.00864398, -0.03096146,  0.00764404, -0.03517822,
                                0.00669104, -0.04044378,  0.00577731, -0.04724363,  0.00489618,
                               -0.05641605,  0.00404177, -0.06954423,  0.0032088 , -0.09002035,
                                0.00239247, -0.12666366,  0.00158833, -0.21181068,  0.00079219,
                               -0.63648784,  0.        ,  0.63648784, -0.00079219,  0.21181068,
                               -0.00158833,  0.12666366, -0.00239247,  0.09002035, -0.0032088 ,
                                0.06954423, -0.00404177,  0.05641605, -0.00489618,  0.04724363,
                               -0.00577731,  0.04044378, -0.00669104,  0.03517822, -0.00764404,
                                0.03096146, -0.00864398,  0.02749287, -0.0096998 ,  0.02457624,
                               -0.01082204,  0.02207802, -0.01202334,  0.01990413, -0.01331904,
                                0.01798631, -0.01472801,  0.0162738 };
                                
    for (uint32_t ix = 0; ix < (hlen/2); ix++)
        sig_out[ix] = 0;
    
    for (uint32_t ix = hlen/2; ix < (Nin-hlen/2); ix++){
        float I = sig_in[ix];
        float Q = 0;
        for (uint32_t hix = 0; hix < hlen; hix++){
            Q += sig_in[ix - hlen/2 + hix]  *  hcoef[hix];
        }
        sig_out[ix] = sqrtf(I*I + Q*Q);
    }
    for (uint32_t ix = Nin-(hlen/2); ix < Nin; ix++)
        sig_out[ix] = 0;
}

uint32_t wbms_georef_sbp_data( sbp_data_packet_t* sbp_data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,/*OUTPUT*/ output_data_t* outbuf){
     double* x = &(outbuf->x[0]);
     double* y = &(outbuf->y[0]);
     double* z = &(outbuf->z[0]);
     float* intensity = &(outbuf->i[0]);
     float* beam_range = &(outbuf->range[0]);
     float* tx_freq_out = &(outbuf->tx_freq);
     float* tx_bw_out = &(outbuf->tx_bw);
     float* tx_plen_out = &(outbuf->tx_plen);
     float* tx_voltage_out = &(outbuf->tx_voltage);
     int* ping_number_out = &(outbuf->ping_number);
     float* sv_out = &(outbuf->sv);

	float sensor_r;

	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    //uint32_t sbp_data_version =  sbp_data->header.version;
	float Fs;
	float c;
	uint16_t Nin;
    uint32_t ping_number;
	uint16_t Nout;
	int32_t ix_in,ix_out;
    const uint16_t ix_in_stride = sensor_params->beam_decimate; 
    const uint32_t ping_number_stride = sensor_params->ping_decimate; 
    float tx_freq;
    float tx_bw;
    float tx_plen;
    float tx_voltage;

    
    tx_freq = sbp_data->sub_header.tx_sec_freq;
    tx_bw = sbp_data->sub_header.tx_sec_bw;
    tx_plen = sbp_data->sub_header.tx_len;
    tx_voltage = sbp_data->sub_header.tx_voltage;
    Fs = sbp_data->sub_header.sample_rate;
    c = DEFAULT_SBP_SV +sensor_params->sv_offset;
    Nin = sbp_data->sub_header.N;
    Nin = MIN(Nin,SBP_MAX_SAMPLES);
    ping_number =  sbp_data->sub_header.ping_number;
    int32_t* raw_sig = (int32_t *) &(sbp_data->payload[0]);


    *sv_out = c; 
    *tx_freq_out = tx_freq;
    *tx_bw_out = tx_bw;
    *tx_plen_out = tx_plen;
    *tx_voltage_out = tx_voltage;
    *ping_number_out = ping_number;
	float c_div_2Fs = c/(2*Fs);

    //Skip whole dataset condition
    if (   ((ping_number < sensor_params->min_ping_number) || (sensor_params->max_ping_number && (ping_number > sensor_params->max_ping_number))) ||
           ((ping_number%ping_number_stride) != 0)
        ){
        return(0);
    }

    //Find nav data for tx instant
    //Generate a vector with interpolated roll starting at tx time, with 1ms samplerate, for 1/ping_rate duration
    // This is tobe used to correct azimuth and heave data, as this is defined at rx (not tx) time
    //To calculate angle-of insidence, we must sort beams on angle (Strictly only neccessary for ISS data)
    //Sort sbp_data->dp[n] based on sbp_data->dp[n].angle for n in 0-Nin
    if (calc_interpolated_nav_data( posdata, pos_ix, sbp_data->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
        if(verbose) fprintf(stderr, "Could not find navigation data for WBMS sbp record at time %f\n",sbp_data->sub_header.time+sensor_offset->time_offset);
        return 0;
    }

	if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }

    float *xs  = malloc(SBP_MAX_SAMPLES*sizeof(float));
    float *ys  = malloc(SBP_MAX_SAMPLES*sizeof(float));
    float *zs  = malloc(SBP_MAX_SAMPLES*sizeof(float));
    float *sig = malloc(SBP_MAX_SAMPLES*sizeof(float));
    float *temp_sig = malloc(SBP_MAX_SAMPLES*sizeof(float));

    //printf("SBP data tp = %d\n",sbp_data->sub_header.tp);
    switch(sbp_data->sub_header.tp){
        case 0:   //Raw ADC data
            for (uint32_t ix=0;ix<Nin;ix++){
                temp_sig[ix] = (float) raw_sig[ix];
            }
            match_filter_data(/*Input*/ temp_sig, tx_freq, tx_bw, tx_plen,Fs ,Nin, /*Output*/ sig);
            bp_filter_data(/*Input*/ sig, tx_freq, tx_bw, Fs ,Nin, /*Output*/ temp_sig);
            hilbert_envelope_data(/*Input*/ temp_sig,  Nin, /*Output*/ sig);
            break;
        case 10: //Match filtered data
            for (uint32_t ix=0;ix<Nin;ix++){
                temp_sig[ix] = (float) raw_sig[ix];
            }
            hilbert_envelope_data(/*Input*/ temp_sig,  Nin, /*Output*/ sig);
            break;
        default: //Unknown testpoint
            //TODO Rudimentary sig processing, fix match-filter bp-filter and envelope extraction
            for (uint32_t ix=0;ix<Nin;ix++){
                sig[ix] = (float) ABS(raw_sig[ix]);
            }
            break;
    }

    //Calculate sounding positions in sonar reference frame at tx instant
	ix_out = 0;
    //printf("Nin=%d\n",Nin);
    float tx_delay_range = tx_plen*c/2;

    int32_t ix_start = (int32_t) (roundf((sensor_params->min_range+tx_delay_range)/c_div_2Fs));
    int32_t ix_stop  = (int32_t) (roundf((sensor_params->max_range+tx_delay_range)/c_div_2Fs));
    ix_start = LIMIT(ix_start,0,Nin); 
    ix_stop = LIMIT(ix_stop,ix_start,Nin); 

	for (ix_in=ix_start;ix_in<ix_stop;ix_in+=ix_in_stride){
            
	    float inten;
        uint32_t sample_number;
        sample_number = ix_in;
        
        sensor_r   = sample_number*c_div_2Fs - tx_delay_range;	//Calculate range to each point en meters
        inten = sig[ix_in];
        sensor_r  += sensor_offset->r_err;
        
        if (sensor_params->intensity_range_comp){
            inten *= sensor_r;                  //Only comp one-way spreading loss     
            float damping_dB = sensor_params->intensity_range_attenuation * (2*sensor_r/1000); 
            inten *= powf(10.f,damping_dB/20); 
        }

        intensity[ix_out] = inten;
        beam_range[ix_out] = sensor_r;
        // Sub bottom profiler is always pointing straight down
        xs[ix_out] = 0.; 
        ys[ix_out] = 0.; 
        zs[ix_out] = sensor_r;
        ix_out++;
    }
    Nout = ix_out;

    georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
	
	//Post GEO-REF filtering
	Nin = Nout;
	ix_out = 0;
	for (ix_in=0;ix_in<Nin;ix_in++){
		x[ix_out] = x[ix_in];
		y[ix_out] = y[ix_in];
		z[ix_out] = z[ix_in];
		intensity[ix_out] = intensity[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
		if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;

		ix_out++;
	}
	Nout = ix_out;
	
    free(xs);free(ys);free(zs);free(sig);free(temp_sig);
	return Nout;
}
