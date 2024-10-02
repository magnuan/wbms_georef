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
#define ROLL_VECTOR_LEN 512
#define ROLL_VECTOR_RATE 500.

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
            else if (type==PACKET_TYPE_SNIPPET_DATA){
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
	snippet_data_packet_t* wbms_snippet_packet;
	bath_data_packet_t* wbms_bath_packet;
    sbp_data_packet_t* wbms_sbp_packet;
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
			//#define GUESS_NEXT_WBMS_TS
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
			wbms_snippet_packet = (snippet_data_packet_t*) databuffer;
            if (version){
                *version = wbms_snippet_packet->header.version;
            }
			#ifdef GUESS_NEXT_WBMS_TS
			*ts_out = wbms_snippet_packet->sub_header.time+sensor_offset->time_offset + (1./(wbms_snippet_packet->sub_header.ping_rate));
			#else
			*ts_out = wbms_snippet_packet->sub_header.time+sensor_offset->time_offset;
			#endif
			return PACKET_TYPE_SNIPPET_DATA;
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
int cmp_wbms_vX_dp_on_angle_func (const void * a, const void * b) {
	detectionpoint_vX_t *dp_a, *dp_b;
	dp_a = (detectionpoint_vX_t*) a;
	dp_b = (detectionpoint_vX_t*) b;
	
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
__attribute__((unused)) static float calc_sonar_to_cp_corrections(float azimuth, float elevation, float steer){
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

__attribute__((unused)) static float calc_shallow_angle_skew_corrections(float angle, float range, float att){
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

static bath_data_packet_vX_t bvX;

bath_data_packet_vX_t* bath_convert_to_universal(bath_data_packet_t* bath_in, uint32_t force_bath_version){
    uint32_t bath_version =  bath_in->header.version;
    static int first_run=1;
    if (force_bath_version>0){
        bath_version =  force_bath_version;
    }
    if (first_run){
        fprintf(stderr, "bath_version = %d\n",bath_version);
        first_run = 0;
    }
    
    if (bath_version < 5){
        bath_data_packet_t* bath_v4 = (bath_data_packet_t*) bath_in; 
        bvX.sub_header.snd_velocity         = bath_v4->sub_header.snd_velocity;
        bvX.sub_header.sample_rate          = bath_v4->sub_header.sample_rate;
        bvX.sub_header.N                    = bath_v4->sub_header.N;
        bvX.sub_header.ping_number          = bath_v4->sub_header.ping_number;
        bvX.sub_header.time                 = bath_v4->sub_header.time;
        bvX.sub_header.time_net             = bath_v4->sub_header.time_net;
        bvX.sub_header.ping_rate            = bath_v4->sub_header.ping_rate;
        bvX.sub_header.type                 = bath_v4->sub_header.type;
        bvX.sub_header.flags                = bath_v4->sub_header.flags;
        bvX.sub_header.sonar_mode           = bath_v4->sub_header.sonar_mode;
        bvX.sub_header.time_uncertainty     = bath_v4->sub_header.time_uncertainty;
        bvX.sub_header.multiping_scan_number= bath_v4->sub_header.multiping;
        bvX.sub_header.multifreq_band_number= 0;
        bvX.sub_header.multiping_scan_index = bath_v4->sub_header.multiping_seq_nr;
        bvX.sub_header.multifreq_band_index = 1;
        bvX.sub_header.tx_angle             = bath_v4->sub_header.tx_angle;
        bvX.sub_header.gain                 = bath_v4->sub_header.gain;
        bvX.sub_header.tx_freq              = bath_v4->sub_header.tx_freq;
        bvX.sub_header.tx_bw                = bath_v4->sub_header.tx_bw;
        bvX.sub_header.tx_len               = bath_v4->sub_header.tx_len;
        bvX.sub_header.snd_velocity_raw     = bath_v4->sub_header.snd_velocity_raw;
        bvX.sub_header.tx_voltage           = bath_v4->sub_header.tx_voltage;
        bvX.sub_header.swath_dir            = bath_v4->sub_header.swath_dir;
        bvX.sub_header.swath_open           = bath_v4->sub_header.swath_open;
        bvX.sub_header.gate_tilt            = bath_v4->sub_header.gate_tilt;
        bvX.sub_header.intensity_noise_ref  = 0;
        bvX.sub_header.strength_noise_ref   = 0; 
        for (uint32_t n=0; n<bvX.sub_header.N;n++){
            bvX.dp[n].sample_number =  bath_v4->dp[n].sample_number;   
            bvX.dp[n].angle         =  bath_v4->dp[n].angle        ;   
            bvX.dp[n].steer_angle   =  0;   
            bvX.dp[n].upper_gate    =  bath_v4->dp[n].upper_gate   ;   
            bvX.dp[n].lower_gate    =  bath_v4->dp[n].lower_gate   ;   
            bvX.dp[n].intensity     =  bath_v4->dp[n].intensity    ;   
            bvX.dp[n].strength      =  0;   
            bvX.dp[n].flags         =  bath_v4->dp[n].flags        ;   
            bvX.dp[n].quality_flags =  bath_v4->dp[n].quality_flags;   
            bvX.dp[n].quality_val   =  bath_v4->dp[n].quality_val  ;   
            bvX.dp[n].beam_number   =  n;   
        }
        
    }
    else if ((bath_version == 5) || (bath_version == 6)){
        bath_data_packet_v5_t* bath_v5 = (bath_data_packet_v5_t*) bath_in; /* Cast to v5/v6 packet, could use an union type here as well */
        bvX.sub_header.snd_velocity         = bath_v5->sub_header.snd_velocity;
        bvX.sub_header.sample_rate          = bath_v5->sub_header.sample_rate;
        bvX.sub_header.N                    = bath_v5->sub_header.N;
        bvX.sub_header.ping_number          = bath_v5->sub_header.ping_number;
        bvX.sub_header.time                 = bath_v5->sub_header.time;
        bvX.sub_header.time_net             = bath_v5->sub_header.time_net;
        bvX.sub_header.ping_rate            = bath_v5->sub_header.ping_rate;
        bvX.sub_header.type                 = bath_v5->sub_header.type;
        bvX.sub_header.flags                = bath_v5->sub_header.flags;
        bvX.sub_header.sonar_mode           = bath_v5->sub_header.sonar_mode;
        bvX.sub_header.time_uncertainty     = bath_v5->sub_header.time_uncertainty;
        bvX.sub_header.multiping_scan_number= bath_v5->sub_header.multiping;
        bvX.sub_header.multifreq_band_number= 0;
        bvX.sub_header.multiping_scan_index = bath_v5->sub_header.multiping_seq_nr;
        bvX.sub_header.multifreq_band_index = 1;
        bvX.sub_header.tx_angle             = bath_v5->sub_header.tx_angle;
        bvX.sub_header.gain                 = bath_v5->sub_header.gain;
        bvX.sub_header.tx_freq              = bath_v5->sub_header.tx_freq;
        bvX.sub_header.tx_bw                = bath_v5->sub_header.tx_bw;
        bvX.sub_header.tx_len               = bath_v5->sub_header.tx_len;
        bvX.sub_header.snd_velocity_raw     = bath_v5->sub_header.snd_velocity_raw;
        bvX.sub_header.tx_voltage           = bath_v5->sub_header.tx_voltage;
        bvX.sub_header.swath_dir            = bath_v5->sub_header.swath_dir;
        bvX.sub_header.swath_open           = bath_v5->sub_header.swath_open;
        bvX.sub_header.gate_tilt            = bath_v5->sub_header.gate_tilt;
        bvX.sub_header.intensity_noise_ref  = bath_v5->sub_header.intensity_noise_ref;
        bvX.sub_header.strength_noise_ref   = bath_v5->sub_header.strength_noise_ref ; 
        for (uint32_t n=0; n<bvX.sub_header.N;n++){
            bvX.dp[n].sample_number =  bath_v5->dp[n].sample_number;   
            bvX.dp[n].angle         =  bath_v5->dp[n].angle        ;   
            bvX.dp[n].steer_angle   =  bath_v5->dp[n].steer_angle  ;   
            bvX.dp[n].upper_gate    =  bath_v5->dp[n].upper_gate   ;   
            bvX.dp[n].lower_gate    =  bath_v5->dp[n].lower_gate   ;   
            bvX.dp[n].intensity     =  bath_v5->dp[n].intensity    ;   
            bvX.dp[n].strength      =  bath_v5->dp[n].strength     ;   
            bvX.dp[n].flags         =  bath_v5->dp[n].flags        ;   
            bvX.dp[n].quality_flags =  bath_v5->dp[n].quality_flags;   
            bvX.dp[n].quality_val   =  bath_v5->dp[n].quality_val  ;   
            bvX.dp[n].beam_number   =  n;   
        }
    }
    else if (bath_version == 7){
        bath_data_packet_v7_t* bath_v7 = (bath_data_packet_v7_t*) bath_in; /* Cast to v5/v6 packet, could use an union type here as well */
        bvX.sub_header.snd_velocity         = bath_v7->sub_header.snd_velocity;
        bvX.sub_header.sample_rate          = bath_v7->sub_header.sample_rate;
        bvX.sub_header.N                    = bath_v7->sub_header.N;
        bvX.sub_header.ping_number          = bath_v7->sub_header.ping_number;
        bvX.sub_header.time                 = bath_v7->sub_header.time;
        bvX.sub_header.time_net             = bath_v7->sub_header.time_net;
        bvX.sub_header.ping_rate            = bath_v7->sub_header.ping_rate;
        bvX.sub_header.type                 = bath_v7->sub_header.type;
        bvX.sub_header.flags                = bath_v7->sub_header.flags;
        bvX.sub_header.sonar_mode           = bath_v7->sub_header.sonar_mode;
        bvX.sub_header.time_uncertainty     = bath_v7->sub_header.time_uncertainty;
        bvX.sub_header.multiping_scan_number= bath_v7->sub_header.multiping_scan_number;
        bvX.sub_header.multifreq_band_number= bath_v7->sub_header.multifreq_band_number;
        bvX.sub_header.multiping_scan_index = bath_v7->sub_header.multiping_scan_index ;
        bvX.sub_header.multifreq_band_index = bath_v7->sub_header.multifreq_band_index ;
        bvX.sub_header.tx_angle             = bath_v7->sub_header.tx_angle;
        bvX.sub_header.gain                 = bath_v7->sub_header.gain;
        bvX.sub_header.tx_freq              = bath_v7->sub_header.tx_freq;
        bvX.sub_header.tx_bw                = bath_v7->sub_header.tx_bw;
        bvX.sub_header.tx_len               = bath_v7->sub_header.tx_len;
        bvX.sub_header.snd_velocity_raw     = bath_v7->sub_header.snd_velocity_raw;
        bvX.sub_header.tx_voltage           = bath_v7->sub_header.tx_voltage;
        bvX.sub_header.swath_dir            = bath_v7->sub_header.swath_dir;
        bvX.sub_header.swath_open           = bath_v7->sub_header.swath_open;
        bvX.sub_header.gate_tilt            = bath_v7->sub_header.gate_tilt;
        bvX.sub_header.intensity_noise_ref  = 0; 
        bvX.sub_header.strength_noise_ref   = 0;
        for (uint32_t n=0; n<bvX.sub_header.N;n++){
            bvX.dp[n].sample_number =  bath_v7->dp[n].sample_number;   
            bvX.dp[n].angle         =  bath_v7->dp[n].angle        ;   
            bvX.dp[n].steer_angle   =  0;   
            bvX.dp[n].upper_gate    =  bath_v7->dp[n].upper_gate   ;   
            bvX.dp[n].lower_gate    =  bath_v7->dp[n].lower_gate   ;   
            bvX.dp[n].intensity     =  bath_v7->dp[n].intensity    ;   
            bvX.dp[n].strength      =  0;   
            bvX.dp[n].flags         =  bath_v7->dp[n].flags        ;   
            bvX.dp[n].quality_flags =  bath_v7->dp[n].quality_flags;   
            bvX.dp[n].quality_val   =  bath_v7->dp[n].quality_val  ;   
            bvX.dp[n].beam_number   =  n;   
        }
    }
    else {//if (bath_version == 8){
        bath_data_packet_v8_t* bath_v8 = (bath_data_packet_v8_t*) bath_in; /* Cast to v5/v6 packet, could use an union type here as well */
        bvX.sub_header.snd_velocity         = bath_v8->sub_header.snd_velocity;
        bvX.sub_header.sample_rate          = bath_v8->sub_header.sample_rate;
        bvX.sub_header.N                    = bath_v8->sub_header.N;
        bvX.sub_header.ping_number          = bath_v8->sub_header.ping_number;
        bvX.sub_header.time                 = bath_v8->sub_header.time;
        bvX.sub_header.time_net             = bath_v8->sub_header.time_net;
        bvX.sub_header.ping_rate            = bath_v8->sub_header.ping_rate;
        bvX.sub_header.type                 = bath_v8->sub_header.type;
        bvX.sub_header.flags                = bath_v8->sub_header.flags;
        bvX.sub_header.sonar_mode           = bath_v8->sub_header.sonar_mode;
        bvX.sub_header.time_uncertainty     = bath_v8->sub_header.time_uncertainty;
        bvX.sub_header.multiping_scan_number= bath_v8->sub_header.multiping_scan_number;
        bvX.sub_header.multifreq_band_number= bath_v8->sub_header.multifreq_band_number;
        bvX.sub_header.multiping_scan_index = bath_v8->sub_header.multiping_scan_index ;
        bvX.sub_header.multifreq_band_index = bath_v8->sub_header.multifreq_band_index ;
        bvX.sub_header.tx_angle             = bath_v8->sub_header.tx_angle;
        bvX.sub_header.gain                 = bath_v8->sub_header.gain;
        bvX.sub_header.tx_freq              = bath_v8->sub_header.tx_freq;
        bvX.sub_header.tx_bw                = bath_v8->sub_header.tx_bw;
        bvX.sub_header.tx_len               = bath_v8->sub_header.tx_len;
        bvX.sub_header.snd_velocity_raw     = bath_v8->sub_header.snd_velocity_raw;
        bvX.sub_header.tx_voltage           = bath_v8->sub_header.tx_voltage;
        bvX.sub_header.swath_dir            = bath_v8->sub_header.swath_dir;
        bvX.sub_header.swath_open           = bath_v8->sub_header.swath_open;
        bvX.sub_header.gate_tilt            = bath_v8->sub_header.gate_tilt;
        bvX.sub_header.intensity_noise_ref  = bath_v8->sub_header.intensity_noise_ref;
        bvX.sub_header.strength_noise_ref   = bath_v8->sub_header.strength_noise_ref ; 
        for (uint32_t n=0; n<bvX.sub_header.N;n++){
            bvX.dp[n].sample_number =  bath_v8->dp[n].sample_number;   
            bvX.dp[n].angle         =  bath_v8->dp[n].angle        ;   
            bvX.dp[n].steer_angle   =  bath_v8->dp[n].steer_angle  ;   
            bvX.dp[n].upper_gate    =  bath_v8->dp[n].upper_gate   ;   
            bvX.dp[n].lower_gate    =  bath_v8->dp[n].lower_gate   ;   
            bvX.dp[n].intensity     =  bath_v8->dp[n].intensity    ;   
            bvX.dp[n].strength      =  bath_v8->dp[n].strength     ;   
            bvX.dp[n].flags         =  bath_v8->dp[n].flags        ;   
            bvX.dp[n].quality_flags =  bath_v8->dp[n].quality_flags;   
            bvX.dp[n].quality_val   =  bath_v8->dp[n].quality_val  ;   
            bvX.dp[n].beam_number   =  n;   
        }
    }
    return &bvX;
}


uint32_t wbms_georef_data( bath_data_packet_t* bath_in, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,/*OUTPUT*/ output_data_t* outbuf,/*INPUT*/ uint32_t force_bath_version){
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

    bath_data_packet_vX_t* bath_vX = bath_convert_to_universal(bath_in,force_bath_version);
    
    tx_freq = bath_vX->sub_header.tx_freq;
    tx_bw = bath_vX->sub_header.tx_bw;
    tx_plen = bath_vX->sub_header.tx_len;
    tx_angle = bath_vX->sub_header.tx_angle;
    tx_voltage = bath_vX->sub_header.tx_voltage;
    Fs = bath_vX->sub_header.sample_rate;
    c = bath_vX->sub_header.snd_velocity+sensor_params->sv_offset;
       
    //For CW pulses, bandwidth is given by pulse length
    tx_bw = MAX(tx_bw,1/tx_plen);
    
    if (sensor_params->force_sv > 0){
        c = sensor_params->force_sv;
    }
    if (c != c){
        fprintf(stderr, "NaN sound velocity encountered in data");
    }
    Nin = bath_vX->sub_header.N;
    multifreq_index =bath_vX->sub_header.multifreq_band_index;
    ping_number =  bath_vX->sub_header.ping_number;
    multiping_index =  bath_vX->sub_header.multiping_scan_index;
    ping_rate =  bath_vX->sub_header.ping_rate;

    if (0){//rcnt%100==0){
        char str_buf[256];
        sprintf_unix_time(str_buf, bath_in->sub_header.time);
        fprintf(stderr,"WBMS bathy ver=%d ping=%7d  multi freq=%2d  multi ping %2d tx=%5.1fdeg %s\n",
                bath_in->header.version,
                ping_number,
                multifreq_index,
                multiping_index,
                tx_angle*180/M_PI, 
                str_buf);
        }

    *sv_out = c; 
    *intensity_noise_ref_out =  bath_vX->sub_header.intensity_noise_ref;
    *strength_noise_ref_out =  bath_vX->sub_header.strength_noise_ref;

    *multiping_index_out = multiping_index;
    *multifreq_index_out = multifreq_index;
    *tx_freq_out = tx_freq;
    *tx_bw_out = tx_bw;
    *tx_plen_out = tx_plen;
    *tx_voltage_out = tx_voltage;
    *ping_number_out = ping_number;
    *fs_out = Fs;
    *ping_rate_out = ping_rate;
	float div_Fs = 1.f/Fs;
	float c_div_2Fs = c/(2*Fs);
    /*printf("ping_number=%9d, multiping= %d tx_angle=%5.1f multifreq_index=%d\n", 
    ping_number, multiping_index,tx_angle*180/M_PI, multifreq_index);*/
    //printf("tx_angle=%f, Fs=%f, c=%f, Nin=%d, multifreq_index=%d\n", tx_angle,  Fs,  c,  Nin,  multifreq_index);
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
    const uint8_t unsorted_data = 0;  //TODO set this flag when data is exported from a HDS mode system 
    
    if (calc_interpolated_nav_data( posdata, pos_ix, bath_vX->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
        if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",bath_vX->sub_header.time+sensor_offset->time_offset);
        return 0;
    }
    calc_interpolated_roll_and_z_vector(posdata, pos_ix, bath_vX->sub_header.time+sensor_offset->time_offset, (1.f/bath_vX->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
    if(sensor_params->calc_aoi && unsorted_data){
        qsort(bath_vX->dp, Nin, sizeof(detectionpoint_vX_t), cmp_wbms_vX_dp_on_angle_func);
    }

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
    
        float sample_number;
        if (sensor_params->sonar_sample_mode == upper_gate) 
            sample_number = bath_vX->dp[ix_in].upper_gate;
        else if (sensor_params->sonar_sample_mode == lower_gate) 
            sample_number = bath_vX->dp[ix_in].lower_gate;
        else
            sample_number = bath_vX->dp[ix_in].sample_number;

        sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
        sensor_ug = bath_vX->dp[ix_in].upper_gate*c_div_2Fs;
        sensor_lg = bath_vX->dp[ix_in].lower_gate*c_div_2Fs;
        #ifdef OUTPUT_QUALITY_VAL
        sensor_quality = (float)(bath_vX->dp[ix_in].quality_val);
        #endif
        sensor_strength = (float)(bath_vX->dp[ix_in].strength);
        sensor_t =  sample_number*div_Fs;		//Calculate tx to rx time for each point 
        sensor_az  = bath_vX->dp[ix_in].angle;
        sensor_elec_steer = bath_vX->dp[ix_in].steer_angle;
        sensor_quality_flags = bath_vX->dp[ix_in].quality_flags;
        flags = bath_vX->dp[ix_in].flags;



        priority_flags = ((flags)>>9) & (0x0F);
        sensor_r  += sensor_offset->r_err;
        sensor_ug  += sensor_offset->r_err;
        sensor_lg  += sensor_offset->r_err;


        //#define STX_GEOM_COR
        #ifdef STX_GEOM_COR
        sensor_az += calc_sonar_to_cp_corrections(sensor_az,sensor_el,sensor_elec_steer);
        #endif
       
        // Apply correctiom from beam corection polynom if defined
        if (sensor_params->beam_corr_poly_order){
            sensor_az = apply_beam_correction_poly(sensor_az, sensor_params->beam_corr_poly, sensor_params->beam_corr_poly_order);
        }
    
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
            
            beam_number[ix_out] = bath_vX->dp[ix_in].beam_number;
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
            quality_flags[ix_out] = sensor_quality_flags;
            classification_val[ix_out] = (sensor_quality_flags==3);  //Just calssify as Seafloor (=1) if Q=3 and Noise(=0) otherwise
            priority[ix_out] = (float) priority_flags;
            *tx_angle_out = sensor_el;
			

            intensity[ix_out] = bath_vX->dp[ix_in].intensity;

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


            //z_var[ix_out] =   sigma_aoi*sigma_aoi;
            //z_var[ix_out] =  sigma_z_pitch*sigma_z_pitch;

			ix_out++;
			
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
        beam_number[ix_out] = beam_number[ix_in];
        beam_steer[ix_out] = beam_steer[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
        upper_gate_range[ix_out] = upper_gate_range[ix_in];
        lower_gate_range[ix_out] = lower_gate_range[ix_in];
        strength[ix_out] = strength[ix_in];

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
        intensity[ix]  *= calc_intensity_range_scaling(beam_range[ix], sensor_params);
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


    
uint32_t wbms_georef_snippet_data( snippet_data_packet_t* snippet_in, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,/*OUTPUT*/ output_data_t* outbuf,/*INPUT*/ uint32_t force_bath_version){
    double* x = &(outbuf->x[0]);
    double* y = &(outbuf->y[0]);
    double* z = &(outbuf->z[0]);
    float* intensity = &(outbuf->i[0]);
    float* beam_range = &(outbuf->range[0]);
    float* beam_angle = &(outbuf->teta[0]);
    int * beam_number = &(outbuf->beam[0]);
    int * snp_len = &(outbuf->snp_len[0]);  
    float * footprint_time = &(outbuf->footprint_time[0]);  
    float* footprint_area = &(outbuf->footprint[0]);  
    float* aoi = &(outbuf->aoi[0]);
    float* fs_out = &(outbuf->sample_rate);
    float* ping_rate_out = &(outbuf->ping_rate);
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    float* tx_bw_out = &(outbuf->tx_bw);
    float* tx_plen_out = &(outbuf->tx_plen);
    float* tx_voltage_out = &(outbuf->tx_voltage);
    int* ping_number_out = &(outbuf->ping_number);
    int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);

    double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
    float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    float tx_angle; 
    float Fs;
    float gain_scaling;
    float ping_rate;
    float c;
    uint16_t Nin;
    uint32_t ping_number;
    uint16_t multiping_index;
    uint16_t multifreq_index;
    uint16_t Nout;
    uint16_t ix_out;
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


    tx_freq = snippet_in->sub_header.tx_freq;
    tx_bw = snippet_in->sub_header.tx_bw;
    tx_plen = snippet_in->sub_header.tx_len;
    tx_angle = snippet_in->sub_header.tx_angle;
    tx_voltage = snippet_in->sub_header.tx_voltage;
    Fs = snippet_in->sub_header.sample_rate;
    gain_scaling = 1./snippet_in->sub_header.gain;

    float vga_t0 = (float) snippet_in->sub_header.vga_t0;
    float vga_g0 = snippet_in->sub_header.vga_g0;
    float vga_t1 = (float) snippet_in->sub_header.vga_t1;
    float vga_g1 = snippet_in->sub_header.vga_g1;
    float vga_dgdt = (vga_g1-vga_g0)/(vga_t1-vga_t0);
    
    float eff_plen = MIN(tx_plen, 2./tx_bw);


    c = snippet_in->sub_header.snd_velocity+sensor_params->sv_offset;
    if (sensor_params->force_sv > 0){
        c = sensor_params->force_sv;
    }
    if (c != c){
        fprintf(stderr, "NaN sound velocity encountered in data");
    }

    Nin = snippet_in->sub_header.N;  //TODO first we try to just generate one sounding per snippet
    multifreq_index =snippet_in->sub_header.multifreq_band_index;
    ping_number =  snippet_in->sub_header.ping_number;
    multiping_index =  snippet_in->sub_header.multiping_scan_index;
    ping_rate =  snippet_in->sub_header.ping_rate;

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
    float div_Fs = 1.f/Fs;
    float c_div_2Fs = c/(2*Fs);
    /*printf("ping_number=%9d, multiping= %d tx_angle=%5.1f multifreq_index=%d\n", 
      ping_number, multiping_index,tx_angle*180/M_PI, multifreq_index);*/
    //printf("tx_angle=%f, Fs=%f, c=%f, Nin=%d, multifreq_index=%d\n", tx_angle,  Fs,  c,  Nin,  multifreq_index);
    tx_angle *= sensor_params->scale_tx_angle;

    //Skip whole dataset condition
    float sensor_el  = tx_angle;								 
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

    if (calc_interpolated_nav_data( posdata, pos_ix, snippet_in->sub_header.time+sensor_offset->time_offset,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){ 
        if(verbose) fprintf(stderr, "Could not find navigation data for WBMS bathy record at time %f\n",snippet_in->sub_header.time+sensor_offset->time_offset);
        return 0;
    }
    calc_interpolated_roll_and_z_vector(posdata, pos_ix, snippet_in->sub_header.time+sensor_offset->time_offset, (1.f/snippet_in->sub_header.ping_rate), ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);

    if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
        return 0;
    }

    size_t Nn = (Nin/ix_in_stride)+1;
    float *xs = malloc(Nn*sizeof(float));
    float *ys = malloc(Nn*sizeof(float));
    float *zs = malloc(Nn*sizeof(float));



    //Calculate sounding positions in sonar reference frame at tx instant
    ix_out = 0;
    //printf("Nin=%d\n",Nin);

    float *snippet_start_ix     =  (float*)     (snippet_in->payload+(0*4*Nin));
    float *snippet_stop_ix      =  (float*)     (snippet_in->payload+(1*4*Nin));
    float *snippet_detection_ix =  (float*)     (snippet_in->payload+(2*4*Nin));
    float *snippet_angle        =  (float*)     (snippet_in->payload+(3*4*Nin));
    //uint32_t *sippet_tbd      =  (float*)     (snippet_in->payload+(4*4*Nin));
    uint16_t *snippet_intensity =  (uint16_t*)  (snippet_in->payload+(5*4*Nin));

    //Index location of each snippet data section, This needs to be done on all, undecimated data as it is cumulative
    int32_t snippet_intensity_offset[MAX_DP]; 
    int32_t snippet_length[MAX_DP];

    int32_t snp_acum_len = 0;                              //Snippet intensity  start index, increments as we progress through all snippets
    for (uint16_t ix_in=0;ix_in<Nin;ix_in++){
        int32_t snippet_len = (int32_t) (roundf(snippet_stop_ix[ix_in] - snippet_start_ix[ix_in]))+1;
        snippet_length[ix_in] = snippet_len;
        snippet_intensity_offset[ix_in] = snp_acum_len;
        snp_acum_len += snippet_len;
    }

    for (uint16_t ix_in=0;ix_in<Nin;ix_in++){
        int32_t snippet_len = snippet_length[ix_in];
        if (ix_in%ix_in_stride==0 && snippet_len>1){
            float sample_number = snippet_detection_ix[ix_in];
            float sensor_r   = sample_number*c_div_2Fs;	//Calculate range to each point en meters
            float sensor_t =  sample_number*div_Fs;		//Calculate tx to rx time for each point 
            float sensor_az  = snippet_angle[ix_in];

            sensor_r  += sensor_offset->r_err;

            // Apply correctiom from beam corection polynom if defined
            if (sensor_params->beam_corr_poly_order){
                sensor_az = apply_beam_correction_poly(sensor_az, sensor_params->beam_corr_poly, sensor_params->beam_corr_poly_order);
            }

#ifdef SHALLOW_ANGLE_SKEW_COR
            sensor_az += calc_shallow_angle_skew_corrections(sensor_az,sensor_r, sensor_params->intensity_range_attenuation);
#endif

            // Add correction for roll during tx2rx period for each beam individually
            sensor_az_tx2rx_corr = -roll_vector[(size_t) round(sensor_t*ROLL_VECTOR_RATE)]; //Roll is given in opposite angles than sonar azimuth
            sensor_z_tx2rx_corr = z_vector[(size_t) round((sensor_t/2)*ROLL_VECTOR_RATE)]; // Z correction is for half tx to rx time
            //printf("sonar_min_quality_flag=%d, sensor_params->max_quality_flag=%d, sensor_params->min_priority_flag=%d, sensor_params->max_priority_flag=%d\n",sensor_params->min_quality_flag,sensor_params->max_quality_flag,sensor_params->min_priority_flag,sensor_params->max_priority_flag);
            if (	(sensor_az > sensor_params->min_azimuth) && (sensor_az < sensor_params->max_azimuth) &&
                    (sensor_r > sensor_params->min_range) && (sensor_r < sensor_params->max_range) 
               ){

                beam_number[ix_out] = ix_in;
                beam_angle[ix_out] =  sensor_az;  //Store raw beam angle from sonar for data analysis
                beam_range[ix_out] = sensor_r;


                /***** Converting sensor data from spherical to kartesian coordinates *********/
                // Projection assumes cone-plane coordinates (See assembla ticket #1725)								 
                // Standard sonar mounting az=0 el=0 => Nadir(0,0,1), az=90 el=0 => Starbord(0,1,0), az=0 el=90 => Forward(1,0,0) 
                // Sign of y axis is flipped (compared to #1725) because az rotation is left to right, and not right to left which it should be if it was a positive rotation around x-axis
                xs[ix_out] = sensor_r * sinf(sensor_el);
                ys[ix_out] = sensor_r * sinf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el); //Sign flipped compared to standard right hand system
                zs[ix_out] = sensor_r * cosf((sensor_az+sensor_az_tx2rx_corr))*cosf(sensor_el);
                zs[ix_out] += sensor_z_tx2rx_corr;
                
                ix_out++;
            }
        }
    }
    Nout = ix_out;

    georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);


#if 1
    //Post GEO-REF filtering
    Nin = Nout;
    ix_out = 0;
    for (uint16_t ix_in=0;ix_in<Nin;ix_in++){
        x[ix_out] = x[ix_in];
        y[ix_out] = y[ix_in];
        z[ix_out] = z[ix_in];
        beam_angle[ix_out] = beam_angle[ix_in];
        beam_number[ix_out] = beam_number[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
        
        if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;

        ix_out++;
    }
    Nout = ix_out;
#endif 
    //printf("Nout2 = %d\n",Nout);
    //Calculate AOI on  Post-filtered data
    if (sensor_params->calc_aoi){
        calc_aoi(beam_range, beam_angle, Nout, /*output*/ aoi);
    }
    else{
        for (ix_out=0;ix_out<Nout;ix_out++){
            aoi[ix_out] = beam_angle[ix_out];
        }
    }
                
	for (ix_out=0;ix_out<Nout;ix_out++){
        //Calculate time domain footprint for each beam / sounding 
        footprint_time[ix_out] = calc_beam_time_response(beam_range[ix_out], aoi[ix_out], beam_angle[ix_out],eff_plen , sensor_params);
	}
    

    //Calculate corrected intensity on  Post-filtered data
	// Populate r,az,el and t with data from bath data
	for (ix_out=0;ix_out<Nout;ix_out++){
        uint16_t ix_in = beam_number[ix_out]; //This is the index used in the original dataset, used to index: snippet_start_ix,snippet_stop_ix,snippet_detection_ix,snippet_angle,snippet_intensity,snippet_intensity_offset,snippet_length
        float sample_number = snippet_detection_ix[ix_in];
        int32_t detection_offset = (int32_t)(roundf(sample_number-snippet_start_ix[ix_in])); //Index of detection point in snippet

        /*** Calculate intensity from snippet data ***/
        float inten;
        float acum_pow;
        switch (sensor_params->snippet_processing_mode){
            case snippet_detection_value:
                {
                    //Take intensity from detection sample
                    inten = snippet_intensity[snippet_intensity_offset[ix_in]+detection_offset];       //Pick detection sample from snippet data
                }
                break;
            default:
            case snippet_mean_pow:
                //Mean snippet power  (root-mean-square)
                acum_pow = 0;
                for (size_t ix = 0; ix<snippet_length[ix_in];ix++){
                    acum_pow += powf((float)snippet_intensity[snippet_intensity_offset[ix_in]+ix],2);
                }
                inten = sqrtf(acum_pow/snippet_length[ix_in]);
                break;
            case snippet_sum_pow:
                acum_pow = 0;
                for (size_t ix = 0; ix<snippet_length[ix_in];ix++){
                    acum_pow += powf((float)snippet_intensity[snippet_intensity_offset[ix_in]+ix],2);
                }
                inten = sqrtf(acum_pow);
                break;
            case snippet_3dB_footprint_mean_pow:
                {
                    #if 1
                    // Crop snippet to maximum the 3dB time domain footprint
                    int32_t snippet_3dB_length=roundf(footprint_time[ix_out]*Fs);
                    snippet_3dB_length=MAX(snippet_3dB_length,1);
                    int32_t ix0 = detection_offset-snippet_3dB_length/2;
                    int32_t ix1 = ix0+snippet_3dB_length;
                    ix0=MAX(0,ix0);
                    ix1=MIN(ix1,snippet_length[ix_in]);
                    acum_pow = 0;
                    for (size_t ix = ix0; ix<ix1;ix++){
                        inten = (float)snippet_intensity[snippet_intensity_offset[ix_in]+ix];
                        acum_pow += powf(inten,2);
                    }
                    inten = sqrtf(acum_pow/(ix1-ix0));
                    snippet_length[ix_in]=ix1-ix0;    


                    #else
                    int32_t detection_offset = (int32_t)(roundf(sample_number-snippet_start_ix[ix_in]));
                    float peak_sig = snippet_intensity[snippet_intensity_offset[ix_in]+detection_offset];       //Pick detection sample from snippet data
                    /*float peak_sig = 0;
                      for (size_t ix = 0; ix<snippet_length[ix_in];ix++){
                      peak_sig = MAX(peak_sig,(float)snippet_intensity[snippet_intensity_offset[ix_in]+ix]);
                      }*/
                    float th = peak_sig*0.7071f;

                    acum_pow = 0;
                    for (size_t ix = 0; ix<snippet_length[ix_in];ix++){
                        inten = (float)snippet_intensity[snippet_intensity_offset[ix_in]+ix];
                        if (inten>=th){
                            acum_pow += powf(inten,2);
                            snippet_3dB_length++;
                        }
                    }
                    inten = sqrtf(acum_pow/snippet_3dB_length);
                    #endif
                }
                break;
        }


        // Correct intesity for internal VGA
        float vga_gain_dB;
        if (sample_number <= vga_t0){
            vga_gain_dB = vga_g0;
        }
        else if (sample_number < vga_t1){
            vga_gain_dB = vga_g0 + vga_dgdt*(sample_number-vga_t0);
        }
        else{
            vga_gain_dB = vga_g1;
        }
        float vga_gain_scaling = powf(10,-vga_gain_dB/20);
        inten *= gain_scaling * vga_gain_scaling;
        
        //Compensate intensity for range and AOI
        inten *= calc_intensity_range_scaling(beam_range[ix_out], sensor_params);
        
        if (sensor_params->snippet_processing_mode == snippet_sum_pow){
            // When calculating total snippet energy, the footprint is based on the entire footprint of the snippet, not a sample footprint
            inten *= calc_footprint_scaling(beam_range[ix_out], aoi[ix_out],  beam_angle[ix_out],eff_plen+snp_len[ix_out]*div_Fs , sensor_params, /*OUTPUT*/ &(footprint_area[ix_out]));
        }
        else{
            inten  *= calc_footprint_scaling(beam_range[ix_out], aoi[ix_out],  beam_angle[ix_out],eff_plen , sensor_params, /*OUTPUT*/ &(footprint_area[ix_out]));
        }
        
        inten  *= calc_ara_scaling(aoi[ix_out], sensor_params);
    
        // Write results to output buffers
        intensity[ix_out] = inten;
        snp_len[ix_out] = snippet_length[ix_in];
	}

    free(xs);free(ys);free(zs);
    outbuf->N = Nout;
    return Nout;
}



uint32_t wbms_georef_sbp_data( sbp_data_packet_t* sbp_data, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,/*OUTPUT*/ output_data_t* outbuf){
     double* x = &(outbuf->x[0]);
     double* y = &(outbuf->y[0]);
     double* z = &(outbuf->z[0]);
     float* intensity = &(outbuf->i[0]);
     float* beam_range = &(outbuf->range[0]);

     float* beam_angle = &(outbuf->teta[0]);
     float* beam_steer = &(outbuf->steer[0]);
     int * beam_number = &(outbuf->beam[0]);

     float* tx_freq_out = &(outbuf->tx_freq);
     float* tx_bw_out = &(outbuf->tx_bw);
     float* tx_plen_out = &(outbuf->tx_plen);
     float* tx_voltage_out = &(outbuf->tx_voltage);
     float* fs_out = &(outbuf->sample_rate);
     float* ping_rate_out = &(outbuf->ping_rate);
     int* ping_number_out = &(outbuf->ping_number);
     float* sv_out = &(outbuf->sv);
     float* tx_angle_out = &(outbuf->tx_angle);

	float sensor_r;

	double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
    float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
    //uint32_t sbp_data_version =  sbp_data->header.version;
	float Fs;
	float c;
	uint16_t Nin;
    uint32_t ping_number;
	int32_t ix_in,ix_out;
    uint16_t ix_in_stride = MAX(1,sensor_params->beam_decimate); 
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
    *tx_angle_out = 0;
    *ping_number_out = ping_number;
    *fs_out = Fs;
    *ping_rate_out = sbp_data->sub_header.ping_rate;
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
    
    if (sensor_params->sbp_motion_stab){
        nav_pitch=0;
        nav_roll=0;
    }

    float *sig = malloc(Nin*sizeof(float));
    float *temp_sig = malloc(Nin*sizeof(float));
    if ( (sig==NULL)||(temp_sig==NULL) ) return 0;

    //Apply additional post-processing band pass filter
    if((sensor_params->sbp_bp_filter_start_freq>0) ||  (sensor_params->sbp_bp_filter_stop_freq>0)){
        float f0 = sensor_params->sbp_bp_filter_start_freq*1e3;
        float f1 = sensor_params->sbp_bp_filter_stop_freq*1e3;
        f1 = f1==0?30e3:f1;
        bp_filter_data(/*Input*/ temp_sig, (f0+f1)/2, f1-f0, Fs ,Nin, /*Output*/ sig);
        memcpy(temp_sig, sig,Nin*sizeof(float));
    } 

    //printf("SBP data tp = %d\n",sbp_data->sub_header.tp);
    if (sensor_params->sbp_raw_data){
        memcpy(sig, temp_sig,Nin*sizeof(float));
    }
    else{
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
    }

    //Calculate sounding positions in sonar reference frame at tx instant
	ix_out = 0;
    //printf("Nin=%d\n",Nin);
    float tx_delay_range = tx_plen*c/2;

    int32_t ix_start = (int32_t) (roundf((sensor_params->min_range+tx_delay_range)/c_div_2Fs));
    int32_t ix_stop  = (int32_t) (roundf((sensor_params->max_range+tx_delay_range)/c_div_2Fs));
    ix_start = LIMIT(ix_start,0,Nin); 
    ix_stop = LIMIT(ix_stop,ix_start,Nin); 

    //Calculate number of output datapoints, and allocate memory for them
    ix_in_stride = MAX(ix_in_stride,((ix_stop-ix_start)/(4*1024)+1)); //Force decimation to limit to maximum 4k points per ping
    //fprintf(stderr,"ix_start=%d   ix_stop=%d   ix_in_stride=%d\n",ix_start,ix_stop,ix_in_stride);
    uint32_t Nout = ((ix_stop-ix_start)/ix_in_stride)+1;
    float *xs  = malloc(Nout*sizeof(float));
    float *ys  = malloc(Nout*sizeof(float));
    float *zs  = malloc(Nout*sizeof(float));
    if ( (xs==NULL)||(ys==NULL)||(zs==NULL) ) return 0;

	for (ix_in=ix_start;ix_in<ix_stop;ix_in+=ix_in_stride){
            
	    float inten;
        uint32_t sample_number;
        sample_number = ix_in;
        
        sensor_r   = sample_number*c_div_2Fs - tx_delay_range;	//Calculate range to each point en meters
        inten = sig[ix_in];
        sensor_r  += sensor_offset->r_err;
        
        if (sensor_params->intensity_correction){
            inten *= sensor_r;                  //Only comp one-way spreading loss     
            float damping_dB = sensor_params->intensity_range_attenuation * (2*sensor_r/1000); 
            inten *= powf(10.f,damping_dB/20); 
        }

        intensity[ix_out] = inten;
        beam_range[ix_out] = sensor_r;
        beam_angle[ix_out] = 0;
        beam_steer[ix_out] = 0;
        beam_number[ix_out] = sample_number;
        // Sub bottom profiler is always pointing straight down
        xs[ix_out] = 0.; 
        ys[ix_out] = 0.; 
        zs[ix_out] = sensor_r;
        ix_out++;
    }
    Nout = ix_out;
    //fprintf(stderr,"Nout=%d\n",Nout);

    georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
	
	//Post GEO-REF filtering
	Nin = Nout;
	ix_out = 0;
	for (ix_in=0;ix_in<Nin;ix_in++){
		x[ix_out] = x[ix_in];
		y[ix_out] = y[ix_in];
		z[ix_out] = z[ix_in];
		intensity[ix_out] = intensity[ix_in];
		beam_angle[ix_out] = beam_angle[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
        beam_number[ix_out] = beam_number[ix_in];
        beam_steer[ix_out] = beam_steer[ix_in];
        beam_range[ix_out] = beam_range[ix_in];
        *fs_out = Fs;
        *ping_number_out = ping_number;
		if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;

		ix_out++;
	}
	Nout = ix_out;
	
    free(xs);free(ys);free(zs);free(sig);free(temp_sig);
    outbuf->N = Nout;
	return Nout;
}
