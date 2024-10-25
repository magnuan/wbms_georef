#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
#ifdef __unix__
#include <unistd.h>
#endif
#include <time.h>

#include <string.h>
#include <stdint.h>
#include "reson7k.h"
#include "time_functions.h"

#include <math.h>
#include "wbms_georef.h"
#include "georef_tools.h"
#include "intensity_scaling.h"
#include "cmath.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

static uint8_t verbose = 0;

static offset_t* sensor_offset;

#ifdef COUNT_S7K_SNIPPET_SATURATION
extern uint32_t s7k_snp_satcount;
extern uint32_t s7k_snp_count;
#endif

#ifdef COLLECT_S7K_STATS
static uint32_t s7k_stat_packet_count[S7K_ID_MAX+1];
#endif

void r7k_init(void){
    #ifdef COLLECT_S7K_STATS
    for (uint32_t id=0;id<S7K_ID_MAX;id++){
        s7k_stat_packet_count[id]=0;
    }
    #endif
}

void r7k_print_stats(void){
    #ifdef COLLECT_S7K_STATS
    uint32_t total_data = 0;
    for (uint32_t id=0;id<S7K_ID_MAX;id++){
        total_data+=s7k_stat_packet_count[id];
    }
    if (total_data){
        fprintf(stderr,"------- Reson S7K data packet count -----\n");
    }
    for (uint32_t id=0;id<S7K_ID_MAX;id++){
        if(s7k_stat_packet_count[id]>0){
            fprintf(stderr, "S7K record type %5d: %8d\n",id,s7k_stat_packet_count[id]);
        }
    }
    if (total_data){
        fprintf(stderr,"\n");
    }
    #endif
}

void r7k_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}


uint8_t r7k_test_file(int fd,int req_types[], size_t n_req_types){
    uint8_t pass=0;
    char* data = malloc(MAX_S7K_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<1000000;test++){     //Test the first 1000 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = r7k_fetch_next_packet(data, fd);
        //printf("r7k_test_file, %d len=%d test_cnt=%d\n",req_types[0],len,test);
        if (len > 0 ){
            double ts;
            int type = r7k_identify_sensor_packet(data, len, &ts);
            //fprintf(stderr,"type = %d\n",type);fflush(stderr);
            for (size_t req_type_ix = 0; req_type_ix<n_req_types;req_type_ix++){
                if (type==req_types[req_type_ix]){
                    pass=1;
                    break;
                }
            }
            if (pass) break;
        }
        else{
            break;
        }
    }
    free(data);
    return pass;
}

uint8_t r7k_test_nav_file(int fd){
    int req_types[] = {1003,1012,1013,1015,1016};
    return r7k_test_file(fd,req_types,2);
}
uint8_t r7k_test_bathy_file(int fd){
    int req_types[] = {7027,10018};
    return r7k_test_file(fd,req_types,2);
}

void r7k_calc_checksum(r7k_DataRecordFrame_t* drf){
	uint32_t len,crc;
	uint8_t* d;
	crc = 0;
	len = drf->size-4;
	d = (uint8_t*)drf;
	while (len--)
	       crc+=(*(d++));
	*((uint32_t*)d) = crc;
}

void r7k_ts_to_r7ktime(double ts,r7k_Time_t* t){
	gm_to_irigb(ts, &(t->year), &(t->day), &(t->hour), &(t->min), &(t->sec));
}

double r7k_r7ktime_to_ts(r7k_Time_t* t){
	return irigb_to_gm((t->year), (t->day), (t->hour), (t->min), (t->sec));
}


int r7k_seek_next_header(int fd, /*out*/ uint8_t* pre_sync){
	char state = 0;
	uint8_t v;
	int n;
	int dump= 0;
	uint8_t pre_sync_buffer[8]; //Reson puts 4 bytes of header data BEFORE sync pattern (screw you Reson for that)
	uint8_t psb_ix = 0;
    int read_bytes = 0;
	while (read_bytes<(MAX_S7K_PACKET_SIZE+4)){
		n = read(fd,&v,1);                              //TODO: Speed this up, it takes too long time to read one and one byte
        read_bytes++;
		if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
		if(n==0){ fprintf(stderr,"End of S7K stream\n");return -1;}
		if(n>0){
			pre_sync_buffer[(psb_ix++)%8] = v;
			dump += 1;
			switch (state){
				case 0: state = (v==0xff)?1:0;break;
				case 1: state = (v==0xff)?2:0;break;
				case 2: state = (v==0x00)?3:0;break;
				case 3: state = (v==0x00)?4:0;break;
			}
			if (state==4){
				dump-=8;
				if (pre_sync){
					for (uint8_t ii= 0;ii<8;ii++){
						pre_sync[ii] = pre_sync_buffer[(psb_ix++)%8];
					}
				}
				if(dump>2) fprintf(stderr,"S7K seek dump %d bytes\n",dump);
				return 0;	
			}
		}
	}
	return -1;
}
#include <errno.h>


int r7k_fetch_next_packet(char * data, int fd){
	size_t rem;
    ssize_t n;
	char * dp;
	if(r7k_seek_next_header(fd,(uint8_t*)data)) return 0;
	dp = &(data[8]);                                //Read in 8 bytes allready 4-bytes before sync and 4 bytes sync
	n = read(fd, dp,4);dp +=4;                        //Read in four more bytes, containing packet size
	uint32_t s7k_size = ((uint32_t*) data)[2];
	
    
	rem = s7k_size-12;                                //Remaining bytes - the 12 we allready have      
	while (rem>0){ 
        n= read(fd,dp,rem);		
        if(n<0){ fprintf(stderr,"Got error from socket 2. Errno=%d  %s  data=0x%08lx  dp=0x%08lx\n",errno, strerror(errno), (uint64_t) data, (uint64_t) dp);return 0;}
        if(n==0){ fprintf(stderr,"End of POS_MODE_S7K stream\n");return 0;}
        rem -= n; dp+=n;

	}
    #ifdef COLLECT_S7K_STATS
    uint32_t s7k_record_id = ((uint32_t*) data)[8];
    if(s7k_record_id<S7K_ID_MAX){
        s7k_stat_packet_count[s7k_record_id]++;
    }
    #endif

    #if 0
    uint16_t s7k_version = ((uint16_t*) data)[0];
	uint16_t s7k_offset = ((uint16_t*) data)[1];
    uint32_t s7k_record_id = ((uint32_t*) data)[8];
	fprintf(stderr, "FETCH: S7K ver = %d, S7K off = %d,  size = %d recordID = %d\n",s7k_version,s7k_offset, s7k_size, s7k_record_id);
    #endif
	
    return s7k_size;
}

int r7k_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out){
    
	r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
    
	//So far we only process s7k record 7000, 7027,7028,10000,10018  and 7610  
    // we dont care about time stamps in any other records as they could come out ot order (like 7030)
    if ((drf->record_id == 7000) || (drf->record_id == 7027) || (drf->record_id == 7028)|| (drf->record_id == 7058)|| (drf->record_id == 7610)|| (drf->record_id == 10000)|| (drf->record_id == 10018)){
        double ts = r7k_r7ktime_to_ts(&(drf->time));
        // If record data is from sensor (sonar) add sensor time offset
        if ( drf->record_id >= 7000  && drf->record_id < 11000){
            ts += sensor_offset->time_offset;
        }
        *ts_out = ts;
    }
	//union r7k_RecordTypeHeader rth;
	//rth.dummy = (r7k_RecordTypeHeader_dummy_t*) (databuffer+4+(drf->offset));
    //fprintf(stderr,"IDENTIFY: len:%d S7K record  id:%d dev:%d  Y=%d doy=%d  %02d:%02d:%09.6f   ts = %f\n",len, drf->record_id,drf->dev_id,drf->time.year,drf->time.day,drf->time.hour,drf->time.min,drf->time.sec,ts);
    return drf->record_id;
}


/**
*
* This function is assuming that position and attitude data comes synched, with same time stamp. As from an integrated navigation system
* For more generic s7k streams, where attitude and position messages might come from different sources, at different time and rate. This will not work
*/
int s7k_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
	static double last_lon, last_lat;
	static float last_alt, last_heave;
    static uint8_t has_1015;
    static uint8_t has_1016;
    
    static navdata_t navdata_collector; 
	static uint8_t have_pos; 
	static uint8_t have_attitude; 
	static uint8_t have_heading; 

    r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
	
	//So far we process s7k record 1003, 1012, 1013, 1015 and 1016 for navigation  (we dont care about time stamps in any other records)
	if ((drf->record_id != 1015) && (drf->record_id != 1016) && (drf->record_id != 1003) && (drf->record_id != 1012) && (drf->record_id != 1013))
		return NO_NAV_DATA;	
	
    double ts = r7k_r7ktime_to_ts(&(drf->time));
	*ts_out = ts;

    //fprintf(stderr,"S7K record  id:%d dev:%d  Y=%d doy=%d  %02d:%02d:%09.6f   ts = %f\n",drf->record_id,drf->dev_id,drf->time.year,drf->time.day,drf->time.hour,drf->time.min,drf->time.sec,ts);
	//fprintf(stderr, "%d\n",drf->offset);	
	union r7k_RecordTypeHeader rth;
	rth.dummy = (r7k_RecordTypeHeader_dummy_t*) (databuffer+4+(drf->offset));
	

    // The data collector needs to collect one set of posision and one set of attitude data with same timestamp
    // to generate a navigation out entry. 
    if (ts != navdata_collector.ts){ 	// If this data has a new timestamp
        navdata_collector.ts = ts;
        //New entry, still no navigation data 
	    if ((have_pos != have_attitude) || (have_pos != have_heading)){
            if(verbose) fprintf(stderr,"Dumping incomplete nav dataset\n");
        }
        have_pos=0;
        have_attitude=0;	
        have_heading=0;
    }


	switch (drf->record_id){
		case 1003: // Position
            if (has_1015) break;  //Prefer 1015, do not process this record if 1015 records are available
			if (verbose > 2) fprintf(stderr,"ts=%f s7k:1003 Datum=%d Latency=%f Lat=%f Lon=%f Height=%f PosType=%d, UTMZone=%d, Qual=%d Method=%d\n", \
                                ts, \
							    rth.r1003->datum,rth.r1003->latency,rth.r1003->lat_northing*180/M_PI, rth.r1003->lon_easting*180/M_PI, rth.r1003->height,  \
							    rth.r1003->pos_type, rth.r1003->utm_zone, rth.r1003->quality, rth.r1003->pos_method);
			if (rth.r1003->pos_type != 0){
                fprintf(stderr,"Not supporting Grid coordinates in s7k record 1003 yet\n");
                break;
            }
            navdata_collector.lon = rth.r1003->lon_easting;
			navdata_collector.lat = rth.r1003->lat_northing;
			last_lon = rth.r1003->lon_easting;
			last_lat = rth.r1003->lat_northing;
			navdata_collector.alt = rth.r1003->height;
			last_alt =  rth.r1003->height;
			aux_navdata->vert_accuracy = 0;
			aux_navdata->hor_accuracy = 0;

			if (proj){
                double alt;
				switch (alt_mode){ 
					case 1:  alt = last_alt; break; //GPS altitude from posmv is positive up (When having RTK, we typically want to use GPS altitude instread of heave+tide)
					case 2:  alt = last_heave; break; //heave from s7k is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
					default: alt = 0;
				}
                double z;
                latlon_to_kart(navdata_collector.lon, navdata_collector.lat , alt, proj, /*output*/ &(navdata_collector.x), &(navdata_collector.y) , &z);
				if (alt_mode ==1){ 
					navdata_collector.z = z; 
				}
			}
			have_pos = 1;
			break;
		case 1012:	// Roll Pitch Heave
            if (has_1016) break;  //Prefer 1016, do not process this record if 1016 records are available
			if (verbose >2 )fprintf(stderr,"ts=%f s8k:1012 roll=%f pitch=%f heave=%f\n",ts,rth.r1012->roll*180/M_PI,rth.r1012->pitch*180/M_PI, rth.r1012->heave);
			navdata_collector.roll = rth.r1012->roll;
			navdata_collector.pitch = rth.r1012->pitch;
			navdata_collector.heave = rth.r1012->heave;

            last_heave = navdata_collector.heave;
			if (alt_mode ==2){ 
				navdata_collector.z = -navdata_collector.heave;  //heave from posmv is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
			}
			have_attitude = 1;
			break;
		case 1013:	// Heading
            if (has_1016) break;  //Prefer 1016, do not process this record if 1016 records are available
			if (verbose > 2) fprintf(stderr,"ts=%f s7k:1013 heading=%f\n", ts, rth.r1013->heading*180/M_PI);
			navdata_collector.yaw = rth.r1013->heading;
			if (proj){
				navdata_collector.yaw += north_to_northing_offset(last_lon, last_lat, proj);
			}
			have_heading = 1;
			break;


		case 1015:	// Navigation
            has_1015 = 1;
			if (verbose>2 ) fprintf(stderr,"ts=%f s7k:1015 Vert_ref=%d Lat=%f Lon=%f Height=%f Accuracy=%f, %f, SoG=%f, Cog=%f Heading=%f\n", \
                    ts, \
					rth.r1015->vert_ref,rth.r1015->lat*180/M_PI, rth.r1015->lon*180/M_PI, rth.r1015->height,  \
					rth.r1015->hor_accuracy, rth.r1015->vert_accuracy, rth.r1015->speed_over_ground, rth.r1015->course_over_ground*180/M_PI, rth.r1015->heading*180/M_PI);

			navdata_collector.lon = rth.r1015->lon;
			navdata_collector.lat = rth.r1015->lat;
			last_lon = rth.r1015->lon;
			last_lat = rth.r1015->lat;
			navdata_collector.alt = rth.r1015->height;
			last_alt =  rth.r1015->height;
			aux_navdata->vert_accuracy = rth.r1015->vert_accuracy;
			aux_navdata->hor_accuracy = rth.r1015->hor_accuracy;

			if (proj){
                double alt;
				switch (alt_mode){ 
					case 1:  alt = last_alt; break; //GPS altitude from posmv is positive up (When having RTK, we typically want to use GPS altitude instread of heave+tide)
					case 2:  alt = last_heave; break; //heave from s7k is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
					default: alt = 0;
				}
                double z;
                latlon_to_kart(navdata_collector.lon, navdata_collector.lat , alt, proj, /*output*/ &(navdata_collector.x), &(navdata_collector.y) , &z);
				if (alt_mode ==1){ 
					navdata_collector.z = z; 
				}
			}
			have_pos = 1;
			break;
		case 1016:	// Attitude
            has_1016=1;
			if (verbose>2 ) fprintf(stderr,"ts=%f s7k:1016 t_off=%dms roll=%f pitch=%f heading=%f heave=%f\n", \
            ts, \
			rth.r1016->entry[0].t_off_ms,rth.r1016->entry[0].roll*180/M_PI,rth.r1016->entry[0].pitch*180/M_PI, rth.r1016->entry[0].heading*180/M_PI, rth.r1016->entry[0].heave);
			
			navdata_collector.roll = rth.r1016->entry[0].roll;
			navdata_collector.pitch = rth.r1016->entry[0].pitch;
			navdata_collector.yaw = rth.r1016->entry[0].heading;
			navdata_collector.heave = rth.r1016->entry[0].heave;
          

            last_heave = navdata_collector.heave;
			
			if (alt_mode ==2){ 
				navdata_collector.z = -navdata_collector.heave;  //heave from posmv is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
			}

			if (proj){
				navdata_collector.yaw += north_to_northing_offset(last_lon, last_lat, proj);
			}
			have_attitude = 1;
			have_heading = 1;
			break;
	}

	if (have_pos && have_attitude && have_heading){
        memcpy(navdata, &navdata_collector, sizeof(navdata_t));
        have_pos=0;
        have_attitude=0;	
        have_heading=0;	
        //fprintf(stderr,"Complete nav data at ts=%f\n",ts);
        return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    return NO_NAV_DATA;
}




uint32_t s7k_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,  /*OUTPUT*/ output_data_t* outbuf){

    static char* backlog_databuffer = NULL;
    static uint32_t backlog_databuffer_len = 0;
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
    float* tx_angle_out = &(outbuf->tx_angle);
    float* fs_out = &(outbuf->sample_rate);
    float* ping_rate_out = &(outbuf->ping_rate);
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    float* tx_plen_out = &(outbuf->tx_plen);
    float* tx_bw_out = &(outbuf->tx_bw);
    float* tx_voltage_out = &(outbuf->tx_voltage);
    int* ping_number_out = &(outbuf->ping_number);
    //int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);
    int* classification_val = &(outbuf->classification[0]);
    
    uint16_t ix_in_stride = MAX(1,sensor_params->beam_decimate); 
    const uint32_t ping_number_stride = sensor_params->ping_decimate; 

    static float sv;
    static float mbes_tx_freq;
    static float mbes_tx_bw;
    static float mbes_tx_plen;
    static float mbes_tx_power;
    static float mbes_ping_rate;
    static float mbes_gain;
    static float mbes_absorbtion;
    static float mbes_spread_loss;
    static float mbes_eff_plen;
    
    static float sbes_tx_freq;
    static float sbes_tx_bw;
    static float sbes_tx_plen;
    static float sbes_ping_rate;

	r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
 	double ts = r7k_r7ktime_to_ts(&(drf->time));
    ts += sensor_offset->time_offset;
	union r7k_RecordTypeHeader rth;
	rth.dummy = (r7k_RecordTypeHeader_dummy_t*) (databuffer+4+(drf->offset));
    

	//So far we only process s7k record 7000, 7027,7028,7058,10000,10018  and 7610
    //fprintf(stderr,"S7K record  id:%d dev:%d  Y=%d doy=%d  %02d:%02d:%09.6f   ts = %f\n",drf->record_id,drf->dev_id,drf->time.year,drf->time.day,drf->time.hour,drf->time.min,drf->time.sec,ts);
    if (drf->record_id == 7610){
       sv = rth.r7610->sv ;
       sv = sv + sensor_params->sv_offset;
       //fprintf(stderr, "Read in sound velocity from S7K 7610  sv=%f\n", sv);
       return 0;
    }
    if (sensor_params->force_sv > 0){
        sv = sensor_params->force_sv;
    }
    if (sv != sv){
        fprintf(stderr, "NaN sound velocity encountered in data");
    }

    *sv_out = sv;
    
    // --- Process S7K 7000 record ----
    if (drf->record_id == 7000){

       #define PRINT_S7K_7000_CHANGES
       #ifdef PRINT_S7K_7000_CHANGES
       uint8_t new_param = mbes_tx_freq != rth.r7000->tx_freq ||\
                           mbes_tx_bw != rth.r7000->bw ||\
                           mbes_tx_plen != rth.r7000->tx_len ||\
                           /*mbes_ping_rate != 1./(rth.r7000->ping_period) ||\*/
                           mbes_gain != rth.r7000->gain ||\
                           mbes_tx_power != rth.r7000->tx_power ||\
                           mbes_absorbtion != rth.r7000->absorbtion ||\
                           mbes_spread_loss != rth.r7000->spread_loss;
        #endif

       mbes_tx_freq = rth.r7000->tx_freq ;
       mbes_tx_bw = rth.r7000->bw ;
       mbes_tx_plen = rth.r7000->tx_len;
       mbes_ping_rate = 1./(rth.r7000->ping_period);

       //For CW pulses, bandwidth is given by pulse length
       mbes_tx_bw = MAX(mbes_tx_bw,1/mbes_tx_plen);

       //mbes_tx_bw = 40e3; 
            
       //Calculate effective pulse length
       mbes_eff_plen = MIN(mbes_tx_plen, 2./mbes_tx_bw);

       mbes_gain = rth.r7000->gain;
       mbes_tx_power = rth.r7000->tx_power;
       mbes_absorbtion = rth.r7000->absorbtion;
       mbes_spread_loss = rth.r7000->spread_loss;
       
       #ifdef PRINT_S7K_7000_CHANGES
       if (new_param){
           fprintf(stderr, "Read in S7K 7000  tx_freq=%fkHz  tx_bw=%fkHz txplen=%fus txpower=%fdB gain=%fdB abs=%fdB/km spread=%fdB/log10(r)\n", mbes_tx_freq/1e3,mbes_tx_bw/1e3,mbes_tx_plen*1e6,mbes_tx_power,mbes_gain,mbes_absorbtion,mbes_spread_loss);
       }
       #endif
       // Find out exactly how the r7000-gain is used on bathy intensity data.
       return 0;
    }

	else if (drf->record_id == 10000){
       sbes_tx_freq = rth.r10000->freq_center;
       sbes_tx_bw = rth.r10000->sweep_width;
       sbes_tx_plen = rth.r10000->pulse_length;
       sbes_ping_rate = 1./(rth.r10000->ping_period);
    }
	else if (drf->record_id == 10004){}

    // --- Process S7K 10018 record ----
	else if (drf->record_id == 10018){
        float sensor_el = 0;
        uint16_t multifreq_index = rth.r10018->multi_ping;
        float Fs = rth.r10018->fs;
        float c = rth.r10018->sound_velocity;
        uint32_t Nin = rth.r10018->number_of_samples;  
        uint32_t ping_number = rth.r10018->ping_nr;
        float c_div_2Fs = c/(2*Fs);
        uint8_t* rd_ptr = (((uint8_t*) rth.r10018) + sizeof(r7k_RecordTypeHeader_10018_t));


        //fprintf(stderr, "Fs = %f c=%f Nin = %d\n",Fs,c,Nin);
        
        //Skip whole dataset condition
        if (    ((sensor_params->multifreq_index>=0) && (sensor_params->multifreq_index!=multifreq_index)) ||
                ((sensor_el < sensor_params->min_elevation) || (sensor_el > sensor_params->max_elevation)) ||
                ((ping_number < sensor_params->min_ping_number) || (sensor_params->max_ping_number && (ping_number > sensor_params->max_ping_number))) ||
                ((ping_number%ping_number_stride) != 0)

        ){
            return 0;
        }
        //Calculate navigation data at tx instant
        double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	    float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
        float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
        if (calc_interpolated_nav_data( posdata, pos_ix, ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
            if(verbose) fprintf(stderr, "Could not find navigation data for s7k 10018 record at time %f\n",ts);
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
       
        
        switch ( rth.r10018->bits_per_sample){
            case 32:
                for (uint32_t ix=0;ix<Nin;ix++){
                    temp_sig[ix] = (float) (((uint32_t*)(rd_ptr))[ix]) - (float)((rth.r10018->full_scale/2)); 
                }
                break;
            case 16:
                for (uint32_t ix=0;ix<Nin;ix++){
                    temp_sig[ix] = (float) (((uint16_t*)(rd_ptr))[ix]) - (float)((rth.r10018->full_scale/2)); 
                }
                break;
            default:
            case 8:
                for (uint32_t ix=0;ix<Nin;ix++){
                    temp_sig[ix] = (float) (((uint8_t*)(rd_ptr))[ix]) - (float)((rth.r10018->full_scale/2)); 
                }
                break;
        }
                
        //Remove DC
        float sig_mean=0;
        for (uint32_t ix=0;ix<Nin;ix++){
            sig_mean += temp_sig[ix]; 
        }
        sig_mean /= Nin;
        for (uint32_t ix=0;ix<Nin;ix++){
            temp_sig[ix]-=sig_mean; 
        }

    
        //Apply additional post-processing band pass filter
        if((sensor_params->sbp_bp_filter_start_freq>0) ||  (sensor_params->sbp_bp_filter_stop_freq>0)){
            float f0 = sensor_params->sbp_bp_filter_start_freq*1e3;
            float f1 = sensor_params->sbp_bp_filter_stop_freq*1e3;
            f1 = f1==0?30e3:f1;
            bp_filter_data(/*Input*/ temp_sig, (f0+f1)/2, f1-f0, Fs ,Nin, /*Output*/ sig);
            memcpy(temp_sig, sig,Nin*sizeof(float));
        } 
        
        if (sensor_params->sbp_raw_data){
            memcpy(sig, temp_sig,Nin*sizeof(float));
        }
        else{
            hilbert_envelope_data(/*Input*/ temp_sig,  Nin, /*Output*/ sig);
        }

        //Calculate sounding positions in sonar reference frame at tx instant
        uint32_t ix_out = 0;
        float tx_delay_range = rth.r10018->effective_pulse_length*c/2;

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
        float attenuation = calc_attenuation(sbes_tx_freq, sensor_params);
        
        for (uint32_t ix_in=ix_start;ix_in<ix_stop;ix_in+=ix_in_stride){
            float inten;
            uint32_t sample_number;
            sample_number = ix_in;
            
            float sensor_r   = sample_number*c_div_2Fs - tx_delay_range;	//Calculate range to each point en meters
            inten = sig[ix_in];
            sensor_r  += sensor_offset->r_err;
            
            if (sensor_params->intensity_correction){
                inten *= sensor_r;                  //Only comp one-way spreading loss     
                float damping_dB = attenuation * (2*sensor_r/1000); 
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

        georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode,  sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
        
        *fs_out = Fs;
        *tx_angle_out = sensor_el;
        *ping_number_out = ping_number;
        *tx_freq_out = sbes_tx_freq;
        *tx_bw_out = sbes_tx_bw;
        *tx_plen_out = sbes_tx_plen;
        *ping_rate_out = sbes_ping_rate;   
        //Post GEO-REF filtering
        Nin = Nout;
        ix_out = 0;
        for (uint32_t ix_in=0;ix_in<Nin;ix_in++){
            x[ix_out] = x[ix_in];
            y[ix_out] = y[ix_in];
            z[ix_out] = z[ix_in];
            intensity[ix_out] = intensity[ix_in];
            beam_angle[ix_out] = beam_angle[ix_in];
            beam_range[ix_out] = beam_range[ix_in];
            beam_number[ix_out] = beam_number[ix_in];
            beam_steer[ix_out] = beam_steer[ix_in];
            beam_range[ix_out] = beam_range[ix_in];
            if((z[ix_in]<sensor_params->min_depth) || (z[ix_in]>sensor_params->max_depth)) continue;

            ix_out++;
        }
        Nout = ix_out;
        outbuf->N = Nout;
        free(xs);free(ys);free(zs);free(sig);free(temp_sig);
        
        return Nout;

    }
    // --- Process S7K 7027 record ----
	else if (drf->record_id == 7027){
        float sensor_el = rth.r7027->tx_angle;
        sensor_el *= sensor_params->scale_tx_angle;
        uint16_t multifreq_index = rth.r7027->multi_ping;
        float Fs = rth.r7027->fs;
        float c = sv;
        uint32_t Nin = rth.r7027->N;  //Number of beams
        /*Coordinate with respect to sensor (forward,starboard, down)*/																					 
        
        uint32_t Nout = 0;
        uint32_t ping_number = rth.r7027->ping_nr;
        float div_Fs = 1./Fs;
        float c_div_2Fs = c/(2*Fs);
        uint32_t dfs = rth.r7027->data_field_size;
        uint8_t* rd_ptr = (((uint8_t*) rth.r7027) + sizeof(r7k_RecordTypeHeader_7027_t));
        float attenuation = calc_attenuation(mbes_tx_freq, sensor_params);
        //fprintf(stderr, "GEOREF: Serial=%ld ping_nr=%d Nin=%d dfs=%d\n",rth.r7027->serial, rth.r7027->ping_nr,Nin,dfs);

        if (sv==0){
            fprintf(stderr,"WARNING: Received 7027 record before SV has been set/received. Unable to process bathy data\n");
        }
        //fprintf(stderr,"S7k time = %f\t\tping=%d\n",ts,ping_number);

        //Calculate navigation data at tx instant
        double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	    float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
        float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
        if (calc_interpolated_nav_data( posdata, pos_ix, ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
            fprintf(stderr, "Could not find navigation data for s7k 7027 record at time %f\n",ts);
            return 0;
        }
        if (attitude_test(sensor_params, nav_yaw,  nav_pitch,  nav_roll, nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt)){ 
            return 0;
        }
        

        //Calculate a high freq sampled vector for roll and Z, to be used for each detection it rx instant
        #define ROLL_VECTOR_LEN 512
        #define ROLL_VECTOR_RATE 500.
        float roll_vector[ROLL_VECTOR_LEN];
        float z_vector[ROLL_VECTOR_LEN];
       
        //Find time interval to calculate interpolated roll vector for, base it on the fathest detection point
        float max_time = 0;
        for (uint32_t ix_in = 0; ix_in<Nin; ix_in++){
            r7k_RecordData_7027_t* rd = (r7k_RecordData_7027_t*)(rd_ptr+(ix_in*dfs));
            max_time   = MAX(max_time,(rd->detection_point));
        }
        max_time *= div_Fs;

        calc_interpolated_roll_and_z_vector(posdata, pos_ix, ts, max_time, ROLL_VECTOR_RATE, ROLL_VECTOR_LEN, /*output*/ roll_vector, z_vector);
        

        //Skip whole dataset condition
        if (    ((sensor_params->multifreq_index>=0) && (sensor_params->multifreq_index!=multifreq_index)) ||
                ((sensor_el < sensor_params->min_elevation) || (sensor_el > sensor_params->max_elevation)) ||
                ((ping_number < sensor_params->min_ping_number) || (sensor_params->max_ping_number && (ping_number > sensor_params->max_ping_number))) ||
                ((ping_number%ping_number_stride) != 0)

        ){
            return 0;
        }

        float* xs =  calloc(Nin,sizeof(float));
        float* ys =  calloc(Nin,sizeof(float));
        float* zs =  calloc(Nin,sizeof(float));
        if ( (xs==NULL)||(ys==NULL)||(zs==NULL)) return 0;
       
        // Populate r,az,el and t with data from bath data
        //float prev_sensor_r = 0.;
        //float prev_sensor_az = 0.;
	    uint32_t ix_out = 0;
        
	    for (uint32_t ix_in = 0; ix_in<Nin; ix_in+=ix_in_stride){
            r7k_RecordData_7027_t* rd = (r7k_RecordData_7027_t*)(rd_ptr+(ix_in*dfs));
            //fprintf(stderr, "%0.1f,", rd->rx_angle*180/M_PI);
            float sensor_az  = rd->rx_angle;
            float sensor_r   = (rd->detection_point)*c_div_2Fs;	//Calculate range to each point en meters
            float sensor_t   = (rd->detection_point)*div_Fs;		//Calculate tx to rx time for each point 
            uint32_t sensor_quality_flags = rd->quality;
            uint32_t flags = rd->flags;
            float inten = 0;
            float sensor_ug = 0;
            float sensor_lg = 0;
            if (dfs>=26){ 
                inten = rd->signal_strength;
            }
            if (dfs>=34){
                sensor_ug   = rd->limit_min*c_div_2Fs;	//Calculate range to each point en meters
                sensor_lg   = rd->limit_max*c_div_2Fs;	//Calculate range to each point en meters
            }
            sensor_r  += sensor_offset->r_err;
            sensor_ug  += sensor_offset->r_err;
            sensor_lg  += sensor_offset->r_err;
		
            uint8_t priority_flags = ((flags)>>9) & (0x0F);
        
            // Apply correctiom from beam corection polynom if defined
            if (sensor_params->beam_corr_poly_order){
                sensor_az = apply_beam_correction_poly(sensor_az, sensor_params->beam_corr_poly, sensor_params->beam_corr_poly_order);
            }

            #ifdef FORCE_MULTIDETECT_TO_QUALITY3
            if (priority_flags==1 || priority_flags ==2){
                sensor_quality_flags = 3;
            }
            #endif
        
            if (	(sensor_quality_flags >= sensor_params->min_quality_flag) && 
                    (sensor_quality_flags <= sensor_params->max_quality_flag) &&
                    (priority_flags >= sensor_params->min_priority_flag) && 
                    (priority_flags <= sensor_params->max_priority_flag) &&
                    (sensor_az > sensor_params->min_azimuth) && (sensor_az < sensor_params->max_azimuth) &&
                    (sensor_r > sensor_params->min_range) && (sensor_r < sensor_params->max_range) 
               ){
                
                // Add correction for roll during tx2rx period for each beam individually
                ssize_t roll_index=(ssize_t) round(sensor_t*ROLL_VECTOR_RATE);
                
                #if 0
                if (roll_index<0){
                            fprintf(stderr,"Warning, roll_index under value (=%d)\n",(int)roll_index);
                            roll_index = 0;
                        }
                        if (roll_index>=ROLL_VECTOR_LEN){
                            fprintf(stderr,"Warning, roll_index over value (=%d)\n",(int)roll_index);
                            roll_index = ROLL_VECTOR_LEN-1;
                        }
                #endif
                             
                float sensor_az_tx2rx_corr = -roll_vector[roll_index]; //Roll is given in opposite angles than sonar azimuth
                float sensor_z_tx2rx_corr = z_vector[roll_index/2]; // Z correction is for half tx to rx time
                
                quality[ix_out] = (float) sensor_quality_flags;
                quality_flags[ix_out] = sensor_quality_flags;
                classification_val[ix_out] = (sensor_quality_flags==3);  //Just calssify as Seafloor (=1) if Q=3 and Noise(=0) otherwise
                priority[ix_out] = (float) priority_flags;
                beam_number[ix_out] = ix_in;
                beam_angle[ix_out] =  sensor_az;  //Store raw beam angle from sonar for data analysis
                beam_steer[ix_out] = sensor_az;
                beam_range[ix_out] = sensor_r;
                upper_gate_range[ix_out] = sensor_ug;
                lower_gate_range[ix_out] = sensor_lg;
                *fs_out = Fs;
                *tx_angle_out = sensor_el;
                *multifreq_index_out = multifreq_index;
                *ping_number_out = ping_number;
                
                float sin_az = sinf((sensor_az+sensor_az_tx2rx_corr));
                float sin_el = sinf(sensor_el);
                xs[ix_out] = sensor_r * sin_el;
                ys[ix_out] = sensor_r * sin_az; //Sign flipped compared to standard right hand system
                zs[ix_out] = (sensor_r * sqrtf(1.f - (sin_az*sin_az) - (sin_el*sin_el))) + sensor_z_tx2rx_corr;
            
                intensity[ix_out] = inten;
                ix_out++;
            }
            /*else{
                fprintf(stderr,"Filter out data from s7k qf=%d, pf=%d az=%.2f r=%.3f\n",sensor_quality_flags,priority_flags,sensor_az*180/M_PI, sensor_r);
            }*/
        }
        Nout = ix_out;
        if (Nout){
            georef_to_global_frame(sensor_offset,xs, ys, zs,  Nout,c, nav_x, nav_y, nav_z,  nav_yaw, nav_pitch,  nav_roll, sensor_params->ray_tracing_mode, sensor_params->mounting_depth, /*OUTPUT*/ x,y,z);
            //Calculate acrosstrack pos rel sonar, for analyzis 
            float sin_nav_yaw = sin(nav_yaw);
            float cos_nav_yaw = cos(nav_yaw);
            for (ix_out=0;ix_out<Nout;ix_out++){
                swath_y[ix_out] = -((float)(x[ix_out]-nav_x))*sin_nav_yaw + ((float)(y[ix_out]-nav_y))*cos_nav_yaw;   //Offset of sounding accross-track   
            }
            // printf("Nout1 = %d\n",Nout);

            //Post GEO-REF filtering
            Nin = Nout;
            *tx_freq_out = mbes_tx_freq;
            *tx_bw_out = mbes_tx_bw;
            *tx_voltage_out = mbes_tx_power;
            *tx_plen_out = mbes_tx_plen;
            *ping_rate_out = mbes_ping_rate;   
            ix_out = 0;
            for (uint32_t ix_in=0;ix_in<Nin;ix_in++){

                x[ix_out] = x[ix_in];
                y[ix_out] = y[ix_in];
                z[ix_out] = z[ix_in];
                intensity[ix_out] = intensity[ix_in];
                beam_angle[ix_out] = beam_angle[ix_in];
                swath_y[ix_out] = swath_y[ix_in];
                //aoi[ix_out] = aoi[ix_in];
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

            //Calculate AOI on  Post-filtered data
            if (sensor_params->calc_aoi){
                calc_aoi(beam_range, beam_angle, Nout, /*output*/ aoi);
            }
            else{
                for (ix_out=0;ix_out<Nout;ix_out++){
                    aoi[ix_out] = beam_angle[ix_out];
                }
            }
             
            //Calculate time domain footprint for each beam / sounding 
            for (size_t ix=0;ix<Nout;ix++){
                outbuf->footprint_time[ix] = calc_beam_time_response(beam_range[ix], aoi[ix], beam_angle[ix],mbes_eff_plen , sensor_params);
            }

            if (sensor_params->s7k_backscatter_source == s7k_backscatter_bathy){
                // First we remove s7k added Gain/TVG 
                if (!sensor_params->keep_s7k_tvg){
                    for (size_t ix=0;ix<Nout;ix++){
                        float r = beam_range[ix];
                        float gain_scaling_dB = -(mbes_tx_power-TX_POWER_REF) -mbes_gain - (2*r/1000)*mbes_absorbtion - mbes_spread_loss*log10f(r);
                        float gain_scaling = powf(10.f,gain_scaling_dB/20);
                        intensity[ix] *= gain_scaling;
                    }
                }
                //Then we apply our own TVG
                for (size_t ix=0;ix<Nout;ix++){
                    intensity[ix] *= calc_intensity_range_scaling(beam_range[ix],attenuation, sensor_params);
                    intensity[ix] *= calc_footprint_scaling(beam_range[ix], aoi[ix_out], beam_angle[ix], mbes_eff_plen, sensor_params, /*OUTPUT*/ &(outbuf->footprint[ix_out]));
                    intensity[ix] *= calc_ara_scaling(aoi[ix_out], sensor_params);
                }
            }
            else{// If we are not using backscatter from bathy data, just set it to NaN to mark that it is missing
                for (size_t ix=0;ix<Nout;ix++){
                    intensity[ix] = 0./0.;
                    outbuf->footprint[ix] = 0./0.;
                }
            }
            variance_model(beam_range, beam_angle,aoi,Nout,nav_droll_dt,nav_dpitch_dt,/*output*/ z_var);

        }
      
        free(xs); free(ys); free(zs);
        
        outbuf->N = Nout;
        //If we are outputting backscatter from bathy data direclty, just return the data here
        if(sensor_params->s7k_backscatter_source == s7k_backscatter_bathy){
            return Nout;
        }
        /*Otherwise we need to process a snippet record to obtain backscatter
        * WARNING, a bit dodgy code. Here we assume
        *  1) that the bathy record for a given ping comes together (right before or after) the snippet data
        *  2) that the ouput buffer (outbuf) remains unchanged between the call to this function that processes bathy and coresponding snippet data. 
        */
        else{ 
            //The backscatter data might have arrived before the bathy data, if we have such a set in hte backlog, we try to process it now
            if (backlog_databuffer){
                char * tmp_databuffer = backlog_databuffer;
                uint32_t tmp_len = backlog_databuffer_len;
                backlog_databuffer = NULL;
                backlog_databuffer_len = 0;
                uint32_t ret = s7k_georef_data( tmp_databuffer,tmp_len, posdata,pos_ix, sensor_params,  /*OUTPUT*/ outbuf);
                free(tmp_databuffer);
                return ret;
            }
            else{
                return 0;
            }
        }

    }
    // --- Process S7K 7028 record for bathy intensity----
	else if ((drf->record_id == 7028) && (sensor_params->s7k_backscatter_source == s7k_backscatter_snippets)){
        // 1 Check that pingnumber for 7027-record matches outbuf->ping_number
        if ( rth.r7028->ping_nr != outbuf->ping_number ){
            //fprintf(stderr, "7028 record ping number do not match last 7027 ping number  (%d  vs %d)\n",rth.r7028->ping_nr,outbuf->ping_number);
            //Put 7028 record in backlog, to be rechecked on next 7027 processing
            if (backlog_databuffer){
                free(backlog_databuffer); //Make sure we dont have allocate more than one backlog databuffer
            }
            backlog_databuffer = malloc(databuffer_len);
            memcpy(backlog_databuffer,databuffer,databuffer_len);
            backlog_databuffer_len = databuffer_len;
            return 0;
        }
        //fprintf(stderr,"Snippet data 7028 found for ping %d, procesing intensity data from this\n",outbuf->ping_number); 
        //Create pointer to record data
        uint8_t* rd_ptr = (((uint8_t*) rth.r7028) + sizeof(r7k_RecordTypeHeader_7028_t));
        r7k_SnippetDescriptor_7028_t* rd = (r7k_SnippetDescriptor_7028_t*)(rd_ptr);
        
        float attenuation = calc_attenuation(mbes_tx_freq, sensor_params);
        float Fs = outbuf->sample_rate;
        float div_Fs = 1./Fs;
        int Nbath = outbuf->N;      //Number of soundings from bathy data
        int Nsnp = rth.r7028->N;    //Number of snippets in 7028 record
        uint8_t sample_size;        //Size in bytes of each snippet sample
        if ((rth.r7028->flags) & 0x001){
            sample_size = 4;  //32-bit snipptes
        }
        else{
            sample_size = 2;  //16-bit snippets
        }
        
        //Create pointer to beginning of first snippet sample data set
        uint8_t * snp_data_ptr = rd_ptr + sizeof(r7k_SnippetDescriptor_7028_t)*Nsnp;
        
        
        // This is the 10**(mbes_absorbtion*2*r/(20*1000))
        // This is the spreading loss that has been applied  powf(r,mbes_spread_loss/10)
        // Data in s7k records are assumed to have had gain, absorbtion, and spreadloss applied to them
        // So the data we receive is (in dB) :   20*np.log10(s) + mbes_gain                 +   2*r*mbes_absorbtion/1000          +   mbes_spread_loss*log10(r)
        // In linear units this becomes      :         s        * powf(10, mbes_gain/20)    *   powf(10, mbes_absorbtion*r/10000) *   powf(r,mbes_spread_loss/20)   
        // So to remove Gain and TVG, we must multiply by  : powf(10, -mbes_gain/20)  *  powf(10, -(2*r/1000)*(mbes_absorbtion/20))  *  powf(r,-mbes_spread_loss/20)    


        for (uint32_t ix_snp=0;ix_snp<Nsnp;ix_snp++){
            r7k_SnippetDescriptor_7028_t * sd = rd + ix_snp; 
            uint16_t beam = sd->beam_descriptor;
            uint32_t len = 1+ sd->snippet_end - sd->snippet_start;
            
            //fprintf(stderr,"snippet beam_descriptor=%4d \t start=%6d, detection=%6d, end=%6d\n",beam, sd->snippet_start,sd->detection_sample,sd->snippet_end);
            //Try to find sounding belonging to snippet 
            uint8_t match=0;
            uint32_t ix_bath;
            for (ix_bath=0;ix_bath<Nbath;ix_bath++){
                //fprintf(stderr,"snippet beam_descriptor=%4d \t bathy beam=%d\n",beam, outbuf->beam[ix_bath]);
                if (beam == outbuf->beam[ix_bath]){ 
                    match=1;
                    break;
                }
            }
            if (match){
                /**** Here we process the snippet into sounding data intensity ****/
                float inten;
                float acum_pow;
                
                int32_t detection_offset = sd->detection_sample - sd->snippet_start;
                int32_t snippet_len;       //This is the snippet length that we will be outputting 

                switch (sensor_params->snippet_processing_mode){
                    case snippet_detection_value:
                        //Take intensity from detection sample
                        if(sample_size==4){
                            inten = *( ((uint32_t*)snp_data_ptr) + detection_offset);
                        }
                        else{
                            inten = *( ((uint16_t*)snp_data_ptr) + detection_offset);
                        }
                        snippet_len=len;
                        break;
                    default:
                    case snippet_max:
                        //Max snippet singal
                        inten = 0;
                        if(sample_size==4){
                            for (size_t ix = 0; ix<len;ix++){
                                inten = MAX(inten, *( ((uint32_t*)snp_data_ptr) + ix));
                            }
                        }
                        else{
                            for (size_t ix = 0; ix<len;ix++){
                                inten = MAX(inten, *( ((uint16_t*)snp_data_ptr) + ix));
                                #ifdef COUNT_S7K_SNIPPET_SATURATION
                                if(val>=65530){ 
                                    //fprintf(stderr,"WARNING, Saturation in s7k snippet\n");
                                    s7k_snp_satcount++;
                                }
                                s7k_snp_count++;
                                #endif
                            }
                        }
                        snippet_len=len;
                        break;
                    case snippet_mean_pow:
                        //Mean snippet power  (root-mean-square)
                        acum_pow = 0;
                        if(sample_size==4){
                            for (size_t ix = 0; ix<len;ix++){
                                float val = *( ((uint32_t*)snp_data_ptr) + ix);
                                acum_pow += powf(val,2);
                            }
                        }
                        else{
                            for (size_t ix = 0; ix<len;ix++){
                                float val = *( ((uint16_t*)snp_data_ptr) + ix);
                                #ifdef COUNT_S7K_SNIPPET_SATURATION
                                if(val>=65530){ 
                                    //fprintf(stderr,"WARNING, Saturation in s7k snippet\n");
                                    s7k_snp_satcount++;
                                }
                                s7k_snp_count++;
                                #endif
                                acum_pow += powf(val,2);
                            }
                        }
                        inten = sqrtf(acum_pow/len);
                        snippet_len=len;
                        break;
                    case snippet_sum_pow:
                        acum_pow = 0;
                        if(sample_size==4){
                            for (size_t ix = 0; ix<len;ix++){
                                float val = *( ((uint32_t*)snp_data_ptr) + ix);
                                acum_pow += powf(val,2);
                            }
                        }
                        else{
                            for (size_t ix = 0; ix<len;ix++){
                                float val = *( ((uint16_t*)snp_data_ptr) + ix);
                                #ifdef COUNT_S7K_SNIPPET_SATURATION
                                if(val>=65530){ 
                                    //fprintf(stderr,"WARNING, Saturation in s7k snippet\n");
                                    s7k_snp_satcount++;
                                }
                                s7k_snp_count++;
                                #endif
                                acum_pow += powf(val,2);
                            }
                        }
                        inten = sqrtf(acum_pow);
                        snippet_len=len;
                        break;
                    case snippet_3dB_footprint_mean_pow:
                        {
                            // Crop snippet to maximum the 3dB time domain footprint
                            int32_t snippet_3dB_length=roundf(outbuf->footprint_time[ix_bath]*Fs);
                            snippet_3dB_length=MAX(snippet_3dB_length,1);
                            int32_t ix0 = detection_offset-snippet_3dB_length/2;
                            int32_t ix1 = ix0+snippet_3dB_length;
                            ix0=MAX(0,ix0);
                            ix1=MIN(ix1,len);
                            acum_pow = 0;


                            if(sample_size==4){
                                for (size_t ix = ix0; ix<ix1;ix++){
                                    float val = *( ((uint32_t*)snp_data_ptr) + ix);
                                    acum_pow += powf(val,2);
                                }
                            }
                            else{
                                for (size_t ix = ix0; ix<ix1;ix++){
                                    float val = *( ((uint16_t*)snp_data_ptr) + ix);
                                    #ifdef COUNT_S7K_SNIPPET_SATURATION
                                    if(val>=65530){ 
                                        //fprintf(stderr,"WARNING, Saturation in s7k snippet\n");
                                        s7k_snp_satcount++;
                                    }
                                    s7k_snp_count++;
                                    #endif
                                    acum_pow += powf(val,2);
                                }
                            }
                            inten = sqrtf(acum_pow/(ix1-ix0));
                            snippet_len=ix1-ix0;
                        }
                }
                
                // First we remove s7k added Gain/TVG
                if (!sensor_params->keep_s7k_tvg){
                    float r = outbuf->range[ix_bath];
                    float gain_scaling_dB = -(mbes_tx_power-TX_POWER_REF) -mbes_gain - (2*r/1000)*mbes_absorbtion - mbes_spread_loss*log10f(r);
                    float gain_scaling = powf(10.f,gain_scaling_dB/20);
                    //fprintf(stderr,"gain scaling = %f\n",gain_scaling);
                    inten *= gain_scaling;
                }
                //Then we apply our own TVG
                float eff_plen = mbes_eff_plen;
                if (sensor_params->snippet_processing_mode == snippet_sum_pow){
                    // When calculating total snippet energy, the footprint is based on the entire footprint of the snippet, not a sample footprint
                    eff_plen += len*div_Fs;
                }
                inten *= calc_intensity_range_scaling(outbuf->range[ix_bath],attenuation, sensor_params);
                inten *= calc_footprint_scaling(outbuf->range[ix_bath], outbuf->aoi[ix_bath],outbuf->teta[ix_bath], eff_plen, sensor_params, /*OUTPUT*/ &(outbuf->footprint[ix_bath]));
                inten *= calc_ara_scaling(outbuf->aoi[ix_bath], sensor_params);

                outbuf->i[ix_bath] = inten;
                outbuf->snp_len[ix_bath] = snippet_len; 
            }
            else{
                outbuf->snp_len[ix_bath] = 0; 
            }
            //Go to next snippet
            snp_data_ptr = snp_data_ptr + sample_size*len;
        }
        return Nbath;
    }
    

    // --- Process S7K 7058 record for bathy intensity----
	else if ((drf->record_id == 7058) && (sensor_params->s7k_backscatter_source == s7k_backscatter_norm_snippets)){
        // 1 Check that pingnumber for 7027-record matches outbuf->ping_number
        if ( rth.r7058->ping_nr != outbuf->ping_number ){
            //fprintf(stderr, "7058 record ping number do not match last 7027 ping number  (%d  vs %d)\n",rth.r7058->ping_nr,outbuf->ping_number);
            //Put 7058 record in backlog, to be rechecked on next 7027 processing
            if (backlog_databuffer){
                free(backlog_databuffer); //Make sure we dont have allocate more than one backlog databuffer
            }
            backlog_databuffer = malloc(databuffer_len);
            memcpy(backlog_databuffer,databuffer,databuffer_len);
            backlog_databuffer_len = databuffer_len;
            return 0;
        }
        //fprintf(stderr,"Snippet data 7058 found for ping %d, procesing intensity data from this\n",outbuf->ping_number); 
        //Create pointer to record data
        uint8_t* rd_ptr = (((uint8_t*) rth.r7058) + sizeof(r7k_RecordTypeHeader_7058_t));
        r7k_SnippetDescriptor_7058_t* rd = (r7k_SnippetDescriptor_7058_t*)(rd_ptr);
        
        float Fs = outbuf->sample_rate;
        int Nbath = outbuf->N;      //Number of soundings from bathy data
        int Nsnp = rth.r7058->N;    //Number of snippets in 7058 record
        uint32_t  control_flag = rth.r7058->control_flag;
        uint8_t footprint_included = (control_flag & (1<<6)) != 0;

        if (!(sensor_params->keep_s7k_footprint_comp) && !(footprint_included)){
            fprintf(stderr,"WARNING: Requested to remove s7k footprond correction from 7058 records, but footprint data is not included\n");
        }

        uint32_t total_snippet_samples = 0;
        // Calculate the sum of all snippet length
        for (uint32_t ix_snp=0;ix_snp<Nsnp;ix_snp++){
            r7k_SnippetDescriptor_7058_t * sd = rd + ix_snp; 
            uint32_t len = 1+ sd->snippet_end - sd->snippet_start;
            total_snippet_samples += len;
        }

        //Create pointer to beginning of first snippet backscatter data set
        float * snp_bs_data_ptr         = (float*) (rd_ptr + sizeof(r7k_SnippetDescriptor_7058_t)*Nsnp);
        float * snp_footprint_data_ptr   = snp_bs_data_ptr + total_snippet_samples;
        

        for (uint32_t ix_snp=0;ix_snp<Nsnp;ix_snp++){
            r7k_SnippetDescriptor_7058_t * sd = rd + ix_snp; 
            uint16_t beam = sd->beam_descriptor;
            uint32_t len = 1+ sd->snippet_end - sd->snippet_start;
            
            //fprintf(stderr,"snippet beam_descriptor=%4d \t start=%6d, detection=%6d, end=%6d\n",beam, sd->snippet_start,sd->detection_sample,sd->snippet_end);
            //Try to find sounding belonging to snippet 
            uint8_t match=0;
            uint32_t ix_bath;
            for (ix_bath=0;ix_bath<Nbath;ix_bath++){
                //fprintf(stderr,"snippet beam_descriptor=%4d \t bathy beam=%d\n",beam, outbuf->beam[ix_bath]);
                if (beam == outbuf->beam[ix_bath]){ 
                    match=1;
                    break;
                }
            }
            if (match){
                /**** Here we process the snippet into sounding data intensity ****/

                // Mean snippet power  (root-mean-square)
                // Snippets are reported in bs = 10log10(bcs), where bcs is the scaled power
                // To calculate a scaled rms intensity, similar as for 7028 and WBMS data, we calulate intensity as  sqrt mean bcs
    
                float acum_pow;
                int32_t detection_offset = sd->detection_sample - sd->snippet_start;
                uint8_t remove_s7k_footprint_comp = !(sensor_params->keep_s7k_footprint_comp);
                switch (sensor_params->snippet_processing_mode){
                    case snippet_detection_value:
                        {
                        //Take intensity from detection sample
                        float bs  = *( snp_bs_data_ptr + detection_offset);   //Reported backscattering stength = 10log10(bcs) corrected for footprint
                        float bcs = powf(10,bs/10.f);                         //Backscattering cross sectioni(power value)
                        if( remove_s7k_footprint_comp ){
                            float footprint  = *( snp_footprint_data_ptr + detection_offset);   //Reported snippet footprint im m2   
                            bcs *= footprint;
                        }
                        outbuf->i[ix_bath] = sqrtf(bcs);        //Sqrt of sample power
                        outbuf->snp_len[ix_bath] = len; 
                        }
                        break;
                    default:
                    case snippet_mean_pow:
                        //Mean snippet power  (root-mean-square)
                        acum_pow = 0;
                        for (size_t ix = 0; ix<len;ix++){
                            float bs  = *( snp_bs_data_ptr + ix);   //Reported backscattering stength = 10log10(bcs) corrected for footprint
                            float bcs = powf(10,bs/10.f);           //Backscattering cross sectioni(power value)
                            if( remove_s7k_footprint_comp ){
                                float footprint  = *( snp_footprint_data_ptr + ix);   //Reported snippet footprint im m2   
                                bcs *= footprint;
                            }
                            acum_pow += bcs;
                        }
                        outbuf->i[ix_bath] = sqrtf(acum_pow/len);   //Sqrt of mean power
                        outbuf->snp_len[ix_bath] = len; 
                        break;
                    case snippet_sum_pow:
                        acum_pow = 0;
                        for (size_t ix = 0; ix<len;ix++){
                            float bs  = *( snp_bs_data_ptr + ix);   //Reported backscattering stength = 10log10(bcs) corrected for footprint
                            float bcs = powf(10,bs/10.f);           //Backscattering cross sectioni(power value)
                            if( remove_s7k_footprint_comp ){
                                float footprint  = *( snp_footprint_data_ptr + ix);   //Reported snippet footprint im m2   
                                bcs *= footprint;
                            }
                            acum_pow += bcs;
                        }
                        outbuf->i[ix_bath] = sqrtf(acum_pow);       //Sqrt of summed power
                        outbuf->snp_len[ix_bath] = len; 
                        break;
                    case snippet_3dB_footprint_mean_pow:
                        {
                            // Crop snippet to maximum the 3dB time domain footprint
                            int32_t snippet_3dB_length=roundf(outbuf->footprint_time[ix_bath]*Fs);
                            snippet_3dB_length=MAX(snippet_3dB_length,1);
                            int32_t ix0 = detection_offset-snippet_3dB_length/2;
                            int32_t ix1 = ix0+snippet_3dB_length;
                            ix0=MAX(0,ix0);
                            ix1=MIN(ix1,len);
                            acum_pow = 0;
                            for (size_t ix = ix0; ix<ix1;ix++){
                                float bs  = *( snp_bs_data_ptr + ix);   //Reported backscattering stength = 10log10(bcs) corrected for footprint
                                float bcs = powf(10,bs/10.f);           //Backscattering cross sectioni(power value)
                                if( remove_s7k_footprint_comp ){
                                    float footprint  = *( snp_footprint_data_ptr + ix);   //Reported snippet footprint im m2   
                                    bcs *= footprint;
                                }
                                acum_pow += bcs;
                            }
                            outbuf->i[ix_bath] = sqrtf(acum_pow/(ix1-ix0));   //Sqrt of mean power
                            outbuf->snp_len[ix_bath] = ix1-ix0; 
                            break;
                        }
                }
        
                // If we remove the 7058 record footprint correction, we instead apply our own footprint correction here
                if( !(sensor_params->keep_s7k_footprint_comp) ){
                    //TODO dont we need to calculate different eff plen here based on processing mode like we do for 7028?
                    outbuf->i[ix_bath] *= calc_footprint_scaling(outbuf->range[ix_bath], outbuf->aoi[ix_bath],outbuf->teta[ix_bath], mbes_eff_plen, sensor_params, /*OUTPUT*/ &(outbuf->footprint[ix_bath]));
                }
                else{
                    //Report mean fooprint from 7058 record if not removed 
                    if (footprint_included){
                        float acum_footprint = 0;
                        for (size_t ix = 0; ix<len;ix++){
                            float footprint  = *( snp_footprint_data_ptr + ix);   //Reported snippet footprint im m2
                            acum_footprint += footprint;
                        }
                        outbuf->footprint[ix_bath] = acum_footprint/len;
                    }
                    else{
                        outbuf->footprint[ix_bath] = 0./0.;
                    }
                }
                outbuf->i[ix_bath] *= calc_ara_scaling(outbuf->aoi[ix_bath], sensor_params);
            }
            else{
                outbuf->snp_len[ix_bath] = 0; 
            }
            //Go to next snippet
            snp_bs_data_ptr = snp_bs_data_ptr + len;
            snp_footprint_data_ptr = snp_footprint_data_ptr + len;
        }
        

        return Nbath;
    }

    return 0;

}

