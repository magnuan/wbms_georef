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
#include "cmath.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

static uint8_t verbose = 0;

static offset_t* sensor_offset;

void r7k_set_sensor_offset(offset_t* s){
    sensor_offset = s;
}


uint8_t r7k_test_file(int fd,int req_types[], size_t n_req_types){
    uint8_t pass=0;
    char* data = malloc(MAX_S7K_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<200;test++){     //Test the first 200 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = r7k_fetch_next_packet(data, fd);
        //printf("r7k_test_file, %d len=%d\n",req_types[0],len);
        if (len > 0 ){
            double ts;
            int type = r7k_identify_sensor_packet(data, len, &ts);
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
    int req_types[] = {1015,1016};
    return r7k_test_file(fd,req_types,2);
}
uint8_t r7k_test_bathy_file(int fd){
    int req_types[] = {7027};
    return r7k_test_file(fd,req_types,1);
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
    
    /*uint16_t s7k_version = ((uint16_t*) data)[0];
	uint16_t s7k_offset = ((uint16_t*) data)[1];
    uint32_t s7k_record_id = ((uint32_t*) data)[8];
	fprintf(stderr, "FETCH: S7K ver = %d, S7K off = %d,  size = %d recordID = %d\n",s7k_version,s7k_offset, s7k_size, s7k_record_id);
	*/
    return s7k_size;
}

int r7k_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out){
    
	r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
 	double ts = r7k_r7ktime_to_ts(&(drf->time));
    // If record data is from sensor (sonar) add sensor time offset
    if ( drf->record_id >= 7000  && drf->record_id < 8000){
        ts += sensor_offset->time_offset;
    }
	*ts_out = ts;
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
    
    static navdata_t navdata_collector; 
	static uint8_t have_pos; 
	static uint8_t have_attitude; 

    r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
 	double ts = r7k_r7ktime_to_ts(&(drf->time));
	*ts_out = ts;

    //fprintf(stderr,"S7K record  id:%d dev:%d  Y=%d doy=%d  %02d:%02d:%09.6f   ts = %f\n",drf->record_id,drf->dev_id,drf->time.year,drf->time.day,drf->time.hour,drf->time.min,drf->time.sec,ts);
	//fprintf(stderr, "%d\n",drf->offset);	
	union r7k_RecordTypeHeader rth;
	rth.dummy = (r7k_RecordTypeHeader_dummy_t*) (databuffer+4+(drf->offset));
	
	//So far we only process s7k record 1015 and 1016 for navigation
	if ((drf->record_id != 1015) && (drf->record_id != 1016))
		return NO_NAV_DATA;	

    // The data collector needs to collect one set of posision and one set of attitude data with same timestamp
    // to generate a navigation out entry. 
    if (ts != navdata_collector.ts){ 	// If this data has a new timestamp
        navdata_collector.ts = ts;
        //New entry, still no navigation data 
	    if (have_pos != have_attitude){
            if(verbose) fprintf(stderr,"Dumping incomplete nav dataset\n");
        }
        have_pos=0;
        have_attitude=0;	
    }


	switch (drf->record_id){
		case 1003: // Position
			fprintf(stderr,"1003 (NOT USED YET) Datum=%d Latency=%f Lat=%f Lon=%f Height=%f PosType=%d, UTMZone=%d, Qual=%d Method=%d\n ", \
							rth.r1003->datum,rth.r1003->latency,rth.r1003->lat_northing*180/M_PI, rth.r1003->lon_easting*180/M_PI, rth.r1003->height,  \
							rth.r1003->pos_type, rth.r1003->utm_zone, rth.r1003->quality, rth.r1003->pos_method);
			break;
		case 1012:	// Roll Pitch Heave
			fprintf(stderr,"1012 (NOT USED YET) roll=%f pitch=%f heave=%f\n",rth.r1012->roll*180/M_PI,rth.r1012->pitch*180/M_PI, rth.r1012->heave);
			break;
		case 1013:	// Heading
			fprintf(stderr,"1013 (NOT USED YET) heading=%f\n", rth.r1015->heading*180/M_PI);
			break;
		case 1015:	// Navigation
			have_pos = 1;
			if (verbose>2 ) fprintf(stderr,"ts=%f 1015 Vert_ref=%d Lat=%f Lon=%f Height=%f Accuracy=%f, %f, SoG=%f, Cog=%f Heading=%f\n", \
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

			break;
		case 1016:	// Attitude
			have_attitude = 1;
			if (verbose>2 ) fprintf(stderr,"ts=%f 1016 t_off=%dms roll=%f pitch=%f heading=%f heave=%f\n", \
            ts, \
			rth.r1016->entry[0].t_off_ms,rth.r1016->entry[0].roll*180/M_PI,rth.r1016->entry[0].pitch*180/M_PI, rth.r1016->entry[0].heading*180/M_PI, rth.r1016->entry[0].heave);
			
			navdata_collector.roll = rth.r1016->entry[0].roll;
			navdata_collector.pitch = rth.r1016->entry[0].pitch;
			navdata_collector.yaw = rth.r1016->entry[0].heading;
			navdata_collector.heave = rth.r1016->entry[0].heave;
			last_heave = rth.r1016->entry[0].heave;
			
			if (alt_mode ==2){ 
				navdata_collector.z = -navdata_collector.heave;  //heave from posmv is positive up ( Without RTK it is usually better to use heave + tide  (TODO add some ay to input tide files)) 
			}

			if (proj){
				navdata_collector.yaw += north_to_northing_offset(last_lon, last_lat, proj);
			}
			break;
	}

	if (have_pos && have_attitude){
        memcpy(navdata, &navdata_collector, sizeof(navdata_t));
        have_pos=0;
        have_attitude=0;	
        //fprintf(stderr,"Complete nav data at ts=%f\n",ts);
        return (proj?NAV_DATA_PROJECTED:NAV_DATA_GEO);
    }
    return NO_NAV_DATA;
}


uint32_t s7k_georef_data( char* databuffer, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params,  /*OUTPUT*/ output_data_t* outbuf){
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
    //float* aoi = &(outbuf->aoi[0]);
    float* upper_gate_range = &(outbuf->up_gate[0]);
    float* lower_gate_range = &(outbuf->low_gate[0]);
    float* quality = &(outbuf->quality[0]);
    float* priority = &(outbuf->priority[0]);
    float* tx_angle_out = &(outbuf->tx_angle);
    float* sv_out = &(outbuf->sv);
    float* tx_freq_out = &(outbuf->tx_freq);
    float* tx_bw_out = &(outbuf->tx_bw);
    int* ping_number_out = &(outbuf->ping_number);
    //int* multiping_index_out = &(outbuf->multiping_index);
    int* multifreq_index_out = &(outbuf->multifreq_index);

    static float sv;
    static float tx_freq;
    static float tx_bw;

	r7k_DataRecordFrame_t* drf = (r7k_DataRecordFrame_t*) databuffer;
 	double ts = r7k_r7ktime_to_ts(&(drf->time));
    ts += sensor_offset->time_offset;
	union r7k_RecordTypeHeader rth;
	rth.dummy = (r7k_RecordTypeHeader_dummy_t*) (databuffer+4+(drf->offset));
    
    #define HEAP
    #ifndef HEAP
    float xs[MAX_DP], ys[MAX_DP], zs[MAX_DP];	/*Coordinate with respect to sensor (forward,starboard, down)*/
	#endif

	//So far we only process s7k record 7027  and 7610
    //fprintf(stderr,"S7K record  id:%d dev:%d  Y=%d doy=%d  %02d:%02d:%09.6f   ts = %f\n",drf->record_id,drf->dev_id,drf->time.year,drf->time.day,drf->time.hour,drf->time.min,drf->time.sec,ts);
    if (drf->record_id == 7610){
       sv = rth.r7610->sv ;
       //fprintf(stderr, "Read in sound velocity from S7K 7610  sv=%f\n", sv);
       return 0;
    }
    *sv_out = sv;
    
    if (drf->record_id == 7000){
       tx_freq = rth.r7000->tx_freq ;
       tx_bw = rth.r7000->bw ;
       //fprintf(stderr, "Read in tx freq from S7K 7000  tx_freq=%f\n", tx_freq);
       return 0;
    }
    *tx_freq_out = tx_freq;
    *tx_bw_out = tx_bw;
       

	if (drf->record_id == 7027){
        float sensor_el = rth.r7027->tx_angle;
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
        uint8_t* rd_ptr = (((uint8_t*) rth.r7610) + sizeof(r7k_RecordTypeHeader_7027_t));
        //fprintf(stderr, "GEOREF: Serial=%ld ping_nr=%d Nin=%d dfs=%d\n",rth.r7027->serial, rth.r7027->ping_nr,Nin,dfs);

        //Calculate navigation data at tx instant
        double nav_x, nav_y, nav_z; 			    /*Position in global coordinates (north,east,down)*/
	    float nav_yaw,  nav_pitch,  nav_roll;       /*Rotations of posmv coordinates*/
        float nav_droll_dt, nav_dpitch_dt, nav_dyaw_dt;
        if (calc_interpolated_nav_data( posdata, pos_ix, ts,/*OUTPUT*/ &nav_x, &nav_y, &nav_z, &nav_yaw, &nav_pitch, &nav_roll, &nav_dyaw_dt, &nav_dpitch_dt, &nav_droll_dt)){
            if(verbose) fprintf(stderr, "Could not find navigation data for s7k 7027 record at time %f\n",ts);
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
        
        //TODO fix AOI calculation for S7K 7027 record, need to set up a sorting function
        /*if(calc_aoi){
            qsort(bath->dp, Nin, sizeof(detectionpoint_t), cmp_wbms_dp_on_angle_func);
        }*/
        
        
        //Skip whole dataset condition
        if (    ((sensor_params->multifreq_index>=0) && (sensor_params->multifreq_index!=multifreq_index)) ||
                ((sensor_el < sensor_params->min_elevation) || (sensor_el > sensor_params->max_elevation)) ||
                ((ping_number < sensor_params->min_ping_number) || (sensor_params->max_ping_number && (ping_number > sensor_params->max_ping_number)))
        ){
            return 0;
        }

        #ifdef HEAP
        float* xs =  calloc(Nin,sizeof(float));
        float* ys =  calloc(Nin,sizeof(float));
        float* zs =  calloc(Nin,sizeof(float));
        if ( (xs==NULL)||(ys==NULL)||(zs==NULL)) return 0;
        #endif
       
        // Populate r,az,el and t with data from bath data
        //float prev_sensor_r = 0.;
        //float prev_sensor_az = 0.;
	    uint32_t ix_out = 0;
        
        for (uint32_t ix_in = 0; ix_in<Nin; ix_in++){
            r7k_RecordData_7027_t* rd = (r7k_RecordData_7027_t*)(rd_ptr+(ix_in*dfs));
            //fprintf(stderr, "%0.1f,", rd->rx_angle*180/M_PI);
            float sensor_az  = rd->rx_angle;
            float sensor_r   = (rd->detection_point)*c_div_2Fs;	//Calculate range to each point en meters
            float sensor_t   = (rd->detection_point)*div_Fs;		//Calculate tx to rx time for each point 
            uint32_t quality_flags = rd->quality;
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

            #ifdef FORCE_MULTIDETECT_TO_QUALITY3
            if (priority_flags==1 || priority_flags ==2){
                quality_flags = 3;
            }
            #endif
        
            if (	(quality_flags >= sensor_params->min_quality_flag) && 
                    (quality_flags <= sensor_params->max_quality_flag) &&
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
                
              
                quality[ix_out] = (float) quality_flags;
                priority[ix_out] = (float) priority_flags;
                beam_number[ix_out] = ix_in;
                beam_angle[ix_out] =  sensor_az;  //Store raw beam angle from sonar for data analysis
                beam_steer[ix_out] = sensor_az;
                beam_range[ix_out] = sensor_r;
                upper_gate_range[ix_out] = sensor_ug;
                lower_gate_range[ix_out] = sensor_lg;
                *tx_angle_out = sensor_el;
                *multifreq_index_out = multifreq_index;
                *ping_number_out = ping_number;
                
                float sin_az = sinf((sensor_az+sensor_az_tx2rx_corr));
                float sin_el = sinf(sensor_el);
                xs[ix_out] = sensor_r * sin_el;
                ys[ix_out] = sensor_r * sin_az; //Sign flipped compared to standard right hand system
                zs[ix_out] = (sensor_r * sqrtf(1.f - (sin_az*sin_az) - (sin_el*sin_el))) + sensor_z_tx2rx_corr;
            
                //Compensate intensity for range and AOI
                //TODO fix AOI calculation for S7K 7027 record, need to set up a sorting function
                /*if (calc_aoi){
                    aoi[ix_out] = atan2((sensor_r-prev_sensor_r), (sensor_az-prev_sensor_az)*sensor_r);         //AOI defined as angle between seafloor normal and beam (not seafloor and beam)
                }
                else{
                    aoi[ix_out] = sensor_az;        //Just asume that AOI is equal to beam angle (flat seafloor assumption)
                }
                aoi[ix_out] = (aoi[ix_out]);
                if (intensity_aoi_comp){
                    if(use_intensity_angle_corr_table){
                        int ix = ABS(aoi[ix_out])/INTENSITY_ANGLE_STEP;
                        ix = LIMIT(ix,0,INTENSITY_ANGLE_MAX_VALUES-1);
                        inten *= intenity_angle_corr_table[ix].intensity_scale;
                    }
                    else{
                        inten = inten/((MAX(cos(aoi[ix_out]),0.1)));
                    }
                }*/

                if (sensor_params->intensity_range_comp){
                    inten *= sensor_r;     //Only comp one-way spreading loss     
                    float damping_dB = sensor_params->intensity_range_attenuation * (2*sensor_r/1000); 
                    inten *= powf(10.f,damping_dB/20); 
                }
                intensity[ix_out] = inten;
      
                //TODO insert uncertainty model here
                float beam_width = (0.1*M_PI/180.) / cosf(0.85*sensor_az);
                float sigma_teta =  M_SQRT2 * beam_width;
                const float sigma_range = M_SQRT2 * 1450./80e3;
                const float sigma_t = M_SQRT2 * 0.005;
                float sigma_z_teta  = sigma_teta*sensor_r*sinf(sensor_az);
                float sigma_z_range = sigma_range*cosf(sensor_az);
                float sigma_z_roll = nav_droll_dt * sigma_t *sensor_r*cosf(sensor_az);
                float sigma_z_pitch = nav_dpitch_dt * sigma_t *sensor_r*cosf(sensor_az);
                z_var[ix_out] =  sigma_z_teta*sigma_z_teta + sigma_z_range*sigma_z_range + sigma_z_roll*sigma_z_roll + sigma_z_pitch*sigma_z_pitch;

                ix_out++;

                //prev_sensor_r = sensor_r;
                //prev_sensor_az = sensor_az;
            }
            /*else{
                fprintf(stderr,"Filter out data from s7k qf=%d, pf=%d az=%.2f r=%.3f\n",quality_flags,priority_flags,sensor_az*180/M_PI, sensor_r);
            }*/
        }
        Nout = ix_out;

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
        ix_out = 0;
        for (uint32_t ix_in=0;ix_in<Nin;ix_in++){

            x[ix_out] = x[ix_in];
            y[ix_out] = y[ix_in];
            z[ix_out] = z[ix_in];
            z_var[ix_out] = z_var[ix_in];
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
       
        #ifdef HEAP
        free(xs); free(ys); free(zs);
        #endif
        return Nout;
    }
    return 0;

}

