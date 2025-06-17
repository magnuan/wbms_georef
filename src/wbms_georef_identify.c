#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#ifdef __unix__
#include <unistd.h>
#endif



#include "cmath.h"
#include <math.h>

#include "wbms_georef.h"
#include "georef_tools.h"
#include "time_functions.h"
#include "sim_data.h"
#include "wbms_data.h"
#include "3dss_dx.h"
#include "reson7k.h"
#include "velodyne.h"
#include "gsf_wrapper.h"
#include "posmv.h"
#include "xtf_nav.h"
#include "sim_nav.h"
#include "sbet_nav.h"
#include "eelume_sbd_nav.h"
#include "nmea_nav.h"
#include "wbm_tool_nav.h"
#include "json_output.h"
#include "misc.h"
#include "navdata_abstractor.h"
#include "sensordata_abstractor.h"
#include "svpdata.h"

#if defined(_MSC_VER)
#include "non_posix.h"
#include <corecrt_math_defines.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#include <io.h>
#include <windows.h>
#include <tchar.h>
#include "XGetopt.h"
#endif



#ifdef COUNT_S7K_SNIPPET_SATURATION
uint32_t s7k_snp_satcount=0;
uint32_t s7k_snp_count=0;
#endif


static char output_projection_string[256];
pos_mode_e pos_mode = pos_mode_autodetect;
sensor_mode_e sensor_mode = sensor_mode_autodetect;


uint8_t force_nav_epoch = 0;
double nav_epoch_week_ts = 0;

void error(const char *msg)
{
	perror(msg);
	exit(0);
}


void showUsage(char *pgmname)
{
		printf("Usage: %s file \n", pgmname);
		printf(	"Checks if a file can be read by wbms_georef, and output information about the file in JSON format\n");
}





/************************************** MAIN *********************************************/


int main(int argc,char *argv[])
{


	//File input and output stuff
	FILE * input_fileptr = NULL;
    static char * input_file_string = NULL;
    file_stats_t* file_stats= NULL;

    wbms_init();
    r7k_init();
    posmv_init();
    gsf_init();

    static offset_t sensor_offset;
    wbms_set_sensor_offset(&sensor_offset);
    r7k_set_sensor_offset(&sensor_offset);
    p3dss_set_sensor_offset(&sensor_offset);
    velodyne_set_sensor_offset(&sensor_offset);
    lakibeam_set_sensor_offset(&sensor_offset);
	/**** PARSING COMMAND LINE OPTIONS ****/


	int c;
    while (optind < argc) {
        if ((c = getopt (argc, argv, "?h")) != -1) {
            switch (c) {
                case '?':
                case 'h':
                    showUsage(argv[0]);
                    return(0);
                default:
                    fprintf(stderr,"Unknown option - %c. Use -h to get a list of valid options.\n",c);
                    return(0);
            }
        } else {
		    input_file_string = argv[optind++];
        }
    }


    double ts_svp;
    size_t svp_datapoints;
    svp_mode_e svp_mode = svp_test_file(input_file_string, &ts_svp, &svp_datapoints);
	
    #if 1 //Additional Check for data sanity
    sv_meas_t* sv_meas = malloc(MAX_SV_MEAS*sizeof(sv_meas_t));
    int sv_meas_len = svp_read_from_file(input_file_string, sv_meas, MAX_SV_MEAS);
    if (svp_mode==svp_mode_ascii_tuple) svp_auto_swap_sv_depth(sv_meas, sv_meas_len);
    sv_meas_len = svp_discard_insane(sv_meas, sv_meas_len);
    if (sv_meas_len <1){
        svp_mode = 0;
    }
    free(sv_meas);
    #endif


    if (svp_mode>0){    //File recognized as SVP data
        int version = 0;
        if (svp_mode == svp_mode_caris_v2) version = 2;
        char ver_string[32];
        snprintf(ver_string,32,"%d",version);
        /* Write file stats */
        file_stats= calloc(1,sizeof(file_stats_t));

        file_stats->file_type = svp_mode_short_names[svp_mode];     
        file_stats->file_version = (version>0)?ver_string:NULL;  
        file_stats->data_type = "svp";     
        file_stats->sensor_type = "unknown";   
        file_stats->num_record_types = 0;
        file_stats->has_svp = 1;
        file_stats->datapoints = svp_datapoints;
        file_stats->datasets = 1;

        file_stats->start_time = ts_svp;
        file_stats->duration = 0;

        /************ Generate JSON  ******************/

        char* json_buf = malloc(2048);
        write_stats_json_to_buffer(file_stats, /*OUTPUT*/json_buf);
        fprintf(stdout,"%s",json_buf);
        free(file_stats);
        free(json_buf);

        return(0);
    }

    fprintf(stderr,"Reading sensor data from FILE: %s\n",input_file_string);
    input_fileptr = fopen(input_file_string,"rb");
    if (input_fileptr == NULL){
        fprintf(stderr,"Could not open sensor data file%s\n",input_file_string);
        exit(-1);
    }
    //The GSF library needs to access the file by filename, so we need to tell it that.
    gsf_set_sensor_filename(input_file_string);
    gsf_set_nav_filename(input_file_string);
	int input_fd = fileno(input_fileptr);
   
    //Try to derive a time from the filename, this will be used to set the epoch week, for SBET and POSMV data which does not contatin full timestamps
    double ts0 = parse_timestamp_from_filename(input_file_string);
    if (ts0){
        char timestring[256];
        sprintf_unix_time(timestring,ts0);
        fprintf(stderr,"Time from filename: %s\n",timestring);
    }
    else{
        fprintf(stderr,"Could not parse time from filename\n");
        ts0 = 60*60*24*7;   //Setting one week into 1970, to avoid negative epoch
    }
    set_sbet_epoch(ts0);
    set_posmv_alt_gps_epoch(ts0);


    #if 1
    /************ Parse file as navigation data file ******************/
    pos_mode = navigation_autodetect_file(input_fileptr);
    fprintf(stderr,"pos_mode = %d\n", pos_mode);
    if (pos_mode != pos_mode_unknown){
        double ts_sensor = 0.; //For nav data sources that does not give full time, thios might need to be popolated
        double ts_pos;
        fprintf(stderr,"Allocating memory for navigation data buffer\n");
        char * navigation_data_buffer = malloc(MAX_NAVIGATION_PACKET_SIZE);
       
        //We just need to set some epoch for navigation files that dont contain full timestamps
        
        // Read out first position to derive UTM zone for data
        while (1){
            uint32_t navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_fd,pos_mode);
            //fprintf(stderr,"navigation_data_buffer_len = %d\n",navigation_data_buffer_len);
            if(navigation_data_buffer_len>0){

                int new_nav_data = process_nav_data_packet(navigation_data_buffer,navigation_data_buffer_len,ts_sensor, &ts_pos,pos_mode,0);  
                //fprintf(stderr,"new_nav_data=%d\n",new_nav_data);
                if( new_nav_data){
                    if (proj_latlon_to_output_utm==NULL){
                        double lat;
                        double lon;
                        //TODO this is not very clean code. When received data is Goegraphic only, and not projected , it arrives in the index following the last valid
                        if( new_nav_data == NAV_DATA_GEO){
                            lat = navdata[(navdata_ix+1)%NAVDATA_BUFFER_LEN].lat;
                            lon = navdata[(navdata_ix+1)%NAVDATA_BUFFER_LEN].lon;
                        }
                        else{
                            lat = navdata[navdata_ix].lat;
                            lon = navdata[navdata_ix].lon;
                        }
                        fprintf(stderr,"First nav data lat = %8.5fdegN lon = %8.5fdegE\n",lat*180/M_PI,lon*180/M_PI);
                        time_t raw_time = (time_t) ts_pos;
                        fprintf(stderr,"First nav data time: ts=%0.3f %s  ",ts_pos,ctime(&raw_time));

                        // Set UTM zone for calculating path length
                        default_projection(output_projection_string,lat,lon);
                        fprintf(stderr, "Using projection %s for calculating path length\n",output_projection_string);
                        if(latlon_to_proj_from_string(output_projection_string,&proj_latlon_to_output_utm)<0){
                            fprintf(stderr, "Error configuring georef parameters, exiting\n");
                            return(-1);
                        }
                    }
                    if (ts_pos>(7*24*60*60)){
                        break;
                    }
                }
            }
            else{
                break;
            }
        }
        if(proj_latlon_to_output_utm==NULL){
            fprintf(stderr, "ERROR: Could not find any position data in navigation data stream\n");
            return(-1);
        }

        if (pos_mode == pos_mode_gsf){
            gsf_rewind(fileno(input_fileptr)); 
        }
        else{
            fseek(input_fileptr, 0, SEEK_SET); //Rewind
        }
        //Running through the entire file, to find centroid, path length and lat/lon bounding box
        double lat_min=M_PI/2; 
        double lat_max=-M_PI/2;
        double lon_min=M_PI; 
        double lon_max=-M_PI;
        double lat_start=0.;
        double lon_start=0.;
        double lat_end=0.;
        double lon_end=0.;
        double ts_start=0;
        double ts_end=0;
        uint32_t cnt=0;
        double prev_x=0;
        double prev_y=0;
        double x_start=0;
        double y_start=0;
        double z_start=0;
        double x_end=0; 
        double y_end=0;
        double z_end=0;
        float acum_dist = 0;

        double lat=0;
        double lon=0;
        double x=0;
        double y=0;
        double z=0;

        fprintf(stderr,"Reading out navigation records\n");
        while(1){
            uint32_t navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_fd,pos_mode);
            //fprintf(stderr,"navigation_data_buffer_len = %d\n",navigation_data_buffer_len);
            if (navigation_data_buffer_len==0) break;
            if(process_nav_data_packet(navigation_data_buffer,navigation_data_buffer_len,ts_sensor, &ts_pos,pos_mode,0)){
                lat = navdata[navdata_ix].lat;
                lon  = navdata[navdata_ix].lon;
                x = navdata[navdata_ix].x;
                y = navdata[navdata_ix].y;
                z = navdata[navdata_ix].z;
                lat_min = MIN(lat,lat_min);
                lon_min = MIN(lon,lon_min);
                lat_max = MAX(lat,lat_max);
                lon_max = MAX(lon,lon_max);

                if (cnt==0){
                    ts_start = ts_pos;
                    x_start = x;
                    y_start = y;
                    z_start = z;
                    prev_x = x;
                    prev_y = y;
                    lat_start = lat;
                    lon_start = lon;
                }
                cnt++;
                float dx = x-prev_x;
                float dy = y-prev_y;
                acum_dist += sqrtf(dx*dx + dy*dy);
                prev_x = x;
                prev_y = y;
            }
        }
        fprintf(stderr,"Done reading nav records %d records read\n",cnt);

        x_end = x;
        y_end = y;
        z_end = z;
        lat_end = lat;
        lon_end = lon;
        ts_end = ts_pos;
        float dx = x_end-x_start;
        float dy = y_end-y_start;
        float start_stop_dist = sqrtf(dx*dx + dy*dy);

        fprintf(stderr, "start/stop: lat: %f - %f  lon %f - %f  ts %f - %f\n",lat_start*180/M_PI, lat_end*180/M_PI,  lon_start*180/M_PI, lon_end*180/M_PI, ts_start, ts_end);
        fprintf(stderr, "bbox      : lat: %f - %f  lon %f - %f\n",lat_min*180/M_PI, lat_max*180/M_PI,  lon_min*180/M_PI, lon_max*180/M_PI);
        fprintf(stderr, "start/stop dist %f\n", start_stop_dist);
        fprintf(stderr, "acum dist       %f\n", acum_dist);
        fprintf(stderr, "count           %d\n", cnt);
        
        free(navigation_data_buffer);
        /* Write file stats */
        if (file_stats==NULL){ // If file stats is filled out for the first time, add these fields
            uint32_t num_record_types = navigation_num_record_types(pos_mode);
            file_stats= calloc(1,sizeof(file_stats_t) + num_record_types*sizeof(record_count_t));
        
            file_stats->file_type = pos_mode_short_names[pos_mode];     
            file_stats->file_version = NULL;  
            file_stats->data_type = "nav";     
            file_stats->sensor_type = "unknown";   
            file_stats->num_record_types = navigation_get_record_count(pos_mode, file_stats->records);
        }

        file_stats->has_navigation = 1;
        file_stats->navigation_points = cnt;
        file_stats->start_time = ts_start;
        file_stats->duration  = ts_end - ts_start;
        file_stats->latitude = (lat_max+lat_min)/2;     
        file_stats->longitude = (lon_max+lon_min)/2;    
        file_stats->altitude = (z_start+z_end)/2;
        file_stats->line_length = acum_dist;
        file_stats->start_stop_distance = start_stop_dist;
    } 

    // Check for sensor data in file
    if (pos_mode == pos_mode_gsf){
        gsf_close(fileno(input_fileptr)); 
    }
    else{
        fseek(input_fileptr, 0, SEEK_SET); //Rewind
    }
    #else
    pos_mode = pos_mode_unknown;
    #endif
    
    #if 1

    /************ Parse file as sensor data file ******************/
    /* Only these formats allows sensor data merged with nav data */
    if ( (pos_mode == pos_mode_unknown) || (pos_mode==pos_mode_s7k) || (pos_mode==pos_mode_gsf) || (pos_mode==pos_mode_3dss_stream)){ 
        switch(pos_mode){
            default:
            case pos_mode_unknown:
                sensor_mode = sensor_autodetect_file(input_fileptr);
                break;
            case pos_mode_s7k:
                sensor_mode = sensor_mode_s7k;
                break;
            case pos_mode_gsf:
                sensor_mode = sensor_mode_gsf;
                break;
            case pos_mode_3dss_stream:
                sensor_mode = sensor_mode_3dss_stream;
                break;
        }

        int version;
        sensor_test_file(input_fd,sensor_mode,&version);
        if (sensor_mode == sensor_mode_gsf){
            gsf_rewind(fileno(input_fileptr)); 
        }
        else{
            fseek(input_fileptr, 0, SEEK_SET); //Rewind
        }
        char ver_string[32];
        snprintf(ver_string,32,"%d",version);



        if (sensor_mode != sensor_mode_unknown){
            //TODO Code to parse sensor data here
            fprintf(stderr,"Allocating memory for sensor data buffer\n");
            char * sensor_data_buffer = malloc(MAX_SENSOR_PACKET_SIZE);
            double ts_start=0;
            double ts_end=0;
            uint32_t datasets=0;
            double ts_pos = 0.; //For nav data sources that does not give full time, this might need to be popolated
            double ts_sensor;
            uint32_t datapoints = 0;
            uint32_t force_bath_version = 0;

            while(1){
                uint32_t sensor_data_buffer_len = sensor_fetch_next_packet(sensor_data_buffer, input_fd, sensor_mode);
                //fprintf(stderr,"sensor_data_buffer_len = %d\n",sensor_data_buffer_len);fflush(stderr);
                if(sensor_data_buffer_len==0) break;
                int new_sensor_data = sensor_identify_packet(sensor_data_buffer,sensor_data_buffer_len,ts_pos, &ts_sensor, sensor_mode);
                //fprintf(stderr,"new_sensor_data = %d\n",new_sensor_data);

                if (new_sensor_data){ 
                    uint32_t cnt=0;
                    switch (sensor_mode){
                        case sensor_mode_wbms: case sensor_mode_wbms_v5:
                            if (new_sensor_data == PACKET_TYPE_BATH_DATA){
                                cnt = wbms_count_data( (bath_data_packet_t*) sensor_data_buffer, force_bath_version,&ts_sensor);
                            }
                            else if (new_sensor_data == PACKET_TYPE_SNIPPET_DATA){
                                cnt = wbms_count_snippet_data( (snippet_data_packet_t*) sensor_data_buffer, &ts_sensor);
                            }
                            else if (new_sensor_data == PACKET_TYPE_SBP_DATA){
                                cnt = wbms_count_sbp_data( (sbp_data_packet_t*) sensor_data_buffer,&ts_sensor);
                            }
                            else{
                                cnt = 0;
                            }
                            break;
                        case sensor_mode_sim:
                            cnt = 0;
                            break;
                        case sensor_mode_velodyne:
                            cnt = velodyne_count_data( (uint16_t *) sensor_data_buffer,&ts_sensor);
                            break;
                        case sensor_mode_lakibeam:
                            cnt = lakibeam_count_data( (uint16_t *) sensor_data_buffer,&ts_sensor);
                            break;
                        case sensor_mode_s7k:
                            cnt = s7k_count_data( sensor_data_buffer,sensor_data_buffer_len,&ts_sensor);
                            break;
                        case sensor_mode_gsf:
                            cnt = gsf_count_data( sensor_data_buffer,sensor_data_buffer_len,&ts_sensor);
                            break;
                        case sensor_mode_3dss_stream:
                            //cnt = p3dss_count_data( sensor_data_buffer,sensor_data_buffer_len,&ts_sensor);
                            break;
                        default:
                            cnt = 0;
                    }
                    if(datasets==0){
                        ts_start = ts_sensor;
                    }
                    if(cnt){
                        datasets++;
                        datapoints += cnt;
                    }
                    //fprintf(stderr, "Datapoints = %d\n",datapoints);
                }
            }


            ts_end = ts_sensor;

            free(sensor_data_buffer);
            /* Write file stats */
            if (file_stats==NULL){ // If file stats is filled out for the first time, add these fields
                uint32_t num_record_types = sensor_num_record_types(sensor_mode);
                file_stats= calloc(1,sizeof(file_stats_t) + num_record_types*sizeof(record_count_t));

                file_stats->file_type = sensor_mode_short_names[sensor_mode];     
                file_stats->file_version = (version>0)?ver_string:NULL;  
                file_stats->sensor_type = "unknown";   
                file_stats->num_record_types = sensor_get_record_count(sensor_mode,file_stats->records);
            }
            file_stats->data_type = sensor_get_data_type(sensor_mode);     
            file_stats->has_sensor = 1;
            file_stats->datapoints = datapoints;
            file_stats->datasets = datasets;

            file_stats->start_time = ts_start;
            file_stats->duration = ts_end - ts_start;
            /*file_stats->mean_depth;
              file_stats->coverage_area;*/
            sensor_count_stats_t* count_stats = sensor_get_count_stats(sensor_mode);
            if (count_stats){
                fprintf(stderr, "Count stats freq=%f bw=%f\n", count_stats->freq, count_stats->bw);
                file_stats->freq = count_stats->freq;
                file_stats->bandwidth = count_stats->bw;
            }
        }
    }
    #endif


    fclose(input_fileptr);
    /************ Generate JSON  ******************/
    if (file_stats==NULL){
        fprintf(stderr,"Could not parse input file\n");
        return(-1);
    }
    
    char* json_buf = malloc(2048);
    write_stats_json_to_buffer(file_stats, /*OUTPUT*/json_buf);
    fprintf(stdout,"%s",json_buf);
    
    if(file_stats) free(file_stats);
    free(json_buf);

    return(0);
}







