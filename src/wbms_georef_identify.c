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


static char verbose = 0;

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
enum time_ref_e {tref_line=1, tref_day=2, tref_week=3, tref_unix=0};

static const char *input_source_names[] = {
	"None",
	"TCP",
	"UDP",
	"stdin",
	"File",
	"zero",
	"sim",
};


int main(int argc,char *argv[])
{


	//File input and output stuff
	FILE * input_fileptr = NULL;

	int input_fd;
	
	uint32_t len;
	uint32_t sensor_data_buffer_len = 0;
	uint32_t output_databuffer_len = 0;
	uint32_t navigation_data_buffer_len = 0;
	
	uint32_t navigation_total_packets = 0;
	uint32_t sensor_data_packet_counter = 0;
	uint32_t navigation_total_data = 0;
	uint32_t sensor_total_data = 0;
	uint32_t output_total_data = 0;
    uint32_t sensor_total_datapoints = 0;

	uint32_t datapoints;

	double ts_pos = 0.;
	double ts_sensor = 0.;
   
    static char * input_file_string = NULL;

    file_stats_t* file_stats= NULL;

    r7k_init();
    posmv_init();

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

    fprintf(stderr,"Reading sensor data from FILE: %s\n",input_file_string);
    input_fileptr = fopen(input_file_string,"rb");
    if (input_fileptr == NULL){
        fprintf(stderr,"Could not open sensor data file%s\n",input_file_string);
        exit(-1);
    }
    

    
    /************ Parse file as navigation data file ******************/
    pos_mode = navigation_autodetect_file(input_fileptr);
    fprintf(stderr,"pos_mode = %d\n", pos_mode);
    if (pos_mode != pos_mode_unknown){
        double ts_sensor = 0.; //For nav data sources that does not give full time, thios might need to be popolated
        double ts_pos;
        char navigation_data_buffer[MAX_NAVIGATION_PACKET_SIZE];
        
        int input_navigation_fd = fileno(input_fileptr);
        
        // Read out first position to derive UTM zone for data
        while (1){
            uint32_t navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_navigation_fd,pos_mode);
            if(navigation_data_buffer_len>0){

                int new_nav_data = process_nav_data_packet(navigation_data_buffer,navigation_data_buffer_len,ts_sensor, &ts_pos,pos_mode,0);  
                if( new_nav_data){
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
                    break;
                }
            }
        }
        if(proj_latlon_to_output_utm==NULL){
            fprintf(stderr, "ERROR: Could not find any position data in navigation data stream\n");
            return(-1);
        }

        fseek(input_fileptr, 0, SEEK_SET); //Rewind
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
        double prev_x,prev_y;
        double x_start, y_start, z_start;
        double x_end, y_end, z_end;
        float acum_dist = 0;
        uint32_t navigation_data_buffer_len;

        double lat,lon;
        double x,y,z;

        while (navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_navigation_fd,pos_mode)){
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
        
        /* Write file stats */
        if (file_stats==NULL){ // If file stats is filled out for the first time, add these fields
            uint32_t num_record_types = r7k_num_record_types();
            file_stats= calloc(1,sizeof(file_stats_t) + num_record_types*sizeof(record_count_t));
        
            file_stats->file_type = "s7k";     
            file_stats->file_version = "0";  
            file_stats->data_type = "mbes";     
            file_stats->sensor_type = "unknown";   
            file_stats->num_record_types = r7k_get_record_count(&(file_stats->records));
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
    fseek(input_fileptr,0,SEEK_SET);
   

    /************ Parse file as sensor data file ******************/
    sensor_mode = sensor_autodetect_file(input_fileptr);
    if (sensor_mode != sensor_mode_unknown){
        //TODO Code to parse sensor data here



        /* Write file stats */
        if (file_stats==NULL){ // If file stats is filled out for the first time, add these fields
            uint32_t num_record_types = r7k_num_record_types();
            file_stats= calloc(1,sizeof(file_stats_t) + num_record_types*sizeof(record_count_t));
            
            file_stats->file_type = "s7k";     
            file_stats->file_version = "0";  
            file_stats->data_type = "mbes";     
            file_stats->sensor_type = "unknown";   
            file_stats->num_record_types = r7k_get_record_count(&(file_stats->records));
        }

        file_stats->has_sensor = 1;
        /*file_stats->datapoints;
        file_stats->start_time = ts_start;
        file_stats->duration ts_end - ts_start;
        file_stats->mean_depth;
        file_stats->coverage_area;*/
    }


    /************ Generate JSON  ******************/
    if (file_stats==NULL){
        fprintf(stderr,"Could not parse input file\n");
        return(-1);
    }
    
    char* json_buf = malloc(2048);
    write_stats_json_to_buffer(file_stats, /*OUTPUT*/json_buf);
    fprintf(stdout,"%s",json_buf);

    fclose(input_fileptr);
    return(0);
}







