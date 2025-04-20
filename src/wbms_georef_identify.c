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
    sensor_mode = sensor_autodetect_file(input_fileptr);
    if (sensor_mode != sensor_mode_unknown){
        //TODO Code to parse sensor data here
    }

    fseek(input_fileptr,0,SEEK_SET);
    
    pos_mode = navigation_autodetect_file(input_fileptr);
    if (pos_mode != pos_mode_unknown){
        //TODO Code to parse navigation data here
    }

   
    fclose(input_fileptr);
    return(0);
}







