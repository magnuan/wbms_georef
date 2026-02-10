#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
#include <time.h>
#include <string.h>

#ifdef __unix__
#include <unistd.h>
#define ENABLE_NETWORK_IO
#endif


#ifdef ENABLE_NETWORK_IO
#ifdef __unix__
#include <arpa/inet.h>
#include <netdb.h>
#else
#include <winsock2.h>
#endif
#include <errno.h>
#endif

#include "cmath.h"
#include <math.h>

#include "wbms_georef.h"
#include "georef_tools.h"
#include "raytracing.h"
#include "time_functions.h"
#include "sim_data.h"
#include "wbms_data.h"
#include "3dss_dx.h"
#include "reson7k.h"
#include "velodyne.h"
#include "lakibeam.h"
#include "gsf_wrapper.h"
#include "posmv.h"
#include "xtf_nav.h"
#include "sim_nav.h"
#include "sbet_nav.h"
#include "eelume_sbd_nav.h"
#include "nmea_nav.h"
#include "wbm_tool_nav.h"
#include "sbf_output.h"
#include "nmea_output.h"
#include "json_output.h"
#include "csv_output.h"
#include "bin_output.h"
#include "reson7k_output.h"
#include "intensity_scaling.h"
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


#ifdef ENABLE_NETWORK_IO
#include <winsock2.h>
#include <ws2tcpip.h>

// Need to link with Ws2_32.lib
#pragma comment(lib, "ws2_32.lib")

int init_windows_winsock_dll(void)
{
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

    /* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        /* Tell the user that we could not find a usable */
        /* Winsock DLL.                                  */
        fprintf(stderr,"WSAStartup failed with error: %d\n", err);
        return 1;
    }

    /* Confirm that the WinSock DLL supports 2.2.*/
    /* Note that if the DLL supports versions greater    */
    /* than 2.2 in addition to 2.2, it will still return */
    /* 2.2 in wVersion since that is the version we      */
    /* requested.                                        */
    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
        /* Tell the user that we could not find a usable */
        /* WinSock DLL.                                  */
        fprintf(stderr,"Could not find a usable version of Winsock.dll\n");
        WSACleanup();
        return 1;
    }
    else
        fprintf(stderr,"The Winsock 2.2 dll was found okay\n");

    /* The Winsock DLL is acceptable. Proceed to use it. */
    /* Add network programming using Winsock here */
    /* then call WSACleanup when done using the Winsock dll */
    return 0;
}

void close_windows_winsock_dll(void){
    WSACleanup();
}

#endif
#endif

#define MAX_CLIENTS 8


static char verbose = 0;

#define PREFER_HEAP

#ifdef ENABLE_NETWORK_IO
    #ifdef __unix__
        #define TCP_SEND_FLAGS (MSG_NOSIGNAL)
    #else
        #define TCP_SEND_FLAGS (0)
        //#define bzero(a,b) (ZeroMemory(a,b))
        #define bzero(s, n) memset((s), 0, (n))
        #define bcopy(s1, s2, n) memmove((s2), (s1), (n))
    #endif
#else
    //typedef int fd_set;
    int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout){ 
        return 0;
    };
    #undef FD_ZERO
    #undef FD_ISSET
    #undef FD_SET
    #undef FD_CLR
    #define FD_ZERO(a)
    #define FD_ISSET(a,b)  (0)
    #define FD_SET(a,b)  
    #define FD_CLR(a,b)  
#endif

#ifdef COUNT_S7K_SNIPPET_SATURATION
uint32_t s7k_snp_satcount=0;
uint32_t s7k_snp_count=0;
#endif

static offset_t sensor_offset;
static sensor_params_t sensor_params;

static uint32_t  force_bath_version = 0;
static float sim_data_period = 0.5;

static char output_projection_string[256];
static char input_projection_string[256];
static float input_timezone = 0;           
	
static char * input_sensor_source_string = NULL;
static char * input_navigation_source_string = NULL;
static char * sv_file_name = NULL;
static char * angle_intensity_file_name = NULL;
static char * output_string = NULL;
            

typedef enum  {output_binary=0, output_csv=1, output_sbf=2, output_nmea=3, output_json=4, output_s7k=5} output_mode_e;
static output_mode_e output_mode = output_sbf;
const char *output_mode_name[] = {"BINARY","CSV","SBF","NMEA","JSON","S7K"};
output_format_e output_format[MAX_OUTPUT_FIELDS];

pos_mode_e pos_mode = pos_mode_autodetect;
sensor_mode_e sensor_mode = sensor_mode_autodetect;

double z_off = 0;
static int projection_header=1;
static const char *ray_tracing_mode_names[] = {
    "None",
	"Fixed depth LUT",
	"Fixed depth direct",
	"Variable depth direct",
};

static const char *alt_mode_names[] = {
	"None",
	"GPS altitude",
	"Heave",
};

uint8_t force_nav_epoch = 0;
uint8_t nav_epoch_set = 0;
double nav_epoch_week_ts = 0;



/* COORDINATE SYSTEM
/  X = Northing / Forward
/  Y = Easting / Starboard
/  Z = Down / Down
/  Roll around pos X, 
/  Pitch around pos Y, 
/  Yaw around pos Z
/
/ Output to X=easting, Y=northing, Z=up
*/

//WBMS sonar inherently converts to polar coordinates, CONE-PLATE
// Uncomment this to treat sonar as two-cone-intersect
//#define CONE_CONE_COORD

void error(const char *msg)
{
	perror(msg);
	exit(0);
}




void showUsage(char *pgmname)
{
		printf("Usage: %s [OPTIONS] \n", pgmname);
		printf(	"Starts the wbms_georef.\n"
			"Program collecting data from WBMS (bathy) PosMV and Velodyne (lidar), and combining it into georeference points in a X,Y,Z format\n"
			"PLEASE RUN WITH -x FIRST TO GENERATE A TEMPLATE CONFIG, THIS ALSO CONTAINS MORE USAGE INFO\n"
			"Input and outputs can be stdin/stdout, file, TCP or UDP ports\n"
			//"TCP ports are entered: Ex. 192.168.53.44:2210\n"
			//"UDP ports are entered: Ex. 192.168.53.100#5602\n"
			//"File is entered as filename.dump\n"
			//"Std in/out is denoted with dash (-)\n"
			"\t-i source\t Common data source\n"  
			"\t-s source\t Sensor data source\n"  
			"\t-S mode\t Sensor mode: 1=WBMS 2=WBMS_V5 3=Velodyne 4=SIM 5=S7K 10=Autodetect (default) \n"
			"\t-p source\t Pos data source (PosMv:5602)\n"
			"\t-P mode\t Pos mode: 0=POSMV 1=XTF_NAV 2=WBM_TOOL 3=SBET 4=SIM 5=S7K 10=Autodetect (default)\n"
			"\t-r roll\t Roll offset in degrees(additional to config file offset)\n"
			"\t-d draft\t Sonar draft in meters (overriding config file value)\n"
			"\t-w source\t Sound velocity input file, for raybending correcting\n"
			"\t-y source\t Angle intensity correction table\n"
			"\t-5 \t Force wbms bathy format version 5\n"
			"\t-o file\t Output drain\n"
			"\t-C \t Output in csv format\n"
			"\t-7 \t Output in s7k format\n"
			"\t-V \t Verbose mode\n"
			"\t-x \t Generate template config file, wbms_georef.conf.template\n"
			"\t-c config_file \t Read in configuration from file\n"
			"\t-F freq_index \t Index of multifreq set to use (-1 for all, default)\n"
			"\t-S speed \t Simulated speed in m/s for simulated nav data\n"

			"\t Sound velocity file should be comma separated sound velocity [m/s] and depth [m], one measurement per line\n"
			"\t Angle intensity file should be comma separated angle [deg] and val [mean_measured_intensity] , one measurement per line\n"
			);
}


void generate_template_config_file(char* fname){
	FILE * fp = fopen(fname,"w");
	fprintf(fp,"# WBMS georef, Template config file\n\n");
	

	
	fprintf(fp,"#### SENSOR MOUNTING ####\n");
	fprintf(fp,"# Linear offsets are given in meters X=pos.forward, Y=pos.starboard, Z=pos.down\n");
	fprintf(fp,"# Rotation offsets are given in degrees, Yaw=pos.turning starboard, Pitch=pos.tilting up, Roll=Rotating CW\n\n");
	fprintf(fp,"# Sonar offset and roll relative to Navigation origo. 0,0,0 attitude is Rx facing downwards, Tx pointing aft\n");
	fprintf(fp,"sensor_x_offset 0.0\n");
	fprintf(fp,"sensor_y_offset 0.0\n");
	fprintf(fp,"sensor_z_offset 0.0\n");
	fprintf(fp,"sensor_yaw_offset 0.0\n");
	fprintf(fp,"sensor_pitch_offset 0.0\n");
	fprintf(fp,"sensor_roll_offset 0.0\n\n");
	
	fprintf(fp,"#### SENSOR ERROR COMPENSATION ####\n");
	fprintf(fp,"# Add a fixed range adjustment to sonar data\n");
    fprintf(fp,"#sensor_r_error 0.0\n");
	fprintf(fp,"# Add a range scaling adjustment to sonar data\n");
    fprintf(fp,"#sensor_r_scaling 0.0\n");
	fprintf(fp,"# Add a time offset to sensor data timestamp\n");
	fprintf(fp,"#sensor_time_offset 0.0\n");
	fprintf(fp,"# Add a correction value to sonar SV measurement\n");
	fprintf(fp,"#sensor_sv_offset 0.0\n\n");
	fprintf(fp,"# Force specific SV value\n");
	fprintf(fp,"#sensor_force_sv 0.0\n\n");
	
    fprintf(fp,"#### SENSOR DATA FILTERS ####\n");
	fprintf(fp, "# Sounding quality limits\n");
	fprintf(fp,"sensor_min_quality_flag 3\n");
	fprintf(fp,"sensor_max_quality_flag 4\n");
	fprintf(fp, "# Sounding priority limits\n");
	fprintf(fp,"sensor_min_priority_flag 0\n");
	fprintf(fp,"sensor_max_priority_flag 4\n");
	fprintf(fp, "# Sounding multifreq select (-1 for all)\n");
	fprintf(fp,"sensor_multifreq_index -1\n");
	fprintf(fp, "# First bathy ata set to process, and max number of sets (0 fir all)\n");
	fprintf(fp,"sensor_data_skip 0\n");
	fprintf(fp,"sensor_data_length 0\n");
	fprintf(fp, "# Global depth limits\n");
	fprintf(fp,"sensor_min_depth -10000\n");
	fprintf(fp,"sensor_max_depth 10000\n");
	fprintf(fp,"sensor_min_range 0\n");
	fprintf(fp,"sensor_max_range 10000\n");
	fprintf(fp, "# Angular limits wrt sensor\n");
	fprintf(fp,"sensor_min_azimuth -90\n");
	fprintf(fp,"sensor_max_azimuth 90\n");
	fprintf(fp,"sensor_min_elevation -90\n");
	fprintf(fp,"sensor_max_elevation 90\n");
	fprintf(fp,"sensor_min_ping_number 0\n");
	fprintf(fp,"sensor_max_ping_number 0\n");
	fprintf(fp, "# Limits wrt swath nadir\n");
	fprintf(fp,"sensor_swath_min_y -1000\n");
	fprintf(fp,"sensor_swath_max_y 1000\n\n");
	fprintf(fp, "# Decimate sensor data\n");
	fprintf(fp,"# sensor_beam_decimate 1\n");
	fprintf(fp,"# sensor_ping_decimate 1\n");
            

    fprintf(fp,"#### NAVIGATION DATA FILTERS ####\n");
    fprintf(fp,"# Set to non-zero to enable \n");
    fprintf(fp,"# roll,pitch,yaw in deg \n");
    fprintf(fp,"# droll_dt,dpitch_dt,dyaw_dt in deg/sec \n");
    fprintf(fp,"navigation_max_abs_roll 0\n");
    fprintf(fp,"navigation_max_abs_pitch 0\n");
    fprintf(fp,"navigation_max_abs_yaw 0\n");
    fprintf(fp,"navigation_max_abs_droll_dt 0\n");
    fprintf(fp,"navigation_max_abs_dpitch_dt 0\n");
    fprintf(fp,"navigation_max_abs_dyaw_dt 0\n");


	fprintf(fp,"#### RAY TRACING PARAMETERS ####\n");
	fprintf(fp,"# Ray tracing is always using sonar measured SV as inital SV\n");
	fprintf(fp,"# Ray tracing mode and Mounting depth used for SV-prfile correction (raytracing).\n");
	fprintf(fp,"#  Ray tracing mode 0: None\n"); 
	fprintf(fp,"#  Ray tracing mode 1: Fixed depth, LUT.    Using a look up table for ray tracing adjustments. Assuming sensor at fixed depth (sonar_mounting_depth) wrt SV-profile 0\n");
	fprintf(fp,"#  Ray tracing mode 2: Fixed depth, Direct. Using  direct ray tracing. Assuming sensor at fixed depth (sonar_mounting_depth) wrt SV-profile 0\n");
	fprintf(fp,"#  Ray tracing mode 3: Variable depth, Direct. Using  direct ray tracing. Assuming sensor at variable depth based on navigation (nav+offset+sonar_mounting_depth) wrt SV-profile 0\n");
	fprintf(fp,"ray_tracing_mode 1\n");
	fprintf(fp,"# Sensor mounting depth im meters. With respect to SV-profile depth value\n");
	fprintf(fp,"sensor_mounting_depth 1\n");
	fprintf(fp,"# Which sv value to use for initial ray parameter when doing raytracing\n");
	fprintf(fp,"raytrace_use_sonar_sv\n");
	fprintf(fp,"#raytrace_use_table_sv\n\n");


	fprintf(fp,"#### BACKSCATTER PARAMETERS ####\n");
	fprintf(fp,"# Snippet processing mode.  0: Sqrt Mean power.  1: Sqrt Sum power (energy).  2: Detection intensity, 3: Sqrt mean power within 3dB footprint, 4: Sqrt Sum power (energy) within 10dB footprint, 5:Snippet max\n");
	fprintf(fp,"snippet_processing_mode 0\n");
	fprintf(fp,"# Backscatter source for s7k records. 0: Bathy record,  1: Snippets (7028)  2: Normalized snippets (7058)\n");
	fprintf(fp,"s7k_backscatter_source 0\n");
	fprintf(fp,"# By default applied S7K Gain and TVG (in 7000 record) is removed, uncomment to keep s7k applied gain/TVG\n");
	fprintf(fp,"# keep_s7k_tvg \n");
	fprintf(fp,"# By default, the 7058 footprint compensation is removed and replaced with the internal footprint model, uncomment to keep 7058 footprint instead\n");
	fprintf(fp,"# keep_s7k_footprint \n");
	fprintf(fp,"# With this enabled, the backscatter is compensated for attenuation, spreading and footprint\n");
	fprintf(fp,"# Comment to disable\n");
	fprintf(fp,"intensity_correction\n");
	fprintf(fp,"# Damping / attenuation in dB/km one-way when applying intensity range comp (Default = 0, means attenuation is based on frequency)\n");
	fprintf(fp,"#intensity_range_attenuation  0\n\n");
	fprintf(fp,"# Beamwidth parameters used for footprint calculation\n");
	fprintf(fp,"rx_nadir_beamwidth 0.5 \n\n");
	fprintf(fp,"tx_nadir_beamwidth 1.0 \n\n");
	fprintf(fp,"# ARA curve compensation\n");
	fprintf(fp,"# 0: No ARA compensation.  1: Incidence angle comp only 10*log10(cos(aoi))  2: sandy gravel  3: sandy gravel w/some mud 4: gravelly muddy sand  5: muddy sand   6: gravelly mud   7: clay\n");

	fprintf(fp,"# If intensity compensation table is given as a CSV file with the -y option, this is ignored\n");
	fprintf(fp,"ara_model 0\n");
    fprintf(fp,"# This is a multiplier applied to the ara model (scaling the model in dB), uncomment and set different from 1.0, to tune the ara model\n");
	fprintf(fp,"#ara_model_multiplier 1.0\n");
	fprintf(fp,"# Calculaste angle of incidence:   1: calculate aoi from data (default)  0: assume flat seafloor\n");
	fprintf(fp,"calc_aoi 1\n\n");


	
	fprintf(fp,"#### ALTERNATIVE OUTPUT ####\n");
    fprintf(fp,"# Uncomment to process with upper gate value instead of detection value\n");
	fprintf(fp,"#upper_gate\n\n");
	fprintf(fp,"# Uncomment to process with lower gate value instead of detection value\n");
	fprintf(fp,"#lower_gate\n\n");
	fprintf(fp,"# Uncomment to process with center gate value instead of detection value\n");
	fprintf(fp,"#center_gate\n\n");


	fprintf(fp,"#### SENSOR DATA FORMAT ####\n");
	fprintf(fp,"# Sensor mode 1=WBMS 2=WBMS_V5 3=Velodyne 4=SIM 5=S7K 6=PingDSP_stream 10=Autodetect (default)\n");
	fprintf(fp,"# sensor_mode 10\n");
    fprintf(fp,"# Uncomment to force wbms bathy data to be read as specific version\n");
    fprintf(fp,"#force_bath_version 0\n\n");
    fprintf(fp,"# Uncomment to specify minimum period between sim data generation\n");
    fprintf(fp,"#sim_data_period  0.5\n\n");

	fprintf(fp,"#### NAVIGATION DATA FORMAT ####\n");
	fprintf(fp,"# Nav input parameters \n");
	fprintf(fp,"# Pos mode 0=POSMV 1=XTF_NAV 2=WBM_TOOL 3=SBET 4=SIM 5=S7K 6=PingDSP_stream 10=Autodetect (default)\n");
	fprintf(fp,"# pos_mode 10\n");
	fprintf(fp,"# Set forward speed in m/s when using simulated navigation data \n");
	fprintf(fp,"pos_sim_speed 1\n\n");
	fprintf(fp,"# SBET files and POSMV files without Grop3 only contains GPS time-of-week\n");
	fprintf(fp,"# With sensor data, the week will be chosen from sensor data time\n");
	fprintf(fp,"# Without sensor data  (sim mode), the week will be set to week of unix epoc\n");
	fprintf(fp,"# To force a specific week, enable and enter a unix timestamp value within that week to the value below\n");
	fprintf(fp,"# nav_epoch_week 0\n\n");
	fprintf(fp,"# Altitude mode 0=none 1=gps altitude, 2=heave \n");
	fprintf(fp,"alt_mode 1\n");
	fprintf(fp,"vert_offset 0\n");
	fprintf(fp,"input_timezone 0\n");
	fprintf(fp,"# Maximum allowed time difference between sensor and navigation data\n");
	fprintf(fp,"time_diff_limit 0.2\n");
	fprintf(fp,"# Only for navigation data in projected coordinates, specify navigation data projection parameters\n");
	fprintf(fp,"# projection_input +proj=utm +zone=33 +ellps=WGS84\n\n\n");
	
    fprintf(fp,"#### SPECIAL OPTIONS ####\n");
	fprintf(fp,"# Motion stabilized SBP. Set to 1 to assume sub bottom profiler has perfectlymotion stabilized tx \n");
	fprintf(fp,"# sbp_motion_stab 1\n");
	fprintf(fp,"# Output raw SBP data. Set to 1 to output sbp raw data without any processing (filtering / rectification)  \n");
	fprintf(fp,"# sbp_raw_data 1\n");
    fprintf(fp,"# Sub bottom profiler bandpass filter, set start and stop frequency in kHz to enable\n");        
    fprintf(fp,"# sbp_bandpass_filter 0 0\n");
	fprintf(fp,"# Scale tx steering angle (tx angle) from sonar data (default 1.0)\n");
	fprintf(fp,"# scale_tx_angle 1.0\n");

			
    fprintf(fp,"# Beam angle correction as polynom. Model for beam angle correction as a Taylor series, comma separated in radians [rad, rad/rad, rad/rad^2, etc...] maximum %d parameters\n" , MAX_BEAM_ANGLE_MODEL_ORDER);        
    fprintf(fp,"# Ex. beam_correction_poly 7.58391e-04, 2.25886e-03, -9.76763e-04, -1.99206e-03     for a 4th ordel model with the given parameters\n");
    fprintf(fp,"# beam_correction_poly \n");

    fprintf(fp,"# Beam angle correction table. Beam angle corrections in degrees. Assumed to be for angles uniformly distributed from -90 to +90 degrees. maximum %d parameters\n", MAX_BEAM_ANGLE_TABLE_LEN);
    fprintf(fp,"# Ex. beam_correction_table 0.10, 0.05, 0.00, 0.05, 0.10\n");
    fprintf(fp,"# beam_correction_table \n");

    fprintf(fp,"# NOTE: Only one beam correction poly or table can be defined. If multiple entries in config file, only the last will be used");
	
	fprintf(fp,"#### DATA SOURCE ####\n");
	fprintf(fp,"# Normally given as input argument with -i or -p and -s, but can also be specified here\n");
	fprintf(fp,"# Define source and output drain: '-' for stdin/stdout ':' separated TCP ip/port '#' separated UDP ip/port else filename\n");
	fprintf(fp,"# sensor_source localhost:2210\n");
	fprintf(fp,"# navigation_source localhost#5602\n");
	fprintf(fp,"# output localhost:9999\n\n\n");
	
	fprintf(fp,"#### OUTPUT DATA FORMAT ####\n");
	fprintf(fp,"# Uncomment to output in CSV format\n");
	fprintf(fp,"#csv_output\n");
	fprintf(fp,"# Uncomment to output in SBF format (CloudCompare)\n");
	fprintf(fp,"sbf_output\n");
	fprintf(fp,"# Uncomment to output in BIN format\n");
	fprintf(fp,"#bin_output\n");
	fprintf(fp,"# Uncomment to output in NMEA format (vessel navigation data only)\n");
	fprintf(fp,"#nmea_output\n");
	fprintf(fp,"# Uncomment to output in JSON format (vessel navigation data only)\n");
	fprintf(fp,"#json_output\n");
	fprintf(fp,"# Uncomment to output in rudamentary Reson s7k format\n");
	fprintf(fp,"#s7k_output\n");
    fprintf(fp,"# Write projection header to file output, only for CSV output\n");
    fprintf(fp,"projection_header true\n\n");
	fprintf(fp,"# Coordinate system definition string, as described in http://proj4.org/ Leave commented for auto detect\n");
	fprintf(fp,"# projection_output +proj=utm +zone=33 +ellps=WGS84\n\n\n");

    /* CUSTOM_CSV_OUTPUT */
	fprintf(fp,"#### OUTPUT DATA FORMAT FIELDS ####\n");
    fprintf(fp,"# Output format string for SBF and CSV files\n");
    fprintf(fp,"#  time stamp:                t\n");
    fprintf(fp,"#  detection coordinates:     x,y,z\n");
    fprintf(fp,"#  detection variance:        z_var, z_stddev\n");
    fprintf(fp,"#  detection beam angle:      teta\n");
    fprintf(fp,"#  detection range :          range\n");
    fprintf(fp,"#  beam number :              beam\n");
    fprintf(fp,"#  upper_gate_dist :          ugate\n");
    fprintf(fp,"#  lower_gate_dist :          lgate\n");
    fprintf(fp,"#  center_gate_dist :         cgate\n");
    fprintf(fp,"#  gate width :               gatew\n");
    fprintf(fp,"#  tx frequency :             freq\n");
    fprintf(fp,"#  tx bandwidth :             bw\n");
    fprintf(fp,"#  tx pulselength :           plen\n");
    fprintf(fp,"#  tx voltage :               voltage\t  #For reson s7k this is tx power in dB rel 1uPa\n");
    fprintf(fp,"#  pingrate :                 pingrate\n");
    fprintf(fp,"#  gain :                     Sonar processing gain\t #Not in use for reson s7k\n");
    fprintf(fp,"#  multiping index :          multiping\n");
    fprintf(fp,"#  multifreq index :          multifreq\n");
    fprintf(fp,"#  ping number :              pingnumber\n");
    fprintf(fp,"#  ping number diff:          pingdiff\n");
    fprintf(fp,"#  detection beam angle (electronic steering part):      steer\n");
    fprintf(fp,"#  detection beam tx angle:    el\n");
    fprintf(fp,"#  detection intensity value: val\n");
    fprintf(fp,"#  detection intensity value (dB): val_dB\n");
    fprintf(fp,"#  detection swath y offset:  swath_y\n");
    fprintf(fp,"#  detection angle of incidence:  aoi\n");
    fprintf(fp,"#  detection quality:         quality\n");
    fprintf(fp,"#  detection priority:         priority\n");
    fprintf(fp,"#  detection strength:        strength\n");
    fprintf(fp,"#  snippet length:            snp_len\t  #Snippet length in samples \n");
    fprintf(fp,"#  footprint time (s):        footprint_time\t # in ms for CSV output\n");
    fprintf(fp,"#  footprint size (m2):       footprint\n");
    fprintf(fp,"#  vessel coordinates:        LAT,LON\n");
    fprintf(fp,"#  vessel coordinates:        X,Y,Z\n");
    fprintf(fp,"#  vessel altitude (=Z):      ALTITUDE\n");
    fprintf(fp,"#  vessel altimeter :         ALTIMETER\t #Altimeter data (from s7k record 1006) \n" );
    fprintf(fp,"#  vessel depth :             DEPTH\t #Depth data (from s7k record 1008) \n" );
    fprintf(fp,"#  vessel attitude:           YAW,PITCH,ROLL\n");
    fprintf(fp,"#  vessel pos accuracy:       HOR_ACC,VERT_ACC\n");
    fprintf(fp,"#  vessel speed:              SPEED\n");
    fprintf(fp,"#  vessel movement (meter per ping):  MOVEMENT\n");
    fprintf(fp,"#  vessel course:             COURSE\n");
    fprintf(fp,"#  vessel satellites:         SATELLITES\n");
    fprintf(fp,"#  vessel accuracy(=HOR_ACC): GPS_ACCURACY\n");
    fprintf(fp,"#  speed of sound:            c\n");
    fprintf(fp,"#  intensity noise ref:       iref\n");
    fprintf(fp,"#  strength noise ref:        sref\n");
    fprintf(fp,"#  classification:            classification\n");
    fprintf(fp,"#  line_of_sight_vector (3 values):  line_of_sight\n");
    fprintf(fp,"output_format x,y,z,val\n");
	
	fprintf(fp,"# BIN output format is [(uint16) preamble , (uint16) type, (float) val, (double) x, (double) y, (double) z] where preamble is 0xBEEF \n");
	fprintf(fp,"# Here X,Y,Z depends on projection but will typically be in meters Easting, Northing, Altitude\n");
	fprintf(fp,"# Val is intensity values, as given by either sonar or lidar\n");
    
	fprintf(fp,"# Autocalib required fields \n");
    fprintf(fp,"# output_format x,y,z,teta,el,range,YAW,PITCH,ROLL,Z\n");
        


	fclose(fp);
}


static void sensor_params_default(sensor_params_t* s){
    s->min_quality_flag=3;
    s->max_quality_flag=4;
    s->min_priority_flag=0;
    s->max_priority_flag=4;
    s->multifreq_index=-1;
    s->data_skip = 0;     //Number of datasets to skip 
    s->data_length = 0;   //Number of datasets to process (0 for all)
    s->min_depth=-10000;
    s->max_depth=10000;
    s->min_range=2;
    s->max_range=10000;
    s->min_azimuth=-90*(float)M_PI/180;
    s->max_azimuth= 90*(float)M_PI/180;
    s->min_elevation=-90* (float)M_PI/180;
    s->max_elevation= 90* (float)M_PI/180;
    s->min_ping_number = 0;
    s->max_ping_number = 0;
    s->swath_min_y = -1000;
    s->swath_max_y = 1000;
    s->sv_offset = 0.0;
    s->force_sv = 0.0;
    s->mounting_depth = 1.0;		//Mounting depth, only for raytracing, SV-profile compensation
    s->intensity_correction = 0;
    s->intensity_range_attenuation = 0;
    s->ara_model = ara_model_none;
    s->ara_model_multiplier = 1.0;
    s->rx_nadir_beamwidth= 0.5*(M_PI/180);
    s->tx_nadir_beamwidth = 1.0*(M_PI/180);
    s->calc_aoi = 1;
    s->sonar_sample_mode = detection;
    s->ray_tracing_mode = ray_trace_fixed_depth_lut;
    s->beam_decimate = 1;
    s->ping_decimate = 1;

    s->max_abs_roll=0;
    s->max_abs_pitch=0;
    s->max_abs_yaw=0;
    s->max_abs_droll_dt=0;
    s->max_abs_dpitch_dt=0;
    s->max_abs_dyaw_dt=0;
    s->sbp_motion_stab = 0;
    s->sbp_raw_data = 0;
    s->sbp_bp_filter_start_freq = 0;
    s->sbp_bp_filter_stop_freq = 0;
    s->scale_tx_angle = 1.0;
    s->s7k_backscatter_source = s7k_backscatter_bathy;
    s->keep_s7k_tvg = 0;
    s->keep_s7k_footprint_comp = 0;
    s->snippet_processing_mode = snippet_mean_pow;
    s->tx_sensitivity = powf(10,(220/20)-6); //220dB uPa / V => 100000Pa/V 
    s->rx_sensitivity = powf(10,-195/20 +6); //-190dB V/uPa = 0.0001778 V/Pa
    s->rx_processing_gain = 1000.;         //TODO find a value, Just guesswork so far  LSB/V
}




void fprint_output_format(FILE* f,output_format_e* format){
    int cnt;
    output_format_e* fptr = format;
    for(cnt = 0;cnt < MAX_OUTPUT_FIELDS;cnt++){
        if(*fptr == none){
            fprintf(f,"\n");
            return;
        }
        if(cnt) fprintf(f,",");
        switch (*fptr){
#define foo(x) case x: fprintf(f,STRINGIFY(x));break
            iterate_output_format(foo);
#undef foo
            default: break;
        }
        fptr++;
    }
    fprintf(f,"\n");
}


int parse_output_format(output_format_e* format, char* str){
    char delim[] = ", \n";
    char *ptr = strtok(str, delim);
    output_format_e* fptr = format;
    while(ptr != NULL)
	{
#define foo(x) if (strcmp(ptr,STRINGIFY(x))==0) {*(fptr++)=x;}
        iterate_output_format(foo);
#undef foo
		ptr = strtok(NULL, delim);
	}
    *fptr = none; 
    fprint_output_format(stderr,format);
    return 1;
}



int read_config_from_file(char* fname){
	FILE * fp = fopen(fname,"r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	char * c;
	char * pstring = &output_projection_string[0];
	char * pstring_input = &input_projection_string[0];
	offset_t * soff = &sensor_offset;
    float pos_sim_speed = 1.0;
	

	if (fp==NULL){
		fprintf(stderr,"Could not open file %s for reading config\n",fname);
		return -1;
	}
	
	while ((read = getline(&line, &len, fp)) != -1) {
		if (read>0){
			c = line;
			while (*c==' ' || *c=='\t' || *c=='\n' || *c=='\r') c++; 	//Skip leading white spaces
			if(*c==0) continue;											//End of line
			if(*c=='#') continue;										//Comment
			if (strncmp(c,"projection_output",17)==0) strcpy(pstring,c+17);	// Reading in projection string
			if (strncmp(c,"projection_header",17)==0) projection_header = atobool(c+17);	
			
			if (strncmp(c,"projection_input",16)==0) strcpy(pstring_input,c+16);	// Reading in projection string
			if (strncmp(c,"pos_mode",8)==0) pos_mode = atoi(c+8);	
			if (strncmp(c,"sensor_mode",11)==0) sensor_mode = atoi(c+11);	
			if (strncmp(c,"pos_sim_speed",13)==0) pos_sim_speed = (float)atof(c+13);
			if (strncmp(c,"sbet_epoch_week",15)==0){ nav_epoch_week_ts  = atof(c+15);   force_nav_epoch = 1; } //LEGACY
			if (strncmp(c,"nav_epoch_week",14)==0){ nav_epoch_week_ts  = atof(c+14);   force_nav_epoch = 1; }

			if (strncmp(c,"input_timezone",14)==0)  input_timezone = (float)atof(c+14);
			if (strncmp(c,"time_diff_limit",15)==0)  set_time_diff_limit((float)atof(c+15));
			if (strncmp(c,"sensor_sv_offset",16)==0)  sensor_params.sv_offset = (float)atof(c+16);
			if (strncmp(c,"sensor_force_sv",15)==0)  sensor_params.force_sv = (float)atof(c+15);
			if (strncmp(c,"alt_mode",8)==0) navdata_alt_mode = atoi(c+8);	
			if (strncmp(c,"vert_offset",11)==0) z_off = -(float)atof(c+11); //We read in vertical offset as positive upwards, while the code in general assumes z downwards
			
            if (strncmp(c,"sbp_motion_stab",15)==0) sensor_params.sbp_motion_stab = atoi(c+15);	
            if (strncmp(c,"sbp_raw_data",12)==0) sensor_params.sbp_raw_data = atoi(c+12);	
            if (strncmp(c,"sbp_bandpass_filter",19)==0) sscanf( c+19, "%f %f", &(sensor_params.sbp_bp_filter_start_freq),&(sensor_params.sbp_bp_filter_stop_freq)  ); 
            if (strncmp(c,"scale_tx_angle",14)==0) sensor_params.scale_tx_angle = (float)atof(c+14);	
    
            if (strncmp(c,"beam_correction_poly",20)==0){ 
                char * token = strtok(c+20,",");
                uint32_t pord = 0;
                float * beam_corr_poly = (float*) calloc(MAX_BEAM_ANGLE_MODEL_ORDER, sizeof(float));
                while ((token != NULL) && (pord<MAX_BEAM_ANGLE_MODEL_ORDER)) {
                    //fprintf(stderr, "token= %s\n",token);
                    beam_corr_poly[pord] = atof(token);
                    token = strtok(NULL, ",");
                    pord++;
                }
                calculate_beam_corrections_from_poly(beam_corr_poly,pord);
                free(beam_corr_poly);
            }
            
            if (strncmp(c,"beam_correction_table",21)==0){ 
                char * token = strtok(c+21,",");
                uint32_t tlen = 0;
                float * beam_corr_table = (float*) calloc(MAX_BEAM_ANGLE_TABLE_LEN, sizeof(float));
                while ((token != NULL) && (tlen<MAX_BEAM_ANGLE_TABLE_LEN)) {
                    //fprintf(stderr, "token= %s\n",token);
                    beam_corr_table[tlen] = atof(token)*((float)M_PI/180); //Input is in degrees, but internally we always use radians
                    token = strtok(NULL, ",");
                    tlen++;
                }
                /* Create angle table, uniformly distributed from -90 to 90 degrees (in radians)*/
                float * beam_corr_table_angles = (float*) calloc(tlen, sizeof(float));
                const float a0  = - (float)M_PI * 0.5f;
                const float step  = (float)M_PI / (float)(tlen - 1);
                for(size_t ix=0; ix<tlen; ix++){
                    beam_corr_table_angles[ix] = a0 + ix*step;
                }

                calculate_beam_corrections_from_table(beam_corr_table_angles,beam_corr_table,tlen);
                free(beam_corr_table);
                free(beam_corr_table_angles);
            }
            

			if (strncmp(c,"s7k_backscatter_source",22)==0) sensor_params.s7k_backscatter_source = (atoi(c+22));	
			if (strncmp(c,"snippet_processing_mode",23)==0) sensor_params.snippet_processing_mode = (atoi(c+23));	
			if (strncmp(c,"keep_s7k_tvg",12)==0) sensor_params.keep_s7k_tvg = 1;	
			if (strncmp(c,"keep_s7k_footprint",18)==0) sensor_params.keep_s7k_footprint_comp = 1;	

            if (strncmp(c,"raytrace_use_sonar_sv",21)==0)  sensor_params.sv_from_table=0;
            if (strncmp(c,"raytrace_use_table_sv",21)==0)  sensor_params.sv_from_table=1;
			
			if (strncmp(c,"sensor_x_offset",15)==0) soff->x = (float)atof(c+15);	// Reading in sensor x offset
			if (strncmp(c,"sensor_y_offset",15)==0) soff->y = (float)atof(c+15);	// Reading in sensor y offset
			if (strncmp(c,"sensor_z_offset",15)==0) soff->z = (float)atof(c+15);	// Reading in sensor z offset
			if (strncmp(c,"sensor_r_error",14)==0) soff->r_err = (float)atof(c+14);		// Reading in sensor r error
			if (strncmp(c,"sensor_yaw_offset",17)==0) soff->yaw = ((float)atof(c+17)* (float)M_PI/180);	// Reading in sensor yaw offset
			if (strncmp(c,"sensor_pitch_offset",19)==0) soff->pitch = ((float)atof(c+19)* (float)M_PI/180);	// Reading in sensor pitch offset
			if (strncmp(c,"sensor_roll_offset",18)==0) soff->roll = ((float)atof(c+18)* (float)M_PI/180);	// Reading in sensor roll offset
			if (strncmp(c,"sensor_time_offset",18)==0)  soff->time_offset = (double)atof(c+18);
			if (strncmp(c,"sensor_r_scaling",16)==0) soff->r_scale = (float)atof(c+16);		// Reading in sensor r scaling

			if (strncmp(c,"sensor_min_quality_flag",23)==0) sensor_params.min_quality_flag = (atoi(c+23));	
			if (strncmp(c,"sensor_max_quality_flag",23)==0) sensor_params.max_quality_flag = (atoi(c+23));	
			if (strncmp(c,"sensor_min_priority_flag",24)==0) sensor_params.min_priority_flag = (atoi(c+24));	
			if (strncmp(c,"sensor_max_priority_flag",24)==0) sensor_params.max_priority_flag = (atoi(c+24));	
			if (strncmp(c,"sensor_multifreq_index",22)==0) sensor_params.multifreq_index = (atoi(c+22));	
			if (strncmp(c,"sensor_data_skip",16)==0) sensor_params.data_skip = (atoi(c+16));	
			if (strncmp(c,"sensor_data_length",18)==0) sensor_params.data_length = (atoi(c+18));	
			if (strncmp(c,"sensor_min_depth",16)==0) sensor_params.min_depth = ((float)atof(c+16));
			if (strncmp(c,"sensor_max_depth",16)==0) sensor_params.max_depth = ((float)atof(c+16));
			if (strncmp(c,"sensor_min_range",16)==0) sensor_params.min_range = ((float)atof(c+16));
			if (strncmp(c,"sensor_max_range",16)==0) sensor_params.max_range = ((float)atof(c+16));
			
			if (strncmp(c,"sensor_min_azimuth",18)==0) sensor_params.min_azimuth = ((float)atof(c+18))* (float)M_PI/180;
			if (strncmp(c,"sensor_max_azimuth",18)==0) sensor_params.max_azimuth = ((float)atof(c+18))* (float)M_PI/180;
			if (strncmp(c,"sensor_min_elevation",20)==0) sensor_params.min_elevation = ((float)atof(c+20))* (float)M_PI/180;
			if (strncmp(c,"sensor_max_elevation",20)==0) sensor_params.max_elevation = ((float)atof(c+20))* (float)M_PI/180;
			if (strncmp(c,"sensor_min_ping_number",22)==0) sensor_params.min_ping_number = (atoi(c+22));	
			if (strncmp(c,"sensor_max_ping_number",22)==0) sensor_params.max_ping_number = (atoi(c+22));	
			if (strncmp(c,"sensor_swath_min_y",18)==0) sensor_params.swath_min_y = ((float)atof(c+18));
			if (strncmp(c,"sensor_swath_max_y",18)==0) sensor_params.swath_max_y = ((float)atof(c+18));
			if (strncmp(c,"sensor_mounting_depth",21)==0) sensor_params.mounting_depth = ((float)atof(c+21));
			if (strncmp(c,"ray_tracing_mode",16)==0) sensor_params.ray_tracing_mode = (atoi(c+16));	
			
            if (strncmp(c,"navigation_max_abs_roll",23)==0) sensor_params.max_abs_roll = ((float)atof(c+23))* (float)M_PI/180;
            if (strncmp(c,"navigation_max_abs_pitch",24)==0) sensor_params.max_abs_pitch = ((float)atof(c+24))* (float)M_PI/180;
            if (strncmp(c,"navigation_max_abs_yaw",22)==0) sensor_params.max_abs_yaw = ((float)atof(c+22))* (float)M_PI/180;
            if (strncmp(c,"navigation_max_abs_droll_dt",27)==0) sensor_params.max_abs_droll_dt = ((float)atof(c+27))* (float)M_PI/180;
            if (strncmp(c,"navigation_max_abs_dpitch_dt",28)==0) sensor_params.max_abs_dpitch_dt = ((float)atof(c+28))* (float)M_PI/180;
            if (strncmp(c,"navigation_max_abs_dyaw_dt",26)==0) sensor_params.max_abs_dyaw_dt = ((float)atof(c+26))* (float)M_PI/180;
			
            if (strncmp(c,"intensity_range_comp",20)==0) sensor_params.intensity_correction = 1;	//LEGACY
            if (strncmp(c,"intensity_correction",20)==0) sensor_params.intensity_correction = 1;	
            if (strncmp(c,"intensity_range_attenuation",27)==0) sensor_params.intensity_range_attenuation = ((float)atof(c+27));
            if (strncmp(c,"ara_model ",10)==0) sensor_params.ara_model = (atoi(c+10));	
            if (strncmp(c,"ara_model_multiplier",20)==0) sensor_params.ara_model_multiplier = ((float)atof(c+20));


			if (strncmp(c,"rx_nadir_beamwidth",18)==0) sensor_params.rx_nadir_beamwidth = ((float)atof(c+18))* (float)M_PI/180;
			if (strncmp(c,"tx_nadir_beamwidth",18)==0) sensor_params.tx_nadir_beamwidth = ((float)atof(c+18))* (float)M_PI/180;

            if (strncmp(c,"calc_aoi",8)==0) sensor_params.calc_aoi = (atoi(c+8));		
            if (strncmp(c,"force_bath_version",18)==0) force_bath_version = (atoi(c+18));	
            if (strncmp(c,"sensor_beam_decimate",20)==0) sensor_params.beam_decimate = (atoi(c+20));	
            if (strncmp(c,"sensor_ping_decimate",20)==0) sensor_params.ping_decimate = (atoi(c+20));	
            
            if (strncmp(c,"sim_data_period",15)==0) sim_data_period = ((float)atof(c+15));	
			
			if (strncmp(c,"sensor_source",13)==0){ input_sensor_source_string = malloc(read); strcpy(input_sensor_source_string,c+13);}
			if (strncmp(c,"navigation_source",17)==0){ input_navigation_source_string = malloc(read); strcpy(input_navigation_source_string,c+17);}
			
            if (strncmp(c,"output ",7)==0){ output_string = malloc(read); strcpy(output_string,c+6);}
			if (strncmp(c,"csv_output",10)==0) output_mode = output_csv;
			if (strncmp(c,"sbf_output",10)==0) output_mode = output_sbf;
			if (strncmp(c,"s7k_output",10)==0) output_mode = output_s7k;
			if (strncmp(c,"nmea_output",10)==0) output_mode = output_nmea;
			if (strncmp(c,"json_output",10)==0) output_mode = output_json;
			if (strncmp(c,"bin_output",10)==0) output_mode = output_binary;
			
            if (strncmp(c,"upper_gate",10)==0) sensor_params.sonar_sample_mode = upper_gate;
            if (strncmp(c,"lower_gate",10)==0) sensor_params.sonar_sample_mode = lower_gate;
            if (strncmp(c,"center_gate",11)==0) sensor_params.sonar_sample_mode = center_gate;
            /* CUSTOM CSV/SBF OUTPUT */
			if (strncmp(c,"output_format",10)==0) parse_output_format(output_format, c+10);	
		}
	}
    sensor_params.ray_tracing_mode = LIMIT(sensor_params.ray_tracing_mode,0,3);
	//Some string sanitizing
	c = pstring;while(*c){if(*c=='\n' ||*c=='\r') *c=0;c++;}									//Null terminating on first NL or CR;
	c = pstring_input;while(*c){if(*c=='\n' ||*c=='\r') *c=0;c++;}									//Null terminating on first NL or CR;
	c = input_sensor_source_string; if(c){while(*c){if(*c=='\n' ||*c=='\r') *c=0;c++;}}					//Null terminating on first NL or CR;
	c = input_navigation_source_string; if(c){while(*c){if(*c=='\n' ||*c=='\r') *c=0;c++;}}					//Null terminating on first NL or CR;
	c = output_string; if(c){while(*c){if(*c=='\n' ||*c=='\r') *c=0;c++;}}						//Null terminating on first NL or CR;
	c = input_sensor_source_string; if(c){while(*c=='\t' ||*c<=' '||*c>=127)c++; input_sensor_source_string=c;}			//Skipping white space and control before filename
	c = input_navigation_source_string; if(c){while(*c=='\t' ||*c<=' '||*c>=127)c++; input_navigation_source_string=c;}			//Skipping white space and control before filename
	c = output_string; if(c){while(*c=='\t' ||*c<=' ' ||*c>=127)c++; output_string=c;}					//Skipping white space and control before filename
	c = input_sensor_source_string; if(c){while(*c){if(*c<' ' ||*c>=127) *c=0;c++;}}					//Null terminating on first control 
	c = input_navigation_source_string; if(c){while(*c){if(*c<=' ' ||*c>=127) *c=0;c++;}}				//Null terminating on first control
	c = output_string; if(c){while(*c){if(*c<=' ' ||*c>=127) *c=0;c++;}}					//Null terminating on first control
    
    sim_nav_set_params(pos_sim_speed);	
    set_min_sim_data_period(sim_data_period);

	fprintf(stderr,"Settings read from config file:\n");

	fclose(fp);
	if (line) free(line);
	return 0;
}




/************************************** TCP SERVER FUNCTIONS *****************************/

int output_clientlist[MAX_CLIENTS];
int output_clientlist_num(void){
	int ii,num=0;
	for (ii=0;ii<MAX_CLIENTS;ii++){
		if (output_clientlist[ii] >= 0) num++;
	}
	return num;	
}
void output_clientlist_clear(void){
	int ii;
	for (ii=0;ii<MAX_CLIENTS;ii++) output_clientlist[ii] = -1;	
}
//returns first index with given fd or -1 if not found
int output_clientlist_index(int fd){
	int ii=0;
	while (ii<MAX_CLIENTS){if (output_clientlist[ii]==fd) return ii;ii++;}
	return -1;
}
//returns 0 if no room available, otherwise 1
int output_clientlist_available(void){
	return (output_clientlist_index(-1)>=0);
}
//Adds clientfd at first available space, 0 on success, -1 on failure
int output_clientlist_add(int fd){
	int ii;
	ii = output_clientlist_index(-1);
	if (ii>=0){
		output_clientlist[ii] = fd;
		if(verbose) fprintf(stderr,"Added command client on socket %d\n",fd);
		return 0;
	}
	return -1;
}
//Remove client from output_clientlist if found (returns 0), if not found, returns -1 
int output_clientlist_remove(int fd){
	int ii;
	ii = output_clientlist_index(fd);
	if(ii>=0){
		output_clientlist[ii] = -1;
		if(verbose) fprintf(stderr,"Removed command client on socket %d\n",fd);
		return 0;
	}
	return -1;
}
int output_clientlist_count(void){
	int ii,cnt=0;
	for (ii=0;ii<MAX_CLIENTS;ii++){if(output_clientlist[ii]>=0) cnt++;}
	return cnt;
}
int output_clientlist_max(void){
	int ii,max=0;
	for (ii=0;ii<MAX_CLIENTS;ii++) max = MAX(max, output_clientlist[ii]);	
	return max;
}






/************************************** MAIN *********************************************/
#define MAX_FD 256
enum input_source_e {i_none=0,i_tcp=1,i_udp=2, i_stdin=3, i_file=4, i_zero=5,i_sim=6};
enum output_drain_e {o_file,o_tcp,o_udp, o_stdout};
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


#ifdef ENABLE_NETWORK_IO
void udp_broadcast(void* data, int len,int fd, struct sockaddr* addr){
    #define MAX_UDP_LEN 65504
    int bc_len = 0;
    int bc_sent = 0;
    int ret = 0;
    while (bc_sent<len){
        bc_len = MIN(len-bc_sent,MAX_UDP_LEN);
        ret = sendto(fd, ((char*)data) + bc_sent, bc_len, 0, addr, sizeof(struct sockaddr_in));
        if (ret<0){
            fprintf(stderr,"Broadcasting %d bytes data on UDP, fails with %s\n",bc_len,strerror(errno));
            break;
        }
        else{
            //fprintf(stderr, "Broadcasting %d bytes data on UDP broadcast\n",ret);
            bc_sent += ret;
        }
    }
}
#endif		

int main(int argc,char *argv[])
{
	int c;

	enum input_source_e input_sensor_source;
	enum input_source_e input_navigation_source;
	enum output_drain_e output_drain;
	
    enum time_ref_e output_time_ref = tref_day;

	//File input and output stuff
	FILE * input_sensor_fileptr = NULL;
	FILE * input_navigation_fileptr = NULL;
	FILE * output_fileptr = NULL;
	FILE * meta_output_fileptr = NULL;

	int input_sensor_fd;
	int input_navigation_fd;
	int new_sensor_data = 0;

	
	#ifdef ENABLE_NETWORK_IO
	int ii,ret;
	char * str_addr;
	char * str_port;
	//Input Sensor TCP-client stuff
	int input_sensor_portno;
	struct sockaddr_in input_sensor_serv_addr;
	struct hostent *input_sensor_server;
	uint32_t input_sensor_ip_addr;
	//Input POS_MODE_POSMV UDP-client stuff
	int input_navigation_portno;
	struct sockaddr_in input_navigation_serv_addr;
	struct hostent *input_navigation_server;
	uint32_t input_navigation_ip_addr;
    char input_navigation_filter_udp_source = 0;
    struct in_addr input_navigation_allowed_addr;
	//Output TCP-server stuff
	int output_port = 2222;
	struct sockaddr_in output_my_addr;
	struct sockaddr_in output_clientaddr;
	output_clientlist_clear();
	int output_listener = 0;
    int output_broadcaster = 0;
	int output_client = 0;
	unsigned int addrlen;
	int yes = 1;
    
	#endif
	
	//File descriptor sets (for select)
	fd_set read_fds;
	fd_set write_fds;
	int fdmax = 0;
	
	
	#ifdef PREFER_HEAP
    char* sensor_data_buffer = calloc(MAX_SENSOR_PACKET_SIZE/8,8);
	char* navigation_data_buffer = calloc(MAX_NAVIGATION_PACKET_SIZE/8,8);
	char *output_databuffer = calloc(2*MAX_DP*MAX_LINELEN/8,8);
    output_data_t* outbuf = calloc(1,sizeof(output_data_t));
	#else
    char sensor_data_buffer[MAX_SENSOR_PACKET_SIZE];
	char navigation_data_buffer[MAX_NAVIGATION_PACKET_SIZE];
	char output_databuffer[2*MAX_DP*MAX_LINELEN];
    output_data_t _outbuf;
    output_data_t* outbuf = &_outbuf;
	#endif

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
	uint32_t prev_navigation_total_packets = 0;
	uint32_t prev_sensor_data_packet_counter = 0;
	uint32_t prev_navigation_total_data = 0;
	uint32_t prev_sensor_total_data = 0;
	uint32_t prev_output_total_data = 0;

	int32_t datapoints;

	double ts_pos = 0.;
    double ts_pos_lookahead = POS_PREFETCH_SEC;      //The lookahead defines how many sec of data pos should be ahead of other sources when reading in data. We want to be a bit in the future to be able to interpolate. With 1 sec lookahead, and 128 point buffer, we have data for about +/- 1 sec at 50Hz
	double ts_sensor = 0;
	double ts_min = 0;
    float additional_roll_offset = 0.0;
    float draft_override = -1000;       //Large negative number indicates NO draft override
	/*Default CSV format*/ 
	output_format[0] = x; output_format[1] = y; output_format[2] = z; output_format[3] = val; output_format[4] = none;
    
    wbms_init();
    r7k_init();
    gsf_init();
    posmv_init();
	
    sensor_params_default(&sensor_params);
    //Default range scaling value is 1
    sensor_offset.r_scale = 1.0;
    
    wbms_set_sensor_offset(&sensor_offset);
    r7k_set_sensor_offset(&sensor_offset);
    gsf_set_sensor_offset(&sensor_offset);
    p3dss_set_sensor_offset(&sensor_offset);
    velodyne_set_sensor_offset(&sensor_offset);
    lakibeam_set_sensor_offset(&sensor_offset);

    char *endptr;
	/**** PARSING COMMAND LINE OPTIONS ****/
        while ((c = getopt (argc, argv, "c:i:s:p:P:r:d:S:F:w:y:o:?hVxC75")) != -1) {
		switch (c) {
			case 'c':
				if(read_config_from_file(optarg)){
                    exit(-1);
                }
				break;
			case 'i':
				input_sensor_source_string= optarg;
				input_navigation_source_string= optarg;
				break;
			case 's':
				input_sensor_source_string= optarg;
				break;
			case 'p':
				input_navigation_source_string= optarg;
				break;
			case 'P':
				pos_mode = atoi(optarg);
                pos_mode = LIMIT(pos_mode,0,10);
				break;
			case 'r':
                additional_roll_offset = strtod(optarg, &endptr);
                if (*endptr != '\0') {
                    fprintf(stderr, "Invalid floating point value when reading in roll offset: %s\n", optarg);
                    return -1;
                }
                break;
			case 'd':
                draft_override = strtod(optarg, &endptr);
                if (*endptr != '\0') {
                    fprintf(stderr, "Invalid floating point value when reading in draft value: %s\n", optarg);
                    return -1;
                }
                break;
			case 'S':
				sensor_mode = atoi(optarg);
                sensor_mode = LIMIT(sensor_mode,0,10);
				break;
			case 'F':
				sensor_params.multifreq_index = atoi(optarg);
				break;
			case 'w':
				sv_file_name = optarg;
				break;
			case 'y':
				angle_intensity_file_name = optarg;
				break;
			case 'o':
				output_string= optarg;
				break;
			case '?':
			case 'h':
				showUsage(argv[0]);
				return(0);
			case 'V':
				verbose = 1;
				break;
			case 'C':
				output_mode = output_csv;
				break;
			case '7':
				output_mode = output_s7k;
                //If output mode is set to s7k by parameter, we set the default filters to basically pass anything
                sensor_params.min_range = -1000;
                sensor_params.min_quality_flag = 0;
				break;
			case '5':
				force_bath_version = 5;
				break;
			case 'x':
				generate_template_config_file("wbms_georef.conf.template");
				return(0);
			default:
				fprintf(stderr,"Unknown option - %c. Use -h to get a list of valid options.\n",c);
				return(0);
		}
	}
    //Add the additional roll offset from the input arguments
    sensor_offset.roll += additional_roll_offset*M_PI/180;

    //Override draft value from config file
    if (draft_override>-100){
        fprintf(stderr,"Forcinf sonar mounting depth (draft) to %1.1fm\n",draft_override);
	    sensor_params.mounting_depth = draft_override;
    }

	
    if (angle_intensity_file_name){
        if (read_intensity_angle_corr_from_file(angle_intensity_file_name,INTENSITY_ANGLE_STEP,INTENSITY_ANGLE_MAX_VALUES, /*output*/ intenity_angle_corr_table)>0){
            sensor_params.ara_model = ara_model_table;
        }
    }
    if (sensor_mode == sensor_mode_wbms_v5){
        force_bath_version =5;
    }
	
	fprintf(stderr,"Projection:%s\n",output_projection_string);
	fprintf(stderr,"Projection header: %s\n",projection_header?"true":"false");
	fprintf(stderr,"Sonar offset x=%f y=%f z=%f yaw=%f pitch=%f roll=%f\n",sensor_offset.x,sensor_offset.y,sensor_offset.z,sensor_offset.yaw*180/M_PI,sensor_offset.pitch*180/M_PI,sensor_offset.roll*180/M_PI);
	fprintf(stderr,"Sensor time offset:  %f\n",sensor_offset.time_offset);
	fprintf(stderr,"Sonar sv offset:  %fm/s\n",sensor_params.sv_offset);
	fprintf(stderr,"Sonar force sv:  %fm/s\n",sensor_params.force_sv);
	fprintf(stderr,"Altitude mode:  %d %s\n",navdata_alt_mode, alt_mode_names[navdata_alt_mode]);
	fprintf(stderr,"Sonar min quality flag = %d\n",sensor_params.min_quality_flag);
	fprintf(stderr,"Sonar max quality flag = %d\n",sensor_params.max_quality_flag);
	fprintf(stderr,"Sonar min priority flag = %d\n",sensor_params.min_priority_flag);
	fprintf(stderr,"Sonar max priority flag = %d\n",sensor_params.max_priority_flag);
	fprintf(stderr,"Sonar skip data sets = %d\n",sensor_params.data_skip);
	fprintf(stderr,"Sonar number of data sets = %d\n",sensor_params.data_length);
	fprintf(stderr,"Sonar min depth = %f\n",sensor_params.min_depth);
	fprintf(stderr,"Sonar max depth = %f\n",sensor_params.max_depth);
	fprintf(stderr,"Sonar min range = %f\n",sensor_params.min_range);
	fprintf(stderr,"Sonar max range = %f\n",sensor_params.max_range);
	fprintf(stderr,"Sonar min azimuth = %f\n",sensor_params.min_azimuth*180/M_PI);
	fprintf(stderr,"Sonar max azimuth = %f\n",sensor_params.max_azimuth*180/M_PI);
	fprintf(stderr,"Sonar min elevation = %f\n",sensor_params.min_elevation*180/M_PI);
	fprintf(stderr,"Sonar max elevation = %f\n",sensor_params.max_elevation*180/M_PI);
	fprintf(stderr,"Sonar min ping_number = %d\n",sensor_params.min_ping_number);
	fprintf(stderr,"Sonar max ping_number = %d\n",sensor_params.max_ping_number);
    if((sensor_params.sbp_bp_filter_start_freq>0) ||  (sensor_params.sbp_bp_filter_stop_freq>0))
	    fprintf(stderr,"SBP bandpass filter = %0.2f  to %0.2f kHz\n",sensor_params.sbp_bp_filter_start_freq,sensor_params.sbp_bp_filter_stop_freq); 
	fprintf(stderr,"Swath min y = %f\n",sensor_params.swath_min_y);
	fprintf(stderr,"Swath max y = %f\n",sensor_params.swath_max_y);
	fprintf(stderr,"Sonar mounting depth = %f\n",sensor_params.mounting_depth);
	fprintf(stderr,"Ray tracing mode = %d : %s\n",sensor_params.ray_tracing_mode, ray_tracing_mode_names[sensor_params.ray_tracing_mode]);
	fprintf(stderr,"Sonar source: %s\n", (input_sensor_source_string)?input_sensor_source_string:"None");
	fprintf(stderr,"Pos  source: %s\n", (input_navigation_source_string)?input_navigation_source_string:"None");
	fprintf(stderr,"Output: %s\n", (output_string)?output_string:"None");

	fprintf(stderr,"Output mode: %s \n", output_mode_name[output_mode]);
	fprintf(stderr,"Nav input mode:  %d %s\n",pos_mode, pos_mode_names[pos_mode]);
	fprintf(stderr,"Sensor input mode:  %d %s\n",sensor_mode, sensor_mode_names[sensor_mode]);
	fprintf(stderr,"Pos vertical offset: %0.3f\n",-z_off);
	fprintf(stderr,"Input timezone:  %f\n",input_timezone);
	//fprintf(stderr,"Time diff limit:  %f\n",time_diff_limit);

	
	if (output_string==NULL ){ fprintf(stderr,"No output data drain given, must be set with the -o  parameter\n");return(0);}
	if(verbose) fprintf(stderr,"Verbose...\n");

    #ifdef ENABLE_NETWORK_IO
    #if defined(_MSC_VER)
    if (init_windows_winsock_dll()!=0){
        fprintf(stderr,"Could not init winsock dll\n");
        exit(-1);
    }
    #endif
    #endif

	if (input_navigation_source_string==NULL){ 
		fprintf(stderr,"No nav source given, Using simulated nav data\n");
		input_navigation_source = i_sim;
        pos_mode = pos_mode_sim;
		input_navigation_fd = -1;
	}
	else if (pos_mode == pos_mode_sim){ 
		fprintf(stderr, "Using simulated nav data\n");
		input_navigation_source = i_sim;
		input_navigation_fd = -1;
	}
	else{
		/*** Choosing and initiating input_navigation_source_string source ****/
		if (strcmp(input_navigation_source_string,"-")==0){		//If input string is '-' we read from STDIN
			fprintf(stderr,"Reading pos data from STDIN\n");
			input_navigation_source = i_stdin;
			input_navigation_fileptr = stdin;
			input_navigation_fd = fileno(input_navigation_fileptr);
		}
		else if (strcmp(input_navigation_source_string,"sim")==0){		//If input string is 'sim' we use simulated data
			fprintf(stderr,"Using simulated pos data\n");
			input_navigation_source = i_sim;
			input_navigation_fd = -1;
		}
#ifdef ENABLE_NETWORK_IO
		else if (strchr(input_navigation_source_string,'#')){ //If input string contains '#' we assume it is  host#port udp, we read from UDP socket
			fprintf(stderr,"Reading pos data from UDP: %s\n",input_navigation_source_string);
			input_navigation_source = i_udp;
			str_addr = input_navigation_source_string;
			str_port = strchr(input_navigation_source_string,'#');
			str_port[0] = 0;
			str_port++;
			
			input_navigation_portno = atoi(str_port);
			input_navigation_ip_addr = inet_addr(str_addr);
			
			input_navigation_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (input_navigation_fd < 0) 
				error("ERROR opening socket");
            
            #ifdef __unix__
            setsockopt(input_navigation_fd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(int));  //TODO consider to just use REUSEADDR for linux as well (https://www.geeksforgeeks.org/difference-between-so_reuseaddr-and-so_reuseport/)
            #else
            setsockopt(input_navigation_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
            #endif
			
            bzero((char *) &input_navigation_serv_addr, sizeof(input_navigation_serv_addr));
			input_navigation_serv_addr.sin_family = AF_INET;
			input_navigation_serv_addr.sin_port = htons(input_navigation_portno);
			input_navigation_serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);         //Receive broadcast



            if (is_broadcast_address(input_navigation_ip_addr)){
                input_navigation_filter_udp_source = 0; //Setting to 0 means no filtering on UDP sources
			    fprintf(stderr,"Receiving navigation on UDP Broadcast Port = %d \n", input_navigation_portno);
			}
			else{
				//Addr given	
                input_navigation_filter_udp_source = 1; //Setting to 1 means filter on UDP sources
                inet_aton(str_addr, &input_navigation_allowed_addr);
			    fprintf(stderr,"Receiving navigation on UDP from Ip addr = %d.%d.%d.%d  Port = %d \n",input_navigation_ip_addr%256,(input_navigation_ip_addr>>8)%256,(input_navigation_ip_addr>>16)%256,(input_navigation_ip_addr>>24)%256, input_navigation_portno);
			}

			
			if (bind(input_navigation_fd,(struct sockaddr *) &input_navigation_serv_addr,sizeof(input_navigation_serv_addr)) < 0) 
				error("ERROR POSV UDP connecting");

		}
		else if (strchr(input_navigation_source_string,':')){ //If input string contains ':' we assume it is  host:port tcp, we read from TCP socket
			fprintf(stderr,"Reading pos data from TCP: %s\n",input_navigation_source_string);
			input_navigation_source = i_tcp;
			str_addr = input_navigation_source_string;
			str_port = strchr(input_navigation_source_string,':');
			str_port[0] = 0;
			str_port++;
			
			input_navigation_portno = atoi(str_port);
			input_navigation_ip_addr = inet_addr(str_addr);
			
			input_navigation_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (input_navigation_fd < 0) 
				error("ERROR opening socket");
			bzero((char *) &input_navigation_serv_addr, sizeof(input_navigation_serv_addr));
			input_navigation_serv_addr.sin_family = AF_INET;
			input_navigation_serv_addr.sin_port = htons(input_navigation_portno);

			if(input_navigation_ip_addr == (unsigned long)INADDR_NONE){
				//Hostname lookup
				fprintf(stderr,"Lookup %s\n",str_addr);
				input_navigation_server = gethostbyname(str_addr);
				bcopy((char *)input_navigation_server->h_addr,  (char *)&input_navigation_serv_addr.sin_addr.s_addr, input_navigation_server->h_length);
				input_navigation_ip_addr = input_navigation_serv_addr.sin_addr.s_addr;
			}
			else{
				//Addr given	
				input_navigation_serv_addr.sin_addr.s_addr = input_navigation_ip_addr;
			}

			fprintf(stderr,"Ip addr = %d.%d.%d.%d  Port = %d \n",input_navigation_ip_addr%256,(input_navigation_ip_addr>>8)%256,(input_navigation_ip_addr>>16)%256,(input_navigation_ip_addr>>24)%256, input_navigation_portno);

			if (connect(input_navigation_fd,(struct sockaddr *) &input_navigation_serv_addr,sizeof(input_navigation_serv_addr)) < 0) 
				error("ERROR connecting");

		}
#else
		else if ((strchr(input_navigation_source_string,'#')) || (strchr(input_navigation_source_string,':'))){ 
			error("Network IO not supported\n");
		}
#endif
		else{					// Otherwise we assume it is a FILE

			fprintf(stderr,"Reading pos data from FILE: %s\n",input_navigation_source_string);
			input_navigation_source = i_file;
			input_navigation_fileptr = fopen(input_navigation_source_string,"rb");
            if (input_navigation_fileptr == NULL){
                fprintf(stderr,"Could not open navigation data file%s\n",input_navigation_source_string);
                exit(-1);
            }
            
            //The GSF library needs to access the file by filename, so we need to tell it that.
            gsf_set_nav_filename(input_navigation_source_string);
            //Try to autodetect file format
            if (pos_mode==pos_mode_autodetect){
                pos_mode = navigation_autodetect_file(input_navigation_fileptr);
            }

			if(input_navigation_fileptr>0){
				input_navigation_fd = fileno(input_navigation_fileptr);
				if (navigation_test_file(input_navigation_fd,pos_mode)){
					/*if(verbose)*/ fprintf(stderr,"File %s, opened with valid data in mode %s\r\n",input_navigation_source_string,pos_mode_names[pos_mode]);			
                    fseek(input_navigation_fileptr,0,SEEK_SET);
				}
				else{
					fprintf(stderr,"File %s, does not contain valid navigation data in %s format\r\n",input_navigation_source_string,pos_mode_names[pos_mode]);			
					fclose(input_navigation_fileptr);
					return(-1);
				}
			}
			else{
				fprintf(stderr,"Could not open file %s for reading\r\n",input_navigation_source_string);
				return(-1);
			}
		}
	}

	

	if (input_sensor_source_string==NULL){ 
		fprintf(stderr,"No WBMS data source given, using simulated data (vessel pos)\n");
		input_sensor_source = i_sim;
        sensor_mode = sensor_mode_sim;
		input_sensor_fd = -1;
	}
	else if (sensor_mode == sensor_mode_sim){ 
		fprintf(stderr, "Using simulated sensor data (vessel pos)\n");
		input_sensor_source = i_sim;
		input_sensor_fd = -1;
	}
	else{
		/*** Choosing and initiating input_wbms source ****/
		if (strcmp(input_sensor_source_string,"-")==0){		//If input string is '-' we read from STDIN
			fprintf(stderr,"Reading sensor data from STDIN\n");
			input_sensor_source = i_stdin;
			input_sensor_fileptr = stdin;
			input_sensor_fd = fileno(input_sensor_fileptr);
		}
#ifdef ENABLE_NETWORK_IO
		else if (strchr(input_sensor_source_string,':')){ //If input string contains ':' we assume it is  host:port tcp, we read from TCP socket
			
			fprintf(stderr,"Reading sensor data from TCP: %s\n",input_sensor_source_string);
			input_sensor_source = i_tcp;
			str_addr = input_sensor_source_string;
			str_port = strchr(input_sensor_source_string,':');
			str_port[0] = 0;
			str_port++;
			
			input_sensor_portno = atoi(str_port);
			input_sensor_ip_addr = inet_addr(str_addr);
			
			input_sensor_fd = socket(AF_INET, SOCK_STREAM, 0);
			if (input_sensor_fd < 0) 
				error("ERROR opening socket");
			bzero((char *) &input_sensor_serv_addr, sizeof(input_sensor_serv_addr));
			input_sensor_serv_addr.sin_family = AF_INET;
			input_sensor_serv_addr.sin_port = htons(input_sensor_portno);

			if(input_sensor_ip_addr == (unsigned long)INADDR_NONE){
				//Hostname lookup
				fprintf(stderr,"Lookup %s\n",str_addr);
				input_sensor_server = gethostbyname(str_addr);
				bcopy((char *)input_sensor_server->h_addr,  (char *)&input_sensor_serv_addr.sin_addr.s_addr, input_sensor_server->h_length);
				input_sensor_ip_addr = input_sensor_serv_addr.sin_addr.s_addr;
			}
			else{
				//Addr given	
				input_sensor_serv_addr.sin_addr.s_addr = input_sensor_ip_addr;
			}

			fprintf(stderr,"Ip addr = %d.%d.%d.%d  Port = %d \n",input_sensor_ip_addr%256,(input_sensor_ip_addr>>8)%256,(input_sensor_ip_addr>>16)%256,(input_sensor_ip_addr>>24)%256, input_sensor_portno);

			if (connect(input_sensor_fd,(struct sockaddr *) &input_sensor_serv_addr,sizeof(input_sensor_serv_addr)) < 0) 
				error("ERROR connecting");

		}
#else
		else if (strchr(input_sensor_source_string,':')){ //If input string contains ':' we assume it is  host:port tcp, we read from TCP socket
			error("Network IO not supported\n");
		}
#endif
		else{					// Otherwise we assume it is a FILE
			fprintf(stderr,"Reading sensor data from FILE: %s\n",input_sensor_source_string);
			input_sensor_source = i_file;
			input_sensor_fileptr = fopen(input_sensor_source_string,"rb");
            if (input_sensor_fileptr == NULL){
                fprintf(stderr,"Could not open sensor data file%s\n",input_sensor_source_string);
                exit(-1);
            }
            //The GSF library needs to access the file by filename, so we need to tell it that.
            gsf_set_sensor_filename(input_sensor_source_string);
            //Try to autodetect file format
            if (sensor_mode==sensor_mode_autodetect){
                sensor_mode = sensor_autodetect_file(input_sensor_fileptr);
            }
			if(input_sensor_fileptr>0){
				input_sensor_fd = fileno(input_sensor_fileptr);
                int version;
				if (sensor_test_file(input_sensor_fd,sensor_mode,&version)){
					/*if(verbose)*/ fprintf(stderr,"File %s, opened with valid data in mode %s version=%d\r\n",input_sensor_source_string, sensor_mode_names[sensor_mode],version);			
					lseek(input_sensor_fd,0,SEEK_SET);
				}
				else{
					fprintf(stderr,"File %s, does not contain valid sensor data in %s format\r\n",input_sensor_source_string,sensor_mode_names[sensor_mode]);			
					fclose(input_sensor_fileptr);
					return(-1);
				}
			}
			else{
				fprintf(stderr,"Could not open file %s for reading\r\n",input_sensor_source_string);
				return(-1);
			}
		}
	}
    
	
	/**** Choosing and initiating output drain ****/
	if (strcmp(output_string,"-")==0){
		output_drain = o_stdout;
		output_fileptr = stdout;
	}
#ifdef ENABLE_NETWORK_IO
    else if (strchr(output_string,'#')){ //If input string contains '#' we assume it is  host#port udp, we read from UDP socket
        
        output_drain = o_udp;
        str_addr = output_string;
        str_port = strchr(output_string,'#');
        str_port[0] = 0;
        str_port++;
        
        output_port = atoi(str_port);
        fprintf(stderr,"Writing data to UDP broadcast on port: %d\n",output_port);
        //Create a UDP socket
        if ((output_broadcaster = socket(AF_INET, SOCK_DGRAM,0))==-1)
        {fprintf(stderr,"Could not create Output UDP-socket\n");exit(1);}
        if  (setsockopt(output_broadcaster, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
        {fprintf(stderr,"Server-setsockopt() error\n");exit(1);}
        if  (setsockopt(output_broadcaster, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(int)) == -1)
        {fprintf(stderr,"Server-setsockopt() error\n");exit(1);}
        bzero((char *)&output_my_addr, sizeof(output_my_addr));
        output_my_addr.sin_family = AF_INET;
        output_my_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        output_my_addr.sin_port = htons(output_port);
        
    }
	else if (strchr(output_string,':')){ //If out string contains ':' we assume it is  localhost:port tcp, we write to TCP clients
		
		output_drain = o_tcp;
		str_addr = output_string;
		str_port = strchr(output_string,':');
		str_port[0] = 0;
		str_port++;
		
		output_port = atoi(str_port);
		//output_ip_addr = inet_addr(str_addr);
		fprintf(stderr,"Writing data to clients on TCP: %d\n",output_port);
		
	 	//Create a TCP socket
		if ((output_listener = socket(AF_INET, SOCK_STREAM,0))==-1)
		{fprintf(stderr,"Could not create Output TCP-socket\n");exit(1);}
		if (setsockopt(output_listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1)
		{fprintf(stderr,"Server-setsockopt() error\n");exit(1);}
		//Bind socket to given port number
		output_my_addr.sin_family = AF_INET;
		output_my_addr.sin_port = htons(output_port);
		output_my_addr.sin_addr.s_addr = INADDR_ANY;
		memset(&(output_my_addr.sin_zero), 0, 8);
		if(bind(output_listener, (struct sockaddr *)&output_my_addr, sizeof(struct sockaddr)) == -1)
		{fprintf(stderr,"Could not bind to Output TCP-socket\n");exit(1);}
		//Start listening to socket for incomming requests, MAX_CLIENTS clients in backlog allowed
		if(listen(output_listener,MAX_CLIENTS) == -1)	
		{fprintf(stderr,"Could not listen to Output TCP-socket\n");exit(1);}
		// keep track of the biggest file descriptor 

	}
#else
	else if (strchr(output_string,':')){ //If out string contains ':' we assume it is  localhost:port tcp, we write to TCP clients
		error("Network IO not supported\n");
	}
#endif
	else{
		output_drain = o_file;
        //SBF output generates two files, an ascii metafile (.sbf) + a binary datafile (.sbf.data)
        if (output_mode==output_sbf){
            char output_string_data[1024];
            sprintf(output_string_data,"%s.sbf.data",output_string);
            output_fileptr =        fopen(output_string_data,"wb");
            char output_string_meta[1024];
            sprintf(output_string_meta,"%s.sbf",output_string);
            meta_output_fileptr =   fopen(output_string_meta,"wb");
            if(output_fileptr>0 && meta_output_fileptr>0){
                fprintf(stderr,"Open file %s and %s for writing\r\n",output_string_data, output_string_meta);
            }
            else{
                fprintf(stderr,"Could not open file %s for writing\r\n",output_string);
                return(-1);
            }
        }
		else{
            output_fileptr = fopen(output_string,"wb");
            if(output_fileptr>0){
                if(verbose) fprintf(stderr,"Open file %s for writing\r\n",output_string);
            }
            else{
                fprintf(stderr,"Could not open file %s for writing\r\n",output_string);
                return(-1);
            }
        }
	}

	if (verbose){
		fprintf(stderr,"Posmv source = %d: %s, Sensor source = %d: %s \n",input_navigation_source, input_source_names[input_navigation_source],input_sensor_source,  input_source_names[input_sensor_source]);
	}

    //When using navigation data without full timestamp we need to set an epoch fort the reported time, 
    if (force_nav_epoch){
        //force epoch based on timestamp in config file 
        set_sbet_epoch(nav_epoch_week_ts);
        set_posmv_alt_gps_epoch(nav_epoch_week_ts);
        nav_epoch_set = 1;
    }
    if (!nav_epoch_set && (input_navigation_source==i_file)) {
        //Try to set epoch from navigation file filename
        double ts_filename = parse_timestamp_from_filename(input_navigation_source_string);
        if (ts_filename){
            fprintf(stderr,"Using time from filename for NAV epoch\n");
            set_sbet_epoch(ts_filename);
            set_posmv_alt_gps_epoch(ts_filename);
            nav_epoch_set = 1;
        }
    }
    //If bathy data is available fetch and process a bathy data packet to get time  (TODO for lidar (valodyne and lakkibeam)  we need navigation time before sensor data is processed, as it does not send full time)
    if (input_sensor_source != i_none  && input_sensor_source != i_sim){ 
        //Find first usable packet in stream
        while (1){
            sensor_data_buffer_len = sensor_fetch_next_packet(sensor_data_buffer, input_sensor_fd, sensor_mode);
            //fprintf(stderr,"sensor_data_buffer_len = %d\n", sensor_data_buffer_len);
            if (sensor_data_buffer_len==0) break;

            //printf("sensor_identify_packet(sensor_data_buffer,%d,%f,%f)\n", sensor_data_buffer_len,ts_pos,ts_sensor);
            if ( sensor_identify_packet(sensor_data_buffer,sensor_data_buffer_len,ts_pos, &ts_sensor, sensor_mode)){
                //ts_sensor += sensor_offset.time_offset;
                time_t raw_time = (time_t) ts_sensor;
                fprintf(stderr,"First sensor data time: ts=%0.3f %s  ",ts_sensor,ctime(&raw_time));
                //When using navigation data without full timestamp, we need to set an epoch, base this on first sensor timestamp 
                if (!nav_epoch_set){
                    set_sbet_epoch(ts_sensor);
                    set_posmv_alt_gps_epoch(ts_sensor);
                    nav_epoch_set = 1;
                }
                break;
            }
        }
        if (input_sensor_source==i_file) fseek(input_sensor_fileptr, 0, SEEK_SET); //Rewind
    }
    if ((input_sensor_source == i_sim) && (!force_nav_epoch)){
        if (!nav_epoch_set){
            fprintf(stderr,"Simulated sensor data, just setting epoch to Unix epoch for nav data without full timestamp\n");
            const double ts0 = 60*60*24*7;
            set_sbet_epoch(ts0);
            set_posmv_alt_gps_epoch(ts0);
            nav_epoch_set = 1;
         }
    }

    if (output_mode==output_s7k){
        switch (output_drain){
            case o_file:    set_r7k_output_parameters(1, 0, 0); break;          //Output to file, S7K should be marked as recorded data, and no network frame, no packet size limit
            case o_stdout:  set_r7k_output_parameters(0, 0, 0); break;          //Output to stdout, S7K should be marked as live data, and no network frame, no packet size limit
            case o_tcp:     set_r7k_output_parameters(0, 1, 0); break;          //Output to tcp, S7K should be marked as live data, with network frame, no packet size limit
            case o_udp:     set_r7k_output_parameters(0, 1, 65536); break;      //Output to file, S7K should be marked as live data, with network frame, 64k packet size limit
        }
    }
	

    if (sv_file_name){
        int ret = 0;
        float min_sv = 0.; //0,0 means default range
        float max_sv = 0.;
        float max_depth = 0.; //Maximum depth relative to sonar, 0 means use max depth from extrapolated sv-file
        switch(sensor_params.ray_tracing_mode){
            case ray_trace_fixed_depth_lut: //Fixed depth LUT
                // Before we can create the LUT, we need to know the sound velocity range wee need to calculate it for
                if (sensor_params.force_sv > 0){       //With a forced sensor sv it is just the forced value
                    min_sv = sensor_params.force_sv;
                    max_sv = sensor_params.force_sv;
                }
                else if (input_sensor_source == i_file){ // Otherwise, we need to go through the sensor file to read out the sv range
                    // Read out SV-range for datafile
                    sensor_get_sv_range(input_sensor_fd,sensor_mode, &min_sv, &max_sv);
                    min_sv += sensor_params.sv_offset;
                    max_sv += sensor_params.sv_offset;
                    // Rewind file 
                    if (sensor_mode == sensor_mode_gsf){
                        gsf_rewind(fileno(input_sensor_fileptr)); 
                    }
                    else{
                        fseek(input_sensor_fileptr, 0, SEEK_SET); //Rewind
                    }
                    // Read out max range from datafile
                    max_depth = sensor_get_max_range(input_sensor_fd,sensor_mode); //TODO Ideally we should use max depth here, but since we know that max_depth<max_range, this is safe, but is a bit suboptimal, as unneccessary long SV table needs to be calculated
                    // Rewind file 
                    if (sensor_mode == sensor_mode_gsf){
                        gsf_rewind(fileno(input_sensor_fileptr)); 
                    }
                    else{
                        fseek(input_sensor_fileptr, 0, SEEK_SET); //Rewind
                    }
                }
                ret = generate_ray_bending_table_from_sv_file(sv_file_name, sensor_params.mounting_depth, 1,min_sv, max_sv,max_depth); //In LUT mode, we asume that the sonar draft is fixed, and generate the table with respect to this
                break;
            case ray_trace_fixed_depth_direct: //Fixed depth, direct
                ret = generate_ray_bending_table_from_sv_file(sv_file_name, sensor_params.mounting_depth, 0,0,0,0); //In direct raytracing, wixed depth, we generate the table with respect to sonar
                break;
            case ray_trace_var_depth: //Variable depth, direct
                ret= generate_ray_bending_table_from_sv_file(sv_file_name, 0., 0,0,0,0); //In direct raytracing, we generate the table with respect to water surface, and input sonar position wrt. this when doing raytracing
                break;
            case ray_trace_none:
                break;
        }
        if (ret<0){
            exit(-1);
        }
    }
    else{
        sensor_params.ray_tracing_mode=ray_trace_none;
    }

    //Fetch and process a few pos datasets to guess correct UTM zone and finding offset values for Northing and Easting
    double first_lat=0; 
    double first_lon=0;
    while (1){
        navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_navigation_fd,pos_mode);
        //fprintf(stderr,"navigation_fetch_next_packet = %d\n",navigation_data_buffer_len);
        if(navigation_data_buffer_len>0){
            int new_nav_data = process_nav_data_packet(navigation_data_buffer,navigation_data_buffer_len,ts_sensor, &ts_pos,pos_mode,z_off);  //(TODO but for navigation simulator, we need sensor time before navigation, what to do)
            if( new_nav_data){
                //TODO this is not very clean code. When received data is Goegraphic only, and not projected , it arrives in the index following the last valid
                if( new_nav_data == NAV_DATA_GEO){
                    first_lat = navdata[(navdata_ix+1)%NAVDATA_BUFFER_LEN].lat;
                    first_lon = navdata[(navdata_ix+1)%NAVDATA_BUFFER_LEN].lon;
                }
			    else{
                    first_lat = navdata[navdata_ix].lat;
                    first_lon = navdata[navdata_ix].lon;
                }
                fprintf(stderr,"First nav data lat = %8.5fdegN lon = %8.5fdegE\n",first_lat*180/M_PI,first_lon*180/M_PI);
                time_t raw_time = (time_t) ts_pos;
                fprintf(stderr,"First nav data time: ts=%0.3f %s  ",ts_pos,ctime(&raw_time));

                //Lakibeam sensor does not have global time, only time since bootup
                // As a quickfix (hack) we assume that the navigation stream and sensor stream starts at the same instant, and set the difference as the lakibeam epoch
                if(sensor_mode_lakibeam){
                    lakibeam_set_epoch(ts_pos - ts_sensor);
                }
                break;
            }
        }
        else break;
    }
    if (input_navigation_source==i_file) fseek(input_navigation_fileptr, 0, SEEK_SET); //Rewind


    if(output_projection_string[0] == 0){
        default_projection(output_projection_string,first_lat,first_lon);
		fprintf(stderr, "No output projection given, defaulting to %s\n",output_projection_string);
	}
	if (input_projection_string[0] == 0){
        default_projection(input_projection_string,first_lat,first_lon);
		fprintf(stderr, "No input projection given, defaulting to %s\n",input_projection_string);
	}
    xtf_nav_set_params(input_timezone, input_projection_string);	
            
    //Use first position as offset for coordinates (In SBF coordinates is given as 32-bit float, and need a 64-bit float for offset to avoid discretization issues
    double x_offset,y_offset,z_offset; 
    double ts_offset = ts_pos;
    
    
    switch (output_time_ref){
        case tref_unix: ts_offset = 0;break;
        case tref_line: ts_offset = ts_pos;break;
        case tref_day: ts_offset = floor(ts_pos/(3600*24))*(3600*24);break;
        case tref_week: ts_offset = floor(ts_pos/(3600*24*7))*(3600*24*7);break;
    }
    

    if(latlon_to_proj_from_string(output_projection_string,&proj_latlon_to_output_utm)<0){
		fprintf(stderr, "Error configuring georef parameters, exiting\n");
		return(-1);
	}
    
    latlon_to_kart(first_lon,first_lat,0,proj_latlon_to_output_utm,&x_offset, &y_offset, &z_offset);
    z_offset = 0;
    fprintf(stderr,"First nav data Northing = %8.5f Easting = %8.5f \n",x_offset,y_offset);


	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	double ts_now,ts_last;
	int dataset_counter = 0;
	char str_buf[256];
	ts_last = 0;
   
    // Insert projection string in output file 
    if((output_drain == o_file && projection_header) ){
        output_databuffer_len = write_csv_proj_string_to_buffer(&(output_projection_string[0]), /*OUTPUT*/ output_databuffer);
        fprintf(stderr,"%s\n",output_databuffer);
        fprintf(stderr,"proj_string len = %d\n",output_databuffer_len);
        switch (output_mode){
            case output_binary:
                output_databuffer_len = write_bin_proj_string_to_buffer(&(output_projection_string[0]), /*OUTPUT*/ output_databuffer);
                fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
                break;
            case output_csv:
                output_databuffer_len = write_csv_proj_string_to_buffer(&(output_projection_string[0]), /*OUTPUT*/ output_databuffer);
                fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
                break;
        
            case output_nmea: break;
            case output_json: break;
            case output_sbf: break;
            case output_s7k: break;
            default: break;
        }
    }
    
    if((output_drain == o_file) && (output_mode==output_csv)){
        output_databuffer_len = write_csv_header_to_buffer(output_format, output_databuffer);
        fprintf(stderr,"%s",output_databuffer);
		fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
    }
    
    if((output_drain == o_file) && (output_mode==output_sbf)){
        output_databuffer_len = write_sbf_header_to_buffer(output_format,0,x_offset,y_offset,z_offset, output_databuffer);
		fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
    }

    if((output_drain == o_file) && (output_mode==output_s7k)){
        output_databuffer_len = write_r7k_header_to_buffer(ts_sensor, output_databuffer);
		fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
    }
    

	// While PosMV is a valid source OR PosMv is zero-source or sim and one of the other sources is available
	while ( 
                ((input_navigation_source!=i_none) && (input_navigation_source!=i_zero) && (input_navigation_source!=i_sim)) || 
                (((input_navigation_source==i_zero)||(input_navigation_source==i_sim)) && ((input_sensor_source != i_none) )) 
         ){
		//DISPLAY STATISTICS
		ts_now = os_time();
		if (ts_now>(ts_last+1)){
            //printf("%d, %d\n", input_navigation_source, input_sensor_source);
			//printf("Tpos=%f\t Tsensor=%f\t \t\t Tmin=%f\n",ts_pos, ts_sensor,  ts_min);
			sprintf_unix_time(str_buf, ts_min);
			fprintf(stderr,"%s",str_buf);
			if (input_navigation_fd>0) fprintf(stderr,"| NAV   %6d (%5d/s) %4dMB (%5dkB/s)  ",navigation_total_packets,(navigation_total_packets-prev_navigation_total_packets),	navigation_total_data>>20, (navigation_total_data-prev_navigation_total_data)/1024);
			if (input_sensor_fd>0) fprintf(stderr,"| SENSOR  %6d (%5d/s) %4dMB (%5dkB/s)  ",sensor_data_packet_counter,(sensor_data_packet_counter-prev_sensor_data_packet_counter),	sensor_total_data>>20, (sensor_total_data-prev_sensor_total_data)/1024);
			fprintf(stderr,"| OUT   %4dMB (%4dkB/s)  ",	output_total_data>>20, (output_total_data-prev_output_total_data)/1024);
			fprintf(stderr,"|\n");
			prev_navigation_total_packets = navigation_total_packets;
			prev_navigation_total_data = navigation_total_data;
			prev_sensor_data_packet_counter = sensor_data_packet_counter;
			prev_sensor_total_data = sensor_total_data;
			prev_output_total_data = output_total_data;
			ts_last = ts_now;
		}

		// Reset read fd sets
		FD_ZERO(&read_fds);
		fdmax = 0;
		
		#ifdef ENABLE_NETWORK_IO
		if (output_drain == o_tcp){ 
			FD_SET(output_listener, &read_fds);
			fdmax = MAX(fdmax,output_listener);
		}
		#endif

		if ((input_navigation_source!=i_none) && (input_navigation_source!=i_zero) && (input_navigation_source!=i_sim)){ 
			FD_SET(input_navigation_fd, &read_fds);
			fdmax = MAX(fdmax,input_navigation_fd);
		}
		if ((input_sensor_source != i_none) && (input_sensor_source != i_sim)){ 
			FD_SET(input_sensor_fd, &read_fds);
			fdmax = MAX(fdmax,input_sensor_fd);
		}
		fdmax = MAX(fdmax,output_clientlist_max());	
        
		
		if(select(fdmax+1, &read_fds, &write_fds, NULL, NULL) == -1){ 
            #if defined(_MSC_VER)
            int error_code = WSAGetLastError();
            fprintf(stderr,"Server-select() error. Error code = %d\n", error_code);
            exit(1);
            #else
            fprintf(stderr,"Server-select() error\n");
            exit(1);
            #endif

        }

		// IF OUTPUT DRAIN IS A (OR MANY) TCP CLIENT(S), WE HANDLE CLIENTS CONNECTING / DISCONNECTING AND SNEDING DATA TO THEM HERE
		#ifdef ENABLE_NETWORK_IO
		if (output_drain == o_udp){
            udp_broadcast(output_databuffer, output_databuffer_len, output_broadcaster, (struct sockaddr*) &output_my_addr);
        }
		else if (output_drain == o_tcp){
            int new_clients = 0;
			//** Checking if new client has connected as drain
			if (FD_ISSET(output_listener, &read_fds)){ 
				addrlen = sizeof(output_clientaddr);
				if((output_client = accept(output_listener, (struct sockaddr *)&output_clientaddr, &addrlen)) == -1) fprintf(stderr,"Server-accept() error\n");
				else if (output_clientlist_available()){
					output_clientlist_add(output_client);
					fprintf(stderr,"New connection from %s on socket %d\n",  inet_ntoa(output_clientaddr.sin_addr), output_client);
                    new_clients++;
				}
				else{
					fprintf(stderr,"Politely rejecting connection from %s on socket %d, clientlist full\n",  inet_ntoa(output_clientaddr.sin_addr), output_client);
                    char* msg_out_buf = "Max number of clients connected!\n";
					if(send(output_client, msg_out_buf, strlen(msg_out_buf),  TCP_SEND_FLAGS) == -1) fprintf(stderr,"Server-send() error\n");
					close(output_client);
				}

			}
            
            //If a new client has connected, and we are streaming s7k data, resend the header sequence  
            if(new_clients && (output_mode==output_s7k)){
			    fprintf(stderr,"Resending s7k header to TCP clients\n");
                output_databuffer_len += write_r7k_header_to_buffer(ts_sensor, &(output_databuffer[output_databuffer_len]));
            }

			//Sending data to all ready clients
			for(ii = 0;ii< MAX_CLIENTS;ii++){
				output_client = output_clientlist[ii];
				if (output_client<=0) continue;
				if(FD_ISSET(output_client, &write_fds)){ 
					ret = send(output_client, output_databuffer, output_databuffer_len,  TCP_SEND_FLAGS); 
					//fprintf(stderr,"Sending dataset %d to client %d\n",dataset_counter, output_client);
					FD_CLR(output_client, &write_fds);
					if(ret == -1){ 
						fprintf(stderr,"Server-send() error removing client from list\n");
						close(output_client);
						output_clientlist_remove(output_client);
						output_client = -1;
					}
				}
			}
		}
		#endif

		static double prev_ts_detections=0;
		
		output_databuffer_len = 0;	

		double ts_detections = ts_sensor;	//Minimum time stamp for non-navigation data
	
        //fprintf(stderr,".");
		if((input_navigation_source == i_sim) && (ts_detections<prev_ts_detections)){
			//fprintf(stderr, "Detection data stream jumping backwards in time %f < %f \n",ts_detections,prev_ts_detections);
			ts_pos = ts_detections; //In simulator mode, if detections jumps back in time, so should simulated nav data
		}
		prev_ts_detections = ts_detections;


		ts_min = 10000000000.;
		if((input_sensor_source != i_none) ) ts_min = MIN(ts_min, ts_sensor);
        if(input_navigation_source != i_none) ts_min = MIN(ts_min, ts_pos-ts_pos_lookahead);

        //fprintf(stderr,"Ts_sensor = %f, Ts_pos =%f \n",ts_sensor, ts_pos);
		//** Process new data from navigation source
        if((ts_pos-ts_pos_lookahead == ts_min)){
            while(1){

                if ((input_navigation_source==i_none) || (input_navigation_source==i_zero)) break;  //Dont run at all if input source is ZERO or NONE 

                if (1){ //Here is the data fetch loop 
                    navigation_data_buffer_len = 0;
                    uint8_t runnavigation = 0;
                    if (input_navigation_source==i_sim) runnavigation=1;
                    else if (input_navigation_source==i_file) runnavigation=1;
                    else if (FD_ISSET(input_navigation_fd,&read_fds)) runnavigation=1;
                    if (runnavigation){
                        if(input_navigation_source==i_udp) {
                            #if 0
                            navigation_data_buffer_len = read(input_navigation_fd,navigation_data_buffer,MAX_NAVIGATION_PACKET_SIZE);
                            #else
                            struct sockaddr_in udp_source_addr;
                            socklen_t addr_len = sizeof(udp_source_addr);
                            ssize_t n = recvfrom(input_navigation_fd, navigation_data_buffer, MAX_NAVIGATION_PACKET_SIZE, 0, (struct sockaddr *)&udp_source_addr, &addr_len);
                            if (n < 0) {
                                //fprintf(stderr,"Nav data recvfrom failed");
                                navigation_data_buffer_len = 0;
                            }
                            else{
                                // Compare sender IP with the allowed one
                                if ((input_navigation_filter_udp_source == 0) || (udp_source_addr.sin_addr.s_addr == input_navigation_allowed_addr.s_addr)) {
                                    //printf("Nav data received from %s\n", inet_ntoa(udp_source_addr.sin_addr));
                                    navigation_data_buffer_len = n;
                                } 
                                else {
                                    //printf("Nav data ignored packet from %s\n", inet_ntoa(udp_source_addr.sin_addr));
                                    navigation_data_buffer_len = 0;
                                }
                            }
                            #endif

                        }
                        else if(input_navigation_source==i_sim){
                            navigation_data_buffer_len = 1; //Just mark that we have data
                        }
                        else {
                            navigation_data_buffer_len = navigation_fetch_next_packet(navigation_data_buffer, input_navigation_fd,pos_mode);
                        }

                        if (navigation_data_buffer_len>0){
                            navigation_total_data += navigation_data_buffer_len;

                            if (process_nav_data_packet(navigation_data_buffer,navigation_data_buffer_len,ts_detections, &ts_pos,pos_mode,z_off)){
                                navigation_total_packets++;
                            }
                        }
                        else if((navigation_data_buffer_len==0) && (input_navigation_source!=i_udp))  {
                            input_navigation_source = i_none;
                        }
                    }
                }
                if ((input_navigation_source==i_tcp) || (input_navigation_source==i_udp) || (input_navigation_source==i_stdin)) break;  //Run once per turn,  if input source is a stream (TCP, UDP, STDIN)

                //fprintf(stderr, "ts_pos = %f, ts_pos-ts_pos_lookahead = %f, ts_sensor=%f\n", ts_pos,ts_pos-ts_pos_lookahead, ts_sensor);
                if (ts_pos-ts_pos_lookahead>ts_sensor ) break;       //If input source is a file or simulator, run until we are ahead of sensor
            }
        }
		
        ts_min = 10000000000.;
		if(input_sensor_source != i_none) ts_min = MIN(ts_min, ts_sensor);
        if(input_navigation_source != i_none) ts_min = MIN(ts_min, ts_pos-ts_pos_lookahead);
        //fprintf(stderr,"Ts_sensor = %f, Ts_pos =%f  Ts_min=%f\n",ts_sensor, ts_pos,ts_min);
		

		//** Process new data from sensor source
		if (input_sensor_source != i_none && (ts_sensor==ts_min || input_sensor_source != i_file)){
            //fprintf(stderr,"proc sensor\n");
			sensor_data_buffer_len = 0;
            uint8_t runsensor=0;
            if (input_sensor_source==i_sim) runsensor=1;
            else if (input_sensor_source==i_file) runsensor=1;
			else if (FD_ISSET(input_sensor_fd,&read_fds)) runsensor=1;

			if (runsensor){
				sensor_data_buffer_len = sensor_fetch_next_packet(sensor_data_buffer, input_sensor_fd, sensor_mode);
                		//fprintf(stderr,"sensor_data_buffer_len = %d\n",sensor_data_buffer_len);fflush(stderr);
				if (sensor_data_buffer_len){
					sensor_total_data += sensor_data_buffer_len;
                    double new_ts_sensor;
					new_sensor_data = sensor_identify_packet(sensor_data_buffer,sensor_data_buffer_len,ts_pos, &new_ts_sensor, sensor_mode);

					//ts_sensor += sensor_offset.time_offset;
				    //if(new_sensor_data) fprintf(stderr,"new_sensor_data = %d Ts_sensor = %f  Ts_pos =%f navdata_count=%d\n", new_sensor_data, ts_sensor, ts_pos,navdata_count);
                    //fprintf(stderr,"Ts_sensor = %f, Ts_pos =%f navdata_count=%d\n",ts_sensor, ts_pos,navdata_count);

                    //TODO Here we should really force the navigation data to get up to speed, in case the newly fetched sensor data is ahead
		

					if (new_sensor_data){ 
                        ts_sensor = new_ts_sensor;
                        if (1){ //if((navdata_count > NAVDATA_BUFFER_LEN/4) || (pos_mode == pos_mode_sim)){ //Need a quarter buffer before we can start doing georeferencing
                            uint8_t more_data_to_process = 1;
                            uint16_t sector = 0;                                          // GSF data contains data from multiple sectors (multifreq) in one dataset and needs multiple passes to process and output one sector at a time
                            while(more_data_to_process){
                            
                                switch (sensor_mode){
                                    case sensor_mode_wbms: case sensor_mode_wbms_v5:
                                        if (new_sensor_data == PACKET_TYPE_BATH_DATA){
                                            datapoints = wbms_georef_data( (bath_data_packet_t*) sensor_data_buffer, navdata, navdata_ix,  &sensor_params, outbuf, force_bath_version);
                                        }
                                        else if (new_sensor_data == PACKET_TYPE_SNIPPET_DATA){
                                            datapoints = wbms_georef_snippet_data( (snippet_data_packet_v8_t*) sensor_data_buffer, navdata, navdata_ix,  &sensor_params, outbuf, force_bath_version);
                                        }
                                        else if (new_sensor_data == PACKET_TYPE_SBP_DATA){
                                            datapoints = wbms_georef_sbp_data( (sbp_data_packet_t*) sensor_data_buffer, navdata, navdata_ix,  &sensor_params, outbuf);
                                        }
                                        else{
                                            datapoints = 0;
                                        }
                                        more_data_to_process = 0;
                                        break;
                                    case sensor_mode_sim:
                                        datapoints = sim_georef_data( navdata, navdata_ix,  &sensor_params, outbuf);
                                        more_data_to_process = 0;
                                        break;
                                    case sensor_mode_velodyne:
                                        datapoints = velodyne_georef_data( (uint16_t *) sensor_data_buffer, navdata, navdata_ix, &sensor_params,         outbuf);
                                        more_data_to_process = 0;
                                        break;
                                    case sensor_mode_lakibeam:
                                        datapoints = lakibeam_georef_data( (uint16_t *) sensor_data_buffer, navdata, navdata_ix, &sensor_params,         outbuf);
                                        more_data_to_process = 0;
                                        break;
                                    case sensor_mode_s7k:
                                        datapoints = s7k_georef_data( sensor_data_buffer,sensor_data_buffer_len, navdata, navdata_ix, &sensor_params,                          outbuf);
                                        more_data_to_process = 0;
                                        break;
                                    case sensor_mode_gsf:
                                        datapoints = gsf_georef_data( sensor_data_buffer,sensor_data_buffer_len, navdata, navdata_ix, &sensor_params,sector,                     outbuf);
                                        sector += 1;
                                        if (datapoints == -1){
                                            datapoints = 0;
                                            more_data_to_process = 0;
                                        }
                                        break;
                                    case sensor_mode_3dss_stream:
                                        datapoints = p3dss_georef_data( sensor_data_buffer,sensor_data_buffer_len, navdata, navdata_ix, &sensor_params,                          outbuf);
                                        more_data_to_process = 0;
                                        break;
                                    default:
                                        datapoints = 0;
                                        more_data_to_process = 0;
                                }


                                #define LIMIT_OUTPUT_WHEN_STATIONARY_IN_SIMULATED_SENSOR_MODE
                                #ifdef LIMIT_OUTPUT_WHEN_STATIONARY_IN_SIMULATED_SENSOR_MODE
                                static double last_stationary_ts=0;
                                if ((input_sensor_source==i_sim) && (input_navigation_source!=i_file)){
                                    // If vessel is stationary
                                    //fprintf(stderr, "Speed = %f, ts=%f last_stationary_ts=%f\n", navdata[navdata_ix].speed, ts_pos, last_stationary_ts);
                                    if (navdata[navdata_ix].speed < 0.10){
                                        //Every 30 sec we pass data through anyways
                                        if (ABS(last_stationary_ts - ts_pos) > 30.){
                                            last_stationary_ts = ts_pos;
                                        }
                                        //Otherwise we just skip it
                                        else{
                                            datapoints = 0;
                                        }
                                    }
                                }
                                #endif
                                //fprintf(stderr, "Datapoints = %d\n",datapoints);
                                if(datapoints) sensor_data_packet_counter++;
                                //fprintf(stderr,"(sensor_data_packet_counter=%d, sensor_params.data_skip=%d  sensor_params.data_length=%d\n",sensor_data_packet_counter,sensor_params.data_skip,sensor_params.data_length);
                                if (sensor_data_packet_counter<sensor_params.data_skip){}
                                else if (sensor_data_packet_counter && (sensor_data_packet_counter == sensor_params.data_skip)){
                                    time_t int_ts = (time_t) ts_sensor; 
                                    fprintf(stderr, "Skipping sensor packets up to packet %d  at time %s",sensor_data_packet_counter,ctime(&int_ts));
                                }
                                else if (sensor_params.data_length && (sensor_data_packet_counter==(sensor_params.data_skip+sensor_params.data_length))){
                                    time_t int_ts = (time_t) ts_sensor; 
                                    fprintf(stderr, "Skipping sensor packets from packet %d  at time %s",sensor_data_packet_counter,ctime(&int_ts));
                                }
                                else if (sensor_params.data_length && (sensor_data_packet_counter>=(sensor_params.data_skip+sensor_params.data_length))){}
                                else if (datapoints)
                                { 
                                    if (output_mode == output_binary)           len = write_bin_to_buffer(outbuf, datapoints, &(output_databuffer[output_databuffer_len]));
                                    else if (output_mode == output_csv)			len = write_csv_to_buffer(ts_sensor, outbuf, datapoints,navdata, navdata_ix, &aux_navdata,  output_format, &(output_databuffer[output_databuffer_len]));
                                    else if (output_mode == output_sbf)			len = write_sbf_to_buffer(x_offset,y_offset,z_offset,ts_offset,ts_sensor, outbuf, datapoints,navdata, navdata_ix, &aux_navdata,  output_format, &(output_databuffer[output_databuffer_len]));
                                    else if (output_mode == output_nmea)		len = write_nmea_to_buffer(ts_sensor, outbuf, datapoints,navdata, navdata_ix, &aux_navdata, &(output_databuffer[output_databuffer_len]));
                                    else if (output_mode == output_json)		len = write_json_to_buffer(ts_sensor, outbuf, datapoints,navdata, navdata_ix, &aux_navdata,  output_format, &(output_databuffer[output_databuffer_len]));
                                    else if (output_mode == output_s7k)			len = write_r7k_bathy_to_buffer(ts_sensor, outbuf, datapoints, &(output_databuffer[output_databuffer_len]));
                                    else len = 0;
                                    output_total_data += len;
                                    output_databuffer_len += len;
                                    time_t int_ts = (time_t) ts_sensor; 
                                    if (sensor_total_datapoints==0) fprintf(stderr, "Generating first %d datapoints from  packet %d  at time %s",datapoints,sensor_data_packet_counter,ctime(&int_ts));
                                    sensor_total_datapoints += datapoints;
                                }
                            }
                        }
                    new_sensor_data = 0;
					}
				}
				else {
					input_sensor_source = i_none;
					fprintf(stderr, "WBMS source = None\n");
				}
			}
		}
			
		if (output_databuffer_len){
			// ********* Write to drain *****************'
			switch (output_drain){ 
				case o_file: case o_stdout:
					fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
					break;
				case o_udp:
                    break;
				case o_tcp: 
					#ifdef ENABLE_NETWORK_IO
					FD_ZERO(&write_fds);
					for(ii = 0;ii< MAX_CLIENTS;ii++){
						output_client = output_clientlist[ii];
						if (output_client>0){ 
							FD_SET(output_client, &write_fds);
						}
					}
					#endif
					break;
			}
			dataset_counter++;
		}
	}
    
    #ifdef ENABLE_NETWORK_IO
    #if defined(_MSC_VER)
    close_windows_winsock_dll();
    #endif
    #endif
    
    //SBF format requires number of points to be written in header. So we must rewind the file and update this now that we know
    if((output_drain == o_file) && (output_mode==output_sbf)){
        fseek(output_fileptr, 0, SEEK_SET); //Rewind
        output_databuffer_len = write_sbf_header_to_buffer(output_format,sensor_total_datapoints,x_offset,y_offset,z_offset, output_databuffer);
		fwrite(output_databuffer,1,output_databuffer_len,output_fileptr );
        fprintf(stderr,"Updating SBF header to %d datapoints\n",sensor_total_datapoints);
        
        output_databuffer_len = write_sbf_meta_to_buffer(output_format,sensor_total_datapoints,x_offset,y_offset,z_offset, output_databuffer);
		fwrite(output_databuffer,1,output_databuffer_len,meta_output_fileptr );
    }
	
	fprintf(stderr, "Ray bending calculations %d valid, %d invalid\n", get_ray_bend_valid(), get_ray_bend_invalid());
    
    #ifdef COUNT_S7K_SNIPPET_SATURATION
    if (s7k_snp_count){
        fprintf(stderr, "S7K snippets saturation %d out of %d,  %.2f%%\n",s7k_snp_satcount,s7k_snp_count, (100.0*s7k_snp_satcount)/(1.0*s7k_snp_count));
    }
    #endif
    gsf_print_stats();
    r7k_print_stats();
    posmv_print_stats();

	if(output_drain == o_udp){}
	else if(output_drain == o_tcp){}
	else fclose(output_fileptr);
	#ifdef PREFER_HEAP
    free(sensor_data_buffer);
	free(navigation_data_buffer);
	free(output_databuffer);
	free(outbuf);
	#endif

	return(0);
}







