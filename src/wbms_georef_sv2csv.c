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
#include "time_functions.h"
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



void showUsage(char *pgmname)
{
		printf("Usage: %s file \n", pgmname);
		printf(	"Reads in a file as SVP data, and if success output as CSV\n");
}





/************************************** MAIN *********************************************/


int main(int argc,char *argv[])
{


	//File input and output stuff
    static char * input_file_string = NULL;

   uint8_t run_filter=0;

	int c;
    while (optind < argc) {
        if ((c = getopt (argc, argv, "f?h")) != -1) {
            switch (c) {
                case 'f':
                    run_filter=1;
                    break;
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
    if (svp_mode==0) return -1;
    if (svp_datapoints==0) return -1;
	
    sv_meas_t* sv_meas = malloc(MAX_SV_MEAS*sizeof(sv_meas_t));
	sv_meas_t* sv_meas_filtered;
    
    int sv_meas_len = svp_read_from_file(input_file_string, sv_meas, MAX_SV_MEAS);
    if (svp_mode==svp_mode_ascii_tuple) svp_auto_swap_sv_depth(sv_meas, sv_meas_len);

    if (run_filter){
        sv_meas_len = svp_discard_insane(sv_meas, sv_meas_len);
        sv_meas_len = svp_extrapolate_sv_table(sv_meas,1.3f,sv_meas_len, MAX_SV_MEAS);
        sv_meas_len = svp_filter_and_resample(sv_meas, sv_meas_len, &sv_meas_filtered);
        free(sv_meas);
        sv_meas = sv_meas_filtered;
    }

    for (size_t ix=0; ix <sv_meas_len; ix++){
        fprintf(stdout,"%0.3f,\t %0.3f\n",sv_meas[ix].depth, sv_meas[ix].sv);
    }
    free(sv_meas);
}







