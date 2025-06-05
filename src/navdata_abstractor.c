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
#include "3dss_dx.h"
#include "reson7k.h"
#include "gsf_wrapper.h"
#include "posmv.h"
#include "xtf_nav.h"
#include "sim_nav.h"
#include "sbet_nav.h"
#include "eelume_sbd_nav.h"
#include "nmea_nav.h"
#include "wbm_tool_nav.h"
#include "navdata_abstractor.h"

navdata_t navdata[NAVDATA_BUFFER_LEN];
size_t navdata_ix = 0;
uint32_t navdata_count = 0;
aux_navdata_t aux_navdata;
uint16_t navdata_alt_mode = 1;
PJ *proj_latlon_to_output_utm;

const char *pos_mode_names[] = {
	"Posmv",
	"XTF_nav",
	"wbm_tool dump",
    "SBET",
    "Simulator",
    "s7k",
    "PingDSP 3DSS stream",
    "gsf",
    "nmea",
    "SBET CSV",
    "eelume sbd",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "Autodetect",
    "Unknown"
};
const char *pos_mode_short_names[] = {
	"posmv",
	"csv",
	"wbm_dump",
    "sbet",
    "sim",
    "s7k",
    "3dss",
    "gsf",
    "nmea",
    "sbet_csv",
    "sbd",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "auto",
    "unknown"
};

uint8_t navigation_test_file(int fd, pos_mode_e mode){
    switch (mode){
        case pos_mode_posmv:    return  posmv_test_file(fd);
        case pos_mode_sbet:     return  sbet_test_file(fd);
        case pos_mode_sbet_csv:     return  sbet_csv_test_file(fd);
        case pos_mode_xtf:      return  xtf_test_file(fd);
        case pos_mode_wbm_tool: return  wbm_tool_nav_test_file(fd);
        case pos_mode_sim:      return 1;
        case pos_mode_s7k:      return  r7k_test_nav_file(fd);
        case pos_mode_gsf:      return  gsf_test_nav_file(fd);
        case pos_mode_3dss_stream:      return  p3dss_test_nav_file(fd);
        case pos_mode_nmea:      return  nmea_nav_test_file(fd);
        case pos_mode_eelume:      return  eelume_sbd_nav_test_file(fd);
        default: case pos_mode_autodetect: case pos_mode_unknown: return 0;
    }
    return 0;
}

pos_mode_e navigation_autodetect_file(FILE* fp){
    int fd = fileno(fp);
    pos_mode_e ret = pos_mode_unknown;

    for (pos_mode_e mode=pos_mode_posmv; mode<=pos_mode_end;mode++){
        if(mode==pos_mode_sim) continue;
        fprintf(stderr,"Testing nav file in mode %s\n", pos_mode_names[mode]);
        if (navigation_test_file(fd,mode)){
            ret = mode;
            break;
        }
        fseek(fp,0,SEEK_SET);
    }
    fseek(fp,0,SEEK_SET);
    fprintf(stderr,"Autodetecting navigation file to mode: %s\r\n",pos_mode_names[ret]);			
    return ret;

}


//Read next set of pos data from fd into data
int navigation_fetch_next_packet(char * data, int fd, pos_mode_e mode){
    int len = 0;
    switch (mode){
        case pos_mode_posmv:    len= posmv_fetch_next_packet(data,fd);break;
        case pos_mode_sbet:     len= sbet_nav_fetch_next_packet(data,fd);break;
        case pos_mode_sbet_csv:     len= sbet_csv_nav_fetch_next_packet(data,fd);break;
        case pos_mode_xtf:      len= xtf_nav_fetch_next_packet(data,fd);break;
        case pos_mode_wbm_tool: len= wbm_tool_nav_fetch_next_packet(data,fd);break;
        case pos_mode_sim:      len= sim_nav_fetch_next_packet(data,fd);break;
        case pos_mode_s7k:      len= r7k_fetch_next_packet(data,fd);break;
        case pos_mode_gsf:      len= gsf_fetch_next_packet(data,fd);break;
        case pos_mode_3dss_stream:      len= p3dss_fetch_next_packet(data,fd);break;
        case pos_mode_eelume:    len= eelume_sbd_nav_fetch_next_packet(data,fd);break;
        case pos_mode_nmea:      len= nmea_nav_fetch_next_packet(data,fd);break;
        default: case pos_mode_autodetect: case pos_mode_unknown: return -1;
    }
    //fprintf(stderr,"navigation_fetch_next_packet = %d\n",len);
    if (len>MAX_NAVIGATION_PACKET_SIZE){
        fprintf(stderr,"ERROR: Over sized navigation data = %d, MAX=%d\n", len,MAX_NAVIGATION_PACKET_SIZE);
        return(-1);
    };
	return len;
}


int process_nav_data_packet(char* databuffer, uint32_t len, double ts_in, double* ts_out, pos_mode_e mode, double z_offset){
    int ret = 0;
    size_t next_navdata_ix = (navdata_ix+1)%NAVDATA_BUFFER_LEN;
        
    switch (mode){
        case pos_mode_posmv:    ret= posmv_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_sbet:     ret= sbet_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_sbet_csv:     ret= sbet_csv_nav_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_xtf:      ret = xtf_nav_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_wbm_tool: ret= wbm_tool_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_sim:      ret = sim_nav_process_packet(ts_in,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_s7k:      ret = s7k_process_nav_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_gsf:      ret = gsf_process_nav_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_3dss_stream:      ret = p3dss_process_nav_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_eelume:    ret = eelume_sbd_nav_process_packet(databuffer,len,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        case pos_mode_nmea:      ret = nmea_nav_process_nav_packet(databuffer,len,0,ts_out,z_offset, navdata_alt_mode, proj_latlon_to_output_utm, &(navdata[next_navdata_ix]),&aux_navdata);break;
        default: case pos_mode_autodetect: case pos_mode_unknown: return 0;
    }
    if (ret == NAV_DATA_PROJECTED){  //Only count when projected coordinates are returned
        navdata_ix = next_navdata_ix;  //navdata_ix now points to the last updated nav data set
        navdata_count +=1;
    }
	return ret;
}

int navigation_num_record_types(pos_mode_e mode){
	switch (mode){
		case pos_mode_s7k:	
            return r7k_num_record_types();
		case pos_mode_gsf:	
            return gsf_num_record_types();
		case pos_mode_posmv:	
            return posmv_num_record_types();
        default:
			return 0;
	}
	return 0;
}

int navigation_get_record_count(pos_mode_e mode, record_count_t* records){
	switch (mode){
		case pos_mode_s7k:	
            return r7k_get_record_count(records);
		case pos_mode_gsf:	
            return gsf_get_record_count(records);
		case pos_mode_posmv:	
            return posmv_get_record_count(records);
        default:
			return 0;
	}
	return 0;
}
