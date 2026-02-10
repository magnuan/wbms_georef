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
#include "gsf_wrapper.h"
#include "velodyne.h"
#include "lakibeam.h"
#include "misc.h"
#include "sensordata_abstractor.h"

const char *sensor_mode_names[] = {
	"-",
	"WBMS",
	"WBMS_V5",
    "Velodyne",
    "Simulator",
    "s7k",
    "PingDSP 3DSS stream",
    "gsf",
    "Lakibeam",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "Autodetect",
    "Unknown"
};

char *sensor_mode_short_names[] = {
	"-",
	"wbms",
	"wbms",
    "velodyne",
    "sim",
    "s7k",
    "3dss",
    "gsf",
    "lakibeam",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "-",
    "auto",
    "unknown"
};

// Get the maximum range fond in sensor data file
// Returning 0, means that max range could not be computed
float sensor_get_max_range(int fd, sensor_mode_e mode){
    switch (mode){
        case  sensor_mode_wbms: case sensor_mode_wbms_v5:
            return wbms_get_max_range(fd);
        case sensor_mode_s7k:	
            return r7k_get_max_range(fd);
        default:
        case sensor_mode_gsf:	
        case sensor_mode_3dss_stream:	
        case sensor_mode_lakibeam:
        case sensor_mode_velodyne:
        case sensor_mode_autodetect: 
        case sensor_mode_unknown:
        case  sensor_mode_sim:
            return 0.;
    }
}

void sensor_get_sv_range(int fd, sensor_mode_e mode, float* min_sv, float* max_sv){
    *min_sv = 0.;
    *max_sv = 0.;
    switch (mode){
        case  sensor_mode_wbms: case sensor_mode_wbms_v5:
            wbms_get_sv_range(fd, min_sv, max_sv);
            break;
        case sensor_mode_s7k:	
            r7k_get_sv_range(fd, min_sv, max_sv);
            break;
        case sensor_mode_gsf:	
            gsf_get_sv_range(fd, min_sv, max_sv);
            break;
        case sensor_mode_3dss_stream:	
            break;
        default:
        case sensor_mode_lakibeam:
        case sensor_mode_velodyne:
        case sensor_mode_autodetect: 
        case sensor_mode_unknown:
        case  sensor_mode_sim:
            *min_sv=1500.;
            *max_sv=1500.;
            break;
    }
    return;
}

uint8_t sensor_test_file(int fd, sensor_mode_e mode, int* version){
    if(version){
        *version = -1;
    }
    switch (mode){
        case  sensor_mode_wbms: case sensor_mode_wbms_v5:
            return wbms_test_file(fd,version);
        case  sensor_mode_sim:
            return 1;
        case sensor_mode_lakibeam:
            return lakibeam_test_file(fd);
        case sensor_mode_velodyne:
            return velodyne_test_file(fd);
        case sensor_mode_s7k:	
            return r7k_test_bathy_file(fd);
        case sensor_mode_gsf:	
            return gsf_test_bathy_file(fd);
        case sensor_mode_3dss_stream:	
            return p3dss_test_bathy_file(fd);
        default:
        case sensor_mode_autodetect: //TODO fix this
        case sensor_mode_unknown:
            return 0;
    }
    return 0;
}

sensor_mode_e sensor_autodetect_file(FILE* fp){
    int fd = fileno(fp);
    int version;
    sensor_mode_e ret = sensor_mode_unknown;

    for (sensor_mode_e mode=sensor_mode_wbms; mode<sensor_mode_end;mode++){
        fprintf(stderr,"Testing sensor file in mode %s\n", sensor_mode_names[mode]);
        if(mode==sensor_mode_sim) continue;
        if (sensor_test_file(fd,mode,&version)){
            ret = mode;
            break;
        }
        fseek(fp,0,SEEK_SET);
    }
    fseek(fp,0,SEEK_SET);
    fprintf(stderr,"Autodetecting sensor file to mode: %s version %d\r\n",sensor_mode_names[ret],version);			

    return ret;

}




int sensor_fetch_next_packet(char * data, int fd, sensor_mode_e mode){ 
    int len=0;
    //fprintf(stderr,"sensor_fetch_next_packet mode=%d\n",mode);
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			len = wbms_fetch_next_packet(data, fd);break;
		case sensor_mode_velodyne:
			len = velodyne_fetch_next_packet(data, fd);break;
		case sensor_mode_lakibeam:
			len = lakibeam_fetch_next_packet(data, fd);break;
		case sensor_mode_s7k:	
			len = r7k_fetch_next_packet(data, fd);break;
		case sensor_mode_gsf:	
			len = gsf_fetch_next_packet(data, fd);break;
		case sensor_mode_3dss_stream:	
			len = p3dss_fetch_next_packet(data, fd);break;
		case  sensor_mode_sim:
			len = sim_fetch_next_packet(data, fd);break;
		case sensor_mode_autodetect: //TODO fix this
        default:
		case sensor_mode_unknown:
			len = 0;break;
	}
    if (len>MAX_SENSOR_PACKET_SIZE){
        fprintf(stderr,"ERROR: Over sized sensor data = %d, MAX=%d\n", len,MAX_SENSOR_PACKET_SIZE);
        return(-1);
    };
	return len;
}

int sensor_identify_packet(char* databuffer, uint32_t len, double ts_in, double* ts_out, sensor_mode_e mode){
    int id;
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			return wbms_identify_packet(databuffer, len, ts_out, NULL);
		case sensor_mode_velodyne:
			return velodyne_identify_packet(databuffer, len, ts_out, ts_in);
		case sensor_mode_lakibeam:
			return lakibeam_identify_packet(databuffer, len, ts_out, ts_in);
		case sensor_mode_s7k:	
            id = r7k_identify_sensor_packet(databuffer, len, ts_out);
	        //So far we only process s7k record 7027 and 10018 for sensor  bathy data and 7610 for SV data
            return ((id==7027) ||(id==7028) ||(id==7058)|| (id==7610) || (id==7000) ||(id==10000)||(id==10018))?id:0;
		case sensor_mode_gsf:	
            id = gsf_identify_sensor_packet(databuffer, len, ts_out);
            return id;
		case sensor_mode_3dss_stream:	
            return  p3dss_identify_sensor_packet(databuffer, len, ts_out);
		case  sensor_mode_sim:
			return sim_identify_packet(databuffer, len, ts_out, ts_in);
		case sensor_mode_autodetect: //TODO fix this
        default:
		case sensor_mode_unknown:
			return 0;
	}
	return 0;
}

int sensor_num_record_types(sensor_mode_e mode){
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			return wbms_num_record_types();
		case sensor_mode_velodyne:
			return 0; //TODO
		case sensor_mode_lakibeam:
			return 0; //TODO
		case sensor_mode_s7k:	
            return r7k_num_record_types();
		case sensor_mode_gsf:	
            return gsf_num_record_types();
		case sensor_mode_3dss_stream:	
            return  0; //TODO
		case  sensor_mode_sim:
			return 0;
        default:
		case sensor_mode_autodetect:
		case sensor_mode_unknown:
			return 0;
	}
	return 0;
}

int sensor_get_record_count(sensor_mode_e mode, record_count_t* records){
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			return wbms_get_record_count(records);
		case sensor_mode_velodyne:
			return 0; //TODO
		case sensor_mode_lakibeam:
			return 0; //TODO
		case sensor_mode_s7k:	
            return r7k_get_record_count(records);
		case sensor_mode_gsf:	
            return gsf_get_record_count(records);
		case sensor_mode_3dss_stream:	
            return  0; //TODO
		case  sensor_mode_sim:
			return 0;
        default:
		case sensor_mode_autodetect:
		case sensor_mode_unknown:
			return 0;
	}
	return 0;
}

const char *  sensor_get_data_type(sensor_mode_e mode){
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			return wbms_get_data_type();
		case sensor_mode_velodyne:
		case sensor_mode_lakibeam:
			return "lidar";
		case sensor_mode_s7k:	
            return r7k_get_data_type();
		case sensor_mode_gsf:	
            return gsf_get_data_type();
		case sensor_mode_3dss_stream:	
            return  "mbes";
		case  sensor_mode_sim:
			return "sim";
		case sensor_mode_autodetect:
		case sensor_mode_unknown:
        default:
			return "unknown";
	}
}
    
sensor_count_stats_t* sensor_get_count_stats(sensor_mode_e mode){
	switch (mode){
		case  sensor_mode_wbms: case sensor_mode_wbms_v5:
			return wbms_get_count_stats();
		case sensor_mode_velodyne:
		case sensor_mode_lakibeam:
			return NULL;
		case sensor_mode_s7k:	
			return s7k_get_count_stats();
		case sensor_mode_gsf:	
			return gsf_get_count_stats();
		case sensor_mode_3dss_stream:	
            return  NULL;
		case  sensor_mode_sim:
			return NULL;
		case sensor_mode_autodetect:
		case sensor_mode_unknown:
        default:
			return NULL;
	}
}
