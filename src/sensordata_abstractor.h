#ifndef _SENSORDATA_ABSTRACTOR_H_
#define _SENSORDATA_ABSTRACTOR_H_

typedef enum  {sensor_mode_wbms=1,sensor_mode_wbms_v5=2, sensor_mode_velodyne=3, sensor_mode_sim=4, sensor_mode_s7k=5, sensor_mode_3dss_stream=6, sensor_mode_autodetect=10,sensor_mode_unknown=11} sensor_mode_e; 
static const char *sensor_mode_names[] = {
	"-",
	"WBMS",
	"WBMS_V5",
    "Velodyne",
    "Simulator",
    "s7k",
    "PingDSP 3DSS stream",
    "-",
    "-",
    "-",
    "Autodetect",
    "Unknown"
};

#endif
