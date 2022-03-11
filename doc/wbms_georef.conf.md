```bash
# WBMS georef, Template config file

#### SENSOR MOUNTING ####
# Linear offsets are given in meters X=pos.forward, Y=pos.starboard, Z=pos.down
# Rotation offsets are given in degrees, Yaw=pos.turning starboard, Pitch=pos.tilting up, Roll=Rotating CW

# Sonar offset and roll relative to Navigation origo. 0,0,0 attitude is Rx facing downwards, Tx pointing aft
sensor_x_offset 0.0
sensor_y_offset 0.0
sensor_z_offset 0.0
sensor_yaw_offset 0.0
sensor_pitch_offset 0.0
sensor_roll_offset 0.0

#### SENSOR ERROR COMPENSATION ####
# Add a fixed range adjustment to sonar data
#sensor_r_error 0.0
# Add a time offset to sensor data timestamp
#sensor_time_offset 0.0
# Add a correction value to sonar SV measurement
#sensor_sv_offset 0.0

#### SENSOR DATA FILTERS ####
# Sounding quality limits
sensor_min_quality_flag 0
sensor_max_quality_flag 4
# Sounding priority limits
sensor_min_priority_flag 0
sensor_max_priority_flag 4
# Sounding multifreq select (-1 for all)
sensor_multifreq_index -1
# First bathy ata set to process, and max number of sets (0 fir all)
sensor_data_skip 0
sensor_data_length 0
# Global depth limits
sensor_min_depth -10000
sensor_max_depth 10000
sensor_min_range 0
sensor_max_range 10000
# Angular limits wrt sensor
sensor_min_azimuth -90
sensor_max_azimuth 90
sensor_min_elevation -90
sensor_max_elevation 90
sensor_min_ping_number 0
sensor_max_ping_number 0
# Limits wrt swath nadir
sensor_swath_min_y -1000
sensor_swath_max_y 1000

#### RAY TRACING PARAMETERS ####
# Ray tracing is always using sonar measured SV as inital SV
# Ray tracing mode and Mounting depth used for SV-prfile correction (raytracing).
#  Ray tracing mode 0: Fixed depth, LUT.
#  Ray tracing mode 1: Fixed depth, LUT.    Using a look up table for ray tracing adjustments. Assuming sensor at fixed depth (sonar_mounting_depth) wrt SV-profile 0
#  Ray tracing mode 2: Fixed depth, Direct. Using  direct ray tracing. Assuming sensor at fixed depth (sonar_mounting_depth) wrt SV-profile 0
#  Ray tracing mode 3: Variable depth, Direct. Using  direct ray tracing. Assuming sensor at variable depth based on navigation (nav+offset+sonar_mounting_depth) wrt SV-profile 0
ray_tracing_mode 1
# Sensor mounting depth im meters. With respect to SV-profile depth value
sensor_mounting_depth 1
# Which sv value to use for initial ray parameter when doing raytracing
raytrace_use_sonar_sv
#raytrace_use_table_sv

#### INTENSITY CORRECTION PARAMETERS ####
#  Set to 0 to disable range and AOI compensation of intensity
#  AOI compensation is either by model or from angle/intensity CSV file if given with -y option
# Uncomment to compensate intensity for range
#intensity_range_comp
# Uncomment to compensate intensity for AOI
# AOI compensation is either by model or from angle/intensity CSV file if given with -y option
#intensity_aoi_comp
# Uncomment to calculate aoi from data, otherwise assume flat seafloor
#calc_aoi 

#### ALTERNATIVE OUTPUT ####
# Uncomment to process with upper gate value instead of detection value
#upper_gate

# Uncomment to process with lower gate value instead of detection value
#lower_gate

# Uncomment to process with center gate value instead of detection value
#center_gate

#### SENSOR DATA FORMAT ####
# Sensor mode 1=WBMS 2=WBMS_V5 3=Velodyne  5=S7K 10=Autodetect (default)
# sensor_mode 10
# Uncomment to force wbms bathy data to be read as specific version
#force_bath_version 0

#### NAVIGATION DATA FORMAT ####
# Nav input parameters 
# Pos mode 0=POSMV 1=XTF_NAV 2=WBM_TOOL 3=SBET 4=SIM 5=S7K 10=Autodetect (default)
# pos_mode 10
# Set forward speed in m/s when using simulated navigation data 
pos_sim_speed 1
# Altitude mode 0=none 1=gps altitude, 2=heave 
alt_mode 1
vert_offset 0
input_timezone 0
# Maximum allowed time difference between sensor and navigation data
time_diff_limit 0.1
# Only for navigation data in projected coordinates, specify navigation data projection parameters
# projection_input +proj=utm +zone=33 +ellps=WGS84


#### DATA SOURCE ####
# Normally given as input argument with -i or -p and -s, but can also be specified here
# Define source and output drain: '-' for stdin/stdout ':' separated TCP ip/port '#' separated UDP ip/port else filename
# sensor_source localhost:2210
# navigation_source localhost#5602
# output localhost:9999


#### OUTPUT DATA FORMAT ####
# Uncomment to output in CSV format
#csv_output
# Uncomment to output in SBF format (CloudCompare)
sbf_output
# Uncomment to output in BIN format
#bin_output
# Write projection header to file output, only for CSV output
projection_header true

# Coordinate system definition string, as described in http://proj4.org/ Leave commented for auto detect
# projection_output +proj=utm +zone=33 +ellps=WGS84


#### OUTPUT DATA FORMAT FIELDS ####
# Output format string for SBF and CSV files
#  time stamp:                t
#  detection coordinates:     x,y,z
#  detection beam angle:      teta
#  detection range :          range
#  beam number :              beam
#  upper_gate_dist :          ugate
#  lower_gate_dist :          lgate
#  center_gate_dist :         cgate
#  gate width :               gatew
#  tx frequency :             freq
#  multiping index :          multiping
#  multifreq index :          multifreq
#  detection beam angle (electronic steering part):      steer
#  detection beam tx angle:    el
#  detection intensity value: val
#  detection swath y offset:  swath_y
#  detection quality:         quality
#  detection strength:        strength
#  vessel coordinates:        X,Y,Z
#  vessel attitude:           YAW,PITCH,ROLL
#  vessel pos accuracy:       HOR_ACC,VERT_ACC
#  speed of sound:            c
output_format x,y,z,val
# BIN output format is [(uint16) preamble , (uint16) type, (float) val, (double) x, (double) y, (double) z] where preamble is 0xBEEF 
# Here X,Y,Z depends on projection but will typically be in meters Easting, Northing, Altitude
# Val is intensity values, as given by either sonar or lidar
```
