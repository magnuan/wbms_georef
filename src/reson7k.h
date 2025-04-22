#ifndef __RESON7K_H__
#define __RESON7K_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"

/*Downstream:
7500 7k Remote Control

Upstream:
Single records:
7001 Config data
7503 System state


Subscription records:
7000 Sonar settings
7004 Beam Geometry
7006 Bottom detection results
7027 Raw bathemetry 

Each packet is max 60.000 Bytes, larger records are fragmented with a NETWORK_FRAME field for reconstruction

7k Record
-network frame
-data record frame
-record type header
-(record data section)
-(extra data section)
-Checksum (4 bytes)

typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_T DRF;
	r7k_RecordTypeHeader_xxxx_t RTH;
	r7k_RecordData_xxxx_t RD;
	//r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record;
*/

#define MAX_S7K_PACKET_SIZE (4*1024*1024)
#define S7K_ID_MAX 65535

#pragma pack(1)
typedef struct{
	uint16_t year; // i.ex. 2013
	uint16_t day; // 1 - 366
	float	 sec; //0.0 - 59.999
	uint8_t  hour; //0-23
	uint8_t  min; //0-59
	uint16_t record_ver;			//Moved the previously reserved 16-bit entry into here to make the size a multiple of 4
}r7k_Time_t;	//12B

typedef struct{
	uint16_t version; // (= 5) Protocol version if this frame
	uint16_t offset;  //sizeof(r7k_NetworkFrame)
	uint32_t tot_packets; // = 1
	uint16_t tot_records; // = 1
	uint16_t trans_id; // = 1
	uint32_t packet_size; //Size of this packet
	uint32_t tot_size; //Size of all packets in transmission (excluding network frames)
	uint32_t seq_num;  //Sequential packet number
	uint32_t dest_dev_id;
	uint16_t dest_dev_enum;
	uint16_t src_dev_enum;
	uint32_t src_dev_id;

}r7k_NetworkFrame_t;


typedef struct{
	uint16_t version; // (= 5) Protocol version of this frame
	uint16_t offset; //Size from start of sync pattern to start of RecordTypeHeader = sizeof(r7k_DataRecordFrame) - 4
	uint32_t sync; //0x0000FFFF
	uint32_t size; //From start of r7k_DataRecordFrame.version to end of r7k_Checksum
	uint32_t opt_offset; // = 0
	uint32_t opt_id; // = 0
	r7k_Time_t time;
	uint32_t record_id;
	uint32_t dev_id;
	uint32_t dev_enum;	// To differ between devices with same dev_id
	uint32_t res2;
	uint16_t flags;
	uint16_t res3;
	uint32_t res4;
	uint32_t tot_frag_num;
	uint32_t frag_num;
}r7k_DataRecordFrame_t; //64B

typedef struct{
	uint32_t checksum;
}r7k_Checksum_t;

 
// ****7000 Sonar settings****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t ping_nr; //sequential number
	uint16_t multi_ping; // = 0
	float tx_freq; //Tx frequency in Hz
	float fs; //Sample rate in Hz
	float bw; //Receiver Bandwidth in Hz (always 0 ?)
	float tx_len; //Tx pulse length in sec
	uint32_t tx_type; // 0 = CW, 1 = Chirp

	uint32_t tx_taper; // 0 = tapered rect, 1 = Tukey
	float tx_taper_param; // = 0
	uint16_t tx_pulse_mode; // = 1 (single ping)
	uint16_t tx_pulse_res; // = 0
	float max_ping_rate;
	float ping_period; //Seconds since last ping
	float range; // in Meter
	float tx_power; // in dB rel 1 uPa
	float gain; // in dB
	uint32_t ctrl_flags;
	
    uint32_t project_id; //Projector identifier
	float project_steer_vert; //Projector stering in radians
	float project_steer_hor; //Projector steering in radians
	float project_open_vert; // -3dB in radians
	float project_open_hor; // -3dB in radians
	float project_focus; // in m
	uint32_t project_taper; // 0 = rect, 1 = Cheb
	
    float project_taper_param; // = 0
	uint32_t tx_flags; // = 0 (0-3 Pitch stab meth 4-7 Yaw stab meth)
	uint32_t hydrophone_id; //= 0
	uint32_t rx_taper; //0 = cheb 1 = kais
	float rx_taper_param; // = 0
	uint32_t rx_flags; 
	float rx_beam_witdth; // in rad
	float min_gate_range;
	float max_gate_range;
	float min_gate_depth;
	float max_gate_depth;
	float absorbtion; // in dB/km
	float sound_velocity; // in m/s
	float spread_loss; // in dB
	uint16_t res1;
}r7k_RecordTypeHeader_7000_t; //7k Sonar Settings  (No record data section)

typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7000_t RTH;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc

}r7k_Record_7000_t;


// ****7001 Configuration****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t N; // = 1 Number of devices / sonars
}r7k_RecordTypeHeader_7001_t; 
typedef struct{
	uint32_t id; //Unique id number
	char     desc[60]; //ASCII string decribing device
    uint32_t alpha_data_card; //Defines the type of the AlphaData card. 0x0400 – Virtex 2 card 0x0800 – Virtex 5 card 0x1000 – Virtex 6 card
	uint64_t serial;
	uint32_t info_len; // = 0 (in bytes)
	char	info[];	    //XML info about device
}r7k_RecordData_7001_t;

typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7001_t RTH;
	r7k_RecordData_7001_t RD;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc

}r7k_Record_7001_t;

// ****7004 Beam Geometry ****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t N; // = 256 Number of receiver beams
}r7k_RecordTypeHeader_7004_t;
typedef struct{
	float dir_vert[256]; //Elevation angles in radians, all = 0 
	float dir_hor[256]; //Azimuth angles in radians
	float open_vert[256]; //Elevation opening angle in radians
	float open_hor[256]; //Azimuth opening angle in radians
}r7k_RecordData_7004_t; 

typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7004_t RTH;
	r7k_RecordData_7004_t RD;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc

}r7k_Record_7004_t;


// ****7006 Bathymetric Data ****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t ping_nr; //sequential number
	uint16_t multi_ping; // = 0
	uint32_t N; // = 256 Number of receiver beams
	uint8_t flags; // = 0
	uint8_t snd_vel_flag; //0=measured 1=entered
	float sound_velocity; // in m/s
}r7k_RecordTypeHeader_7006_t;
typedef struct{
	float range[256]; // Two way travel time in secs
	uint8_t quality[256]; // [0:bright,1:colin,2:mag,3:phase,0...]
	float intensity[256];
	float min_gate_range[256];
	float max_gate_range[256];
}r7k_RecordData_7006_t; 
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7006_t RTH;
	r7k_RecordData_7006_t RD;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc

}r7k_Record_7006_t;

// ****7027 Raw detection Data ****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t ping_nr; //sequential number
	uint16_t multi_ping; // = 0
	uint32_t N; // = 256 Number of receiver beams
	uint32_t data_field_size; // sizeof(r7k_RecordTypeHeader_7027), Size of detection information block in bytes
	uint8_t detect_alg; //0=simple 1=blend_filt
	uint32_t flags; // = 0
	float fs; //Sample rate in Hz
	float tx_angle; // In radians
    float applied_roll;
	uint32_t res1[15]; 
}r7k_RecordTypeHeader_7027_t;
typedef struct{
	uint16_t beam; //Beam number the detection is taken from
	float detection_point; //Non-corrected fractional sample number with reference to receiver’s acoustic center with the zero sample at the transmit time
	float rx_angle; //Beam steering angle
	uint32_t flags; //[0:mag, 1:phase,2-8:qual = 1,0....]
	uint32_t quality; //[0: bright, 1:Colin,0...]
	float uncertainty;
    float signal_strength;
    float limit_min;
    float limit_max;
}r7k_RecordData_7027_t; 
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7027_t RTH;
	r7k_RecordData_7027_t RD[];
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_7027_t;


// ****7028 Snippet data ****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t ping_nr; //sequential number
	uint16_t multi_ping; // = 0
	uint16_t N; // = 256 Number of receiver beams
    uint8_t  error_flag;
    uint8_t  control_flag;
    uint32_t flags; //0:16-bit snippets  1:32-bit snippets 
	uint32_t res1[6]; 
}r7k_RecordTypeHeader_7028_t;
typedef struct{
    uint16_t beam_descriptor; 
    uint32_t snippet_start;
    uint32_t detection_sample;
    uint32_t snippet_end;
}r7k_SnippetDescriptor_7028_t;


typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7028_t RTH;
    r7k_SnippetDescriptor_7028_t snippet_descriptor[]; //Variable length record data, contains N snippet descriptors N*14 bytes
    //uintXX_t data[x];                     //Variable data of either uint32 or uint16
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_7028_t;

// ****7058 Snippet Backscattering strength ****
typedef struct{
	uint64_t serial; //Sonar serial number
	uint32_t ping_nr; //sequential number
	uint16_t multi_ping; // = 0
	uint16_t N; // = 256 Number of receiver beams
    uint8_t  error_flag;
    uint32_t  control_flag;
    float absorbtion; //Absorption value in dB/km. Only valid when control flag bit 8 is set.
	uint32_t res1[6]; 
}r7k_RecordTypeHeader_7058_t;
typedef struct{
    uint16_t beam_descriptor; 
    uint32_t snippet_start;
    uint32_t detection_sample;
    uint32_t snippet_end;
}r7k_SnippetDescriptor_7058_t;


typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_7058_t RTH;
    r7k_SnippetDescriptor_7058_t snippet_descriptor[]; //Variable length record data, contains N snippet descriptors N*14 bytes
    //float bs_data[x];                                //Variable data with backscatter data 10log10(BCS), for each sample in the N snippets, length is End-Start+1 samples of 32-bit float 
    //float footprint_data[x];                         //Variable data with footprint area in m^2, for each sample in the N snippets, length is End-Start+1 samples of 32-bit float 
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_7058_t;



// ****10000 SBES Channel settings ****
typedef struct{
	uint32_t ping_nr; //sequential number
	uint32_t multi_ping; // = 0
    uint32_t res1;
    uint32_t transport_latency;
    float    freq_center;
    float    sweep_width;
    float    fs;
    float    rx_bw;
    float    absorbtion;
    float    spreading_loss;
    float    initial_gain;
    float    range;
    float    res2;
    float    power;
    float    pulse_length;
    uint32_t pulse_type;
    uint32_t pulse_envelope_type;
    float    pulse_envelope_param;
    uint32_t multi_ping_count;
    uint32_t res3;
    float    max_ping_rate;
    float    ping_period;
    float    res4;
    uint32_t tx_id;
    float    tx_beam_width_across;
    float    tx_beam_width_along;
    uint32_t rx_id;
    float    rx_beam_width_across;
    float    rx_beam_width_along;
    float    corrected_transducer_draft;
    float    absorbtion_tweak;
    float    pulse_length_tweak;
    float    spreading_loss_tweak;
    float    initial_gain_tweak;
    float    range_tweak;
    float    power_tweak;
    float    min_gate;
    float    max_gate;
    uint32_t gate_mode;
    float    tx_draft;
    float    tx_index_correction;
}r7k_RecordTypeHeader_10000_t;
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_10000_t RTH;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_10000_t;



// ****10018 SBES Echogram Water Column Data ****
typedef struct{
	uint32_t ping_nr; //sequential number
	uint32_t multi_ping; // = 0
    float sound_velocity; // in m/s
    float not_applied_corrected_transducer_draft; // [meter]. Draft is not applied to data in this record. Clients may apply this value to display water depth echograms.
    float fs; //Sample rate in Hz
    float effective_pulse_length; // For CW this is the time duration of the transmitted pulse. [seconds].
    float start_sample_delay;  // [seconds]. The TWT corresponding to the first sample in this record. Shift the echogram down by this amount.
    uint32_t bits_per_sample;  // 8,16,32
    uint32_t full_scale; //  Highest Value /echo strength possible.
    uint32_t number_of_samples; //
}r7k_RecordTypeHeader_10018_t;
typedef struct{
	uint32_t sample_data;
}r7k_RecordData_10018_t; 
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
	r7k_RecordTypeHeader_10018_t RTH;
	r7k_RecordData_10018_t RD[];
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_10018_t;


// ****7030 Sonar installation parameters ****
typedef struct{
    float Frequency;
    uint16_t Length_of_firmware_version_info;
    char  Firmware_version_info[128];
    uint16_t Length_of_software_version_info;
    char  Software_version_info[128];
    uint16_t  Length_of_s7k_software_version_info;
    char  s7k_software_version_info[128];
    uint16_t  Length_of_record_protocol_info;
    char  Record_protocol_version_info[128];
    float  Transmit_array_X; //X_offset_in_meters
    float  Transmit_array_Y; //Y_offset_in_meters
    float  Transmit_array_Z;//Z_offset_in_meters
    float  Transmit_array_roll; //Radians
    float  Transmit_array_pitch; //Radians
    float  Transmit_array_heading; //Radians
    float  Receive_array_X; //X_offset_in_meters
    float  Receive_array_Y; //Y_offset_in_meters
    float  Receive_array_Z; //Z_offset_in_meters
    float  Receive_array_roll; //Radians
    float  Receive_array_pitch; //Radians
    float  Receive_array_heading; //Radians
    float  Motion_sensor_X; //X_offset_in_meters
    float  Motion_sensor_Y; //Y_offset_in_meters
    float  Motion_sensor_Z; //Z_offset_in_meters
    float  Motion_sensor_roll_calibration; //Radians
    float  Motion_sensor_pitch_calibration; //Radians
    float  Motion_sensor_heading_calibration; //Radians
    uint16_t  Motion_sensor_time_delay; //Milliseconds
    float  Position_sensor_X; //X_offset_in_meters
    float  Position_sensor_Y; //Y_offset_in_meters
    float  Position_sensor_Z; //Z_offset_in_meters
    uint16_t  Position_sensor_time_delay; //Milliseconds_
    float  Water_line_vertical_offset; //Vertical offset from reference point to waterline in meters
}r7k_RecordTypeHeader_7030_t;
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
    r7k_RecordTypeHeader_7030_t RTH;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_7030_t;

// ****7200 Sonar installation parameters ****
typedef struct{
    uint64_t File_identifier[2];
    uint16_t Version_number; //File_format_version_number
    uint16_t Reserved;
    uint64_t Session_identifier[2]; //User_defined_session_identifier._Used_to_associate_multiple_files_for_a_given_session.
    uint32_t Record_data_size; //Size_of_record_data._0_–_If_not_present
    uint32_t N; //Number_of_devices_(N_≥_0)
    char Recording_name[64]; //Null_terminated_UTF-8_string
    char Recording_program_version_number[16]; //Null_terminated_UTF-8_string
    char User_defined_name[64];
    char Notes[128];
}r7k_RecordTypeHeader_7200_t;
typedef struct{
    uint32_t dev_id;    // Identifier for record type of embedded data
    uint16_t sys_enum;  // Identifier for the device enumerator
}r7k_RecordData_7200_t; 
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
    r7k_RecordTypeHeader_7200_t RTH;
	r7k_RecordData_7200_t RD[];
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc
}r7k_Record_7200_t;

// ****7610 Sound velocity ****
typedef struct{
	float sv; //Sound velocity in m/s
}r7k_RecordTypeHeader_7610_t;
typedef struct{
	r7k_NetworkFrame_t NF;
	r7k_DataRecordFrame_t DRF;
    r7k_RecordTypeHeader_7610_t RTH;
    //r7k_Checksum_t CS; // Checksum must be added separately to allow for variable length payload in struct/gc

}r7k_Record_7610_t;



// ****1003 Position ****
typedef struct{
	uint32_t 	datum;
	float	 	latency;
	double	 	lat_northing;
	double		lon_easting;
	double	 	height;
	uint8_t		pos_type;
	uint8_t		utm_zone;
	uint8_t		quality;
	uint8_t		pos_method;
}r7k_RecordTypeHeader_1003_t;  //36B

// ****1006 Altitude ****
typedef struct{
	float	 	distance;       //Distance from seafloor in meters to sensor, positive up (0 at sea bottom).
}r7k_RecordTypeHeader_1006_t;  //4B

// ****1008 Depth ****
typedef struct{
	uint8_t		depth_descriptor; //  0 – Depth to sensor 1 – Water depth
	uint8_t		correction_flag; //  0 – RAW depth (as measured) 1 – Corrected depth (relative to mean-sea level)
	uint16_t    res1;
	float	 	depth;       //The deeper, the bigger (positive) this value becomes
}r7k_RecordTypeHeader_1008_t;  //8B

// ****1012 Roll Pitch Heave ****
typedef struct{
	float		roll;
	float		pitch;
	float		heave;
}r7k_RecordTypeHeader_1012_t;  //12B


// ****1013 Heading ****
typedef struct{
	float		heading;
}r7k_RecordTypeHeader_1013_t;  //4B

// ****1015 Navigation ****
typedef struct{
	uint8_t		vert_ref;
	double		lat;
	double		lon;
	float		hor_accuracy;
	float		height;
	float		vert_accuracy;
	float		speed_over_ground;
	float		course_over_ground;
	float		heading;
}r7k_RecordTypeHeader_1015_t;  

// ****1016 Attitude ****
typedef struct{
	uint16_t	t_off_ms;
	float		roll;
	float		pitch;
	float		heave;
	float		heading;		
}r7k_RecordData_1016_entry_t;

typedef struct{
	uint8_t		N; //Number of sets in record data
	r7k_RecordData_1016_entry_t entry[];
}r7k_RecordTypeHeader_1016_t;  //


#pragma pack(4)
// ****Dummy ****
typedef struct{
	uint8_t data[32];
}r7k_RecordTypeHeader_dummy_t;

union r7k_RecordTypeHeader{
	r7k_RecordTypeHeader_dummy_t* dummy;

	r7k_RecordTypeHeader_7000_t* r7000;
	r7k_RecordTypeHeader_7001_t* r7001;
	r7k_RecordTypeHeader_7004_t* r7004;
	r7k_RecordTypeHeader_7006_t* r7006;
	r7k_RecordTypeHeader_7027_t* r7027;
	r7k_RecordTypeHeader_7028_t* r7028;
	r7k_RecordTypeHeader_7058_t* r7058;
	r7k_RecordTypeHeader_7610_t* r7610;
	r7k_RecordTypeHeader_10000_t* r10000;
	r7k_RecordTypeHeader_10018_t* r10018;

	r7k_RecordTypeHeader_1003_t* r1003;
	r7k_RecordTypeHeader_1006_t* r1006;
	r7k_RecordTypeHeader_1008_t* r1008;
	r7k_RecordTypeHeader_1012_t* r1012;
	r7k_RecordTypeHeader_1013_t* r1013;
	r7k_RecordTypeHeader_1015_t* r1015;
	r7k_RecordTypeHeader_1016_t* r1016;
};



#pragma pack()

void r7k_init(void);
void r7k_print_stats(void);
void r7k_set_sensor_offset(offset_t* s);
uint8_t r7k_test_nav_file(int fd);
uint8_t r7k_test_bathy_file(int fd);
void r7k_calc_checksum(r7k_DataRecordFrame_t* drf);
void r7k_ts_to_r7ktime(double ts,r7k_Time_t* t);
double r7k_r7ktime_to_ts(r7k_Time_t* t);
int r7k_seek_next_header(int fd, /*out*/ uint8_t* pre_sync);
int r7k_fetch_next_packet(char * data, int fd);
int r7k_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out);
int s7k_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);
uint32_t s7k_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/ output_data_t* outbuf);
uint32_t s7k_count_data( char* databuffer,uint32_t databuffer_len, double* ts);
uint32_t r7k_num_record_types(void);
uint32_t r7k_get_record_count(record_count_t* records);
const char * r7k_get_data_type(void);

#endif
