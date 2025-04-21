#ifndef __3DSS_DX_H__
#define __3DSS_DX_H__
#include "proj_wrapper.h"
#include "wbms_georef.h"


#define MAX_3DSS_PACKET_SIZE (4*1024*1024)

#pragma pack(1)
typedef struct{
    ///
    /// Seconds since January 1, 1970 12:00 AM.
    /// 
    uint64_t seconds;
    
    ///
    /// Nanoseconds (remainder from seconds).
    ///
    uint32_t nanoseconds;

    ///
    /// Proprietary status flags.
    ///
    uint32_t flags;
}p3dss_Time_t;	//16B

typedef struct{
    ///
    /// Bulk (average) water column sound velocity in m/s.
    /// 
    float bulk;

    ///
    /// Single point sound velocity at the sonar transducer front face in m/s.
    ///
    float face;
}p3dss_SoundVelocity_t;	//8B

typedef struct{
    ///
    /// Coefficient that specifies a constant or fixed gain in dB.
    ///
    float constant;

    ///
    /// Coefficient that specifies a gain that varies linearly with range in 
    /// dB/m.
    ///
    float linear;

    ///
    /// Coefficient that specifies a gain that varies logarithmically with
    /// range in dB/log10(m). 
    ///
    float logarithmic;
}p3dss_Gain_t;	//12B

typedef struct{
    ///
    /// The sonar time when first character of the sentence was received by 
    /// the sonar.
    ///
    p3dss_Time_t time;

    ///
    /// The Ascii sentence beginning with either a '$'or ':' character.
    /// The character array is null terminated without <CR><LF>.
    ///
    char sentence[256];

    ///
    /// An identifier indicating the sonar system origin of the sentence.
    ///
    uint32_t source;

    ///
    /// An identifier indicating the sonar serial port origin of the sentence.
    ///
    uint32_t port;
    
}p3dss_Ascii_t;	//16+256+4+4 = 280

typedef struct{
    ///
    /// The sonar time when first character of the message was received by 
    /// the sonar.
    ///
    p3dss_Time_t time;

    ///
    /// The SBG binary data message.  The binary format is descibed in detail
    /// in SBG documentation, but a simple description of the encapsulation is
    //  given below:
    ///
    /// Offset, Length (bytes), Field name, Description
    /// 0       2               sync        Fixed 2 bytes, 0xFF 0x5A always.
    /// 2       1               msg_id      Identifies message type.
    /// 3       1               msg_class   Identifies message class.
    /// 4       2               len         Little-endian length of data bytes.
    /// 6       len             data        Payload data bytes.
    /// 6+len   2               crc         CRC-16 checksum
    /// 8+len   1               etx         End of frame, 0x33 always.
    ///
    uint8_t bytes[256];	  

}p3dss_SBGMessage_t;	//272B


typedef struct{
    ///
    /// The sidescan mode of operation, specified by name. 
    ///
    char mode[32];

    ///
    /// The method of combining array elements for operation in incoherent
    /// mode, specified by name.
    ///
    char incoherent_method[32];

    ///
    /// The vertical sidescan beams used in coherent sidescan mode,
    /// specified as a list of beam names.  
    ///
    char coherent_beams[64];
}p3dss_SidescanSettings_t;	//128B

typedef struct{
    ///
    /// The number of receive samples to include in angle computations.
    ///
    int32_t smoothing;

    ///
    /// The tolerance for non-plane wave arrivals in angle computations.
    ///
    float tolerance;

    ///    
    /// The receive signal level threshold in dB re FS (fullscale) for angle 
    /// computations.
    ///
    float threshold;

    ///
    /// The number of angles to compute at each range step.
    ///
    int32_t number_of_angles;
}p3dss_Sidescan3DSettings_t;	//16B

typedef struct{
    ///
    /// The transmit beam angle (direction) in degrees relative to the maximum 
    /// response axis of the transducer. Positive angles are downward.
    ///
    float angle;

    ///
    /// The transmit power as a percentage of maximum.
    ///
    uint32_t power;

    ///
    /// The transmit beamwidth (beampattern) specified by name.
    ///
    char beamwidth[32];

    ///
    /// The transmit pulse waveform specified by name.
    ///
    char pulse[32];
}p3dss_TransmitSettings_t;	//72B

typedef struct{
    ///
    /// The trigger source used by the sonar specified by name.
    ///
    char source[32];

    ///
    /// The duty cycle specifying the trigger repetition rate as a fraction of 
    /// the maximum rate for the current sonar range setting. Only applicable
    /// when the trigger source is set to Continuous.
    ///
    float countinuousdutycycle;
  
    ///
    /// Reserved - likely ping rate in the future.
    /// \since 0.3
    ///
    float reserved;
}p3dss_TriggerSettings_t;	//40B

typedef struct{
    ///
    /// The range in meters to the sidescan data point.
    ///
    float range;

    ///
    /// The amplitude of the sidescan data point. 
    ///
    /// Amplitudes represent the magnitude of the received signal envelope 
    /// after gain and sidescan processing methods are applied.
    ///
    float amplitude;
}p3dss_SidescanPoint_t;	//8B

typedef struct{
    ///
    /// The range in meters to the sidescan-3D data point.    
    ///
    float range;

    ///
    /// The angle in radians of the sidescan-3D data point. Negative angles
    /// are downward from the maximum response axis of the transducer.    
    ///	
    float angle;

    ///
    /// The amplitude of the sidescan-3D data point after gain and sidescan 3D
    /// processing methods are applied.    
    ///
    float amplitude;
  
    ///
    /// Reserved for a future value, either SNR or quality factor for the point.
    /// \since 0.3
    ///
    float reserved;
}p3dss_Sidescan3DPoint_t;	//16B

typedef struct{
    ///
    /// The range in meters to the bathymetry data point.    
    ///
    float range;

    ///
    /// The angle in radians of the bathymetry data point. Negative angles
    /// are downward from the maximum response axis of the transducer.    
    ///	
    float angle;

    ///
    /// The amplitude of the bathymetry data point after gain and bathymetry
    /// processing methods are applied.    
    ///
    float amplitude;
  
    ///
    /// Reserved for a future value, either SNR or quality factor for the point.
    ///
    float reserved1;
	
    ///
    /// Reserved for a future value, either SNR or quality factor for the point.
    ///
    float reserved2;
}p3dss_BathymetryPoint_t;	//8B


typedef struct{
    ///
    /// The sonar range setting.    
    ///
    float range;

    ///
    /// The sonar trigger settings.
    ///
    p3dss_TriggerSettings_t trigger;

    ///
    /// The sonar sound velocity setting.
    ///
    p3dss_SoundVelocity_t sound_velocity;

    ///
    /// The sonar port-side receive gain setting.
    ///
    p3dss_Gain_t port_gain;

    ///
    /// the sonar starboard-side receive gain settings.
    ///
    p3dss_Gain_t starboard_gain;

    ///
    /// The sonar port-side sidescan settings.
    ///
    p3dss_SidescanSettings_t port_sidescan;

    ///
    /// The sonar starboard-side sidescan settings.
    ///
    p3dss_SidescanSettings_t starboard_sidescan;

    ///
    /// The sonar port-side sidescan-3d settings.
    ///
    p3dss_Sidescan3DSettings_t port_sidescan3d;

    ///
    /// The sonar starboard-side sidescan-3d settings.
    ///
    p3dss_Sidescan3DSettings_t starboard_sidescan3d;

    ///
    /// The sonar port-side transmit settings.
    ///
    p3dss_TransmitSettings_t port_transmit;

    ///
    /// The sonar starboard-side transmit settings.
    ///
    p3dss_TransmitSettings_t stardboard_transmit;

    ///
    /// Temporary reserved bytes and also padding to 8byte boundary.
    /// \since 0.3
    ///
    char reserved[68];
}p3dss_DxParameters_t;	
 
typedef struct{
    ///
    /// A 32 character null terminated string identifying the sonar.
    ///
    char id[32];

    ///
    /// The acoustic frequency of this sonar in Hertz.
    ///
    float acoustic_frequency;

    ///
    /// The sample rate for the sidescan data in Hertz.
    ///
    float sample_rate;

    ///
    /// The maximum ping rate for the sonar in Hertz for the current settings.
    ///
    float maximum_ping_rate;

    ///
    /// The range resolution in meters for the port sidescan data for this ping.
    ///
    float port_sidescan_range_resolution;

    ///
    /// The range resolution in meters for the starboard sidescan data.
    ///
    float starboard_sidescan_range_resolution;

    ///
    /// The range resolution in meters for the port sidescan-3d data.
    ///
    float port_sidescan3d_range_resolution;

    ///
    /// The range resolution in meters for the starboard sidescan-3d data.
    ///
    float starboard_sidescan3d_range_resolution;

    ///
    /// The angle of the port transducer in degrees as mounted in the sonar
    /// housing relative to horizontal and with positive values indicating 
    /// downward.
    ///
    float port_transducer_angle;

    ///
    /// The angle of the starboard transducer in degrees as mounted in the sonar
    /// housing relative to horizontal and with positive values indicating 
    /// downward.
    ///
    float starboard_transducer_angle;

    ///
    /// Temporary reserved bytes and also padding to 8byte boundary.
    ///
    char reserved[60];
}p3dss_DxSystemInfo_t;	


typedef struct{
    ///
    /// A unique 16 byte preamble.
    /// {0x50, 0x49, 0x4e, 0x47,0x27, 0x2b, 0x3a, 0xd8,0x74,0x2a, 0x1c, 0x33, 0xe9, 0xb0, 0x73, 0xb1}
    ///
    uint8_t preamble[16];

    ///
    /// The size of the data structure to follow in bytes.
    ///
    uint32_t data_count;
}p3dss_DxHeader_t;	


typedef struct{
    ///
    /// The data id (ping number).
    ///
    uint64_t id;

    ///
    /// Time at which the trigger occured.
    ///
    p3dss_Time_t time;

    ///
    /// The zero range time. (ie the time at which the sonar range=0)
    ///
    /// \note This timestamp, along with the bulk sound velocity and range value,
    ///       allows the exact time to be computed from the range values for any
    ///       sidescan or sidescan3D data point.
    ///
    p3dss_Time_t time_range_zero;

    ///
    /// The sonar settings in use at the time the data was acquired.
    ///
    p3dss_DxParameters_t parameters;

    ///
    /// The sonar system info.
    ///
    p3dss_DxSystemInfo_t system_info;
  

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the Ascii sentence data section.
    ///
    uint32_t ascii_sentence_offset;
    
    ///
    /// The number of Ascii sentence present.
    ///
    uint32_t ascii_sentence_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the port sidescan data section.
    ///
    uint32_t port_sidescan_offset;

    ///
    /// The number of port sidescan points present.
    ///
    uint32_t port_sidescan_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the starboard sidescan data section.
    ///
    uint32_t starboard_sidescan_offset;

    ///
    /// The number of starboard sidescan points present.
    ///
    uint32_t starboard_sidescan_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the port sidescan-3d data section.
    ///
    uint32_t port_sidescan3d_offset;

    ///
    /// The number of port sidescan-3d points present.
    ///
    uint32_t port_sidescan3d_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the starboard sidescan-3d data section.
    ///
    uint32_t starboard_sidescan3d_offset;

    ///
    /// The number of starboard-3d sidescan points present.
    ///
    uint32_t starboard_sidescan3d_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the port bathymetry data section.
	  /// \since 0.5
    ///
    uint32_t port_bathymetry_offset;

    ///
    /// The number of port bathymetry points present.
	  /// \since 0.5
    ///
    uint32_t port_bathymetry_count;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the starboard bathymetry data section.
	  /// \since 0.5
    ///
    uint32_t starboard_bathymetry_offset;

    ///
    /// The number of bathymetry sidescan points present.
	  /// \since 0.5
    ///
    uint32_t starboard_bathymetry_count;	

    ///
    /// The offset in bytes from the start of the DxData structure to the start
    /// of the recorded filename which is a null terminated string.
    /// \since 0.6
    ///
    uint32_t recorded_filename_offset;

    ///
    /// The offset in bytes from the start of the DxData structure to the start
    /// of the recorded vesrion which is a null terminated string.
    /// \since 0.6
    ///
    uint32_t recorded_version_offset;

    ///
    /// The offset in bytes from the start of the DxData structure to the start 
    /// of the SBG message data section.
    ///
    uint32_t sbg_message_offset;
    
    ///
    /// The number of SBG messages present.
    ///
    uint32_t sbg_message_count;

    ///
    /// Added a few reserved bytes to give us the capability to add additional data without
    /// breaking existing code.  Allows for up to 8 additional data spaces.
    /// \since 0.3
    ///
    uint32_t reserved_space[14];
  
    ///
    /// Zero length array (can be omitted).  The first variable length data
    /// section starts here (typically the AsciiSentences, 
    /// ie ascii_sentence_offset should point to this)
    ///
    //uint8_t variable_length_data[];
}p3dss_DxData_t;	

#pragma pack()

void p3dss_init(void);
void p3dss_set_sensor_offset(offset_t* s);
uint8_t p3dss_test_nav_file(int fd);
uint8_t p3dss_test_bathy_file(int fd);
int p3dss_seek_next_header(int fd);
int p3dss_fetch_next_packet(char * data, int fd);
int p3dss_identify_sensor_packet(char* databuffer, uint32_t len, double* ts_out);
int p3dss_process_nav_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);
uint32_t p3dss_georef_data( char* databuffer,uint32_t databuffer_len, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, sensor_params_t* sensor_params, /*OUTPUT*/ output_data_t* outbuf);

#endif
