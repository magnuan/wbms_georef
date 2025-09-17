#ifndef _BATHY_PACKET_H_
#define _BATHY_PACKET_H_
/*******************************************************************************
 * (c) Copyright 2012-2019 Norbit Subsea. All rights reserved.                 
 *******************************************************************************/
/* This file has been prepared for Doxygen automatic documentation generation. */
/****************************************************************************//**
* @file bathy_packet.h
*
* All data structs are set up ensuring all datatypes are aligned according to their size.
* Datatypes are defined with Little Endianness 
*
* @brief Header file defining WBMS sonar native data format
* @copyright 2012-2019 Norbit Subsea. All rights reserved.
* @author Magnus Andersen (Magnus.Andersen@norbit.no)
*
*/

/** Packet preamble, all WBMS packets starts with this 32-bit magic number */
#define PACKET_PREAMBLE 0xDEADBEEF

//@{
/** WBMS packet type  
*   Value in type field of main packet header, specifying format of following packet. */
#define PACKET_TYPE_BATH_DATA                1 
#define PACKET_TYPE_WATERCOL_DATA            2 
#define PACKET_TYPE_MISC_XML                 3 
#define PACKET_TYPE_SNIPPET_DATA             4 
#define PACKET_TYPE_SIDESCAN_DATA            5 
#define PACKET_TYPE_GEOREF_SIDESCAN_DATA     6 
#define PACKET_TYPE_HEARTBEAT                7 
//@}


//@{
/** WBMS packet version for given packet type 
*   Value in version field of main packet header, specifying format of following packet. */
#define BATH_DATA_VERSION               4
#define WATERCOL_DATA_VERSION           4
#define SNIPPET_DATA_VERSION            4
#define SIDESCAN_DATA_VERSION           4
#define HEARTBEAT_VERSION               1
#define GEOREF_SIDESCAN_DATA_VERSION    3
//@}


/* The following limitations is not inherent packet format limitations, but just for specifying assignment of memory */
/** Max number of bathy detections in a single WBMS type 1 packet*/
#define BATH_DATA_MAX_DETECTION_POINTS    4096
/** Max size in bytes of a watercolumn packet payload section  (1024x1024 16-bit 2D data + 1024 32-bit float angles) */
#define WATERCOL_MAX_PAYLOAD_SIZE    ((2*1024*1024)+(4*1024))

/** Max size in bytes of a snippet packet payload section  (1024 32-bit start index, + 1024 32-bit stop index, 1024 32-bit detection index, 1024 32-bit angle, 1024 32-bit reserved, 512*1024 16-bit values) */
#define MAX_SNIPPET_LENGTH (512)
#define SNIPPET_MAX_PAYLOAD_SIZE    (5*4*1024 + 2*1024*MAX_SNIPPET_LENGTH  )



//@{
/** WBMS bathy data sub-type 
*   For the type field in the bathy data sub-header. */
#define BATH_DATA_TYPE_AUTO_GATE_BATHY                   0x00
#define BATH_DATA_TYPE_FULL_GATE_BATHY                   0x01
#define BATH_DATA_TYPE_PRE_GATE_BATHY                    0x02
#define BATH_DATA_TYPE_MULTI_DETECT1                     0x03
#define BATH_DATA_TYPE_GEOREF_SIDESCAN                   0x04
#define BATH_DATA_TYPE_POST_GATE_BATHY                   0x05
#define BATH_DATA_TYPE_AUTO_GATE_BATHY_MULTIFREQ         0x06

#define BATH_DATA_TYPE_AUTO_GATE_UPPER      0x10
#define BATH_DATA_TYPE_AUTO_GATE_LOWER      0x12
#define BATH_DATA_TYPE_AUTO_GATE_ALT        0x14
#define BATH_DATA_TYPE_FULL_GATE_UPPER      0x11
#define BATH_DATA_TYPE_FULL_GATE_LOWER      0x13
#define BATH_DATA_TYPE_FULL_GATE_ALT        0x15

#define BATH_DATA_TYPE_SEAFLOOR_MODEL       0x21
#define BATH_DATA_TYPE_NOISE_REF            0x22
#define BATH_DATA_TYPE_INTENSITY            0x23

#define BATH_DATA_TYPE_DEBUG                0xFF
//@}



//@{
/** Enumerated data type values
*   For the Dtype field in Watercolum, Sidscan and Snippet sub-headers */ 
#define DTYPE_UINT8 0x00 
#define DTYPE_INT8 0x01
#define DTYPE_UINT16 0x02 
#define DTYPE_INT16 0x03
#define DTYPE_UINT32 0x04 
#define DTYPE_INT32 0x05
#define DTYPE_UINT64 0x06 
#define DTYPE_INT64 0x07
#define DTYPE_FLOAT32  0x15
#define DTYPE_FLOAT64  0x17

#define DTYPE_CUINT8 0x20 
#define DTYPE_CINT8 0x21
#define DTYPE_CUINT16 0x22 
#define DTYPE_CINT16 0x23
#define DTYPE_CUINT32 0x24 
#define DTYPE_CINT32 0x25
#define DTYPE_CUINT64 0x26 
#define DTYPE_CINT64 0x27
#define DTYPE_CFLOAT32  0x35
#define DTYPE_CFLOAT64  0x37
//@}


#pragma pack(4)

/** Struct for a single detection point in a type 1 (bathy) packet*/
typedef struct{
    uint32_t sample_number;         /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float    angle;                 /**< Angle. Detection point direction in radians */
    uint16_t upper_gate;            /**< Upper adaptive gate range. Same unit a sample_number*/
    uint16_t lower_gate;            /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
}detectionpoint_t;

#define SAMPLE_NUMBER_V104_UPSCALE 256
typedef struct{
    uint32_t upscaled_sample_number;         /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float    angle;                 /**< Angle. Detection point direction in radians */
    uint32_t upscaled_upper_gate;            /**< Upper adaptive gate range. Same unit a sample_number*/
    uint32_t upscaled_lower_gate;            /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
}detectionpoint_v104_t;

typedef struct{
    float sample_number;            /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float angle;                    /**< Angle. Detection point direction in radians */
    float steer_angle;              /**< Steer Angle. Part of angle that is done by pahse steering (not by shifting aperture) */
    float upper_gate;               /**< Upper adaptive gate range. Same unit a sample_number*/
    float lower_gate;               /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    float strength;                 /**< Strength. Bottom detection strength after combined amp/phase */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
    uint8_t reserved[16];
}detectionpoint_v5_t;


typedef struct{
    uint32_t sample_number;         /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float    angle;                 /**< Angle. Detection point direction in radians */
    uint16_t upper_gate;            /**< Upper adaptive gate range. Same unit a sample_number*/
    uint16_t lower_gate;            /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
}detectionpoint_v7_t;

typedef struct{
    float sample_number;            /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float angle;                    /**< Angle. Detection point direction in radians */
    float steer_angle;              /**< Steer Angle. Part of angle that is done by pahse steering (not by shifting aperture) */
    float upper_gate;               /**< Upper adaptive gate range. Same unit a sample_number*/
    float lower_gate;               /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    float strength;                 /**< Strength. Bottom detection strength after combined amp/phase */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
    uint8_t reserved[16];
}detectionpoint_v8_t;

typedef struct{
    float sample_number;            /**< Range. Detection point sample index (Distance from acc-centre is (ix*sound_velocity)/(2*Fs) */
    float angle;                    /**< Angle. Detection point direction in radians */
    float steer_angle;              /**< Steer Angle. Part of angle that is done by pahse steering (not by shifting aperture) */
    float upper_gate;               /**< Upper adaptive gate range. Same unit a sample_number*/
    float lower_gate;               /**< Lower adaptive gate range. Same unit a sample_number*/
    float intensity;                /**< Internsity. Backscatter return signal. Scaled to compensate for VGA and processing gain */
    float strength;                 /**< Strength. Bottom detection strength after combined amp/phase */
    uint16_t flags;                 /**< Bit0: Mag based detection | Bit1: Phase based detection | Bit2-8: Quality type | Bit9-12: Detection priority */
    uint8_t quality_flags;          /**< Bit0: SNr test pass | Bit1: Colinarity test pass */
    uint8_t quality_val;            /**< Ad-Hoc detection signal quality metric, based on detection strength and variance ovet time and swath */ 
    uint16_t beam_number;
    uint8_t reserved[14];
}detectionpoint_vX_t;


/** COMMON HEADER, common packet header struct for all WBMS data packets*/
typedef struct{
    uint32_t preamble;              /**< Preable / Magic number, to simplify detection of a valid packet in a stream or blob of data. Allways 0xDEADBEEF */
    uint32_t type;                  /**< WBMS packet type */ 
    uint32_t size;                  /**< Total packet size (including headers) in bytes */
    uint32_t version;               /**< WBMS packet version     */
    uint32_t RESERVED;              /**< 4 bytes reserved for furure use */
    uint32_t crc;                   /**< CRC32 of the size-24 bytes following the header  (crc32, zlib-style SEED=0x00000000, POLY=0xEDB88320)*/
}packet_header_t; //24 Bytes


/** BATH_DATA PACKET SUB-HEADER (WBMS Type 1 packet) */
typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Number of bathy points in packet*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract (tx time)*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    uint16_t    type;                   /**< WBMS Bathy data sub-type*/
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #4-6: Multifreq index
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/
    uint16_t    multiping;              /**< Number of pings in multiping sequence*/
    uint16_t    multiping_seq_nr;       /**< Number of this ping in multiping sequence*/
    float       tx_angle;               /**< Tx beam steering in radians*/
    float       gain;                   /**< Intensity value gain*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       gate_tilt;              /**< Gate tilt, in radians */
}bath_data_header_t; //88 bytes

typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Number of bathy points in packet*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract (tx time)*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    uint16_t    type;                   /**< WBMS Bathy data sub-type*/
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #4-6: Multifreq index
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/
    uint16_t    multiping;              /**< Number of pings in multiping sequence*/
    uint16_t    multiping_seq_nr;       /**< Number of this ping in multiping sequence*/
    float       tx_angle;               /**< Tx beam steering in radians*/
    float       gain;                   /**< Intensity value gain*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       gate_tilt;              /**< Gate tilt, in radians */
    float       intensity_noise_ref;
    float       strength_noise_ref;
    uint8_t     res[16];
}bath_data_header_v5_t; //112 bytes 

typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Number of bathy points in packet*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract (tx time)*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    uint16_t    type;                   /**< WBMS Bathy data sub-type*/
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/


    uint8_t    multiping_scan_number;               /**< Number of pings in a stx multiping sequence*/
    uint8_t    multifreq_band_number;               /**< Number of bands in multifreq processing*/
    uint8_t    multiping_scan_index;                /**< Number of this ping in a stx multiping sequence*/
    uint8_t    multifreq_band_index;                /**< Number of this band in multifreq processsing*/



    float       tx_angle;               /**< Tx beam steering in radians*/
    float       gain;                   /**< Intensity value gain*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       gate_tilt;              /**< Gate tilt, in radians */
}bath_data_header_v7_t;

typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Number of bathy points in packet*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract (tx time)*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    uint16_t    type;                   /**< WBMS Bathy data sub-type*/
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #4-6: Multifreq index
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/
    
    uint8_t    multiping_scan_number;               /**< Number of pings in a stx multiping sequence*/
    uint8_t    multifreq_band_number;               /**< Number of bands in multifreq processing*/
    uint8_t    multiping_scan_index;                /**< Number of this ping in a stx multiping sequence*/
    uint8_t    multifreq_band_index;                /**< Number of this band in multifreq processsing*/
    
    float       tx_angle;               /**< Tx beam steering in radians*/
    float       gain;                   /**< Intensity value gain*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       gate_tilt;              /**< Gate tilt, in radians */
    float       intensity_noise_ref;
    float       strength_noise_ref;
    uint8_t     res[16];
}bath_data_header_v8_t; //112 bytes 

typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Number of bathy points in packet*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract (tx time)*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    uint16_t    type;                   /**< WBMS Bathy data sub-type*/
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #4-6: Multifreq index
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/
    
    uint8_t    multiping_scan_number;               /**< Number of pings in a stx multiping sequence*/
    uint8_t    multifreq_band_number;               /**< Number of bands in multifreq processing*/
    uint8_t    multiping_scan_index;                /**< Number of this ping in a stx multiping sequence*/
    uint8_t    multifreq_band_index;                /**< Number of this band in multifreq processsing*/
    
    float       tx_angle;               /**< Tx beam steering in radians*/
    float       gain;                   /**< Intensity value gain*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       gate_tilt;              /**< Gate tilt, in radians */
    float       intensity_noise_ref;
    float       strength_noise_ref;
    uint8_t     res[16];
}bath_data_header_vX_t; //112 bytes 



/** BATHY PACKET, including common header, sub-header and payload */
typedef struct{
    packet_header_t         header;                                    //24B
    bath_data_header_t      sub_header;                                //88B
    detectionpoint_t        dp[BATH_DATA_MAX_DETECTION_POINTS];        //20B*4k = 80kB
}bath_data_packet_t;

typedef struct{
    packet_header_t         header;                                    //24B
    bath_data_header_t      sub_header;                                //88B
    detectionpoint_v104_t   dp[BATH_DATA_MAX_DETECTION_POINTS];        //24B*4k = 96kB
}bath_data_packet_v104_t;

typedef struct{
    packet_header_t             header;                                    //24B
    bath_data_header_v5_t     sub_header;                                //112B
    detectionpoint_v5_t       dp[BATH_DATA_MAX_DETECTION_POINTS];        //32B*4k = 128kB
}bath_data_packet_v5_t;

typedef struct{
    packet_header_t             header;                                    //24B
    bath_data_header_v7_t     sub_header;                                //88B
    detectionpoint_v7_t       dp[BATH_DATA_MAX_DETECTION_POINTS];        //24B*4k = 96kB
}bath_data_packet_v7_t;

typedef struct{
    packet_header_t             header;                                    //24B
    bath_data_header_v8_t     sub_header;                                //112B
    detectionpoint_v8_t       dp[BATH_DATA_MAX_DETECTION_POINTS];        //32B*4k = 128kB
}bath_data_packet_v8_t;

typedef struct{
    bath_data_header_vX_t     sub_header;                                //112B
    detectionpoint_vX_t       dp[BATH_DATA_MAX_DETECTION_POINTS];        //32B*4k = 128kB
}bath_data_packet_vX_t;
    
//#define SIZEOF_BATHY_PACKET_HEADER (sizeof(packet_header_t) + sizeof(bath_data_header_t))


/** WATERCOL_DATA PACKET SUB-HEADER (WBMS Type 2 packet) */
typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Payload dimension (low stride)*/
    uint32_t    M;                      /**< Payload dimension (high stride)*/
    double      time;                   /**< Timestamp Unix time as fract*/
    uint32_t    Dtype;                  /**< Datatype*/
    int32_t     t0;                     /**< Sample number for first line (tx2rx-delay), relative to a theoritical rx point in accoustic origo*/
    float       gain;                   /**< Signal gain through processing chain*/
    uint32_t    tp;                     /**< Testpoint*/
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    uint32_t    tx_amp;                 /**< Tx amplitude*/
    uint16_t    rx_aperture;            /**< Total number of elements in rx aperture*/
    uint16_t    rx_sub_aperture_shift;  /**< Number of elements to shift for each sub-aperture*/
    float       rx_pitch;               /**< Element pitch [m]*/
    float       rx_radius;              /**< Rx array radius [m]*/
    float       ping_rate;              /**< Set ping rate in Hz*/
    float       snd_velocity_raw;       /**< Unfiltered sound velocity data from SVP probe*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    uint32_t    beams;                  /**< Number of beams in beamformer*/
    int32_t     vga_t0;                 /**< Sample number for first vga value*/
    float       vga_g0;                 /**< vga gain in dB for first vga value,  */
    int32_t     vga_t1;                 /**< Sample number for second vga value*/
    float       vga_g1;                 /**< vga gain in dB for second vga value,*/
                                        /*  For sample k  gain vga_gain(k) in dB is:                */
                                        /*  g0                                      for k < t0      */
                                        /*  g0 + ( (g1-g0)/(t1-t0) ) * (k-t0)       for t0 < k < t1 */
                                        /*  g1                                      for k > t1      */
                                        /*  Where k is sample number relative to origo, so k = m + t0, where m is line number in this data set.*/
    uint8_t    multiping_scan_number;               /**< Number of pings in a stx multiping sequence*/
    uint8_t    multifreq_band_number;               /**< Number of bands in multifreq processing*/
    uint8_t    multiping_scan_index;                /**< Number of this ping in a stx multiping sequence*/
    uint8_t    multifreq_band_index;                /**< Number of this band in multifreq processsing*/
    float       tx_angle;               /**< Tx beam steering in radians*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    uint8_t     flags;                  /**< Beam distribution mode:  As defined in sonar_config.h.  
                                                Bit #0-3: 0=Undef, 1=512EA, 2=256EA, 3=256ED, 4=512EDx 5=256EDx 
                                                Bit #7: 0=NonAdaptiveRange 1=AdaptiveRange 
                                                */
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    uint16_t    res1;
    float       gate_tilt;              /**< Gate tilt, in radians */
    float       time_uncertainty;       /**< For PPS/IRIG-B modes PPS jitter RMS, For pure NTP: NTP-offset*/
    uint32_t    RESERVEDX[7];           /**< reserved for future use*/
}backscatter_data_header_v7_t; //168 Bytes

/** WATERCOLUMN PACKET, including common header, sub-header and payload */
typedef struct{
    packet_header_t                 header;                             /**< Common packet header */
    backscatter_data_header_v7_t          sub_header;                         /**< Packet type sub-header*/
    uint8_t                         payload[WATERCOL_MAX_PAYLOAD_SIZE]; /**< Watercoloumn data payload section. 2D intensity data + 1D angle array */
     // Dtype  watercol_data[M][N], 
     // float  watercol_angles[N] 
}watercol_data_packet_v7t;

typedef struct{
    float       snd_velocity;           /**< Filtered sanitized sound velocity in m/s*/
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Payload dimension (low stride)*/
    uint32_t    M;                      /**< Payload dimension (high stride)*/
    double      time;                   /**< Timestamp Unix time as fract*/
    uint32_t    Dtype;                  /**< Datatype*/
    int32_t     t0;                     /**< Sample number for first line (tx2rx-delay), relative to a theoritical rx point in accoustic origo*/
    float       gain;                   /**< Signal gain through processing chain*/
    uint32_t    res1;                   /**< Reserved (testpoint)*/
    float       swath_dir;              /**< Center beam direction in radians */
    float       swath_open;             /**< Swath opening angle, edge to edge of swath in radians.*/
    float       tx_freq;                /**< Tx frequency [Hz]*/
    float       tx_bw;                  /**< Tx bandwidth [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    uint32_t    tx_amp;                 /**< Tx amplitude*/

    uint32_t    res2;            /**< Reserved*/
    uint32_t    res3;            /**< Reserved*/
    uint32_t    res4;            /**< Reserved*/

    float       ping_rate;              /**< Set ping rate in Hz*/
    uint32_t    res5;                   /**< Reserved*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time_net;               /**< Timestamp Unix time as fract (send on network time)*/
    uint32_t    res6;                   /**< Reserved*/
    int32_t     vga_t0;                 /**< Sample number for first vga value*/
    float       vga_g0;                 /**< vga gain in dB for first vga value,  */
    int32_t     vga_t1;                 /**< Sample number for second vga value*/
    float       vga_g1;                 /**< vga gain in dB for second vga value,*/
                                        /*  For sample k  gain vga_gain(k) in dB is:                */
                                        /*  g0                                      for k < t0      */
                                        /*  g0 + ( (g1-g0)/(t1-t0) ) * (k-t0)       for t0 < k < t1 */
                                        /*  g1                                      for k > t1      */
                                        /*  Where k is sample number relative to origo, so k = m + t0, where m is line number in this data set.*/
    //TODO check the four values here 
    uint8_t    multiping_scan_number;               /**< Number of pings in a stx multiping sequence*/
    uint8_t    multifreq_band_number;               /**< Number of bands in multifreq processing*/
    uint8_t    multiping_scan_index;                /**< Number of this ping in a stx multiping sequence*/
    uint8_t    multifreq_band_index;                /**< Number of this band in multifreq processsing*/


    float       tx_angle;                   /**< Tx beam steering in radians*/
    float       tx_voltage;                 /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    uint8_t     beam_distribution_mode;                  /**< Beam distribution mode:  As defined in sonar_config.h.*/ 
    uint8_t     sonar_mode;             /**< Sonar mode: As defined in sonar_config.h */
    uint16_t    res7;
    float       gate_tilt;              /**< Gate tilt, in radians */
    uint32_t    RESERVEDX[8];           /**< reserved for future use*/
}backscatter_data_header_v8_t; //168 Bytes

/** WATERCOLUMN PACKET, including common header, sub-header and payload */
typedef struct{
    packet_header_t                 header;                             /**< Common packet header */
    backscatter_data_header_v8_t          sub_header;                         /**< Packet type sub-header*/
    uint8_t                         payload[WATERCOL_MAX_PAYLOAD_SIZE]; /**< Watercoloumn data payload section. 2D intensity data + 1D angle array */
     // Dtype  watercol_data[M][N], 
     // float  watercol_angles[N] 
}watercol_data_packet_v8_t;


/* TODO so far we have only implemented for version 7 and 8*/
/** SNIPPET PACKET, including common header, sub-header and payload */
typedef struct{
    packet_header_t                 header;                             /**< Common packet header */
    backscatter_data_header_v7_t    sub_header;                         /**< Packet type sub-header, Snippet uses same sub header as watercoloumn*/
    uint8_t                         payload[SNIPPET_MAX_PAYLOAD_SIZE];  /**< Snippet payload section. 2D intensity data, 1D angle array, 1D start index array, 1D detection index array*/
        // Dtype        snippet_data[M][N]
        // float        watercol_angles[N]
        // uint16_t     snippet_start_sample[N]
        // uint16_t     detection_sample[N]
}snippet_data_packet_v7_t;
#define SIZEOF_WATERCOL_PACKET_V7_HEADER (sizeof(packet_header_t) + sizeof(backscatter_data_header_v7_t))

typedef struct{
    packet_header_t                 header;                             /**< Common packet header */
    backscatter_data_header_v8_t          sub_header;                         /**< Packet type sub-header*/
    uint8_t                         payload[SNIPPET_MAX_PAYLOAD_SIZE]; /**< Watercoloumn data payload section. 2D intensity data + 1D angle array */
        
        /*
            float[N]     snippet_start
            float[N]     snippet_stop
            float[N]     detection
            float[N]     angle
            uint32[N]    res
            uint16[N][v] intensity 

            v for each snippet is snippet_stop - snippet_start

        */
}snippet_data_packet_v8_t;
#define SIZEOF_WATERCOL_PACKET_V8_HEADER (sizeof(packet_header_t) + sizeof(backscatter_data_header_v8_t))


#define MAX_WBMS_PACKET_SIZE (SIZEOF_WATERCOL_PACKET_V8_HEADER+WATERCOL_MAX_PAYLOAD_SIZE)
#pragma pack()
#endif
