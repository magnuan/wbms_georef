#ifndef _LOKI_PACKET_H_
#define _LOKI_PACKET_H_
/*******************************************************************************
 * (c) Copyright 2012-2019 Norbit Subsea. All rights reserved.                 
 *******************************************************************************/
/* This file has been prepared for Doxygen automatic documentation generation. */
/****************************************************************************//**
* @file loki_packet.h
*
* All data structs are set up ensuring all datatypes are aligned according to their size.
* Datatypes are defined with Little Endianness 
*
* @brief Header file defining WBMS sonar native data format
* @copyright 2012-2019 Norbit Subsea. All rights reserved.
* @author Magnus Andersen (Magnus.Andersen@norbit.no)
*
*/

#include "bathy_packet.h"
#include "wbms_georef.h"

//@{
/** WBMS packet type  
*   Value in type field of main packet header, specifying format of following packet. */
#define PACKET_TYPE_SBP_DATA                1000 
//@}


//@{
/** WBMS packet version for given packet type 
*   Value in version field of main packet header, specifying format of following packet. */
#define SBP_DATA_VERSION                1
//@}


/* The following limitations is not inherent packet format limitations, but just for specifying assignment of memory */
/** Max size in bytes of a watercolumn packet payload section  (128x1024 32-bit data) 
* wbms georef treats each sample in the SBP data as a sounding, so we can not have more samples than the max allowed number of detection points per ping
*/
#define SBP_MAX_SAMPLES (MAX_DP)
#define SBP_DATA_MAX_PAYLOAD_SIZE    (4*SBP_MAX_SAMPLES)

#define DEFAULT_SBP_SV 1470.

#pragma pack(4)


/** WATERCOL_DATA PACKET SUB-HEADER (WBMS Type 2 packet) */
typedef struct{
    float       sample_rate;            /**< Sample rate in reported range sample index, in Hz*/
    uint32_t    N;                      /**< Payload dimension (low stride)*/
    uint32_t    M;                      /**< Payload dimension (high stride)*/
    uint32_t    ping_number;            /**< Ping number, increasing ping counter since sonar startup*/
    double      time;                   /**< Timestamp Unix time as fract*/
    float       gain;                   /**< Signal gain through processing chain*/
    float       tx_voltage;             /**< Tx voltage - peak-voltage signal over ceramics NaN for sonars without measurement */
    uint32_t    Dtype;                  /**< Datatype*/
    uint32_t    tp;                     /**< Testpoint*/
    float       tx_prim_freq;           /**< Tx frequency carrier [Hz]*/
    float       tx_prim_bw;             /**< Tx bandwidth carrier [Hz]*/
    float       tx_len;                 /**< Tx length [s]*/
    float       tx_sec_freq;            /**< Tx frequency modulation [Hz]*/
    float       tx_sec_bw;              /**< Tx bandwidth modulation [Hz]*/
    int32_t     tx_pulse_type;          /**< */
    uint32_t    tx_amp;                 /**< Tx amplitude*/
    float       ping_rate;              /**< Set ping rate in Hz*/

    uint32_t    RESERVEDX[8];           /**< reserved for future use*/
}sbp_data_header_t; //104 Bytes

/** WATERCOLUMN PACKET, including common header, sub-header and payload */
typedef struct{
    packet_header_t                 header;                             /**< Common packet header */
    sbp_data_header_t               sub_header;                         /**< Packet type sub-header*/
    uint8_t                         payload[SBP_DATA_MAX_PAYLOAD_SIZE]; /**< Sub bottom profiler data payload section. 1D data */
}sbp_data_packet_t;

#define SIZEOF_SBP_PACKET_HEADER (sizeof(packet_header_t) + sizeof(sbp_data_header_t))

#pragma pack()
#endif
