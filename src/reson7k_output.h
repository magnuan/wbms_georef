#ifndef __RESON7K_OUTPUT_H__
#define __RESON7K_OUTPUT_H__

#include "wbms_georef.h"


/*
* GUI s7k file export contains
* Init: 7030, 7200,
* Then: 7000, 7004, 7027, 7610
*
*
*
* GUI Server exports
*
* 7kRTT 
* Requests: 7022, 7004, 7000, 7002, 7027
*
*
* Init send: 7502, 7501, 7001, 7502, 7501, 7503, 7501
* Cont send: 7000, 7004, 7027
*  
* 
* QPS:
* Request: 7000, 7006, 7027, 7028, 7042,
* 
* Init Send: 7000, 7001, 7006, 7027, 7028, 7042, 7501, 7501
* Cont Send: 7000, 7006, 7027, 7028, 7042
*
*
* 7000 Sonar settings
* 7004 Beam geometry
* 7006 Bathymetric data
* 7027 Raw detection data
* 7028 Snippet data
* 7030 Sonar installation parameters
* 7042 Compressed water column data
* 7200 File header
*
*
* TODO: 
*  - Add at least 7200 to file export  (header)
*  - Add 7000 and perhaps 7004 to all export (every ping)
*  - 7006 Should be unneccessary
*/

void set_r7k_output_parameters(uint8_t recorded_not_live, uint8_t write_network_frame, uint32_t max_packet_size);


/* 
*  Stream:
*  7001
*  7502
*
*
*  File:
*  r7k_Record_7030_t            Sonar installation parameters      
*  r7k_RecordData_7200_t        File Header
*/
int write_r7k_header_to_buffer(/*INPUT TBD*/double ts, /*OUTPUT*/char* outbuf);


/* r7k_Record_7000_t            Sonar settings
*  r7k_Record_7027_t            Raw detection Data
*  r7k_RecordTypeHeader_7610_t  Sound velocity
*/
int write_r7k_bathy_to_buffer(double ts, output_data_t* data, uint32_t N, /*OUTPUT*/char* outbuf);

/* 
*  r7k_RecordTypeHeader_1015_t  Navigation
*  r7k_RecordData_1016_entry_t  Attitude
*  or
*  r7k_RecordTypeHeader_1003_t  Position 
*  r7k_RecordTypeHeader_1012_t  Roll Pitch Heave
*  r7k_RecordTypeHeader_1013_t  Heading
*/
int write_r7k_nav_to_buffer(double ts, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, aux_navdata_t *aux_navdata, /*OUTPUT*/char* outbuf);

#endif
