#ifndef __RESON7K_OUTPUT_H__
#define __RESON7K_OUTPUT_H__

#include "wbms_georef.h"

/* r7k_Record_7000_t            Sonar settings
*  r7k_RecordData_7001_t        Configuration
*/
int write_r7k_header_to_buffer(/*INPUT TBD*/ /*OUTPUT*/char* outbuf);


/* r7k_Record_7027_t            Raw detection Data
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
