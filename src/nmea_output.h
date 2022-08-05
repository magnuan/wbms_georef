#ifndef __NMEA_OUTPUT_H__
#define __NMEA_OUTPUT_H__

#include "wbms_georef.h"

int write_nmea_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, /*OUTPUT*/char* outbuf);

#endif
