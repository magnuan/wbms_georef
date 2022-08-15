#ifndef __JSON_OUTPUT_H__
#define __JSON_OUTPUT_H__

#include "wbms_georef.h"

int write_json_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, aux_navdata_t *aux_navdata,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf);

#endif
