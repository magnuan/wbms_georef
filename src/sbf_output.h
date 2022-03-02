#ifndef __SBF_OUTPUT_H__
#define __SBF_OUTPUT_H__

#include "wbms_georef.h"

int write_sbf_meta_to_buffer(output_format_e format[MAX_OUTPUT_FIELDS], uint64_t point_count, double x_offset, double y_offset, double z_offset,  /*OUTPUT*/char* outbuf);
int write_sbf_header_to_buffer(output_format_e format[MAX_OUTPUT_FIELDS], uint64_t point_count, double x_offset, double y_offset, double z_offset,  /*OUTPUT*/char* outbuf);
int write_sbf_to_buffer(double x_offset, double y_offset, double z_offset,double ts_offset, double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf);

#endif
