#ifndef __BIN_OUTPUT_H__
#define __BIN_OUTPUT_H__

#include "wbms_georef.h"
int write_bin_to_buffer(output_data_t* data, uint32_t n, /*OUTPUT*/char* outbuf);
int write_bin_proj_string_to_buffer(char* str, /*OUTPUT*/char* outbuf);

#endif
