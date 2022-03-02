#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "georef_tools.h"
#include "wbms_georef.h"
#include <math.h>


int write_bin_proj_string_to_buffer(char* str, /*OUTPUT*/char* outbuf){
    char * dp;
    dp = outbuf;

    *((uint64_t*)dp) = PROJ_STR_MAGIC_NUMBER;
    dp+=8;       //8Byte Preamble 
    dp += sprintf(dp,"%s",str);
    *((uint8_t*)dp) = 0;
    dp+=1;       //Null termination 
    while((dp-outbuf)%32){
        *((uint8_t*)dp) = 0;
        dp+=1;       //Pad to multiple of 32 
    }
    return dp-outbuf;
}

int write_bin_to_buffer(output_data_t* data, uint32_t n, /*OUTPUT*/char* outbuf){
    double* x = &(data->x[0]);
    double* y = &(data->y[0]);
    double* z = &(data->z[0]);
    float* val = &(data->i[0]);

    char * dp;
	dp = outbuf;
	uint32_t ii,len;
	len = 0;
	
    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up
	for (ii = 0;ii<n;ii++){
		*((uint16_t*)dp) = 0xBEEF; dp+=2;		//2Byte Preamble	
		*((uint16_t*)dp) = 1; dp+=2;		    //2Byte Type (This is no longer in use, so we just set it to 1 for legacy)
		*((float*)dp) = val[ii]; dp+=4;			//4Byte Val
		*((double*)dp) = y[ii]; dp+=8;			//8Byte X
		*((double*)dp) = x[ii]; dp+=8;			//8Byte Y
		*((double*)dp) = -z[ii]; dp+=8;			//8Byte Z
		len += 2+2+4+8+8+8;
	}
	//fprintf(stderr,"Writing %d points %d bytes\n",n,len);
	return len;
}
