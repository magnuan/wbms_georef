#ifndef __TIME_FUNCTIONS_H__
#define __TIME_FUNCIONS_H__

void gm_to_irigb(double ts, uint16_t* year, uint16_t* doy, uint8_t* hour, uint8_t* min, float* sec);
double irigb_to_gm(uint16_t year,uint16_t doy,uint8_t HH,uint8_t MM,float SS);

uint16_t date_to_doy(uint16_t year,uint16_t month, uint16_t date);
int sprintf_unix_time(char * str, double ts);
double os_time(void);
double parse_timestamp_from_filename(const char *filename);
#endif
