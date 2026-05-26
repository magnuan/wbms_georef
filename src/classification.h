#ifndef __CLASSIFICATION_H__
#define __CLASSIFICATION_H__

#include <stdint.h>

int classification_test_file(char* fname, uint32_t* pings,uint32_t* beams,uint32_t* mp,uint32_t* rows);
int classification_read_from_file(char* fname);
int16_t classification_get_class(int32_t ping, uint32_t beam, uint32_t mp);


#endif
