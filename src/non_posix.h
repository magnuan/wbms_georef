#ifndef __NON_POSIX_H__
#define __NON_POSIX_H__
#ifndef __unix__

#include <stdio.h>
#include <stdlib.h>

size_t getline(char **lineptr, size_t *n, FILE *stream);

#define  read(a,b,c) _read(a,b,c)
#endif
#endif
