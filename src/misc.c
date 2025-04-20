/** @cond */
#include <stdlib.h>
//#include <unistd.h>
#include <string.h>
/** @endcond */




int atobool(char* str){
    if (strstr(str,"True")) return 1;
    if (strstr(str,"true")) return 1;
    if (strstr(str,"TRUE")) return 1;
    if (strstr(str,"yes")) return 1;
    if (strstr(str,"Yes")) return 1;
    if (strstr(str,"YES")) return 1;
    if (strstr(str,"1")) return 1;
    return 0;
}
