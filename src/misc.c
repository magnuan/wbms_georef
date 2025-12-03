/** @cond */
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
/** @endcond */


void skip(int fd, size_t nbytes) {
    char buf[4096];
    while (nbytes > 0) {
        size_t chunk = nbytes < sizeof(buf) ? nbytes : sizeof(buf);
        ssize_t r = read(fd, buf, chunk);
        if (r <= 0) break;   // error or EOF
        nbytes -= r;
    }
}



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

int is_broadcast_address(uint32_t addr_net_order) {
    uint32_t addr_host_order = ntohl(addr_net_order);
    return (addr_host_order & 0xFF) == 0xFF;
}
