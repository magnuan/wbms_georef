#ifndef _MISC_H_
#define _MISC_H_

int atobool(char* str);
int is_broadcast_address(uint32_t addr_net_order);
void skip(int fd, size_t nbytes);

#endif
