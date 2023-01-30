
int eelume_sbd_nav_seek_next_header(int fd);
int eelume_sbd_nav_fetch_next_packet(char * data, int fd);
int eelume_sbd_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out);
uint8_t eelume_sbd_nav_test_file(int fd);
int eelume_sbd_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata);

#define MAX_EELUME_SBD_NAV_PACKET_SIZE 100
