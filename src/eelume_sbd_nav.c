#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
#ifdef __unix__
#include <unistd.h>
#endif
#include <time.h>

#include <string.h>
#include <stdint.h>
#include "time_functions.h"

#include <math.h>
#include "wbms_georef.h"
#include "nmea_nav.h"
#include "eelume_sbd_nav.h"
#include "georef_tools.h"
#include "cmath.h"
#if defined(_MSC_VER)
#include "non_posix.h"
#include <io.h>
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif
//Eelume SBD data has misc data as entries wrapped with this header

/*
SDB file containing WBMS data and NMEA type nav

entry_type      dont_care[3]    relative time       tv_sec          tv_usec             entry_size (payload+garble)     garble                                              entry_size

09              00 00 00        bc 23 00 00         c4 80 45 62     88 90 00 00         70 28 00 00                     09 00 00 00 bd 23 00 00 c4 80 45 62 70 94 00 00     70 28 00 00
=== Here comes the bathy packet 10352 Bytes===


08              00 00 00        a0 26 00 00         c4 80 45 62     28 db 0b 00         55 00 00 00                     00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00     41 00 00 00
$EIPOS,064,102156.777,1648722116777,63.461570,N,10.546177,E*73 \x0d \x0a \x00

04              00 00 00        b4 $ĀE              bHYK            �5hC                �(�@;
$EIDEP,058,102156.285,1648722116285,232.21,m,005.88,m*6C \x0d \x0a \x00

02              00 00 00        a0 26 00 00         c4 80 45 62     28 db 0b 00         44 00 00 00                      f6 28 98 43 00 00 00 00 00 00 00 00 3b 00 00 00     30 00 00 00
$EIHEA,047,102156.777,1648722116777,304.32*4E   \x0d \x0a \x00

03              00 00 00        a0 26 00 00         c4 80 45 62     28 db 0b 00         48 00 00 00                      52 b8 94 41 b8 1e a5 bf 00 00 00 00                 38 00 00 00
$EIORI,055,102156.777,1648722116777,018.59,-001.29*43 \x0d \x0a \x00

08              00 00 00        68 27 00 00         c4 80 45 62     68 e8 0e 00         55 00 00 00                      00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00     41 00 00 00
$EIPOS,064,102156.977,1648722116977,63.461570,N,10.546176,E*72  \x0d \x0a \x00

04              00 00 00        a8 26 00 00         c4 80 45 62     68 fa 0b 00         4b 00 00 00                      c3 35 68 43 00 00 00 00 3d 0a bf 40                 3b 00 00 00
$EIDEP,058,102156.785,1648722116785,232.21,m,005.97,m*62 \x0d \x0a \x00

02              00 00 00        68 27 00 00 c4      80 45 62 68     e8 0e 00 44         00 00 00 a4                      30 98 43 00 00 00 00 00 00 00 00 3b 00 00 00        30 00 00 00
$EIHEA,047,102156.977,1648722116977,304.38*44   \x0d \x0a \x00
*/


typedef struct SbdEntryHeader
{
    #if 0
    enum : char
    {
        NMEA_EIHEA = 2,     /*Heading */
        NMEA_EIORI = 3,     /*Orientation (Roll, Pitch)*/
        NMEA_EIDEP = 4,     /*Depth*/
        NMEA_EIPOS = 8,     /*Position*/
        WBMS_BATH = 9,      /*Raw WBMS sonar data*/
        HEADER = 21,
    } entry_type;
    #endif
    char entry_type;

    char dont_care[3];
    int32_t relative_time;

    struct
    {
        int32_t tv_sec;
        int32_t tv_usec;
    } absolute_time;

    uint32_t entry_size;
} sbdEntryHeader_t;







#define EELUME_BUFFER_SIZE (MAX_EELUME_SBD_NAV_PACKET_SIZE+4)
static int buffered_read(int fd, void* data, int len){
    static uint8_t buffer[EELUME_BUFFER_SIZE];
    static uint8_t* buf_ptr;
    static int available = 0;
    while (1){
        if (len>0){
            if (len<=available){
                memcpy(data,buf_ptr,len);
                available -= len;
                buf_ptr += len;
                return len;
            }
            else if(available>0){
                len = available;
                memcpy(data,buf_ptr,len);
                available = 0;
                return len;
            }
            else{
                buf_ptr = buffer;
                available = read(fd,buffer,EELUME_BUFFER_SIZE);
                if (available==0){
                    return 0;
                }
            }
        }
    }
}


int eelume_sbd_nav_seek_next_header(int fd){    //Search for a legal entry type, followed by three don't care bytes set to 0 
	char state = 0;
	uint8_t v;
    int type = 0;
	int n;
	int dump= 0;
    int read_bytes = 0;
	while (read_bytes<(MAX_EELUME_SBD_NAV_PACKET_SIZE+4)){
        n = buffered_read(fd,&v,1);
        read_bytes++;
        if(n<0){ fprintf(stderr,"Got error from socket\n");return -1;}
        if(n==0){ /*fprintf(stderr,"End of POS_MODE_EELUME stream\n");*/return -1;}
        if(n>0){
            dump += 1;
            switch (state){
                case 0: state = (v==2 ||v==3||v==4||v==8||v==9||v==21)?1:0;
                    type=v;
                    break;
                case 1: state = (v==0)?2:0;break;
                case 2: state = (v==0)?3:0;break;
                case 3: state = (v==0)?4:0;break;
            }
            if (state==4){
                dump-=4;
                //if(dump>2) fprintf(stderr,"Eelume nav seek dump %d bytes\n",dump);
                return type;	
            }
        }
    }
	return -1;
}
    
int eelume_sbd_nav_fetch_next_packet(char * data, int fd){
	int rem,n;
	char * dp;
    sbdEntryHeader_t* header = (sbdEntryHeader_t*) data;
    int type = eelume_sbd_nav_seek_next_header(fd);
    if(type<0) return 0;
    header->entry_type = type;
    dp = data+4;
    n = buffered_read(fd,dp,sizeof(sbdEntryHeader_t)-4);
    if (header->entry_size>MAX_EELUME_SBD_NAV_PACKET_SIZE) return 0;
    /*for (int ix = 0; ix<32;ix++){
        fprintf(stderr, "%02x ", data[ix]);
    }
    fprintf(stderr, "\n sizeof(sbdEntryHeader_t) = %d",sizeof(sbdEntryHeader_t));*/

    double ts = header->absolute_time.tv_usec;
    ts = header->absolute_time.tv_sec + ts*1e-6;
    //fprintf(stderr, "fetch EElume packet type=%d, Rel time = %d, Time =%f, Size=%d\n",header->entry_type, header->relative_time, ts,header->entry_size); 
    dp += sizeof(sbdEntryHeader_t);
    rem = header->entry_size; //Fetch packet header (minus the preamble we allready have)
    while (rem>0){ 
        n= buffered_read(fd,dp,rem);rem -= n; dp+=n;
        if(n<0){ fprintf(stderr,"Got error from socket\n");return 0;}
        if(n==0){ /*fprintf(stderr,"End of POS_MODE_EELUME stream\n");*/return 0;}
    }
    return header->entry_size+sizeof(sbdEntryHeader_t);
}

int eelume_sbd_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
    sbdEntryHeader_t* header = (sbdEntryHeader_t*) databuffer;
    double ts = header->absolute_time.tv_usec;
    ts = header->absolute_time.tv_sec + ts*1e-6;
    //fprintf(stderr, "identify EElume packet type=%d, Rel time = %d, Time =%f, Size=%d\n",header->entry_type, header->relative_time, ts,header->entry_size); 
    *ts_out = ts;
    return header->entry_type; 
}

uint8_t eelume_sbd_nav_test_file(int fd){
    uint8_t pass=0;
    char* data = malloc(MAX_EELUME_SBD_NAV_PACKET_SIZE);
    if (data==NULL){
        return 0;
    }
    for(int test=0;test<20;test++){     //Test the first 20 packets, if none of them contains requested data it is pobably not a valid data file
        int len; 
        len = eelume_sbd_nav_fetch_next_packet(data, fd);
        //printf("len=%d\n",len);
        if (len > 0 ){
            double ts;
            int type = eelume_sbd_nav_identify_packet(data, len, &ts);
            int req_types[] = {2,3,4,8,9,21};    
            size_t n_req_types = 6;
            for (size_t req_type_ix = 0; req_type_ix<n_req_types;req_type_ix++){
                if (type==req_types[req_type_ix]){
                    pass=1;
                    break;
                }
            }
            if (pass) break;
        }
        else{
            break;
        }
    }
    free(data);
    return pass;
}

int eelume_sbd_nav_process_packet(char* databuffer, uint32_t len, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
    sbdEntryHeader_t* header = (sbdEntryHeader_t*) databuffer;
    if (header->entry_type==2 || header->entry_type==3 ||header->entry_type==4 ||header->entry_type==8){
        char* dp = databuffer + sizeof(sbdEntryHeader_t);
        len -= sizeof(sbdEntryHeader_t);
        while ( (len>0) && (*dp!='$') ){
            dp++;
            len--;
        }
        return nmea_nav_process_nav_packet(dp,len,0,ts_out,z_offset, alt_mode, proj, navdata,aux_navdata);
    }
    return NO_NAV_DATA;
}
