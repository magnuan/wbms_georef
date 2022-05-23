//EIVA SDB data has misc data as entries wrapped with this header

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


struct SbdEntryHeader
{
    enum : char
    {
        NMEA_EIHEA = 2,
        NMEA_EIORI = 3,
        NMEA_EIDEP = 4,
        NMEA_EIPOS = 8,
        WBMS_BATH = 9,
        HEADER = 21,
    } entry_type;

    char dont_care[3];
    int32_t relative_time;

    struct
    {
        int32_t tv_sec;
        int32_t tv_usec;
    } absolute_time;

    uint32_t entry_size;
};
static_assert(sizeof(SbdEntryHeader) == 20);
