#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#ifdef __unix__
//#include <unistd.h>
#include <sys/time.h>
#endif
//#include <sys/ioctl.h>
#include <time.h>

#include <string.h>
#include <stdint.h>
#include <math.h>

#if defined(_MSC_VER)
#include <Windows.h>
/*typedef struct timeval {
    long tv_sec;
    long tv_usec;
} timeval;
*/




int gettimeofday(struct timeval* tp, struct timezone* tzp)
{
    // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
    // This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
    // until 00:00:00 January 1, 1970 
    static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time;

    GetSystemTime(&system_time);
    SystemTimeToFileTime(&system_time, &file_time);
    time = ((uint64_t)file_time.dwLowDateTime);
    time += ((uint64_t)file_time.dwHighDateTime) << 32;

    tp->tv_sec = (long)((time - EPOCH) / 10000000L);
    tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
    return 0;
}
#endif

/***************************************************************************//**
*
* @brief Stateless function, converting time from unix timestamp to irig-b format
*
* @param[in]  ts     input unix timestamp, seconds since Jan 1st 1970 (not counting leap secs)
* @param[out] year   output year 
* @param[out] doy    output day of year 1-366 (Jan 1st = 1)
* @param[out] HH     output hour of day 0-23
* @param[out] MM     output minute 0-59
* @param[out] SS     output second 0.0-59.9999
*
* @note Works from Jan 1 1970 to Dec 31 2104. Then fails completely at Feb 7 2106 07:20:00, when we roll over to 1970
******************************************************************************/
void gm_to_irigb(double ts, uint16_t* year, uint16_t* doy, uint8_t* hour, uint8_t* min, float* sec){
    uint32_t TS, Y, Y0, Y1;
    int32_t D,H,M,ly;
    float S;
    double sec_frac, sec_int;
    sec_frac = modf(ts,&sec_int);
    TS = sec_int;
    Y = (TS+18*60*60) / (31557600);             //Approx number of full years since epoc (Adding 3/4th of a day to skew the new years day problem to 2400h)
    Y0 = Y;
    ly = (Y+1)/4  - (Y+69)/100 + (Y+369)/400;   //Full leap years since EPOCH
    TS -= ly*60*60*24;                          //Subtracting one day for each leap year since epoc
    Y = TS/(31536000);                          //Full Years since EPOC
    Y1 = Y;
    TS -= Y*31536000;
    D = TS/(86400);
    TS -= D*86400;
    H = TS/3600;
    TS -= H*3600;
    M = TS/60;
    TS -= M*60;
    S = TS + sec_frac;
    if(D==364 && (((Y+2)%4)==0) && (Y0!=Y1)) //Fix the problem with day 366 every leap year
        D = 365;
    *year = Y + 1970;       //Epoch is 00:00:00 Jan. 1st 1970
    *doy = D+1;             //Jan 1st is doy=1
    *hour = H;
    *min = M;
    *sec = S;
}

/***************************************************************************//**
*
* @brief Stateless function, converting time from irig-b format to unix timestamp.
*
* @param[in] year   input year 
* @param[in] doy    input day of year 1-366 (Jan 1st = 1)
* @param[in] HH     input hour of day 0-23
* @param[in] MM     input minute 0-59
* @param[in] SS     input second 0-59
* @return           unix timestamp, seconds since Jan 1st 1970 (not counting leap secs)
*
******************************************************************************/
double irigb_to_gm(uint16_t year,uint16_t doy,uint8_t HH,uint8_t MM,float SS){
    double ts;
    int32_t yy,ly;

    //Full year since EPOCH
    yy = year-1970;
    //Full leap years since EPOCH
    if (yy>=0)
        ly = (yy+1)/4  - (yy+69)/100 + (yy+369)/400;
    else
        ly = (yy-2)/4  - (yy-30)/100 + (yy-330)/400;
    //Calculate integer unix timestamp
    ts = yy*365+ly+(doy-1); //Day 1 is Jan 1
    ts = ts*24+HH;
    ts = ts*60+MM;
    ts = ts*60+SS;
    return ts;
}


uint16_t date_to_doy(uint16_t year,uint16_t month, uint16_t date){
	uint16_t dim[] = {0, 31,  59,  90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
	uint16_t dim_leap[] = {0, 31,  60,  91, 121, 152, 182, 213, 244, 274, 305, 335, 366};

	uint8_t leap;
	if (year%400==0) leap = 1;
	else if (year%100==0) leap = 0;
	else if (year%4==0) leap = 1;
	else leap = 0;

	if (leap)
		return dim_leap[month-1]+date;
	else
		return dim[month-1]+date;
}

int sprintf_unix_time(char * str, double ts){
	uint16_t t_year; uint16_t t_doy; uint8_t t_hour; uint8_t t_min; float t_sec;
	gm_to_irigb(ts, &t_year, &t_doy, &t_hour, &t_min, &t_sec);
	return sprintf(str,"Year = %d doy= %d %02d:%02d:%06.3f Unix_ts=%9.2f",t_year,t_doy,t_hour,t_min,t_sec,ts); 
}

static double try_patterns(const char *filename, const char *patterns[], int n_patterns, int date_only) {
    size_t n = strlen(filename);

    for (int p = 0; p < n_patterns; p++) {
        for (size_t ii = 0; ii < n; ii++) {
            struct tm tm_time;
            memset(&tm_time, 0, sizeof(tm_time));  // ensure unset fields default to 0
            tm_time.tm_isdst = 0;                  // time should be in UTC so no DST

            if (strptime(filename + ii, patterns[p], &tm_time)){
                // If we matched only a date, midnight is already implied by the memset.
                // (For datetime patterns, strptime will have filled H/M/S.)
                time_t t = mktime(&tm_time);
                if (t != (time_t)-1) {
                    double ts = (double)t;
                    // Sanity check: must be in the 21st century
                    if (ts > 946681200.0 && ts < 4102441200.0) {
                        return ts;
                    }
                }
            }
        }
    }
    return 0.0;
}

double parse_timestamp_from_filename(const char *filename) {
    // Common datetime patterns (date + time)
    static const char *datetime_patterns[] = {
        "%Y%m%d-%H%M%S",
        "%Y%m%d_%H%M%S",
        "%Y-%m-%d_%H-%M-%S",
        "%Y%m%d%H%M%S",
        "%Y-%m-%dT%H:%M:%S"
    };

    // Date-only fallbacks (assume 00:00:00 local time)
    static const char *date_only_patterns[] = {
        "%Y-%m-%d",
        "%Y%m%d"
    };

    // First try full datetime patterns
    double ts = try_patterns(filename, datetime_patterns,   (int)(sizeof(datetime_patterns)/sizeof(datetime_patterns[0])), 0);
    if (ts != 0.0) return ts;

    // Then try date-only patterns (midnight implied)
    ts = try_patterns(filename, date_only_patterns, (int)(sizeof(date_only_patterns)/sizeof(date_only_patterns[0])), 1);
    if (ts != 0.0) return ts;

    return 0.0; // no match
}



	

double os_time(void){
	double ts;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	ts = tv.tv_sec;
	ts += tv.tv_usec/1000000.;
	return ts;
}
