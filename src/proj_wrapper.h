#ifndef __PROJ_EXTRA_H__
#define __PROJ_EXTRA_H__

#define PREFER_GEOTRANZ


#if  defined (ENABLE_PROJ)  && !defined (PREFER_GEOTRANZ)
    #define USE_PROJ
#else
    #define USE_GEOTRANZ
#endif

#ifdef USE_PROJ
#include <proj.h>
#endif

#ifdef USE_GEOTRANZ
//Struct describing the coordinate transformation from geodetic to projected coordinates to use when using geotranz
typedef struct { 
    double ellipsoid_a; // Semi-major axis of ellipsoid, in meters
    double ellipsoid_f; // Flattening of ellipsoid
    long utm_zone;      // UTM override zone, zero indicates no override
    char hemisphere;
} PJ;
#endif



void latlon_to_kart(double lon, double lat , double alt, PJ* trans, /*output*/ double* x_out, double* y_out , double* z_out);
void latlon_from_kart(double x, double y , double z, PJ* trans, /*output*/ double* lon_out, double* lat_out , double* alt_out);

double north_to_northing_offset(double lon, double lat, PJ* proj_trans);
int latlon_to_proj_from_string(char* pstring,  PJ** latlon_to_proj);
void default_projection(char* pstring,double lat, double lon);

#endif
