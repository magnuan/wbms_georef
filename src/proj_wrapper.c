#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "proj_wrapper.h"
#include "cmath.h"
#include <math.h>


#ifdef USE_GEOTRANZ
#include "utm.h"

//Do forward transformation from geodetic to projected coordinates
void latlon_to_kart(double lon, double lat , double alt, PJ* trans, /*output*/ double* x_out, double* y_out , double* z_out){
    long utm_zone;
    char hemisphere;
    Set_UTM_Parameters(trans->ellipsoid_a ,trans->ellipsoid_f, trans->utm_zone);
    Convert_Geodetic_To_UTM (lat,lon, &utm_zone, &hemisphere,y_out,x_out);
    *z_out = -alt;
}

//Do forward transformation from projected to geodetic coordinates
void latlon_from_kart(double x, double y , double z, PJ* trans, /*output*/ double* lon_out, double* lat_out , double* alt_out){
    if(trans){
        Convert_UTM_To_Geodetic( trans->utm_zone, trans->hemisphere,y,x,lat_out,lon_out);
    }
    else{
        *lat_out = 0;
        *lon_out = 0;
    }
    *alt_out = -z;
}

char* find_field(char* pstring, char* field){
    char *s0, *ss, *se, *outstr;  
    size_t outlen;
    s0 = strstr(pstring,field);
    if(s0){
        ss=strstr(s0,"=");
        if (ss){    
            ss++;
            se = strstr(ss," ");
            if(se){
                outlen = se-ss+1;
            }
            else{
                outlen = strlen(ss)+1;
            }
            outstr = malloc(outlen);
            if (outstr) {
            memcpy(outstr,ss,outlen);
            outstr[outlen-1] = 0;
            }
            return outstr;
        }
    }
    return NULL;
}

int latlon_to_proj_from_string(char* pstring,  PJ** latlon_to_proj){
    fprintf(stderr,"Using GEOTRANZ for coordinate transformation\n");
    *latlon_to_proj = malloc(sizeof(PJ));
    if (*latlon_to_proj == NULL) return -1;
    double a,f;
    long utm_zone;
    char hemisphere;
    Get_UTM_Parameters(&a,&f,&utm_zone);

    fprintf(stderr,"Interpreting  Proj string '%s' to:\n",pstring);
     //" +proj=utm +zone=10 +ellps=WGS84"
    char* proj_str = find_field(pstring,"+proj=");
    char* zone_str = find_field(pstring,"+zone=");
    char* ellps_str = find_field(pstring,"+ellps=");
    if(proj_str){
        //fprintf(stderr,"proj_str=%s\n",proj_str);
        if (!(strstr(proj_str,"UTM") || strstr(proj_str,"utm"))){
            fprintf(stderr, "WARNING: GEOTRANZ ONLY SUPPORTS UTM RIGHT NOW. FOR GENERAL PROJ STRING SUPPORT, COMPILE WITH LIBPROJ\n");
        }
        free(proj_str);
    }
    if(zone_str){ 
        utm_zone = atoi(zone_str);
        //fprintf(stderr,"zone_str=%s\n",zone_str);
        free(zone_str);
    }
    else{
        fprintf(stderr,"No zone info given, defaulting to UTM zone 33\n");
        utm_zone = 33;
    }

    if(ellps_str){ 
        //fprintf(stderr,"ellps_str=%s\n",ellps_str);
        if (!(strstr(ellps_str,"WGS84") || strstr(ellps_str,"wgs84"))){
            fprintf(stderr, "WARNING: GEOTRANZ ONLY SUPPORTS WGS84 RIGHT NOW. FOR GENERAL PROJ STRING SUPPORT, COMPILE WITH LIBPROJ\n");
        }
        free(ellps_str);
    }
    if (strstr(pstring,"+south"))
         hemisphere = 'S';
    else
        hemisphere = 'N';

    fprintf(stderr,"Using Ellipsoid a=%f f=%f utm_zone=%ld hemisphere=%c\n",a,f,utm_zone, hemisphere);
    Set_UTM_Parameters(a,f,utm_zone);

    (*latlon_to_proj)->ellipsoid_a = a;
    (*latlon_to_proj)->ellipsoid_f = f;
    (*latlon_to_proj)->utm_zone = utm_zone;
    (*latlon_to_proj)->hemisphere = hemisphere;

    
    return 0;
}
#endif




#ifdef USE_PROJ
//Do forward transformation from geodetic to projected coordinates
void latlon_to_kart(double lon, double lat , double alt, PJ* trans, /*output*/ double* x_out, double* y_out , double* z_out){
    PJ_COORD utm_coord,latlon_coord;
    //PROJ uses geodetic coordinates in degrees
    latlon_coord.lpz.lam = lon*180/M_PI;
    latlon_coord.lpz.phi = lat*180/M_PI;
    latlon_coord.lpz.z = alt;
    utm_coord = proj_trans(trans,PJ_FWD,latlon_coord);
    *x_out=     utm_coord.xyz.y; //PROJ has x=east we have x north
    *y_out =    utm_coord.xyz.x;
    *z_out =   -utm_coord.xyz.z; //PROJ has z up, we have z down
}

//Do forward transformation from projected to geodetic coordinates
void latlon_from_kart(double x, double y , double z, PJ* trans, /*output*/ double* lon_out, double* lat_out , double* alt_out){
    PJ_COORD utm_coord,latlon_coord;
    
    utm_coord.xyz.x = y;    //PROJ has x=east we have x north
    utm_coord.xyz.y = x;                                  
    utm_coord.xyz.z = -z;   //PROJ has z up, we have z down  
    latlon_coord = proj_trans(trans,PJ_INV,utm_coord);
    //PROJ uses geodetic coordinates in degrees
    *lon_out = latlon_coord.lpz.lam*M_PI/180;
    *lat_out = latlon_coord.lpz.phi*M_PI/180;
    *alt_out = latlon_coord.lpz.z;
}

int latlon_to_proj_from_string(char* pstring,  PJ** latlon_to_proj){
   fprintf(stderr,"Using PROJ for coordinate transformation\n");
    //TODO FIX USING USER SPECIFIED PSTRINGS
    
    PJ *pj_longlat  = proj_create(PJ_DEFAULT_CTX,"+proj=longlat +ellps=WGS84 +type=crs");
    char ostr[1024];
    snprintf(ostr,1024,"%s +type=crs",pstring);
    PJ *pj_out      = proj_create(PJ_DEFAULT_CTX,ostr);//"+proj=utm +zone=32 +ellps=WGS84 +type=crs");

    *latlon_to_proj = proj_create_crs_to_crs_from_pj(PJ_DEFAULT_CTX,pj_longlat,pj_out,NULL,NULL);
    if(*latlon_to_proj == 0) return -1;
    return 0;
}
#endif



#define EARTH_CIRC 39931000.
//Because of the projection, there will be a offset between the true north, given by the navigation system, and the projection north (northing)
//This function is a simple way to calculate this angular offset.
double north_to_northing_offset(double lon, double lat, PJ* trans){

    double easting[2];
    double northing[2];
    double alt[2];
    latlon_to_kart(lon, lat , 0, trans, /*output*/ &(northing[0]), &(easting[0]) , &(alt[0]));
    latlon_to_kart(lon, lat+(100*360./EARTH_CIRC) , 0, trans, /*output*/ &(northing[1]), &(easting[1]) , &(alt[1]));  //Go 100m north;
    //fprintf(stderr,"N=%f E=%f  -- N=%f E=%f\n",northing[0],easting[0],northing[1],easting[1]);
    return atan2(easting[1]-easting[0], northing[1]-northing[0]);
}

void default_projection(char* pstring,double lat, double lon){
    char proj_string[256];
    int utm_zone;
    if (lat==0 && lon==0)
        utm_zone = 33;
    else
        utm_zone = ((int)round((((lon/M_PI)+1)*30)))%60 +1;

    sprintf(proj_string,"+proj=utm +zone=%d +ellps=WGS84",utm_zone);
    fprintf(stderr,"Guessing UTM zone %d (lat = %8.5fdeg lon = %8.5fdeg)\n",utm_zone,lat*180/M_PI,lon*180/M_PI);
    strcpy(pstring, proj_string);
}
