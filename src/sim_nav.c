#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>

#include "cmath.h"
#include <math.h>
#include "wbms_georef.h"
#include "sim_nav.h"

static double pos_sim_speed = 1.0; 

//Simulator is always in sync
int sim_nav_seek_next_header(int fd){
    return 0;
}

//Simulator always has data
int sim_nav_fetch_next_packet(char * data, int fd){
    return 1;
}

//Whatever
int sim_nav_identify_packet(char* databuffer, uint32_t len, double* ts_out){
    return 1;
}

void sim_nav_set_params(float speed){
    pos_sim_speed = speed;
}

int sim_nav_process_packet(double ts_in, double* ts_out, double z_offset, uint16_t alt_mode, PJ *proj, navdata_t *navdata, aux_navdata_t *aux_navdata){
	static double northing = 0.;
	static double easting = 0.;
	static double ts = 0.; 
	double dt = 1./20;

	if (ts_in > (ts+10)) ts = ts_in; //If more than 10 sec behind detection data, fast forward nav time
	if (ts_in < (ts-10)) ts = ts_in; //If more than 10 sec ahead detection data, fast forward nav time
    
    //Generate simulated nav data	
    navdata->ts = ts;
    navdata->roll =0 ;
    navdata->pitch = 0;
    navdata->yaw =0;
    navdata->alt = 0;
    navdata->heave = 0;
    *ts_out = ts;

    //fprintf(stderr, "Gen sim posmv data ts = %f  ts detections = %f\n",ts, ts_in);
    northing += pos_sim_speed*dt;
                
    navdata->x = northing;				  //Positive North
    navdata->y = easting;				  //Positive East
    navdata->z = 0;
    double alt_out;
    latlon_from_kart(navdata->x, navdata->y , navdata->z, proj, /*output*/ &(navdata->lon), &(navdata->lat) , &alt_out);
    ts += dt;

	return NAV_DATA_PROJECTED;
}
