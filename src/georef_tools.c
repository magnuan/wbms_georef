#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "linalg.h"

#include "cmath.h"
#include <math.h>
#include "wbms_georef.h"
#include "raytracing.h"
#include "time_functions.h"

static uint8_t verbose = 0;
static float time_diff_limit = 0.1f;
static uint8_t use_sonar_sv_for_initial_ray_parameter = 1;

void set_time_diff_limit(float t){
    time_diff_limit = t;
}

void set_use_sonar_sv_for_initial_ray_parameter(uint8_t val){
    use_sonar_sv_for_initial_ray_parameter = val;
}
	
int attitude_test(sensor_params_t* sensor_params, float yaw,  float pitch,  float roll, float droll_dt, float dpitch_dt, float dyaw_dt){ 
    if (sensor_params->max_abs_yaw){
        if (ABS(yaw) > sensor_params->max_abs_yaw) return 1;
    }
    if (sensor_params->max_abs_dyaw_dt){
        if (ABS(dyaw_dt) > sensor_params->max_abs_dyaw_dt) return 1;
    }
    if (sensor_params->max_abs_pitch){
        if (ABS(pitch) > sensor_params->max_abs_pitch) return 1;
    }
    if (sensor_params->max_abs_dpitch_dt){
        if (ABS(dpitch_dt) > sensor_params->max_abs_dpitch_dt) return 1;
    }
    if (sensor_params->max_abs_roll){
        if (ABS(roll) > sensor_params->max_abs_roll) return 1;
    }
    if (sensor_params->max_abs_droll_dt){
        if (ABS(droll_dt) > sensor_params->max_abs_droll_dt) return 1;
    }
    return 0;
}


/* Function to apply the beam correction polynom to a beam angle
*  TODO consider to optimize using a LUT
*/
float apply_beam_correction_poly(float angle, float* poly, uint32_t order){
    if (order){
        float adj = poly[0];
        for (uint32_t n = 1;n<order;n++){
            adj += poly[n] * powf(angle,(float) n);
        }
        return angle+adj;
    }
    return angle;
}


/*
 offset  	Offset for sensor (sonar/lidar) rel navigation origo
 x,y,z		array of coordinates (x=fwd y=stb, z=down)
 t			array of transmitt to receive delay 
 n			number of elements in r,az,el arrays

 nav_x, nav_y, nav_z Position of navigation origo in global coordinates (i.ex UTM easting,northing,up)
 nav_dx_p, nav_dy_p, nav_dz_p Speed of navigation origo in navigation coordinates (fwd, stb, down)
 nav_yaw, nav_pitch, nav_roll, nav_droll Rotations of navigation coordinates relative to global coordinates

 Function assumes that sampling of nav data and sensor data is done simultanious in time, if not, please interpolate/extrapolate nav data before sending it to this functon.
*/

int georef_to_global_frame(
					/* Sensor offset*/ 		offset_t* offset, 
					/*sensor data arrays*/ 	float* xs, float* ys, float* zs, uint32_t n, /* Detection points in SONAR frame */
					/*sonar sv */			float c,												/* Sound velocity for sonar data at sonar head, to apply raybending corrections. 0 for no correction*/
					/*Nav pos*/ 			double nav_x, double nav_y, double nav_z, 			/*Position in global coordinates (east,north,up)*/
					/*Nav attitude*/		float nav_yaw, float nav_pitch, float nav_roll, /*Rotations of posmv coordinates*/
                    /*Ray tracing mode*/    int ray_tracing_mode, float sonar_mounting_depth,
					/*OUTPUT*/
					/*sensor data arrays*/	double* x, double* y, double* z
					){

	//*** Detection points in POS frame ***/
	//float xp[MAX_DP], yp[MAX_DP], zp[MAX_DP];																						 
    float* xp = malloc(n*sizeof(float));
    float* yp = malloc(n*sizeof(float));
    float* zp = malloc(n*sizeof(float));
	//*** Detection points in GLOBAL frame  (with boat in origin)***/
	//float xg[MAX_DP], yg[MAX_DP], zg[MAX_DP];																						 
    float* xg = malloc(n*sizeof(float));
    float* yg = malloc(n*sizeof(float));
    float* zg = malloc(n*sizeof(float));

    if ((xp==NULL) || (yp==NULL) ||(zp==NULL) ||(xg==NULL) ||(yg==NULL) ||(zg==NULL)){
        return -1;
    }

	//*** Detection points time derivative in GLOBAL frame***/
	//*** Rotation matrixes ***/
	float Rsp[3][3]; //SONAR to POS frame
	float Rpg[3][3]; //POS to GLOBAL frame

						
	/**** Rotate from SONAR to POS frame, Applying attitude offsets*****/
	rot_matrix(offset->yaw,offset->pitch,offset->roll,Rsp);
	rot_coordinates(xs,ys,zs,Rsp,n, xp,yp,zp);
	/**** Rotating from POS to GLOBAL frame, Applying pos rotations *******/
	//Rotation explenation:
	// Yaw and Pitch is (mostly) defined by the TX time, as the TX-lobe is narrowest allongtrack
	// Roll is defined by RX time, since we only have acrosstrack resolution in RX
	// For YAW and PITCH we use the vessel rotations at TX time
	// For ROLL we use approximate rotations at RX time, calculated by using the time offset for each detectionpoint, and the time derivate of the rotation matrix = dR/droll * droll/dt * deltaTx2Rx
	//Rotation matrix from posmv to global reference frame (At ts time, iow TX time)
	rot_matrix(nav_yaw,nav_pitch,nav_roll,Rpg);

    if (ray_tracing_mode==ray_trace_none){
        /**** Translate from SONAR to POS frame, Applying mounting offsets*****/
        for (uint32_t ii = 0;ii<n;ii++){
            xp[ii] = xp[ii] + offset->x; //xp is vessel fwd
            yp[ii] = yp[ii] + offset->y; //yb is vessel stb
            zp[ii] = zp[ii] + offset->z; //zb is vesel down
        }
        //Positions in global frame before translation to boat position, if data was instantanoiusly sampled at TX-moment
        rot_coordinates(xp,yp,zp,Rpg,n, xg,yg,zg);
    }
    else{

        //Positions in global frame before translation to boat position, if data was instantanoiusly sampled at TX-moment
        rot_coordinates(xp,yp,zp,Rpg,n, xg,yg,zg);
        //Rotating pos referenced offsets to global frame 
        float offset_xg, offset_yg, offset_zg;
        rot_coordinates(&(offset->x),&(offset->y),&(offset->z),Rpg,1, &offset_xg,&offset_yg,&offset_zg);
        //Apply SV-profile ray-bending correction to POS-ref sonar sounding data, before we start translating and moving to global reference frame
        if (c>=0){
            if (use_sonar_sv_for_initial_ray_parameter==0) c = 0;   //Set c=0 to force ray-tracing to use table sv instead of sonar sv for initial ray parameter
            switch(ray_tracing_mode){
                case ray_trace_fixed_depth_lut: //LUT fixed depth
                    apply_ray_bending(xg,yg,zg,n,c);
                    break;
                case ray_trace_fixed_depth_direct: //Direct fixed_depth
                    apply_ray_bending_direct(xg,yg,zg,n,c,0); //SV-table is calculated wrt sonar, so depth input  is always 0
                    break;
                case ray_trace_var_depth: //Direct variable_depth
                    apply_ray_bending_direct(xg,yg,zg,n,c,offset_zg-(float)nav_z+sonar_mounting_depth);	//Offset_zg is pos down, nav_z is po up, sonar mounting depth is pos down (here sonar mounting depth is the water depth that gives nav=0 
                    break;
            }
        }
        //Finally we need to add the linear offsets with pos-global rotated offsets (a bit aquard to do it likethis, but we need untranslated global ref coordinated soundings to do SV correction
        for (uint32_t ii = 0;ii<n;ii++){
            xg[ii] = xg[ii] + offset_xg; //xg is north
            yg[ii] = yg[ii] + offset_yg; //yg is east
            zg[ii] = zg[ii] + offset_zg; //zg is down
        }
    }
	//An unrotated vessel (yaw=0, pitch=0, roll=0) is now sitting flatt pointing north, so 
	// xg is global north
	// yg is global east
	// zg is global down
	

	/******Translating Position of detection points to global frame********/ 
	//Now we need double precision, since we might be thousands of km away from origin
	for (uint32_t ii = 0;ii<n;ii++){
		x[ii] = nav_x + xg[ii];
		y[ii] = nav_y + yg[ii];
		z[ii] = nav_z + zg[ii];
	}
    
    free(xp);free(yp);free(zp);
    free(xg);free(yg);free(zg);
	return 0;
}



size_t find_closest_index_in_posdata(navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, double ts){
    navdata_t *pos0;      //Navdata last before and firs after ts instant
	double ts_offset0=1000;
    double ts_offset1=1000;
    size_t pos_ix0, pos_ix1;
    pos_ix0 = pos_ix;

    //Start at last posdata index, go through datasets bacwards in time until we have the dataset before last bathy data
    pos0 = &(posdata[pos_ix0]);
    for(size_t ii=0;ii<NAVDATA_BUFFER_LEN;ii++){
        pos_ix1 = pos_ix0;  
        ts_offset1 = ts_offset0;
        pos0 = &(posdata[pos_ix0]);
	    ts_offset0 = ts - pos0->ts;
        if (ts_offset0>0) break;
        pos_ix0 = (pos_ix0+NAVDATA_BUFFER_LEN-1)%NAVDATA_BUFFER_LEN;
    }
    ts_offset0 = ABS(ts_offset0);
    ts_offset1 = ABS(ts_offset1);
    return (ts_offset0>ts_offset1)?pos_ix1:pos_ix0; 
}


    

/*
/ INPUT:
/ posdata   Pointer to NAVDATA_BUFFER_LEN long array on navigation data (circular buffer)
/ pos_ix    Index of last dataset in circular buffer posdata
/ ts        Time stamp value to find inverpolated nav data for
/
/ OUTPUT:
/ nav_x, nav_y, nav_z Output navigation position
/ nav_yaw, nav_pitch, nav_roll Output navigation attitude
/ nav_dyaw_dt, nav_dpitch_dt, nav_droll_dt Output navigation attitude time derivates
*/
int calc_interpolated_nav_data( navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, double ts,/*OUTPUT*/ double* nav_x, double* nav_y, double* nav_z, float* nav_yaw, float* nav_pitch, float* nav_roll, float* nav_dyaw_dt, float* nav_dpitch_dt, float* nav_droll_dt){
    
    navdata_t *pos0, *pos1;      //Navdata last before and firs after tx instant
	double ts_offset0, ts_offset1, max_ts_offset;
    double scaling0, scaling1;

    //Start at last posdata index, go through datasets bacwards in time until we have the dataset before last bathy data
    pos0 = &(posdata[pos_ix]);
    for(size_t ii=0;ii<NAVDATA_BUFFER_LEN;ii++){
        pos1 = pos0;
        pos0 = &(posdata[pos_ix]);
	    ts_offset0 = ts - pos0->ts;
        if (ts_offset0>0) break;
        pos_ix = (pos_ix+NAVDATA_BUFFER_LEN-1)%NAVDATA_BUFFER_LEN;
    }
	ts_offset1 = ts - pos1->ts;
    //fprintf(stderr,"ts_offset0=%f ts_offset1=%f\n",ts_offset0,ts_offset1);
    if (ts_offset0<0 || ts_offset1>0) return -1;
    
    max_ts_offset = MAX(ts_offset0,-ts_offset1);

	if ((pos0->ts) == 0) return -1;
	
	if (max_ts_offset>time_diff_limit){
		char str_buf[256];
		if(verbose){
			fprintf(stderr,"LARGE TS diff (sensor-nav)= %f sec\n",ts_offset0);
			sprintf_unix_time(str_buf, ts);
			fprintf(stderr,"BATH: %s\n",str_buf);
			sprintf_unix_time(str_buf, pos0->ts);
			fprintf(stderr,"NAV : %s\n",str_buf);
		}
		return -1;
	}
    //Finding interpolation scaling of the two pos data sets.
    scaling0 = (-ts_offset1)/(ts_offset0-ts_offset1);
    scaling1 = (ts_offset0)/(ts_offset0-ts_offset1);
	
    //-- Calculate interpolated nav data at ts instant --
	*nav_x = (scaling0 * pos0->x) + (scaling1 * pos1->x);
	*nav_y = (scaling0 * pos0->y) + (scaling1 * pos1->y);
	*nav_z = (scaling0 * pos0->z) + (scaling1 * pos1->z);
	*nav_yaw = (float)((scaling0 * pos0->yaw) + (scaling1 * pos1->yaw));          //Rotation around vessel Z axis (downwards) positive turning right. 0,0,0 means vessel X (forward) pointing towards north horizonth,  vessel Y (Starbord) towards east horizonth
	*nav_pitch = (float)((scaling0 * pos0->pitch) + (scaling1 * pos1->pitch));    //Rotation around vessel Y axis (starbord) positive pitching up.
	*nav_roll = (float)((scaling0 * pos0->roll) + (scaling1 * pos1->roll));       //Rotation around vessel X axix (forward) positive rolling clockwise

    float dt = (pos1->ts - pos0->ts);
    *nav_dyaw_dt = (pos1->yaw - pos0->yaw)/dt;
    *nav_dpitch_dt = (pos1->pitch - pos0->pitch)/dt;
    *nav_droll_dt = (pos1->roll - pos0->roll)/dt;

	//fprintf(stderr,"Nav roll = %8.3fdeg, pitch = %8.3fdeg, yaw = %8.3fdeg\n",nav_roll*180/M_PI, nav_pitch*180/M_PI, nav_yaw*180/M_PI);
    return 0;
}

/*
/ INPUT:
/ posdata   Pointer to NAVDATA_BUFFER_LEN long array on navigation data (circular buffer)
/ pos_ix_tail    Index of last dataset in circular buffer posdata
/ ts        Time stamp value for first roll value in output vector
/ t_dur     Time duration to calculate 
/ fs        Output vector sample rate
/ N         Maximum number of values to calculate (roll_vector length)
/
/ OUTPUT:
/ pointer to output vector for caluclated roll and z values
/ All walues are wrt roll/z at ts instant
/
/ TODO  This function needs to calculate roll as rotation around sonar x-axis (azmuth angle)
/       If sonar is significantly rotated with resoect to vessel / nav. We need tor transform yaw-pith-roll to rotation and heave wrt sonar coordinates
*/
int calc_interpolated_roll_and_z_vector(navdata_t posdata[NAVDATA_BUFFER_LEN], size_t  pos_ix_tail, double ts, float t_dur, float fs, size_t N, /*output*/ float* roll_vector, float* z_vector){
    navdata_t *pos0, *pos1;      //Navdata last before and firs after tx instant
	float ts_offset0, ts_offset1;
    float scaling0, scaling1;
    float t = 0;
    float dt = 1.f/fs;
    size_t pos_ix = pos_ix_tail;

    //Start at last posdata index, go through datasets bacwards in time until we have the dataset before last bathy data
    for(size_t ii=0;ii<NAVDATA_BUFFER_LEN;ii++){
        pos0 = &(posdata[pos_ix]);
	    ts_offset0 = (float)(ts - pos0->ts);
        if (ts_offset0>0) break;
        pos_ix = (pos_ix+NAVDATA_BUFFER_LEN-1)%NAVDATA_BUFFER_LEN;
    }
    pos_ix = (pos_ix+1)%NAVDATA_BUFFER_LEN;
    pos1 = &(posdata[pos_ix]);
    
	if ((pos0->ts) == 0) return -1;

    float roll0, z0;

    t = 0;
    size_t ii;
    for (ii = 0; ii < N; ii++){ 
        if(t>=t_dur) break;
        ts_offset0 = (float)(ts + (double)t - pos0->ts);
        ts_offset1 = (float)(ts + (double)t - pos1->ts);
        //Finding interpolation scaling of the two pos data sets.
        scaling0 = (-ts_offset1)/(ts_offset0-ts_offset1);
        scaling1 = (ts_offset0)/(ts_offset0-ts_offset1);
        roll_vector[ii] = (scaling0 * (float)pos0->roll) + (scaling1 * (float)pos1->roll);      //TODO: Here we must calculate sensor roll from vessel attitude. Not just use vessel roll
        z_vector[ii] = (scaling0 * (float)pos0->z) + (scaling1 * (float)pos1->z);      //TODO: Here we must calculate sensor z from vessel attitude. Not just use vessel z
        if (ii==0) roll0 = roll_vector[0];
        if (ii==0) z0 = z_vector[0];
        roll_vector[ii] -= roll0;
        z_vector[ii] -= z0;
    
        t+=dt;                          //Increment one step in time data
        if(pos_ix != pos_ix_tail){      // Don't wrap (This could happen if the sensor data gets ahead of the navigation data (with prefetch), 
                                        //    if this happens we will end up extrapolating, which is not good, but better than wrapping the navigation data
                                        //    this check is not neccessary if we can guarantee that navigation data is ahead of sensor data.
            if ( (t+ts) > pos1->ts ){       // If new time is after pos1 time, go one step forward in input time data
                pos0 = pos1;
                pos_ix = (pos_ix+1)%NAVDATA_BUFFER_LEN;
                pos1 = &(posdata[pos_ix]);
            }
        }
    }
    return 0;

}
