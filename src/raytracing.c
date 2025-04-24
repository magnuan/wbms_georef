#include <stdio.h>
#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <math.h>
#include "cmath.h"
#include "non_posix.h"
#include "svpdata.h"

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#define INTERPOL_COR_TABLE
#define PYTHON_PRINTOUT



static int ray_bend_valid=0;
static int ray_bend_invalid=0;



int get_ray_bend_valid(void){
    return ray_bend_valid;
}
int get_ray_bend_invalid(void){
    return ray_bend_invalid;
}


static int calc_dr_dt_rev_hovem(double xi,double c0,double c1,double dz,float* dr, float* dt){
	double si0,si1;
    double g,a,b,h;

	if(xi==0) return 1;					// Zero ray parameter case,initial beam horizontal, soundings with this, should be scrapped
	si0 = (1- (xi*xi*c0*c0 )); 				// = (1-cos2(teta0)) = sin2(teta0)  +: beam goes vertically down through the layer, 0: beam goes parallell with layer, -: beam can not exist in layer
	si1 = (1- (xi*xi*c1*c1 ));
	if ( (si0<=0) || (si1<=0)) return 1;	 //Beam reversing, downwards going beam turning upwards or opposite, soundings wiht this should be scrapped
	g = (c1-c0) / dz;  						 //Sound speed gradient

	//Handle zero gradient case, this is a valid case and needs to have an alternative dr and dt calculation, it is simple as it is a straight line
	if (g==0){ 
		a=c0*xi;			//cos(teta)
		b=sqrtf(1-(a*a));	//sin(teta)
		h = dz / (b/a);	
		*dr = h;			//dr = dz/tan(teta)
		*dt = (double)(sqrt( dz*dz + h*h))/c0;
		return 0;
	}
	//Since we now have checked out cases where si0 or si1 is zero or negative, we can do this, to get sin(teta0) and sin(teta1), sin of beam angle entering and exiting layer
	si0 = sqrt(si0);
	si1 = sqrt(si1);
	//Here is the calulation of dr for a beam passing through the layer, assumning a constant radius curve
	*dr = (double)(si0 - si1) / (xi*g);
	//Here is the calulation of dt for a beam passing through the layer, assumning a constant radius curve
	*dt = (double)fabs(log((double)(c1/c0)* ((1+si0)/(1+si1))) / (double)g);
	return 0;
}


//Resolution and length of depth axis of correction table
// TODO dynamically calculate DZ based on sv table range
//#define DZ 0.25f				
static float DZ = 0.25f;
#define NZ 400
//Resolution and length of start-angle axis of correction table
// angle is 0 defined deg for horizontal and 90 deg for nadir
//#define DANGLE (1.*M_PI/180)
#define NANGLE 180
#define DANGLE (0.5f*M_PI/NANGLE)
//Resolution, length and starting point of start-sound-velocity axis of correction table
#define DC 2.0f
#define NC 90
#define MINIMUM_C 1400.f

/*Calculate horizontal distance and propagation time for: 
 a single depth layer of thickness (dz)
 with given start and stop sound velocity (c0 and c1) 
 given ray parameter (xi) cos(teta0)/c0
 
	#Based on "Ray trace modeling of underwater sound" by Jens M Hovem
	# Simplified by the fact that we dont care about beams turning upwards, they are no good for bathemetry anyways
	#
	# Snells law states cos(teta(z)) / c(z) = cos(teta0)/c0 = xi
	# Where xi is the "ray parameter" given by the initial condition (teta0, c0)
	# teta is the angle of the ray relative to horizontal (plane normal to z), iow 90deg is vertical, 0 deg is horizontal
	#
	# Beams will bend with a radius of curvature R(Z) given the "ray parameter" and the sound velocity gradient g(z) = dc(z)/dz
	# R(z) = - 1/(xi * g(z))
	#
	# Assuming
	#  A) that sound only changes with depth
	#  B) That the ray parameter is constant for a given beam (this assummtion does not hold when the beam is reflected, then a new ray parameter needs to be calculated for the reflected beam
	# We can model the water volume as discrete layers z[i], for in each we can assume a constant gradient g(z) and thus a constant radius of curvature R(z).
	#
	# Hovem shows that for each layer, defined from z[i] to z[i+1], the horizontal propagation of the beam (r[i+1] - r[i]) can be given by the formula
	# r[i+1] -r[i] = 1/(xi g[i]) * ( sqrt(1- (xi*c[i] )**2 ) - sqrt(1- (xi*c[i+1])**2) )			(17)
	# Also, the propagation time through each layer
	# t[i+1] - t[i] = 1/abs(g[i]) * ln ( c[i+1]/c[i] * (1+sqrt(1 -(xi*c[i])**2))/(1+sqrt(1-(xi*c[i+1]))) )
*/

//Table of interpolated sound velocities per depth 
static float sv_table[NZ];

static int has_corr = 0;
//Table of angle correction values indexed by initial sound velocity, initial/obseved direction and true depth (true-observed)
static float corr_angle[NC][NANGLE][NZ];
//Table of range correction values, relative values corr = (true-observed)/observed
static float corr_range[NC][NANGLE][NZ];
// Table with 0 for all valid corrections and 1 for invalid
static uint8_t corr_invalid[NC][NANGLE][NZ];

//Dilate invald table, entry at index is invalid if it or any of its 7 higher neighbour is
static void dilate_invalid_table_for_interpolation(void){
	int cix,aix,zix;
	for (cix = 0; cix<(NC-1);cix++){
		for (aix = 0; aix<(NANGLE-1);aix++){
			for (zix = 0; zix<(NZ-1);zix++){
				corr_invalid[cix][aix][zix] = 
					corr_invalid[cix  ][aix  ][zix  ] ||
					corr_invalid[cix+1][aix  ][zix  ] ||
					corr_invalid[cix  ][aix+1][zix  ] ||
					corr_invalid[cix+1][aix+1][zix  ] ||
					corr_invalid[cix  ][aix  ][zix+1] ||
					corr_invalid[cix+1][aix  ][zix+1] ||
					corr_invalid[cix  ][aix+1][zix+1] ||
					corr_invalid[cix+1][aix+1][zix+1];
			}
		}
	}

}

int generate_ray_bending_table_from_sv_file(char* fname,float sonar_depth, uint8_t generate_lut){
	sv_meas_t* sv_meas;
	sv_meas_t* sv_meas_filtered;
	float depth,sv,ddepth,dsv;
	int ix,ii;
	int sv_meas_len;

	sv_meas = malloc(MAX_SV_MEAS*sizeof(sv_meas_t));
	
    sv_meas_len = svp_read_from_file(fname, sv_meas, MAX_SV_MEAS);
    svp_auto_swap_sv_depth(sv_meas, sv_meas_len);
	
    #ifdef PYTHON_PRINTOUT
    if (1){
        // Python print input sv table
        FILE *ppfd = fopen("/tmp/wbms_georef_raytrace_read_debug.dump","w");
        fprintf(ppfd,"import numpy as np\n");
        fprintf(ppfd,"import matplotlib.pyplot as plt\n");
        fprintf(ppfd,"sv_raw=np.asarray([");
        for(size_t ix=0;ix<sv_meas_len;ix++){
            fprintf(ppfd,"[%6.2f, %7.2f],",sv_meas[ix].depth, sv_meas[ix].sv);
        }
        fprintf(ppfd,"])\n");
        fclose(ppfd);
    }
	#endif
    
    sv_meas_len = svp_extrapolate_sv_table(sv_meas, sv_meas_len, MAX_SV_MEAS);

    sv_meas_len = svp_filter_and_resample(sv_meas, sv_meas_len, &sv_meas_filtered);
    free(sv_meas);
    sv_meas = sv_meas_filtered;
    
	
    #ifdef PYTHON_PRINTOUT
    if(1){
        // Python print input sv table
        FILE *ppfd = fopen("/tmp/wbms_georef_raytrace_read_debug.dump","a");
        fprintf(ppfd,"sv_filt=np.asarray([");
        for(size_t ix=0;ix<sv_meas_len;ix++){
            fprintf(ppfd,"[%6.2f, %7.2f],",sv_meas[ix].depth, sv_meas[ix].sv);
        }
        fprintf(ppfd,"])\n");
        fprintf(ppfd,"plt.figure('profile')\n");
        fprintf(ppfd,"plt.plot(sv_raw[:,1],  -sv_raw[:,0]);plt.plot(sv_filt[:,1],  -sv_filt[:,0])\n");
        fclose(ppfd);
    }
	#endif
	
	fprintf(stderr, "Correcting SV data for sonar_depth = %fm\n", sonar_depth);
	//Subtract sonar depth, to make SV measurements relative to sonar and not surface
	for (ii = 0;ii<sv_meas_len;ii++){
		sv_meas[ii].depth -= sonar_depth;
	}

    float max_depth = 0;
	for (ii = 0;ii<sv_meas_len;ii++){
		max_depth = max_depth > sv_meas[ii].depth ? max_depth:sv_meas[ii].depth;
	}
	fprintf(stderr, "Max depth in SV profile file = %f\n", max_depth);
    DZ = 1.5*max_depth/NZ; // Extrapolating SV profile to 150% of measured data


	fprintf(stderr, "Resample sv readings to sv table 0 - %fm dz=%f\n", NZ*DZ,DZ);
	for(ix=0;ix<NZ;ix++){
		depth = (float)ix*DZ;
		for (ii=0;(sv_meas[ii].depth<depth)&&(ii<sv_meas_len);ii++){}	//Find index of first entry in sv_meas table with depth greater than target depth
		if (ii==0) sv = sv_meas[ii].sv;							//Before first entry, use first
		else if (ii==sv_meas_len) sv = sv_meas[sv_meas_len-1].sv;			//After last entry, use last
		else{													//Else interpolate
			ddepth = sv_meas[ii].depth-sv_meas[ii-1].depth;
			dsv = sv_meas[ii].sv-sv_meas[ii-1].sv;
			sv = sv_meas[ii-1].sv+(dsv/ddepth)*(depth-sv_meas[ii-1].depth); //Linear interpolation of sound 
		}
		sv_table[ix] = sv;
	}
	free(sv_meas);
	#if 0
	// ASCII plot resampled sv table
	for(ix=0;ix<NZ;ix++){
		fprintf(stderr,"%3d %6.2f %7.2f",ix,ix*DZ, sv_table[ix]);
		for (ii=0;ii<(sv_table[ix]-1450);ii++) fprintf(stderr,"*");
		fprintf(stderr,"\n");
	}
	#endif
	#ifdef PYTHON_PRINTOUT
    if(1){
        FILE *ppfd = fopen("/tmp/wbms_georef_raytrace_read_debug.dump","a");
        // Python print resampled sv table
        fprintf(ppfd,"sv_resamp=np.asarray([");
        for(ix=0;ix<NZ;ix++){
            fprintf(ppfd,"[%6.2f, %7.2f],",ix*DZ, sv_table[ix]);
        }
        fprintf(ppfd,"])\n");
        fclose(ppfd);
    }
	#endif
	
    if (generate_lut){
        //Index of initial sound velocity, initial angle and true depth in table
        int cix, aix, zix ; 
        //Value of initial/observed sound velocity and angle in table
        double c,a_obs;
        //Value of ray parameter
        double xi;
        //dr (horizontal distance) per depth layer 
        float dr[NZ];
        //dt (propagation time) per depth layer 
        float dt[NZ];
        //Mark if respective dr and/or dt is invalid
        int	invalid[NZ];

        for(cix = 0;cix<NC;cix++){
            c = MINIMUM_C+ (double)cix*DC;
            for(aix = 0;aix<NANGLE;aix++){
                a_obs = (double)aix*DANGLE;
                xi = cos(a_obs)/c;				//Ray parameter for beam, based on initial pointing angle and sound velocity
                //First we calculate dr and dt for every z-layer for given beam
                for(zix = 0;zix<(NZ-1);zix++)
                    invalid[zix] = calc_dr_dt_rev_hovem(xi,sv_table[zix],sv_table[zix+1],DZ,&(dr[zix]), &(dt[zix]));

                corr_invalid[cix][aix][0] = 0;
                corr_range[cix][aix][0] = 0;
                corr_angle[cix][aix][0] = 0;
                // r_true (true horizontal distance for a sounding, bouncing of at true depth given by index
                double r_true = 0;
                // t_obs (observed time delay for a sounding, bouncing of at true depth given by index 
                double t_obs = 0;
                int prev_zobs_ix = 0;
                for(zix = 1;zix<NZ;zix++){      //Iterate over true depth 
                    //Accumulate dr to get true horizontal range
                    r_true += dr[zix-1];
                    //Accumulate dt to get observed one way travel time
                    t_obs += dt[zix-1];
                    double z_true = zix*DZ;
                    // d_true, a_true (true distance and angle for a sounding, bouncing of at true depth given by index
                    double d_true = sqrt(r_true*r_true + z_true*z_true);
                    double a_true = atan2(z_true,r_true);
                    // d_obs, observed distance to sounding, based on observed sound velocity, and observed travel time
                    double d_obs = c * t_obs;
                    // z_obs, obseved depth of sounding
                    double z_obs = d_obs*sin(a_obs);
                    //zix, is index of true z value, while what we are going to do lookup with, is observed z value. So we need reindex 
                    int zobs_ix = (int) round(z_obs/DZ);
                    zobs_ix = LIMIT(zobs_ix,0,NZ-1);
                    //Fill out all indexes from previous to this with calculated corrections (to avoid gaps)
                    for (int zz=prev_zobs_ix; zz<=zobs_ix; zz++){
                        //Table of range correction values, relative values corr = (true-observed)/observed
                        corr_range[cix][aix][zz] = (d_true-d_obs)/d_obs;
                        //Table of angle correction values indexed by initial sound velocity, initial/obseved direction and true depth (true-observed)
                        corr_angle[cix][aix][zz] = (a_true-a_obs);
                        //fprintf(stderr,"%5.3f ",corr_angle[cix][aix][zix]*180/M_PI);
                        corr_invalid[cix][aix][zz] = invalid[zix];
                        prev_zobs_ix=zobs_ix+1;
                    }
                }
                // For the remaining part, the zobs not covered by ztrue, just fill in with last value
                for (int zz=zz=prev_zobs_ix;zz<NZ; zz++){
                        corr_range[cix][aix][zz] = corr_range[cix][aix][prev_zobs_ix-1];
                        corr_angle[cix][aix][zz] = corr_angle[cix][aix][prev_zobs_ix-1];
                        corr_invalid[cix][aix][zz] = 1;//corr_invalid[cix][aix][prev_zobs_ix-1]; //TODO This (literal) corner case, could perhaps just be set to extrapolate the valid flag as well, now it is just set to invalid. 
                }
            }
        }
        dilate_invalid_table_for_interpolation();
    }
	
    #ifdef PYTHON_PRINTOUT
    if(1){
        FILE *ppfd = fopen("/tmp/wbms_georef_raytrace_read_debug.dump","a");
        // Python print resampled sv table
        fprintf(ppfd,"import numpy as np\n");
        fprintf(ppfd,"import matplotlib.pyplot as plt\n");
        fprintf(ppfd,"c=np.asarray([");
        int cix_sel = round( (1480.-MINIMUM_C)/DC);
        for(int cix = cix_sel;cix<=cix_sel;cix++){
            fprintf(ppfd,"%6.2f,", MINIMUM_C+ (float)cix*DC);
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"z=np.asarray([");
        for(int zix = 1;zix<NZ;zix++){
            fprintf(ppfd,"%6.2f,", (float)zix*DZ);
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"angle=np.asarray([");
        for(int aix = 0;aix<NANGLE;aix++){
            fprintf(ppfd,"%6.3f,", (float)aix*DANGLE);
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"corr_angle=np.asarray([");
        for(int cix = cix_sel;cix<=cix_sel;cix++){
            fprintf(ppfd,"[");
            for(int aix = 0;aix<NANGLE;aix++){
                fprintf(ppfd,"[");
                for(int zix = 1;zix<NZ;zix++){      //Iterate over true depth 
                    fprintf(ppfd,"%5.3f,",corr_angle[cix][aix][zix]);
                }
                fprintf(ppfd,"],");
            }
            fprintf(ppfd,"],");
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"corr_range=np.asarray([");
        for(int cix = cix_sel;cix<=cix_sel;cix++){
            fprintf(ppfd,"[");
            for(int aix = 0;aix<NANGLE;aix++){
                fprintf(ppfd,"[");
                for(int zix = 1;zix<NZ;zix++){      //Iterate over true depth 
                    fprintf(ppfd,"%5.3f,",corr_range[cix][aix][zix]);
                }
                fprintf(ppfd,"],");
            }
            fprintf(ppfd,"],");
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"corr_invalid=np.asarray([");
        for(int cix = cix_sel;cix<=cix_sel;cix++){
            fprintf(ppfd,"[");
            for(int aix = 0;aix<NANGLE;aix++){
                fprintf(ppfd,"[");
                for(int zix = 1;zix<NZ;zix++){      //Iterate over true depth 
                    fprintf(ppfd,"%d,",corr_invalid[cix][aix][zix]);
                }
                fprintf(ppfd,"],");
            }
            fprintf(ppfd,"],");
        }
        fprintf(ppfd,"])\n");

        fprintf(ppfd,"## Angles here are in radians relative to horiziont. We want degrees relative to nadir, as that os the 'normal' convention used here.\n");
        fprintf(ppfd,"angle= 90 - np.rad2deg(angle[::-1])\n");
        fprintf(ppfd,"corr_angle= np.rad2deg(-corr_angle[:,::-1,:])\n");
        fprintf(ppfd,"corr_range= corr_range[:,::-1,:]\n");
        fprintf(ppfd,"corr_invalid= corr_invalid[:,::-1,:]\n");

        fprintf(ppfd,"plt.figure('table')\n");
        fprintf(ppfd,"ax1=plt.subplot(3,1,1)\n");
        fprintf(ppfd,"ax1.imshow(corr_angle[0,:,:].T,extent=[angle[0],angle[-1],-z[-1],-z[0]],aspect='auto',cmap='seismic',vmin=-5, vmax=5)\n");
        fprintf(ppfd,"ax1.set_title('Angle correction c=%%0.2fm/s'%%(c[0]))\n");
        fprintf(ppfd,"ax2=plt.subplot(3,1,2)\n");
        fprintf(ppfd,"ax2.imshow(corr_range[0,:,:].T,extent=[angle[0],angle[-1],-z[-1],-z[0]],aspect='auto',cmap='seismic',vmin=-0.1, vmax=0.1)\n");
        fprintf(ppfd,"ax2.set_title('Range correction')\n");
        fprintf(ppfd,"ax3=plt.subplot(3,1,3)\n");
        fprintf(ppfd,"ax3.imshow(corr_invalid[0,:,:].T,extent=[angle[0],angle[-1],-z[-1],-z[0]],aspect='auto',cmap='binary',vmin=0, vmax=1)\n");
        fprintf(ppfd,"ax3.set_title('Correction valid')\n");
        fprintf(ppfd,"plt.show()\n");

        fclose(ppfd);
    }
	#endif

	has_corr = 1;
	return 0;
}

//Calculate ray bending directly, by processing every beam without LUT lookup
// Doing this we can take into acount that the sonar is not at (or close to) 0 meter depth when doing raytracing
// Setting input SoS (c==0), will use sv from table at sonar depth instead of measured sv for ray tracing
void apply_ray_bending_direct(float* X,float* Y,float* Z,int N, float c, float z0){
	int n;
	float x,y,z,a_obs,r_obs,d_obs,t_obs,t;
	float r_true,z_true;
	float xi;
	float xc,yc,zc;
	int zix;
	float dr, dt;
	int invalid;
	float overshoot;
	if(has_corr==0) return;		//Dont do anything if correction table has not been calculated
	
	//printf("Doing direct raytracing z0 = %f\n",z0);
	
	for (n = 0;n<N;n++){
		x = X[n];
		y = Y[n];
		z = Z[n];
		
		//Transform into vertical and horizontal distance, 
		r_obs = sqrtf(x*x+y*y);					//Calculate observed horizontal distance
		d_obs = sqrtf(x*x+y*y+z*z);				//Calculate observed total distance
		a_obs = atan2f(z,r_obs);						//Calculate observed beam angle (relative to horizontal)
		t_obs = d_obs / c;							//Calculate observed one way travel time
		if (t_obs==0) continue;


		//Now we start with horizontal and vertical distance = 0 (all relative to sonar) and trace the beam until it has propagated more than the observed one way travel time
		t = 0;
		r_true = 0;
		z_true = 0; 	
		zix = roundf(z0/DZ);				//Start with sound velocity measured at depth equal to sonar depth
		zix = LIMIT(zix, 0,NZ-1);
        //Find initial ray parameter
        if (c==0){
            c = sv_table[0];				//If input c is 0, use c from sv_table at sonar depth (always index 0) instead 
        }
        xi = cosf(a_obs)/c;				//Ray parameter for beam, based on initial pointing angle and sound velocity
		
        do{
			invalid = calc_dr_dt_rev_hovem(xi,sv_table[zix],sv_table[zix+1],DZ,&dr, &dt);
			if (invalid) break;
			t += dt;
			r_true += dr;
			z_true += DZ;
			zix = MIN(zix+1,NZ-1);
		} while(t<t_obs);

		if (invalid){ 
			xc=0;yc=0;zc=0;					//Invalid soundings are placed in origo
			ray_bend_invalid++;
		}
		else{
			//Then, subtract overshoot
			overshoot = (t-t_obs)/dt;
			r_true -= dr*overshoot;
			z_true -= DZ*overshoot;

			//Convert back to kartesian
			zc = z_true;
			xc = x*(r_true/r_obs);
			yc = y*(r_true/r_obs);
			ray_bend_valid++;
		}
		X[n] = xc;
		Y[n] = yc;
		Z[n] = zc;
		//if (ray_bend_valid%64==0) printf("x=%7.3f -> %7.3f, y=%7.3f -> %7.3f ,z=%7.3f -> %7.3f\n", x,xc,y,yc,z,zc); 
		//if (ray_bend_valid%64==0) printf("z=%7.3f -> %7.3f    %7.3f\n", z,zc, zc-z); 
	}
	//printf("Valid = %10d, Invalid = %10d\n",ray_bend_valid,ray_bend_invalid);
}

#ifndef INTERPOL_COR_TABLE
//Apply ray bending corrections to coordinates in place
// Setting input SoS (c==0), will use sv from table at sonar depth instead of measured sv for ray tracing
void apply_ray_bending(float* X,float* Y,float* Z,int N, float c){
	int n;
	float x,y,z,a,r,d;
	float xc,yc,zc,ac,rc,dc;
	int cix,aix,zix;
	if(has_corr==0) return;		//Dont do anything if correction table has not been calculated
    if (c==0){
        c = sv_table[0];				//If input c is 0, use c from sv_table at sonar depth (always index 0) instead 
	}

    cix = roundf((c - MINIMUM_C)/DC); 	//Index along sound velocity axis in correction table
	
	for (n = 0;n<N;n++){
		x = X[n];
		y = Y[n];
		z = Z[n];
		
		r = sqrt(x*x+y*y);					//Calculate observed horizontal distance
		d = sqrt(x*x+y*y+z*z);				//Calculate observed total distance
		a = atan2(z,r);						//Calculate observed beam angle (relative to horizontal)
		aix = roundf(a/DANGLE); 		    //Index along angle axis in correction table
		zix = roundf(z/DZ);				    //Index	along depth axis in correction table
		
		if (corr_invalid[cix][aix][zix]){ 
			xc=0;yc=0;zc=0;					//Invalid soundings are placed in origo
			ray_bend_invalid++;
		}
		else{
			ac = a+corr_angle[cix][aix][zix];		//Calculate corrected beam angle
			dc = d + d*corr_range[cix][aix][zix];	//Calculate corrected beam range
			zc = dc*sin(ac);
			rc = dc*cos(ac);
			xc = x*(rc/r);
			yc = y*(rc/r);
			ray_bend_valid++;
			//if (ray_bend_valid%128==0) printf("a=%7.3f -> %7.3f, d=%7.3f -> %7.3f\n", a*180/M_PI,ac*180/M_PI,d,dc); 
		}
		X[n] = xc;
		Y[n] = yc;
		Z[n] = zc;
		//if (ray_bend_valid%64==0) printf("x=%7.3f -> %7.3f, y=%7.3f -> %7.3f ,z=%7.3f -> %7.3f\n", x,xc,y,yc,z,zc); 
	}
	//printf("Valid = %10d, Invalid = %10d\n",ray_bend_valid,ray_bend_invalid);
}
#else

// Setting input SoS (c==0), will use sv from table at sonar depth instead of measured sv for ray tracing
void apply_ray_bending(float* X,float* Y,float* Z,int N, float c){
	int n;
	float x,y,z,a,r,d;
	float xc,yc,zc,ac,rc,dc;
	int cix,aix,zix;
	float fcix0, faix0, fzix0;
	float fcix1, faix1, fzix1;
	float c_angle, c_range;
	if(has_corr==0) return;		//Dont do anything if correction table has not been calculated
    if (c==0){
        c = sv_table[0];				//If input c is 0, use c from sv_table at sonar depth (always index 0) instead 
    }
	
    fcix1 = (c - MINIMUM_C)/DC; 	//Index along sound velocity axis in correction table
	cix = (int) floorf(fcix1); cix = LIMIT(cix,0,NC-2);fcix1 -= (float)cix; fcix0=1.f-fcix1;
	
	for (n = 0;n<N;n++){
		x = X[n];
		y = Y[n];
		z = Z[n];
		
		r = sqrtf(x*x+y*y);					//Calculate observed horizontal distance
		d = sqrtf(x*x+y*y+z*z);				//Calculate observed total distance
		a = atan2f(z,r);						//Calculate observed beam angle (relative to horizontal)
	
		faix1 = a/DANGLE; 	
		aix = (int) floorf(faix1); aix = LIMIT(aix,0,NANGLE-2);faix1 -= (float)aix; faix0=1.f-faix1;
		fzix1 = z/DZ; 	
		zix = (int) floorf(fzix1); zix = LIMIT(zix,0,NZ-2);fzix1 -= (float)zix; fzix0=1.f-fzix1;
		
		if (corr_invalid[cix][aix][zix]){ 
			xc=0;yc=0;zc=0;					//Invalid soundings are placed in origo
			ray_bend_invalid++;
		}
		else{
			//3D linear interpolation
			c_angle = 
				corr_angle[cix  ][aix  ][zix  ] * (fcix0*faix0*fzix0) + 
				corr_angle[cix+1][aix  ][zix  ] * (fcix1*faix0*fzix0) + 
				corr_angle[cix  ][aix+1][zix  ] * (fcix0*faix1*fzix0) + 
				corr_angle[cix+1][aix+1][zix  ] * (fcix1*faix1*fzix0) + 
				corr_angle[cix  ][aix  ][zix+1] * (fcix0*faix0*fzix1) + 
				corr_angle[cix+1][aix  ][zix+1] * (fcix1*faix0*fzix1) + 
				corr_angle[cix  ][aix+1][zix+1] * (fcix0*faix1*fzix1) + 
				corr_angle[cix+1][aix+1][zix+1] * (fcix1*faix1*fzix1);
			c_range = 
				corr_range[cix  ][aix  ][zix  ] * (fcix0*faix0*fzix0) + 
				corr_range[cix+1][aix  ][zix  ] * (fcix1*faix0*fzix0) + 
				corr_range[cix  ][aix+1][zix  ] * (fcix0*faix1*fzix0) + 
				corr_range[cix+1][aix+1][zix  ] * (fcix1*faix1*fzix0) + 
				corr_range[cix  ][aix  ][zix+1] * (fcix0*faix0*fzix1) + 
				corr_range[cix+1][aix  ][zix+1] * (fcix1*faix0*fzix1) + 
				corr_range[cix  ][aix+1][zix+1] * (fcix0*faix1*fzix1) + 
				corr_range[cix+1][aix+1][zix+1] * (fcix1*faix1*fzix1);
			ac = a+c_angle;		//Calculate corrected beam angle
			dc = d + d*c_range;	//Calculate corrected beam range
			zc = dc*sinf(ac);
			rc = dc*cosf(ac);
			xc = x*(rc/r);
			yc = y*(rc/r);
			ray_bend_valid++;
			//if (ray_bend_valid%128==0) printf("a=%7.3f -> %7.3f, d=%7.3f -> %7.3f\n", a*180/M_PI,ac*180/M_PI,d,dc); 
		}
		X[n] = xc;
		Y[n] = yc;
		Z[n] = zc;
		//if (ray_bend_valid%64==0) printf("x=%7.3f -> %7.3f, y=%7.3f -> %7.3f ,z=%7.3f -> %7.3f\n", x,xc,y,yc,z,zc); 
	}
	//printf("Valid = %10d, Invalid = %10d\n",ray_bend_valid,ray_bend_invalid);
}
#endif

