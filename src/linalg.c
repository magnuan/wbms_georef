#include <stdio.h>
#include <fcntl.h>
//#include <sys/ioctl.h>
//#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <math.h>


#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define ABS(x) (((x)>0)?(x):-(x))


void dot3x3(float a[][3], float b[][3], float result[][3]){
	int ii, jj, kk;
	for(ii = 0; ii < 3; ii++){
		for(jj = 0; jj < 3; jj++){
			result[ii][jj]=0;
		}
	}
	for(ii = 0; ii < 3; ii++){
		for(jj = 0; jj < 3; jj++){
			for(kk = 0; kk < 3; kk++){
				result[ii][jj] +=  a[ii][kk] *  b[kk][jj];
			}
		}
	}
}

void print3x3(float a[][3]){
	int ii, jj;
	for (ii = 0; ii < 3; ii++){
		for(jj = 0; jj < 3; jj++){
			fprintf(stderr,"%f\t", a[ii][jj]);
		}
		fprintf(stderr,"\n");
	}
}

#if 1
// ROLL AROUND X, PITCH AROUND Y, YAW AROUND POS Z
//Generate rotation matrix for given yaw-pith-roll rotation
void rot_matrix(float y,float p,float r, /*output*/ float result[][3]){
	float Rx[3][3] ={{1.f,0.f,0.f},{0.f,cosf(r),-sinf(r)},{0.f,sinf(r),cosf(r)}};     // Roll rotates around x axis
	float Ry[3][3] ={{cosf(p),0,sinf(p)},{0.f,1.f,0.f},{-sinf(p),0.f,cosf(p)}};     // Pitch rotates around y axis 
	float Rz[3][3] ={{cosf(y),-sinf(y),0.f},{sinf(y),cosf(y),0.f},{0.f,0.f,1.f}};     // Yaw rotates around pos z axis
	float tmp[3][3];
	dot3x3(Rz,Ry,tmp);
	dot3x3(tmp,Rx,result);                                              //Rotation order Yaw(Z), Pitch(Y), Roll(X)
}

//Rotation matrix derivated with respect to roll
void rot_matrix_d_dr(float y,float p,float r, /*output*/ float result[][3]){
	float dRx_dr[3][3] = {{0.f,0.f,0.f},{0.f,-sinf(r),-cosf(r)},{0.f,cosf(r),-sinf(r)}};
	float Ry[3][3] ={{cosf(p),0.f,sinf(p)},{0.f,1,0.f},{-sinf(p),0.f,cosf(p)}};
	float Rz[3][3] ={{cosf(y),-sinf(y),0.f},{sinf(y),cosf(y),0.f},{0.f,0.f,1.f}};
	float tmp[3][3];
	dot3x3(Rz,Ry,tmp);
	dot3x3(tmp,dRx_dr,result);
}
#else
// ROLL AROUND Y, PITCH AROUND X, YAW AROUND NEG Z
//Generate rotation matrix for given yaw-pith-roll rotation
void rot_matrix(float y,float p,float r, /*output*/ float result[][3]){
	float Ry[3][3] ={{cosf(r),0,sinf(r)},{0,1,0},{-sinf(r),0,cosf(r)}};         // Roll rotates around y axis 
	float Rx[3][3] ={{1,0,0},{0,cosf(p),-sinf(p)},{0,sinf(p),cosf(p)}};         // Pitch rotates around x axis
	float Rz[3][3] ={{cosf(-y),-sinf(-y),0},{sinf(-y),cosf(-y),0},{0,0,1}};     // Yaw rotates around pos z axis
	float tmp[3][3];
	dot3x3(Rz,Rx,tmp);
	dot3x3(tmp,Ry,result);                                              //Rotation order Yaw(Z), Pitch(X), Roll(Y)
}
void rot_matrix_d_dr(float y,float p,float r, /*output*/ float result[][3]){
	float dRy_dr[3][3] ={{-sinf(r),0,cosf(r)},{0,0,0},{-cosf(r),0,-sinf(r)}};           // Roll rotates around y axis 
    float Rx[3][3] ={{1,0,0},{0,cosf(p),-sinf(p)},{0,sinf(p),cosf(p)}};                 // Pitch rotates around x axis
	float Rz[3][3] ={{cosf(-y),-sinf(-y),0},{sinf(-y),cosf(-y),0},{0,0,1}};             // Yaw rotates around pos z axis
	
    float tmp[3][3];
	dot3x3(Rz,Rx,tmp);
	dot3x3(tmp,dRy_dr,result);
}
#endif





//Rotate n sets of xyz cordinates with R matrix 
void rot_coordinates(float* x_in,float* y_in,float* z_in,float R[][3],int n, /*output*/float* x_out, float* y_out, float* z_out){
	int ii;
	for (ii=0;ii<n;ii++){
		//x_out[ii] = x_in[ii]*R[0][0] + y_in[ii]*R[1][0] + z_in[ii]*R[2][0]; 
		//y_out[ii] = x_in[ii]*R[0][1] + y_in[ii]*R[1][1] + z_in[ii]*R[2][1]; 
		//z_out[ii] = x_in[ii]*R[0][2] + y_in[ii]*R[1][2] + z_in[ii]*R[2][2];
		x_out[ii] = x_in[ii]*R[0][0] + y_in[ii]*R[0][1] + z_in[ii]*R[0][2]; 
		y_out[ii] = x_in[ii]*R[1][0] + y_in[ii]*R[1][1] + z_in[ii]*R[1][2]; 
		z_out[ii] = x_in[ii]*R[2][0] + y_in[ii]*R[2][1] + z_in[ii]*R[2][2];
	}
}
