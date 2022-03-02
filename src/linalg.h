#ifndef __linalg_h__
#define __linalg_h__

void dot3x3(float a[][3], float b[][3], float result[][3]);
void print3x3(float a[][3]);
void rot_matrix(float y,float p,float r, /*output*/ float result[][3]);
void rot_matrix_d_dr(float y,float p,float r, /*output*/ float result[][3]);
void rot_coordinates(float* x_in,float* y_in,float* z_in,float R[][3],int n, /*output*/float* x_out, float* y_out, float* z_out);

#endif
