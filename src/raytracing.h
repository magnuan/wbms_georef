#ifndef __raytracing_h__
#define __raytracing_h__
void apply_ray_bending(float* X,float* Y,float* Z,int N, float c);
void apply_ray_bending_direct(float* X,float* Y,float* Z,int N, float c, float z0);
int generate_ray_bending_table_from_sv_file(const char* fname,float sonar_depth, uint8_t generate_lut,float c_min, float c_max);

int get_ray_bend_valid(void);
int get_ray_bend_invalid(void);
#endif
