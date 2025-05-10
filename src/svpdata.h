#ifndef __SVPDATA_H__
#define __SVPDATA_H__

//Max number of depth values to read from sv-file
#define MAX_SV_MEAS  8192

typedef enum{svp_mode_none = 0, svp_mode_caris_v2=1, svp_mode_ascii_tuple=2, svp_mode_swift_svp=3} svp_mode_e;
typedef struct{
	float sv;
	float depth;
}sv_meas_t;

extern char *svp_mode_short_names[];

svp_mode_e svp_test_file(char* fname, double* ts, size_t* cnt);
int svp_read_from_file(char* fname, sv_meas_t* sv_meas_out, const size_t max_count);
int svp_read_from_ascii_tuplet_file(char* fname, sv_meas_t* sv_meas_out, const size_t max_count);
int svp_read_from_swift_svp_file(char* fname, sv_meas_t* sv_meas, const size_t max_count);
uint8_t svp_auto_swap_sv_depth(sv_meas_t* sv_meas_in, size_t count_in);
size_t  svp_extrapolate_sv_table(sv_meas_t* sv_meas_in, float extrapolate, size_t count_in, const size_t max_count);
size_t svp_filter_and_resample(sv_meas_t* sv_meas_in, size_t count_in, sv_meas_t** sv_meas_out);
size_t svp_discard_insane(sv_meas_t* sv_meas_in, size_t count_in);

#endif
