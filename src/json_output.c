#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "georef_tools.h"
#include "time_functions.h"
#include "wbms_georef.h"
#include <math.h>
#include "cmath.h"
#include "json_output.h"


/*
{
  "file_type": "xyz",
  "file_version": "1.2",
  "data_type": "depth",
  "sensor_type": "multibeam",
  "has_navigation": true,
  "datapoints": 5423,
  "start_time": 1713091825.123,
  "duration": 365.2,
  "latitude": 59.9139,
  "longitude": 10.7522,
  "altitude": 3.0,
  "line_length": 402.5,
  "start_stop_distance": 1.1,
  "mean_depth": 83.7,
  "coverage_area": 1250.0,
  "records": [[101, 13], [102, 8], [103, 2]]
}


*/
int write_stats_json_to_buffer(file_stats_t* stats, /*OUTPUT*/char* outbuf){
    int len=0;
    
    len += sprintf(&(outbuf[len]),"{\n");

    if(stats->file_type)    {    len += sprintf(&(outbuf[len]),"\t\"file_type\": \"%s\",\n",         stats->file_type);}
    if(stats->file_version) {    len += sprintf(&(outbuf[len]),"\t\"file_version\": \"%s\",\n",      stats->file_version);}
    if(stats->data_type)    {    len += sprintf(&(outbuf[len]),"\t\"data_type\": \"%s\",\n",         stats->data_type);}
    if(stats->sensor_type)  {    len += sprintf(&(outbuf[len]),"\t\"sensor_type\": \"%s\",\n",       stats->sensor_type);}
                            {    len += sprintf(&(outbuf[len]),"\t\"has_navigation\": %s,\n",        stats->has_navigation?"true":"false");}
                            {    len += sprintf(&(outbuf[len]),"\t\"has_sensor\": %s,\n",            stats->has_sensor?"true":"false");}
                            {    len += sprintf(&(outbuf[len]),"\t\"has_svp\": %s,\n",               stats->has_svp?"true":"false");}
    if(stats->datapoints)   {    len += sprintf(&(outbuf[len]),"\t\"datapoints\": %d,\n",            stats->datapoints);}
    if(stats->datasets)     {    len += sprintf(&(outbuf[len]),"\t\"datasets\": %d,\n",              stats->datasets);}
    if(stats->navigation_points)   {    len += sprintf(&(outbuf[len]),"\t\"navigation_points\": %d,\n",            stats->navigation_points);}
    if(stats->start_time)   {    len += sprintf(&(outbuf[len]),"\t\"start_time\": %.3f,\n",          stats->start_time);}
    if(stats->duration)     {    len += sprintf(&(outbuf[len]),"\t\"duration\": %.3f,\n",            stats->duration);}
    if(stats->latitude)     {    len += sprintf(&(outbuf[len]),"\t\"latitude\": %.6f,\n",            stats->latitude*180/M_PI);}
    if(stats->longitude)    {    len += sprintf(&(outbuf[len]),"\t\"longitude\": %.6f,\n",           stats->longitude*180/M_PI);}
    if(stats->altitude)     {    len += sprintf(&(outbuf[len]),"\t\"altitude\": %.3f,\n",            stats->altitude);}
    
    if(!(stats->freq==0))     {    len += sprintf(&(outbuf[len]),"\t\"frequency\": %.3f,\n",            stats->freq);}
    if(!(stats->bandwidth==0))     {    len += sprintf(&(outbuf[len]),"\t\"bandwidth\": %.3f,\n",            stats->bandwidth);}

    if(stats->line_length)  {    len += sprintf(&(outbuf[len]),"\t\"line_length\": %.3f,\n",         stats->line_length);}
    if(stats->start_stop_distance){    len += sprintf(&(outbuf[len]),"\t\"start_stop_distance\": %.3f,\n", stats->start_stop_distance);}
    if(stats->num_record_types){
        len += sprintf(&(outbuf[len]),"\t\"records\": [");
        for (int ix=0;ix<stats->num_record_types;ix++){
            len += sprintf(&(outbuf[len]),"[%d, %d],",stats->records[ix].type,stats->records[ix].count);
        }
        len -= 1;
        len += sprintf(&(outbuf[len]),"],\n");
    }

    
    len -= 2; //Remove the last ",\n" from the JSON dict
    len += sprintf(&(outbuf[len]),"\n}\n");
    return len;
}

int write_json_to_buffer(double ts, output_data_t* data,uint32_t n, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, aux_navdata_t *aux_navdata,output_format_e format[MAX_OUTPUT_FIELDS], /*OUTPUT*/char* outbuf){
    double* x_val = &(data->x[0]);
    double* y_val = &(data->y[0]);
    double* z_val = &(data->z[0]);
    float* z_var_val = &(data->z_var[0]);
    float* intensity_val = &(data->i[0]);
    float* quality_val = &(data->quality[0]);
    float* strength_val = &(data->strength[0]);
    float* range_val = &(data->range[0]);
    float* teta_val = &(data->teta[0]);
    float* steer_val = &(data->steer[0]);
    int * beam_number = &(data->beam[0]);
    float* swath_y_val = &(data->swath_y[0]);
    float* aoi_val = &(data->aoi[0]);
    float* up_gate = &(data->up_gate[0]);
    float* low_gate = &(data->low_gate[0]);
    float* tx_angle = &(data->tx_angle);
    float* sv = &(data->sv);
    float* tx_freq = &(data->tx_freq);
    int*   multiping_index = &(data->multiping_index);
    int*   multifreq_index = &(data->multifreq_index);
    int*   ping_number = &(data->ping_number);
    float*   ping_rate = &(data->ping_rate);

    uint32_t ii,jj,len;
    len = 0;
    pos_ix = find_closest_index_in_posdata(posdata,pos_ix, ts);
    navdata_t* pos = &(posdata[pos_ix]);

    //Calculate vessel horizontal speed
    navdata_t* pos_old = &(posdata[(pos_ix+NAVDATA_BUFFER_LEN-10)%NAVDATA_BUFFER_LEN]); //Navdata 10 samples back in time TODO there is a slight risk of posdata log wrapping when doing this
    float dx = pos->x - pos_old->x;
    float dy = pos->y - pos_old->y;
    float dt = pos->ts - pos_old->ts;
    float speed = sqrtf(dx*dx+dy*dy)/dt;
	

    //When writing, we need to swap x and y coordinate and negate z since output is X-east Y-north Z-up

    len += sprintf(&(outbuf[len]),"{"); 
    //Write navigation data to JSON (one value per dataset) 
    for(jj=0; jj<MAX_OUTPUT_FIELDS; jj++){
        if (format[jj]==none) break;
        if(jj)  len += sprintf(&(outbuf[len]),", ");
        switch(format[jj]){
            case none: default: break;
            // Per ping values (scalars)
            case LAT: len += sprintf(&(outbuf[len]),"\"latitude\":%1.7f",pos->lat*180/M_PI);break;
            case LON: len += sprintf(&(outbuf[len]),"\"longitude\":%1.7f",pos->lon*180/M_PI);break;
            case X: len += sprintf(&(outbuf[len]),"\"X\":%1.3f",pos->y);break;
            case Y: len += sprintf(&(outbuf[len]),"\"Y\":%1.3f",pos->x);break;
            case Z: len += sprintf(&(outbuf[len]),"\"Z\":%1.3f",-pos->z);break;
            case ALTITUDE: len += sprintf(&(outbuf[len]),"\"altitude\":%1.3f",-pos->z);break;
            case HOR_ACC: len += sprintf(&(outbuf[len]),"\"hdop\":%1.3f",aux_navdata->hor_accuracy);break;
            case VERT_ACC: len += sprintf(&(outbuf[len]),"\"vdop\":%1.3f",aux_navdata->vert_accuracy);break;
            case YAW: len += sprintf(&(outbuf[len]),"\"yaw\":%1.3f",pos->yaw*180/M_PI);break;
            case PITCH: len += sprintf(&(outbuf[len]),"\"pitch\":%1.3f",pos->pitch*180/M_PI);break;
            case ROLL: len += sprintf(&(outbuf[len]),"\"roll\":%1.3f",pos->roll*180/M_PI);break;
            case SPEED: len += sprintf(&(outbuf[len]),"\"speed\":%1.3f",speed);break;
            case MOVEMENT: len += sprintf(&(outbuf[len]),"\"movement\":%1.3f",speed/(*ping_rate));break;
            case COURSE: len += sprintf(&(outbuf[len]),"\"course\":%1.3f",pos->course*180/M_PI);break; 
            case GPS_ACCURACY: len += sprintf(&(outbuf[len]),"\"gps_accuracy\":%1.3f",aux_navdata->hor_accuracy);break; //TODO
            case GPS_STATUS: len += sprintf(&(outbuf[len]),"\"gps_status\":%1d",aux_navdata->gps_status);break; //TODO
            case SATELLITES: len += sprintf(&(outbuf[len]),"\"satellites\":%1d",aux_navdata->sv_n);break; //TODO*/
            case t: len += sprintf(&(outbuf[len]),"\"ts\":%12.5f",ts);break;
            case c: len += sprintf(&(outbuf[len]),"\"sound_velocity\":%8.3f",*sv);break;
            case pingrate: len += sprintf(&(outbuf[len]),"\"ping_rate\":%8.3f",*ping_rate);break;
            case pingnumber: len += sprintf(&(outbuf[len]),"\"ping_number\":%1d",*ping_number);break;
            case el: len += sprintf(&(outbuf[len]),"\"tx_angle\":%1.3f",*tx_angle*180/M_PI);break;
            case freq: len += sprintf(&(outbuf[len]),"\"freq\":%1.3f",*tx_freq);break;
            case multiping: len += sprintf(&(outbuf[len]),"\"multiping\":%1d",*multiping_index);break;
            case multifreq: len += sprintf(&(outbuf[len]),"\"multifreq\":%1d",*multifreq_index);break;
                                  
            // Per sounding values (arrays)
            case line_of_sight: 
                        len += sprintf(&(outbuf[len]),"\"line_of_sight\":[");
                        len += sprintf(&(outbuf[len]),"\"x\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",y_val[ii]-pos->y);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",y_val[ii]-pos->y);
                        len += sprintf(&(outbuf[len]),"\"y\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",x_val[ii]-pos->x);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",x_val[ii]-pos->x);
                        len += sprintf(&(outbuf[len]),"\"z\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",-(z_val[ii]-pos->z));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",-(z_val[ii]-pos->z));
                        len += sprintf(&(outbuf[len]),"]");
                        break;
            case x: 
                        len += sprintf(&(outbuf[len]),"\"x\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",y_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",y_val[ii]);
                        break;
            case y: 
                        len += sprintf(&(outbuf[len]),"\"y\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",x_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",x_val[ii]);
                        break;
            case z: 
                        len += sprintf(&(outbuf[len]),"\"z\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",-z_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",-z_val[ii]);
                        break;
            case z_var: 
                        len += sprintf(&(outbuf[len]),"\"z_var\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",z_var_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",z_var_val[ii]);
                        break;
            case z_stddev: 
                        len += sprintf(&(outbuf[len]),"\"z_stddev\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",sqrtf(z_var_val[ii]));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",sqrtf(z_var_val[ii]));
                        break;
            case teta: 
                        len += sprintf(&(outbuf[len]),"\"teta\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",teta_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",teta_val[ii]);
                        break;
            case beam: 
                        len += sprintf(&(outbuf[len]),"\"beam\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1d,",beam_number[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1d]",beam_number[ii]);
                        break;
            case range: 
                        len += sprintf(&(outbuf[len]),"\"range\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",range_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",range_val[ii]);
                        break;
            case steer: 
                        len += sprintf(&(outbuf[len]),"\"steer\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",steer_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",steer_val[ii]);
                        break;
            case val: 
                        len += sprintf(&(outbuf[len]),"\"intensity\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",intensity_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",intensity_val[ii]);
                        break;
            case quality: 
                        len += sprintf(&(outbuf[len]),"\"quality\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",quality_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",quality_val[ii]);
                        break;
            case strength: 
                        len += sprintf(&(outbuf[len]),"\"strength\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",strength_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",strength_val[ii]);
                        break;
            case swath_y: 
                        len += sprintf(&(outbuf[len]),"\"swath_y\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",swath_y_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",swath_y_val[ii]);
                        break;
            case aoi: 
                        len += sprintf(&(outbuf[len]),"\"aoi\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",aoi_val[ii]);
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",aoi_val[ii]);
                        break;
            case cgate: 
                        len += sprintf(&(outbuf[len]),"\"cgate\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",0.5*(low_gate[ii]+up_gate[ii]));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",0.5*(low_gate[ii]+up_gate[ii]));
                        break;
            case ugate: 
                        len += sprintf(&(outbuf[len]),"\"ugate\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",(up_gate[ii]));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",(up_gate[ii]));
                        break;
            case lgate: 
                        len += sprintf(&(outbuf[len]),"\"lgate\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",(low_gate[ii]));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",(low_gate[ii]));
                        break;
            case gatew: 
                        len += sprintf(&(outbuf[len]),"\"gatew\":[");
                        for (ii = 0;ii<(n-1);ii++){
                            len += sprintf(&(outbuf[len]),"%1.3f,",(low_gate[ii]-up_gate[ii]));
                        }
                        len += sprintf(&(outbuf[len]),"%1.3f]",(low_gate[ii]-up_gate[ii]));
                        break;
        }
    }


    len += sprintf(&(outbuf[len]),"}\n");

    return len;
}

