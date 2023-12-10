#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
//#include <sys/time.h>
#include <fcntl.h>
//#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include "reson7k.h"
#include "wbms_georef.h"
#include <math.h>
#include "cmath.h"
#include "sonar_info_xml.h"

#define R7K_VERSION 5
#define R7K_DEV_ID 14001

static uint8_t recorded_not_live = 1;
static uint8_t write_network_frame = 0;
static uint32_t max_packet_size = 0;

void set_r7k_output_parameters(uint8_t rec, uint8_t net, uint32_t max){
    recorded_not_live = rec;
    write_network_frame = net;
    max_packet_size = max;
}

/* packet_size is payload + drf + checksum   i
 * =  payload_size + sizeof(r7k_DataRecordFrame) + sizeof(r7k_Checksum_t)
 * =  record_size - network_frame_header_size 
 * =  sizeof(r7k_Record_xxxx_t) + rd (variable length payload) - sizeof(r7k_NetworkFrame_t) 
 */
static void fill_out_network_frame(/*INPUT*/ size_t packet_size, /*OUTPUT*/ r7k_NetworkFrame_t* nf){
    static uint16_t trans_id = 1;
    nf->version = R7K_VERSION;
    nf->offset = sizeof(r7k_NetworkFrame_t);
	nf->tot_packets = 1;
	nf->tot_records = 1;
	nf->trans_id = trans_id++;
	nf->packet_size = packet_size;         //Size of this packet including network frame
	nf->tot_size = packet_size-sizeof(r7k_NetworkFrame_t);             //Size of all packets in transmission (excluding network frames)
	nf->seq_num = 0;                        //Sequential packet number (TODO: to be used if more packets in transmission
	nf->dest_dev_id = R7K_DEV_ID;
	nf->dest_dev_enum = 0;
	nf->src_dev_enum = 0;
	nf->src_dev_id = R7K_DEV_ID;
}

/* payload_size is  record_type_header + rd ( variable length payload)*/
static void fill_out_data_record_frame(/*INPUT*/ size_t record_size, double ts, uint32_t record_id, uint8_t recorded_not_live ,/*OUTPUT*/ r7k_DataRecordFrame_t* drf){
    //fprintf(stderr,"fill_out_data_record_frame record_size=%d, record_id=%d\n",record_size,record_id);
    //Input record size is from start of network frame to end of checksum
    drf->version = R7K_VERSION;                     // (= 5) Protocol version of this frame
    drf->offset = sizeof(r7k_DataRecordFrame_t) - 4;  //Size from start of sync pattern to start of RecordTypeHeader 
    drf->sync = 0x0000FFFF; 
    drf->size = record_size - sizeof(r7k_NetworkFrame_t) -4;  //TODO understand the -4 , shoudl be from start of r7k_DataRecordFrame.version to end of r7k_Checksum 
    drf->size += sizeof(r7k_Checksum_t);     
    drf->opt_offset  = 0;
    drf->opt_id = 0;
    r7k_ts_to_r7ktime(ts, &(drf->time));
    drf->time.record_ver = 1;
    drf->record_id = record_id;
    drf->dev_id = R7K_DEV_ID;
    drf->dev_enum = 0;	// To differ between devices with same dev_id
    drf->res2 = 0;
    drf->flags = 0 + (recorded_not_live)?(1<<15):0;  //Invalid checksum
    drf->res3 = 0;
    drf->res4 = 0;
    drf->tot_frag_num = 0;
    drf->frag_num = 0;
}


static void fill_out_7027_record_type_header(output_data_t* data, uint32_t N, /*OUTPUT*/ r7k_RecordTypeHeader_7027_t* rth){
	rth->serial = 0;                            //Sonar serial number
	rth->ping_nr = data->ping_number;           //sequential number
	rth->multi_ping = data->multifreq_index;    // TODO: should this be multi ping or multi freq index
	rth->N = N;                                              //Number of detection points
	rth->data_field_size = sizeof(r7k_RecordData_7027_t); //Size of each detection information block in bytes
	rth->detect_alg = 1;        //0=simple 1=blend_filt
	rth->flags = 0;             // Just leave this at 0 for now
	rth->fs = data->sample_rate;    // Sample rate in Hz
	rth->tx_angle = data->tx_angle;          // In radians
    rth->applied_roll = 0;

}

static void fill_out_7000_record_type_header(output_data_t* data,  /*OUTPUT*/ r7k_RecordTypeHeader_7000_t* rth){
	rth->serial = 0;                            //Sonar serial number
    if (data){
        rth->ping_nr = data->ping_number;           //sequential number
        rth->multi_ping = data->multifreq_index;    // TODO: should this be multi ping or multi freq index
        rth->tx_freq    = data->tx_freq;    //Tx frequency in Hz
        rth->fs         = data->sample_rate;    //Sample rate in Hz
        rth->bw         = data->tx_bw;      //Bandwidth in Hz
        rth->tx_len     = data->tx_plen;    //Tx pulse length in sec
        rth->tx_type    = (data->tx_bw)?1:0;// 0 = CW, 1 = Chirp
        rth->ping_per = 1./data->ping_rate;             //Seconds since last ping
        rth->range = data->sv * rth->ping_per / 2 ;     // Rudamentary aproximate value (does not have the real value from most sources)
        rth->tx_power = (data->tx_voltage)?(180+(20*log10f(data->tx_voltage))):215; // in dB rel 1 uPa (very aproximate model)
        rth->project_steer_vert = data->tx_angle; //Projector stering in radians
        rth->sound_velocity     = data->sv; // in m/s
	}
    else{ //Just send some somewhat sane values as a default before real data is available 
        rth->ping_nr = 0 ;
        rth->multi_ping = 0 ;
        rth->tx_freq    = 400e3; 
        rth->fs         = 78125;
        rth->bw         = 0;
        rth->tx_len     = 100e-6;
        rth->tx_type    = 0;
        rth->ping_per = 1.;
        rth->range = 100.;
        rth->tx_power = 0;
        rth->project_steer_vert = 0;
        rth->sound_velocity     = 1500;
	}

    rth->tx_taper   = 0; // 0 = tapered rect, 1 = Tukey
	rth->tx_taper_param = 0; // = 0
	rth->tx_pulse_mode = 1; // = 0      TODO just 1 (single ping) for now, how to set this based on multifreq setting
	rth->tx_pulse_res = 0; // = 0
	rth->max_ping_rate = 60.;
	rth->gain               = 0; // in dB
	rth->ctrl_flags         = 0; //Just set all to zero for now
	rth->project_id         = 0; //Projector identifier
	rth->project_steer_hor  = 0; //Projector steering in radians
	rth->project_open_vert  = 1*M_PI/2;     // -3dB in radians
	rth->project_open_hor   = 160*M_PI/2;   // -3dB in radians
	rth->project_focus      = 1000.; // in m
	rth->project_taper      = 0; // 0 = rect, 1 = Cheb
	rth->project_taper_param=0; // = 0
	rth->tx_flags           = 0; // = 0 (0-3 Pitch stab meth 4-7 Yaw stab meth)
	rth->hydrophone_id      = 0; //= 0
	rth->rx_taper           = 0; //0 = cheb 1 = kais
	rth->rx_taper_param     = 0; // = 0
	rth->rx_flags           = 0; // Just set all to zero for now 
	rth->rx_beam_witdth     = 1.0*M_PI/180; // in rad, just set to some sane value
	rth->min_gate_range     = 0;
	rth->max_gate_range     = 0;
	rth->min_gate_depth     = 0;
	rth->max_gate_depth     = 0;
	rth->absorbtion         = 98.; // in dB/km  (Just a somewhat sane value)
	rth->spread_loss        = 20.; // in dB
}

static void fill_out_7030_record_type_header( /*OUTPUT*/ r7k_RecordTypeHeader_7030_t* rth){
    memset(rth,0,sizeof(r7k_RecordTypeHeader_7030_t));
    rth->Length_of_software_version_info = 8;
    strcpy(&(rth->Software_version_info[0]), "0.0.0 ()");
    rth->Length_of_record_protocol_info = 4;
    strcpy(&(rth->Record_protocol_version_info[0]), "2.43 ");
    //TODO lot of filling in here
}
static void fill_out_7200_record_type_header( int num_devices,/*OUTPUT*/ r7k_RecordTypeHeader_7200_t* rth){
    rth->File_identifier[0] = (((uint64_t) rand())<<32) + rand();
    rth->File_identifier[1] = (((uint64_t) rand())<<32) + rand();
    rth->Version_number = 1; //File_format_version_number
    rth->Session_identifier[0] = (((uint64_t) rand())<<32) + rand(); //User_defined_session_identifier._Used_to_associate_multiple_files_for_a_given_session.
    rth->Session_identifier[1] = (((uint64_t) rand())<<32) + rand(); //User_defined_session_identifier._Used_to_associate_multiple_files_for_a_given_session.
    rth->Record_data_size = 0; //Size_of_record_data._0_–_If_not_present
    rth->N = num_devices; //Number_of_devices_(N_≥_0)
    strcpy(&(rth->Recording_name[0]), "wbms_georef converted data");
    strcpy(&(rth->Recording_program_version_number[0]), "");
    strcpy(&(rth->User_defined_name[0]), "");
    strcpy(&(rth->Notes[0]), "");
}

static void fill_out_7001_record_type_header( /*OUTPUT*/ r7k_RecordTypeHeader_7001_t* rth){
    rth->serial = 0;
    rth->N = 1;
}
static void fill_out_7001_record_type_data( /*OUTPUT*/ r7k_RecordData_7001_t* rd){
    rd->id = 0;
    rd->desc[0] = 0;
    rd->alpha_data_card = 0;
    rd->serial = 0;
    rd->info_len = sizeof(sonar_info_xml);
    strcpy(&(rd->info[0]), sonar_info_xml);
 }

/* s7k packets to write on stream connection
* 7000 Sonar settings
* 7001 7k configuration
* 7502 remote NAK 
*/
int write_r7k_stream_header_to_buffer(/*INPUT TBD*/double ts, /*OUTPUT*/char* outbuf){
    size_t total_data_size = 0;
    char* dp = outbuf;
    size_t record_size;

    // TODO split data into network framnes based on max_packet_size  (Only neccessary for UDP output streams)
    //Add 7000 record to data stream
    record_size = sizeof(r7k_Record_7000_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7000_t* rec0 = malloc(record_size);
    if (write_network_frame){
        fill_out_network_frame( record_size , &(rec0->NF));
    }
    fill_out_data_record_frame( record_size,
                                ts, 
                                7000, 
                                recorded_not_live ,
                                &(rec0->DRF)
                              );
    fill_out_7000_record_type_header(NULL, &(rec0->RTH));
    if (write_network_frame){
        memcpy(dp, rec0, record_size);
    }
    else{
        record_size -= sizeof(r7k_NetworkFrame_t);
        memcpy(dp, &(rec0->DRF), record_size);
    }
    dp += record_size;
    total_data_size += record_size;
    free(rec0);
   
    //Add 7001 record to data stream
    
    record_size = sizeof(r7k_Record_7001_t) + sizeof(sonar_info_xml) + sizeof(r7k_Checksum_t);     
    r7k_Record_7001_t* rec1 = malloc(record_size);
    if (write_network_frame){
        fill_out_network_frame( record_size , &(rec1->NF));
    }
    fill_out_data_record_frame( record_size,
                                ts, 
                                7001, 
                                recorded_not_live ,
                                &(rec1->DRF)
                              );
    fill_out_7001_record_type_header(&(rec1->RTH));
    fill_out_7001_record_type_data(&(rec1->RD));

    if (write_network_frame){
        memcpy(dp, rec1, record_size);
    }
    else{
        record_size -= sizeof(r7k_NetworkFrame_t);
        memcpy(dp, &(rec1->DRF), record_size);
    }
    dp += record_size;
    total_data_size += record_size;
    free(rec1);

    return total_data_size;
}

/* s7k packets to write to file start
* 7030 Sonar installation parameters
* 7200 File header
*/
int write_r7k_file_header_to_buffer(/*INPUT*/ double ts, /*OUTPUT*/char* outbuf){
    size_t total_data_size = 0;
    char* dp = outbuf;
    size_t record_size;

    //add 7030 record to data stream  (never with network header, as it is only for file output
    record_size = sizeof(r7k_Record_7030_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7030_t* rec0 = malloc(record_size);
    fill_out_data_record_frame( record_size,
                                ts, 
                                7030, 
                                recorded_not_live ,
                                &(rec0->DRF)
                              );
    fill_out_7030_record_type_header(&(rec0->RTH));
    record_size -= sizeof(r7k_NetworkFrame_t);
    memcpy(dp, &(rec0->DRF), record_size);
    dp += record_size;
    total_data_size += record_size;
    free(rec0);
    
    //add 7200 record to data stream  (never with network header, as it is only for file output
    uint16_t dev_list[] = {7000, 7200, 7027,7610};
    uint16_t num_dev = sizeof(dev_list)/sizeof(dev_list[0]);
    record_size = sizeof(r7k_Record_7200_t) + num_dev*sizeof(r7k_RecordData_7200_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7200_t* rec1 = malloc(record_size);
    fill_out_data_record_frame( record_size,
                                ts, 
                                7200, 
                                recorded_not_live ,
                                &(rec1->DRF)
                              );
    fill_out_7200_record_type_header(num_dev ,&(rec1->RTH));
    for (uint16_t n = 0; n<num_dev;n++){
         rec1->RD[n].dev_id = dev_list[n];
         rec1->RD[n].sys_enum = 0;
    }
    record_size -= sizeof(r7k_NetworkFrame_t);
    memcpy(dp, &(rec1->DRF), record_size);
    dp += record_size;
    total_data_size += record_size;
    free(rec1);

    return total_data_size; 
}

int write_r7k_header_to_buffer(/*INPUT TBD*/double ts, /*OUTPUT*/char* outbuf){
    if (recorded_not_live){
        fprintf(stderr, "write_r7k_file_header_to_buffer\n");
        return write_r7k_file_header_to_buffer(ts,outbuf);
    }
    else{
        fprintf(stderr, "write_r7k_stream_header_to_buffer\n");
        return write_r7k_stream_header_to_buffer(ts,outbuf);
    }
}


/* s7k Packets to write on bathy data
* 7000 Sonar settings
* 7004 Beam geometry        (perhaps skip)
* 7027 Raw detection data
* 7610 Sound velocity       (perhaps skip)
*/
int write_r7k_bathy_to_buffer(double ts, output_data_t* data, uint32_t N, /*OUTPUT*/char* outbuf){
    size_t total_data_size = 0;
    char* dp = outbuf;
    size_t record_size;

    // TODO split data into network framnes based on max_packet_size  (Only neccessary for UDP output streams)
    
    //Add 7000 record to data stream
    record_size = sizeof(r7k_Record_7000_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7000_t* rec0 = malloc(record_size);
    if (write_network_frame){
        fill_out_network_frame( record_size , &(rec0->NF));
    }
    fill_out_data_record_frame( record_size,
                                ts, 
                                7000, 
                                recorded_not_live ,
                                &(rec0->DRF)
                              );
    fill_out_7000_record_type_header(data, &(rec0->RTH));
    if (write_network_frame){
        memcpy(dp, rec0, record_size);
    }
    else{
        record_size -= sizeof(r7k_NetworkFrame_t);
        memcpy(dp, &(rec0->DRF), record_size);
    }
    dp += record_size;
    total_data_size += record_size;
    free(rec0);

    // Add 7027 record to data stream
    record_size = sizeof(r7k_Record_7027_t) + N*sizeof(r7k_RecordData_7027_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7027_t* rec1 = malloc(record_size);
    if (write_network_frame){
        fill_out_network_frame( record_size , &(rec1->NF));
    }
    fill_out_data_record_frame( record_size,
                                ts, 
                                7027, 
                                recorded_not_live ,
                                &(rec1->DRF)
                              );
    fill_out_7027_record_type_header(data,N, &(rec1->RTH));
    float sample_number_per_meter = 2*data->sample_rate / data->sv;
    for (uint32_t n=0; n<N;n++){
         rec1->RD[n].beam = data->beam[n];   //Beam number the detection is taken from
         rec1->RD[n].detection_point = data->range[n] * sample_number_per_meter; //Non-corrected fractional sample number with reference to receiver’s acoustic center with the zero sample at the transmit time
         rec1->RD[n].rx_angle = data->teta[n]; //Beam steering angle
         rec1->RD[n].flags = 3 + (1<<2) + (((uint8_t) (data->priority[n]))&(0x0f)<<9) ; //[0:mag, :phase,2-8:qual = 1,0...., 9-12:pri]
         rec1->RD[n].quality = data->quality_flags[n]; //[0: bright, 1:Colin,0...]
         rec1->RD[n].uncertainty = sqrtf(data->z_var[n]);
         rec1->RD[n].signal_strength = data->i[n];
         rec1->RD[n].limit_min = data->up_gate[n]  * sample_number_per_meter;
         rec1->RD[n].limit_max = data->low_gate[n] * sample_number_per_meter;
    }
    if (write_network_frame){
        memcpy(dp, rec1, record_size);
    }
    else{
        record_size -= sizeof(r7k_NetworkFrame_t);
        memcpy(dp, &(rec1->DRF), record_size);
    }
    dp += record_size;
    total_data_size += record_size;
    free(rec1);

    //Add 7610 record to data stream
    record_size = sizeof(r7k_Record_7610_t) + sizeof(r7k_Checksum_t);     
    r7k_Record_7610_t* rec2 = malloc(record_size);
    if (write_network_frame){
        fill_out_network_frame( record_size , &(rec2->NF));
    }
    fill_out_data_record_frame( record_size,
                                ts, 
                                7610, 
                                recorded_not_live ,
                                &(rec2->DRF)
                              );
    rec2->RTH.sv = data->sv;
    if (write_network_frame){
        memcpy(dp, rec2, record_size);
    }
    else{
        record_size -= sizeof(r7k_NetworkFrame_t);
        memcpy(dp, &(rec2->DRF), record_size);
    }
    dp += record_size;
    total_data_size += record_size;
    free(rec2);

    return total_data_size;
}


int write_r7k_nav_to_buffer(double ts, navdata_t posdata[NAVDATA_BUFFER_LEN],size_t pos_ix, aux_navdata_t *aux_navdata, /*OUTPUT*/char* outbuf){
    return 0;
}
