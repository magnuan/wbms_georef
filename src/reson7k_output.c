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

#define R7K_VERSION 5
#define R7K_DEV_ID 14001
#define R7K_OUTPUT_FS 78125.

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
	nf->packet_size = packet_size+sizeof(r7k_NetworkFrame_t);         //Size of this packet
	nf->tot_size = packet_size;             //Size of all packets in transmission (excluding network frames)
	nf->seq_num = 0;                        //Sequential packet number (TODO: to be used if more packets in transmission
	nf->dest_dev_id = R7K_DEV_ID;
	nf->dest_dev_enum = 0;
	nf->src_dev_enum = 0;
	nf->src_dev_id = R7K_DEV_ID;
}

/* payload_size is  record_type_header + rd ( variable length payload)*/
static void fill_out_data_record_frame(/*INPUT*/ size_t payload_size, double ts, uint32_t record_id, uint8_t recorded_not_live ,/*OUTPUT*/ r7k_DataRecordFrame_t* drf){
    drf->version = R7K_VERSION;                     // (= 5) Protocol version of this frame
    drf->offset = sizeof(r7k_DataRecordFrame_t) - 4;  //Size from start of sync pattern to start of RecordTypeHeader 
    drf->sync = 0x0000FFFF; 
    drf->size = payload_size + sizeof(r7k_DataRecordFrame_t); //From start of r7k_DataRecordFrame.version to end of r7k_Checksum
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
	rth->fs = R7K_OUTPUT_FS;    // Sample rate in Hz
	rth->tx_angle = data->tx_angle;          // In radians
    rth->applied_roll = 0;

}

int write_r7k_header_to_buffer(/*INPUT TBD*/ /*OUTPUT*/char* outbuf){
    return 0; 
}


int write_r7k_bathy_to_buffer(double ts, output_data_t* data, uint32_t N, /*OUTPUT*/char* outbuf){
    size_t total_data_size = 0;
    char* dp = outbuf;
    size_t record_size;

    // TODO split data into network framnes based on max_packet_size  (Only neccessary for UDP output streams)

    // Add 7027 record to data stream
    record_size = sizeof(r7k_Record_7027_t) + N*sizeof(r7k_RecordData_7027_t);
    record_size += sizeof(r7k_Checksum_t);     
    r7k_Record_7027_t* rec1 = malloc(record_size);
    if (write_network_frame)
        fill_out_network_frame( record_size - sizeof(r7k_NetworkFrame_t), &(rec1->NF));
    fill_out_data_record_frame( sizeof(r7k_RecordTypeHeader_7027_t) + N*sizeof(r7k_RecordData_7027_t), 
                                ts, 
                                7027, 
                                recorded_not_live ,
                                &(rec1->DRF)
                              );
    fill_out_7027_record_type_header(data,N, &(rec1->RTH));
    float sample_number_per_meter = 2*R7K_OUTPUT_FS / data->sv;
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
    record_size = sizeof(r7k_Record_7610_t); 
    record_size += sizeof(r7k_Checksum_t);     
    r7k_Record_7610_t* rec2 = malloc(record_size);
    if (write_network_frame)
        fill_out_network_frame( record_size - sizeof(r7k_NetworkFrame_t), &(rec2->NF));
    fill_out_data_record_frame( sizeof(r7k_RecordTypeHeader_7610_t), 
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
