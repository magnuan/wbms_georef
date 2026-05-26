#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "cmath.h"
#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif
#include "time_functions.h"
#include "classification.h"

#define CLASSIFICATION_MAX_LINE_LEN (16*1024)

/* Library to read in classification data from a CSV style file
 * The input file contains classification values for datapoints, one datapoint per row.
 * Each row consists of two or three columns (keys) that uniquely identifies the sounding, that is:
 * - ping_number 
 * - beam_number
 * - multi_ping   (optional, set to 0 if not present as data column)
 * It must also have one column:
 * - class
 * That is the value that should be associated with the datapoint
 *
 * We assume that the three keys together represents a unique identifier.
 * and that now two rows in the input file contains has the same combination
 * 
 * The library should read the "class" data into a 3-dimentional array for fast readout
 * where the dimentions represets each of the three keys

 * For efficiency the library will read through the file, and fill out:
 * - one 1D LUT that associates ping numbers with ping_ix 
 * - one 3D LUT that gives a class value based on a ping_ix, beam_number and multi_ping
 *
 *
 *
 * Input file consists of:
 *
 * Comments:
 * - lines starting with #
 * - e.g.: # This file was manually classified by Sant A. Claus
 * 
 * Collumns title:  
 * - line starting with //
 * - Comma separated Collumns titles
 * - e.g.:    //X,Y,Z,range,teta,quality,ROLL,PITCH,beam,pingnumber,freq,multifreq,aoi,el,val,val_dB,Classification
 *
 * Data:
 * - Comma separated values
 * - e.g.: 752063.53168869,4042746.07653809,-3.33670878,7.905000,-63.660000,3.000000,1.330000,-2.660000,0.000000,62967.000000,400000.000000,0.000000,-62.269062,0.000000,205768.031250,106.267555,0.000000
 *
 * 
 * The file must contain a column with ping_number, allowed titles for this is: "ping", "pingnumber", or "ping_number" (non-case sensitive)
 * The file must contain a column with beam_number, allowed titles for this is: "beam", "beamnumber", or "beam_number" (non-case sensitive)
 * The file could contain a column with multi_ping, allowed titles for this is: "multiping", "multi_ping", "multifreq", "multi_freq" (non-case sensitive)
 * The file must contain a column with class, allowed titles for this is: "class", "classification" (non-case sensitive)
 * 
 * The values in the file could be represented as floation point values, in which case tehy should be rounded off to integer values in parsing
 *
 */


typedef struct{
    uint32_t ping_number;       // Ping number as given in input file
    int32_t ping_ix;            // Ping index in class_table (negative for invalid)
} ping_tuple_t;

typedef struct{
    int ping;
    int beam;
    int mp;
    int class_id;
} classification_columns_t;

typedef struct{
    uint32_t ping;
    uint32_t beam;
    uint32_t mp;
    int16_t class_id;
} classification_row_t;

/* Since we only will be operating on a single file at a time, these values can be global and singleton
 */

static uint32_t n_pings;
static uint32_t n_beams;
static uint32_t n_mp;
static uint8_t cached_ping_valid = 0;
static uint32_t cached_ping = 0;
static int32_t cached_ping_ix = -1;



/* Table mapping ping_number from input file, to a index in the class_table
 * 1D array of length n_pings
*/ 

static ping_tuple_t* ping_tuple = NULL;    

/* This should be pointing to a 3D array with class value from input file.
 *
 * 3D array of size n_pings * n_beams * n_mp
 *
 * Class value for a specific sounding (ping_ix, beam, mp, is stored in):
 * class_table[ping_ix*(n_beams*n_mp) + beam*n_mp + mp]
 *
 * Entries not found in file should be given the value -1
 */
static int16_t*  class_table = NULL;   


static void classification_reset_cache(void){
    cached_ping_valid = 0;
    cached_ping = 0;
    cached_ping_ix = -1;
}

static char* classification_trim(char* str){
    while ((*str != 0) && isspace((unsigned char)*str)){
        str++;
    }

    char* end = str + strlen(str);
    while ((end > str) && isspace((unsigned char)end[-1])){
        end--;
        *end = 0;
    }

    return str;
}

static char* classification_next_csv_field(char** cursor){
    if ((cursor == NULL) || (*cursor == NULL)){
        return NULL;
    }

    char* field = *cursor;
    char* comma = strchr(field, ',');
    if (comma){
        *comma = 0;
        *cursor = comma + 1;
    }
    else{
        *cursor = NULL;
    }

    return classification_trim(field);
}

static void classification_init_columns(classification_columns_t* columns){
    columns->ping = -1;
    columns->beam = -1;
    columns->mp = -1;
    columns->class_id = -1;
}

static void classification_normalize_header_name(const char* in, char* out, size_t out_len){
    size_t ix = 0;

    while ((*in != 0) && (ix + 1 < out_len)){
        unsigned char c = (unsigned char)*in;
        if ((c != '_') && (c != '-') && (c != '"') && (c != '\'') && !isspace(c)){
            out[ix++] = (char)tolower(c);
        }
        in++;
    }

    out[ix] = 0;
}

static int classification_column_type(char* header_name){
    char name[64];
    classification_normalize_header_name(header_name, name, sizeof(name));

    if ((strcmp(name, "ping") == 0) || (strcmp(name, "pingnumber") == 0)){
        return 1;
    }
    if ((strcmp(name, "beam") == 0) || (strcmp(name, "beamnumber") == 0)){
        return 2;
    }
    if ((strcmp(name, "multiping") == 0) || (strcmp(name, "multifreq") == 0)){
        return 3;
    }
    if ((strcmp(name, "class") == 0) || (strcmp(name, "classification") == 0)){
        return 4;
    }

    return 0;
}

static int classification_parse_header_line(char* line, classification_columns_t* columns){
    char* header = classification_trim(line);

    if (strncmp(header, "//", 2) == 0){
        header += 2;
    }

    classification_init_columns(columns);

    int field_ix = 0;
    char* cursor = header;
    char* field;
    while ((field = classification_next_csv_field(&cursor)) != NULL){
        switch (classification_column_type(field)){
            case 1:
                if (columns->ping < 0) columns->ping = field_ix;
                break;
            case 2:
                if (columns->beam < 0) columns->beam = field_ix;
                break;
            case 3:
                if (columns->mp < 0) columns->mp = field_ix;
                break;
            case 4:
                if (columns->class_id < 0) columns->class_id = field_ix;
                break;
            default:
                break;
        }
        field_ix++;
    }

    return ((columns->ping >= 0) && (columns->beam >= 0) && (columns->class_id >= 0));
}

static int classification_find_header(FILE* fp, classification_columns_t* columns, size_t* line_number){
    char line[CLASSIFICATION_MAX_LINE_LEN];
    
    uint32_t line_cnt=0;
    while (fgets(line, sizeof(line), fp)){
        (*line_number)++;
        
        if (line_cnt++ > 100){ //Give up if header not found in the first 100 lines
            return -1;
        }

        char* trimmed = classification_trim(line);
        if ((*trimmed == 0) || (*trimmed == '#')){
            continue;
        }

        if (classification_parse_header_line(trimmed, columns)){
            return 0;
        }
    }

    return -1;
}

static int classification_parse_rounded_double(char* field, double min, double max, double* value_out){
    char* field_start = classification_trim(field);
    if (*field_start == 0){
        return 0;
    }

    errno = 0;
    char* endptr;
    double value = strtod(field_start, &endptr);
    if (field_start == endptr){
        return 0;
    }

    endptr = classification_trim(endptr);
    if (*endptr != 0){
        return 0;
    }

    if ((errno == ERANGE) || isnan(value) || (value < min) || (value > max)){
        return 0;
    }

    value = round(value);
    if ((value < min) || (value > max)){
        return 0;
    }

    *value_out = value;
    return 1;
}

static int classification_parse_uint32(char* field, uint32_t* value_out){
    double value;
    if (!classification_parse_rounded_double(field, 0.0, (double)UINT32_MAX, &value)){
        return 0;
    }

    *value_out = (uint32_t)value;
    return 1;
}

static int classification_parse_int16(char* field, int16_t* value_out){
    double value;
    if (!classification_parse_rounded_double(field, (double)INT16_MIN, (double)INT16_MAX, &value)){
        return 0;
    }

    *value_out = (int16_t)value;
    return 1;
}

static int classification_parse_row(char* line, classification_columns_t* columns, classification_row_t* row){
    char* trimmed = classification_trim(line);
    if ((*trimmed == 0) || (*trimmed == '#') || (strncmp(trimmed, "//", 2) == 0)){
        return 0;
    }

    char* ping_field = NULL;
    char* beam_field = NULL;
    char* mp_field = NULL;
    char* class_field = NULL;

    int field_ix = 0;
    char* cursor = trimmed;
    char* field;
    while ((field = classification_next_csv_field(&cursor)) != NULL){
        if (field_ix == columns->ping){
            ping_field = field;
        }
        if (field_ix == columns->beam){
            beam_field = field;
        }
        if (field_ix == columns->mp){
            mp_field = field;
        }
        if (field_ix == columns->class_id){
            class_field = field;
        }
        field_ix++;
    }

    if ((ping_field == NULL) || (beam_field == NULL) || (class_field == NULL)){
        return -1;
    }
    if ((columns->mp >= 0) && (mp_field == NULL)){
        return -1;
    }

    if (!classification_parse_uint32(ping_field, &row->ping)){
        return -1;
    }
    if (!classification_parse_uint32(beam_field, &row->beam)){
        return -1;
    }
    if (columns->mp >= 0){
        if (!classification_parse_uint32(mp_field, &row->mp)){
            return -1;
        }
    }
    else{
        row->mp = 0;
    }
    if (!classification_parse_int16(class_field, &row->class_id)){
        return -1;
    }

    return 1;
}

static int classification_append_unique_ping(uint32_t** pings, size_t* n, size_t* capacity, uint32_t ping){
    if ((*n > 0) && ((*pings)[*n - 1] == ping)){
        return 0;
    }

    for (size_t ix = 0; ix < *n; ix++){
        if ((*pings)[ix] == ping){
            return 0;
        }
    }

    if (*n == *capacity){
        size_t new_capacity = (*capacity == 0) ? 1024 : (*capacity * 2);
        if (new_capacity < *capacity){
            return -1;
        }

        uint32_t* new_pings = realloc(*pings, new_capacity * sizeof((*pings)[0]));
        if (new_pings == NULL){
            return -1;
        }

        *pings = new_pings;
        *capacity = new_capacity;
    }

    (*pings)[*n] = ping;
    (*n)++;
    return 0;
}

static int32_t classification_find_or_add_ping_tuple(ping_tuple_t* tuples, uint32_t* n, uint32_t capacity, uint32_t ping){
    if ((*n > 0) && (tuples[*n - 1].ping_number == ping)){
        return tuples[*n - 1].ping_ix;
    }

    for (uint32_t ix = 0; ix < *n; ix++){
        if (tuples[ix].ping_number == ping){
            return tuples[ix].ping_ix;
        }
    }

    if (*n >= capacity){
        return -1;
    }

    tuples[*n].ping_number = ping;
    tuples[*n].ping_ix = (int32_t)*n;
    (*n)++;

    return tuples[*n - 1].ping_ix;
}

static int classification_table_entry_count(uint32_t pings, uint32_t beams, uint32_t multiping, size_t* entries){
    if ((pings == 0) || (beams == 0) || (multiping == 0)){
        return -1;
    }

    size_t pings_s = pings;
    size_t beams_s = beams;
    size_t multiping_s = multiping;

    if (beams_s > (SIZE_MAX / multiping_s)){
        return -1;
    }
    size_t beams_multiping = beams_s * multiping_s;

    if (pings_s > (SIZE_MAX / beams_multiping)){
        return -1;
    }

    *entries = pings_s * beams_multiping;
    return 0;
}

/* classification_count 
*   Reads through a classification file and derives the array size needed to store the data
*
*   fname       (input)     Input filename to process
*   
*   pings       (output)    Number of unique ping_number values in file
*   beams       (output)    Max beam_number in file
*   multiping   (output)    Max mult_iping value in file
*   rows        (output)    Total number of valid data rows
*/
static int classification_count(char* fname, /*OUTPUT*/ uint32_t* pings, uint32_t* beams, uint32_t* multiping, uint32_t* rows){
    static const uint8_t report_errors = 1;
    if (fname == NULL){
        return -1;
    }

    FILE * fp = fopen(fname,"r");
    if (fp == NULL){
        if (report_errors) fprintf(stderr, "Could not open classification file: %s\n", fname);
        return -1;
    }

    classification_columns_t columns;
    size_t line_number = 0;
    if (classification_find_header(fp, &columns, &line_number) < 0){
        if (report_errors) fprintf(stderr, "Could not find classification file header in: %s\n", fname);
        fclose(fp);
        return -1;
    }

    uint32_t* unique_pings = NULL;
    size_t unique_ping_count = 0;
    size_t unique_ping_capacity = 0;
    uint32_t max_beam = 0;
    uint32_t max_mp = 0;
    uint8_t got_data = 0;
    uint32_t row_count = 0;

    char line[CLASSIFICATION_MAX_LINE_LEN];
    while (fgets(line, sizeof(line), fp)){
        line_number++;

        classification_row_t row;
        int parsed = classification_parse_row(line, &columns, &row);
        if (parsed < 0){
            if (report_errors) fprintf(stderr, "Invalid classification row in %s:%zu\n", fname, line_number);
            free(unique_pings);
            fclose(fp);
            return -1;
        }
        if (parsed == 0){
            continue;
        }

        got_data = 1;
        if (classification_append_unique_ping(&unique_pings, &unique_ping_count, &unique_ping_capacity, row.ping) < 0){
            if (report_errors) fprintf(stderr, "Could not allocate classification ping table\n");
            free(unique_pings);
            fclose(fp);
            return -1;
        }
        row_count++;
        if (row.beam > max_beam) max_beam = row.beam;
        if (row.mp > max_mp) max_mp = row.mp;
    }

    fclose(fp);

    if (!got_data || (unique_ping_count == 0) || (unique_ping_count > (size_t)INT32_MAX)){
        if (report_errors) fprintf(stderr, "No classification data found in: %s\n", fname);
        free(unique_pings);
        return -1;
    }
    if ((max_beam == UINT32_MAX) || (max_mp == UINT32_MAX)){
        if (report_errors) fprintf(stderr, "Classification beam or multiping index is too large\n");
        free(unique_pings);
        return -1;
    }

    *pings = (uint32_t)unique_ping_count;
    *beams = max_beam + 1;
    *multiping = max_mp + 1;
    *rows = row_count;

    free(unique_pings);
    return 0;
}




/* Find ping_ix from ping, using the ping_tuple LUT
 * Since this function typically will be called multiple times with same ping number, 
 * we retain the last value, and only do a look-up when th e ping number changes
 */
static inline int32_t classification_find_ping_ix(uint32_t ping){
    // Same ping as last time
    if (cached_ping_valid && (ping == cached_ping)) return cached_ping_ix;
    // New ping number, find in ping_tuple table
    for(size_t ix=0; ix<n_pings;ix++){
        if (ping_tuple[ix].ping_number == ping){
            cached_ping_ix = ping_tuple[ix].ping_ix;
            cached_ping = ping;
            cached_ping_valid = 1;
            return cached_ping_ix;
        }
    }
    //Ping not found in table, return -1
    cached_ping_ix = -1;
    cached_ping = ping;
    cached_ping_valid = 1;
    return -1; 
}

int classification_read_from_file(char* fname){
    int ret;
    uint32_t new_n_pings;
    uint32_t new_n_beams;
    uint32_t new_n_mp;
    uint32_t new_rows;
    size_t table_entries;

    // Calculate the number of unique pings, beams and multi pings. For table dimentions.
    ret = classification_count(fname, &new_n_pings, &new_n_beams, &new_n_mp, &new_rows);
    if (ret<0) return ret;

    if (classification_table_entry_count(new_n_pings, new_n_beams, new_n_mp, &table_entries) < 0){
        fprintf(stderr, "Classification table size is invalid\n");
        return -1;
    }
    if (table_entries > (SIZE_MAX / sizeof(class_table[0]))){
        fprintf(stderr, "Classification table is too large\n");
        return -1;
    }
    
    // Allocate required memory for new tables
    ping_tuple_t* new_ping_tuple  = calloc(new_n_pings, sizeof(ping_tuple_t));
    if (new_ping_tuple==NULL) return -1;
    int16_t* new_class_table = malloc(table_entries*sizeof(new_class_table[0]));
    if (new_class_table==NULL){
        free(new_ping_tuple);
        return -1;
    }

    // Pre-fill the class table with -1
    for (size_t ix=0; ix<table_entries;ix++){
        new_class_table[ix] = -1;
    }

    FILE * fp = fopen(fname,"r");
    if (fp == NULL){
        fprintf(stderr, "Could not open classification file: %s\n", fname);
        free(new_ping_tuple);
        free(new_class_table);
        return -1;
    }

    classification_columns_t columns;
    size_t line_number = 0;
    if (classification_find_header(fp, &columns, &line_number) < 0){
        fprintf(stderr, "Could not find classification file header in: %s\n", fname);
        fclose(fp);
        free(new_ping_tuple);
        free(new_class_table);
        return -1;
    }

    uint32_t unique_ping_count = 0;
    char line[CLASSIFICATION_MAX_LINE_LEN];
    while (fgets(line, sizeof(line), fp)){
        line_number++;

        classification_row_t row;
        int parsed = classification_parse_row(line, &columns, &row);
        if (parsed < 0){
            fprintf(stderr, "Invalid classification row in %s:%zu\n", fname, line_number);
            fclose(fp);
            free(new_ping_tuple);
            free(new_class_table);
            return -1;
        }
        if (parsed == 0){
            continue;
        }

        int32_t ping_ix = classification_find_or_add_ping_tuple(new_ping_tuple, &unique_ping_count, new_n_pings, row.ping);
        if (ping_ix < 0){
            fprintf(stderr, "Could not add ping %u to classification table\n", row.ping);
            fclose(fp);
            free(new_ping_tuple);
            free(new_class_table);
            return -1;
        }

        if ((row.beam >= new_n_beams) || (row.mp >= new_n_mp)){
            fprintf(stderr, "Classification row index outside allocated table in %s:%zu\n", fname, line_number);
            fclose(fp);
            free(new_ping_tuple);
            free(new_class_table);
            return -1;
        }

        size_t table_ix = ((size_t)ping_ix*((size_t)new_n_beams*new_n_mp)) + ((size_t)row.beam*new_n_mp) + row.mp;
        new_class_table[table_ix] = row.class_id;
    }

    fclose(fp);

    if (unique_ping_count != new_n_pings){
        fprintf(stderr, "Classification file changed while reading: %s\n", fname);
        free(new_ping_tuple);
        free(new_class_table);
        return -1;
    }

    // Free up any existing tables before replacing them with the new data.
    if (ping_tuple) free(ping_tuple);
    if (class_table) free(class_table);

    ping_tuple = new_ping_tuple;
    class_table = new_class_table;
    n_pings = new_n_pings;
    n_beams = new_n_beams;
    n_mp = new_n_mp;
    classification_reset_cache();

    return 0;
}

int16_t classification_get_class(int32_t ping, uint32_t beam, uint32_t mp){
    if ((ping < 0) || (ping_tuple == NULL) || (class_table == NULL)){
        return -1;
    }
    if ((beam >= n_beams) || (mp >= n_mp)){
        return -1;
    }

    int32_t ping_ix = classification_find_ping_ix((uint32_t)ping);
    if (ping_ix<0) return -1;
    
    return class_table[((size_t)ping_ix*((size_t)n_beams*n_mp)) + ((size_t)beam*n_mp) + mp];
}
    
        



/* classification_test_file
 * Checks if the input file is a valid classification file
 */
int classification_test_file(char* fname, uint32_t* pings,uint32_t* beams,uint32_t* mp,uint32_t* rows){
    return (classification_count(fname, pings, beams, mp, rows) == 0);
}
