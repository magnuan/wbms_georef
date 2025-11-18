/*******************************************************************************
 * (c) Copyright 2012-2019 Norbit Subsea. All rights reserved.                 
 *******************************************************************************/
/* This file has been prepared for Doxygen automatic documentation generation. */
/****************************************************************************//**
* @file cmath.c
*
* @brief Collection of various math functions 
* @copyright 2012-2019 Norbit Subsea. All rights reserved.
* @author Magnus Andersen (Magnus.Andersen@norbit.no)
*
*/

#include "cmath.h"
/** @cond */
#include <stdlib.h>
//#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <assert.h>
/** @endcond */


/***************************************************************************************/
/************ GENERAL MATH FUNCTIONS ***************************************************/
/***************************************************************************************/


double trigwrap(double x){
    return atan2(sin(x),cos(x));
}

/* wrap x -> [0,max) */
double wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}


/***************************************************************************//**
* @brief    Ceil of log2 of value 
*
* @param[in]    v       Input value, unsigned 32 bit value
* @return               ceil(log2(v))
******************************************************************************/
uint8_t ceil_log2(uint32_t v){
    uint8_t w = 0;
    if (v&0xFFFF0000){w+=16;v>>=16;}
    if (v&0xFFFFFF00){w+=8;v>>=8;}
    if (v&0xFFFFFFF0){w+=4;v>>=4;}
    if (v&0xFFFFFFFC){w+=2;v>>=2;}
    if (v&0xFFFFFFFE){w+=1;v>>=1;}
    if (v&0xFFFFFFFF){w+=1;v>>=1;}
    return w;
}
/***************************************************************************//**
* @brief    Ceil of log2 of abs of value 
*
* @param[in]    v       Input value, signed 32 bit value
* @return               ceil(log2(abs(v)))
******************************************************************************/
uint8_t ceil_log2_abs(int32_t v){
    uint8_t w = 0;
    v = v>0?v:-v;
    if (v&0xFFFF0000){w+=16;v>>=16;}
    if (v&0xFFFFFF00){w+=8;v>>=8;}
    if (v&0xFFFFFFF0){w+=4;v>>=4;}
    if (v&0xFFFFFFFC){w+=2;v>>=2;}
    if (v&0xFFFFFFFE){w+=1;v>>=1;}
    if (v&0xFFFFFFFF){w+=1;v>>=1;}
    return w;
}



float quadratic_interpolate(float x, const float* xs, const float* ys, size_t len) {
    assert(len >= 2);

    // Handle out-of-range x by clamping to endpoints (extrapolation optional)
    if (x <= xs[0])       return ys[0];
    if (x >= xs[len - 1]) return ys[len - 1];

    // Find the interval [xs[i], xs[i+1]] that contains x
    size_t i = 0;
    while (i + 1 < len && x > xs[i + 1]) {
        i++;
    }

    // Choose 3 points for quadratic interpolation:
    // Prefer one on each side, but clamp safely at ends
    size_t i0, i1, i2;
    if (i == 0) {
        i0 = 0; i1 = 1; i2 = 2;
    } else if (i >= len - 2) {
        i0 = len - 3; i1 = len - 2; i2 = len - 1;
    } else {
        i0 = i - 1; i1 = i; i2 = i + 1;
    }

    float x0 = xs[i0], y0 = ys[i0];
    float x1 = xs[i1], y1 = ys[i1];
    float x2 = xs[i2], y2 = ys[i2];

    // Lagrange form of quadratic interpolation
    float L0 = ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2));
    float L1 = ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2));
    float L2 = ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1));

    return y0 * L0 + y1 * L1 + y2 * L2;
}


/***************************************************************************//**
* @brief    Round up to nearest power of 2
* 1=1, 2=2, 3=4, 4=4, 5=8, ...etc
*
* @param[in]    v       Input value, unsigned 32 bit value
* @return               2^ (ceil(log2(v)))
******************************************************************************/
uint32_t upper_power_of_two(uint32_t v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}


/***************************************************************************//**
* @brief    Floating point log2 function
*           Logarithmic function with base 2
*
* @param[in]    x      Input value
* @return              log2(x) = log(x) * log2(e)
******************************************************************************/
/*float log2f(const float x){
    return  logf(x) * M_LOG2E;
}*/


#define QSSWAP(a,b) temp=(a);(a)=(b);(b)=temp;

float quickselect(float *arr, uint32_t n, uint32_t k) {
    uint32_t i,ir,j,l,mid;
    float a,temp;

    l=0;
    ir=n-1;
    for(;;) {
        if (ir <= l+1) { 
            if (ir == l+1 && arr[ir] < arr[l]) {
                QSSWAP(arr[l],arr[ir]);
            }
            return arr[k];
        }
        else {
            mid=(l+ir) >> 1; 
            QSSWAP(arr[mid],arr[l+1]);
            if (arr[l] > arr[ir]) {
                QSSWAP(arr[l],arr[ir]);
            }
            if (arr[l+1] > arr[ir]) {
                QSSWAP(arr[l+1],arr[ir]);
            }
            if (arr[l] > arr[l+1]) {
                QSSWAP(arr[l],arr[l+1]);
            }
            i=l+1; 
            j=ir;
            a=arr[l+1]; 
            for (;;) { 
                do i++; while (arr[i] < a); 
                do j--; while (arr[j] > a); 
                if (j < i) break; 
                QSSWAP(arr[i],arr[j]);
            } 
            arr[l+1]=arr[j]; 
            arr[j]=a;
            if (j >= k) ir=j-1; 
            if (j <= k) l=i;
        }
    }
}


float median(float *arr, uint32_t n){ 
    float* buf = malloc(n*sizeof(float));
    if (buf){
        memcpy(buf,arr,n*sizeof(float));
        return(quickselect(buf,n,n/2));
    }
    return 0;
}


#pragma pack(4)
/** Struct used to sort elements in median filter */
typedef struct med_lelement{
    float val;                       /**< Value of element */
    struct med_lelement* prev;    /**< Pointer to prev element in list */
    struct med_lelement* next;    /**< Pointer to next element in list */
}med_lelement_t;

/***************************************************************************//**
* @brief sliding window combined median / maximum filter
*
* @param[in]    in    Pointer to array of input values
* @param[out]   med   Pointer to array to store median values
* @param[in]    flen  Filter length, length of sliding window
* @param[in]    len   Data length, length of data (in, med)
*
* @note         Input data is padded with first and last element value in beginning and end, to generate same number of output as input values.
******************************************************************************/
int med_filter(const float* in, float* med, const size_t flen, const size_t len){
    med_lelement_t* list = malloc(flen*sizeof(med_lelement_t));
    size_t l;    //Next element index in list to be replaced
    size_t n;    //Input index, 
    size_t m;    //Output index, 
    size_t ii;   //Counter
    float val;
    med_lelement_t *top, *bottom;    //Pointer to highest and lowest value element in kernel
    med_lelement_t *el, *p;    

    if (list==NULL)
        return -1;
    //Initialize filter, fill entire list with first element, any order is sorted so just chain them together
    for(ii=0;ii<flen;ii++){
        list[ii].val=in[0];
    }
    top = &(list[0]);    //All elements the same, Any element can be top
    list[0].prev=NULL;
    list[0].next=&(list[1]);
    for(ii=1;ii<flen-1;ii++){
        list[ii].prev = &(list[ii-1]);
        list[ii].next = &(list[ii+1]);
    }
    bottom = &(list[flen-1]);    //All elements the same, Any element can be top
    list[flen-1].prev=&(list[flen-2]);
    list[flen-1].next=NULL;
    
    //Run filter (three stages)
    // -first flen/2         : Only read in data, dont generate output
    // -then len-(flen/2)    : Both read in data and generate output
    // -then flen/2            : Only generate output (pad input with last element)
    l=0;
    m=0;
    for(n=1;n<(len+flen/2);n++){
        val = in[MIN(n,len-1)];            //Value to be inserted
        el = &(list[l]);            //Element to be replaced

        //Remove element from linked list
        if (el->prev) el->prev->next = el->next; //If not topmost, update element above
        else top = el->next;                     // otherwise update top
        if (el->next) el->next->prev = el->prev; //If not bottommost, update element bellow
        else bottom = el->prev;                    // otherwise update bottom
        
        //Add element, sort-by-insertion
        if (val >= top->val){ //Insert as top
            el->next = top;
            el->prev = NULL;
            top->prev = el;
            top = el;
        }
        else if (val <= bottom->val){ //Insert as bottom
            el->next = NULL;
            el->prev = bottom;
            bottom->next = el;
            bottom = el;
        }
        else{                        //Insert in middle
            p=top->next;
            while(p->val > val) p = p->next; //Decend until p points on the first element with value less than el
            p->prev->next = el;
            el->prev = p->prev;
            el->next = p;                     //el is larger than p, so insert before
            p->prev = el;
        }
        //And set the value
        el->val = val;
        l = (l+1)%flen;

        if(n>=(flen/2)){
            //Generate output
            p = top;
            for (ii=0;ii<flen/2;ii++) p= p->next;
            med[m] = p->val;
            //max[m] = top->val;
            m++;
        }
    }
    free(list);
    return 0;
}

/***************************************************************************//**
* @brief Least-square-fit input data x,y with linear model y = ax+ b
*
* @param[in]    x             input x value vector
* @param[in]    y             input y value vector
* @param[in]    N             Length of x and y vector
* @param[out]    a            output value 1th order lin fit
* @param[out]    b            output value 0th order lin fit
*
* min   sum((yn - (a*xn + b))²) = sum( b²   +  a² xn²    +   2ab xn  -  2b yn  -   2a xn yn   +  yn²) 
* min   b² sum(1)    +  a² sum(xn²)  +   2ab sum(xn)  -  2b sum(yn)  -   2a sum(xn yn)  +  sum(yn²) 
* min   A b²  +  B a² +   2C ab  -  2D b   -   2E a  +  F
* A = sum(1) = N
* B = sum(x*x)
* C = sum(x)  
* D = sum(y)  
* E = sum(x*y)
* F = sum(y*y)
* min   p'Qp -2cp + F    where p = [b a]',  Q = [A C; C B] c = [D E]'   
* p = inv(Q)c
* b = 1/Qdet ( B*D - C*E)
* a = 1/Qdet (-C*D + A*E)
* Qdet = A*B-C*C
*
******************************************************************************/
void lin_fit(const float* x, const float* y, size_t N, float* a, float* b){
    float A,B,C,D,E, Qdet;
    float aa, bb;
    B=0;C=0;D=0;E=0;
    A = (float) N;
    for (size_t n = 0; n < N ;n++){
        B += x[n]*x[n];    
        C += x[n];                        
        D += y[n];                
        E += x[n]*y[n];
    }
    Qdet = A*B-C*C;
    aa = A*E-C*D;
    bb = B*D-C*E;
    if(Qdet==0) Qdet = 1; 
    *a = aa/Qdet;
    *b = bb/Qdet;
}

/***************************************************************************//**
* @brief  2D Least-square-fit input data x,y,z with linear model z = ax + by + c
*
* @param[in]    x             input x value vector
* @param[in]    y             input y value vector
* @param[in]    z             input y value vector
* @param[in]    N             Length of x,y and z vector
* @param[out]    a            output value 1th order lin fit
* @param[out]    b            output value 1th order lin fit
* @param[out]    c            output value 0th order lin fit
*
* min   sum((a*x + b*y + c - z)² ) = sum( a²x² + b²y² + c² + z² + 2abxy + 2acx + 2bcy -  2axz  - 2byz - 2cz) 
* min   a²sum(x²) + b²sum(y²) + c²sum(1) + sum(z²) + 2ab sum(xy) + 2ac sum(x) + 2bc sum(y) -2a sum(xz) -2b sum(yz) - 2c sum(z) 
* min   Aa² + Bb² + Cc² + D + 2Eab + 2Fac + 2Gbc - 2Ha -2Ib -2Jc
* A = sum(x²) 
* B = sum(y²)
* C = sum(1) = N  
* D = sum(z²)
* E = sum(x*y)
* F = sum(x)
* G = sum(y)
* H = sum(x*z)
* I = sum(y*z)
* J = sum(z)
* min   p'Qp -2rp + D   where p = [a b c]'  r = [H I J]',  Q = [A E F; E B G; F G C ]    
* d/dp => Qp -2r = 0
* p = 2*inv(Q)r
* Allthough inversing the Q matrix might not be the computationally most efficient way to solve this equation, 
* it probably does not matter, since the summing of the x,y,z arguments ie the main job
*
* Qadj = [U X Z ; X V Y ; Z Y W ]
* U = B*C-G*G
* V = A*C-F*F
* W = A*B-E*E
* X = F*G-C*E
* Y = E*F-A*G
* Z = E*G-B*F
* Qdet = A*U + E*X + F*Z
* Qinv = Qadj/Qdet
* a = (U*H + X*I + Z*J) / Qdet
* b = (X*H + V*I + Y*J) / Qdet
* c = (Z*H + Y*I + W*J) / Qdet
*
******************************************************************************/

//Assumes x_in and x_out to be sorted (non-decreasing order)

int non_uniform_1order_savgol(const float* x_in, const float* y_in, const size_t len_in, const float* x_out, float* y_out, const size_t len_out, const float flen){
   
    float* xx = (float*) malloc(len_in*sizeof(float));
    if (xx==NULL) return -1;
    const float* x = x_in;
    const float* y = y_in;
    float* xy = (float*) malloc(len_in*sizeof(float));
    if (xy==NULL) return -1;
   

    for (size_t ix=0 ; ix<len_in ; ix++){
        xx[ix] = x_in[ix] * x_in[ix]; 
        xy[ix] = x_in[ix] * y_in[ix]; 
    }
    //printf("non_uniform_1order_savgol\n");
    //For each output value
    size_t cix_in = 0;      //Center index in input array, index which x value is closest to x_out[ix_out]
    for(size_t ix_out = 0; ix_out < len_out; ix_out++){
        //Find closest index in input array
        while( (x_in[cix_in] < x_out[ix_out]) && (cix_in<(len_in-1))) cix_in++;                                 //Find first input index where x_in>x_out
        if ( (cix_in>0) && ( (x_in[cix_in]-x_out[ix_out]) > (x_out[ix_out]-x_in[cix_in-1]) ) ) cix_in--;    //Go one step back, if previous was closer
        //Find start of linfit section
        size_t six_in = cix_in;
        while ((six_in>0) && ( x_in[six_in] > (x_out[ix_out]-flen/2))) six_in--;
        //Find end of linfit section
        size_t pix_in = cix_in;
        while ((pix_in<(len_in-1)) && ( x_in[pix_in] < (x_out[ix_out]+flen/2))) pix_in++;
        //Need at least 2 points for each section
        while ((pix_in-six_in)<2){
            if (pix_in<(len_in-1)) pix_in+=1;
            if (six_in>0) six_in-=1;
            if ((six_in==0) && (pix_in==(len_in-1))) break;
        }
    
        //Do linfit over section
        double A = (double)(pix_in-six_in); //Number of samples in linfit
        double B=0;
        double C=0;
        double D=0;
        double E=0;
        for (size_t ix=six_in ; ix<pix_in ; ix++){
            B += xx[ix];
            C += x[ix];
            D += y[ix];
            E += xy[ix];
        }
        double Qdet = A*B-C*C;
        double aa = A*E-C*D;
        double bb = B*D-C*E;
        if(Qdet==0) Qdet = 1;
        double a = aa/Qdet;
        double b = bb/Qdet;
        
        //Calculate output value from linfit model
        y_out[ix_out] = a*x_out[ix_out] + b;
        //printf("x=%6.3f  n=%ld a=%6.3f, b=%6.3f AA=%f BB=%f Qdet=%f A=%f, B=%f, C=%f, D=%f, E=%f\n",x_out[ix_out],pix_in-six_in,a,b,aa,bb,Qdet,A,B,C,D,E);
    }
    return 0;
}


void hilbert_envelope_data(/*Input*/ float* sig_in, int32_t Nin, /*Output*/ float* sig_out){
    static const uint32_t hlen = 63;
    static const float hcoef[] = {    -0.0162738 ,  0.01472801, -0.01798631,  0.01331904, -0.01990413,
                                0.01202334, -0.02207802,  0.01082204, -0.02457624,  0.0096998 ,
                               -0.02749287,  0.00864398, -0.03096146,  0.00764404, -0.03517822,
                                0.00669104, -0.04044378,  0.00577731, -0.04724363,  0.00489618,
                               -0.05641605,  0.00404177, -0.06954423,  0.0032088 , -0.09002035,
                                0.00239247, -0.12666366,  0.00158833, -0.21181068,  0.00079219,
                               -0.63648784,  0.        ,  0.63648784, -0.00079219,  0.21181068,
                               -0.00158833,  0.12666366, -0.00239247,  0.09002035, -0.0032088 ,
                                0.06954423, -0.00404177,  0.05641605, -0.00489618,  0.04724363,
                               -0.00577731,  0.04044378, -0.00669104,  0.03517822, -0.00764404,
                                0.03096146, -0.00864398,  0.02749287, -0.0096998 ,  0.02457624,
                               -0.01082204,  0.02207802, -0.01202334,  0.01990413, -0.01331904,
                                0.01798631, -0.01472801,  0.0162738 };
                                
    for (uint32_t ix = 0; ix < (hlen/2); ix++)
        sig_out[ix] = 0;
    
    for (uint32_t ix = hlen/2; ix < (Nin-hlen/2); ix++){
        float I = sig_in[ix];
        float Q = 0;
        for (uint32_t hix = 0; hix < hlen; hix++){
            Q += sig_in[ix - hlen/2 + hix]  *  hcoef[hix];
        }
        sig_out[ix] = sqrtf(I*I + Q*Q);
    }
    for (uint32_t ix = Nin-(hlen/2); ix < Nin; ix++)
        sig_out[ix] = 0;
}

static uint32_t calc_match_filter(float fc,float bw,float len, float Fs, /*output*/ float* mfir){
    float t,df,dfdn,dtdn;
    uint32_t N,n;
    N = (uint32_t) (len*Fs);

    dtdn = (1./Fs);
    t=-len/(2.*Fs);

    df = -bw/2;
    dfdn =  bw/(2.*len*Fs);
    
    for (n=0;n<N;n++){
        float apz_val = 1.f;//(0.54+0.46*Q_cos((2.*M_PI*(n+0.5))/N-M_PI));
        mfir[n] = sinf(2.*M_PI*(fc+df)*t) * apz_val;
        df += dfdn;
        t += dtdn; 
    }
    //Remove DC in filter
    float dc = 0;
    for (n=0;n<N;n++)
        dc+=mfir[n];
    dc/=N;
    for (n=0;n<N;n++)
        mfir[n] -= dc;
    //printf("Calc match filter: freq %1.2fkHz bw %1.2fkHz  len %1.0f us Fs = %1.2fkHz %d taps\n",fc/1000,bw/1000,len*1e6,Fs/1000,N);
    return N;
}

void match_filter_data(/*Input*/ float* sig_in, float freq, float bw, float plen, float Fs, int32_t Nin, /*Output*/ float* sig_out){
    uint32_t mfir_len = (uint32_t) (plen*Fs);
    float* mfir = malloc(mfir_len * sizeof(float));
    calc_match_filter(freq,bw,plen, Fs, /*output*/ mfir);

    for (uint32_t ix = 0; ix < (mfir_len/2); ix++)
        sig_out[ix] = 0;
    for (uint32_t ix = mfir_len/2; ix < (Nin-mfir_len/2); ix++){
        sig_out[ix] = 0;
        for (uint32_t mix = 0; mix < mfir_len; mix++){
            sig_out[ix] += sig_in[ix - mfir_len/2 + mix]  *  mfir[mix];
        }
    }
    for (uint32_t ix = Nin-(mfir_len/2); ix < Nin; ix++)
        sig_out[ix] = 0;

    free(mfir);
}

void bp_filter_data(/*Input*/ float* sig_in, float freq, float bw, float Fs, int32_t Nin, /*Output*/ float* sig_out){
    bw = LIMIT(bw,0,Fs);
    uint32_t bpfir_len = (uint32_t) (2*Fs/bw);
    float* bpfir = malloc(bpfir_len * sizeof(float));
    
    /*Windowed sinc with hamming window*/
    float fcn = freq/Fs;     //Normalized center freq 
    float bwn = bw/2/Fs;     //Normalized cutoff freq
    //Sinc function
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        float phi = 2*M_PI*bwn*ii;
        bpfir[ix] = phi==0?1.0f:sinf(phi)/phi;
    }
    //Adding window function (hamming)
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        bpfir[ix] *= (0.54 + 0.46*cos((2.*M_PI*ii)/bpfir_len));
    }
    //Scale to unity DC gain
    float bpfir_scale = 0;
    for (size_t ix = 0; ix<bpfir_len; ix++)
        bpfir_scale += bpfir[ix];
    bpfir_scale = 1./bpfir_scale;
    for (size_t ix = 0; ix<bpfir_len; ix++)
        bpfir[ix] *= bpfir_scale;


    //Mixing up to center freq
    for (size_t ix = 0; ix<bpfir_len; ix++){
        float ii = (float) ix-(0.5*(bpfir_len-1));
        float phi = 2*M_PI*fcn*ii;
        bpfir[ix] *= cosf(phi);
    }

    
    for (uint32_t ix = 0; ix < (bpfir_len/2); ix++)
        sig_out[ix] = 0;
    for (uint32_t ix = bpfir_len/2; ix < (Nin-bpfir_len/2); ix++){
        sig_out[ix] = 0;
        for (uint32_t mix = 0; mix < bpfir_len; mix++){
            sig_out[ix] += sig_in[ix - bpfir_len/2 + mix]  *  bpfir[mix];
        }
    }
    for (uint32_t ix = Nin-(bpfir_len/2); ix < Nin; ix++)
        sig_out[ix] = 0;

    free(bpfir);
}







