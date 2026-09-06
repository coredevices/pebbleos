/* Project Kraepelin, Main file
The MIT License (MIT)

Copyright (c) 2015, Nathaniel T. Stockham

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

This license is taken to apply to any other files in the Project Kraepelin
Pebble App project.
*/


#include "raw_stats.h"
#include <pbl/logging/logging.h>


/* RAW STATISTICS */


// TEST : PASSED
int32_t mean_l1_stat(int16_t *d, int16_t dlen){
  int32_t mean = 0;

  for(int16_t i = 0; i < dlen; i++ ){
    mean += d[i];
  }
  /* We don't overflow protection the mean output because the  values can
  * exceed 125 over the course of a second  */
  return mean/dlen; // divide out # samples to get mean
}

// TEST : NONE
int32_t x100_mean_l1_stat(int16_t *d, int16_t dlen){
  int32_t mean = 0;

  for(int16_t i = 0; i < dlen; i++ ){
    mean += d[i];
  }
  /* We don't overflow protection the mean output because the  values can
  * exceed 125 over the course of a second  */
  return (mean*100)/dlen; // divide out # samples to get mean
}

// https://en.wikipedia.org/wiki/Fixed-point_arithmetic
static __inline__ sfxp fxp_mul(sfxp a, sfxp b){
  // FORMAT Q31.32, so the integers are signed and the fractional parts
  // are unsigned

  int32_t ai, bi; // need this so that the negative bit is in the right place.
  ai = a >> 32;
  bi = b >> 32;

  uint32_t af, bf;
  af = (uint32_t) a; // just get the bottom elements, assume positive
  bf = (uint32_t) b; // ibid

  // have to add all the elements as unsigned types, because otherwise
  // C will try to interpret the high bit of the low 32 bits of the fractional
  // segment as a sign, and cause substraction, when all we really want is to
  // assume all elements are positive, and then encode the sign as the highest
  // bit of var "si",
	ufxp si, uf, uif;

  // multiply the integers, and use them to store the sign in the highest bit.
  // but, we also need to encode overflow.
  si = ((ufxp)(ai*bi)) << 32;
  // multiply the fractional only part, and store them
  uf = (((ufxp)af)*((ufxp)bf)) >> 32;
  // we can convert the fractional part safely to unsigned long long because we
  // know that the range is <= 63 bit, as these are 32 and 31 bit integers multi

  uif = ((ai)*((ufxp)bf)) + (((ufxp)af)*(bi));

	return (sfxp) (si + uf + uif);
}


// we statically encode the coefficients because we only need this
// filter to run fast. ALSO, MUST initialize to 0
static sfxp yt[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
static sfxp xt[5][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

void pim_filt_init(void) {
  memset(yt, 0, sizeof(yt));
  memset(xt, 0, sizeof(xt));
}

// -----------------------------------------------------------------------------------------
// Prime the butterworth filter used in the pim filter. This helps reduce the high VMC
// produced from the first set of samples fed in right after the algorithm has been initialized.
// It works by priming the butterworth filter with an odd-symmetric extension of the first few
// samples. These priming samples have roughly the same frequency characteristics as
// the first set of samples. Since the butterworth filter's memory is 5 samples, the priming
// sequence must be at least 10 long.
// If the first few real samples are:
//    10, 13, 9, 15, 6, ...
// Then the priming samples would be:
//    14, 5, 11, 7
// The value for priming sample i, based on N is:
//    p[i] = x[0] - (x[N-1-i] - x[0]) - x[0]
//    p[i] = 2 * x[0] - x[N-1-i]
// So, in the above example, when N is 5:
//    p[0] = 2 * 10 - 6 = 14
//    p[1] = 2 * 10 - 15 = 5
//    p[2] = 2 * 10 - 9 = 11
//    p[3] = 2 * 10 - 13 = 7
void pim_filt_prime(int16_t *d, int16_t dlen, int16_t axis) {
  const int n = 11;
  int16_t prime_data[n];

  for (int i = 0; i < n; i++) {
    prime_data[i] = 2 * d[0] - d[n - 1 - i];
  }
  pim_filt(prime_data, n - 1, axis);
}


// // TEST : PASSED
uint32_t pim_filt(int16_t *d, int16_t dlen, int16_t axis){
// uint32_t pim_filt(int16_t *d, int16_t dlen, int16_t axis){
  // we use a butterworth second order digital filter with a bandpass
  // design of 0.25 to 2 hz, with coefficients double *ca_d,double *cb_dof

  // and, we calculate all of these by multiplying by 100000x
  int32_t x1000_thres = 3750; // this is calibrated to pebble, 125 = 1G
  // the pim
  int32_t pim=0;
  sfxp ytmp=0;

  //25hz @ 0.25 to 2hz
  sfxp ca[4] = {0xfffffffc92b0910cLL, 0x0000000473f9a693LL, 0xfffffffd633c7d23LL, 0x0000000096405b5cLL};
  sfxp cb[5] = {0x000000000721d150LL, 0x0000000000000000LL,  0xfffffffff1bc5d60LL, 0x0000000000000000LL, 0x000000000721d150LL};
  //25hz @ 0.25 to 2.5hz
//   sll ca[4] = {0xfffffffcd72d69bdLL, 0x00000003caed5dc8LL, 0xfffffffdeadc79a0LL, 0x0000000073506446LL};
//   sll cb[5] = {0x000000000e73680bLL, 0x0000000000000000LL, 0xffffffffe3192feaLL, 0x0000000000000000LL, 0x000000000e73680bLL};

  for(int16_t i = 0; i < dlen; i++){
    // shift the input over by one
    for(int16_t k = 4; k >0;k-- ){ xt[k][axis] = xt[k-1][axis];}
    xt[0][axis] = int2sll(d[i]);

    // get the next output element, ympt
    ytmp = ( ( fxp_mul(cb[0],xt[0][axis])
              + fxp_mul(cb[1],xt[1][axis])
              + fxp_mul(cb[2],xt[2][axis])
              + fxp_mul(cb[3],xt[3][axis])
              + fxp_mul(cb[4],xt[4][axis]))
            - (fxp_mul(ca[0],yt[0][axis])
               + fxp_mul(ca[1],yt[1][axis])
               + fxp_mul(ca[2],yt[2][axis])
               + fxp_mul(ca[3],yt[3][axis])) );

    // shift the y output elements
    for(int16_t k = 3; k >0;k-- ){ yt[k][axis] = yt[k-1][axis];}

    // update the y output element and prevent overflow and unrealistic outputs (yt[0]<= xt[0] always)
    yt[0][axis] = ytmp;
    pim += abs( (int32_t)sll2int(ytmp) );
  }
  // REMEMBER, the scoring is done on the 1 SECOND level, so we
  // ONLY do thresholding at the 1 second level.
  return (uint32_t) ( (pim - (x1000_thres*dlen)/1000>0) ? (pim - (x1000_thres*dlen)/1000 ) : 0) ;
}



// TEST : PASSED
uint32_t calc_scaled_vmc(uint32_t *pim_ary){
  // copy over the elements

  // This function calculates the VMCPM from the PIM array for each
  // epoch.
  // INPUTS
  //   *pim_ary -> the actual array of cpm for each axis
  //   oflw_scl -> dividing factor to reduce pim_ary so doesn't overflow
  // PARAMETERS
  //   oflw_cap -> cap to prevent overflow when pim_ary[:].^2 is summed
  // OUTPUT
  //  @ return -> the sqrt of the scaled vmcpm

  uint32_t d[3];
  uint32_t oflw_cap = 37500;
  uint32_t VMCPM_SCL = 10;
  for(int16_t axis = 0; axis < 3; axis++){

    d[axis] = pim_ary[axis]/VMCPM_SCL;
    d[axis] = (d[axis] < oflw_cap) ? d[axis] : oflw_cap;
  }

  /* VMCPM = sqrt(pim_x^2 + pim_y^2 + pim_z^2)*/
  // calculate VMCPM, then take sqrt to compress for storage
  return (isqrt( d[0]*d[0] + d[1]*d[1] + d[2]*d[2])*VMCPM_SCL) ;
}

uint8_t compressed_vmc(uint32_t *pim_ary){
  return (uint8_t) isqrt(calc_real_vmc(pim_ary) );
}

uint32_t calc_real_vmc(uint32_t *pim_ary){

  uint32_t scl_vmcpm = calc_scaled_vmc(pim_ary);
  // we first assume the PIM is calculated on 1 = 1G, then we scale
  // to x10 the real vmcpm, THEN we divide by 1250 cause 125*10, and
  // we divide the PIM calculated on 125 = 1G so we FINALLY get the
  // *real* VMCPM given that we calculated on the Pebble accel where
  // 1000/8 = 125 = 1G.
  return (scl_vmcpm*x100_RAW_1G_PIM_CPM_TO_REAL_CPM)/12500;
}

// TEST : PASSED
uint32_t calc_real_c(uint32_t pim){
  // This function calculates the CPM from the PIM array for each
  // epoch.
  // INPUTS
  // *pim_ary -> the actual array of cpm for each axis

  // we first assume the PIM is calculated on 1 = 1G, then we scale
  // to x10 the real cpm, THEN we divide by 1250 cause 125*10, and
  // we divide the PIM calculated on 125 = 1G so we FINALLY get the
  // *real* VMCPM given that we calculated on the Pebble accel where
  // 1000/8 = 125 = 1G.
  return (pim*x100_RAW_1G_PIM_CPM_TO_REAL_CPM)/12500;
}



uint32_t calc_x1000_kcal(uint32_t *r_c_ary){
  int32_t x1000_kcal = 0;
  int32_t we_c_ary[3]; // wrist equivalent counts
  int32_t r_c;

  // update the general config 
  struct config_general cur_config = {
    .pweight_kg = 70,
  };
#if PEBBLE_APP
  persist_read_data(CONFIG_GENERAL_PERSIST_KEY,&cur_config,sizeof(cur_config));
#endif
  int32_t pweight_kg = (int32_t) cur_config.pweight_kg;
  // int32_t pweight_kg = 0;
  // CONVERT WRIST COUNTS INTO EQUIVALENT COUNTS AS SPECIFIED BY ACTIGRAPH @
  // https://help.theactigraph.com/entries/21187041-What-does-the-Worn-on-Wrist-option-do-in-the-Data-Scoring-tab-

  for(int16_t axis = 0; axis < 3; axis++){
    r_c = (int32_t) r_c_ary[axis];
    if( r_c < 644){
      we_c_ary[axis] = 534*r_c;
    }else if((r_c >= 645) && (r_c < 1272)){
      we_c_ary[axis] = 1713*r_c - 759414;
    }else if((r_c >= 1273) && (r_c < 3806)){
      we_c_ary[axis] = 400*r_c + 911501;
    }else if(r_c >= 3806){
      we_c_ary[axis] = 13*r_c + 2383904;
    }
    we_c_ary[axis] = we_c_ary[axis]/1000;
  }

  // get the equivalent scaled vmc
  // NOTE: We have already scaled to actual count values, so we don't
  // need to worry about overflow on the uint32_t.
  uint32_t we_vmc  = isqrt(we_c_ary[0]*we_c_ary[0] + we_c_ary[1]*we_c_ary[1]
                           + we_c_ary[2]*we_c_ary[2] );


  // NOTE!!, this assumes the equation from Freedson VM2 11'
  //     if VMCPM > 2453
  //   Kcals/min= 0.001064×VM + 0.087512(BM) - 5.500229
  //     else
  //   Kcals/min=CPM×0.0000191×BM
  //
  //     where
  // VM = Vector Magnitude Combination (per minute) of all 3 axes (sqrt((Axis 1)^2+(Axis 2)^2+(Axis 3)^2])
  // VMCPM = Vector Magnitude Counts per Minute
  // CPM = Counts per Minute
  // BM = Body Mass in kg

  // NOTE, we have to scale ALL of these things by 10000, and since we
  // know that each element of the pim_ary maxes out at around 375000,
  // we know that the upper bound of cpm is 3*400,000 = 1,200,000, so
  // to stay under the 4,294,967,296 overflow cap, we can only go to
  // a factor of 10,000. BUT, problems still, cause we need to selectively
  // decide which ones we are going to add/subtract. We WILL assume
  // that the output of this function will be (kcal/min)*1000, which
  // is also captured in the worker's function, cause we don't
  // want to have big rounding down errors when we are at basal metabolic
  // rate estimates.

  if(we_vmc > 2453){
    //     if VMCPM > 2453
    //   Kcals/min= 0.001064×VM + 0.087512(BM) - 5.500229
    x1000_kcal = (1064*((int32_t) we_vmc))/1000 + (875*pweight_kg)/10 - 5500;
  }else{
    // Kcals/min=CPM×0.0000191×BM
    x1000_kcal = (we_c_ary[0]*19*pweight_kg)/1000;
  }
  return (uint32_t) abs(x1000_kcal);
}

uint8_t compressed_x1000_kcal(uint32_t x1000_kcal, int16_t num_min){
  // we assume that we can't get more than 17 kcal per min
  // assume max is 1000 calories per hour, so ~17 cal min max, so multiply
  // by number of minutes, then scale & sqrt it so it fits within the uint8
  uint32_t max_kcal = 17*num_min;
  return (uint8_t) isqrt( ((x1000_kcal/1000)*65535)/max_kcal );
}



uint8_t compressed_stepc(uint32_t stepc, int16_t num_min){
  // This function simply takes in the number of steps in a given number of
  // minutes. Then, we calculate the maximum number of steps possible in num_min
  // given a max of 4hz running speed. Then, that max is turned in a scaling
  // factor that the step count is premultiplied by, and then the sqrt is taken
  uint32_t max_stepc = 4*60*num_min; // @ 4hz * 60secs * num_min
  return (uint8_t) isqrt( (stepc*65535)/max_stepc) ;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++ SIGNALS ANALYSIS +++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



// TEST : PASSED
void vm_accel(int16_t **d, int16_t *w, int16_t max_vm, int16_t dlen){
  /* EVALUATE THE VECTOR ACCELERATION MAGNITUDE, WRITE TO first array */
  // NOTE: we can look at the entire array, because we only write to
  // the first N_SMP_EPOCH, the remainder being 0's, which will
  // evaluate to a vector mag of 0
  for(int16_t i = 0; i < dlen; i++){
    // evaluate the vector mag of acceleration, write to work array w
    w[i] = (int16_t) isqrt( d[0][i]*d[0][i] + d[1][i]*d[1][i] + d[2][i]*d[2][i] );
    //     w[i] = (int16_t) ( (d[0][i]*d[0][i]) + (d[1][i]*d[1][i]) + (d[2][i]*d[2][i]) );
    //  cap the vector mags as max_vm to prevent overflow
    w[i] = ((abs(w[i]) <= max_vm) ? w[i] : ((w[i] > 0 ) ? 1 : (-1))*max_vm);
  }
}

// TEST : PASSED
void vm_accel_xy(int16_t **d, int16_t *w, int16_t max_vm, int16_t dlen){
  /* EVALUATE THE VECTOR ACCELERATION MAGNITUDE, WRITE TO first array */
  // NOTE: we can look at the entire array, because we only write to
  // the first N_SMP_EPOCH, the remainder being 0's, which will
  // evaluate to a vector mag of 0
  for(int16_t i = 0; i < dlen; i++){
    // evaluate the vector mag of acceleration, write to work array w
    w[i] = (int16_t) isqrt( d[0][i]*d[0][i] + d[1][i]*d[1][i]);
    //     w[i] = (int16_t) ( (d[0][i]*d[0][i]) + (d[1][i]*d[1][i]) + (d[2][i]*d[2][i]) );
    //  cap the vector mags as max_vm to prevent overflow
    w[i] = ((abs(w[i]) <= max_vm) ? w[i] : ((w[i] > 0 ) ? 1 : (-1))*max_vm);
  }
}



// TEST : PASSED
int16_t score_fft_alg_0pad(int16_t *d, int16_t dlen_smp, int16_t dlenpwr_ary, int16_t oflw_scl){
  // NOTE !!! WE ASSUME THE d INPUT IS ALREADY SCALED TO PREVENT OVERFLOW
  // AND THAT IS MEAN-PADDED
  /* PARAMETERS */
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);
  int16_t lhz_i = 3;
  int16_t hhz_i = 10;

  // reduce the scale
  for(int16_t i = 0; i < dlen_smp; i++){
    d[i] = d[i]/oflw_scl;
  }

  // set the last few elements to the mean of the first dlen_smp elements
  int16_t mean = mean_l1_stat(d, dlen_smp);
  for(int16_t i = dlen_smp ; i < dlen_ary; i++){
    d[i] = mean;
  }

  /* EVALUATE THE MAGNITUDE OF THE FFT COEFFICIENTS AND WRITE TO d ARRAY */
  fft_2radix_real(d, dlenpwr_ary);
  fft_mag(d, dlenpwr_ary);

  /* EVALUATE THE STEP SCORE FOR EPOCH */
  // -> This section can be fairly complex if needed
  // When remove the mean, the DC is zero, so we add the DC value back
  // -> (mean * length of array)
  //  to evaluate the 4-20hz integral against the total integral
  return (100*(integral_abs(d, lhz_i, hhz_i)))/(integral_abs(d, 0, dlen_ary/2));
}



void get_fftmag_0pad(int16_t *d, int16_t dlen_smp, int16_t dlenpwr_ary, int16_t oflw_scl){
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);

  // reduce the scale
  for(int16_t i = 0; i < dlen_smp; i++){
    d[i] = d[i]/oflw_scl;
  }

  // set the last few elements to the mean of the first dlen_smp elements
  int16_t mean = mean_l1_stat(d, dlen_smp);
  for(int16_t i = dlen_smp ; i < dlen_ary; i++){
    d[i] = mean;
  }

  /* EVALUATE THE MAGNITUDE OF THE FFT COEFFICIENTS AND WRITE TO d ARRAY */
  fft_2radix_real(d, dlenpwr_ary);
  fft_mag(d, dlenpwr_ary);

}


void get_fftmag_0pad_mean0(int16_t *d, int16_t dlen_smp, int16_t dlenpwr_ary, int16_t oflw_scl){
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);

  // reduce the scale
  for(int16_t i = 0; i < dlen_smp; i++){
    d[i] = d[i]/oflw_scl;
  }

  // set the last few elements to the mean of the first dlen_smp elements
  int16_t mean = mean_l1_stat(d, dlen_smp);
  for(int16_t i = 0 ; i < dlen_smp; i++){
    d[i] = d[i] - mean;
  }
  for(int16_t i = dlen_smp ; i < dlen_ary; i++){
    d[i] = 0;
  }

  /* EVALUATE THE MAGNITUDE OF THE FFT COEFFICIENTS AND WRITE TO d ARRAY */
  fft_2radix_real(d, dlenpwr_ary);
  fft_mag(d, dlenpwr_ary);

}


uint16_t score_fftmag_hz_rng_abs(int16_t *d, int16_t dlenpwr_ary,int16_t lhz_i,int16_t hhz_i){
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);

  return (100*(integral_abs(d, lhz_i, hhz_i)))/(integral_abs(d, 0, dlen_ary/2));
}

uint16_t score_fftmag_hz_rng_l2(int16_t *d, int16_t dlenpwr_ary,int16_t lhz_i,int16_t hhz_i){
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);

  return (100*(integral_l2(d, lhz_i, hhz_i)))/(integral_l2(d, 0, dlen_ary/2));
}

int16_t score_fftmag_max_hz_harm(int16_t *d, int16_t dlenpwr_ary,int16_t max_mag_hz){
  int16_t dlen_ary = pow_int(2,dlenpwr_ary);
  int16_t next_max_mag_hz = (d[max_mag_hz+1] > d[max_mag_hz-1]) ? (max_mag_hz+1) : (max_mag_hz-1);

  int32_t num = 0;
  for(int16_t i = 1; i < 4 ; i++){
    if( max_mag_hz+1 <= (dlen_ary/2)){
      num += ((d[max_mag_hz*i]) + (d[max_mag_hz*i])); // highest harmonics
    }
  }
  return ((100*num) / integral_abs(d, 1, dlen_ary/2) );
}



void filt_hann_win_mean0(int16_t *d, int16_t dlen_smp, int32_t g_fctr){
  int32_t d_mean = mean_l1_stat(d,dlen_smp);
  // g_fctr = gain factor
  // d[i] = d[i]*w[i]
  // -> w[i] = 0.5*(1 - cos(2*pi*i/dlen_smp)) ::: i = 0:dlen_smp
   for( uint16_t i = 0; i < dlen_smp; i++ ){
     d[i] = (int16_t) ( ((d[i] - d_mean)*g_fctr*(TRIG_MAX_RATIO
                          - cos_lookup((TRIG_MAX_ANGLE*i)/dlen_smp)) ) / (2*TRIG_MAX_RATIO));
   }
}

void filt_cosine_win_mean0(int16_t *d, int16_t dlen_smp, int32_t g_fctr){
  int32_t d_mean = mean_l1_stat(d,dlen_smp);
  // g_fctr = gain factor
  // d[i] = d[i]*w[i]
  // -> w[i] = 0.5*(1 - cos(2*pi*i/dlen_smp)) ::: i = 0:dlen_smp
   for( uint16_t i = 0; i < dlen_smp; i++ ){
     d[i] = (int16_t) ( ((d[i] - d_mean)*g_fctr*
      sin_lookup((TRIG_MAX_ANGLE*i)/(2*dlen_smp)) ) / (TRIG_MAX_RATIO));
   }
}


// TEST : PASSED
int16_t max_mag_hz_0pad(int16_t *d){
  // NOTE !!! WE ASSUME THE d INPUT IS ALREADY THE FFT MAG
  int16_t lhz_i = 3;
  int16_t hhz_i = 25;
  // int16_t is_step = 0; // assume it is not a step period

  // evaluate if the period is a step epoch, based on score
  uint32_t max_hz_val = 0;
  int16_t max_hz_i = 0; // if NOT a step period, then return 0 for no steps
  // if it is a step epoch, find the hz index with largest mag
  uint32_t test_val = 0;
  for(int16_t i = lhz_i; i <= hhz_i; i++){
    test_val = (abs(d[i])*(hhz_i - i)*100 )/ 100;
    // test_val = d[i];
    if(test_val > max_hz_val ){
      max_hz_val = test_val;
      max_hz_i = i;
    }
  }
  /* RETURN NUMBERS OF STEPS IN EPOCH */
  return max_hz_i; // DC index is 0, so max_hz_i is HZ directly
}

int16_t calc_stepc_5sec(int16_t *work_ary, int16_t dlen_smp, int16_t dlenpwr_ary,
  uint32_t *pim_5sec_ary, int16_t fft_oflw_scl){
    // get the mean acceleration to serve as minimum thresholds for running and
    // walking
    uint32_t real_vmc_5s = calc_real_vmc(pim_5sec_ary);
    
    // struct config_general cg = get_config_general();
    // Values copied from reset_config_general_persistent_storage() in helper.c
    struct config_general cg = (struct config_general) {
      .stepc_fft_thres0 = {22, 100},
      .stepc_vma_thres0 = 0, 
      .stepc_vmc_thres0 = 50,
      .wear_class_thres = 290 * 12 * 10,
      .pts_goal = 10000,
    };

    // filter the result
    filt_cosine_win_mean0(work_ary, dlen_smp, 5);
    get_fftmag_0pad_mean0(work_ary, dlen_smp, dlenpwr_ary, fft_oflw_scl);

    int16_t max_mag_hz = max_mag_hz_0pad(work_ary);

    int16_t score0 = score_fftmag_max_hz_harm(work_ary, dlenpwr_ary, max_mag_hz);
    // uint8_t score0 = score_fftmag_hz_rng_abs(work_ary, dlenpwr_ary, max_mag_hz-1, max_mag_hz+1);
    int16_t stepc_tmp = 0;
    // NOTE!, use a simple linear regression to scale the fft_threshold with the
    // vmc. Actually, this is quite computationally sound, we just need to shift
    // it over by a few for safety, and we can auto adjust the parameters so that
    // they can be *very* tight. This way, we can reject steps very easily.
    if( (score0 >= cg.stepc_fft_thres0[0])
      && (real_vmc_5s >= cg.stepc_vmc_thres0)
      && (max_mag_hz < 40) ){
        // we subtract by an expected 0.5 cause the window truncation and
        //  integer fft approximation causes to push up a half frequency  ~0.1Hz.
      // stepc_tmp = max_mag_hz - ((uint16_t) (rand()%2));
      stepc_tmp = max_mag_hz;
    }
  PBL_LOG_DBG("steps: %d, freq: %d, vmc: %d, score: %d", stepc_tmp, max_mag_hz,
          real_vmc_5s, score0);
  return stepc_tmp;
}

//
// int16_t calc_stepc_5sec(int16_t *work_ary, int16_t dlen_smp, int16_t dlenpwr_ary,
//   uint32_t *pim_5sec_ary, int16_t fft_oflw_scl){
//     // get the mean acceleration to serve as minimum thresholds for running and
//     // walking
// //     int32_t x100_mean_vm_accel = x100_mean_l1_stat(work_ary, dlen_smp);
//     uint32_t real_vmc_5s = calc_real_vmc(pim_5sec_ary);
//
// //     persist_write_int(DAILY_x1000_KCAL_PERSIST_KEY,real_vmc_5s*1000);
//
//
//     struct config_general cg = get_config_general();
//
//     // filter the result
//     // filt_cosine_win_mean0(work_ary, dlen_smp, 1);
//
//     get_fftmag_0pad_mean0(work_ary, dlen_smp, dlenpwr_ary, fft_oflw_scl);
//
//     uint16_t max_mag_hz = max_mag_hz_0pad(work_ary);
//
//     //uint8_t score0 = score_fftmag_hz_rng_l2(work_ary, dlenpwr_ary, max_mag_hz-1, max_mag_hz+1);
//     // uint8_t score0 = score_fftmag_hz_rng_abs(work_ary, dlenpwr_ary, max_mag_hz, max_mag_hz);
//     uint8_t score0 = score_fftmag_hz_rng_abs(work_ary, dlenpwr_ary, max_mag_hz-1, max_mag_hz+1);
//
//     uint16_t stepc_tmp = 0;
//     // NOTE!, use a simple linear regression to scale the fft_threshold with the
//     // vmc. Actually, this is quite computationally sound, we just need to shift
//     // it over by a few for safety, and we can auto adjust the parameters so that
//     // they can be *very* tight. This way, we can reject steps very easily.
//     if(
//         ((score0 >= cg.stepc_fft_thres0[0]) && (score0 <= cg.stepc_fft_thres0[1])
//           && (real_vmc_5s >= cg.stepc_vmc_thres0)&&(real_vmc_5s < cg.stepc_vmc_thres1))
//         || ((score0 >= cg.stepc_fft_thres1[0]) && (score0 <= cg.stepc_fft_thres1[1])
//           && (real_vmc_5s >= cg.stepc_vmc_thres1)&&(real_vmc_5s < cg.stepc_vmc_thres2 ))
//         || ((score0 >= cg.stepc_fft_thres2[0]) && (score0 <= cg.stepc_fft_thres2[1])
//           && (real_vmc_5s >= cg.stepc_vmc_thres2)&&(real_vmc_5s < cg.stepc_vmc_thres3))
//         || ((score0 >= cg.stepc_fft_thres3[0]) && (score0 <= cg.stepc_fft_thres3[1])
//           && (real_vmc_5s >= cg.stepc_vmc_thres3 ) )
//       ){
//         // we subtract by an expected 0.5 cause the window truncation and
//         //  integer fft approximation causes to push up a half frequency  ~0.1Hz.
//       // stepc_tmp = max_mag_hz - ((uint16_t) (rand()%2));
//       stepc_tmp = max_mag_hz;
//     }
//   return stepc_tmp;
// }


// if(
//     ((score0 >= cg.stepc_fft_thres0)
//       && (real_vmc_5s >= cg.stepc_vmc_thres0)&&(real_vmc_5s < cg.stepc_vmc_thres1))
//     || ((x100_mean_vm_accel >= cg.stepc_vma_thres0)
//         && (((score0 >= cg.stepc_fft_thres1)
//             && (real_vmc_5s >= cg.stepc_vmc_thres1)&&(real_vmc_5s < cg.stepc_vmc_thres2 ))
//           || ((score0 >= cg.stepc_fft_thres2)
//             && (real_vmc_5s >= cg.stepc_vmc_thres2)&&(real_vmc_5s < cg.stepc_vmc_thres3))
//           || ((score0 >= cg.stepc_fft_thres3) && (real_vmc_5s >= cg.stepc_vmc_thres3 ) )))
//   ){
//
// if(
//     ((score0 >= cg.stepc_fft_thres0)
//       && (real_vmc_5s >= cg.stepc_vmc_thres0)&&(real_vmc_5s < cg.stepc_vmc_thres1))
//     || ( ((score0 >= cg.stepc_fft_thres1)
//             && (real_vmc_5s >= cg.stepc_vmc_thres1)&&(real_vmc_5s < cg.stepc_vmc_thres2 ))
//           || ((score0 >= cg.stepc_fft_thres2)
//             && (real_vmc_5s >= cg.stepc_vmc_thres2)&&(real_vmc_5s < cg.stepc_vmc_thres3))
//           || ((score0 >= cg.stepc_fft_thres3) && (real_vmc_5s >= cg.stepc_vmc_thres3 ) ))
//   ){






// /* RAW STATISTICS */
//
//
// // TEST : PASSED
// int32_t mean_l1_stat(int16_t *d, int16_t dlen){
//   int32_t mean = 0;
//
//   for(int16_t i = 0; i < dlen; i++ ){
//     mean += d[i];
//   }
//   /* We don't overflow protection the mean output because the  values can
//     * exceed 125 over the course of a second  */
//   return mean/dlen; // divide out # samples to get mean
// }
//
//
// // TEST : PASSED
// int32_t pim_filt(int16_t *d, int16_t dlen){
//   /* This function calculates the pim of the given array, single axis
//   * construction and application of filters are contained with the function */
//   int32_t pim_local = 0;
//
//   /* calculate the filter */
//   // the mean
//   int32_t mean = mean_l1_stat(d, dlen);
//
//   /* apply the filter */
//   // remove the mean
//   for(int16_t i = 0; i < dlen; i++){
//     pim_local += abs(d[i] - mean);
//   }
//
//   /* Return the local pim for given axis */
//   return pim_local;
// }
//
//
// // TEST : PASSED
// uint32_t calc_scaled_vmc(uint32_t *pim_ary, uint32_t oflw_scl){
//   // This function calculates the VMCPM from the PIM array for each
//   // epoch.
//   // INPUTS
//   //   *pim_ary -> the actual array of cpm for each axis
//   //   oflw_scl -> dividing factor to reduce pim_ary so doesn't overflow
//   // PARAMETERS
//   //   oflw_cap -> cap to prevent overflow when pim_ary[:].^2 is summed
//   // OUTPUT
//   //  @ return -> the sqrt of the scaled vmcpm
//   uint32_t oflw_cap = 37500;
//   for(int16_t axis = 0; axis < 3; axis++){
//     pim_ary[axis] = pim_ary[axis]/oflw_scl;
//     pim_ary[axis] = (pim_ary[axis] < oflw_cap) ? pim_ary[axis] : oflw_cap;
//   }
//
//   /* VMCPM = sqrt(pim_x^2 + pim_y^2 + pim_z^2)*/
//   // calculate VMCPM, then take sqrt to compress for storage
//   return isqrt( pim_ary[0]*pim_ary[0] + pim_ary[1]*pim_ary[1] + pim_ary[2]*pim_ary[2]) ;
// }
//
//
// uint8_t compressed_vmcpm(uint32_t *pim_ary, uint32_t oflw_scl){
//   return (uint8_t) isqrt(calc_scaled_vmc(pim_ary, oflw_scl));
// }
//
//
//
// uint32_t calc_real_vmc(uint32_t *pim_ary, uint32_t oflw_scl){
//
//   uint32_t scl_vmcpm = calc_scaled_vmc(pim_ary,oflw_scl);
//   // we first assume the PIM is calculated on 1 = 1G, then we scale
//   // to x10 the real vmcpm, THEN we divide by 1250 cause 125*10, and
//   // we divide the PIM calculated on 125 = 1G so we FINALLY get the
//   // *real* VMCPM given that we calculated on the Pebble accel where
//   // 1000/8 = 125 = 1G.
//   return (oflw_scl*scl_vmcpm*x10_RAW_1G_PIM_CPM_TO_REAL_CPM)/1250;
// }
//
//
//
// // TEST : PASSED
// uint32_t calc_real_cpm(uint32_t *pim_ary){
//   // This function calculates the CPM from the PIM array for each
//   // epoch.
//   // INPUTS
//   // *pim_ary -> the actual array of cpm for each axis
//
//   // we first assume the PIM is calculated on 1 = 1G, then we scale
//   // to x10 the real cpm, THEN we divide by 1250 cause 125*10, and
//   // we divide the PIM calculated on 125 = 1G so we FINALLY get the
//   // *real* VMCPM given that we calculated on the Pebble accel where
//   // 1000/8 = 125 = 1G.
//   return ((pim_ary[0] + pim_ary[1] + pim_ary[2])*x10_RAW_1G_PIM_CPM_TO_REAL_CPM)/1250;
// }
//
// uint32_t calc_x1000_kcalpm(uint32_t *pim_ary, uint32_t oflw_scl){
//   uint32_t x1000_kcalpm;
//
//   // update the general config
//
//   struct config_general cur_config;
//   persist_read_data(CONFIG_GENERAL_PERSIST_KEY,&cur_config,sizeof(cur_config));
//   uint8_t pweight_kg = cur_config.pweight_kg;
//
//
//   // NOTE, we have to re-expand the vmcpm by its scaling factor
//   // used to prevent overflow
//   uint32_t real_vmcpm = calc_real_vmc(pim_ary, oflw_scl);
//   uint32_t real_cpm = calc_real_cpm(pim_ary);
//
//
//   // NOTE!!, this assumes the equation from Freedson VM2 11'
//   //     if VMCPM > 2453
//   //   Kcals/min= 0.001064×VM + 0.087512(BM) - 5.500229
//   //     else
//   //   Kcals/min=CPM×0.0000191×BM
//   //
//   //     where
//   // VM = Vector Magnitude Combination (per minute) of all 3 axes (sqrt((Axis 1)^2+(Axis 2)^2+(Axis 3)^2])
//   // VMCPM = Vector Magnitude Counts per Minute
//   // CPM = Counts per Minute
//   // BM = Body Mass in kg
//
//   // NOTE, we have to scale ALL of these things by 10000, and since we
//   // know that each element of the pim_ary maxes out at around 375000,
//   // we know that the upper bound of cpm is 3*400,000 = 1,200,000, so
//   // to stay under the 4,294,967,296 overflow cap, we can only go to
//   // a factor of 10,000. BUT, problems still, cause we need to selectively
//   // decide which ones we are going to add/subtract. We WILL assume
//   // that the output of this function will be (kcal/min)*1000, which
//   // is also captured in the worker's function, cause we don't
//   // want to have big rounding down errors when we are at basal metabolic
//   // rate estimates.
//
//   if(real_vmcpm > 2453){
//     //     if VMCPM > 2453
//     //   Kcals/min= 0.001064×VM + 0.087512(BM) - 5.500229
//     x1000_kcalpm = (1064*real_vmcpm)/1000 + (875*pweight_kg)/10 + 5500;
//   }else{
//     //     else
//     //   Kcals/min=CPM×0.0000191×BM
//     x1000_kcalpm = (real_cpm*19*pweight_kg)/1000;
//   }
//
//   return x1000_kcalpm;
// }
//
//
//
// // TEST : PASSED
// void vm_accel(int16_t **d, int16_t *w, int16_t max_vm, int16_t dlen){
//   /* EVALUATE THE VECTOR ACCELERATION MAGNITUDE, WRITE TO first array */
//   // NOTE: we can look at the entire array, because we only write to
//   // the first N_SMP_EPOCH, the remainder being 0's, which will
//   // evaluate to a vector mag of 0
//   for(int16_t i = 0; i < dlen; i++){
//     // evaluate the vector mag of acceleration, write to work array w
//     w[i] = (int16_t) isqrt( d[0][i]*d[0][i] + d[1][i]*d[1][i] + d[2][i]*d[2][i] );
// //     w[i] = (int16_t) ( (d[0][i]*d[0][i]) + (d[1][i]*d[1][i]) + (d[2][i]*d[2][i]) );
//     //  cap the vector mags as max_vm to prevent overflow
//     w[i] = ((abs(w[i]) <= max_vm) ? w[i] : ((w[i] > 0 ) ? 1 : (-1))*max_vm);
//   }
// }
//
//
// // TEST : PASSED
// int16_t stepc_fftalg_0pad(int16_t *d, int16_t dlen_smp, int16_t dlenpwr_ary, int16_t oflw_scl){
//   // NOTE !!! WE ASSUME THE d INPUT IS ALREADY SCALED TO PREVENT OVERFLOW
//   // AND THAT IS MEAN-PADDED
//   /* PARAMETERS */
//   int16_t dlen_ary = pow_int(2,dlenpwr_ary);
//   int16_t lhz_i = 3;
//   int16_t hhz_i = 20;
//   struct config_general init_ps = get_config_general();
// //   int16_t thres = 21; // 0.21 *100   // NOTE, thres range 0 to 100: 2 digits of precision
//   int16_t thres = init_ps.step_class_thres; // 0.21 *100   // NOTE, thres range 0 to 100: 2 digits of precision
//
//   // int16_t is_step = 0; // assume it is not a step period
//
//   // reduce the scale
//   for(int16_t i = 0; i < dlen_smp; i++){
//     d[i] = d[i]/oflw_scl;
//   }
//
//   // evaluate the mean and remove it from the vm_accel
//   // we do this here because we want the end of the work array to maintain
//   // a known value of 0 from the calloc operation
//
//   // We PRETEND that the last few values of the dlen_ary elements are
//   //  equal to the mean of the dlen_smp. Hence, the mean of the first
//   //  dlen_smp elements is the same as the mean of the dlen_ary elements
//   // THEN, we pretend that we removed the mean of the dlen_ary (also
//   //  mean of dlen_smp) from each in dlen_ary. Then, the last few element
//   //  in dlen_ary are 0, like we desire.
//   int16_t mean = mean_l1_stat(d, dlen_smp);
//   for(int16_t i = 0; i < dlen_smp; i++){
//     d[i] = d[i] - mean;
//   }
//
//   /* EVALUATE THE MAGNITUDE OF THE FFT COEFFICIENTS AND WRITE TO d ARRAY */
//   fft_2radix_real(d, dlenpwr_ary);
//   fft_mag(d, dlenpwr_ary);
//
//   /* EVALUATE THE STEP SCORE FOR EPOCH */
//   // -> This section can be fairly complex if needed
//   // When remove the mean, the DC is zero, so we add the DC value back
//   // -> (mean * length of array)
//   //  to evaluate the 4-20hz integral against the total integral
//   int score = (100*(integral_abs(d, lhz_i, hhz_i)))/(integral_abs(d, 0, dlen_ary/2) + mean*dlen_ary);
//
//   // evaluate if the period is a step epoch, based on score
//   int16_t max_hz_val = 0;
//   int16_t max_hz_i = 0; // if NOT a step period, then return 0 for no steps
//   if( score >= thres){
//     // if it is a step epoch, find the hz index with largest mag
//     for(int16_t i = lhz_i; i <= hhz_i; i++){
//       if(d[i] > max_hz_val ){
//         max_hz_val = d[i];
//         max_hz_i = i;
//       }
//     }
//   }
//   /* RETURN NUMBERS OF STEPS IN EPOCH */
//   return max_hz_i; // DC index is 0, so max_hz_i is HZ directly
// }
//
//
//
//
//
// // TEST : PASSED
// int16_t stepc_fftalg_0pad(int16_t *d, int16_t dlen_smp, int16_t dlenpwr_ary, int16_t oflw_scl){
//   // NOTE !!! WE ASSUME THE d INPUT IS ALREADY SCALED TO PREVENT OVERFLOW
//   // AND THAT IS MEAN-PADDED
//   /* PARAMETERS */
//   int16_t dlen_ary = pow_int(2,dlenpwr_ary);
//   int16_t lhz_i = 3;
//   int16_t hhz_i = 20;
//   struct config_general init_ps = get_config_general();
//   //   int16_t thres = 21; // 0.21 *100   // NOTE, thres range 0 to 100: 2 digits of precision
//   int16_t thres = init_ps.stepc_fft_thres3; // 0.21 *100   // NOTE, thres range 0 to 100: 2 digits of precision
//
//   // int16_t is_step = 0; // assume it is not a step period
//
//   // reduce the scale
//   for(int16_t i = 0; i < dlen_smp; i++){
//     d[i] = d[i]/oflw_scl;
//   }
//
//   // evaluate the mean and remove it from the vm_accel
//   // we do this here because we want the end of the work array to maintain
//   // a known value of 0 from the calloc operation
//
//   // We PRETEND that the last few values of the dlen_ary elements are
//   //  equal to the mean of the dlen_smp. Hence, the mean of the first
//   //  dlen_smp elements is the same as the mean of the dlen_ary elements
//   // THEN, we pretend that we removed the mean of the dlen_ary (also
//   //  mean of dlen_smp) from each in dlen_ary. Then, the last few element
//   //  in dlen_ary are 0, like we desire.
//
//   int16_t mean = mean_l1_stat(d, dlen_smp);
//   for(int16_t i = 0; i < dlen_smp; i++){
//     d[i] = d[i] - mean;
//   }
//
//   /* EVALUATE THE MAGNITUDE OF THE FFT COEFFICIENTS AND WRITE TO d ARRAY */
//   fft_2radix_real(d, dlenpwr_ary);
//   fft_mag(d, dlenpwr_ary);
//
//   /* EVALUATE THE STEP SCORE FOR EPOCH */
//   // -> This section can be fairly complex if needed
//   // When remove the mean, the DC is zero, so we add the DC value back
//   // -> (mean * length of array)
//   //  to evaluate the 4-20hz integral against the total integral
//   int score = (100*(integral_abs(d, lhz_i, hhz_i)))/(integral_abs(d, 0, dlen_ary/2) + mean*dlen_ary);
//
//   // evaluate if the period is a step epoch, based on score
//   int16_t max_hz_val = 0;
//   int16_t max_hz_i = 0; // if NOT a step period, then return 0 for no steps
//   if( score >= thres){
//     // if it is a step epoch, find the hz index with largest mag
//     for(int16_t i = lhz_i; i <= hhz_i; i++){
//       if(d[i] > max_hz_val ){
//         max_hz_val = d[i];
//         max_hz_i = i;
//       }
//     }
//   }
//   /* RETURN NUMBERS OF STEPS IN EPOCH */
//   return max_hz_i; // DC index is 0, so max_hz_i is HZ directly
// }



// int16_t calc_stepc_5sec(int16_t *work_ary, int16_t dlen_smp, int16_t dlenpwr_ary,
//   uint32_t *pim_5sec_ary, int16_t fft_oflw_scl){
//     // get the mean acceleration to serve as minimum thresholds for running and
//     // walking
//     int32_t x100_mean_vm_accel = x100_mean_l1_stat(work_ary, dlen_smp);
//     uint32_t real_vmc_5s = calc_real_vmc(pim_5sec_ary);
//
//     struct config_general cg = get_config_general();
//
//     // filter the result
//     filt_cosine_win_mean0(work_ary, dlen_smp, 1);
//
//     get_fftmag_0pad_mean0(work_ary, dlen_smp, dlenpwr_ary, fft_oflw_scl);
//
//     uint16_t max_mag_hz = max_mag_hz_0pad(work_ary);
//
//     //uint8_t score0 = score_fftmag_hz_rng_l2(work_ary, dlenpwr_ary, max_mag_hz-1, max_mag_hz+1);
//     // uint8_t score0 = score_fftmag_hz_rng_abs(work_ary, dlenpwr_ary, max_mag_hz, max_mag_hz);
//     uint8_t score0 = score_fftmag_hz_rng_abs(work_ary, dlenpwr_ary, max_mag_hz-1, max_mag_hz+1);
//
//     uint16_t stepc_tmp = 0;
//
//     struct FeatureSample new_fsmp;
//     new_fsmp.f[0] = real_vmc_5s;
//     new_fsmp.f[1] = score0;
//     update_fsmp_buf(new_fsmp);
//     update_trained_dists();
//     if( classify_feature_sample(new_fsmp) > 0){
//       stepc_tmp = max_mag_hz;
//     }
//
//   return stepc_tmp;
// }


// // TEST : PASSED
// int32_t pim_filt(int16_t *d, int16_t dlen){
//   /* This function calculates the pim of the given array, single axis
//   * construction and application of filters are contained with the function */
//   int32_t pim_local = 0;

//   /* calculate the filter */
//   // the mean
//   int32_t mean = mean_l1_stat(d, dlen);

//   /* apply the filter */
//   // remove the mean
//   for(int16_t i = 0; i < dlen; i++){
//     pim_local += abs(d[i] - mean);
//   }

//   /* Return the local pim for given axis */
//   return pim_local;
// }

// if(
//     ((score0 >= cg.stepc_fft_thres0[0]) && (score0 <= cg.stepc_fft_thres0[1])
//       && (real_vmc_5s >= cg.stepc_vmc_thres0)&&(real_vmc_5s < cg.stepc_vmc_thres1))
//     || ((score0 >= cg.stepc_fft_thres1[0]) && (score0 <= cg.stepc_fft_thres1[1])
//       && (real_vmc_5s >= cg.stepc_vmc_thres1)&&(real_vmc_5s < cg.stepc_vmc_thres2 ))
//     || ((score0 >= cg.stepc_fft_thres2[0]) && (score0 <= cg.stepc_fft_thres2[1])
//       && (real_vmc_5s >= cg.stepc_vmc_thres2)&&(real_vmc_5s < cg.stepc_vmc_thres3))
//     || ((score0 >= cg.stepc_fft_thres3[0]) && (score0 <= cg.stepc_fft_thres3[1])
//       && (real_vmc_5s >= cg.stepc_vmc_thres3 ) )
//   ){
//     // we subtract by an expected 0.5 cause the window truncation and
//     //  integer fft approximation causes to push up a half frequency  ~0.1Hz.
//   // stepc_tmp = max_mag_hz - ((uint16_t) (rand()%2));
//   stepc_tmp = max_mag_hz;
// }
