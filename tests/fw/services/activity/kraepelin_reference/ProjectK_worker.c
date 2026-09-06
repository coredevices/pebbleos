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


#include "ProjectK_worker.h"
#include <stdlib.h>

/* META DATA CODES */

/* BEHAVIOR CONSTANTS */
static const int16_t SMP_HZ = 25; // number of samples taken per second
static const int16_t N_SMP_EPOCH = (5*25); // 5*25 = 125 samples recorded, 5 seconds for step count
static const int16_t FFT_PWR_TWO = 7; // 2^7 = 128 elements > 125 to allow fft
static const int16_t SIZE_EPOCH = 128; // 2^7 = 128 points, zero padding by 13 samples
static const int16_t SIZE_BLK_HEAD = 16; // # of bytes in head, all non-acti data including timemood
static const int16_t SIZE_BLK_TIME = 4; // # of bytes for storing timestamp
static const int16_t SIZE_SUMM = 2; // number of bytes in individual summary
static const int16_t N_BLK_PERSIST = 12; // 12 < 14, # blocks, ie: # key-value pairs in persist storage
static const int16_t N_SUMM_BLK = 120; // 120, number of summaries in a appmessage block
                    // ^-->NEEDS TO BE DIVISIBLE by N_STEPC_BLK

static const int16_t N_STEPC_BLK = 10; // 4 number of step count recording per block
static const int16_t N_MIN_VMC_0_NONWEAR = 60; // 4 number of step count recording per block


/* METRIC MANIPULATION */
static const int16_t LVL_SCL = 8; // the number of milligs between discrete levels
static const int16_t LVL_SHIFT = 3; // 2^LVL_SHIFT = LVL_SCL
static const uint8_t N_ANG = 16; // for orient_encode, # of angles possible chosen
/* FFT_SCL reduces the data size so doesn't overflow on the transforms. Divide by 4 to
* get max at 125, and divide by 2 to get  fourier DC to max at 512*125/2 = 32000 < 32768 the
* overflow boundary for 16 bit *signed* ints */
static const int16_t FFT_SCL = 2; // 125*500/2 = 25000 prevent overflow on the transforms, assuming +-250
static const int16_t MAX_VM = 1000; // make smaller to prevent overflow on the FFT
static const uint32_t VMCPM_SCL = 10; // scaled needed to prevent overflow in adding
static const uint32_t PIM_LVL_THRES = 5; /* mean # levels thres PIM, 2 lvl -> 5*0.008G = 0.056G
* we do this to match the actigraph, and this seems to be the *minimum* level (==3)that rejects
* noise when the pebble is perfectly still */

/* METRIC STORAGE SCALING*/
static const uint32_t STEPC_SCL = 25; // step resolution, scale count to stuff into uint8_t

/* AXIS AND ORIENTATION VARIABLES */
static const int16_t N_AXIS = 3;

/* BEHAVIOUR CONSTS */
// REMEMBER, CURRENTLY THIS IS ONLY CHECKED ONCE PER EACH BLOCK, SO IT WANT
// CHECKED MORE OFTEN THEN MUST PUT IN MINUTE HANDLER!!
static const uint16_t SUMM_BTWN_TRANSMIT_SERVER = 120; // 300 = 5 hours
static const uint16_t SUMM_BTWN_WEAR_REMINDER = 60; // 60 = 1 hours


/* ARRAY POINTERS : storage and work arrays */
static int16_t *pt_ary[3];
static int16_t *work_ary;
static uint8_t *blk_buf;

/* STATE VARIABLES */
static uint16_t i_smp = 0; // the index of the sample, relative to the epoch

/* SUMMARY METRICS */
static int16_t mean_ary[3];
static uint32_t pim_ary[3];
static uint32_t stepc;
static uint32_t x1000_kcal_blk;
static uint32_t stepc_prev_5sec = 0;
static uint32_t stepc_prev_10sec = 0;
// static uint32_t stepc_daily;


/* BEHAVIOUR VARIABLES */
static uint16_t summ_since_trans_server = 0;

static bool s_pim_filter_primed = false;

/* ++++ MAIN BODY ++++ */

/* HELPER FUNCTIONS */


/* METRIC MANIPULATION */
static void reset_epoch_metrics(){
  i_smp = 0; // dont need to reset the accel_data_handler cause we reset
      // the i_smp, only problem is that slightly off (not quite 1 sec aligned)

  for(int16_t axis = 0; axis < N_AXIS; axis++){
    // zero out the arrays
    for(int16_t i = 0; i < SIZE_EPOCH; i++ ){
      pt_ary[axis][i] = 0;
    }
  }
  for(int16_t i = 0; i < SIZE_EPOCH; i++ ){
    work_ary[i] = 0;
  }
}

static void reset_summ_metrics(){
  /* resets all metrics to 0 */
  for(int16_t axis = 0; axis < N_AXIS; axis++){
    mean_ary[axis] = 0;
    pim_ary[axis] = 0;
  }
}

#if PEBBLE_APP
static void reset_daily_metrics(){
  // check the previous time tag for this array. Roll the number
  // of days off that has passed since it was last written to.
  // BE sure to zero out the

  // use

  // push the days back by one on the

  // update the array tag to the current time (so, a little unintuitive)

  // add the past day's summaries to the history

  // within the first 5 minutes of the day, reset the daily summaries
  // stepc_daily = 0;
  persist_write_int(DAILY_STEPC_PERSIST_KEY,0);
  persist_write_int(DAILY_x1000_KCAL_PERSIST_KEY,0);

  // shift the daily ratings over by one
  struct pinteract_state pis = get_pinteract_state();
  struct daily_acti da = get_daily_acti();
  for(int8_t i = (NUM_DAYS_HISTORY-1); i > 0; i--){
    pis.pi_11[i] = pis.pi_11[i-1];
    da.steps[i] = da.steps[i-1];
    da.kcal[i] = da.kcal[i-1];
  }
  persist_write_data(PINTERACT_STATE_PERSIST_KEY, &pis, sizeof(pis));
  persist_write_data(DAILY_ACTI_PERSIST_KEY, &da, sizeof(da));


}


/* APPMESSAGE MANAGEMENT VIA PERSISTENT DATA */

static void write_blk_buf_to_persist(){
  /* the persist_read_int(I_BLK_PERSIST_KEY) is NOT a count, it is the current index
  * but 1-indexed, so 1 is the first block and zero is the empty state */

  if(persist_read_int(I_BLK_PERSIST_KEY) < N_BLK_PERSIST){
    // set the index of the current block
    persist_write_int(I_BLK_PERSIST_KEY,persist_read_int(I_BLK_PERSIST_KEY)+1);
    persist_write_data(persist_read_int(I_BLK_PERSIST_KEY),
                       blk_buf,((SIZE_SUMM*N_SUMM_BLK)+SIZE_BLK_HEAD));
  }

  /* attempt to send a message if bluetooth connection is there */
  if(bluetooth_connection_service_peek()){
    // attempt to send all data to phone
    if(summ_since_trans_server < SUMM_BTWN_TRANSMIT_SERVER){
      persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY,
        WFAWR_PUSH_ALL_DATA_TO_PHONE);
    }else{
      /* >>>>> EXECUTE, SEND TO SERVER <<<<< */
      persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY,
        WFAWR_PUSH_ALL_DATA_TO_SERVER);
        summ_since_trans_server = 0; // once try to go to server, reset
    }
    worker_launch_app();
  }else if( persist_read_int(I_BLK_PERSIST_KEY) >= (N_BLK_PERSIST-1) ){
    // if the I_BLK_PERSIST_KEY is greater than 90% of storage, then prompt
    // that need to connect the phone

    persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY,
      WFAWR_MEMORY_LOW_REMINDER);
    worker_launch_app();
  }
  // else if(){
  //   // reset the counter for consecutive summaries
  //   persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY,
  //     WFAWR_WEAR_REMINDER);
  //   worker_launch_app();
  // }

}


/* SUMMARY DATALOGGING */

static void summ_datalog(){

  static uint16_t i_summ_blk = 0; // initialize the summ_blk index to 0

  /* >>>>> START MODIFY BLOCK HEADER, TIMESTAMP AND MOOD <<<<< */
  // add the timestamp at the top of first block and reset the mood index
  if(i_summ_blk == 0){
    write_time_to_array_head(time(NULL),blk_buf);
    blk_buf[4] = CUR_PK_VERSION;
  }

  /* >>>>> START WRITING REGULAR ACTIGRAPHY DATA <<<<<*/
  /* WRITE DATA TO INDIVIDUAL SUMMARY BODIES */
  int16_t offset = (SIZE_SUMM*i_summ_blk + SIZE_BLK_HEAD);
  // write orientation
  blk_buf[offset + 0] = (uint8_t) orient_encode(mean_ary, N_ANG);
  // write vmcpm
  blk_buf[offset + 1] = (uint8_t) compressed_vmc(pim_ary);

//   persist_write_int(DAILY_x1000_KCAL_PERSIST_KEY,calc_real_vmc(pim_ary)*1000);

  /* >>>>> CALCULATE MET/MOTION KCAL AND UPDATE PERSIST <<<<< */
  // calculate the real counts, r_c_ary
  uint32_t r_c_ary[3];
  for(int16_t axis = 0; axis < 3; axis++){ r_c_ary[axis] = calc_real_c(pim_ary[axis]); }
  //calculate the kcal for this minute
  uint32_t x1000_kcal_min = calc_x1000_kcal(r_c_ary);
  persist_write_int(DAILY_x1000_KCAL_PERSIST_KEY,
                    (x1000_kcal_min
                    + persist_read_int(DAILY_x1000_KCAL_PERSIST_KEY) ));
  x1000_kcal_blk += x1000_kcal_min;
  // >> Write the step counts : finish a section of summaries, write stepc
  // we write the steps directly here because we dont want to have another
  // array to track the size, etc, and add extra indirection. Moreover
  // it keeps the theme of writing regular occurring data in this section.
  //  > (i_summ_blk + 1)%(N_SUMM_BLK/N_STEPC_BLK) == 0) --> that
  //  N_SUMM_BLK/N_STEPC_BLK is # of SUMMaries between writing the
  //  step_count to the BLK.
  //  > Hence N_SUMM_BLK must be divisible by N_STEPC_BLK. Also
  //  (i_summ_blk + 1) accounts for the 0 indexing.
  //   Hence, at the START, where
  //      i_summ_blk = 0, we have
  //      (i_summ_blk+1) % (N_SUMM_BLK/N_STEPC_BLK) == 1
  //   and at the END, where:
  //      i_summ_blk = (N_SUMM_BLK - 1), then
  //      (i_summ_blk + 1)%(N_SUMM_BLK/N_STEPC_BLK) == 0
  if( (i_summ_blk + 1)%(N_SUMM_BLK/N_STEPC_BLK) == 0){
    // as ( (i_summ_blk + 1)%(N_SUMM_BLK/N_STEPC_BLK) == 0) we have
    //  that  (i_summ_blk/(N_SUMM_BLK/N_STEPC_BLK)) rounds down, and
    blk_buf[(i_summ_blk/(N_SUMM_BLK/N_STEPC_BLK)) + SIZE_BLK_TIME + 2] =
      compressed_stepc(stepc, N_SUMM_BLK/N_STEPC_BLK);
    stepc = 0;
  }
  /* >>>>> END WRITING REGULAR ACTIGRAPHY DATA <<<<<*/

  /* >>>>> INCREMENT THE SUMM COUNTER IN THE blk_buf <<<<< */
  i_summ_blk += 1;
  /* >>>>> INCREMENT NUMBER OF SUMM SINCE LAST TRANSMIT TO SERVER <<<<< */
  summ_since_trans_server += 1;
  /* >>>>> WRITE FULL blk_buf TO am_buf AND SEND TO SERVER <<<<< */
  if( i_summ_blk >= N_SUMM_BLK){
    /* write the last hours total calories */
    blk_buf[5] = compressed_x1000_kcal(x1000_kcal_blk,N_SUMM_BLK);
    x1000_kcal_blk = 0;
    /* reset the summ counter */
    i_summ_blk = 0;
    /* write to appmessage dict cause have full blk_buf */
    write_blk_buf_to_persist();
  }

  //* >>>>> WRITE THE CURRENT KCAL and STEPS to CURRENT Daily Acti history <<<<< */
  struct daily_acti da = get_daily_acti();
  da.steps[0] = isqrt(persist_read_int(DAILY_STEPC_PERSIST_KEY));
  da.kcal[0] = isqrt(persist_read_int(DAILY_x1000_KCAL_PERSIST_KEY)/1000);
  persist_write_data(DAILY_ACTI_PERSIST_KEY, &da, sizeof(da));
}
#endif // PEBBLE_APP

/* ACCELEROMETER HANDLING */

static void epoch_analysis(){
  if (i_smp == 0) {
    return;
  }

  /* WEAR TIME CALCULATION, NO THRESHOLD */
  uint32_t pim_5sec_ary[3] = {0};

  // If this is the first epoch after an init, we need to prime the butterworth filter used
  // by prv_pim_filter to avoid getting jumps in VMC due to the discontinuity
  if (!s_pim_filter_primed) {
    for (int16_t axis = 0; axis < 3; axis++) {
      pim_filt_prime(&pt_ary[axis][0], SMP_HZ, axis);
    }
    s_pim_filter_primed = true;
  }

  /* CALCULATE THE AXIS DEPENDENT METRICS */
  for(int16_t axis = 0; axis < N_AXIS; axis++){
    // note: move the array ptr by i_smp to give the next start of data array
    // add the local mean to the global mean array, additively
    mean_ary[axis] += mean_l1_stat(pt_ary[axis], N_SMP_EPOCH);

    /* CALCULATE THE 1 SECOND METRICS */
    uint32_t pim_test = 0;
    // calculate the local pim each 1sec basis: N_SMP_EPOCH/SMP_HZ = num of sec in epoch
    for(int16_t sec_i = 0; sec_i < (i_smp/SMP_HZ); sec_i++){
      pim_test = pim_filt(pt_ary[axis] + (sec_i*SMP_HZ), SMP_HZ,axis ); // raw PIM

      // Thresholded pim for the VMCPM calculation, Actigraph equivalent
      pim_ary[axis] += pim_test; // test if PIM above threshold

      // pim step count analysis
      pim_5sec_ary[axis] += pim_test;
    }
  }

  /* CALCULATE ACCELERATION MAGNITUDE METRICS */
  // NOTE, we store acceleration magnitude in work vector, original pt_ary
  // arrays are unmodified for future use.
  vm_accel(pt_ary, work_ary, MAX_VM, i_smp); // write vm into work_ary

  /* CALCULATE STEP COUNT FOR 5-SEC TERM, UPDATE PERSIST */
  uint16_t stepc_tmp = calc_stepc_5sec(work_ary, i_smp, FFT_PWR_TWO, pim_5sec_ary, FFT_SCL);
  // // +++++++++++++++++++++++++++++++++++
  // // EXAMPLE for START/STOP problem
  // int16_t tmp = stepc_tmp;
  // if((stepc_prev_5sec==0) && (stepc_tmp >0)){
  //   // non-walking to walking
  //   stepc_tmp += (stepc_tmp)/2; // add 1/3 current period to previous period
  // }else if((stepc_prev_5sec > 0) && (stepc_tmp == 0)){
  //   // walking to non-walking
  //   stepc_tmp += (stepc_prev_5sec)/2; // add 1/3 prev period to current period
  // }
  // stepc_prev_5sec = tmp;
  // // END EXAMPLE
  // // +++++++++++++++++++++++++++++++++++
  // +++++++++++++++++++++++++++++++++++
  // EXAMPLE for START/STOP problem
  int16_t tmp = stepc_tmp;
  if((stepc_prev_10sec==0) && (stepc_prev_5sec>0) && (stepc_tmp >0)){
    // non-walking to walking
    stepc_tmp += (stepc_tmp)/2; // add 1/2 current period to previous period
  }else if( (stepc_prev_10sec > 0) && (stepc_prev_5sec > 0) && (stepc_tmp == 0)){
    // walking to non-walking
    stepc_tmp += (stepc_prev_5sec)/2; // add 1/2 prev period to current period
  }
  stepc_prev_10sec = stepc_prev_5sec;
  stepc_prev_5sec = tmp;
  // END EXAMPLE
  // +++++++++++++++++++++++++++++++++++

  // UPDATE STEPS
  stepc += stepc_tmp;
  // stepc_daily += stepc_tmp;
#if PEBBLE_APP
  persist_write_int(DAILY_STEPC_PERSIST_KEY,
    persist_read_int(DAILY_STEPC_PERSIST_KEY) + stepc_tmp);
#endif

  // !! RESET THE EPOCH . At the end of each epoch analysis, we ALWAYS need to
  // go back to i_smp=0, regardless of where in the cycle we are
  reset_epoch_metrics();
}

#if PEBBLE_APP
void tick_summ_datalog_second_handler(struct tm *tick_time, TimeUnits units_changed){
  static int8_t cur_min = -1; // this tracks the current min, 0-59 values
  // this is to protect against summ_datalog() being called more than once a min


  /* >>>>> EXECUTE TOP OF EACH MINUTE, XX:XX:00 <<<<< */
  // ALSO, only execute it it is a new second/new minute
  if( (tick_time->tm_sec == 0) && (tick_time->tm_min != cur_min) ){
    // reset the minute gate
    cur_min = tick_time->tm_min;
    /* perform the epoch analysis */
    epoch_analysis();
    /* write the summary to the persist storage */
    summ_datalog();
    /* reset the summary metric arrays after each write to storage */
    reset_summ_metrics();

    /* >>>>> REMINDERS <<<<< */
    /* WEAR TIME */
    /* MEMORY WARNING */
  }

  /* >>>>> RESET THE DAILY METRICS AND CHANGE LONG TERM DATA @ 12:01AM <<<<< */
  if( (tick_time->tm_hour == 0 ) && (tick_time->tm_min == 1 ) && (tick_time->tm_sec == 0 )){
    reset_daily_metrics();
  }
}
#endif

static void accel_data_handler(AccelData *data, uint32_t num_samples ){
  /* WRITE THE SAMPLES INTO THE X,Y,Z STORAGE ARRAYS*/
  for (int16_t i = 0; i < (int16_t) num_samples; i++){
    /* Divide each sample by LVL_SCL to prevent int overflow in wavelet compression */
    pt_ary[0][i + i_smp] = (data[i].x + LVL_SCL / 2) >> LVL_SHIFT;
    pt_ary[1][i + i_smp] = (data[i].y + LVL_SCL / 2) >> LVL_SHIFT;
    pt_ary[2][i + i_smp] = (data[i].z + LVL_SCL / 2) >> LVL_SHIFT;
  }

  /* INCREMENT THE SAMPLES INDEX */
  i_smp += SMP_HZ; // perhaps i_smp += num_samples; in case non-uniform?

  /* when attain N_SMP_EPOCH samples, perform the various summary statistics , write epoch-data to log */
  if(i_smp >= N_SMP_EPOCH){
    epoch_analysis();
    /* RESET THE SAMPLE INDEX*/
  }
}

#if PEBBLE_APP
/* BLUETOOTH CONNECTION SERVICE HANDLER*/
void worker_bt_service_handler(bool connected){
  if(connected){
    // set the foreground app start by worker info persistent storage id
    // try to send all data to phone
    persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY,
      WFAWR_PUSH_ALL_DATA_TO_SERVER);
    worker_launch_app();
  }
}
#endif


/* INITIALIZATION AND MAIN WORKER LOOP */

static void init_mem_log(){
  /* initializing x,y,z storage arrays and work array */
  for(int16_t axis = 0; axis < N_AXIS; axis++){
    pt_ary[axis] = (int16_t*) calloc(SIZE_EPOCH, sizeof(int16_t));
  }
  /* initialize the work array */
  work_ary = (int16_t*) calloc(SIZE_EPOCH, sizeof(int16_t));
  /* initialize the block buffer here */
  blk_buf = (uint8_t*) calloc(((SIZE_SUMM*N_SUMM_BLK)+SIZE_BLK_HEAD), sizeof(uint8_t));
#if PEBBLE_APP
  /* save the size of the block buffer (# of bytes long) to persistent storage */
  persist_write_int(BUF_SIZE_PERSIST_KEY,((SIZE_SUMM*N_SUMM_BLK)+SIZE_BLK_HEAD));
#endif
  pim_filt_init();
  s_pim_filter_primed = false;
}


#if PEBBLE_APP
static void init() {
  init_mem_log();
  // accel_data_service_subscribe(SMP_HZ, accel_data_handler);
  // accel_service_set_sampling_rate(ACCEL_SAMPLING_25HZ);

  // subscribe the summ_datalog tick handler
  // NOTE : we need to be sure to have second-level accuracy, so given the bugs
  // with the MINUTE_UNIT timer, we assume that we need to call every second
  // NOTE! we use SECOND_UNIT, SECOND_UNIT is CORRECT!!
  tick_timer_service_subscribe(SECOND_UNIT, tick_summ_datalog_second_handler);

  // subscribe to the accelerometer handler
  accel_data_service_subscribe(SMP_HZ, accel_data_handler);
  accel_service_set_sampling_rate(ACCEL_SAMPLING_25HZ);

  // set the bluetooth state change to also send the data
  bluetooth_connection_service_subscribe(worker_bt_service_handler);

  // init the learning algorithms params, if HAS NOT been init'ed
  // struct acticlass_learn_alg_state aclas;
  // persist_read_data(ACTICLASS_LEARN_ALG_STATE_PERSIST_KEY, &aclas, sizeof(aclas));
  // if( !aclas.init_alg){
  //   init_learn_alg1_stepc_posts_ps();
  // }
}

static void deinit() {
  accel_data_service_unsubscribe();
  tick_timer_service_unsubscribe();
  /* free the malloc'ed x,y,z arrays and work array */
  for(int16_t axis = 0; axis < N_AXIS; axis++){
    free(pt_ary[axis]);
  }
  free(work_ary);
}

int main(void) {
  init();
  worker_event_loop();
  deinit();
}
#endif // PEBBLE_APP

void ref_init(void) {
  init_mem_log();
  stepc = 0;
  stepc_prev_10sec = 0;
  stepc_prev_5sec = 0;
}

int ref_accel_data_handler(AccelData *data, uint32_t num_samples) {
  accel_data_handler(data, num_samples );
  return stepc;
}

int ref_finish_epoch(void) {
  epoch_analysis();
  return stepc;
}

void ref_minute_stats(uint8_t *orientation, uint8_t *vmc) {
  *orientation = orient_encode(mean_ary, N_ANG);
  *vmc = compressed_vmc(pim_ary);

  uint32_t r_c_ary[3];
  for (int16_t axis = 0; axis < 3; axis++) {
    r_c_ary[axis] = calc_real_c(pim_ary[axis]);
  }
  uint32_t kcals_min = calc_x1000_kcal(r_c_ary);

  PBL_LOG_DBG("minute_stats vmc: %d, orientation:0x%x, calories: %d",
          (int)*vmc, (int)*orientation, (int)kcals_min);

  reset_summ_metrics();
}


//
// void tick_summ_datalog_handler(struct tm *tick_time, TimeUnits units_changed){
//   static int8_t cur_min = -1; // this tracks the current min, 0-59 values
//   // this is to protect against summ_datalog() being called more than once a min
//
//   /* >>>>> EXECUTE TOP OF EACH SECOND, XX:XX:XX <<<<< */
//
//   /* >>>>> EXECUTE TOP OF EACH MINUTE, XX:XX:00 <<<<< */
//   if( (tick_time->tm_sec == 0) && (tick_time->tm_min != cur_min)){
//     // reset the minute gate
//     cur_min = tick_time->tm_min;
//     /* perform the epoch analysis */
//     epoch_analysis();
//     /* write the summary to the persist storage */
//     summ_datalog();
//     /* reset the summary metric arrays after each write to storage */
//     reset_summ_metrics();
//   }
//
//   /* >>>>> RESET THE DAILY METRICS AND CHANGE LONG TERM DATA @ 12:01AM <<<<< */
//   if( (tick_time->tm_hour == 0 ) && (tick_time->tm_min < 1 )){
//     reset_daily_metrics();
//   }
//   /* >>>>> EXECUTE, SEND TO SERVER <<<<< */
//   if( summ_since_trans_server > SUMM_BTWN_TRANSMIT_SERVER){
//     persist_write_int(WORKER_START_FORE_APP_REASON_PERSIST_KEY, 2);
//     summ_since_trans_server = 0; // once try to go to server, reset
//   }
// }
