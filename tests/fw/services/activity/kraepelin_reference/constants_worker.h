#pragma once
#include <stdbool.h>
#include <stdint.h>

/* +++++++++++++++ PERSISTENT STORAGE KEYS +++++++++++++++ */

// +++++++ RESERVED KEYS
// REMEMBER!!! keys 1-20 are reserved for the actigraph keys

// +++++++ intra-app messaging
static const int16_t WORKER_START_FORE_APP_REASON_PERSIST_KEY = 120;

// +++++++ App State
static const int16_t PINTERACT_STATE_PERSIST_KEY = 130;
static const int16_t CONFIG_WAKEUP_IDS_PERSIST_KEY = 131;
static const int16_t ACTIVE_WAKEUP_CONFIG_I_PERSIST_KEY = 132;
static const int16_t ACTICLASS_LEARN_ALG_STATE_PERSIST_KEY = 133;


// +++++++ Configuration Data
static const int16_t CONFIG_GENERAL_PERSIST_KEY = 180;
static const int16_t CONFIG_WAKEUP_PERSIST_KEY = 181;
static const int16_t PK_VERSION_PERSIST_KEY = 183;


// +++++++ Interaction Data
static const int16_t PIRPS_B1_PERSIST_KEY = 190; // Patient Response Persistent Storage, Block 1
static const int16_t PIRPS_B2_PERSIST_KEY = 191; // Patient Response Persistent Storage, Block 2


// +++++++ Continuous Data
static const int16_t BUF_SIZE_PERSIST_KEY = 200;
static const int16_t I_BLK_PERSIST_KEY = 201;
// continuous daily metrics
static const int16_t DAILY_STEPC_PERSIST_KEY = 210;
static const int16_t DAILY_x1000_KCAL_PERSIST_KEY = 211;
static const int16_t DAILY_ACTI_PERSIST_KEY = 212;


// +++++++ Long Term Data
static const int16_t DAILY_SUMMARY_WEEKS_PERSIST_KEY = 220;


// +++++++ General Constants
// # DEFINED CONSTANTS
static const uint32_t NUM_SEC_IN_DAY = 24*60*60;
static const uint32_t NUM_SEC_IN_WEEK = 7*24*60*60;
static const uint8_t CUR_PK_VERSION = 6;


// +++++++ Memory Constraints
static const uint16_t N_B_TRANSMIT_CODE = 800;
static const uint16_t N_B_REMINDER_CODE = 400;




/* +++++++++++++++ ENUMERATED TYPES +++++++++++++++ */

enum WorkerForeAppWakeupReason{
  WFAWR_DO_NOTHING,
  WFAWR_PUSH_ALL_DATA_TO_SERVER,
  WFAWR_PUSH_ALL_DATA_TO_PHONE,
  WFAWR_MEMORY_LOW_REMINDER,
  WFAWR_WEAR_REMINDER,
  NUM_WFAWR
};

enum TransmitReason{
  TR_PUSH_NULL,
  TR_PUSH_ALL_DATA_TO_SERVER,
  TR_PUSH_ALL_DATA_TO_PHONE,
  NUM_TR
};

enum AppMessageKeys{
  AMKEY_NULL,
  AMKEY_PUSHTOSERVER,
  AMKEY_ACTI,
  AMKEY_PINTERACT,
  AMKEY_CONFIG_GENERAL,
  AMKEY_CONFIG_WAKEUP,
  NUM_AMKEY
};

enum ReminderReason{
  RR_NULL,
  RR_MEMORY_LOW,
  RR_WEAR,
  NUM_RR
};

enum ActivityClass {
  NO_ACTICLASS,
  SLOW_WALK,
  WALK,
  FAST_WALK,
  RUN,
  FAST_RUN,
  NUM_ACTICLASS
};

enum ActivityClassLearnFeatures {
  ACLF_VMC,
  ACLF_FFT_SCORE,
  NUM_ACLF
};



/* +++++++++++++++ STRUCTURES +++++++++++++++ */

// NOTES
// uint16_t daily_steps;
//   total daily steps
// uint16_t daily_motion_kcal;
//   daily calories estimated expended only through motion
// uint16_t daily_sleep_tot_min;
//   sleep assigned for the 16 hours previous to the time when the
//   person said that they arose, AND the DAY to which these minutes
//   are assigned is the one that the 16 hours ENDED ON.
// uint16_t daily_arise_time_min;
//   time when person said getting up for the day, triggered by a sudden
//   increase in the number od steps
//   counts

struct config_general{
  uint16_t pheight_cm; // cm;
  uint16_t pweight_kg; // kg;
  uint8_t stepc_fft_thres0[2]; // percent*100 of energy in 0.3-4hz band
  uint16_t stepc_vma_thres0;
  uint16_t stepc_vmc_thres0;

  uint16_t wear_class_thres;
  uint16_t pts_goal;

}__attribute__((__packed__));



#define NUM_DAYS_HISTORY 8

struct daily_acti{
  uint16_t steps[NUM_DAYS_HISTORY]; // total daily steps
  uint16_t kcal[NUM_DAYS_HISTORY]; // daily calories estimated expended only through motion
}__attribute__((__packed__));

struct pinteract_state{
  int8_t pi_11[NUM_DAYS_HISTORY]; // can make this a uint8_t array for last 10 days
  int16_t pi_12;
  int16_t pi_13;
  int16_t pi_14;
  int16_t pi_15;
}__attribute__((__packed__));


struct acticlass_learn_alg_state{
  bool init_alg;
  uint16_t f_mean[NUM_ACTICLASS][NUM_ACLF];
  uint16_t f_std[NUM_ACTICLASS][NUM_ACLF];
}__attribute__((__packed__));


/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* +++++++++++++++++++++ BACKGROUND APP CONSTANTS ONLY +++++++++++++++++++++++ */
/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
