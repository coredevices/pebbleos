/* SPDX-FileCopyrightText: 2026 Core Devices LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/* Because nPM1300 also has the battery monitor, we implement both the
 * pmic_* and the battery_* API here.  */

#include <math.h>

#include <pbl/drivers/pmic.h>
#include <pbl/drivers/battery.h>

#include "board/board.h"
#include "console/prompt.h"
#include <pbl/drivers/battery.h>
#include <pbl/drivers/exti.h>
#include <pbl/drivers/gpio.h>
#include <pbl/drivers/i2c.h>
#include "kernel/events.h"
#include "kernel/util/delay.h"
#include "kernel/util/sleep.h"
#include "pbl/os/mutex.h"
#include "pbl/services/system_task.h"
#include <pbl/logging/logging.h>
#include "system/passert.h"

PBL_LOG_MODULE_DEFINE(driver_pmic_npm1300, CONFIG_DRIVER_PMIC_LOG_LEVEL);

#define CHARGER_DEBOUNCE_MS 400
#define ADC_POLL_DELAY_MS   5     // Delay between ADC poll iterations to reduce I2C traffic
#define ADC_POLL_TIMEOUT_MS 100   // Max time to wait for ADC measurement
static TimerID s_debounce_charger_timer = TIMER_INVALID_ID;
static uint32_t s_dischg_limit_ma;

typedef enum {
  PmicRegisters_MAIN_EVENTSADCCLR = 0x0003,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY = 0x01,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY = 0x02,
  PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY = 0x40,
  PmicRegisters_MAIN_EVENTSBCHARGER1CLR = 0x000B,
  PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET = 0x000C,
  PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED = 16,
  PmicRegisters_MAIN_EVENTSVBUSIN0CLR = 0x0017,
  PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET = 0x0018,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED = 1,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED = 2,
  PmicRegisters_SYSTEM_TESTACCESS = 0x0123,
  PmicRegisters_SYSTEM_TESTACCESS__VAL0 = 0x44,
  PmicRegisters_SYSTEM_TESTACCESS__VAL1 = 0x90,
  PmicRegisters_SYSTEM_TESTACCESS__VAL2 = 0xFA,
  PmicRegisters_SYSTEM_TESTACCESS__VAL3 = 0xCE,
  PmicRegisters_VBUSIN_TASKUPDATELIMSW = 0x0200,
  PmicRegisters_VBUSIN_TASKUPDATELIMSW__EN = 0x01,
  PmicRegisters_VBUSIN_VBUSINILIM0 = 0x0201,
  PmicRegisters_VBUSIN_VBUSINILIMSTARTUP = 0x0202,
  PmicRegisters_VBUSIN_VBUSINSTATUS = 0x0207,
  PmicRegisters_VBUSIN_VBUSINSTATUS__VBUSINPRESENT = 1,
  PmicRegisters_BCHARGER_TASKRELEASEERROR = 0x0300U,
  PmicRegisters_BCHARGER_TASKCLEARCHGERR = 0x0301U,
  PmicRegisters_BCHARGER_BCHGENABLESET = 0x0304,
  PmicRegisters_BCHARGER_BCHGENABLECLR = 0x0305,
  PmicRegisters_BCHARGER_BCHGISETMSB = 0x0308,
  PmicRegisters_BCHARGER_BCHGISETLSB = 0x0309,
  PmicRegisters_BCHARGER_BCHGISETDISCHARGEMSB = 0x030A,
  PmicRegisters_BCHARGER_BCHGISETDISCHARGELSB = 0x30B,
  PmicRegisters_BCHARGER_BCHGVTERM = 0x030CU,
  PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V20 = 0x8U,
  PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V35 = 0xBU,
  PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V45 = 0xDU,
  PmicRegisters_BCHARGER_BCHGVTERMR = 0x030DU,
  PmicRegisters_BCHARGER_BCHGVTERMR__BCHGVTERMREDUCED_4V00 = 0x4U,
  PmicRegisters_BCHARGER_BCHGITERMSEL = 0x030F,
  PmicRegisters_BCHARGER_BCHGITERMSEL__SEL10 = 0U,
  PmicRegisters_BCHARGER_BCHGITERMSEL__SEL20 = 1U,
  PmicRegisters_BCHARGER_NTCHOT = 0x0316U,
  PmicRegisters_BCHARGER_NTCHOTLSB = 0x0317U,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS = 0x0334,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED = 2,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE = 4,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT = 8,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE = 16,
  PmicRegisters_BCHARGER_BCHGERRREASON = 0x0336,
  PmicRegisters_BCHARGER_BCHGDEBUG = 0x0346,
  PmicRegisters_BCHARGER_BCHGDEBUG__DISABLEBATTERYDETECT = 0x04,
  PmicRegisters_BCHARGER_BCHGVBATLOWCHARGE = 0x0350,
  PmicRegisters_ADC_TASKVBATMEASURE  = 0x0500,
  PmicRegisters_ADC_TASKNTCMEASURE   = 0x0501,
  PmicRegisters_ADC_TASKVSYSMEASURE  = 0x0503,
  PmicRegisters_ADC_TASKIBATMEASURE  = 0x0506,
  PmicRegisters_ADC_TASKVBUS7MEASURE = 0x0507,
  PmicRegisters_ADC_ADCIBATMEASSTATUS = 0x0510,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_MASK = 0x0C,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_DISCHRG = 0x04,
  PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_CHRG = 0x0C,
  PmicRegisters_ADC_ADCNTCRSEL = 0x050AU,
  PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_HIZ = 0x0U,
  PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K = 0x1U,
  PmicRegisters_ADC_ADCVBATRESULTMSB = 0x0511,
  PmicRegisters_ADC_ADCNTCRESULTMSB = 0x512,
  PmicRegisters_ADC_ADCVSYSRESULTMSB = 0x0514,
  PmicRegisters_ADC_ADCGP0RESULTLSBS = 0x0515,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_POS = 0U,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_POS = 2U,
  PmicRegisters_ADC_ADCVBAT2RESULTMSB = 0x0518,
  PmicRegisters_ADC_ADCGP1RESULTLSBS = 0x051a,
  PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_MSK = 0x03,
  PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_POS = 0x04,
  PmicRegisters_ADC_ADCIBATMEASEN = 0x0524,
  PmicRegisters_GPIOS_GPIOMODE1 = 0x0601,
  PmicRegisters_GPIOS_GPIOMODE__GPOIRQ = 5,
  PmicRegisters_GPIOS_GPIOMODE2 = 0x0602,
  PmicRegisters_GPIOS_GPIOMODE__OUTPUT_HIGH = 8,
  PmicRegisters_GPIOS_GPIOMODE__OUTPUT_LOW = 9,
  PmicRegisters_GPIOS_GPIOPUEN2 = 0x060C,
  PmicRegisters_GPIOS_GPIOPUEN__EN = 1,
  PmicRegisters_GPIOS_GPIOPUEN__DIS = 0,
  PmicRegisters_GPIOS_GPIOMODE3 = 0x0603,
  PmicRegisters_GPIOS_GPIOPUEN3 = 0x060D,
  PmicRegisters_GPIOS_GPIOOPENDRAIN1 = 0x0615,
  PmicRegisters_ERRLOG_SCRATCH0 = 0x0E01,
  PmicRegisters_ERRLOG_SCRATCH1 = 0x0E02,
  PmicRegisters_BUCK_BUCK1ENACLR = 0x0401,
  PmicRegisters_BUCK_BUCK1NORMVOUT = 0x0408,
  PmicRegisters_BUCK_BUCK2NORMVOUT = 0x040A,
  PmicRegisters_BUCK_BUCKSWCTRLSEL = 0x040F,
  PmicRegisters_BUCK_BUCKSWCTRLSEL__BUCK1SWCTRLSEL_SWCTRL = 0x01,
  PmicRegisters_BUCK_BUCKSWCTRLSEL__BUCK2SWCTRLSEL_SWCTRL = 0x02,
  PmicRegisters_BUCK_BUCK1VOUTSTATUS = 0x0410,
  PmicRegisters_BUCK_BUCK2VOUTSTATUS = 0x0411,
  PmicRegisters_BUCK_BUCKSTATUS = 0x0434,
  PmicRegisters_LDSW_TASKLDSW1SET = 0x0800,
  PmicRegisters_LDSW_TASKLDSW1CLR = 0x0801,
  PmicRegisters_LDSW_TASKLDSW2SET = 0x0802,
  PmicRegisters_LDSW_TASKLDSW2CLR = 0x0803,
  PmicRegisters_LDSW_LDSWSTATUS = 0x0804,
  PmicRegisters_LDSW_LDSWSTATUS__LDSW2PWRUPLDO = 0x08,
  PmicRegisters_LDSW_LDSWCONFIG = 0x0807,
  PmicRegisters_LDSW_LDSW1LDOSEL = 0x0808,
  PmicRegisters_LDSW_LDSW2LDOSEL = 0x0809,
  PmicRegisters_LDSW_LDSW2LDOSEL__LDSW_MODE = 0,
  PmicRegisters_LDSW_LDSW2LDOSEL__LDO_MODE = 1,
  PmicRegisters_LDSW_LDSW1VOUTSEL = 0x080C,
  PmicRegisters_LDSW_LDSW2VOUTSEL = 0x080D,
  PmicRegisters_LDSW_LDSW2VOUTSEL__3V3 = 23,
  PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE = 0x0B01,
  PmicRegisters_SHIP_TASKENTERSHIPMODE = 0x0B02,
  PmicRegisters_SHIP_SHPHLDCONFIG = 0x0B04,
  PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS = 3,
} PmicRegisters;

#define NPM1300_BCHGISETDISCHARGEMSB_200MA 42U
#define NPM1300_BCHGISETDISCHARGELSB_200MA 0U
#define NPM1300_BCHGISETDISCHARGEMSB_1000MA 207U
#define NPM1300_BCHGISETDISCHARGELSB_1000MA 1U

#define NPM1300_BCHARGER_ADC_BITS_RESOLUTION 1023
#define NPM1300_BCHARGER_ADC_CALC_DISCHARGE_MUL 112
#define NPM1300_BCHARGER_ADC_CALC_DISCHARGE_DIV 100
#define NPM1300_BCHARGER_ADC_CALC_CHARGE_MUL 1250
#define NPM1300_BCHARGER_ADC_CALC_CHARGE_DIV -1000
// Full scale voltage for battery voltage measurement
#define NPM1300_ADC_VFS_VBAT_MV 5000UL
// ADC MSB shift
#define NPM1300_ADC_MSB_SHIFT 2U
#define NPM1300_VBUS_CURRENT_DIVISOR 100U

static bool dischg_limit_ma_set(uint32_t dischg_limit_ma);

static uint16_t prv_ntc_threshold_code(uint8_t celsius) {
  // Ref: PS v1.1 Section 6.2.5: K_NTCTEMP = round(1024 * R_T / (R_T + R_B))
  float t_k = (float)celsius + 273.15f;
  float exponent = (float)NPM1300_CONFIG.thermistor_beta *
                   ((1.f / 298.15f) - (1.f / t_k));
  return (uint16_t)((1024.0f / (1.0f + exp(exponent))) + 0.5f);
}

void battery_init(void) {
}

static bool prv_read_register(uint16_t register_address, uint8_t *result) {
  i2c_use(I2C_NPM1300);
  uint8_t regad[2] = { register_address >> 8, register_address & 0xFF };
  bool rv = i2c_write_read_block(I2C_NPM1300, 2, regad, 1, result);
  i2c_release(I2C_NPM1300);
  return rv;
}

static bool prv_write_register(uint16_t register_address, uint8_t datum) {
  i2c_use(I2C_NPM1300);
  uint8_t d[3] = { register_address >> 8, register_address & 0xFF, datum };
  bool rv = i2c_write_block(I2C_NPM1300, 3, d);
  i2c_release(I2C_NPM1300);
  return rv;
}

// Anomaly 27 workaround: when switching BUCKn to SW control, if BUCKnNORMVOUT
// equals the VSET pin value (BUCKnVOUTSTATUS), quiescent current increases by
// 1mA. To avoid this, first set BUCKnNORMVOUT to a different value, switch to
// SW control, then set the desired voltage.
static bool prv_buck_set_sw_ctrl(uint16_t normvout_reg, uint16_t voutstatus_reg,
                                 uint8_t swctrlsel_bit, uint8_t desired_vout) {
  uint8_t voutstatus;
  if (!prv_read_register(voutstatus_reg, &voutstatus)) {
    return false;
  }

  // Ensure NORMVOUT differs from VOUTSTATUS before enabling SW control
  uint8_t initial_vout = (desired_vout != voutstatus) ? desired_vout : (desired_vout ^ 1);
  bool ok = prv_write_register(normvout_reg, initial_vout);

  // Read current SWCTRLSEL and set our bit
  uint8_t swctrlsel;
  if (!prv_read_register(PmicRegisters_BUCK_BUCKSWCTRLSEL, &swctrlsel)) {
    return false;
  }
  ok &= prv_write_register(PmicRegisters_BUCK_BUCKSWCTRLSEL, swctrlsel | swctrlsel_bit);

  // Now set the actual desired voltage
  if (initial_vout != desired_vout) {
    ok &= prv_write_register(normvout_reg, desired_vout);
  }

  return ok;
}

static void prv_handle_charge_state_change(void *null) {
  const bool is_charging = pmic_is_charging();
  const bool is_connected = pmic_is_usb_connected();
  PBL_LOG_DBG("nPM1300 Interrupt: Charging? %s Plugged? %s",
      is_charging ? "YES" : "NO", is_connected ? "YES" : "NO");

  if (is_connected && NPM1300_CONFIG.vbus_current_lim0 != 0) {
    bool ok = prv_write_register(PmicRegisters_VBUSIN_VBUSINILIM0,
      NPM1300_CONFIG.vbus_current_lim0/NPM1300_VBUS_CURRENT_DIVISOR);
    ok &= prv_write_register(PmicRegisters_VBUSIN_TASKUPDATELIMSW,
      PmicRegisters_VBUSIN_TASKUPDATELIMSW__EN);
    if (!ok) {
      PBL_LOG_ERR("config vbus limite0 failed");
    }
  }

  PebbleEvent event = {
    .type = PEBBLE_BATTERY_CONNECTION_EVENT,
    .battery_connection = {
      .is_connected = battery_is_usb_connected(),
    },
  };
  event_put(&event);
}

static void prv_clear_pending_interrupts() {
  prv_write_register(PmicRegisters_MAIN_EVENTSBCHARGER1CLR, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  prv_write_register(PmicRegisters_MAIN_EVENTSVBUSIN0CLR, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
}

static void prv_pmic_state_change_cb(void *null) {
  prv_clear_pending_interrupts();
  new_timer_start(s_debounce_charger_timer, CHARGER_DEBOUNCE_MS,
                  prv_handle_charge_state_change, NULL, 0 /*flags*/);
}

static void prv_npm1300_interrupt_handler(bool *should_context_switch) {
  system_task_add_callback_from_isr(prv_pmic_state_change_cb, NULL, should_context_switch);
}

static void prv_configure_interrupts(void) {
  prv_clear_pending_interrupts();

  exti_configure_pin(BOARD_CONFIG_POWER.pmic_int, ExtiTrigger_Rising, prv_npm1300_interrupt_handler);
  exti_enable(BOARD_CONFIG_POWER.pmic_int);
}

bool pmic_init(void) {
  bool ok = true;
  uint8_t val;

  s_debounce_charger_timer = new_timer_create();

  // TODO(NPM1300): This needs to be configurable at board level
#ifdef CONFIG_BOARD_ASTERIX
  // Anomaly 27: set BUCK1/BUCK2 to SW control with workaround
  ok &= prv_buck_set_sw_ctrl(PmicRegisters_BUCK_BUCK1NORMVOUT,
                              PmicRegisters_BUCK_BUCK1VOUTSTATUS,
                              PmicRegisters_BUCK_BUCKSWCTRLSEL__BUCK1SWCTRLSEL_SWCTRL,
                              8 /* 1.8V */);
  ok &= prv_buck_set_sw_ctrl(PmicRegisters_BUCK_BUCK2NORMVOUT,
                              PmicRegisters_BUCK_BUCK2VOUTSTATUS,
                              PmicRegisters_BUCK_BUCKSWCTRLSEL__BUCK2SWCTRLSEL_SWCTRL,
                              20 /* 3.0V */);
  
  if (!prv_read_register(PmicRegisters_LDSW_LDSWSTATUS, &val)) {
    PBL_LOG_ERR("failed to read LDSWSTATUS");
    return false;
  }

  if ((val & PmicRegisters_LDSW_LDSWSTATUS__LDSW2PWRUPLDO) == 0U) {
    ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2CLR, 0x01);
    ok &= prv_write_register(PmicRegisters_LDSW_LDSW2VOUTSEL, 8 /* 1.8V */);
    ok &= prv_write_register(PmicRegisters_LDSW_LDSW2LDOSEL, 1 /* LDO */);
    ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2SET, 0x01);
  } else {
    ok &= prv_write_register(PmicRegisters_LDSW_LDSW2VOUTSEL, 8 /* 1.8V */);
  }
#endif

// FIXME(OBELIX,GETAFIX): Needs to be configurable at board level
#if defined(CONFIG_BOARD_OBELIX) || defined(CONFIG_BOARD_GETAFIX)
  // Anomaly 27: set BUCK1 to SW control with workaround, then disable it
  ok &= prv_buck_set_sw_ctrl(PmicRegisters_BUCK_BUCK1NORMVOUT,
                              PmicRegisters_BUCK_BUCK1VOUTSTATUS,
                              PmicRegisters_BUCK_BUCKSWCTRLSEL__BUCK1SWCTRLSEL_SWCTRL,
                              8 /* 1.8V */);
  ok &= prv_write_register(PmicRegisters_BUCK_BUCK1ENACLR, 1);
  //enable 1.8V@LDO1
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW1LDOSEL, 1);  //LDO
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW1VOUTSEL, 8);  //1.8V
  ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW1SET, 1); //enable
#endif

  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSBCHARGER1CLR, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSVBUSIN0CLR, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOMODE1, PmicRegisters_GPIOS_GPIOMODE__GPOIRQ);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOOPENDRAIN1, 0);

  ok &= prv_write_register(PmicRegisters_SHIP_SHPHLDCONFIG, PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS);
  ok &= prv_write_register(PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE, 1);

  // automatic IBAT measurement after VBAT
  ok &= prv_write_register(PmicRegisters_ADC_ADCIBATMEASEN, 1);

  if ((NPM1300_CONFIG.chg_current_ma < 32U) || (NPM1300_CONFIG.chg_current_ma > 800U) ||
      (NPM1300_CONFIG.chg_current_ma % 2U != 0U)) {
    PBL_LOG_ERR("Invalid charge current: %d mA", NPM1300_CONFIG.chg_current_ma);
    return false;
  }

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGENABLECLR, 1);

  ok &= prv_write_register(PmicRegisters_BCHARGER_TASKCLEARCHGERR, 1);
  ok &= prv_write_register(PmicRegisters_BCHARGER_TASKRELEASEERROR, 1);

  // FIXME: this needs to be configurable at board level
#ifdef CONFIG_BOARD_OBELIX
  ok &= prv_write_register(PmicRegisters_ADC_ADCNTCRSEL, PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERM, PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V35);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERMR, PmicRegisters_BCHARGER_BCHGVTERMR__BCHGVTERMREDUCED_4V00);
#elif defined(CONFIG_BOARD_GETAFIX)
  ok &= prv_write_register(PmicRegisters_ADC_ADCNTCRSEL, PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERM, PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V45);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERMR, PmicRegisters_BCHARGER_BCHGVTERMR__BCHGVTERMREDUCED_4V00);
#elif defined(CONFIG_BOARD_ASTERIX)
  ok &= prv_write_register(PmicRegisters_ADC_ADCNTCRSEL, PmicRegisters_ADC_ADCNTCRSEL__ADCNTCRSEL_10K);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERM, PmicRegisters_BCHARGER_BCHGVTERM__BCHGVTERMNORM_4V20);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVTERMR, PmicRegisters_BCHARGER_BCHGVTERMR__BCHGVTERMREDUCED_4V00);
#endif

  {
    uint16_t code = prv_ntc_threshold_code(NPM1300_CONFIG.ntc_hot_celsius);
    ok &= prv_write_register(PmicRegisters_BCHARGER_NTCHOT, (uint8_t)(code >> 2));
    ok &= prv_write_register(PmicRegisters_BCHARGER_NTCHOTLSB, (uint8_t)(code & 0x3U));
  }

  // FIXME: this needs to be configurable at board level
#ifdef CONFIG_BOARD_OBELIX
  //3.3V @ LDO2
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW2LDOSEL, PmicRegisters_LDSW_LDSW2LDOSEL__LDO_MODE);
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW2VOUTSEL, PmicRegisters_LDSW_LDSW2VOUTSEL__3V3);
  ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2CLR, 1);
#elif defined(CONFIG_BOARD_GETAFIX)
  // LDSW2 (3.3V for PDM)
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW2LDOSEL, PmicRegisters_LDSW_LDSW2LDOSEL__LDSW_MODE);
  ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2CLR, 1);
#endif

  val = (uint8_t)(NPM1300_CONFIG.chg_current_ma / 4U);
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGISETMSB, val);
  val = (NPM1300_CONFIG.chg_current_ma / 2U) % 2U;
  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGISETLSB, val);

  ok &= dischg_limit_ma_set(NPM1300_CONFIG.dischg_limit_ma);

  if (NPM1300_CONFIG.vbus_current_startup != 0) {
    ok &= prv_write_register(PmicRegisters_VBUSIN_VBUSINILIMSTARTUP,
      NPM1300_CONFIG.vbus_current_startup/NPM1300_VBUS_CURRENT_DIVISOR);
  }

  if (NPM1300_CONFIG.term_current_pct == 10U) {
    ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGITERMSEL,
                             PmicRegisters_BCHARGER_BCHGITERMSEL__SEL10);
  } else if(NPM1300_CONFIG.term_current_pct == 20U) {
    ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGITERMSEL,
                             PmicRegisters_BCHARGER_BCHGITERMSEL__SEL20);
  } else {
    PBL_LOG_ERR("Invalid termination current: %d", NPM1300_CONFIG.term_current_pct);
    return false;
  }

  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL0);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL1);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL2);
  ok &= prv_write_register(PmicRegisters_SYSTEM_TESTACCESS, 
                           PmicRegisters_SYSTEM_TESTACCESS__VAL3);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGDEBUG,
                           PmicRegisters_BCHARGER_BCHGDEBUG__DISABLEBATTERYDETECT);

  ok &= prv_write_register(PmicRegisters_BCHARGER_BCHGVBATLOWCHARGE, 1);

  prv_configure_interrupts();

  if (!ok) {
    PBL_LOG_ERR("one or more PMIC transactions failed");
  }

  return ok;
}

bool pmic_power_off(void) {
  // TODO: review implementation, see GH-238
  if (pmic_is_usb_connected()) {
    PBL_LOG_ERR("USB is connected, cannot power off");
    return false;
  }

  if (!prv_write_register(PmicRegisters_SHIP_TASKENTERSHIPMODE, 1)) {
    PBL_LOG_ERR("Failed to enter ship mode");
    return false;
  }

  // Give enough time for the PMIC to fully power down (tPWRDN = 100ms).
  // We will die here, if we do not, return false and let upper layers handle
  // the shutdown failure.
  delay_us(100000);

  return false;
}

bool pmic_full_power_off(void) {
  return pmic_power_off();
}

uint16_t pmic_get_vsys(void) {
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR, 0x08 /* EVENTADCVSYSRDY */)) {
    return 0;
  }
  if (!prv_write_register(PmicRegisters_ADC_TASKVSYSMEASURE, 1)) {
    return 0;
  }
  uint8_t reg = 0;
  uint32_t elapsed = 0;
  while ((reg & 0x08) == 0) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return 0;  // Timeout waiting for ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
    }
    if ((reg & 0x08) == 0) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }
  
  uint8_t vsys_msb;
  uint8_t lsbs;
  if (!prv_read_register(PmicRegisters_ADC_ADCVSYSRESULTMSB, &vsys_msb)) {
    return 0;
  }
  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsbs)) {
    return 0;
  }
  uint16_t vsys_raw = (vsys_msb << 2) | (lsbs >> 6);
  uint32_t vsys = vsys_raw * 6375 / 1023;
  
  return vsys;
}

int battery_get_millivolts(void) {
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR, 0x01 /* EVENTADCVBATRDY */)) {
    return 0;
  }
  if (!prv_write_register(PmicRegisters_ADC_TASKVBATMEASURE, 1)) {
    return 0;
  }
  uint8_t reg = 0;
  uint32_t elapsed = 0;
  while ((reg & 0x01) == 0) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return 0;  // Timeout waiting for ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
    }
    if ((reg & 0x01) == 0) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }
  
  uint8_t vbat_msb;
  uint8_t lsbs;
  if (!prv_read_register(PmicRegisters_ADC_ADCVBATRESULTMSB, &vbat_msb)) {
    return 0;
  }
  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsbs)) {
    return 0;
  }
  uint16_t vbat_raw = (vbat_msb << 2) | (lsbs & 3);
  uint32_t vbat = vbat_raw * 5000 / 1023;
  
  return vbat;
}

int battery_get_constants(BatteryConstants *constants) {
  uint8_t ibat_status;
  int32_t full_scale_ua;
  uint8_t msb;
  uint8_t lsb;
  uint16_t raw;
  uint8_t reg;

  // Obtain IBAT full scale
  if (!prv_read_register(PmicRegisters_ADC_ADCIBATMEASSTATUS, &ibat_status)) {
    return -1;
  }

  if ((ibat_status & PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_MASK) ==
      PmicRegisters_ADC_ADCIBATMEASSTATUS__BCHARGERMODE_CHRG) {
    full_scale_ua =
        ((int32_t)NPM1300_CONFIG.chg_current_ma * 1000 * NPM1300_BCHARGER_ADC_CALC_CHARGE_MUL) /
        NPM1300_BCHARGER_ADC_CALC_CHARGE_DIV;
  } else {
    full_scale_ua =
        ((int32_t)s_dischg_limit_ma * 1000 * NPM1300_BCHARGER_ADC_CALC_DISCHARGE_MUL) /
        NPM1300_BCHARGER_ADC_CALC_DISCHARGE_DIV;
  }

  // Clear the ADC ready events for VBAT, IBAT, and NTC
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR,
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY |
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY |
                          PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY)) {
    return -1;
  }

  // Trigger VBAT+IBAT measurement (IBATMEASENABLE is enabled)
  if (!prv_write_register(PmicRegisters_ADC_TASKVBATMEASURE, 1)) {
    return -1;
  }

  // Trigger NTC measurement
  if (!prv_write_register(PmicRegisters_ADC_TASKNTCMEASURE, 1)) {
    return -1;
  }

  // Process the VBAT measurement
  reg = 0U;
  uint32_t elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for VBAT ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCVBATRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCVBATRESULTMSB, &msb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &lsb)) {
    return -1;
  }

  raw = (msb << NPM1300_ADC_MSB_SHIFT) |
        ((lsb >> PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP0RESULTLSBS_VBATRESULTLSB_MSK);

  constants->v_mv = (int32_t)(raw * NPM1300_ADC_VFS_VBAT_MV) / NPM1300_BCHARGER_ADC_BITS_RESOLUTION;

  // Process the IBAT measurement
  elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for IBAT ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCIBATRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCVBAT2RESULTMSB, &msb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP1RESULTLSBS, &lsb)) {
    return -1;
  }

  raw = (msb << NPM1300_ADC_MSB_SHIFT) |
        ((lsb >> PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP1RESULTLSBS_VBAT2RESULTLSB_MSK);

  constants->i_ua = ((int32_t)raw * full_scale_ua) / NPM1300_BCHARGER_ADC_BITS_RESOLUTION;

  // Process the NTC measurement
  elapsed = 0;
  while ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY) == 0U) {
    if (elapsed >= ADC_POLL_TIMEOUT_MS) {
      return -1;  // Timeout waiting for NTC ADC
    }
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return -1;
    }
    if ((reg & PmicRegisters_MAIN_EVENTSADCCLR__EVENTADCNTCRDY) == 0U) {
      psleep(ADC_POLL_DELAY_MS);
      elapsed += ADC_POLL_DELAY_MS;
    }
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCNTCRESULTMSB, &lsb)) {
    return -1;
  }

  if (!prv_read_register(PmicRegisters_ADC_ADCGP0RESULTLSBS, &msb)) {
    return -1;
  }

  raw = (lsb << NPM1300_ADC_MSB_SHIFT) |
        ((msb >> PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_POS) &
         PmicRegisters_ADC_ADCGP0RESULTLSBS_NTCRESULTLSB_MSK);

  // Ref: PS v1.2 Section 7.1.4: Battery temperature (Kelvin)
  float log_result = logf((1024.f / (float)raw) - 1.0f);
  float inv_temp_k = (1.f / 298.15f) - (log_result / (float)NPM1300_CONFIG.thermistor_beta);

  constants->t_mc = (int32_t)(1000.0f * ((1.f / inv_temp_k) - 273.15f));

  return 0;
}

bool pmic_set_charger_state(bool enable) {
  return prv_write_register(enable ? PmicRegisters_BCHARGER_BCHGENABLESET : PmicRegisters_BCHARGER_BCHGENABLECLR, 1);
}

void battery_set_charge_enable(bool charging_enabled) {
  pmic_set_charger_state(charging_enabled);
}

void battery_set_fast_charge(bool fast_charge_enabled) {
  /* the PMIC handles this for us */
}

bool pmic_is_charging(void) {
  uint8_t status;
  if (!prv_read_register(PmicRegisters_BCHARGER_BCHGCHARGESTATUS, &status)) {
    return false;
  }

  return (status & (PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE | PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT | PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE)) != 0;
}

bool battery_charge_controller_thinks_we_are_charging_impl(void) {
  return pmic_is_charging();
}

bool pmic_is_usb_connected(void) {
  uint8_t status;
  if (!prv_read_register(PmicRegisters_VBUSIN_VBUSINSTATUS, &status)) {
    return false;
  }

  return (status & PmicRegisters_VBUSIN_VBUSINSTATUS__VBUSINPRESENT) != 0;
}

bool battery_is_usb_connected_impl(void) {
  return pmic_is_usb_connected();
}

void pmic_read_chip_info(uint8_t *chip_id, uint8_t *chip_revision, uint8_t *buck1_vset) {
}

bool pmic_enable_battery_measure(void) {
  return true;
}

bool pmic_disable_battery_measure(void) {
  return true;
}

void set_ldo3_power_state(bool enabled) {
}

void set_4V5_power_state(bool enabled) {
}

void set_6V6_power_state(bool enabled) {
}

int battery_charge_status_get(BatteryChargeStatus *status) {
  uint8_t chg_status;

  if (!prv_read_register(PmicRegisters_BCHARGER_BCHGCHARGESTATUS, &chg_status)) {
    return -1;
  }

  switch (chg_status & (PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT |
                        PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE)) {
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED:
      *status = BatteryChargeStatusComplete;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE:
      *status = BatteryChargeStatusTrickle;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT:
      *status = BatteryChargeStatusCC;
      break;
    case PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE:
      *status = BatteryChargeStatusCV;
      break;
    default:
      *status = BatteryChargeStatusUnknown;
      break;
  }

  return 0;
}

void command_pmic_read_registers(void) {
  char buffer[64];
#define SAY(x) do { uint8_t reg; int rv = prv_read_register(PmicRegisters_##x, &reg); prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: " #x " = %02x (rv %d)", reg, rv); } while(0)
  SAY(ERRLOG_SCRATCH0);
  SAY(ERRLOG_SCRATCH1);
  SAY(BUCK_BUCK1NORMVOUT);
  SAY(BUCK_BUCK2NORMVOUT);
  SAY(BUCK_BUCKSTATUS);
  SAY(VBUSIN_VBUSINSTATUS);
  SAY(BCHARGER_BCHGCHARGESTATUS);
  SAY(BCHARGER_BCHGERRREASON);
  prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: Vsys = %d mV", pmic_get_vsys());
  prompt_send_response_fmt(buffer, sizeof(buffer), "PMIC: Vbat = %d mV", battery_get_millivolts());
}

void command_pmic_status(void) {
}

void command_pmic_rails(void) {
  // TODO: Implement.
}

static bool gpio_set(Npm1300GpioId_t id, bool is_high) {
  bool rv = false;
  switch (id) {
    case Npm1300_Gpio2:
      rv = prv_write_register(PmicRegisters_GPIOS_GPIOMODE2, 
          is_high ? PmicRegisters_GPIOS_GPIOMODE__OUTPUT_HIGH : PmicRegisters_GPIOS_GPIOMODE__OUTPUT_LOW);
      rv &= prv_write_register(PmicRegisters_GPIOS_GPIOPUEN2,
          is_high ? PmicRegisters_GPIOS_GPIOPUEN__EN : PmicRegisters_GPIOS_GPIOPUEN__DIS);
      break;
    case Npm1300_Gpio3: {
      rv = prv_write_register(PmicRegisters_GPIOS_GPIOMODE3, 
          is_high ? PmicRegisters_GPIOS_GPIOMODE__OUTPUT_HIGH : PmicRegisters_GPIOS_GPIOMODE__OUTPUT_LOW);
      rv &= prv_write_register(PmicRegisters_GPIOS_GPIOPUEN3,
          is_high ? PmicRegisters_GPIOS_GPIOPUEN__EN : PmicRegisters_GPIOS_GPIOPUEN__DIS);
      break;
    }
    default:
      break;
  }

  return rv;
}

static bool ldo2_set_enabled(bool enabled) {
  if (enabled) {
    return prv_write_register(PmicRegisters_LDSW_TASKLDSW2SET, 1);
  } else {
    return prv_write_register(PmicRegisters_LDSW_TASKLDSW2CLR, 1);
  }
}

static bool dischg_limit_ma_set(uint32_t dischg_limit_ma) {
  bool ret;

  if (s_dischg_limit_ma == dischg_limit_ma) {
    return true;
  }

  if (dischg_limit_ma == 200) {
    ret = prv_write_register(PmicRegisters_BCHARGER_BCHGISETDISCHARGEMSB,
                             NPM1300_BCHGISETDISCHARGEMSB_200MA);
    if (!ret) {
      return ret;
    }

    ret = prv_write_register(PmicRegisters_BCHARGER_BCHGISETDISCHARGELSB,
                             NPM1300_BCHGISETDISCHARGELSB_200MA);
    if (!ret) {
      return ret;
    }
  } else if (dischg_limit_ma == 1000) {
    ret = prv_write_register(PmicRegisters_BCHARGER_BCHGISETDISCHARGEMSB,
                             NPM1300_BCHGISETDISCHARGEMSB_1000MA);
    if (!ret) {
      return ret;
    }

    ret = prv_write_register(PmicRegisters_BCHARGER_BCHGISETDISCHARGELSB,
                             NPM1300_BCHGISETDISCHARGELSB_1000MA);
    if (!ret) {
      return ret;
    }
  } else {
    PBL_LOG_ERR("Invalid discharge limit: %" PRIu32 " mA", dischg_limit_ma);
    return false;
  }

  s_dischg_limit_ma = dischg_limit_ma;

  return true;
}

Npm1300Ops_t NPM1300_OPS = {
  .gpio_set = gpio_set,
  .ldo2_set_enabled = ldo2_set_enabled,
  .dischg_limit_ma_set = dischg_limit_ma_set,
};
