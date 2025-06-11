/* Because nPM1300 also has the battery monitor, we implement both the
 * pmic_* and the battery_* API here.  */

#include "drivers/pmic.h"
#include "drivers/battery.h"

#include "board/board.h"
#include "console/prompt.h"
#include "drivers/battery.h"
#include "drivers/exti.h"
#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "drivers/periph_config.h"
#include "kernel/events.h"
#include "kernel/util/delay.h"
#include "os/mutex.h"
#include "services/common/system_task.h"
#include "system/logging.h"
#include "system/passert.h"

#define CHARGER_DEBOUNCE_MS 400
static TimerID s_debounce_charger_timer = TIMER_INVALID_ID;
static PebbleMutex *s_i2c_lock;

typedef enum {
  PmicRegisters_MAIN_EVENTSADCCLR = 0x0003,
  PmicRegisters_MAIN_EVENTSBCHARGER1CLR = 0x000B,
  PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET = 0x000C,
  PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED = 16,
  PmicRegisters_MAIN_EVENTSVBUSIN0CLR = 0x0017,
  PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET = 0x0018,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED = 1,
  PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED = 2,
  PmicRegisters_VBUSIN_VBUSINSTATUS = 0x0207,
  PmicRegisters_VBUSIN_VBUSINSTATUS__VBUSINPRESENT = 1,
  PmicRegisters_BCHARGER_BCHGENABLESET = 0x0304,
  PmicRegisters_BCHARGER_BCHGENABLECLR = 0x0305,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS = 0x0334,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__BATTERYDETECTED = 1,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__COMPLETED = 2,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__TRICKLECHARGE = 4,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTCURRENT = 8,
  PmicRegisters_BCHARGER_BCHGCHARGESTATUS__CONSTANTVOLTAGE = 16,
  PmicRegisters_BCHARGER_BCHGERRREASON = 0x0336,
  PmicRegisters_ADC_TASKVBATMEASURE  = 0x0500,
  PmicRegisters_ADC_TASKNTCMEASURE   = 0x0501,
  PmicRegisters_ADC_TASKVSYSMEASURE  = 0x0503,
  PmicRegisters_ADC_TASKIBATMEASURE  = 0x0506,
  PmicRegisters_ADC_TASKVBUS7MEASURE = 0x0507,
  PmicRegisters_ADC_ADCVBATRESULTMSB = 0x0511,
  PmicRegisters_ADC_ADCVSYSRESULTMSB = 0x0514,
  PmicRegisters_ADC_ADCGP0RESULTLSBS = 0x0515,
  PmicRegisters_GPIOS_GPIOMODE1 = 0x0601,
  PmicRegisters_GPIOS_GPIOMODE__GPOIRQ = 5,
  PmicRegisters_GPIOS_GPIOOPENDRAIN1 = 0x0615,
  PmicRegisters_ERRLOG_SCRATCH0 = 0x0E01,
  PmicRegisters_ERRLOG_SCRATCH1 = 0x0E02,
  PmicRegisters_BUCK_BUCK1NORMVOUT = 0x0408,
  PmicRegisters_BUCK_BUCK2NORMVOUT = 0x040A,
  PmicRegisters_BUCK_BUCKSTATUS = 0x0434,
  PmicRegisters_LDSW_TASKLDSW1SET = 0x0800,
  PmicRegisters_LDSW_TASKLDSW1CLR = 0x0801,
  PmicRegisters_LDSW_TASKLDSW2SET = 0x0802,
  PmicRegisters_LDSW_TASKLDSW2CLR = 0x0803,
  PmicRegisters_LDSW_LDSWSTATUS = 0x0804,
  PmicRegisters_LDSW_LDSWCONFIG = 0x0807,
  PmicRegisters_LDSW_LDSW1LDOSEL = 0x0808,
  PmicRegisters_LDSW_LDSW2LDOSEL = 0x0809,
  PmicRegisters_LDSW_LDSW1VOUTSEL = 0x080C,
  PmicRegisters_LDSW_LDSW2VOUTSEL = 0x080D,
  PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE = 0x0B01,
  PmicRegisters_SHIP_TASKENTERSHIPMODE = 0x0B02,
  PmicRegisters_SHIP_SHPHLDCONFIG = 0x0B04,
  PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS = 3,
} PmicRegisters;

void battery_init(void) {
}

uint32_t pmic_get_last_reset_reason(void) {
  return 0;
}

static bool prv_read_register(uint16_t register_address, uint8_t *result) {
  mutex_lock(s_i2c_lock);
  i2c_use(I2C_NPM1300);
  uint8_t regad[2] = { register_address >> 8, register_address & 0xFF };
  bool rv = i2c_write_block(I2C_NPM1300, 2, regad);
  if (rv)
    rv = i2c_read_block(I2C_NPM1300, 1, result);
  i2c_release(I2C_NPM1300);
  mutex_unlock(s_i2c_lock);
  return rv;
}

static bool prv_write_register(uint16_t register_address, uint8_t datum) {
  mutex_lock(s_i2c_lock);
  i2c_use(I2C_NPM1300);
  uint8_t d[3] = { register_address >> 8, register_address & 0xFF, datum };
  bool rv = i2c_write_block(I2C_NPM1300, 3, d);
  i2c_release(I2C_NPM1300);
  mutex_unlock(s_i2c_lock);
  return rv;
}

static void prv_handle_charge_state_change(void *null) {
  const bool is_charging = pmic_is_charging();
  const bool is_connected = pmic_is_usb_connected();
  PBL_LOG(LOG_LEVEL_DEBUG, "nPM1300 Interrupt: Charging? %s Plugged? %s",
      is_charging ? "YES" : "NO", is_connected ? "YES" : "NO");

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

  s_i2c_lock = mutex_create();
  s_debounce_charger_timer = new_timer_create();

  uint8_t buck_out;
  if (!prv_read_register(PmicRegisters_BUCK_BUCK1NORMVOUT, &buck_out)) {
    PBL_LOG(LOG_LEVEL_ERROR, "failed to read BUCK1NORMVOUT");
    return false;
  }
  PBL_LOG(LOG_LEVEL_DEBUG, "found the nPM1300, BUCK1NORMVOUT = 0x%x", buck_out);
  
  ok &= prv_read_register(PmicRegisters_LDSW_LDSWSTATUS, &buck_out);
  PBL_LOG(LOG_LEVEL_DEBUG, "nPM1300 LDSW status before enabling LDSW2 0x%x", buck_out);
  
  ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2CLR, 0x01);
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW2VOUTSEL, 8 /* 1.8V */);
  ok &= prv_write_register(PmicRegisters_LDSW_LDSW2LDOSEL, 1 /* LDO */);
  ok &= prv_write_register(PmicRegisters_LDSW_TASKLDSW2SET, 0x01);

  ok &= prv_read_register(PmicRegisters_LDSW_LDSWSTATUS, &buck_out);
  PBL_LOG(LOG_LEVEL_DEBUG, "nPM1300 LDSW status after enabling LDSW2 0x%x", buck_out);

  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSBCHARGER1CLR, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSBCHARGER1SET, PmicRegisters_MAIN_EVENTSBCHARGER1__EVENTCHGCOMPLETED);
  ok &= prv_write_register(PmicRegisters_MAIN_EVENTSVBUSIN0CLR, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_MAIN_INTENEVENTSVBUSIN0SET, PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSDETECTED | PmicRegisters_MAIN_EVENTSVBUSIN0__EVENTVBUSREMOVED);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOMODE1, PmicRegisters_GPIOS_GPIOMODE__GPOIRQ);
  ok &= prv_write_register(PmicRegisters_GPIOS_GPIOOPENDRAIN1, 0);

  ok &= prv_write_register(PmicRegisters_SHIP_SHPHLDCONFIG, PmicRegisters_SHIP_SHPHLDCONFIG__SHPHLDTIM_96MS);
  ok &= prv_write_register(PmicRegisters_SHIP_TASKSHPHLDCFGSTROBE, 1);

  prv_configure_interrupts();

  if (!ok) {
    PBL_LOG(LOG_LEVEL_ERROR, "one or more PMIC transactions failed");
  }

  return ok;
}

bool pmic_power_off(void) {
  // TODO: review implementation, see GH-238
  if (pmic_is_usb_connected()) {
    PBL_LOG(LOG_LEVEL_ERROR, "USB is connected, cannot power off");
    return false;
  }

  if (!prv_write_register(PmicRegisters_SHIP_TASKENTERSHIPMODE, 1)) {
    PBL_LOG(LOG_LEVEL_ERROR, "Failed to enter ship mode");
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
  while ((reg & 0x08) == 0) {
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
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

bool battery_is_present(void) {
  uint8_t reg = 0;
  if (!prv_read_register(PmicRegisters_BCHARGER_BCHGCHARGESTATUS, &reg)) {
    return false;
  }
  return (reg & PmicRegisters_BCHARGER_BCHGCHARGESTATUS__BATTERYDETECTED) != 0;
}

int battery_get_millivolts(void) {
  if (!prv_write_register(PmicRegisters_MAIN_EVENTSADCCLR, 0x01 /* EVENTADCVBATRDY */)) {
    return 0;
  }
  if (!prv_write_register(PmicRegisters_ADC_TASKVBATMEASURE, 1)) {
    return 0;
  }
  uint8_t reg = 0;
  while ((reg & 0x01) == 0) {
    if (!prv_read_register(PmicRegisters_MAIN_EVENTSADCCLR, &reg)) {
      return 0;
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
