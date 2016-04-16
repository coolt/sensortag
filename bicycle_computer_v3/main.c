/**
 V3:
 */
// #include "interfaces/board-i2c.h"


// Set up
#include <driverLib/ioc.h>				// Grundeinstallung aktive domains
#include <driverLib/sys_ctrl.h>			// Bus, CPU, Refresh
#include <config.h>						// Konstanten Applkation
#include <system.h>						// Funktionen (Power), Init, Waits

// Sensors
#include "sensor-common.h"
#include "ext-flash.h"
#include "bmp-280-sensor.h"
#include "tmp-007-sensor.h"
#include "hdc-1000-sensor.h"
#include "opt-3001-sensor.h"

// GPIO
#include "board.h"						// Konstanten IO
#include <driverLib/gpio.h>				// Konstanten GPIO Pins

// RF-Chip (M0)
#include <radio.h>
#include <driverLib/rfc.h>				// Set up RFC interrupts

// RTC
#include <rtc.h>
#include <driverLib/aon_rtc.h>
#include <inc/hw_aon_event.h>


// globale variables: declared in config.h, used in radio.c and startup_ccs
volatile bool rfBootDone;
volatile bool rfSetupDone;
volatile bool rfAdvertisingDone;
uint8_t payload[ADVLEN];




int main(void) {

  uint8_t payload[ADVLEN];

  //Disable JTAG to allow for Standby
  AONWUCJtagPowerOff();

  //Force AUX on
  powerEnableAuxForceOn();
  powerEnableRFC();

  powerEnableXtalInterface();
  //powerConfigureRecharge(); --> Optimized version later in this code (brts)
  
  // Divide INF clk to save Idle mode power (increases interrupt latency)
  powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32);

  initRTC();

  powerEnablePeriph();
  powerEnableGPIOClockRunMode();

  /* Wait for domains to power on */
  while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

  sensorsInit();
  ledInit();

  /*
   * erstes Mail
   *  HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU1_EV_PAD | AON_EVENT_MCUWUSEL_WU0_EV_RTC_CH2;  //Does not work with AON_EVENT_MCUWUSEL_WU0_EV_PAD4 --> WHY??
   */

  /*
   * Zweites Mail
   *   AONRTCIncValueCh2Set(WAKE_INTERVAL_TICKS);
   */

  //Config IOID4 for external interrupt on rising edge and wake up
 IOCPortConfigureSet(BOARD_IOID_KEY_RIGHT, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_FALLING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_UP | IOC_INPUT_ENABLE | IOC_WAKE_ON_LOW);
  //Set device to wake MCU from standby on PIN 4 (BUTTON1)
 HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU1_EV_PAD | AON_EVENT_MCUWUSEL_WU2_EV_RTC_CH0 | AON_EVENT_MCUWUSEL_WU0_EV_RTC_CH2;  //Does not work with AON_EVENT_MCUWUSEL_WU0_EV_PAD4 --> WHY??

  IntEnable(INT_EDGE_DETECT);

  powerDisablePeriph();
  //Disable clock for GPIO in CPU run mode
  HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
  // Load clock settings
  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;

  initRFInterrupts(); // new: vorher initInterrupts()
  CPUcpsie();
  initRadio();

  // Turn off FLASH in idle mode
  powerDisableFlashInIdle();

  // Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
  powerEnableCacheRetention();

  //AUX - request to power down (takes no effect since force on is set)
  powerEnableAUXPdReq();
  powerDisableAuxRamRet();

  //Clear payload buffer
    memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);

    //Fill payload buffer with adv parameter data
    uint8_t p;
    p = 0;
    payload[p++] = 0x02;          /* 2 bytes */
    payload[p++] = 0x01;
    payload[p++] = 0x05;          /* LE Limited Discoverable Mode" & "BR/EDR Not Supported */
    payload[p++] = 0x09; //1 + strlen(beacond_config.adv_name);
    payload[p++] = 0x03;//BLE_ADV_TYPE_NAME;
    payload[p++] = 0x00;//BLE_ADV_TYPE_NAME;
    payload[p++] = 0xDE;//BLE_ADV_TYPE_NAME;
    payload[p++] = 0x01;//BLE_ADV_TYPE_NAME;
    payload[p++] = 0x35;//BLE_ADV_TYPE_NAME;

  //Start radio setup and linked advertisment
  radioUpdateAdvData(10, payload);

  while(1) {

    rfBootDone  = 0;
    rfSetupDone = 0;
    rfAdvertisingDone = 0;

    //Wait until RF Core PD is ready before accessing radio
    waitUntilRFCReady();
    initRadioInts();
    runRadio();

    //Wait until AUX is ready before configuring oscillators
    waitUntilAUXReady();

    //Enable 24MHz XTAL
    OSCHF_TurnOnXosc();

    //IDLE until BOOT_DONE interrupt from RFCore is triggered
    while( ! rfBootDone) {
      powerDisableCPU();
      PRCMDeepSleep();
    }

    //This code runs after BOOT_DONE interrupt has woken up the CPU again
    // ->
    //Request radio to keep on system bus
    radioCmdBusRequest(true);

    //Patch CM0 - no RFE patch needed for TX only
    radioPatch();

    //Start radio timer
    radioCmdStartRAT();

    //Enable Flash access while doing radio setup
    powerEnableFlashInIdle();

    //Switch to XTAL
    while( !OSCHF_AttemptToSwitchToXosc())
    {}
  
    //Start radio setup and linked advertisment
    radioSetupAndTransmit();

    //Wait in IDLE for CMD_DONE interrupt after radio setup. ISR will disable radio interrupts
    while( ! rfSetupDone) {
      powerDisableCPU();
      PRCMDeepSleep();
    }

    //Disable flash in IDLE after CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
    powerDisableFlashInIdle();

    //Wait in IDLE for LAST_CMD_DONE after 3 adv packets
    while( ! rfAdvertisingDone) {
      powerDisableCPU();
      PRCMDeepSleep();
    }

    //Request radio to not force on system bus any more
    radioCmdBusRequest(false);
    
    //
    // Standby procedure
    //
    
    powerDisableXtal();

    // Turn off radio
    powerDisableRFC();
    
    // Switch to RCOSC_HF
    OSCHfSourceSwitch();

    // Allow AUX to turn off again. No longer need oscillator interface
    powerDisableAuxForceOn();

    // Goto Standby. MCU will now request to be powered down on DeepSleep
    powerEnableMcuPdReq();

    // Disable cache and retention
    powerDisableCache();
    powerDisableCacheRetention();

    //Calculate next recharge
    SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);

    // Synchronize transactions to AON domain to ensure AUX has turned off
    SysCtrlAonSync();

    //
    // Enter Standby
    //

    powerDisableCPU();
    PRCMDeepSleep();

    SysCtrlAonUpdate();
    SysCtrlAdjustRechargeAfterPowerDown();
    SysCtrlAonSync();

    //
	// Wakeup from RTC every 100ms, code starts execution from here
	//
   
    powerEnableRFC();
    powerEnableAuxForceOn();

    //Re-enable cache and retention
    powerEnableCache();
    powerEnableCacheRetention();

    //MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
    powerDisableMcuPdReq();
  }
}
