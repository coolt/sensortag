/**
  @file  main.c
  @brief main entry of simpleBroadcaster, a bare-bones, speed optimized
         program transmitting BLE advertisment packages every N ms based on input from 5
         GPIOs

  @usage
        - Make a copy of ccfg.c from your CC26XXWARE version and
          configure it to use internal LF RCOSC
        - Configure WAKE_INTERVAL
        - Configure recharge period to 400ms if WAKE_INTERVAL is larger than 400ms(ish)
        - Configure IO's and set up advertisment payload
        - Configure output power to desired value in CMD_RADIO_SETUP (see pa_table_cc26xx.c)

  <!--
  Copyright 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ``AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
  -->
*/


#include <driverLib/ioc.h>
#include <driverLib/sys_ctrl.h>

#include "config.h"
#include "rtc.h"
#include "radio.h"
#include "system.h"

extern volatile bool int_boot_done;

float temp,hum;
uint16_t raw_temp;
uint16_t raw_hum;

static inline void initIO(void) {

  powerEnablePeriph();
  powerEnableGPIOClockRunMode();

  // Set IO as input in IO controller, by default routed to GPIO module
  // TODO: Select pull level if needed
  //HWREG(IOC_BASE + 4*INPUT_DIO) = (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_UP | IOC_HYST_ENABLE;
  HWREG(IOC_BASE + 4*IO_A ) = (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_DOWN | IOC_HYST_ENABLE;
  HWREG(IOC_BASE + 4*IO_B) =  (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_DOWN | IOC_HYST_ENABLE;
  HWREG(IOC_BASE + 4*IO_C) =  (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_DOWN | IOC_HYST_ENABLE;
  HWREG(IOC_BASE + 4*IO_D) =  (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_DOWN | IOC_HYST_ENABLE;
  HWREG(IOC_BASE + 4*IO_E) =  (IOC_STD_INPUT& (~IOC_NO_IOPULL)) | IOC_IOPULL_DOWN | IOC_HYST_ENABLE;

  waitUntilPeriphReady();

}



int main(void) {

	int i,p = 0;
	uint8_t payload[BLE_ADV_PAYLOAD_BUF_LEN];


// IAR I-jet SWO output on RF2.16
//  IOCPortConfigureSet(IOID_28, IOC_PORT_MCU_SWV, IOC_STD_OUTPUT);

  //Disable JTAG to allow for Standby
  AONWUCJtagPowerOff();

  //Force AUX on
  powerEnableAuxForceOn();
  powerEnableRFC();
  powerEnableXtalInterface();
  powerConfigureRecharge();

  initRTC();

  //  initIO()
  // Divide INF clk to save Idle mode power (increases interrupt latency)
  //  powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32);

  initInterrupts();
  initRadio();

  // Turn off FLASH in idle mode
  powerDisableFlashInIdle();

  // Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
  powerEnableCacheRetention();

  //AUX - request to power down (takes no effect since force on is set)
  powerEnableAUXPdReq();
  powerDisableAuxRamRet();

  //Ensure transactions to AON domain are done
  SysCtrlAonSync();


  //Clear payload buffer
  memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);

  //Fill payload buffer with adv parameter data
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

  while(1) {

    //Wait until RF Core PD is ready before accessing radio
    waitUntilRFCReady();
    initRadioInts();
    runRadio();

    //Wait until AUX is ready before configuring oscillators
    waitUntilAUXReady();
    //Enable 24MHz XTAL
    powerEnableXtal();

    //IDLE until BOOT_DONE interrupt from RFCore is triggered

    CPUcpsid(); //Critical section, disable interrupts
    if(int_boot_done == 0) {
      powerDisableCPU();
      PRCMDeepSleep();
    }



    CPUcpsie(); //Enable interrupts, RFCCPE1IntHandler will execute

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

    //change user specific data
    p=9;
    payload[p++] =  raw_hum&0x00FF;
    payload[p++] =  (raw_hum>>8)&0x00FF;
    payload[p++] =  raw_temp&0x00FF;
    payload[p++] =  (raw_temp>>8)&0x00FF;

    //Start radio setup and linked advertisment
    radioUpdateAdvData(p,payload);
    radioSetupAndTransmit();

    //Wait in IDLE for CMD_DONE interrupt after radio setup. ISR will disable radio interrupts
    powerDisableCPU();
    //Ensure transactions to AON domain are done (clearing RTC interrupt out from standby)
    SysCtrlAonSync();
    PRCMDeepSleep();

    //Disable flash in IDLE after CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
    powerDisableFlashInIdle();

    //Wait for LAST_CMD_DONE after 3 adv packets
    powerDisableCPU();
    PRCMDeepSleep();

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

    // Synchronize transactions to AON domain to ensure AUX has turned off
    SysCtrlAonSync();

    //
    // Enter Standby --- here KILL VSUP
    //

    powerDisableCPU();
    PRCMDeepSleep();

    //Wakeup from RTC every 100ms, code starts execution from here
    powerEnableRFC();
    powerEnableAuxForceOn();

    //Re-enable cache and retention
    powerEnableCache();
    powerEnableCacheRetention();

    //MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
    powerDisableMcuPdReq();
  }
}
