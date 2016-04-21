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
#include "radio.h"
#include <driverLib/rfc.h>				// Set up RFC interrupts

// RTC
#include <rtc.h>
#include <driverLib/aon_rtc.h>
#include <inc/hw_aon_event.h>

#include "string.h"


// globale variables: declared in config.h, used in radio.c and startup_ccs
volatile bool rfBootDone;
volatile bool rfSetupDone;
volatile bool rfAdvertisingDone;

char payload[ADVLEN];


// functions
void initSensortag(void){

		// power off
		AONWUCJtagPowerOff(); 									//Disable JTAG to allow for Standby

		// power on
		powerEnableAuxForceOn(); 								// WUC domain
		powerEnableXtalInterface(); 							// clk WUC
		powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32); 	// Divide INF clk to save Idle mode power (increases interrupt latency)
		powerEnablePeriph();
		powerEnableGPIOClockRunMode();
		while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON)); /* Wait for domains to power on */

		// Configuartions
		sensorsInit();											// Enable needed Sensors
		initRadio();											// Set Communicationmode = BLE, Channels = 3, Advertising modus
		ledInit();

		// Configure Interrupts
		//initRTCInterrupts();		// Ziel							// CH0: WakeUp, CH2: Speed calculation
		initRTC();// Dario
		initGPIOInterrupts();									// Define IOPorts for Interrupt, Add GPIO-mask to WU-Event
		initRFInterrupts(); 									// Set RFInterrupts to NVIC
		CPUcpsie();												// All extern interrupts enable (globaly)

		// Setup for next state
		IntEnable(INT_EDGE_DETECT); // Dario
//		IntDisable(INT_EDGE_DETECT);							// Enable specific interrupt. Int_EDGE_DETECT = Nr. 16  (=> all GPIO-interrupts   ?? )
//		AONRTCEnable();											// PA: Enable RTC

		// power off and set Refresh on
		// -- moved functions from in the middle of interrupt settings
		powerDisablePeriph(); //Disable clock for GPIO in CPU run mode
		HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
		HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1; // Load clock settings
		// -- old functions
		powerDisableFlashInIdle();  							// Turn off FLASH in idle mode == stand by mode
		powerEnableCacheRetention(); 							// Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
		powerEnableAUXPdReq(); 									//AUX - request to power down (takes no effect since force on is set)
		powerDisableAuxRamRet();
}


void getData(void){
	// Wakeup from RTC every 100ms, code starts execution from here
		// ---------------------------------------------
		// WAITING FOR INTERRUPT
		// HERE: OLD CODE. FIX WAKE UP TIME
		powerEnableRFC(); // ????????????????????????????????????????????????
		powerEnableAuxForceOn(); // ??????????????????????' not done in RTC interrupt

		//Re-enable cache and retention
		powerEnableCache();
		powerEnableCacheRetention();

		//MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
		powerDisableMcuPdReq();

	int i = 8;

}
void setData(void){

	memset(payload, 0, ADVLEN); // Clear payload buffer  //DOES NOT WORK !!!!!!!!!!

	//Fill payload buffer with adv parameter data
	uint8_t p = 0;
	payload[p++] = 0x01;
	payload[p++] = 0x02;
	payload[p++] = 0x03;
	payload[p++] = 0x04;
	payload[p++] = 0x05;
	payload[p++] = 0x06;
	payload[p++] = 0x07;
	payload[p++] = 0x08;
	payload[p++] = 0x09;

	//Start radio setup and linked advertisment
	radioUpdateAdvData(10, payload); 			//Update advertising byte based on IO inputs
}


void sendData(){

	// Flags for RF-Doorbell-communication between CPU M3 and RFC M0
    rfBootDone  = 0;
    rfSetupDone = 0;
    rfAdvertisingDone = 0;

	powerEnableRFC(); 							// Set power bit
	waitUntilRFCReady();
	enableRadioInterrupts();  					// Set enable bit for CPE communication interrupts
	runRadio();									// Power CPU (M3), RAM and CPE (M0)

	waitUntilAUXReady(); 						// AUX is needed to configure higher oscillator
	OSCHF_TurnOnXosc();  						// Enable 24 MHz XTAL (higher clk for sending)
	int debug = rfBootDone; // = 0
	while( ! rfBootDone) { 						// rfBootDone set by CPE interrupt 												!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! hanging here BootDone = 0
		powerDisableCPU();
		PRCMDeepSleep();	//note:				// if commented out: Request radio to keep on system bus
	}
	radioCmdBusRequest(true); 					// Request radio to keep on system bus
	radioPatch(); 								// Patch CM0 - no RFE patch needed for TX only
	radioCmdStartRAT(); 						// Start radio timer
	powerEnableFlashInIdle(); 					// Enable Flash access while doing radio setup
	while( !OSCHF_AttemptToSwitchToXosc()) 		// Switch to XTAL, higher clock
	{}

	// Sending data
	radioSetupAndTransmit();
	while( ! rfSetupDone) {
		powerDisableCPU();
		PRCMDeepSleep();
	}
	powerDisableFlashInIdle(); 					// Disable flashafter CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
	while( ! rfAdvertisingDone) {				// AdvertisingDone = true, when 3 packets are send
	  powerDisableCPU();
	  PRCMDeepSleep();
	}
	radioCmdBusRequest(false);					// Request radio to not force on system bus any more

	RFCAckIntClear(); 	// add baek.			// clear RFC Interrupts
	AONRTCEnable();		// add baek. 			// defined interrupt state for next state
}


void sleep(){

	// Standby procedure
	powerDisableXtal();
	powerDisableRFC();
	OSCHfSourceSwitch(); 						// lower clk
	powerDisableAuxForceOn(); 					// Higher oscillator interface no more needed
	powerEnableMcuPdReq(); 						// Goto Standby. MCU will now request to be powered down on DeepSleep
	powerDisableCache();
	powerDisableCacheRetention();

	//Calculate next recharge (Refreshtime): must be BEFORE POWER DOWN
	SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);
	SysCtrlAonSync(); 							// Synchronize transactions to AON domain to ensure AUX has turned off

	// Enter Standby
	powerDisableCPU();
	PRCMDeepSleep();
	SysCtrlAonUpdate();
	SysCtrlAdjustRechargeAfterPowerDown();   	// AFTER POWER DOWN: Set refresh cycle
	SysCtrlAonSync();



}


//===============================================================================

int main(void) {

  initSensortag();

  while(1) {

	getData();
	setData();
	sendData();
    sleep();
  }
}
