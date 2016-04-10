/** V2
 */

#include "cc26xxware_2_22_00_16101/driverLib/ioc.h"  		// Grundeinstellungen aktivieren domains
#include "cc26xxware_2_22_00_16101/driverLib/sys_ctrl.h"  	// Bus, CPU, Refresh

#include "sensors/sensor-common.h"
#include "sensors/ext-flash.h"
#include "sensors/bmp-280-sensor.h"
#include "sensors/tmp-007-sensor.h"
#include "sensors/hdc-1000-sensor.h"
#include "sensors/opt-3001-sensor.h"

#include "board.h" 											// Konstanten IO
#include "radio.h"

#include "config.h" 										// Konstanten Applikation
#include "cc26xxware_2_22_00_16101/driverLib/gpio.h" 		// Konstanten GPIO Pins
#include "interfaces/board-i2c.h"
#include "rtc.h"
#include "radio.h"
#include "system.h" 										// Funktionen (Power), Init, Waits
#include "cc26xxware_2_22_00_16101/inc/hw_aon_event.h"

extern volatile bool rfBootDone;
extern volatile bool rfSetupDone;
extern volatile bool rfAdvertisingDone;

// globale variable
uint8_t payload[ADVLEN]; 									// data buffer


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

	// Interrupts
	//initRTC(); // for time-calculation,  !! PA Code: automtisches Aufwachen nach 10 s, dann berechnen
	initGPIOInterrupts();									// Define IOPorts for Interrupt, Add GPIO-mask to WU-Event
	IntEnable(INT_EDGE_DETECT);								// Enable specific interrupt. Int_EDGE_DETECT = Nr. 16  (=> all GPIO-interrupts   ?? )

/*	// power off  -> moved after initRFInterrupts and Int enable
	powerDisablePeriph(); //Disable clock for GPIO in CPU run mode
	HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1; // Load clock settings
*/
	initRFInterrupts(); 									// Set RFInterrupts to NVIC
	CPUcpsie();												// All extern interrupts enable (globaly)

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

	int i = 8;

}
void setData(void){

	//memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN); // Clear payload buffer  //DOES NOT WORK !!!!!!!!!!

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

	// Flags for RF-communication supervision
	rfBootDone  = 0;
	rfSetupDone = 0;
	rfAdvertisingDone = 0;

	powerEnableRFC(); 							// Set power bit
	waitUntilRFCReady();
			initRadioInts();  					// Define which interrupts are detected (int vector table)
	runRadio();

	waitUntilAUXReady(); 						// AUX is needed to configure higher oscillator
	OSCHF_TurnOnXosc();  						// Enable 24 MHz XTAL (higher clk for sending)
	while( ! rfBootDone) {
		powerDisableCPU();
		// Request radio to keep on system
		//busPRCMDeepSleep();
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
	// AdvertisingDone = 3 packets are send
	while( ! rfAdvertisingDone) {
	  powerDisableCPU();
	  PRCMDeepSleep();
	}
	radioCmdBusRequest(false);					// Request radio to not force on system bus any more
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
}

int main(void) {

	initSensortag();

	while(1) {
		// interrupt driven:
		// ------------------
		// Fix-timed RTC-interrupt for wake up from sleep()
		// Read GPIO, set data, send data
		getData();
		setData();
		sendData();
		sleep();
	}
}
