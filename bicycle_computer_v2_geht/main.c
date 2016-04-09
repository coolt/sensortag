/** V2: auf git-branch LED
 *  - Button und Reed (auf Gnd) gehen
 *  - Wegen LED GPIO Port nicht abstellen
 */

#include "cc26xxware_2_22_00_16101/driverLib/ioc.h"  // Alle Grundeinstellungen (was wie aktiv ist)
#include "cc26xxware_2_22_00_16101/driverLib/sys_ctrl.h"  // Bus, CPU, Refresh

#include "sensors/sensor-common.h"
#include "sensors/ext-flash.h"
#include "sensors/bmp-280-sensor.h"
#include "sensors/tmp-007-sensor.h"
#include "sensors/hdc-1000-sensor.h"
#include "sensors/opt-3001-sensor.h"

#include "board.h" // Konstanten IO
#include "radio.h"

#include "config.h" // Konstanten V1
#include "cc26xxware_2_22_00_16101/driverLib/gpio.h" // Konstanten GPIO Pins
#include "interfaces/board-i2c.h"
#include "rtc.h"
#include "radio.h"
#include "system.h" // Funktionen (Power), Init, Waits
#include "cc26xxware_2_22_00_16101/inc/hw_aon_event.h"

extern volatile bool rfBootDone;
extern volatile bool rfSetupDone;
extern volatile bool rfAdvertisingDone;

void initSensortag(){

	// power off
	AONWUCJtagPowerOff(); //Disable JTAG to allow for Standby, (needed for events, baek)

	// power on
	powerEnableAuxForceOn(); 		// set power to WakeUpEvent (WU)
	powerEnableXtalInterface();  	// set clk WUC
	powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32); // Divide clk to save power
	powerEnablePeriph();    		// for GPIO-Settings
	powerEnableGPIOClockRunMode();
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON)); /* Wait for domains to power on */

	// Configure system
	sensorsInit();
	ledInit();
	initRadio();  					// BLE as communication on 3 Adv. channels
	initInterrupts(); 				// 3 RF-Interrputs, 2 RTC -Interrupts, 2 GPIO-Interrupts, Interrupt enable generaly
}

void setData(){

	// Clear payload buffer
	// memset(g_payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
}


// Info Debugging:
// ------------------
// powerEnableRFC(); doesn't work
// - RTC does not enable RFCPowerDomain
// or
// - RTC itself does not wake up

//Waiting for interrupt (RTC Timer)
//-> starts RF core

void sendData(){

	// Flags RF-Command-Process
	rfBootDone  = 0;
	rfSetupDone = 0;
	rfAdvertisingDone = 0;

	powerEnableRFC(); // set power Bit, on which the function waitRFCReady()

	// Load data
	radioUpdateAdvData(10, g_payload); // Start radio setup and linked advertisment: advertising byte based on IO inputs

	// Prepation to send data
	waitUntilRFCReady(); 				// is set from beginning, because of init (o.k.)
	runRadio(); 						// set power to all RF domains
	waitUntilAUXReady(); 				// Wait until Event Fabric is powered
	OSCHF_TurnOnXosc();  				// Enable 24MHz XTAL (higher clk for sending)
	while( ! rfBootDone) {
		powerDisableCPU();
		//Request radio to keep on system
		// busPRCMDeepSleep(); 			????????????????????
	}
	radioCmdBusRequest(true); 			//Request radio to keep on system bus
	radioPatch(); 						//Patch CM0 - no RFE patch needed for TX only
	radioCmdStartRAT(); //Start radio timer
					powerEnableFlashInIdle(); //Enable Flash access while doing radio setup
					while( !OSCHF_AttemptToSwitchToXosc()) //Switch to XTAL
					{}

	// SENDING new DATA
	// ---------------------------------------------------
	radioSetupAndTransmit();
	while( ! rfSetupDone) {
		powerDisableCPU();
		PRCMDeepSleep();
	} //Wait in IDLE for CMD_DONE interrupt after radio setup. ISR will disable radio interrupts !!!!!!!!!!!!!!!!!!!!!

	powerDisableFlashInIdle(); 			//Disable flash in IDLE after CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
	while( ! rfAdvertisingDone) {
	  powerDisableCPU();
	  PRCMDeepSleep();
	} //Wait in IDLE for LAST_CMD_DONE after 3 adv packets

	radioCmdBusRequest(false); 			//Request radio to not force on system bus any more
}

void goToSleep(){

	// power off and set Refresh on
	powerDisableFlashInIdle();  // Turn off FLASH in idle mode == stand by mode
	powerEnableCacheRetention(); // Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
	powerEnableAUXPdReq(); //AUX - request to power down (takes no effect since force on is set)
	powerDisableAuxRamRet();

	// Standby procedure
	// ----------------------------------------------------
	powerDisableXtal();
	powerDisableRFC(); // Turn off radio.  Checked: Flag changes from 1 to 0.
	int debugg = HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // = 0

	OSCHfSourceSwitch(); // Switch to RCOSC_HF
	powerDisableAuxForceOn();
	powerEnableMcuPdReq(); // Goto Standby. MCU will now request to be powered down on DeepSleep
	powerDisableCache(); // Disable cache and retention
	powerDisableCacheRetention();

	//Calculate next recharge (Refreshtime)
	SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);		// BEFORE POWER DOWN
	SysCtrlAonSync(); // Synchronize transactions to AON domain to ensure AUX has turned off

	// Enter Standby
	// --------------------------------------------------
	int testx =  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN);  // for debugging: RFChip = 0, so power down is still correct
	powerDisableCPU();
	PRCMDeepSleep();
	SysCtrlAonUpdate();
	SysCtrlAdjustRechargeAfterPowerDown();   // AFTER POWER DOWN: Set refresh cycle
	SysCtrlAonSync();

	testx =  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // for debugging: if RFChip = power down, then = 0

	/* old code:
	 *  -> is know in RTC Wake up interrupt
			// Wakeup from RTC every 100ms, code starts execution from here
			// ---------------------------------------------
			// WAITING FOR INTERRUPT
			// HERE: OLD CODE. FIX WAKE UP TIME
			powerEnableRFC();
			powerEnableAuxForceOn();
	*/

	//Re-enable cache and retention
	powerEnableCache();
	powerEnableCacheRetention();

	//MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
	powerDisableMcuPdReq();

	// standby endlose loop: waiting for interrupt
}

int main(void) {

	initSensortag();

	// 2 Version, now: Darios (without calculation)
	initRTC(); // CH 0: Wake up alle 10 s, Ch 2: nicht fertig aufgesetzt

	IntEnable(INT_EDGE_DETECT); // setzt Int auf NVIC

	// Interrupt driven device
	// -------------------------
	while(1) {

		setData();
		sendData();
		goToSleep();
	}
}
