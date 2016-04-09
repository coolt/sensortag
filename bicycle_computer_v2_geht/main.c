/** V2: auf git-branch LED
 *  - Button und Reed (auf Gnd) gehen
 *  - Wegen LED GPIO Port nicht abstellen
 */

<<<<<<< HEAD
=======
// Debugging:
//
/*
 * cpu.h
 * -----
 *
 * - uint32_t CPUprimask(void); 		// Get the current interrupt state.
 * - void CPUwfi(void) 					// Wait for interrupt.
 * - .. 								// Wait for event
 * - .. 								// Send event
 * - void CPUdelay(uint32_t ui32Count); // Provide a small delay.
 *
 *
 * ioc.h
 * -----
 * Alle IO Definitione: Pin setzen, welcher Bereich, welche Konfiguration
 * - uint32_t IOCPortConfigureGet(--);
 * - uint32_t IOCPortConfigureGet(..);
 * - void IOCIOShutdownSet(uint32_t ui32IOId, uint32_t ui32IOShutdown); //!! set wake up for this pin
 * - void IOCPinTypeGpioInput(uint32_t ui32IOId); // Aufsetzen eines GPIO-Interrupts
 * ******************************
 * event.h:
 * -------
 * Event in Fabrik eintragen
 *
 *
 * aon_event.h:
 * void AONEventMcuWakeUpSet(uint32_t ui32MCUWUEvent, uint32_t ui32EventSrc);
 *
 *
 * wake up event controller: aon_wuc.h
 * -----------------------------------
 * - void AONWUCMcuWakeUpConfig(uint32_t ui32WakeUp); // direkt aufwachen oder nach delay
 * - uint32_t AONWUCPowerStatusGet(void);  // Zeile 512
 * - AONWUCPowerDownEnable(void);
 * - Recharge Funktionen und Berechnung
 * -
 *
 * ************************************
 * interrupt.h
 *
 * Get the priority of the interrupt
 * -----------------------------------
 * int32_t IntPriorityGet(uint32_t ui32Interrupt);
 *
 * Enable an interrupt
 * -------------------
 * void IntEnable(uint32_t ui32Interrupt);
 *
 * ! Disables an interrupt
 * -----------------------
 * void IntDisable(uint32_t ui32Interrupt);
 *
 * ! Query (Abfragen) whether an interrupt is pending
 * -----------------------------------------
 * bool IntPendGet(uint32_t ui32Interrupt);
 *
 * ! Unpends an interrupt
 * void IntPendClear(uint32_t ui32Interrupt);
 *
 */

>>>>>>> parent of d90f5d5... ongoing
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
<<<<<<< HEAD
	powerEnableAuxForceOn(); 		// set power to WakeUpEvent (WU)
	powerEnableXtalInterface();  	// set clk WUC
	powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32); // Divide clk to save power
	powerEnablePeriph();    		// for GPIO-Settings
=======
	powerEnableAuxForceOn(); // set power to WakeUpEvent
	powerEnableRFC();
	powerEnableXtalInterface();

	// reduce clk
	powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32); // Divide INF clk to save Idle mode power (increases interrupt latency)

	// 2 Version, now: Darios (without calculation)
	initRTC(); // init Interrupt AON RTC, set variables

	// power on
	powerEnablePeriph();
>>>>>>> parent of d90f5d5... ongoing
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

<<<<<<< HEAD
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
=======
	// prepate data buffer
	// --------------------
	// Clear payload buffer
    memset(payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
	//Fill payload buffer with adv parameter data
	uint8_t p;
	p = 0;
	payload[p++] = 0x01;          /* 2 bytes */
	payload[p++] = 0x01;
	payload[p++] = 0x03;          /* LE Limited Discoverable Mode" & "BR/EDR Not Supported */
	payload[p++] = 0x04; //1 + strlen(beacond_config.adv_name);
	payload[p++] = 0x03;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x00;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0xDE;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x01;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x35;//BLE_ADV_TYPE_NAME;


	//Start radio setup and linked advertisment
	radioUpdateAdvData(10, payload); //Update advertising byte based on IO inputs


	// +++++++++++++++++++++++++++++++++++++++
	// Interrupt driven device
	// Interrupt shown on LED 1, 2
	while(1) {  // endlose loop: system is in standby mode, waiting for interrupt on GPIO


		// Info Debugging:
		// goes in while loop
		// last commit: RF hase not stopped
		// current commit: RF does not start anymore: Problem is PRCM_O_PDSTAT0RFC is not set
		// -> see function waitUntilRFReady()

		// Who set this bit ? System is in sleep ->
		// Possibility: RTC does not wake up the Radio   =>>    powerEnableRFC();
		// or
		// RTC wakes up, but don't set this bit

		rfBootDone  = 0;
		rfSetupDone = 0;
		rfAdvertisingDone = 0;

		//Waiting for interrupt (RTC Timer)
		//-> starts RF core

		// ----------------------------------------------------
		// Prepation to send data
		// -----------------------------------------------------
		waitUntilRFCReady(); // is not set (in current version)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		initRadioInts();  // enable 3 RF Interrupts
		runRadio(); // =? set interrupt (because while waits for BootDone: interrupt 9)

		waitUntilAUXReady(); // Wait until AUX is powered
		OSCHF_TurnOnXosc();  // Enable 24MHz XTAL (higher clk for sending)
		while( ! rfBootDone) { //IDLE until BOOT_DONE interrupt from RFCore is triggered
			powerDisableCPU();
			//Request radio to keep on system
			// busPRCMDeepSleep();
		} //This code runs after BOOT_DONE interrupt has woken up the CPU again

		radioCmdBusRequest(true); //Request radio to keep on system bus
		radioPatch(); //Patch CM0 - no RFE patch needed for TX only
		radioCmdStartRAT(); //Start radio timer
		powerEnableFlashInIdle(); //Enable Flash access while doing radio setup
		while( !OSCHF_AttemptToSwitchToXosc()) //Switch to XTAL
		{}

		//Start radio setup and linked advertisment
		// ---------------------------------------------------
		// SENDING new DATA
		// ---------------------------------------------------
		radioSetupAndTransmit();
		while( ! rfSetupDone) {
			powerDisableCPU();
			PRCMDeepSleep();
		} //Wait in IDLE for CMD_DONE interrupt after radio setup. ISR will disable radio interrupts
		powerDisableFlashInIdle(); //Disable flash in IDLE after CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
		while( ! rfAdvertisingDone) {
		  powerDisableCPU();
		  PRCMDeepSleep();
		} //Wait in IDLE for LAST_CMD_DONE after 3 adv packets
		radioCmdBusRequest(false); //Request radio to not force on system bus any more


		// Standby procedure
		// ----------------------------------------------------
		powerDisableXtal();
		powerDisableRFC(); // Turn off radio.  Checked: Flag changes from 1 to 0.
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
		}
	    // standby endlose loop: waiting for interrupt
>>>>>>> parent of d90f5d5... ongoing

		setData();
		sendData();
		goToSleep();
	}
}
