/** V2: auf git-branch LED
 *  - Button und Reed (auf Gnd) gehen
 *  - Wegen LED GPIO Port nicht abstellen
 */

// Generel debugging informations
// ------------------------------------
/* -> Current debugging informations at the begins of the while-loop
 *
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
void initSensortag(){


}
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


int main(void) {

	uint8_t payload[ADVLEN]; // data buffer

	// power off
	AONWUCJtagPowerOff(); //Disable JTAG to allow for Standby, (needed for events, baek)

	// power on
	powerEnableAuxForceOn(); // set power to WakeUpEvent
	powerEnableRFC(); // set power Bit, on which the function waitRFCReady()
	powerEnableXtalInterface();
	int test_powerRFC = HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // = 1, o.k.
	// reduce clk
	powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32); // Divide INF clk to save Idle mode power (increases interrupt latency)

	// 2 Version, now: Darios (without calculation)
	initRTC(); // init Interrupt AON RTC, set variables

	// power on
	powerEnablePeriph();
	powerEnableGPIOClockRunMode();
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON)); /* Wait for domains to power on */

	sensorsInit();
	ledInit();

	// Set Interrupts
	// ---------------
	// Button = BOARD_IOID_KEY_RIGHT= IOID_4, external interrupt on rising edge and wake up
	IOCPortConfigureSet(BOARD_IOID_KEY_RIGHT, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_FALLING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_UP | IOC_INPUT_ENABLE | IOC_WAKE_ON_LOW);
	HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_PAD;  //Set device to wake MCU from standby on all pins
	// Does not work with AON_EVENT_MCUWUSEL_WU0_EV_PAD4, the specific pin for button

	// REED_SWITCH = IOID_25, external interrupt on rising edge and wake up
	IOCPortConfigureSet(REED_SWITCH, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_FALLING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_UP | IOC_INPUT_ENABLE | IOC_WAKE_ON_LOW);
	HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_PAD;  //Set device to wake MCU from standby from all pins


	// BAT_LOW = IOID_28, external interrupt on rising edge and wake up
	//IOCPortConfigureSet(BAT_LOW, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_FALLING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_UP | IOC_INPUT_ENABLE | IOC_WAKE_ON_LOW);
	//HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_PAD;  //Set device to wake MCU from standby all pins

	IntEnable(INT_EDGE_DETECT); // setzt Int auf NVIC
	// -----------------------------------------------------------

	// power off: No because of LED
	powerDisablePeriph(); //Disable clock for GPIO in CPU run mode
	HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1; // Load clock settings

	initInterrupts(); // RF-Interrputs, RTC -Interrupts, enable generaly
	initRadio();  // set BLE, 3 Adv. channels

	// power off and set Refresh on
	powerDisableFlashInIdle();  // Turn off FLASH in idle mode == stand by mode
	powerEnableCacheRetention(); // Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
	powerEnableAUXPdReq(); //AUX - request to power down (takes no effect since force on is set)
	powerDisableAuxRamRet();

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

	test_powerRTC = HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // = 1;  o.k.
	// +++++++++++++++++++++++++++++++++++++++
	// Interrupt driven device
	// Interrupt shown on LED 1, 2
	while(1) {  // endlose loop: system is in standby mode, waiting for interrupt on GPIO


		// Info Debugging:
		// ------------------
		// powerEnableRFC(); doesn't work
		// - RTC does not enable RFCPowerDomain
		// or
		// - RTC itself does not wake up

		rfBootDone  = 0;
		rfSetupDone = 0;
		rfAdvertisingDone = 0;

		//Waiting for interrupt (RTC Timer)
		//-> starts RF core
		test_powerRTC = HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // = 1, o.k.
		// ----------------------------------------------------
		// Prepation to send data
		// -----------------------------------------------------
		waitUntilRFCReady(); // is set from beginning, because of init (o.k.)
		test_powerRTC = HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN); // = 1, o.k.
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
		}
	    // standby endlose loop: waiting for interrupt

}
