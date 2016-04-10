/*
 * @file  main.c
	  @brief main entry of simpleBroadcaster, a bare-bones, speed optimized
	         program transmitting BLE advertisment packages every N ms based on input from 5
	         GPIOs

	  @usage
	        - Make a copy of ccfg.c from your CC26XXWARE version and
	          configure it to use internal LF RCOSC
	        - Configure WAKE_INTERVAL
	        - Configure recharge period to 400 ms if WAKE_INTERVAL is larger than 400ms(ish)
	        - Configure IO's and set up advertisment payload
	        - Configure output power to desired value in CMD_RADIO_SETUP (see pa_table_cc26xx.c)

	// Konzept PA:
	- Aufgrund von Timer Sensortag erhält das Read Relais (über das EM-Board VSUP) alle 10 s Strom.
	- pin 25 erhält Pulsdetektion (Read Relais). Erfasst dort 2 Messungen und kalkuliert daraus die Geschwindigkeit
	- Speisung Reed Relais wird wieder abgeschalten
 */

#include "em8500-pwr-mgnt.h"  // new
#include "board.h"
#include "radio.h"
#include "config.h"
#include "system.h"
#include "rtc.h"

#include "cc26xxware_2_22_00_16101/driverLib/ioc.h"
#include "cc26xxware_2_22_00_16101/driverLib/sys_ctrl.h"
#include "cc26xxware_2_22_00_16101/driverLib/gpio.h"
#include "cc26xxware_2_22_00_16101/driverLib/aon_rtc.h"    // new
#include "cc26xxware_2_22_00_16101/driverLib/interrupt.h"  // new

#include "cc26xxware_2_22_00_16101/inc/hw_aon_event.h"

#include "interfaces/board-i2c.h"

#include "sensors/sensor-common.h"
#include "sensors/bmp-280-sensor.h"
#include "sensors/tmp-007-sensor.h"
#include "sensors/hdc-1000-sensor.h"
#include "sensors/opt-3001-sensor.h"
#include "sensors/ext-flash.h"

// RF Chip
extern volatile bool rfBootDone;
extern volatile bool rfSetupDone;
extern volatile bool rfAdvertisingDone;


// for velocity calculation
uint32_t time1, time2, timeDiff;
bool meas_done, first, int_enable;
uint32_t test;


// Start: Function in PA ----------------------------------------------
// impuls goes from BOARD_IOID_DP0 on pin 25
 void extPinEnable(uint8_t enable){  // Todo: check type: not boolean ?
	// Power on IOC domain
	powerEnablePeriph();
	powerEnableGPIOClockRunMode();

	// Wait for domains to power on
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
			!= PRCM_DOMAIN_POWER_ON));

	if(enable){
		// Enable PIN25 for Interrupts
		IOCPortConfigureSet(BOARD_IOID_DP0, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_RISING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_DOWN  | IOC_INPUT_ENABLE | IOC_WAKE_ON_HIGH);
		//Set device to wake MCU from standby on PIN 25
		HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU1_EV_PAD25;
		// Enable and clear the Interrupt
		IOCIntClear(IOID_25);
		IntPendClear(INT_EDGE_DETECT);
		IntEnable(INT_EDGE_DETECT);
	}
	else{
		// Disable PIN25 for Interrupts
		IOCPortConfigureSet(BOARD_IOID_DP0, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_RISING_EDGE | IOC_INT_DISABLE | IOC_IOPULL_DOWN  | IOC_INPUT_ENABLE | IOC_WAKE_ON_HIGH);
		//Set device to wake MCU from standby on PIN 25
		HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU1_EV_NONE;
		// disable Interrupt
		IntDisable(INT_EDGE_DETECT);
	}

	// Power off IOC domain
	powerDisablePeriph();
	// Disable clock for GPIO in CPU run mode
	HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	// Load clock settings
	HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
}
// ----------------------------------------------------End Function only in PA





// interrupts -----------------------------------------------------------
void GPIOIntHandler(void){

		uint32_t pin_mask;  // only Dario

		// Zusätzlcher Code von PA: ---------------------------- Start
		// Enable GPIO Peripherie
		IntPendClear(INT_EDGE_DETECT);
		// Disable Interrupt because of bouncing of the Pin
		IntDisable(INT_EDGE_DETECT);
		// ------------------------------------------------------Ende

		// Enable Power for the IOC to clear the interrupt flag
		powerEnablePeriph();
		powerEnableGPIOClockRunMode();

		/* Wait for domains to power on */
		while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

		//  Zusätzlicher Code von PA: -----------------------Start
		// wait because Edge is bouncing for approx. 3 ms
		// 24 MHz > 1 Cycle = 42 ns
		// CPUdelay waits for 3 Cycles > 126 ns
		// 100 us / 126 ns = 740
		// to be safe take a big value
		CPUdelay(10000);

		// Interrupt of PAD25 - clear it
		// baek: Pin 25 erhält die Relais-Impulse per Interrupt
		if(IOCIntStatus(IOID_25)){
			IOCIntClear(IOID_25);

			if(time1 == 0){
				// Get the first Interrupt Time
				time1 = AONRTCCurrentSubSecValueGet();
			}
			else{
				// Get the second Interrupt Time
				time2 = AONRTCCurrentSubSecValueGet();
				meas_done = true;
			}
		}
		// ------------------------------------------------------ Ende


		// Code ausschliesslich Dario --------------------------Start
		/* Read interrupt flags */
		pin_mask = (HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) & GPIO_PIN_MASK);
		// zum Vergleich (aus PA) // Clear RTC event flag: Code from PA
	    // HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH2_BITN) = 1;

		/* Clear the interrupt flags */
		HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = pin_mask;
		// ----------------------------------------------Ende Code nur Dario



		powerDisablePeriph();
		// Disable clock for GPIO in CPU run mode
		HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
		// Load clock settings
		HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;

		/* Code nur PA ---------------------------------------------- Start
		 // Enable Interrupt again
		IntEnable(INT_EDGE_DETECT);
		 -------------------------------------------------------------- End
		 */

		// Code nur Dario -------------------------------------------Start
		//To avoid second interupt with register = 0 (its not fast enough!!)
		__asm(" nop");
		__asm(" nop");
		__asm(" nop");
		__asm(" nop");
		__asm(" nop");
		__asm(" nop");
		// ------------------------------------------------------------End
	}

// Funktion nur PA ----------------------------------------------------Start
/** Call this function in Batterymode with TI-SensorTag to Debug your Code
 *  it will enable the LED and goes into a while loop to stop here.
 */
/*
void testLED(void){
	// Enable Power for the IOC to clear the interrupt flag
   powerEnablePeriph();
   powerEnableGPIOClockRunMode();

	// Wait for domains to power on
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
		 != PRCM_DOMAIN_POWER_ON));

	GPIOPinWrite(LED_GREEN, 1);
	while(1);
} */
//---------------------------------------------------------------------End



void sensorsInit(void)
	{

		//Turn off TMP007
	    configure_tmp_007(0);

	    // Init CS Pin for EM8500 to not select the device
	    em8500_Init();										// new from PA

		//Power down Gyro
		IOCPinTypeGpioOutput(BOARD_IOID_MPU_POWER);
		GPIOPinClear(BOARD_MPU_POWER);

		//Power down Mic
		IOCPinTypeGpioOutput(BOARD_IOID_MIC_POWER);
		GPIOPinClear(1 << BOARD_IOID_MIC_POWER);

		//Turn off external flash
		ext_flash_init(); //includes power down instruction

		//Turn off OPT3001
		configure_opt_3001(0);

		configure_bmp_280(0);


		// This code is not in PA ---------------------------------------------
		//Power off Serial domain (Powered on in sensor configurations!)
		PRCMPowerDomainOff(PRCM_DOMAIN_SERIAL);
		while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_OFF));
		//Shut down I2C
		board_i2c_shutdown();
		// End: This code is not in PA ----------------------------------------
	}

void ledInit(void)
	{
		// Start Code Dario: (No LED. Energy)
		IOCPinTypeGpioOutput(BOARD_IOID_LED_1); //LED1
		IOCPinTypeGpioOutput(BOARD_IOID_LED_2); //LED2

		GPIOPinClear(BOARD_LED_1);
		GPIOPinClear(BOARD_LED_2);
		// End Code Dario.


		/* Start Code PA: LED used
			// Config of LED1 Port (DIO_10)
			IOCPortConfigureSet(IOID_10, IOC_PORT_GPIO, IOC_STD_OUTPUT);
			GPIODirModeSet(LED_RED, GPIO_DIR_MODE_OUT);
			GPIOPinWrite(LED_RED, 0);
		 	// Config of LED2 Port (DIO_15)
			IOCPortConfigureSet(IOID_15, IOC_PORT_GPIO, IOC_STD_OUTPUT);
			GPIODirModeSet(LED_GREEN, GPIO_DIR_MODE_OUT);
			GPIOPinWrite(LED_GREEN, 0);
		End Code PA */
	}





int main(void) {



	// init
	  	  int i = 0; // new pa
	  uint8_t payload[ADVLEN];
	  	  time1 = 0, time2 = 0, timeDiff = 0; // new pa
	  	  float time_ms = 0; // new pa
	  	  float time_float = 0; // new pa
	  	  meas_done = false; // new pa
	  //Disable JTAG to allow for Standby
	  AONWUCJtagPowerOff();
	  //Force AUX on
	  powerEnableAuxForceOn();
	  powerEnableRFC();
	  powerEnableXtalInterface();

	  // To DO: Verbesserung powerConfiguration
	  // Dario:
	  //powerConfigureRecharge(); --> Optimized version later in this code (brts)
	  // PA:
	  // AdaptRate = 96
	  // MaxPeriod = 21440
	  // InitPeriod = 14816
	  AONWUCRechargeCtrlConfigSet(true, 34, 2500, 5000);   // neu aus PA !!! kommentiert
	  //AONWUCRechargeCtrlConfigSet(true, 34, 13107, 15000);
	  //********************************************************

	  // Divide INF clk to save Idle mode power (increases interrupt latency)
	  powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32);  // Dario hier. PA in nachfolgendem while als erstes

	  //initRTC();
	  powerEnablePeriph();
	  powerEnableGPIOClockRunMode();

	  // baek notes:-------------------------------------------------start
	  /* system gets energy
		-> configuration low power mode
		-> wake up MCU
		-> wake up RF CPU        ----------------------------------------- end*/

	  /* Wait for domains to power on: System start */
	  while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));
	  sensorsInit();
	  ledInit();

	  // nur bei Dario: nicht in PA ------------------------------------------begin
	  //Config IOID4 for external interrupt on rising edge and wake up
	  IOCPortConfigureSet(BOARD_IOID_KEY_RIGHT, IOC_PORT_GPIO, IOC_IOMODE_NORMAL | IOC_FALLING_EDGE | IOC_INT_ENABLE | IOC_IOPULL_UP | IOC_INPUT_ENABLE | IOC_WAKE_ON_LOW);
	  //Set device to wake MCU from standby on PIN 4 (BUTTON1)
	  HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_PAD;  //Does not work with AON_EVENT_MCUWUSEL_WU0_EV_PAD4 --> WHY??
	  IntEnable(INT_EDGE_DETECT);
	  // --------------------------------------------------------------------end

	  powerDisablePeriph();
	  //Disable clock for GPIO in CPU run mode
	  HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	  // Load clock settings
	  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
	  initInterrupts();
	  initRadio();

	  // nur bei PA:----------------------------------------------start
	  // powerDisableRFC();
	  // -------------------------------------------------------end

	  // Turn off FLASH in idle mode
	  powerDisableFlashInIdle();
	  // Cache retention must be enabled in Idle if flash domain is turned off (to avoid cache corruption)
	  powerEnableCacheRetention();
	  //AUX - request to power down (takes no effect since force on is set)
	  powerEnableAUXPdReq();
	  powerDisableAuxRamRet();

	  // nur bei PA:----------------------------------------------start
	  SysCtrlAonSync();
	  // -------------------------------------------------------end

	  // data buffer
	  //Clear payload buffer:
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

	  // Code only in PA: ---------------------------------------------start
	  //MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
	  // powerDisableMcuPdReq();  // auskommentiert von mir
	  // Enable RTC as free running Counter
	  AONRTCEnable();
	  //  ---------------------------------------------------------------------end

	  //Start radio setup (RF Core) and linked advertisment
	  radioUpdateAdvData(10, payload);  // Code only Dario

	  while(1) {

		// Code only Dario ----------------------------------------------start
	    rfBootDone  = 0;
	    rfSetupDone = 0;
	    rfAdvertisingDone = 0;
	    //Wait until RF Core PD is ready before accessing radio
	    waitUntilRFCReady();
	    initRadioInts();
	    runRadio();
	    // Code only Dario --------------------------------------------------end

	    // Code from PA (idle mode) -------------------------------------------start
	    // enable and configure PIN Interrupt on PIN25 for bicycle reed relais impuls
		extPinEnable(true);

		// IDLE procedure
		// Allow AUX to turn off again. No longer need oscillator interface
		powerDisableAuxForceOn();

		// Disable cache and retention
		powerDisableCache();
		powerDisableCacheRetention();

		// Synchronize transactions to AON domain to ensure AUX has turned off
		SysCtrlAonSync();

		// Enter IDLE
		powerDisableCPU();
		PRCMDeepSleep();

		// Wake up from 1. Extern Interrupt
		// Enter IDLE
		powerDisableCPU();
		PRCMDeepSleep();

		// Wake up from 2. Extern Interrupt
		// Wakeup up from Idle
		powerEnableAuxForceOn();

		//Re-enable cache and retention
		powerEnableCache();
		powerEnableCacheRetention();
	     // --------------------------------------------------------------------end

	    //Wait until AUX is ready before configuring oscillators
	    waitUntilAUXReady();
	    //Enable 24MHz XTAL
	    OSCHF_TurnOnXosc(); // only Dario



	    // Code only PA: Geschwindigkeitsberechnung-----------------------start
	    //MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
		powerDisableMcuPdReq(); // bei Dario letzte Ausführung in Schleife

		// Wake up here when second Interrupt on PIN25 occured
		extPinEnable(false);
		meas_done = false;

		// calculate Time of Wheel
		time1 = time1 >> 16;
		time2 = time2 >> 16;
		// difference of the 2 timestamps
		timeDiff = time2 - time1;
		// If an overflow of the counter occured - this doesnt matter
		timeDiff = timeDiff & 0xFFFF;
		// from s to ms
		time_float = timeDiff * 1000;
		// calculate the time to ms
		time_ms = time_float / 65535.0;
		timeDiff = (uint32_t)(time_ms);

		// Reset Timevalues for the right function of GPIO-Interrupt
		time1 = 0;
		time2 = 0;

		// Disable RTC
		AONRTCDisable();

		// Send Time over BLE
		////////////////////////////////////////////////////////////////////////////////////////////////////

		//Wait until RF Core PD is ready before accessing radio
		powerEnableRFC();
		waitUntilRFCReady();
		initRadioInts();
		runRadio();

		//Enable 24MHz XTAL
		powerEnableXtal();

		//IDLE until BOOT_DONE interrupt from RFCore is triggered

		CPUcpsid(); //Critical section, disable interrupts
		if(rfBootDone == 0) {
			powerDisableCPU();
			PRCMDeepSleep();
		}

		CPUcpsie(); //Enable interrupts, RFCCPE1IntHandler will execute
	     //------------------------------------------------------------------end

		/* Darios code auskommentiert: da oben dasselbe bereits gemacht wird
	    //IDLE until BOOT_DONE interrupt from RFCore is triggered
	    while( ! rfBootDone) {  // pa hat kleine Variante: siehe oben
	      powerDisableCPU();
	      PRCMDeepSleep();
	    } */

	    // Preparation sending Data
	    //This code runs after BOOT_DONE interrupt has woken up the CPU again
	    //Request radio to keep on system busx
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

	    //change user specific data (Code aus PA)-----------start
		p=9;
		payload[p++] =  (uint8_t)(timeDiff & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 8) & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 16) & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 24) & 0x000000FF);
		//Start radio setup and linked advertisment
		radioUpdateAdvData(p,payload);
	    // Code aus PA------------------------------------------end

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
	      // Code PA: ---------------------------------------------------start
	      //Ensure transactions to AON domain are done (clearing RTC interrupt out from standby)
	      SysCtrlAonSync(); //  -----------------------------------------end
	      PRCMDeepSleep();
	    }

	    /* Code nur PA:
	     /Disable flash in IDLE after CMD_RADIO_SETUP is done (radio setup reads FCFG trim values)
		powerDisableFlashInIdle();

		//Wait for LAST_CMD_DONE after 3 adv packets
		powerDisableCPU();
	     */

	    // Code nur Dario:
	    //Request radio to not force on system bus any more
	    radioCmdBusRequest(false);  // nur Dario


	    // Standby procedure
	    powerDisableXtal();
	    // Turn off radio
	    powerDisableRFC();
	    // Switch to RCOSC_HF
	    OSCHfSourceSwitch();

	    // Code nur Dario: STAND BY: Nicht idle ----------------------------start
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

	    // Enter Standby
	    powerDisableCPU();
	    PRCMDeepSleep();
	    SysCtrlAonUpdate();
	    SysCtrlAdjustRechargeAfterPowerDown();
	    SysCtrlAonSync();

		// Wakeup from RTC every 100ms, code starts execution from here
	    powerEnableRFC();
	    powerEnableAuxForceOn();

	    //Re-enable cache and retention
	    powerEnableCache();
	    powerEnableCacheRetention();

	    //MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
	    powerDisableMcuPdReq();
	    // End code nur Dario ------------------------------------------------------end

	    /* Code only PA ------------------------------------------------------------start
		// Turn off VSUP This will shut off the TI-SensorTag
		////////////////////////////////////////////////////////////////////////////////////////////////////
		// Turn off VSUP
		em8500_DisableVSUP();
		// Stay here to be sure Sensortag has been shut off (If VSUP will not be disabled, Energy will be
		// used until shut down
		while(1);
		---------------------------------------------------------------------------------end */

	  }
}
