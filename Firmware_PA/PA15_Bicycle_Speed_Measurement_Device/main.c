/**
    main.c - PA ---
  @brief main entry of simpleBroadcaster. This Code is for using with the
  	  	  EM8500 Harvestin Chip for a Bicycle Speed Measurement Device.
  	  	  If enough Energy is Harvested from the Bicycle, the EM8500 Chip
  	  	  will enable Supply Voltage for TI-SensorTag. It Configures Pin
  	  	  25 for an Interrupt Event, to detect the Speed of the Wheel with
  	  	  a magnet. The Time will be transmittet over BLE.
  	  	  The Supply from EM8500 will then be disabled with SPI and TI-Sensor
  	  	  Tag will shut down.

  @usage
        - Make a copy of ccfg.c from your CC26XXWARE version and
          configure it to use internal LF RCOSC
        - Configure WAKE_INTERVAL
        - Configure recharge period to 400 ms if WAKE_INTERVAL is larger than 400ms(ish)
        - Configure IO's and set up advertisment payload
        - Configure output power to desired value in CMD_RADIO_SETUP (see pa_table_cc26xx.c)
  
*/


#include <driverLib/ioc.h>
#include <driverLib/gpio.h>
#include <driverLib/sys_ctrl.h>
#include <driverLib/aon_rtc.h>
#include <driverlib/interrupt.h>
#include <inc/hw_aon_event.h>

#include "sensor-common.h"
#include "ext-flash.h"
#include "bmp-280-sensor.h"
#include "tmp-007-sensor.h"
#include "hdc-1000-sensor.h"
#include "opt-3001-sensor.h"
#include "em8500-pwr-mgnt.h"

#include "board.h"

#include "config.h"
#include "interfaces/board-i2c.h"
#include "rtc.h"
#include "radio.h"
#include "system.h"

extern volatile bool int_boot_done;

uint32_t time1, time2, timeDiff;
bool meas_done, first, int_enable;
uint32_t test;

void sensorsInit(void)
{

	//Turn off TMP007
    configure_tmp_007(0);

    // Init CS Pin for EM8500 to not select the device
    em8500_Init();

	//Power down Gyro
	IOCPinTypeGpioOutput(BOARD_IOID_MPU_POWER);
	GPIOPinClear(1 << BOARD_IOID_MPU_POWER);

	//Power down Mic
	IOCPinTypeGpioOutput(BOARD_IOID_MIC_POWER);
	GPIOPinClear(1 << BOARD_IOID_MIC_POWER);

	//Turn off external flash
	ext_flash_init(); //includes power down instruction

	//Turn off OPT3001
	configure_opt_3001(0);

	configure_bmp_280(0);
}

void ledInit(void)
{
	// Config of LED1 Port (DIO_10)
	IOCPortConfigureSet(IOID_10, IOC_PORT_GPIO, IOC_STD_OUTPUT);
	GPIODirModeSet(LED_RED, GPIO_DIR_MODE_OUT);
	GPIOPinWrite(LED_RED, 0);
	// Config of LED2 Port (DIO_15)
	IOCPortConfigureSet(IOID_15, IOC_PORT_GPIO, IOC_STD_OUTPUT);
	GPIODirModeSet(LED_GREEN, GPIO_DIR_MODE_OUT);
	GPIOPinWrite(LED_GREEN, 0);
}

void extPinEnable(uint8_t enable){
	// Power on IOC domain
	powerEnablePeriph();
	powerEnableGPIOClockRunMode();

	/* Wait for domains to power on */
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

void GPIOIntHandler(void){
	// Enable GPIO Periphrie
	IntPendClear(INT_EDGE_DETECT);
	// Disable Interrupt because of bouncing of the Pin
	IntDisable(INT_EDGE_DETECT);

	// Enable Power for the IOC to clear the interrupt flag
   powerEnablePeriph();
   powerEnableGPIOClockRunMode();

	/* Wait for domains to power on */
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
		 != PRCM_DOMAIN_POWER_ON));

	// wait because Edge is bouncing for approx. 3 ms
	// 24 MHz > 1 Cycle = 42 ns
	// CPUdelay waits for 3 Cycles > 126 ns
	// 100 us / 126 ns = 740
	// to be safe take a big value
	CPUdelay(10000);

	// Interrupt of PAD25 - clear it
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

	// Disable GPIO Periphrie
	powerDisablePeriph();
	// Disable clock for GPIO in CPU run mode
	HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	// Load clock settings
	HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;

	// Enable Interrupt again
	IntEnable(INT_EDGE_DETECT);
}

/** Call this function in Batterymode with TI-SensorTag to Debug your Code
 *  it will enable the LED and goes into a while loop to stop here.
 */
void testLED(void){
	// Enable Power for the IOC to clear the interrupt flag
   powerEnablePeriph();
   powerEnableGPIOClockRunMode();

	/* Wait for domains to power on */
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
		 != PRCM_DOMAIN_POWER_ON));

	GPIOPinWrite(LED_GREEN, 1);
	while(1);
}

// MAIN ------------------------------------------------------------------------------------------------

int main(void) {

	int p = 0;
	uint8_t payload[BLE_ADV_PAYLOAD_BUF_LEN];
	time1 = 0, time2 = 0, timeDiff = 0;
	float time_ms = 0;
	float time_float = 0;
	meas_done = false;

	//IAR I-jet SWO output on RF2.16
	//IOCPortConfigureSet(IOID_28, IOC_PORT_MCU_SWV, IOC_STD_OUTPUT);

	//Disable JTAG to allow for Standby
	AONWUCJtagPowerOff();

	//Force AUX on
	powerEnableAuxForceOn();
	powerEnableRFC();
	powerEnableXtalInterface();

	//******************************************VERBESSERUNG FOLGT*****************************
	//powerConfigureRecharge();

	// AdaptRate = 96
	// MaxPeriod = 21440
	// InitPeriod = 14816
	AONWUCRechargeCtrlConfigSet(true, 34, 2500, 5000);
	//AONWUCRechargeCtrlConfigSet(true, 34, 13107, 15000);
	//*****************************************************************************************

	powerEnablePeriph();
	powerEnableGPIOClockRunMode();

	/* Wait for domains to power on */
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
			!= PRCM_DOMAIN_POWER_ON));

	// Divide INF clk to save Idle mode power (increases interrupt latency)
	powerDivideInfClkDS(PRCM_INFRCLKDIVDS_RATIO_DIV32);

	sensorsInit();
	ledInit();

	powerDisablePeriph();
	// Disable clock for GPIO in CPU run mode
	HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0;
	// Load clock settings
	HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;

	initInterrupts();
	initRadio();
	powerDisableRFC();

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
	payload[p++] = 0x01;          /* 2 bytes */
	payload[p++] = 0x01;
	payload[p++] = 0x01;          /* LE Limited Discoverable Mode" & "BR/EDR Not Supported */
	payload[p++] = 0x01; //1 + strlen(beacond_config.adv_name);
	payload[p++] = 0x01;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x01;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0xD1;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x01;//BLE_ADV_TYPE_NAME;
	payload[p++] = 0x31;//BLE_ADV_TYPE_NAME;

	//MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
	powerDisableMcuPdReq();
	// Enable RTC as free running Counter
	AONRTCEnable();

	while(1){
		// enable and configure PIN Interrupt on PIN25
		extPinEnable(true);

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// IDLE procedure
		////////////////////////////////////////////////////////////////////////////////////////////////////
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

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Wake up from 1. Extern Interrupt
		////////////////////////////////////////////////////////////////////////////////////////////////////
		// Enter IDLE
		powerDisableCPU();
		PRCMDeepSleep();

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Wake up from 2. Extern Interrupt
		////////////////////////////////////////////////////////////////////////////////////////////////////
		// Wakeup up from Idle
		powerEnableAuxForceOn();

		//Re-enable cache and retention
		powerEnableCache();
		powerEnableCacheRetention();

		//Wait until AUX is ready before configuring oscillators
		waitUntilAUXReady();

		//MCU will not request to be powered down on DeepSleep -> System goes only to IDLE
		powerDisableMcuPdReq();

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

		/////////////////////////////////////////////////////////////////////////////////////////////////////
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
		while( !OSCHF_AttemptToSwitchToXosc()){}

		//change user specific data
		p=9;
		payload[p++] =  (uint8_t)(timeDiff & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 8) & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 16) & 0x000000FF);
		payload[p++] =  (uint8_t)((timeDiff >> 24) & 0x000000FF);

		//Start radio setup and linked advertisment
		radioUpdateAdvData(p,payload);
		radioSetupAndTransmit(); //0dBm (brts)

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

		powerDisableXtal();

		// Turn off radio
		powerDisableRFC();
		// Switch to RCOSC_HF
		OSCHfSourceSwitch();

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Turn off VSUP This will shut off the TI-SensorTag
		////////////////////////////////////////////////////////////////////////////////////////////////////
		// Turn off VSUP
		em8500_DisableVSUP();
		// Stay here to be sure Sensortag has been shut off (If VSUP will not be disabled, Energy will be
		// used until shut down
		while(1);
	}
}




