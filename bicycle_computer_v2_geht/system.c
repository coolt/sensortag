#include "cc26xxware_2_22_00_16101/driverLib/aux_wuc.h"
#include "cc26xxware_2_22_00_16101/driverLib/cpu.h"
#include "cc26xxware_2_22_00_16101/driverLib/prcm.h"
#include "cc26xxware_2_22_00_16101/inc/hw_aon_wuc.h"
#include "cc26xxware_2_22_00_16101/inc/hw_ints.h"
#include "cc26xxware_2_22_00_16101/inc/hw_memmap.h"
#include "cc26xxware_2_22_00_16101/inc/hw_nvic.h"
#include "cc26xxware_2_22_00_16101/inc/hw_prcm.h"
#include "cc26xxware_2_22_00_16101/inc/hw_types.h"
#include "cc26xxware_2_22_00_16101/driverLib/osc.h"
#include "cc26xxware_2_22_00_16101/driverLib/vims.h"
#include "board.h"

// aus radio.c
#include "cc26xxware_2_22_00_16101/inc/hw_rfc_dbell.h"
#include "cc26xxware_2_22_00_16101/inc/hw_rfc_pwr.h"
#include "cc26xxware_2_22_00_16101/inc/hw_fcfg1.h"
#include "radio_files/rfc_api/common_cmd.h"
#include "radio_files/rfc_api/ble_cmd.h"
#include "radio_files/rfc_api/mailbox.h"
#include "radio_files/patches/ble/apply_patch.h"
#include "radio_files/overrides/ble_overrides.h"

// aus rtc.c
#include "config.h"
#include "cc26xxware_2_22_00_16101/inc/hw_aon_event.h"
#include "cc26xxware_2_22_00_16101/inc/hw_ints.h"
#include "cc26xxware_2_22_00_16101/inc/hw_nvic.h"
#include "cc26xxware_2_22_00_16101/driverLib/aon_rtc.h"
#include "cc26xxware_2_22_00_16101/driverLib/sys_ctrl.h"

// Datastructure BLE
// Advertisment data. Must be global for other files to access it.
#pragma data_alignment=4
char advData[ADVLEN] = {0};


#pragma data_alignment=8
static uint64_t devAddress;


#pragma data_alignment=4
rfCoreHal_bleAdvPar_t cmdAdvParam = {
  .advLen                     = ADVLEN,
  .pAdvData                   = (uint8_t*)advData,
  .pDeviceAddress             = (uint16_t*)&devAddress,
  .endTrigger.triggerType     = TRIG_NEVER,
};

#pragma data_alignment=4
rfCoreHal_bleAdvOutput_t advOutput = {0};

#pragma data_alignment=4
rfCoreHal_CMD_BLE_ADV_NC_t cmdAdv2 =  {
  .commandNo                  = CMD_BLE_ADV_NC,
  .pNextOp                    = NULL,
  .condition.rule             = COND_ALWAYS,
  .startTrigger.triggerType   = TRIG_NOW,
  .channel                    = 39,
  .pParams                    = (uint8_t*)&cmdAdvParam,
  .pOutput                    = (uint8_t*)&advOutput,
};

#pragma data_alignment=4
rfCoreHal_CMD_BLE_ADV_NC_t cmdAdv1 =  {
  .commandNo                  = CMD_BLE_ADV_NC,
  .pNextOp                    = NULL,
  .condition.rule             = COND_ALWAYS,
  .startTrigger.triggerType   = TRIG_NOW,
  .channel                    = 38,
  .pParams                    = (uint8_t*)&cmdAdvParam,
  .pOutput                    = (uint8_t*)&advOutput,
};


#pragma data_alignment=4
rfCoreHal_CMD_BLE_ADV_NC_t cmdAdv0 =  {
  .commandNo                  = CMD_BLE_ADV_NC,
  .pNextOp                    = NULL,
  .condition.rule             = COND_ALWAYS,
  .startTrigger.triggerType   = TRIG_NOW,
  .channel                    = 37,
  .pParams                    = (uint8_t*)&cmdAdvParam,
  .pOutput                    = (uint8_t*)&advOutput,
};

#pragma data_alignment=4
rfCoreHal_CMD_FS_POWERDOWN_t cmdFsPd = {
  .commandNo                = CMD_FS_POWERDOWN,
  .startTrigger.triggerType = TRIG_NOW,
  .condition.rule           = COND_NEVER,
};

//#pragma data_alignment=4
//rfCoreHal_CMD_RADIO_SETUP_t cmdSetup = {
//  .commandNo                = CMD_RADIO_SETUP,
//  .pNextOp                  = (uint8_t*)&cmdAdv0,
//  .startTrigger.triggerType = TRIG_NOW,
//  .condition.rule           = COND_ALWAYS,
//  .pRegOverride             = bleSingleOverrides,
//  .config.frontEndMode      = 0x1, // Differential
//  .config.biasMode          = 0x1, // Internal bias
//  .txPower.GC               = 0x1, // 0dbm
//  .txPower.IB               = 0x2C, // 0dbm
//  .txPower.tempCoeff        = 0x56,
//  .mode                     = 0, //BLE mode
//
//};

#pragma data_alignment=4
rfCoreHal_CMD_RADIO_SETUP_t cmdSetup = {
  .commandNo                = CMD_RADIO_SETUP,
  .pNextOp                  = (uint8_t*)&cmdAdv0,
  .startTrigger.triggerType = TRIG_NOW,
  .condition.rule           = COND_ALWAYS,
  .pRegOverride             = bleDifferentialOverrides,
  .config.frontEndMode      = 0x0, // Differential
  .config.biasMode          = 0x0, // Internal bias
  .txPower.GC               = 0x1, // 0dbm
  .txPower.IB               = 0x21, // 0dbm
  .txPower.tempCoeff        = 0x31,
  .mode                     = 0, //BLE mode
};

// ------------------------------------------------------------------------------------------------------------------------------

// Forward declarations:
void powerEnableGPIOClockRunMode();
void powerEnablePeriph(void);

// _______________________-
void initRadio(void) {
  // Set radio to BLE mode
  HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = 0x1;

   //Set up MAC address. Currently using TI Provided adress
   devAddress = *((uint64_t*)(FCFG1_BASE+FCFG1_O_MAC_BLE_0));

  //Chain advertisment commands.
  cmdAdv0.pNextOp = (uint8_t *)&cmdAdv1;
  cmdAdv1.pNextOp = (uint8_t *)&cmdAdv2;
  cmdAdv2.pNextOp = (uint8_t *)&cmdFsPd;
}

void initRadioInts(void) {

  // Enable interrupt for BOOT_DONE and LAST_CMD_DONE
  uint32_t intVecs = RFC_DBELL_RFCPEIEN_BOOT_DONE_M |
                     RFC_DBELL_RFCPEIEN_COMMAND_DONE_M |
                     RFC_DBELL_RFCPEIEN_LAST_COMMAND_DONE_M;

  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = intVecs;
}

//Send command pointer to doorbell
static inline void radioSendCommand(uint32_t cmd) {
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_CMDR ) = cmd;
}

//Timed wait for radio direct commands to complete
static inline void radioWaitCommandOk(void) {
  while( HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA) == CMDSTA_Pending);
}

//Request radio to keep on system bus
void radioCmdBusRequest(bool enabled) {
  // Send bus request command as direct command
  uint32_t cmd = CMDR_DIR_CMD_1BYTE(CMD_BUS_REQUEST, enabled);
  radioSendCommand(cmd);
  radioWaitCommandOk();
}

//Start radio timer
void radioCmdStartRAT(void) {
  uint32_t cmd = CMDR_DIR_CMD(CMD_START_RAT);
  radioSendCommand(cmd);
  radioWaitCommandOk();
}

void radioSetupAndTransmit() {
  radioSendCommand( (uint32_t)&cmdSetup);
}

//Update advertising byte based on IO inputs
void radioUpdateAdvData(int size, char* data) {
	int i;
	for(i = 0; i < size; i++)
	{
	  advData[i] = data[i];
	}
}


/** Call this function in Batterymode with TI-SensorTag to Debug your Code
 *  it will enable the LED and goes into a while loop to stop here.
 */
void setLED1(void){

	// Enable Power for the IOC to clear the interrupt flag
	powerEnablePeriph();
	powerEnableGPIOClockRunMode();


	/* Wait for domains to power on */
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
		 != PRCM_DOMAIN_POWER_ON));

	GPIOPinWrite(BOARD_IOID_LED_1,1);
	//while(1);
}

void setLED2(void){

	// Enable Power for the IOC to clear the interrupt flag
   powerEnablePeriph();
   powerEnableGPIOClockRunMode();

	/* Wait for domains to power on */
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH)
		 != PRCM_DOMAIN_POWER_ON));

	GPIOPinWrite(BOARD_IOID_LED_2,1);
	//while(1);
}

void ledInit(void)
{
	IOCPinTypeGpioOutput(BOARD_IOID_LED_1); //LED1
	IOCPinTypeGpioOutput(BOARD_IOID_LED_2); //LED2

	GPIOPinClear(BOARD_LED_1);
	GPIOPinClear(BOARD_LED_2);
}

void sensorsInit(void)
{
	//Turn off TMP007
    configure_tmp_007(0);

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

	//Power off Serial domain (Powered on in sensor configurations!)
	PRCMPowerDomainOff(PRCM_DOMAIN_SERIAL);
	while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_OFF));

	//Shut down I2C
	board_i2c_shutdown();
}


// *************************************INTERRUPT FUNCTIONS

// Enable interrupt on CPU
void initInterrupts(void) {  //baek: this are only RF interrupts and RTC

	// CPE1 - Int channels 31:16: Boot done is bit 30
	HWREG(NVIC_EN0) = 1 << (INT_RF_CPE1 - 16);
	// CPE0 - Int channels  15:0: CMD_DONE is bit 1, LAST_CMD_DONE is bit 0
	HWREG(NVIC_EN0) = 1 << (INT_RF_CPE0 - 16);

	// RTC combined event output
	HWREG(NVIC_EN0) = 1 << (INT_AON_RTC - 16);

	// Global interrupt enable
	CPUcpsie();
}

void initRTC(void) { // Function from Dario (not in PA

	//Add RTC Ch2 event as input to AON RTC interrupt
	AONRTCCombinedEventConfig(AON_RTC_CH2);

	//Set RTC ch 2 auto increment
	AONRTCIncValueCh2Set(WAKE_INTERVAL_TICKS);
	//Set RTC ch2 initial compare value
	AONRTCCompareValueSet(AON_RTC_CH2, WAKE_INTERVAL_TICKS);
	//Set RTC CH 2 to auto increment mode
	AONRTCModeCh2Set(AON_RTC_MODE_CH2_CONTINUOUS);

	//Enable channel 2
	AONRTCChannelEnable(AON_RTC_CH2);


	//Set device to wake MCU from standby on RTC channel 2
	HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_RTC_CH2;

	//Enable RTC
	AONRTCEnable();
}


void initRTC_WUms(uint32_t ms){  // new function only PA

	uint32_t compare_intervall = ms * 65536 / 1000;
	uint32_t current_compare_value = 0;
	uint32_t wake_compare_value = 0;

	current_compare_value = AONRTCCurrentCompareValueGet();
	wake_compare_value = current_compare_value + compare_intervall;

	//Add RTC Ch2 event as input to AON RTC interrupt
	AONRTCCombinedEventConfig(AON_RTC_CH2);
	//Set RTC ch2 initial compare value
	AONRTCCompareValueSet(AON_RTC_CH2, wake_compare_value);
	//Set RTC CH 2 to auto increment mode
	AONRTCModeCh2Set(AON_RTC_MODE_CH2_NORMALCOMPARE);
	//Enable channel 2
	AONRTCChannelEnable(AON_RTC_CH2);
	//Set device to wake MCU from standby on RTC channel 2
	HWREG(AON_EVENT_BASE + AON_EVENT_O_MCUWUSEL) = AON_EVENT_MCUWUSEL_WU0_EV_RTC_CH2;
}

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void powerEnableAuxForceOn(void) {
  HWREGBITW(AON_WUC_BASE + AON_WUC_O_AUXCTL,AON_WUC_AUXCTL_AUX_FORCE_ON_BITN)=1;
}

void powerDisableAuxForceOn(void) {
  HWREGBITW(AON_WUC_BASE + AON_WUC_O_AUXCTL,AON_WUC_AUXCTL_AUX_FORCE_ON_BITN)=0;
}

void powerEnableCache(void) {
  VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
}
void powerDisableCache(void) {
  VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_OFF, true);
}
void powerEnableCacheRetention(void) {
  // Enable cache SRAM retention
  HWREG(PRCM_BASE + PRCM_O_RAMRETEN) |= PRCM_RAMRETEN_VIMS_M;
}
void powerDisableCacheRetention(void) {
  // Disable cache SRAM retention
  HWREG(PRCM_BASE + PRCM_O_RAMRETEN) &= ~PRCM_RAMRETEN_VIMS_M;
}

void powerDisableAuxRamRet(void) {
  // Turn off AUX cache RAM retention
  HWREGBITW(AON_WUC_BASE + AON_WUC_O_AUXCFG,AON_WUC_AUXCFG_RAM_RET_EN_BITN) = 0;
}

void powerEnableRFC(void) {
  // Enable RF Core power domain
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN) = 1;
}
void powerDisableRFC(void) {
  // Enable RF Core power domain
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0RFC , PRCM_PDCTL0RFC_ON_BITN) = 0;
}

void powerEnablePeriph(void) {
  // Enable PERIPH for GPIO access
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0PERIPH , PRCM_PDCTL0PERIPH_ON_BITN) = 1;
  // Load clock settings
  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
}

void powerDisablePeriph(void) {
  // Disable PERIPH for GPIO access
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL0PERIPH , PRCM_PDCTL0PERIPH_ON_BITN) = 0;
  // Load clock settings
  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
}

void powerDisableCPU(void) {
  // Turn off CPU domain in CPU Deep Sleep
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL1CPU, PRCM_PDCTL1CPU_ON_BITN) = 0;
}

void powerEnableFlashInIdle(void) {
  // Enable flash when CPU is off
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL1VIMS, PRCM_PDCTL1VIMS_ON_BITN) = 1;
}
void powerDisableFlashInIdle(void) {
  // Disable flash when CPU is off
  HWREGBITW(PRCM_BASE + PRCM_O_PDCTL1VIMS, PRCM_PDCTL1VIMS_ON_BITN) = 0;
}

// Make AUX request powerdown mode and disable AUX RAM retention
// Will not take effect until the bit AON_WUC:AUXCTL:FORCE_ON is cleared
void powerEnableAUXPdReq(void) {
  // Make AUX request power down. Will only take effect when force on bit is not set
  HWREGBITW(AUX_WUC_BASE + AUX_WUC_O_PWRDWNREQ, AUX_WUC_PWRDWNREQ_REQ_BITN) = 1;
  HWREGBITW(AUX_WUC_BASE + AUX_WUC_O_MCUBUSCTL, AUX_WUC_MCUBUSCTL_DISCONNECT_REQ_BITN) = 1;
}

//Divide inf clock in Deep Sleep (for IOC, WUC, WDT etc). Will add latency to interrupt handling!
void powerDivideInfClkDS(uint32_t div) {
  HWREG(PRCM_BASE + PRCM_O_INFRCLKDIVDS) = div;
  // Load clock settings
  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
}

// Enable MCU to request to be powered down when CPU is turned off
void powerEnableMcuPdReq(void) {
  HWREGBITW(PRCM_BASE + PRCM_O_VDCTL,PRCM_VDCTL_ULDO_BITN) = 1;
}
// Disable MCU to request to be powered down when CPU is turned off
void powerDisableMcuPdReq(void) {
  HWREGBITW(PRCM_BASE + PRCM_O_VDCTL,PRCM_VDCTL_ULDO_BITN) = 0;
}


void powerEnableGPIOClockRunMode() {
  // Enable clock for GPIO in CPU run mode
  HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 1;
  // Load clock settings
  HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;
}


void powerConfigureRecharge(void) {
  
//  HWREG(AON_WUC_BASE + AON_WUC_O_RECHARGECFG) = 0x8095C79D;
  AONWUCRechargeCtrlConfigSet(true, 34, 13107, 15000);
}

void powerEnableXtal(void) {
  //Enable HF XTAL for HF clock
  HWREGBITW(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_CTL0, DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_BITN) = 1;
}

void powerDisableXtal(void) {
  //Disable HF XTAL for HF clock
  HWREGBITW(AUX_DDI0_OSC_BASE + DDI_0_OSC_O_CTL0, DDI_0_OSC_CTL0_SCLK_HF_SRC_SEL_BITN) = 0;
}

void powerEnableXtalInterface(void) {
  //Enable clock for OSC interface
  HWREG(AUX_WUC_BASE + AUX_WUC_O_MODCLKEN0) |= AUX_WUC_OSCCTRL_CLOCK;
}

void waitUntilRFCReady(void) {
  // Wait until RF Core is turned on
  while(HWREGBITW(PRCM_BASE + PRCM_O_PDSTAT0RFC , PRCM_PDSTAT0RFC_ON_BITN) != 1)
  {}
}
void waitUntilPeriphReady(void) {
  // Wait until periph is turned on
  while(HWREGBITW(PRCM_BASE + PRCM_O_PDSTAT0PERIPH, PRCM_PDSTAT0PERIPH_ON_BITN) != 1)
  {}
}
void waitUntilAUXReady(void) {
  // Wait until AUX is powered on and connected to system bus
  while(HWREGBITW(AON_WUC_BASE + AON_WUC_O_PWRSTAT, AON_WUC_PWRSTAT_AUX_PD_ON_BITN) != 1)
  {}
}
