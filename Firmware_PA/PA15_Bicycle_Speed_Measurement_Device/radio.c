
#include <inc/hw_memmap.h>
#include <inc/hw_rfc_dbell.h>
#include <inc/hw_rfc_pwr.h>
#include <inc/hw_fcfg1.h>
#include <driverlib/prcm.h>

#include "radio_files/rfc_api/common_cmd.h"
#include "radio_files/rfc_api/ble_cmd.h"
#include "radio_files/rfc_api/mailbox.h"
#include "radio_files/patches/ble/apply_patch.h"
#include "config.h"
#include "radio.h"

#include <ble_overrides.h>

#include "system.h"


uint8_t advLength;

volatile bool int_boot_done = 0;

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
  .condition.rule             = COND_NEVER,
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
rfCoreHal_CMD_RADIO_SETUP_t cmdSetup37 = {
  .commandNo                = CMD_RADIO_SETUP,
  .pNextOp                  = (uint8_t*)&cmdAdv0,
  .startTrigger.triggerType = TRIG_NOW,
  .condition.rule           = COND_ALWAYS,
  .pRegOverride             = bleDifferentialOverrides,
  .config.frontEndMode      = 0x0, // Differential
  .config.biasMode          = 0x0, // Internal bias
  .txPower.GC               = 0x3, // 0dbm
  .txPower.IB               = 0x26, // 0dbm
  .mode                     = 0, //BLE mode
};

#pragma data_alignment=4
rfCoreHal_CMD_RADIO_SETUP_t cmdSetup38 = {
  .commandNo                = CMD_RADIO_SETUP,
  .pNextOp                  = (uint8_t*)&cmdAdv1,
  .startTrigger.triggerType = TRIG_REL_PREVEND,
  .condition.rule           = COND_ALWAYS,
  .pRegOverride             = bleDifferentialOverrides,
  .config.frontEndMode      = 0x0, // Differential
  .config.biasMode          = 0x0, // Internal bias
  .txPower.GC               = 0x3, // 0dbm
  .txPower.IB               = 0x26, // 0dbm
  .mode                     = 0, //BLE mode
};

#pragma data_alignment=4
rfCoreHal_CMD_RADIO_SETUP_t cmdSetup39 = {
  .commandNo                = CMD_RADIO_SETUP,
  .pNextOp                  = (uint8_t*)&cmdAdv2,
  .startTrigger.triggerType = TRIG_NOW,
  .condition.rule           = COND_ALWAYS,
  .pRegOverride             = bleDifferentialOverrides,
  .config.frontEndMode      = 0x0, // Differential
  .config.biasMode          = 0x0, // Internal bias
  .txPower.GC               = 0x3, // 0dbm
  .txPower.IB               = 0x26, // 0dbm
  .mode                     = 0, //BLE mode
};

void initRadio(void) {
  // Set radio to BLE mode
  HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = 0x1;

  //TODO: Change device address if needed. Currently using TI Provided adress
  devAddress = (*(uint64_t*)(FCFG1_BASE+FCFG1_O_MAC_BLE_0))&0xFFFFFFFFFFFF;

  //Chain advertisment commands.
  //cmdAdv0.pNextOp = (uint8_t *)&cmdAdv1;
  //cmdAdv1.pNextOp = (uint8_t *)&cmdAdv2;
}

void runRadio(void) {
  // Enable clock to CPE, CPE RAM and RF Core
  HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = RFC_PWR_PWMCLKEN_CPE |  RFC_PWR_PWMCLKEN_CPERAM | RFC_PWR_PWMCLKEN_RFC;
}

 //Send command pointer to doorbell
static inline void radioSendCommand(uint32_t cmd) {
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_CMDR ) = cmd;
}

//Timed wait for radio direct commands to complete
static inline void radioWaitCommandOk(void) {
  while( HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA) == CMDSTA_Pending);
}

//CM0 patching
void radioPatch(void) {
  applyPatch();
}

void radioCmdBusRequest(bool enabled) {
  // Send bus request command as direct command
  uint32_t cmd = CMDR_DIR_CMD_1BYTE(CMD_BUS_REQUEST, enabled);
  radioSendCommand(cmd);
  radioWaitCommandOk();
}


void radioCmdStartRAT(void) {
  uint32_t cmd = CMDR_DIR_CMD(CMD_START_RAT);
  radioSendCommand(cmd);
  radioWaitCommandOk();
}


void radioSetupAndTransmit() {
	// Radio sends on all three channels. Chaining of commands is NOT possible yet. Wait cycles needed between sending the commands.

	// Send packet on channel 37
	  radioSendCommand( (uint32_t)&cmdSetup37);
	  radioWaitCommandOk();

	// Wait until new command can be sent correctly
	  double i;
	  for (i=0; i<180;i++)
	  {
	      ;;
	  }

	// Send packet on channel 38
	  radioSendCommand( (uint32_t)&cmdSetup38);
	  radioWaitCommandOk();

	// Wait until new command can be sent correctly
	  for (i=0; i<180;i++)
	  {
	      ;;
	  }

	// Send packet on channel 39
	  radioSendCommand( (uint32_t)&cmdSetup39);
	  radioWaitCommandOk();
}


//Update advertising data
void radioUpdateAdvData(int length,uint8_t* data) {
  if(length<=BLE_ADV_PAYLOAD_BUF_LEN){
	  cmdAdvParam.advLen = length;
	  cmdAdvParam.pAdvData = (uint8_t*)data;
	  advLength=length;
  }
}

void initRadioInts(void) {

  int_boot_done = 0;
  // Enable interrupt for BOOT_DONE and LAST_CMD_DONE
  uint32_t intVecs = RFC_DBELL_RFCPEIEN_BOOT_DONE_M |
                     RFC_DBELL_RFCPEIEN_COMMAND_DONE_M |
                     RFC_DBELL_RFCPEIEN_LAST_COMMAND_DONE_M;

  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) = intVecs;
}

//Radio CPE ch 1 interrupt - used for BOOT_DONE ISR (by default on CH1)
void RFCCPE1IntHandler(void) {
  int_boot_done = 1;
  //Clear all RF Core ISR flags and wair until done
  do {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
  }
  while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0);
}


//Radio CPE ch 0 interrupt. Used for CMD_DONE and LAST_CMD_DONE (by default on CH0)
//
void RFCCPE0IntHandler(void) {

  //Clear all RF Core ISR flags and wait until done
  do {
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
  }
  while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0);

}
