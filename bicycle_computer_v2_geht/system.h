 // vgl. driverlib: sys_ctrl.
// - set system in power down
// - set system in stand by

#include "cc26xxware_2_22_00_16101/inc/hw_types.h"

void initRadioStructs(void);
void initRadio(void);
void radioUpdateAdvData(int size, char* data); // because of shared data
void radioSetupAndTransmit(void); // dito
static inline void radioSendCommand(uint32_t cmd); // dito
void radioCmdBusRequest(bool enabled); // dito
void radioCmdStartRAT(void); // dito


void initRadioInts(void);
void initInterrupts(void);

void sensorsInit(void); // new
void ledInit(void);  // new
void setLED1(void); // baek
void setLED2(void); // baek

void initRTC(void); // Dario in rtc.c
void initRTC_WUms(uint32_t ms); // PA

void powerEnableAuxForceOn(void);
void powerDisableAuxForceOn(void);

void powerEnableCache(void);
void powerDisableCache(void);

void powerEnableCacheRetention(void);
void powerDisableCacheRetention(void);

void powerDisableAuxRamRet(void);

void powerEnableRFC(void);
void powerDisableRFC(void);

void powerEnablePeriph(void);
void powerDisablePeriph(void);

void powerDisableCPU(void);

void powerEnableFlashInIdle(void);
void powerDisableFlashInIdle(void);

void powerEnableMcuPdReq(void);
void powerDisableMcuPdReq(void);

void powerEnableAUXPdReq(void);

void powerEnableGPIOClockRunMode(void);

void powerConfigureRecharge(void);

void powerEnableXtal(void);
void powerDisableXtal(void);
void powerEnableXtalInterface(void);

void waitUntilRFCReady(void);
void waitUntilPeriphReady(void);
void waitUntilAUXReady(void);   // bilds an endlos loop of sending (old) ble data

void powerDivideInfClkDS(uint32_t);

void powerEnableGPIOClockRunMode();
void powerEnablePeriph(void);
