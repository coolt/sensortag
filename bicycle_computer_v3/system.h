
#include <inc/hw_types.h>

// void initInterrupts(void); // Dario: alt (wird ersetzt)

// system configuration
void sensorsInit(void); // new
void ledInit(void);  // new

// interrupt configuration
void initRTCInterrupts(void);
void initGPIOInterrupts(void); // new baek, from main
void initRFInterrupts(void); // refactored

// power domain set up
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
void waitUntilAUXReady(void);

void powerDivideInfClkDS(uint32_t);
