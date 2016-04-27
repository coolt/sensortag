
#include <inc/hw_types.h>


// interrupt configuration
void initWUCEvent(void);						// add each WUC on a differen channel
void initRTCInterrupts(void); 					// enables RTC0 and RTC2, set RTC2
void initGPIOInterrupts(void);
void initRFInterrupts(void);

// Interrupt-Function
void getCycleTimeFromInterrupt(void);
void start_RTC_speedMeasurement(uint32_t ms); 	// setRTC0  (can not be initalized)
long getEnergyStateFromSPI(void);

// power domain set up
void  enableGPIODomain(void);
void disabeleGPIODomain(void);

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
