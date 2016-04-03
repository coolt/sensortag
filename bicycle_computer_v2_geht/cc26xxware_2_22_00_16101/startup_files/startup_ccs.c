/******************************************************************************
*  Filename:       startup_ccs.c
*
*  Konstanten für die Applikation liegen in config.h
*
* Importierte Handler:
* - GPIO-Handler von main.c
* - RF-Handler von radio.c
* - RTC-Handler von rtc.c
*
******************************************************************************/


//*****************************************************************************
//
// Check if compiler is CCS
//
//*****************************************************************************
#if !(defined(__TI_COMPILER_VERSION__))
#error "startup_ccs.c: Unsupported compiler!"
#endif

// *********************************************
// Includes for Hanlder
//
// *********************************************

// generell
#include "../../config.h"
#include "../system.h"
#include "../inc/hw_aon_event.h"
#include <../inc/hw_types.h>
#include "../inc/hw_memmap.h"
#include "../driverLib/sys_ctrl.h"

// für gpio
#include "../driverLib/gpio.h"
#include "../driverLib/ioc.h"
#include "../../board.h"

// aus radio.c
#include "../inc/hw_rfc_dbell.h"

// aus rtc.c
#include "../inc/hw_aon_event.h"
#include "../inc/hw_ints.h"
#include "../inc/hw_nvic.h"
#include "../driverLib/aon_rtc.h"
#include "../driverLib/sys_ctrl.h"


// *********************************************
// Global variables for Handler
// *********************************************

// set global in main
volatile bool rfBootDone          = 0;
volatile bool rfSetupDone         = 0;
volatile bool rfAdvertisingDone   = 0;


//*****************************************************************************
//
//! Forward declaration of the reset ISR and the default fault handlers.
//
//*****************************************************************************
void        ResetISR( void );
static void NmiSR( void );
static void FaultISR( void );
static void IntDefaultHandler( void );
extern int  main(void);

// -------------------------

extern void MPUFaultIntHandler( void );
extern void BusFaultIntHandler( void );
extern void UsageFaultIntHandler( void );
extern void SVCallIntHandler( void );
extern void DebugMonIntHandler( void );
extern void PendSVIntHandler( void );
extern void SysTickIntHandler( void );
extern void GPIOIntHandler( void ); //used
extern void I2CIntHandler( void );
extern void RFCCPE1IntHandler( void ); // used
extern void AONIntHandler( void ); // used
extern void AONRTCIntHandler( void ); // used
extern void UART0IntHandler( void );
extern void AUXSWEvent0IntHandler( void );
extern void SSI0IntHandler( void );
extern void SSI1IntHandler( void );
extern void RFCCPE0IntHandler( void );
extern void RFCHardwareIntHandler( void );
extern void RFCCmdAckIntHandler( void );
extern void I2SIntHandler( void );
extern void AUXSWEvent1IntHandler( void );
extern void WatchdogIntHandler( void );
extern void Timer0AIntHandler( void );
extern void Timer0BIntHandler( void );
extern void Timer1AIntHandler( void );
extern void Timer1BIntHandler( void );
extern void Timer2AIntHandler( void );
extern void Timer2BIntHandler( void );
extern void Timer3AIntHandler( void );
extern void Timer3BIntHandler( void );
extern void CryptoIntHandler( void );
extern void uDMAIntHandler( void );
extern void uDMAErrIntHandler( void );
extern void FlashIntHandler( void );
extern void SWEvent0IntHandler( void );
extern void AUXCombEventIntHandler( void );
extern void AONProgIntHandler( void );
extern void DynProgIntHandler( void );
extern void AUXCompAIntHandler( void );
extern void AUXADCIntHandler( void );
extern void TRNGIntHandler( void );



//*****************************************************************************
//
//! The entry point for the application startup code and device trim fxn.
//
//*****************************************************************************
extern void _c_int00(void);
extern void trimDevice(void);


//*****************************************************************************
//
// CCS: Linker variable that marks the top of the stack.
//
//*****************************************************************************
extern unsigned long __STACK_END;


//! The vector table. Note that the proper constructs must be placed on this to
//! ensure that it ends up at physical address 0x0000.0000 or at the start of
//! the program if located at a start address other than 0.
//
//*****************************************************************************
#pragma DATA_SECTION(g_pfnVectors, ".intvecs")
void (* const g_pfnVectors[])(void) =
{
    (void (*)(void))((unsigned long)&__STACK_END),
                                            // The initial stack pointer
    ResetISR,                               // The reset handler
    NmiSR,                                  // The NMI handler
    FaultISR,                               // The hard fault handler
    MPUFaultIntHandler,                     // The MPU fault handler
    BusFaultIntHandler,                     // The bus fault handler
    UsageFaultIntHandler,                   // The usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    SVCallIntHandler,                       // SVCall handler
    DebugMonIntHandler,                     // Debug monitor handler
    0,                                      // Reserved
    PendSVIntHandler,                       // The PendSV handler
    SysTickIntHandler,                      // The SysTick handler
    GPIOIntHandler,                         // AON edge detect
    I2CIntHandler,                          // I2C
    RFCCPE1IntHandler,                      // RF Core Command & Packet Engine 1
    AONIntHandler,                          // AON SpiSplave Rx, Tx and CS
    AONRTCIntHandler,                       // AON RTC
    UART0IntHandler,                        // UART0 Rx and Tx
    AUXSWEvent0IntHandler,                  // AUX software event 0
    SSI0IntHandler,                         // SSI0 Rx and Tx
    SSI1IntHandler,                         // SSI1 Rx and Tx
    RFCCPE0IntHandler,                      // RF Core Command & Packet Engine 0
    RFCHardwareIntHandler,                  // RF Core Hardware
    RFCCmdAckIntHandler,                    // RF Core Command Acknowledge
    I2SIntHandler,                          // I2S
    AUXSWEvent1IntHandler,                  // AUX software event 1
    WatchdogIntHandler,                     // Watchdog timer
    Timer0AIntHandler,                      // Timer 0 subtimer A
    Timer0BIntHandler,                      // Timer 0 subtimer B
    Timer1AIntHandler,                      // Timer 1 subtimer A
    Timer1BIntHandler,                      // Timer 1 subtimer B
    Timer2AIntHandler,                      // Timer 2 subtimer A
    Timer2BIntHandler,                      // Timer 2 subtimer B
    Timer3AIntHandler,                      // Timer 3 subtimer A
    Timer3BIntHandler,                      // Timer 3 subtimer B
    CryptoIntHandler,                       // Crypto Core Result available
    uDMAIntHandler,                         // uDMA Software
    uDMAErrIntHandler,                      // uDMA Error
    FlashIntHandler,                        // Flash controller
    SWEvent0IntHandler,                     // Software Event 0
    AUXCombEventIntHandler,                 // AUX combined event
    AONProgIntHandler,                      // AON programmable 0
    DynProgIntHandler,                      // Dynamic Programmable interrupt
                                            // source (Default: PRCM)
    AUXCompAIntHandler,                     // AUX Comparator A
    AUXADCIntHandler,                       // AUX ADC new sample or ADC DMA
                                            // done, ADC underflow, ADC overflow
    TRNGIntHandler                          // TRNG event
};


//*****************************************************************************
//
//! This is the code that gets called when the processor first starts execution
//! following a reset event. Only the absolutely necessary set is performed,
//! after which the application supplied entry() routine is called. Any fancy
//! actions (such as making decisions based on the reset cause register, and
//! resetting the bits in that register) are left solely in the hands of the
//! application.
//
//*****************************************************************************
void
ResetISR(void)
{
    //
    // Final trim of device
    //
    trimDevice();

    //
    // Jump to the CCS C Initialization Routine.
    //
    __asm("    .global _c_int00\n"
            "    b.w     _c_int00");

    //
    // If we ever return signal Error
    //
    FaultISR();
}

//*****************************************************************************
//
//! This is the code that gets called when the processor receives a NMI. This
//! simply enters an infinite loop, preserving the system state for examination
//! by a debugger.
//
//*****************************************************************************
static void
NmiSR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
//! This is the code that gets called when the processor receives a fault
//! interrupt. This simply enters an infinite loop, preserving the system state
//! for examination by a debugger.
//
//*****************************************************************************
static void
FaultISR(void)
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}


//*****************************************************************************
//
//! This is the code that gets called when the processor receives an unexpected
//! interrupt. This simply enters an infinite loop, preserving the system state
//! for examination by a debugger.
//
//*****************************************************************************
static void
IntDefaultHandler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

static void MPUFaultIntHandler( void ){ while(1) {}}
static void BusFaultIntHandler( void ){ while(1) {}}
static void UsageFaultIntHandler( void ){ while(1) {}}
static void SVCallIntHandler( void ){ while(1) {}}
static void DebugMonIntHandler( void ){ while(1) {}}
static void PendSVIntHandler( void ){ while(1) {}}
static void SysTickIntHandler( void ){ while(1) {}}


static void GPIOIntHandler(void){

	uint32_t interrupt_pin_mask;
		const uint32_t button = 16;
		const uint32_t reed_switch = 0x02000000;

		// power on GPIO
		powerEnablePeriph();
		powerEnableGPIOClockRunMode();
		while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON)); /* Wait for domains to power on */

		// get interrupt
		interrupt_pin_mask = (HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) & GPIO_PIN_MASK);

		/* Clear the interrupt flags */
			HWREG(GPIO_BASE + GPIO_O_EVFLAGS31_0) = interrupt_pin_mask;

		switch(interrupt_pin_mask){
			case button:
				setLED1();
				break;
			case reed_switch:
				setLED2();
				break;
			default:
				break;
		}

		int bb = 1;

		// Power off
		powerDisablePeriph();
		HWREGBITW(PRCM_BASE + PRCM_O_GPIOCLKGR, PRCM_GPIOCLKGR_CLK_EN_BITN) = 0; // Disable clock for GPIO in CPU run mode
		// Load clock settings
		HWREGBITW(PRCM_BASE + PRCM_O_CLKLOADCTL, PRCM_CLKLOADCTL_LOAD_BITN) = 1;

	//To avoid second interupt with register = 0 (its not fast enough!!)
	__asm(" nop");
	__asm(" nop");
	__asm(" nop");
	__asm(" nop");
	__asm(" nop");
	__asm(" nop");
}
static void I2CIntHandler( void ){ while(1) {}}
static void AONIntHandler( void ){ while(1) {}}
static void UART0IntHandler( void ){ while(1) {}}
static void AUXSWEvent0IntHandler( void ){ while(1) {}}
static void SSI0IntHandler( void ){ while(1) {}}
static void SSI1IntHandler( void ){ while(1) {}}

static void RFCHardwareIntHandler( void ){ while(1) {}}
static void RFCCmdAckIntHandler( void ){ while(1) {}}


//Radio CPE ch 1 interrupt - used for BOOT_DONE ISR (by default on CH1)
void RFCCPE1IntHandler(void) {

	//Clear all RFCore ISR flags and wait until done
	do {
		HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~RFC_DBELL_RFCPEIFG_BOOT_DONE_M;
	}
	while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG)  & RFC_DBELL_RFCPEIFG_BOOT_DONE_M);

	rfBootDone = 1;
}


//Radio CPE ch 0 interrupt. Used for CMD_DONE and LAST_CMD_DONE (by default on CH0)
//
void RFCCPE0IntHandler(void) {

  uint32_t interrupts = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG);

  if(interrupts & RFC_DBELL_RFCPEIFG_COMMAND_DONE_M) {
    rfSetupDone = 1;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) &= ~RFC_DBELL_RFCPEIEN_COMMAND_DONE_M;

    do {
      HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~RFC_DBELL_RFCPEIFG_COMMAND_DONE_M;
    }
    while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RFC_DBELL_RFCPEIFG_COMMAND_DONE_M);
    }

  if(interrupts & RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE_M) {
    rfAdvertisingDone = 1;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) &= ~RFC_DBELL_RFCPEIEN_LAST_COMMAND_DONE_M;

    do {
      HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE_M;
    }
    while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE_M);
    }


}

static void I2SIntHandler( void ){ while(1) {}}
static void AUXSWEvent1IntHandler( void ){ while(1) {}}
static void WatchdogIntHandler( void ){ while(1) {}}
static void Timer0AIntHandler( void ){ while(1) {}}
static void Timer0BIntHandler( void ){ while(1) {}}
static void Timer1AIntHandler( void ){ while(1) {}}
static void Timer1BIntHandler( void ){ while(1) {}}
static void Timer2AIntHandler( void ){ while(1) {}}
static void Timer2BIntHandler( void ){ while(1) {}}
static void Timer3AIntHandler( void ){ while(1) {}}
static void Timer3BIntHandler( void ){ while(1) {}}
static void CryptoIntHandler( void ){ while(1) {}}
static void uDMAIntHandler( void ){ while(1) {}}
static void uDMAErrIntHandler( void ){ while(1) {}}
static void FlashIntHandler( void ){ while(1) {}}
static void SWEvent0IntHandler( void ){ while(1) {}}
static void AUXCombEventIntHandler( void ){ while(1) {}}

static void AONProgIntHandler( void ){ while(1) {}}

//RTC interrupt handler
void AONRTCIntHandler(void) {

	// Clear RTC event flag: Code from PA
	  HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH2_BITN) = 1;

	 /* normal Code -------------------------------start
	  // Clear RTC event flag
	  do{
		AONRTCEventClear(AON_RTC_CH2);
	  }
	  while( AONRTCEventGet(AON_RTC_CH2));
	 ----------------------------------------------------end*/
}
static void DynProgIntHandler( void ){ while(1) {}}
static void AUXCompAIntHandler( void ){ while(1) {}}
static void AUXADCIntHandler( void ){ while(1) {}}
static void TRNGIntHandler( void ){ while(1) {}}

