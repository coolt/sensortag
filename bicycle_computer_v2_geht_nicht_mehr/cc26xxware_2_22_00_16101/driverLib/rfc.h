/******************************************************************************
*  Filename:       rfc.h

******************************************************************************/

//*****************************************************************************
//
//! \addtogroup rfc_api
//! @{
//
//*****************************************************************************

#ifndef __RFC_H__
#define __RFC_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <../inc/hw_types.h>
#include <../inc/hw_memmap.h>
#include <../inc/hw_rfc_pwr.h>
#include <../inc/hw_rfc_dbell.h>

//*****************************************************************************
//
// Support for DriverLib in ROM:
// This section renames all functions that are not "static inline", so that
// calling these functions will default to implementation in flash. At the end
// of this file a second renaming will change the defaults to implementation in
// ROM for available functions.
//
// To force use of the implementation in flash, e.g. for debugging:
// - Globally: Define DRIVERLIB_NOROM at project level
// - Per function: Use prefix "NOROM_" when calling the function
//
//*****************************************************************************
#if !defined(DOXYGEN)
    #define RFCCpeIntGetAndClear            NOROM_RFCCpeIntGetAndClear
#endif

//*****************************************************************************
//
// API Functions and prototypes
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Enable the RF core clocks.
//!
//! As soon as the RF core is started it will handle clock control
//! autonomously. No check should be performed to check the clocks. Instead
//! the radio can be ping'ed through the command interface.
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
RFCClockEnable(void)
{
    //
    // Enable all clocks
    //
    HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) =
                                 RFC_PWR_PWMCLKEN_RFCTRC |
                                 RFC_PWR_PWMCLKEN_FSCA |
                                 RFC_PWR_PWMCLKEN_PHA |
                                 RFC_PWR_PWMCLKEN_RAT |
                                 RFC_PWR_PWMCLKEN_RFERAM |
                                 RFC_PWR_PWMCLKEN_RFE |
                                 RFC_PWR_PWMCLKEN_MDMRAM |
                                 RFC_PWR_PWMCLKEN_MDM |
                                 RFC_PWR_PWMCLKEN_CPERAM |
                                 RFC_PWR_PWMCLKEN_CPE |
                                 RFC_PWR_PWMCLKEN_RFC;
}

//*****************************************************************************
//
//! \brief Disable the RF core clocks.
//!
//! As soon as the RF core is started it will handle clock control
//! autonomously. No check should be performed to check the clocks. Instead
//! the radio can be ping'ed through the command interface.
//!
//! When disabling clocks it is the programmers responsibility that the
//! RF core clocks can be safely gated. I.e. the RF core should be safely
//! 'parked'.
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
RFCClockDisable(void)
{
    //
    // Disable all clocks
    //
    HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = 0x0;
}

//*****************************************************************************
//
//! Enable CPE0 interrupt
//
//*****************************************************************************
__STATIC_INLINE void
RFCCpe0IntEnable(uint32_t ui32Mask)
{
  //
  // Multiplex RF Core interrupts to CPE0 IRQ.
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL) &= ~ui32Mask;

  do
  {
    //
    // Clear any pending interrupts.
    //
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0x0);

  //
  //  Enable the masked interrupts
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) |= ui32Mask;
}


//*****************************************************************************
//
//! Enable CPE1 interrupt
//
//*****************************************************************************
__STATIC_INLINE void
RFCCpe1IntEnable(uint32_t ui32Mask)
{
  //
  // Multiplex RF Core interrupts to CPE1 IRQ.
  //
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEISL) |= ui32Mask;

  do
  {
    //
    // Clear any pending interrupts.
    //
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0x0);

  //
  //  Enable the masked interrupts
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN) |= ui32Mask;
}


//*****************************************************************************
//
//! This function is used to map only HW interrupts, and
//! clears/unmasks them. These interrupts are then enabled.
//
//*****************************************************************************
__STATIC_INLINE void
RFCHwIntEnable(uint32_t ui32Mask)
{
  //
  // Clear any pending interrupts.
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0x0;

  //
  //  Enable the masked interrupts
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) |= ui32Mask;
}


//*****************************************************************************
//
//! Disable CPE interrupt
//
//*****************************************************************************
__STATIC_INLINE void
RFCCpeIntDisable(uint32_t ui32Mask)
{
  //
  //  Disable the masked interrupts
  //
  HWREG( RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIEN ) &= ~ui32Mask;

  do
  {
    //
    // Clear any pending interrupts.
    //
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0x0;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != 0x0);
}


//*****************************************************************************
//
//! Disable HW interrupt
//
//*****************************************************************************
__STATIC_INLINE void
RFCHwIntDisable(uint32_t ui32Mask)
{
  //
  //  Disable the masked interrupts
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIEN) &= ~ui32Mask;

  //
  // Clear any pending interrupts.
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0x0;
}


//*****************************************************************************
//
//! Get and clear CPE interrupt flags
//
//*****************************************************************************
extern uint32_t RFCCpeIntGetAndClear(void);


//*****************************************************************************
//
//! Clear interrupt flags
//
//*****************************************************************************
__STATIC_INLINE void
RFCCpeIntClear(uint32_t ui32Mask)
{
  do
  {
    //
    // Clear interrupts that may now be pending
    //
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) &= ~ui32Mask;
  }while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) != ~ui32Mask);
}


//*****************************************************************************
//
//! Clear interrupt flags
//
//*****************************************************************************
__STATIC_INLINE void
RFCHwIntClear(uint32_t ui32Mask)
{
  //
  // Clear pending interrupts.
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) &= ~ui32Mask;
}


//*****************************************************************************
//
//! Clear interrupt flags
//
//*****************************************************************************
__STATIC_INLINE void
RFCAckIntClear(void)
{
  //
  // Clear any pending interrupts.
  //
  HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0x0;
}


//*****************************************************************************
//
// Support for DriverLib in ROM:
// Redirect to implementation in ROM when available.
//
//*****************************************************************************
#if !defined(DRIVERLIB_NOROM) && !defined(DOXYGEN)
    #include "../driverLib/rom.h"
    #ifdef ROM_RFCCpeIntGetAndClear
        #undef  RFCCpeIntGetAndClear
        #define RFCCpeIntGetAndClear            ROM_RFCCpeIntGetAndClear
    #endif
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __RFC_H__

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//
//*****************************************************************************
