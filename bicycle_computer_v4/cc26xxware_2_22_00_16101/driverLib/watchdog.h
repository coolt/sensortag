/******************************************************************************
*  Filename:       wdt.h
*  Revised:        2015-09-21 15:19:36 +0200 (Mon, 21 Sep 2015)
*  Revision:       44629
*
*  Description:    Defines and prototypes for the Watchdog Timer.
*
*  Copyright (c) 2015, Texas Instruments Incorporated
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3) Neither the name of the ORGANIZATION nor the names of its contributors may
*     be used to endorse or promote products derived from this software without
*     specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

//*****************************************************************************
//
//! \addtogroup peripheral_group
//! @{
//! \addtogroup wdt_api
//! @{
//
//*****************************************************************************

#ifndef __WDT_H__
#define __WDT_H__

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
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_wdt.h>
#include "../driverLib/debug.h"
#include "../driverLib/interrupt.h"

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
//*****************************************************************************
#define WATCHDOG_LOCK_UNLOCKED       0x00000000  // Unlocked
#define WATCHDOG_LOCK_LOCKED         0x00000001  // Locked
#define WATCHDOG_LOCK_UNLOCK         0x1ACCE551  // Unlocks the Watchdog Timer

//*****************************************************************************
//
// The following are defines for the bit fields in the WDT_ISR, WDT_RIS, and
// WDT_MIS registers.
//
//*****************************************************************************
#define WATCHDOG_INT_TIMEOUT         0x00000001  // Watchdog timer expired

//*****************************************************************************
//
// The type of interrupt that can be generated by the watchdog.
//
//*****************************************************************************
#define WATCHDOG_INT_TYPE_INT   0x00000000
#define WATCHDOG_INT_TYPE_NMI   0x00000004

//*****************************************************************************
//
// API Functions and prototypes
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Determines if the watchdog timer is enabled.
//!
//! This function checks to see if the watchdog timer is enabled.
//!
//! \return Returns status of Watchdog Timer:
//! - \c true  : Watchdog timer is enabled.
//! - \c false : Watchdog timer is disabled.
//
//*****************************************************************************
__STATIC_INLINE bool
WatchdogRunning(void)
{
    //
    // See if the watchdog timer module is enabled, and return.
    //
    return((HWREG(WDT_BASE + WDT_O_CTL) & WDT_CTL_INTEN) ? true : false);
}

//*****************************************************************************
//
//! \brief Enables the watchdog timer.
//!
//! This function enables the watchdog timer counter and interrupt.
//!
//! Once enabled, the watchdog interrupt can only be disabled by a hardware reset.
//!
//! \note This function has no effect if the watchdog timer has been locked.
//!
//! \return None
//!
//! \sa \ref WatchdogLock(), \ref WatchdogUnlock()
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogEnable(void)
{
    // Enable the watchdog timer module.
    HWREGBITW(WDT_BASE + WDT_O_CTL, WDT_CTL_INTEN_BITN) = 1;
}

//*****************************************************************************
//
//! \brief Enables the watchdog timer reset.
//!
//! This function enables the capability of the watchdog timer to issue a reset
//! to the processor after a second timeout condition.
//!
//! \note This function has no effect if the watchdog timer has been locked.
//!
//! \return None
//!
//! \sa \ref WatchdogLock(), \ref WatchdogUnlock()
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogResetEnable(void)
{
    // Enable the watchdog reset.
    HWREGBITW(WDT_BASE + WDT_O_CTL, WDT_CTL_RESEN_BITN) = 1;
}

//*****************************************************************************
//
//! \brief Disables the watchdog timer reset.
//!
//! This function disables the capability of the watchdog timer to issue a
//! reset to the processor after a second timeout condition.
//!
//! \note This function has no effect if the watchdog timer has been locked.
//!
//! \return None
//!
//! \sa \ref WatchdogLock(), \ref WatchdogUnlock()
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogResetDisable(void)
{
    // Disable the watchdog reset.
    HWREGBITW(WDT_BASE + WDT_O_CTL, WDT_CTL_RESEN_BITN) = 0;
}

//*****************************************************************************
//
//! \brief Enables the watchdog timer lock mechanism.
//!
//! This function locks out write access to the watchdog timer configuration
//! registers.
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogLock(void)
{
    //
    // Lock out watchdog register writes. Writing anything to the WDT_O_LOCK
    // register causes the lock to go into effect.
    //
    HWREG(WDT_BASE + WDT_O_LOCK) = WATCHDOG_LOCK_LOCKED;
}

//*****************************************************************************
//
//! \brief Disables the watchdog timer lock mechanism.
//!
//! This function enables write access to the watchdog timer configuration
//! registers.
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogUnlock(void)
{
    //
    // Unlock watchdog register writes.
    //
    HWREG(WDT_BASE + WDT_O_LOCK) = WATCHDOG_LOCK_UNLOCK;
}

//*****************************************************************************
//
//! \brief Gets the state of the watchdog timer lock mechanism.
//!
//! This function returns the lock state of the watchdog timer registers.
//!
//! \return Returns state of lock mechanism.
//! - \c true  : Watchdog timer registers are locked.
//! - \c false : Registers are not locked.
//
//*****************************************************************************
__STATIC_INLINE bool
WatchdogLockState(void)
{
    //
    // Get the lock state.
    //
    return((HWREG(WDT_BASE + WDT_O_LOCK) == WATCHDOG_LOCK_LOCKED) ?
               true : false);
}

//*****************************************************************************
//
//! \brief Sets the watchdog timer reload value.
//!
//! This function configures the value to load into the watchdog timer when the
//! count reaches zero for the first time; if the watchdog timer is running
//! when this function is called, then the value is immediately loaded into the
//! watchdog timer counter.  If the \c ui32LoadVal parameter is 0, then an
//! interrupt is immediately generated.
//!
//! \note This function has no effect if the watchdog timer has been locked.
//!
//! \param ui32LoadVal is the load value for the watchdog timer.
//!
//! \return None
//!
//! \sa \ref WatchdogLock(), \ref WatchdogUnlock(), \ref WatchdogReloadGet()
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogReloadSet(uint32_t ui32LoadVal)
{
    //
    // Set the load register.
    //
    HWREG(WDT_BASE + WDT_O_LOAD) = ui32LoadVal;
}

//*****************************************************************************
//
//! \brief Gets the watchdog timer reload value.
//!
//! This function gets the value that is loaded into the watchdog timer when
//! the count reaches zero for the first time.
//!
//! \return None
//!
//! \sa \ref WatchdogReloadSet()
//
//*****************************************************************************
__STATIC_INLINE uint32_t
WatchdogReloadGet(void)
{
    //
    // Get the load register.
    //
    return(HWREG(WDT_BASE + WDT_O_LOAD));
}

//*****************************************************************************
//
//! \brief Gets the current watchdog timer value.
//!
//! This function reads the current value of the watchdog timer.
//!
//! \return Returns the current value of the watchdog timer.
//
//*****************************************************************************
__STATIC_INLINE uint32_t
WatchdogValueGet(void)
{
    //
    // Get the current watchdog timer register value.
    //
    return(HWREG(WDT_BASE + WDT_O_VALUE));
}

//*****************************************************************************
//
//! \brief Registers an interrupt handler for the watchdog timer interrupt.
//!
//! This function does the actual registering of the interrupt handler. This
//! function also enables the global interrupt in the interrupt controller; the
//! watchdog timer interrupt must be enabled via \ref WatchdogIntEnable(). It is the
//! interrupt handler's responsibility to clear the interrupt source via
//! \ref WatchdogIntClear().
//!
//! \note This function registers the standard watchdog interrupt handler. To
//! register the NMI watchdog handler, use \ref IntRegister() to register the
//! handler for the \b FAULT_NMI interrupt.
//!
//! \param pfnHandler is a pointer to the function to be called when the
//! watchdog timer interrupt occurs.
//!
//! \return None
//!
//! \sa \ref IntRegister() for important information about registering interrupt
//! handlers.
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogIntRegister(void (*pfnHandler)(void))
{
    //
    // Register the interrupt handler.
    //
    IntRegister(INT_WATCHDOG, pfnHandler);

    //
    // Enable the watchdog timer interrupt.
    //
    IntEnable(INT_WATCHDOG);
}

//*****************************************************************************
//
//! \brief Unregisters an interrupt handler for the watchdog timer interrupt.
//!
//! This function does the actual unregistering of the interrupt handler. This
//! function clears the handler to be called when a watchdog timer interrupt
//! occurs. This function also masks off the interrupt in the interrupt
//! controller so that the interrupt handler no longer is called.
//!
//! \note This function registers the standard watchdog interrupt handler. To
//! register the NMI watchdog handler, use \ref IntRegister() to register the
//! handler for the \b FAULT_NMI interrupt.
//!
//! \return None
//!
//! \sa \ref IntRegister() for important information about registering interrupt
//! handlers.
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogIntUnregister(void)
{
    //
    // Disable the interrupt.
    //
    IntDisable(INT_WATCHDOG);

    //
    // Unregister the interrupt handler.
    //
    IntUnregister(INT_WATCHDOG);
}

//*****************************************************************************
//
//! \brief Enables the watchdog timer.
//!
//! This function enables the watchdog timer interrupt by calling \ref WatchdogEnable().
//!
//! \return None
//!
//! \sa \ref WatchdogEnable()
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogIntEnable(void)
{
    // Enable the Watchdog interrupt.
    WatchdogEnable();
}

//*****************************************************************************
//
//! \brief Gets the current watchdog timer interrupt status.
//!
//! This function returns the interrupt status for the watchdog timer module.
//!
//! \return Returns the interrupt status.
//! - 1 : Watchdog time-out has occurred.
//! - 0 : Watchdog time-out has not occurred.
//!
//! \sa \ref WatchdogIntClear();
//
//*****************************************************************************
__STATIC_INLINE uint32_t
WatchdogIntStatus(void)
{
    //
    // Return either the interrupt status or the raw interrupt status as
    // requested.
    //
    return(HWREG(WDT_BASE + WDT_O_RIS));
}

//*****************************************************************************
//
//! \brief Clears the watchdog timer interrupt.
//!
//! The watchdog timer interrupt source is cleared, so that it no longer
//! asserts.
//!
//! \note Due to write buffers and synchronizers in the system it may take several
//! clock cycles from a register write clearing an event in a module and until the
//! event is actually cleared in the NVIC of the system CPU. It is recommended to
//! clear the event source early in the interrupt service routine (ISR) to allow
//! the event clear to propagate to the NVIC before returning from the ISR.
//! At the same time, an early event clear allows new events of the same type to be
//! pended instead of ignored if the event is cleared later in the ISR.
//! It is the responsibility of the programmer to make sure that enough time has passed
//! before returning from the ISR to avoid false re-triggering of the cleared event.
//! A simple, although not necessarily optimal, way of clearing an event before
//! returning from the ISR is:
//! -# Write to clear event (interrupt source). (buffered write)
//! -# Dummy read from the event source module. (making sure the write has propagated)
//! -# Wait two system CPU clock cycles (user code or two NOPs). (allowing cleared event to propagate through any synchronizers)
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogIntClear(void)
{
    //
    // Clear the interrupt source.
    //
    HWREG(WDT_BASE + WDT_O_ICR) = WATCHDOG_INT_TIMEOUT;
}

//*****************************************************************************
//
//! \brief Sets the type of interrupt generated by the watchdog.
//!
//! This function sets the type of interrupt that is generated if the watchdog
//! timer expires.
//!
//! When configured to generate an NMI, the watchdog interrupt must still be
//! enabled with \ref WatchdogIntEnable(), and it must still be cleared inside the
//! NMI handler with \ref WatchdogIntClear().
//!
//! \param ui32Type is the type of interrupt to generate.
//! - \ref WATCHDOG_INT_TYPE_INT : Generate a standard interrupt (default).
//! - \ref WATCHDOG_INT_TYPE_NMI : Generate a non-maskable interrupt (NMI).
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogIntTypeSet(uint32_t ui32Type)
{
    // Check the arguments.
    ASSERT((ui32Type == WATCHDOG_INT_TYPE_INT) ||
           (ui32Type == WATCHDOG_INT_TYPE_NMI));

    // Set the interrupt type.
    HWREGBITW(WDT_BASE + WDT_O_CTL, WDT_CTL_INTTYPE_BITN) = (ui32Type == WATCHDOG_INT_TYPE_INT)? 0 : 1;
}

//*****************************************************************************
//
//! \brief Enables stalling of the watchdog timer during debug events.
//!
//! This function allows the watchdog timer to stop counting when the processor
//! is stopped by the debugger. By doing so, the watchdog is prevented from
//! expiring and resetting the system (if reset is enabled). The watchdog instead expires
//! after the appropriate number of processor cycles have been executed while
//! debugging (or at the appropriate time after the processor has been
//! restarted).
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogStallEnable(void)
{
    // Enable timer stalling.
    HWREGBITW(WDT_BASE + WDT_O_TEST, WDT_TEST_STALL_BITN) = 1;
}

//*****************************************************************************
//
//! \brief Disables stalling of the watchdog timer during debug events.
//!
//! This function disables the debug mode stall of the watchdog timer. By
//! doing so, the watchdog timer continues to count regardless of the processor
//! debug state.
//!
//! \return None
//
//*****************************************************************************
__STATIC_INLINE void
WatchdogStallDisable(void)
{
    // Disable timer stalling.
    HWREGBITW(WDT_BASE + WDT_O_TEST, WDT_TEST_STALL_BITN) = 0;
}

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __WDT_H__

//*****************************************************************************
//
//! Close the Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
