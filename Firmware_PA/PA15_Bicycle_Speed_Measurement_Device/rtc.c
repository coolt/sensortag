
#include "config.h"

#include <inc/hw_aon_event.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_nvic.h>

#include <driverLib/aon_rtc.h>
#include <driverLib/sys_ctrl.h>

void initRTC(void) {
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

/**
 * \brief		This function is for using the RTC just for a short time
 * 				to wake up the chip from standby.
 * \param ms	wake up after time in milliseconds
 */
void initRTC_WUms(uint32_t ms){

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

//RTC interrupt handler
void AONRTCIntHandler(void) {

  // Clear RTC event flag
  HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH2_BITN) = 1;
}

/**
 * \brief			Read the actual value of the RTC subsecond register
 * \param return	32-Bit value of the subsecond register. Divide it by
 * 					2^32 - 1 to get the value in seconds.
 */
uint32_t AONRTCCurrentSubSecValueGet( void )
{
    uint32_t   ui32CurrentSec    ;
    uint32_t   ui32CurrentSubSec ;
    uint32_t   ui32SecondSecRead ;

    //
    // Reading SEC both before and after SUBSEC in order to detect if SEC incremented while reading SUBSEC
    // If SEC incremented, we can’t be sure which SEC the SUBSEC belongs to, so repeating the sequence then.
    //
    do {
        ui32CurrentSec    = HWREG( AON_RTC_BASE + AON_RTC_O_SEC    );
        ui32CurrentSubSec = HWREG( AON_RTC_BASE + AON_RTC_O_SUBSEC );
        ui32SecondSecRead = HWREG( AON_RTC_BASE + AON_RTC_O_SEC    );
    } while ( ui32CurrentSec != ui32SecondSecRead );

    return ui32CurrentSubSec;
}

