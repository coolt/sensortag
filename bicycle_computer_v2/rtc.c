// Code from PA
#include "config.h"

#include "cc26xxware_2_22_00_16101/inc/hw_aon_event.h"
#include "cc26xxware_2_22_00_16101/inc/hw_memmap.h"
#include "cc26xxware_2_22_00_16101/inc/hw_ints.h"
#include "cc26xxware_2_22_00_16101/inc/hw_nvic.h"

#include "cc26xxware_2_22_00_16101/driverLib/aon_rtc.h"
#include "cc26xxware_2_22_00_16101/driverLib/sys_ctrl.h"



/**  // new function from PA
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




// new function from PA
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


