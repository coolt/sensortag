
#include <config.h>

#include <inc/hw_aon_event.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_nvic.h>

#include <driverLib/aon_rtc.h>
#include <driverLib/sys_ctrl.h>







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
