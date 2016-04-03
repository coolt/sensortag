 // RF-Interrupt Handler vorschoben in startup_ccs.c


#include "cc26xxware_2_22_00_16101/inc/hw_memmap.h"
#include "cc26xxware_2_22_00_16101/inc/hw_rfc_dbell.h"
#include "cc26xxware_2_22_00_16101/inc/hw_rfc_pwr.h"
#include "cc26xxware_2_22_00_16101/inc/hw_fcfg1.h"
#include "radio_files/rfc_api/common_cmd.h"
#include "radio_files/rfc_api/ble_cmd.h"
#include "radio_files/rfc_api/mailbox.h"
#include "radio_files/patches/ble/apply_patch.h"
//#include "radio_files/overrides/ble_overrides.h"


#include "config.h"
#include "cc26xxware_2_22_00_16101/driverLib/prcm.h"
#include "radio.h"
#include "system.h"

// Six function are in system.c (because of variable sharing)

void runRadio(void) {
  // Enable clock to CPE, CPE RAM and RF Core
  HWREG(RFC_PWR_NONBUF_BASE + RFC_PWR_O_PWMCLKEN) = RFC_PWR_PWMCLKEN_CPE |  RFC_PWR_PWMCLKEN_CPERAM | RFC_PWR_PWMCLKEN_RFC;
}

//CM0 patching (zusammensetzen)
void radioPatch(void) {
  applyPatch();
}







