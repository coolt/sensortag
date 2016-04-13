// Configuration for V1

#include "cc26xxware_2_22_00_16101/inc/hw_types.h"

// GPIO Pins
#define IO_A  IOID_0
#define IO_B  IOID_1
#define IO_C  IOID_2
#define IO_D  IOID_3
#define IO_E  IOID_4

// RTC wakeup interval
#define WAKE_INTERVAL_MS 1000
#define WAKE_INTERVAL_TICKS WAKE_INTERVAL_MS*65536/1000

// Advertisment payload length in bytes
#define ADVLEN 10

// RF-Chip needs some global variables
extern uint8_t payload[ADVLEN]; 			// data buffer

extern volatile bool rfBootDone;			// communication flag
extern volatile bool rfSetupDone;			// communication flag
extern volatile bool rfAdvertisingDone;		// communication flag
