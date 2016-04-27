// Configuration application
#include <inc/hw_types.h>

// GPIO Pins
/*#define IO_A  IOID_0
#define IO_B  IOID_1
#define IO_C  IOID_2
#define IO_D  IOID_3
#define IO_E  IOID_4 */

// RTC wakeup interval
// --------------------
// Struktur 0x0000'0000		= obere 2 Byte [s], untere 2 Byte [ms]
// Bsp.     0x0005'4000     = 5 s + 0.5 ms

#define WAKE_INTERVAL_LOW_ENERGY 	0x000A0000   	// 10 s
#define WAKE_INTERVAL_MIDDLE_ENERGY 0x00030000 		//  3 s
#define WAKE_INTERVAL_HIGH_ENERGY 	0x00010000		//  1 s

extern long g_current_wake_up_time/* = WAKE_INTERVAL_HIGH_ENERGY*/;		// = max. = 256 s = 4.5 h

// Energy management
// -----------------
#define LOW_ENERGY 					0x01
#define MIDDLE_ENERGY 				0x02
#define HIGH_ENERGY 				0x04

extern long g_current_energy_state;

// RF-Data
// -------------
//* Length of Data-Block (inclusive length, type, uuid)
#define ADVLEN 16

//* RF-Chip needs some global variables
extern char payload[ADVLEN]; 						// data buffer

extern volatile bool rfBootDone;					// communication flag
extern volatile bool rfSetupDone;					// communication flag
extern volatile bool rfAdvertisingDone;				// communication flag

//* speed measurement
// ---------------------
extern uint32_t g_time1, g_time2, g_timeDiff;
extern bool g_measurement_done, g_first_measurement, g_int_enable;
extern float g_time_ms, g_time_float;

//extern uint32_t test;
