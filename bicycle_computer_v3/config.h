// Configuration application
#include <inc/hw_types.h>

// GPIO Pins
#define IO_A  IOID_0
#define IO_B  IOID_1
#define IO_C  IOID_2
#define IO_D  IOID_3
#define IO_E  IOID_4

// RTC wakeup interval
// Struktur 0x0000'0000		= obere 2 Byte [s], untere 2 Byte [ms]
// Bsp.     0x0005'4000     = 5 s + 0.5 ms
#define WAKE_INTERVAL_MS 0x00030000 // Nachladewert: Nach wie vielen s wieder ein Interrupt folgt
#define WAKE_INTERVAL_TICKS 0x00050000  // erster Start eines Interrupts (danch Wiederholung nach Interrupt-Invertal-Zeit)

// Advertisment payload length in bytes
#define ADVLEN 10

// RF-Chip needs some global variables
extern char payload[ADVLEN]; 			// data buffer

extern volatile bool rfBootDone;			// communication flag
extern volatile bool rfSetupDone;			// communication flag
extern volatile bool rfAdvertisingDone;		// communication flag
