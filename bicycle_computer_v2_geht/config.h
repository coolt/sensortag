// Configuration for V1

#include "cc26xxware_2_22_00_16101/inc/hw_types.h"


#define IO_A  IOID_0
#define IO_B  IOID_1
#define IO_C  IOID_2
#define IO_D  IOID_3
#define IO_E  IOID_4

<<<<<<< HEAD
// RTC wakeup interval:
// Register strucutre: 4 Bytes: 2 MSBytes = Seconds    			0x0001  = 1 s,     0x0005 = 5 s
//                              2 LSBytes = Miliseconds			0x4000  = 250 ms,  0x8000 = 500 ms      0xC000 = 750 ms
//                              -> 0x0001'0000 = 1 s
//                              -> 0x0005'8000 = 5.5 s
=======
// RTC wakeup interval
#define WAKE_INTERVAL_MS 1000
#define WAKE_INTERVAL_TICKS WAKE_INTERVAL_MS*65536/1000
>>>>>>> parent of d90f5d5... ongoing

// Wake Intervall darf nicht kürzer als Prozessstruktur sein, sonst ein Interrupt-Konflikt
#define WAKE_INTERVAL 0x10000  // 10 s


// Number of Bytes in Data Buffer
#define ADVLEN 10

extern char g_payload[ADVLEN]
