

//Configuration for simpleBroadcaster

#include <inc/hw_types.h>
#include <driverlib/gpio.h>


#define IO_A  IOID_0
#define IO_B  IOID_1
#define IO_C  IOID_2
#define IO_D  IOID_3
#define IO_E  IOID_4

// RTC wakeup interval
#define WAKE_INTERVAL_MS 1000
#define WAKE_INTERVAL_TICKS WAKE_INTERVAL_MS*65536/1000

//Advertisment payload length in bytes
#define ADVLEN 15

// LED Ports
#define LED_RED			GPIO_PIN_10
#define LED_GREEN		GPIO_PIN_15
