/*
 * spi.c
 *
 * Configuration of em8500 through spi
 * -----------------------------------
 * Konfigurationsdaten abgelegt im Header
 *
 *
 *
 *  Created on: 04.05.2016
 *      Author: katrin
 */


#include <board.h>
#include <board-spi.h>
#include <spi.h>
#include <system.h>
#include "ti-lib.h"

// uint8_t em8500_config_data[EM_CONFIG_BUFFER_LENGTH ];

/*
 * #define ADDRESS_T_HRV_PERIOD 		0x00
 * #define ADDRESS_T_HRV_MEAS			0x01
 *
 * #define VALUE_T_HRV_PERIOD			0x44
 *
 *
 */
uint8_t EM8500read(uint8_t address){

	uint8_t value = 0;

	// set read comands to buffer
	uint8_t buffer = 0x80 | address;							// command for read (read command | start-adress to read)


	ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);	 	// enable Chip select
	CPUdelay(1 * DELAY_M_SEC); 						//

	// set command
	board_spi_write(buffer, 1);						// set to read commands (start read, adress to read)
	CPUdelay(1 * DELAY_M_SEC); 						// tee_rd >= 0.9 ms

	board_spi_read(value, 1);					// Read 1 byte form the set address
	CPUdelay(8 * DELAY_M_SEC); 						//

	ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);		// deselect CS
	CPUdelay(10 * DELAY_M_SEC);


	return value;
}

void configureEM8500(){

	uint32_t bit_rate = 5000; 					// bitrate == f;  EM8500 f_max = 5 MHz
	uint32_t clk_pin = IOID_17; 				// MCU SCLK == Borad IOID SPI_FLASH_CLK  = DP8 SCLK
	uint8_t buffer[100];
	uint8_t bufferRead[100];



	// set values to buffer
	int i = 0;
	for(i = 0; i < 100 ;i++){
		buffer[i]=0x01;
	}
	buffer[0]=0x00;
	buffer[1]=0x01;


	powerEnableSPIdomain();							// power MCU domain and enable clk


	// send configuration per spi					-> board-spi.c
	if ( accessible() ){							// power spi-domain i.O.

		board_spi_open(bit_rate, clk_pin);			// f = 5 kHz, Pin = DP8 SCLK
		ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_DEVPACK_CS);//config Pin CSN as output

		while(1){

			// Set Data in writing-Buffer
			buffer[0]=0x1B;					// Adress of protection key
			buffer[1]=0xA5;					// value protection key (for eeprom-adresses)
			buffer[2]=0x41;					// set address EPROM
			buffer[3]=0x05;					// set value EPROM
			//buffer[4]=0x41;					// set address EPROM
			//buffer[5]=0x01;					// set value EPROM

			// write
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);		// activate CS (from low to high) select
			CPUdelay(1 * DELAY_M_SEC);
			board_spi_write(buffer, 4);						// write first 2 data bytes from buffer
															// little wait befor end of writing by CS set to 0.
			CPUdelay(8 * DELAY_M_SEC); 						// CS is gone to fast to GND !
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);		// deactivates CS (go to GND)

			// wait before read
			CPUdelay(8 * DELAY_M_SEC);

			uint8_t address = 0x41;
			//uint8_t result = EM8500read(address);

			// set read comands to buffer
			buffer[0]=0x80 | 0x41;							// command for read (read command | start-adress to read)

			// read
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);	 	// enable Chip select
			CPUdelay(1 * DELAY_M_SEC);
			board_spi_write(buffer, 1);						// set to read commands (start read, adress to read)
			CPUdelay(1 * DELAY_M_SEC); 						//

			board_spi_read(bufferRead, 1);					// Read 2 byte form the set address
			CPUdelay(8 * DELAY_M_SEC); 						//
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);		// deselect CS
			CPUdelay(10 * DELAY_M_SEC);
			uint8_t debug_1 = bufferRead[0];
			uint8_t debug_2 = bufferRead[1];

		}

		// ti_lib_gpio_pin_write(1 << IOID_28, 1);      // deselect clk (??)
		board_spi_close();

		//board_spi_flush();							// clear data      -> crash !!!!!!!!!
	}

	// powerDisableSPIdomain();
}



