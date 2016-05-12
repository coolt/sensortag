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

void configureEM8500(){

	uint32_t bit_rate = 10000; 						// EM8500 f_max = 5 MHz
	uint32_t clk_pin = IOID_27; // => DP3		//BOARD_IOID_SPI_CLK_FLASH;    // IDIO 17
	uint8_t address_byte = 0x55;
	uint8_t buffer[100];
	uint8_t bufferRead[100];
	uint8_t value_byte = 0x44;
	int i=0;

	for(i=0;i<100;i++){
		buffer[i]=0x01;
	}

	buffer[0]=0x40;
	buffer[1]=0x01;


	powerEnableSPIdomain();							// power MCU domain and enable clk




	// send configuration per spi					-> board-spi.c
	if ( accessible() ){							// power spi-domain i.O.

		board_spi_open(bit_rate, clk_pin);
		ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_DEVPACK_CS);//config as output

		while(1){

			// Set Data in writing-Buffer
			buffer[0]=0x40;					// set address EPROM
			buffer[1]=0x01;					// set Value at this adress

			// write
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);		// activate CS (from low to high) select
			board_spi_write(buffer, 2);						// write first 2 bytes from buffer
															// little wait befor end of writing by CS set to 0.
			CPUdelay(3000); 								// CS is gone to fast to GND !
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);//deselect

			// wait before read
			CPUdelay(10000); 						// 10 ms

			// set read comands to buffer
			buffer[0]=0x80;							// command for read
			buffer[1]=0x40;							// adresse, from wher read out data

			// read
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);	 // enable Chip select
			board_spi_write(buffer, 2);				// set to read commands (start read, adress to read)
			CPUdelay(3000); 						//
			//ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);//deselect
			//CPUdelay(10000); 						// 10 ms
			//ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 1);//select
			board_spi_read(bufferRead, 1);			// Read 1 byte form the set address
			CPUdelay(3000); 						//
			ti_lib_gpio_pin_write(BOARD_DEVPACK_CS, 0);//deselect
			CPUdelay(10000);
		}

		ti_lib_gpio_pin_write(1 << IOID_28, 1);//deselect
		board_spi_close();

		//board_spi_flush();							// clear data      -> crash !!!!!!!!!
	}

	// powerDisableSPIdomain();
}



