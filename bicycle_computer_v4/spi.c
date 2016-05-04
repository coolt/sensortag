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

#include <spi.h>
#include <board.h>
#include <board-spi.h>

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

	uint32_t bit_rate = 5000000; 					// EM8500 f_max = 5 MHz
	uint32_t clk_pin = BOARD_IOID_SPI_CLK_FLASH;    // IDIO 17
	uint8_t address_byte = 0x55;
	uint8_t value_byte = 0x44;

	// send configuration per spi					-> board-spi.c
	if ( /*accessible()*/ true ){
		board_spi_flush();							// clear data
		board_spi_open(bit_rate, clk_pin);
		board_spi_write(address_byte, 1);
		board_spi_close();
	}
}



