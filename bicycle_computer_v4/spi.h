/*
 * spi.h
 *
 *  Created on: 04.05.2016
 *      Author: katrin
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>
#include <config.h>
#include <board.h>

// global variables
uint8_t spiBuffer[SPI_BUFFER_LENGTH];
uint8_t em8500_config_data[EM_CONFIG_BUFFER_LENGTH ];


// em8500 config data
// ----------------------
// structure: adress (1 byte), value (1 byte)

#define ADDRESS_T_HRV_PERIOD 		0x00
#define ADDRESS_T_HRV_MEAS			0x01

#define VALUE_T_HRV_PERIOD			0x44 // just for test


uint8_t EM8500read(uint8_t address);
void configureEM8500(void);

// imported values from read out em8500 config (PA)
/*
em8500_config_data[40] = 0x06;					// check: spi-read 32 bit, here buffer of 8 bits
em8500_config_data[41] = 0x06;
em8500_config_data[42] = 0x02;
em8500_config_data[43]=0x05;
em8500_config_data[44]=0x00;
em8500_config_data[45]=0x04;
em8500_config_data[46]=0x00;
0x47=0x2a
0x48=0x29
0x49=0x1e
0x4a=0x1e
0x4b=0x1d
0x4c=0x25
0x4d=0x21
0x4e=0xcf
0x4f=0x7e
0x50=0x15
0x51=0x01
0x52=0x06
0x53=0x65
0x54=0x99
0x55=0x3a
0x56=0x00
0x57=0x67
0x58=0x1b
0x59=0x00
0x5a=0x00
0x5b=0x00
0x5c=0x00
0x5d=0x00
0x5e=0x00
0x5f=0x00
0x60=0x00
0x61=0x00
0x62=0x00
0x63=0x00
0x64=0x00
0x65=0x00
0x66=0x00
0x67=0x00
0x68=0x00
0x69=0x00
0x6a=0x00
0x6b=0x00
0x6c=0x00
0x6d=0x00
0x6e=0x00
0x6f=0x00
0x70=0x00
0x71=0x00
0x72=0x00
0x73=0x00
0x74=0x00
0x75=0xb4
0x76=0xa5
0x77=0x61
0x78=0xaa
0x79=0x0e
0x7a=0x32
0x7b=0x58
0x7c=0x01
0x7d=0xff
0x7e=0xff
0x7f=0x6b
*/





#endif /* SPI_H_ */
