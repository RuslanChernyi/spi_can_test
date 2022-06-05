/*
 * canfd_stm.h
 *
 *  Created on: Jun 1, 2022
 *      Author: Kimo
 */

#ifndef INC_CANFD_STM_H_
#define INC_CANFD_STM_H_

typedef struct spiCAN_t
{
	SPI_TypeDef * SPIx;

	uint32_t CS_Pin;
	GPIO_TypeDef * CS_Port;

	uint32_t CLKO_Pin;
	GPIO_TypeDef * CLKO_Port;

	uint32_t INT_Pin;
	GPIO_TypeDef * INT_Port;

	uint32_t INT0_Pin;
	GPIO_TypeDef * INT0_Port;

	uint32_t INT1_Pin;
	GPIO_TypeDef * INT1_Port;
}spiCAN;

uint32_t Configure_CAN();
// Write
void spican_writeByte(uint32_t address, uint8_t message, spiCAN * spican);
void spican_write32bitReg(uint32_t address, uint8_t * message, spiCAN * spican);
// Read
uint8_t spican_readByte(uint32_t address, spiCAN * spican);
uint8_t spican_readByte_withDMA(uint32_t address, spiCAN * spican);
void spican_read32bitReg(uint32_t address, uint8_t * reg_buffer, spiCAN * spican);
void spican_readBytes(uint32_t address, uint8_t * rx_buffer, uint32_t size, spiCAN * spican);
void spican_read32bitReg_withDMA(uint32_t address, uint8_t * reg_buffer, spiCAN * spican);


void spiCAN1_Init();
void spiCAN2_Init();
void spiCAN3_Init();
void spiCAN4_Init();
#endif /* INC_CANFD_STM_H_ */
