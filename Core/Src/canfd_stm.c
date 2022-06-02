/*
 * canfd_stm.c
 *
 *  Created on: Jun 1, 2022
 *      Author: Kimo
 */

#include "drv_canfdspi_api.h"
#include "drv_canfdspi_defines.h"
#include "drv_canfdspi_register.h"

#include "main.h"
#include "canfd_stm.h"



void spican_writeByte(uint32_t address, uint8_t message)
{
	uint8_t buffer[3] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_WRITE << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	buffer[2] = message;

	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 0);
	SPI_Transmit(buffer, 3, SPI1);
	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 1);
}

uint8_t spican_readByte(uint32_t address)
{
	uint8_t buffer[3] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 0);
	SPI_Transmit(buffer, 3, SPI1);
	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 1);
	SPI_Receive(&buffer[2], 1, SPI1);


	return buffer[2];
}

void spican_read32bitReg_withDMA(uint32_t address, uint8_t * reg_buffer)
{
	int8_t buffer[6] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	// Disable DMA
	DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	while(DMA2_Stream0->CR & (1U<<0)) // Check that stream is disabled, if not disable again
	{
		DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	}
	//Clear status flags
	DMA2->LIFCR = (1U<<5); // Clear Stream 0 Tranfer complete flag
	// Set amount of data to read by DMA
	DMA2_Stream0->NDTR = 6;
	// Select memory destination
	DMA2_Stream0->M0AR = (uint32_t)reg_buffer;
	// Start DMA
	DMA2_Stream0->CR |= (1U<<0);

	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 0);
	SPI_Transmit(buffer, 6, SPI1);
	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 1);
}

void spican_read32bitReg(uint32_t address, uint8_t * reg_buffer)
{
	int8_t buffer[6] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 0);
	SPI_Transmit(buffer, 6, SPI1);
	HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, 1);
	SPI_Receive(reg_buffer, 4, SPI1);


}

void spican_readBytes(uint32_t address, uint8_t * rx_buffer, uint32_t size)
{
	int8_t buffer[2 + size];
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	for(int i = 2; i < sizeof(buffer); i++)
	{
		buffer[i] = 0;
	}

	HAL_GPIO_WritePin(CAN6_CS_GPIO_Port, CAN6_CS_Pin, 0);
	SPI_Transmit(buffer, sizeof(buffer), SPI2);
	SPI_Receive(rx_buffer, size, SPI2);
	HAL_GPIO_WritePin(CAN6_CS_GPIO_Port, CAN6_CS_Pin, 1);

}
