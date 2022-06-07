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

extern spiCAN spican1;
extern spiCAN spican2;
extern spiCAN spican3;
extern spiCAN spican4;


void spiCAN1_Init()
{
	spican1.SPIx = SPI1;

	spican1.CS_Port = CAN3_CS_GPIO_Port;
	spican1.CS_Pin = CAN3_CS_Pin;

	spican1.CLKO_Pin = CAN3_CLKO_Pin;
	spican1.CLKO_Port = CAN3_CLKO_GPIO_Port;

	spican1.INT_Pin = CAN3_INT_Pin;
	spican1.INT_Port = CAN3_INT_GPIO_Port;

	spican1.INT0_Pin = CAN3_INT0_Pin;
	spican1.INT0_Port = CAN3_INT0_GPIO_Port;

	spican1.INT1_Pin = CAN3_INT1_Pin;
	spican1.INT1_Port = CAN3_INT1_GPIO_Port;
}

//void spiCAN2_Init()
//{
//	spican2.SPIx = SPI1;
//
//	spican2.CS_Port = CAN4_CS_GPIO_Port;
//	spican2.CS_Pin = CAN4_CS_GPIO_Pin;
//
//	spican2.CLKO_Pin = CAN4_CLKO_Pin;
//	spican2.CLKO_Port = CAN4_CLKO_GPIO_Port;
//
//	spican2.INT_Pin = CAN4_INT_Pin;
//	spican2.INT_Port = CAN4_INT_GPIO_Port;
//
//	spican2.INT0_Pin = CAN4_INT0_Pin;
//	spican2.INT0_Port = CAN4_INT0_GPIO_Port;
//
//	spican2.INT1_Pin = CAN4_INT1_Pin;
//	spican2.INT1_Port = CAN4_INT1_GPIO_Port;
//}
//
//void spiCAN3_Init()
//{
//	spican3.SPIx = SPI2;
//
//	spican3.CS_Port = CAN5_CS_GPIO_Port;
//	spican3.CS_Pin = CAN5_CS_GPIO_Pin;
//
//	spican3.CLKO_Pin = CAN5_CLKO_Pin;
//	spican3.CLKO_Port = CAN5_CLKO_GPIO_Port;
//
//	spican3.INT_Pin = CAN5_INT_Pin;
//	spican3.INT_Port = CAN5_INT_GPIO_Port;
//
//	spican3.INT0_Pin = CAN5_INT0_Pin;
//	spican3.INT0_Port = CAN5_INT0_GPIO_Port;
//
//	spican3.INT1_Pin = CAN5_INT1_Pin;
//	spican3.INT1_Port = CAN5_INT1_GPIO_Port;
//}
//
//void spiCAN4_Init()
//{
//	spican4.SPIx = SPI2;
//
//	spican4.CS_Port = CAN6_CS_GPIO_Port;
//	spican4.CS_Pin = CAN6_CS_GPIO_Pin;
//
//	spican4.CLKO_Pin = CAN6_CLKO_Pin;
//	spican4.CLKO_Port = CAN6_CLKO_GPIO_Port;
//
//	spican4.INT_Pin = CAN6_INT_Pin;
//	spican4.INT_Port = CAN6_INT_GPIO_Port;
//
//	spican4.INT0_Pin = CAN6_INT0_Pin;
//	spican4.INT0_Port = CAN6_INT0_GPIO_Port;
//
//	spican4.INT1_Pin = CAN6_INT1_Pin;
//	spican4.INT1_Port = CAN6_INT1_GPIO_Port;
//}
//


void canfd_transmit(spiCAN * spican)
{

	CAN_TX_MSGOBJ message = {0};
	message.bF.id.EID = 0;
	message.bF.id.SID = 0x1;
	message.bF.id.SID11 = 0;

	message.bF.ctrl.DLC = 0x8;
	message.bF.ctrl.RTR = 0;
	message.bF.ctrl.BRS = 0;	// If Bit rate switch is used, data bytes are transmited with DBR otherwise the whole message is transmited with NBR

	uint8_t msg_pld[8] = {0};
//	for(int i = 0; i < sizeof(message_payload); i++)
//	{
//		message_payload[i] = 0xEF;
//	}
	// message_ctrl.ESI and message_ctrl.FDF bits are used only in CAN-FD (Ignored in CAN2.0)

	// Check if FIFO is not full
	uint32_t FIFO_status = 0;
	FIFO_status = spican_readByte_withDMA(cREGADDR_CiFIFOSTA + (CiFIFO_OFFSET * CAN_FIFO_CH1), spican);
	while(!(FIFO_status & 0x1))	// Wait till TFNRFNIF in CiFIFOSTA1 is set (means Transmit FIFO is not full)
	{
		FIFO_status = spican_readByte_withDMA(cREGADDR_CiFIFOSTA + (CiFIFO_OFFSET * CAN_FIFO_CH1), spican);
	}
	// Check next transmit message address
	union FIFO_address
	{
		uint32_t word;
		uint8_t  byte[4];
	};
	union FIFO_address next_message_address = {0};

	spican_read32bitReg_withDMA(cREGADDR_CiFIFOUA + (CiFIFO_OFFSET * CAN_FIFO_CH1), next_message_address.byte, spican);
	// Send message
	spican_write32bitReg(next_message_address.word + 0x400, message.byte, spican);
	spican_write32bitReg(next_message_address.word + 0x404, &message.byte[4], spican);
	spican_write32bitReg(next_message_address.word + 0x408, msg_pld, spican);
	spican_write32bitReg(next_message_address.word + 0x412, &msg_pld[4], spican);
}

// Write
void spican_writeByte(uint32_t address, uint8_t message, spiCAN * spican)
{
	uint8_t buffer[3] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_WRITE << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	buffer[2] = message;

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 3, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
}

void spican_write32bitReg(uint32_t address, uint8_t * message, spiCAN * spican)
{
	uint8_t buffer[6] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_WRITE << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	buffer[2] = message[0];
	buffer[3] = message[1];
	buffer[4] = message[2];
	buffer[5] = message[3];

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 6, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
}


// Read
uint8_t spican_readByte(uint32_t address, spiCAN * spican)
{
	uint8_t buffer[3] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 3, SPI1);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	SPI_Receive(&buffer[2], 1, SPI1);


	return buffer[2];
}

uint8_t spican_readByte_withDMA(uint32_t address, spiCAN * spican)
{
	uint8_t buffer[3] = {0};
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
	DMA2_Stream0->NDTR = 3;
	// Select memory destination
	DMA2_Stream0->M0AR = (uint32_t)buffer;
	// Start DMA
	DMA2_Stream0->CR |= (1U<<0);

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 3, SPI1);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);

	return buffer[2];
}

void spican_read32bitReg_withDMA(uint32_t address, uint8_t * reg_buffer, spiCAN * spican)
{

	int8_t buffer[6] = {0};
	uint8_t rx_buffer[6] = {0};
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
	DMA2_Stream0->M0AR = (uint32_t)rx_buffer;
	// Start DMA
	DMA2_Stream0->CR |= (1U<<0);

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 6, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);

	reg_buffer[0] = rx_buffer[2];
	reg_buffer[1] = rx_buffer[3];
	reg_buffer[2] = rx_buffer[4];
	reg_buffer[3] = rx_buffer[5];
}

void spican_read32bitReg(uint32_t address, uint8_t * reg_buffer, spiCAN * spican)
{
	int8_t buffer[6] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 6, SPI1);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	SPI_Receive(reg_buffer, 4, SPI1);
}

void spican_readBytes(uint32_t address, uint8_t * rx_buffer, uint32_t size, spiCAN * spican)
{
	int8_t buffer[2 + size];
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	for(int i = 2; i < sizeof(buffer); i++)
	{
		buffer[i] = 0;
	}

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, sizeof(buffer), SPI2);
	SPI_Receive(rx_buffer, size, SPI2);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);

}
