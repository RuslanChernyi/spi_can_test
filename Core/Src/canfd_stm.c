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

extern UsedFIFOs canfd1_fifos;

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


// Check if FIFO is not full
uint32_t canfd_checkIfFIFOisNotFull(uint32_t FIFOx, spiCAN * spican)
{
	REG_CiFIFOSTA FIFO_status = {0};
	int32_t timeout = 10;
	uint32_t FIFOstatus_address = cREGADDR_CiFIFOSTA + (CiFIFO_OFFSET * FIFOx);
	spican_read32bitReg_withDMA(FIFOstatus_address, FIFO_status.byte, spican);
	while(FIFO_status.txBF.TxNotFullIF != 1 || timeout <= 0)	// Wait till TFNRFNIF in CiFIFOSTA1 is set (means Transmit FIFO is not full)
	{
		spican_read32bitReg_withDMA(FIFOstatus_address, FIFO_status.byte, spican);
		timeout--;
	}
	if(timeout <= 0)
	{
		return HAL_TIMEOUT;
	}
	return HAL_OK;
}

// Request sending the message
REG_CiTXREQ canfd_requestTransmission(uint32_t FIFOx, spiCAN * spican)
{
	REG_CiTXREQ TXrequest_reg = {0};
	TXrequest_reg.word |= 0x1U<<CAN_FIFO_CH2;
	spican_write32bitReg(cREGADDR_CiTXREQ, TXrequest_reg.byte, spican);
	// Check request register
	spican_read32bitReg_withDMA(cREGADDR_CiTXREQ, TXrequest_reg.byte, spican);
	return TXrequest_reg;
}

// Increment FIFO
void canfd_increment_FIFO(uint32_t FIFOx, REG_CiFIFOCON * fifocon, spiCAN * spican)
{
	fifocon->txBF.TxNotFullIE = 0;
	fifocon->txBF.TxHalfFullIE = 0;
	fifocon->txBF.TxEmptyIE = 0;
	fifocon->txBF.TxAttemptIE = 0;
	fifocon->txBF.RTREnable = 0;
	fifocon->txBF.TxRequest = 0;
	fifocon->txBF.TxPriority = 0;
	fifocon->txBF.TxAttempts = 0;
	fifocon->txBF.UINC = 1;
	fifocon->txBF.TxEnable = 1;
	fifocon->txBF.FRESET = 0;
	fifocon->txBF.FifoSize = 0x3;
	fifocon->txBF.PayLoadSize = 0;

	uint32_t FIFOctrl_address = cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * FIFOx);
	spican_write32bitReg(FIFOctrl_address, fifocon->byte, spican);
	spican_read32bitReg_withDMA(FIFOctrl_address, fifocon->byte, spican);
}

// Check next transmit message address
uint32_t canfd_getNextFIFOmsgAddress(uint32_t FIFOx, spiCAN * spican)
{
	REG_CiFIFOUA FIFO_UserAddress = {0};
	uint32_t FIFOUA_addres = cREGADDR_CiFIFOUA + (CiFIFO_OFFSET * FIFOx);
	spican_read32bitReg_withDMA(FIFOUA_addres, FIFO_UserAddress.byte, spican);
	uint32_t nextFIFOaddressInRAM = FIFO_UserAddress.bF.UserAddress + 0x400;	// Messages start from address 0x400
	return nextFIFOaddressInRAM;
}

// Check FIFO's status register
REG_CiFIFOSTA canfd_getFIFOstatus(uint32_t FIFOx, spiCAN * spican)
{
	REG_CiFIFOSTA FIFO_status = {0};
	uint32_t FIFOstatus_address = cREGADDR_CiFIFOSTA + (CiFIFO_OFFSET * FIFOx);
	spican_read32bitReg_withDMA(FIFOstatus_address, FIFO_status.byte, spican);

	return FIFO_status;
}

void canfd_transmit(spiCAN * spican)
{
	canMsg msgID = {0};

	msgID.id.EID = 0;
	msgID.id.SID = 0x1;
	msgID.id.SID11 = 0;

	msgID.ctrl.DLC = 0x8;
	msgID.ctrl.RTR = 0;
	msgID.ctrl.BRS = 0;	// If Bit rate switch is used, data bytes are transmited with DBR otherwise the whole message is transmited with NBR

	// message_ctrl.ESI and message_ctrl.FDF bits are used only in CAN-FD (Ignored in CAN2.0)

	//// Initialize message array
	for(int i = 0; i < sizeof(msgID.message); i++)
	{
		msgID.message[i] = 0xEF;
	}
	////

	// Check if FIFO is not full
	canfd_checkIfFIFOisNotFull(CAN_FIFO_CH2, spican);

	// Check next transmit message address
	uint32_t InRAMmsg_address = canfd_getNextFIFOmsgAddress(CAN_FIFO_CH2, spican);
	// Send message
	spican_write8bitArray(InRAMmsg_address, msgID.byte, sizeof(msgID.byte), spican);

	// Increment FIFO
	canfd_increment_FIFO(CAN_FIFO_CH2, &canfd1_fifos.FIFO2, spican);

	// Request sending the message
	canfd_requestTransmission(CAN_FIFO_CH2, spican);


	// Check message in RAM
	canMsg msgInRAM[16] = {0};
	spican_readBytes_withDMA(InRAMmsg_address, msgInRAM->byte, sizeof(msgInRAM->byte), spican);
	// Check request register
	uint8_t check_req[4] = {0};
	spican_read32bitReg_withDMA(cREGADDR_CiTXREQ, check_req, spican);
	// Check FIFO's status register
	REG_CiFIFOSTA FIFO_status = canfd_getFIFOstatus(CAN_FIFO_CH2, spican);
	HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	return;
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
void spican_write8bitArray(uint32_t address, uint8_t * message, uint32_t size, spiCAN * spican)
{
	uint8_t buffer[size + 2];
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_WRITE << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	for(int i = 0; i < size; i++)
	{
		buffer[i+2] = message[i];
	}

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, sizeof(buffer), spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	return;
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
	SPI_Transmit(buffer, 3, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	SPI_Receive(&buffer[2], 1, spican->SPIx);


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
	SPI_Transmit(buffer, 3, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);

	// Disable DMA
	DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	while(DMA2_Stream0->CR & (1U<<0)) // Check that stream is disabled, if not disable again
	{
		DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	}

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
	// Disable DMA
	DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	while(DMA2_Stream0->CR & (1U<<0)) // Check that stream is disabled, if not disable again
	{
		DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	}
}
void spican_read32bitReg(uint32_t address, uint8_t * reg_buffer, spiCAN * spican)
{
	int8_t buffer[6] = {0};
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, 6, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	SPI_Receive(reg_buffer, 4, spican->SPIx);
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
	SPI_Transmit(buffer, sizeof(buffer), spican->SPIx);
	SPI_Receive(rx_buffer, size, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);

}
void spican_readBytes_withDMA(uint32_t address, uint8_t * rx_buffer, uint32_t size, spiCAN * spican)
{
	uint32_t buffer_size = 2 + size;
	int8_t buffer[buffer_size];
	uint16_t writeCommand = (address & 0x0FFF) | (cINSTRUCTION_READ << 12);
	buffer[0] = writeCommand >> 8;
	buffer[1] = writeCommand & 0xFF;
	for(int i = 2; i < sizeof(buffer); i++)
	{
		buffer[i] = 0;
	}

	// Disable DMA
	DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	while(DMA2_Stream0->CR & (1U<<0)) // Check that stream is disabled, if not disable again
	{
		DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	}
	//Clear status flags
	DMA2->LIFCR = (1U<<5); // Clear Stream 0 Tranfer complete flag
	// Set amount of data to read by DMA
	DMA2_Stream0->NDTR = buffer_size;
	// Select memory destination
	DMA2_Stream0->M0AR = (uint32_t)rx_buffer;
	// Start DMA
	DMA2_Stream0->CR |= (1U<<0);

	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 0);
	SPI_Transmit(buffer, buffer_size, spican->SPIx);
	HAL_GPIO_WritePin(spican->CS_Port, spican->CS_Pin, 1);
	// Disable DMA
	DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	while(DMA2_Stream0->CR & (1U<<0)) // Check that stream is disabled, if not disable again
	{
		DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
	}

	for(int i = 0; i < size; i++)
	{
		rx_buffer[i] = buffer[i+2];
	}

}
