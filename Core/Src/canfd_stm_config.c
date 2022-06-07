/*
 * canfd_stm_config.c
 *
 *  Created on: 5 июн. 2022 г.
 *      Author: Kimo
 */

#include "main.h"
#include "canfd_stm.h"
#include "canfd_stm_config.h"

#include "drv_canfdspi_defines.h"
#include "drv_canfdspi_register.h"

extern UsedFIFOs canfd1_fifos;

void canfd_configure(spiCAN * spican)
{
	// Go to configuration mode
	spican_writeByte(cREGADDR_CiCON+3, 0x4, spican);

	canfd_configure_OSC(spican);
	canfd_configure_IO_INT(spican);
	canfd_configure_CiCON(spican);
	canfd_configure_Timings(spican);
	canfd_configure_Interrupts(spican);
	canfd_configure_TransmitEventFIFO(spican);
	canfd_configure_TransmitQueue(spican);
	canfd_configure_asReceiveFIFO(CAN_FIFO_CH1, &canfd1_fifos.FIFO1, spican);
	canfd_configure_asTransmitFIFO(CAN_FIFO_CH2, &canfd1_fifos.FIFO2, spican);

	CAN_FILTEROBJ_ID filterID_1;
	CAN_FILTEROBJ_ID filterID_2;
	CAN_FILTEROBJ_ID filterID_3;
	CAN_FILTEROBJ_ID filterID_4;

	filterID_1.EXIDE = 0;
	filterID_1.SID = 0xF0;
	filterID_1.EID = 0;
	filterID_1.SID11 = 0;

	filterID_2.EXIDE = 0;
	filterID_2.SID = 0xF1;
	filterID_2.EID = 0;
	filterID_2.SID11 = 0;

	filterID_3.EXIDE = 0;
	filterID_3.SID = 0xF2;
	filterID_3.EID = 0;
	filterID_3.SID11 = 0;

	filterID_4.EXIDE = 0;
	filterID_4.SID = 0x14;
	filterID_4.EID = 0;
	filterID_4.SID11 = 0;

	canfd_configure_FilterObjectX(CAN_FILTER0, &filterID_1, spican);
	canfd_configure_FilterObjectX(CAN_FILTER1, &filterID_2, spican);
	canfd_configure_FilterObjectX(CAN_FILTER2, &filterID_3, spican);
	canfd_configure_FilterObjectX(CAN_FILTER3, &filterID_4, spican);

	CAN_MASKOBJ_ID filterMask_1;
	CAN_MASKOBJ_ID filterMask_2;
	CAN_MASKOBJ_ID filterMask_3;
	CAN_MASKOBJ_ID filterMask_4;

	filterMask_1.MIDE = 1;
	filterMask_1.MSID11 = 1;
	filterMask_1.MEID = 0;
	filterMask_1.MSID = 0x7FF;

	filterMask_2.MIDE = 1;
	filterMask_2.MSID11 = 1;
	filterMask_2.MEID = 0;
	filterMask_2.MSID = 0x7FF;

	filterMask_3.MIDE = 1;
	filterMask_3.MSID11 = 1;
	filterMask_3.MEID = 0;
	filterMask_3.MSID = 0x7FF;

	filterMask_4.MIDE = 1;
	filterMask_4.MSID11 = 1;
	filterMask_4.MEID = 0;
	filterMask_4.MSID = 0x7FF;

	canfd_configure_FilterMaskX(CAN_FILTER0, &filterMask_1, spican);
	canfd_configure_FilterMaskX(CAN_FILTER1, &filterMask_2, spican);
	canfd_configure_FilterMaskX(CAN_FILTER2, &filterMask_3, spican);
	canfd_configure_FilterMaskX(CAN_FILTER3, &filterMask_4, spican);

	canfd_configure_FilterConX(0, CAN_FIFO_CH1, CAN_FIFO_CH2, CAN_FIFO_CH3, CAN_FIFO_CH4, spican);
	// Go to CAN2.0 mode
	spican_writeByte(cREGADDR_CiCON+3, 0x6, spican);
}

void canfd_configure_OSC(spiCAN * spican)
{
	REG_OSC osc = {0};
	osc.bF.PllEnable = 0;
	osc.bF.SCLKDIV = 0;
	osc.bF.CLKODIV = 0x3;

	spican_write32bitReg(cREGADDR_OSC, osc.byte, spican);
}

void canfd_configure_IO_INT(spiCAN * spican)
{
	REG_IOCON iocon = {0};
	iocon.bF.XcrSTBYEnable = 0;
	iocon.bF.PinMode0 = 0;
	iocon.bF.PinMode1 = 0;
	iocon.bF.SOFOutputEnable = 0;
	iocon.bF.INTPinOpenDrain = 0;


	spican_write32bitReg(cREGADDR_IOCON, iocon.byte, spican);
}

void canfd_configure_CiCON(spiCAN * spican)
{
	REG_CiCON cicon = {0};
	cicon.bF.DNetFilterCount = 0;
	cicon.bF.IsoCrcEnable = 0;
	cicon.bF.ProtocolExceptionEventDisable = 1;
	cicon.bF.WakeUpFilterEnable = 0;
	cicon.bF.WakeUpFilterTime = 0;
	cicon.bF.BitRateSwitchDisable = 1;
	cicon.bF.RestrictReTxAttempts = 0;
	cicon.bF.EsiInGatewayMode = 0;
	cicon.bF.SystemErrorToListenOnly = 1;
	cicon.bF.StoreInTEF = 0;
	cicon.bF.TXQEnable = 0;
	cicon.bF.RequestOpMode = 0x4;
	cicon.bF.AbortAllTx = 0;
	cicon.bF.TxBandWidthSharing = 0;

	spican_write32bitReg(cREGADDR_CiCON, cicon.byte, spican);
}

void canfd_configure_Timings(spiCAN * spican)
{
	REG_CiNBTCFG nom_bit_time_con = {0};
	REG_CiDBTCFG data_bit_time_con = {0};
	REG_CiTDC transmit_delay_compensation = {0};

	nom_bit_time_con.bF.BRP = 0;
	nom_bit_time_con.bF.TSEG1 = 254;
	nom_bit_time_con.bF.TSEG2 = 63;
	nom_bit_time_con.bF.SJW = 63;

	data_bit_time_con.bF.BRP = 0;
	data_bit_time_con.bF.TSEG1 = 30;
	data_bit_time_con.bF.TSEG2 = 7;
	data_bit_time_con.bF.SJW = 7;

	transmit_delay_compensation.bF.TDCOffset = 31;
	transmit_delay_compensation.bF.TDCMode = 0x3;

	spican_write32bitReg(cREGADDR_CiNBTCFG, nom_bit_time_con.byte, spican);
	spican_write32bitReg(cREGADDR_CiDBTCFG, data_bit_time_con.byte, spican);
	spican_write32bitReg(cREGADDR_CiTDC, transmit_delay_compensation.byte, spican);
}

void canfd_configure_Interrupts(spiCAN * spican)
{
	REG_CiINT ciint = {0};

	ciint.bF.IE.TXIE = 0;
	ciint.bF.IE.RXIE = 1;
	ciint.bF.IE.TBCIE = 0;
	ciint.bF.IE.MODIE = 1;
	ciint.bF.IE.TEFIE = 0;
	ciint.bF.IE.ECCIE = 0;
	ciint.bF.IE.SPICRCIE = 0;
	ciint.bF.IE.TXATIE = 0;
	ciint.bF.IE.RXOVIE = 1;
	ciint.bF.IE.SERRIE = 1;
	ciint.bF.IE.CERRIE = 1;
	ciint.bF.IE.WAKIE = 0;
	ciint.bF.IE.IVMIE = 1;

	spican_write32bitReg(cREGADDR_CiINT, ciint.byte, spican);
}

void canfd_configure_TransmitEventFIFO(spiCAN * spican)
{
	REG_CiTEFCON citefcon = {0};

	citefcon.bF.TEFNEIE = 0;
	citefcon.bF.TEFHFIE = 0;
	citefcon.bF.TEFHFIE = 0;
	citefcon.bF.TEFOVIE = 0;
	citefcon.bF.TimeStampEnable = 0;
	citefcon.bF.UINC = 1;
	citefcon.bF.FRESET = 1;
	citefcon.bF.FifoSize = 0x3;

	spican_write32bitReg(cREGADDR_CiTEFCON, citefcon.byte, spican);
}

void canfd_configure_TransmitQueue(spiCAN * spican)
{
	REG_CiTXQCON citxqcon = {0};

	citxqcon.txBF.TxNotFullIE = 0;
	citxqcon.txBF.TxEmptyIE = 0;
	citxqcon.txBF.TxAttemptIE = 0;
	citxqcon.txBF.TxEnable = 1;
	citxqcon.txBF.UINC = 1;
	citxqcon.txBF.TxRequest = 0;
	citxqcon.txBF.FRESET = 1;
	citxqcon.txBF.TxPriority = 0;
	citxqcon.txBF.TxAttempts = 0x3;
	citxqcon.txBF.FifoSize = 0x3;
	citxqcon.txBF.PayLoadSize = 0;

	spican_write32bitReg(cREGADDR_CiTXQCON, citxqcon.byte, spican);
}

void canfd_configure_asReceiveFIFO(uint32_t FIFOx, REG_CiFIFOCON * fifocon, spiCAN * spican)
{
	fifocon->rxBF.RxNotEmptyIE = 1;
	fifocon->rxBF.RxHalfFullIE = 1;
	fifocon->rxBF.RxFullIE = 1;
	fifocon->rxBF.RxOverFlowIE = 1;
	fifocon->rxBF.RxTimeStampEnable = 0;
	fifocon->rxBF.UINC = 0;
	fifocon->rxBF.TxEnable = 0;
	fifocon->rxBF.FRESET = 0;
	fifocon->rxBF.FifoSize = 0x1;
	fifocon->rxBF.PayLoadSize = 0;

	uint32_t FIFOctrl_address = cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * FIFOx);
	spican_write32bitReg(FIFOctrl_address, fifocon->byte, spican);
	spican_read32bitReg_withDMA(FIFOctrl_address, fifocon->byte, spican);
}

void canfd_configure_asTransmitFIFO(uint32_t FIFOx, REG_CiFIFOCON * fifocon, spiCAN * spican)
{
	fifocon->txBF.TxNotFullIE = 1;
	fifocon->txBF.TxHalfFullIE = 1;
	fifocon->txBF.TxEmptyIE = 1;
	fifocon->txBF.TxAttemptIE = 1;
	fifocon->txBF.RTREnable = 0;
	fifocon->txBF.TxRequest = 0;
	fifocon->txBF.TxPriority = 0;
	fifocon->txBF.TxAttempts = 0;
	fifocon->txBF.UINC = 0;
	fifocon->txBF.TxEnable = 1;
	fifocon->txBF.FRESET = 0;
	fifocon->txBF.FifoSize = 0x1;
	fifocon->txBF.PayLoadSize = 0;

	uint32_t FIFOctrl_address = cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * FIFOx);
	spican_write32bitReg(FIFOctrl_address, fifocon->byte, spican);
	spican_read32bitReg_withDMA(FIFOctrl_address, fifocon->byte, spican);
}

void canfd_configure_FilterConX(uint32_t FilterConX, uint32_t FIFO_for_filter0, uint32_t FIFO_for_filter1, uint32_t FIFO_for_filter2, uint32_t FIFO_for_filter3, spiCAN * spican)
{
	uint8_t cifltconX[4] = {0};

	REG_CiFLTCON_BYTE filter0 = {0};
	REG_CiFLTCON_BYTE filter1 = {0};
	REG_CiFLTCON_BYTE filter2 = {0};
	REG_CiFLTCON_BYTE filter3 = {0};

	if(FIFO_for_filter0 > 31)
	{
		filter0.bF.Enable = 0;
	}
	else
	{
		filter0.bF.BufferPointer = FIFO_for_filter0;
		filter0.bF.Enable = 1;
	}

	if(FIFO_for_filter1 > 31)
	{
		filter1.bF.Enable = 0;
	}
	else
	{
		filter1.bF.BufferPointer = FIFO_for_filter1;
		filter1.bF.Enable = 1;
	}

	if(FIFO_for_filter2 > 31)
	{
		filter2.bF.Enable = 0;
	}
	else
	{
		filter2.bF.BufferPointer = FIFO_for_filter2;
		filter2.bF.Enable = 1;
	}

	if(FIFO_for_filter0 > 31)
	{
		filter0.bF.Enable = 0;
	}
	else
	{
		filter3.bF.BufferPointer = FIFO_for_filter3;
		filter0.bF.Enable = 1;
	}

	cifltconX[0] = filter0.byte;
	cifltconX[1] = filter1.byte;
	cifltconX[2] = filter2.byte;
	cifltconX[3] = filter3.byte;

	spican_write32bitReg(cREGADDR_CiFLTCON + (CiFILTER_OFFSET * FilterConX), cifltconX, spican);
}

void canfd_configure_FilterObjectX(uint32_t FilterX, CAN_FILTEROBJ_ID * filterId, spiCAN * spican)
{
	REG_CiFLTOBJ cifltobj = {0};

	cifltobj.bF.EID = filterId->EID;
	cifltobj.bF.EXIDE = filterId->EXIDE;
	cifltobj.bF.SID = filterId->SID;
	cifltobj.bF.SID11 = filterId->SID11;

	spican_write32bitReg(cREGADDR_CiFLTOBJ + (CiFILTER_OFFSET * FilterX), cifltobj.byte, spican);
}

void canfd_configure_FilterMaskX(uint32_t FilterX, CAN_MASKOBJ_ID * filterMask, spiCAN * spican)
{
	REG_CiMASK cimask = {0};

	cimask.bF.MEID = filterMask->MEID;
	cimask.bF.MIDE = filterMask->MIDE;
	cimask.bF.MSID = filterMask->MSID;
	cimask.bF.MSID11 = filterMask->MSID11;

	spican_write32bitReg(cREGADDR_CiMASK + (CiFILTER_OFFSET * FilterX), cimask.byte, spican);
}
