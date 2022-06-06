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

void canfd_configure(spiCAN * spican)
{
	// Go to configuration mode
	//spican_readByte_withDMA(cREGADDR_CiCON+3, 0x4, &spican1);
	spican_writeByte(cREGADDR_CiCON+3, 0x4, spican);
	canfd_configure_OSC(spican);
	canfd_configure_IO_INT(spican);
	canfd_configure_CiCON(spican);
	canfd_configure_Timings(spican);
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
	cicon.bF.StoreInTEF = 1;
	cicon.bF.TXQEnable = 1;
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

// TO DO: Check interrupts for different FIFOs. Make structure with different FIFOs.
// TO DO: Clear interrupt flags for different FIFOs in FIFO register.

void canfd_configure_TransmitEventFIFO(spiCAN * spican)
{
	REG_CiTEFCON citefcon = {0};

	citefcon.bF.TEFNEIE = 0;
	citefcon.bF.TEFHFIE = 0;
	citefcon.bF.TEFHFIE = 0;
	citefcon.bF.TEFOVIE = 0;
	citefcon.bF.TimeStampEnable = 0;
	citefcon.bF.UINC = 0;
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
	citxqcon.txBF.UINC = 0;
	citxqcon.txBF.TxRequest = 0;
	citxqcon.txBF.FRESET = 1;
	citxqcon.txBF.TxPriority = 0;
	citxqcon.txBF.TxAttempts = 0x3;
	citxqcon.txBF.FifoSize = 0x3;
	citxqcon.txBF.PayLoadSize = 0;

	spican_write32bitReg(cREGADDR_CiTXQCON, citxqcon.byte, spican);
}

void canfd_configure_asReceiveFIFO(uint32_t FIFOx, spiCAN * spican)
{
	REG_CiFIFOCON fifocon = {0};

	fifocon.rxBF.RxNotEmptyIE = 1;
	fifocon.rxBF.RxHalfFullIE = 0;
	fifocon.rxBF.RxFullIE = 0;
	fifocon.rxBF.RxOverFlowIE = 0;
	fifocon.rxBF.RxTimeStampEnable = 0;
	fifocon.rxBF.UINC = 0;
	fifocon.rxBF.TxEnable = 0;
	fifocon.rxBF.FRESET = 1;
	fifocon.rxBF.FifoSize = 0x3;
	fifocon.rxBF.PayLoadSize = 0;

	spican_write32bitReg(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * FIFOx), fifocon.byte, spican);
}

void canfd_configure_asTransmitFIFO(uint32_t FIFOx,spiCAN * spican)
{
	REG_CiFIFOCON fifocon = {0};

	fifocon.txBF.TxNotFullIE = 0;
	fifocon.txBF.TxHalfFullIE = 0;
	fifocon.txBF.TxEmptyIE = 0;
	fifocon.txBF.TxAttemptIE = 0;
	fifocon.txBF.RTREnable = 0;
	fifocon.txBF.TxRequest = 0;
	fifocon.txBF.TxPriority = 0;
	fifocon.txBF.TxAttempts = 0;
	fifocon.txBF.UINC = 0;
	fifocon.txBF.TxEnable = 1;
	fifocon.txBF.FRESET = 1;
	fifocon.txBF.FifoSize = 0x3;
	fifocon.txBF.PayLoadSize = 0;

	spican_write32bitReg(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * FIFOx), fifocon.byte, spican);
}

void canfd_configure_Filter0_3_toFIFO(spiCAN * spican)
{
	uint8_t cifltcon0[4] = {0};

	REG_CiFLTCON_BYTE filter0 = {0};
	REG_CiFLTCON_BYTE filter1 = {0};
	REG_CiFLTCON_BYTE filter2 = {0};
	REG_CiFLTCON_BYTE filter3 = {0};

	filter0.bF.BufferPointer = 0x1;
	filter0.bF.Enable = 1;

	filter1.bF.BufferPointer = 0x2;
	filter1.bF.Enable = 1;

	filter2.bF.BufferPointer = 0x3;
	filter2.bF.Enable = 1;

	filter3.bF.BufferPointer = 0x4;
	filter3.bF.Enable = 1;

	cifltcon0[0] = filter0.byte;
	cifltcon0[1] = filter1.byte;
	cifltcon0[2] = filter2.byte;
	cifltcon0[3] = filter3.byte;

	spican_write32bitReg(cREGADDR_CiFLTCON + (CiFILTER_OFFSET * 0), cifltcon0, spican);
}
