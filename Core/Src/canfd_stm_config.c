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


