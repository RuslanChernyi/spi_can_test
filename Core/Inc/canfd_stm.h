/*
 * canfd_stm.h
 *
 *  Created on: Jun 1, 2022
 *      Author: Kimo
 */

#ifndef INC_CANFD_STM_H_
#define INC_CANFD_STM_H_

uint32_t Configure_CAN();
void spican_writeByte(uint32_t address, uint8_t message);
uint8_t spican_readByte(uint32_t address);
void spican_read32bitReg(uint32_t address, uint8_t * reg_buffer);
void spican_readBytes(uint32_t address, uint8_t * rx_buffer, uint32_t size);
void spican_read32bitReg_withDMA(uint32_t address, uint8_t * reg_buffer);


#endif /* INC_CANFD_STM_H_ */
