/*
 * canfd_stm_config.h
 *
 *  Created on: 5 июн. 2022 г.
 *      Author: Kimo
 */

#ifndef INC_CANFD_STM_CONFIG_H_
#define INC_CANFD_STM_CONFIG_H_

void canfd_configure_OSC(spiCAN * spican);
void canfd_configure_IO_INT(spiCAN * spican);
void canfd_configure_CiCON(spiCAN * spican);
void canfd_configure_Timings(spiCAN * spican);

#endif /* INC_CANFD_STM_CONFIG_H_ */
