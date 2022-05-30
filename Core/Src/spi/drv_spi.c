/*******************************************************************************
 Simple SPI Transfer function

  File Name:
    drv_spi.c

  Summary:
    Initializes SPI 1. Transfers data over SPI.
    Uses SPI FIFO to speed up transfer.

  Description:
    .

  Remarks:
    
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 Microchip Technology Inc. and its subsidiaries.  
You may use this software and any derivatives exclusively with Microchip products. 
  
THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, 
COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 

IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER 
RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED 
OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED BY LAW, 
MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE 
WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE TERMS.
 *******************************************************************************/
// DOM-IGNORE-END

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "drv_spi.h"
#include "main.h"

#define SPI1EN	(1U<<12)	//Enable clocking for SPI1 module
#define SPI2EN	(1U<<14)	//Enable clocking for SPI2 module
#define GPIOAEN	(1U<<0)		//Enable clocking fo GPIOA module
#define GPIOBEN (1U<<1)		//Enable clocking fo GPIOB module
#define CPOL_SET (1U<<1)
#define CPHA_SET (1U<<0)
#define RXONLY	(1U<<10)	//Allow only receive communication (otherwise - Full duplex)
#define LSBFIRST (1U<<7)	//Set least significant byte to be transmitted first (otherwise - MSB first)
#define MSTR 	(1U<<2)		//Set this device as master
#define DATAFRAME_FORMAT (1U<<11) // Set data frame to 16 bits (otherwise - 8 bits)
#define SSM 	(1U<<9)		//Software slave management
#define SSI		(1U<<8)		//Internal slave select
#define SPE		(1U<<6)		//SPI enabled
#define SR_TXE	(1U<<1)		//SPI Transmit buffer is empty
#define SR_BSY	(1U<<7)		//SPI is busy
#define SR_RXNE	(1U<<0)		//Receive buffer not empty

int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert)
{
    int8_t error = 0;

    // Select Chip Select
    switch(spiSlaveDeviceIndex)
	{
		case SS1:
			HAL_GPIO_WritePin(SS1_GPIO_Port, SS1_Pin, assert);
			break;
		case SS2:
			HAL_GPIO_WritePin(SS2_GPIO_Port, SS2_Pin, assert);
			break;
		case CS_CAN3:
			HAL_GPIO_WritePin(CAN3_CS_GPIO_Port, CAN3_CS_Pin, assert);
			break;
		case CS_CAN4:
			HAL_GPIO_WritePin(CAN4_CS_GPIO_Port, CAN4_CS_Pin, assert);
			break;
		case CS_CAN5:
			HAL_GPIO_WritePin(CAN5_CS_GPIO_Port, CAN5_CS_Pin, assert);
			break;
		case CS_CAN6:
			HAL_GPIO_WritePin(CAN6_CS_GPIO_Port, CAN6_CS_Pin, assert);
			break;
	}
    return error;
}
/*** SPI1 ***/

/*
 * PA5 -> SPI1_CLK
 * PA6 -> SPI1_MISO
 * PA7 -> SPI1_MOSI
 * PA5 -> SPI1_NSS
 */
void SPI1_GPIO_init(void)
{
	/** Enable clock access to GPIOA module **/
	RCC->AHB1ENR |= GPIOAEN;

	/** Set PA5, PA6, PA7 mode to alternative function (01) **/

	//PA5
	GPIOA->MODER &= ~(1U<<10);
	GPIOA->MODER |= (1U<<11);
	//PA6
	GPIOA->MODER &= ~(1U<<12);
	GPIOA->MODER |= (1U<<13);
	//PA7
	GPIOA->MODER &= ~(1U<<14);
	GPIOA->MODER |= (1U<<15);

	/** Set PA9 as an output (10) **/

	//PA9
	GPIOA->MODER |= (1U<<18);
	GPIOA->MODER &= ~(1U<<19);

	/* Set PA5, PA6, PA7 alternate function type to SPI1 (AF5)(1010) */

	//PA5
	GPIOA->AFR[0] |= (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);
	//PA6
	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27);
	//PA7
	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31);

}

void SPI1_init(void)
{
	/** Enable clock access to SPI1 module **/
	RCC->APB2ENR |= SPI1EN;

	/** Set Baud rate control (BR) **/
	//fPCLK/4 = 001
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	/** Set CPOL to 1 and CPHA to 1 **/
	SPI1->CR1 |= CPOL_SET;
	SPI1->CR1 |= CPHA_SET;

	/** Enable Full-Duplex **/
	SPI1->CR1 &= ~RXONLY;

	/** Set MSB first **/
	SPI1->CR1 &= ~LSBFIRST;

	/** Set mode to MASTER **/
	SPI1->CR1 |= MSTR;

	/** Set 8 bit data mode **/
	SPI1->CR1 &= ~DATAFRAME_FORMAT;

	/** Select software slave management by
	 * setting SSM = 1 and SSI = 1 **/
	SPI1->CR1 |= SSM;
	SPI1->CR1 |= SSI;

	/** Enable SPI module **/
	SPI1->CR1 |= SPE;

}

/**************************************************/

/*** SPI2 ***/

/*
 * PB13 -> SPI2_CLK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB12 -> SPI2_NSS
 */
void SPI2_GPIO_init(void)
{
	/** Enable clock access to GPIOA module **/
	RCC->AHB1ENR |= GPIOBEN;

	/** Set PB13, PB14, PB15 mode to alternative function (01) **/

	//PB13
	GPIOB->MODER &= ~(1U<<26);
	GPIOB->MODER |= (1U<<27);
	//PB14
	GPIOB->MODER &= ~(1U<<28);
	GPIOB->MODER |= (1U<<29);
	//PB15
	GPIOB->MODER &= ~(1U<<30);
	GPIOB->MODER |= (1U<<31);

	/** Set PB12 as an output (10) **/

	//PB12
	GPIOB->MODER |= (1U<<24);
	GPIOB->MODER &= ~(1U<<25);

	/* Set PB13, PB14, PB15 alternate function type to SPI2 (AF5)(1010) */

	//PB13
	GPIOB->AFR[1] |= (1U<<20);
	GPIOB->AFR[1] &= ~(1U<<21);
	GPIOB->AFR[1] |= (1U<<22);
	GPIOB->AFR[1] &= ~(1U<<23);
	//PB14
	GPIOB->AFR[1] |= (1U<<24);
	GPIOB->AFR[1] &= ~(1U<<25);
	GPIOB->AFR[1] |= (1U<<26);
	GPIOB->AFR[1] &= ~(1U<<27);
	//PB15
	GPIOB->AFR[1] |= (1U<<28);
	GPIOB->AFR[1] &= ~(1U<<29);
	GPIOB->AFR[1] |= (1U<<30);
	GPIOB->AFR[1] &= ~(1U<<31);

}

void SPI2_init(void)
{
	/** Enable clock access to SPI2 module **/
	RCC->APB1ENR |= SPI2EN;

	/** Set Baud rate control (BR) **/
	//fPCLK/4 = 001
	SPI2->CR1 |= (1U<<3);
	SPI2->CR1 &= ~(1U<<4);
	SPI2->CR1 &= ~(1U<<5);

	/** Set CPOL to 1 and CPHA to 1 **/
	SPI2->CR1 |= CPOL_SET;
	SPI2->CR1 |= CPHA_SET;

	/** Enable Full-Duplex **/
	SPI2->CR1 &= ~RXONLY;

	/** Set MSB first **/
	SPI2->CR1 &= ~LSBFIRST;

	/** Set mode to MASTER **/
	SPI2->CR1 |= MSTR;

	/** Set 8 bit data mode **/
	SPI2->CR1 &= ~DATAFRAME_FORMAT;

	/** Select software slave management by
	 * setting SSM = 1 and SSI = 1 **/
	SPI2->CR1 |= SSM;
	SPI2->CR1 |= SSI;

	/** Enable SPI module **/
	SPI2->CR1 |= SPE;

}

void DRV_SPI_Initialize(uint32_t SPInumber)
{
    switch(SPInumber)
    {
		case 1:
			SPI1_GPIO_init();
			SPI1_init();
			break;
		case 2:
			SPI2_GPIO_init();
			SPI2_init();
    }

    return;
}

int8_t DRV_SPI_TransferData(SPI_HandleTypeDef *hspi, uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
    int8_t error = 0;
    bool continueLoop;
    uint16_t txcounter = 0;
    uint16_t rxcounter = 0;
    uint8_t unitsTxed = 0;
    const uint8_t maxUnits = 16;

    // Assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, true);
    if (error != 0) return error;

    // Loop until spiTransferSize
    do {
        continueLoop = false;
        unitsTxed = 0;

        // Fill transmit FIFO
        if (SPI2->SR & SR_TXE) {
            while ((txcounter < spiTransferSize) && unitsTxed != maxUnits) {
                HAL_SPI_Transmit(hspi, &SpiTxData[txcounter], 1, 10);
                txcounter++;
                continueLoop = true;
                unitsTxed++;
            }
        }

        // Read as many bytes as were queued for transmission
        while (txcounter != rxcounter) {
        	while(!(SPI2->SR & SR_RXNE)) {};
            HAL_SPI_Receive(hspi, &SpiRxData[rxcounter], 1, 10);
            rxcounter++;
            continueLoop = true;
        }

        // Make sure data gets transmitted even if buffer wasn't empty when we started out with
        if ((txcounter > rxcounter) || (txcounter < spiTransferSize)) {
            continueLoop = true;
        }

    } while (continueLoop);

    // De-assert CS
    error = DRV_SPI_ChipSelectAssert(spiSlaveDeviceIndex, false);

    return error;
}

