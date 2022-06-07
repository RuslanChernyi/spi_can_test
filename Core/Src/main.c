/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_canfdspi_register.h"
#include "canfd_stm.h"
#include "canfd_stm_config.h"
#include "DMA.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define cINSTRUCTION_RESET			0x00
#define cINSTRUCTION_READ			0x03
#define cINSTRUCTION_READ_CRC       0x0B
#define cINSTRUCTION_WRITE			0x02
#define cINSTRUCTION_WRITE_CRC      0x0A
#define cINSTRUCTION_WRITE_SAFE     0x0C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t encoder_counter;
uint32_t eg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi1_rx_buf[20];
uint8_t my_buffer[6] = {0};
spiCAN spican1;
spiCAN spican2;
spiCAN spican3;
spiCAN spican4;
canMsg msgID = {0};
UsedFIFOs canfd1_fifos = {0};
mcp_status canfd1_status = {0};
//mcp2517fd mcp_1 = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
//  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  //SPI1->CR2 |= (1U<<6);	// Enable Rx buffer not empty interrupt
  SPI1->CR1 |= (1U<<6);	// Enable SPI1

  spiCAN1_Init();
  DMA_SPI1RXInit(my_buffer);
  // Init Encoder
//  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

//  uint32_t mcp_size = sizeof(mcp_1);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  canfd_getStatus(&canfd1_status, &spican1);
  canfd_configure(&spican1);
  canfd_getStatus(&canfd1_status, &spican1);
  while (1)
  {
	  static uint32_t counter_rx = 0;
	  static uint32_t h = 1;
//	  encoder_counter = __HAL_TIM_GET_COUNTER(&htim4);//htim4.Instance->CNT;
//	  eg = 32767 - ((encoder_counter - 1) & 0xFFFF) / 2;
	  if(counter_rx == 100000)
	  {
		  canfd_transmit(&spican1);
		  canfd_getStatus(&canfd1_status, &spican1);
//		  if(h == 1)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiCON, mcp_1.CiCON.byte, &spican1);
//			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
//			  h = 2;
//		  }
//		  else if(h == 2)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_OSC, mcp_1.OSC.byte, &spican1);
//			  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
//			  h = 3;
//		  }
//		  else if(h == 3)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_IOCON, mcp_1.IOCON.byte, &spican1);
//			  HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
//			  h = 4;
//		  }
//		  else if(h == 4)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiNBTCFG, mcp_1.CiNBTCFG.byte, &spican1);
//			  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
//			  h = 5;
//		  }
//		  else if(h == 5)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiDBTCFG, mcp_1.CiDBTCFG.byte, &spican1);
//			  h = 6;
//		  }
//		  else if(h == 6)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTDC, mcp_1.CiTDC.byte, &spican1);
//			  h = 7;
//		  }
//		  else if(h == 7)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTSCON, mcp_1.CiTSCON.byte, &spican1);
//			  h = 8;
//		  }
//		  else if(h == 8)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiINT, mcp_1.CiINT.byte, &spican1);
//			  h = 9;
//		  }
//		  else if(h == 9)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiBDIAG0, mcp_1.CiBDIAG0.byte, &spican1);
//			  h = 10;
//		  }
//		  else if(h == 10)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiBDIAG1, mcp_1.CiBDIAG1.byte, &spican1);
//			  h = 11;
//		  }
//		  else if(h == 11)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTEFCON, mcp_1.CiTEFCON.byte, &spican1);
//			  h = 12;
//		  }
//		  else if(h == 12)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * 0), mcp_1.CiFIFOCON1.byte, &spican1);
//			  h = 13;
//		  }
//		  else if(h == 13)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * 1), mcp_1.CiFIFOCON2.byte, &spican1);
//			  h = 14;
//		  }
//		  else if(h == 14)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * 2), mcp_1.CiFIFOCON3.byte, &spican1);
//			  h = 15;
//		  }
//		  else if(h == 15)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOCON + (CiFIFO_OFFSET * 3), mcp_1.CiFIFOCON4.byte, &spican1);
//			  h = 16;
//		  }
//		  else if(h == 16)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON, mcp_1.CiTXQCON.byte, &spican1);
//			  h = 17;
//		  }
//
//		  else if(h == 17)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFLTCON + 4 * 0, mcp_1.CiFLTCON0.byte, &spican1);
//			  h = 18;
//		  }
//		  else if(h == 18)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 1, mcp_1.CiFLTCON1.byte, &spican1);
//			  h = 19;
//		  }
//		  else if(h == 19)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 2, mcp_1.CiFLTCON2.byte, &spican1);
//			  h = 20;
//		  }
//		  else if(h == 20)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 3, mcp_1.CiFLTCON3.byte, &spican1);
//			  h = 21;
//		  }
//		  else if(h == 21)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 4, mcp_1.CiFLTCON4.byte, &spican1);
//			  h = 22;
//		  }
//		  else if(h == 22)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 5, mcp_1.CiFLTCON5.byte, &spican1);
//			  h = 23;
//		  }
//		  else if(h == 23)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 6, mcp_1.CiFLTCON6.byte, &spican1);
//			  h = 24;
//		  }
//		  else if(h == 24)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQCON + 4 * 7, mcp_1.CiFLTCON7.byte, &spican1);
//			  h = 25;
//		  }
//		  else if(h == 25)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFLTOBJ + (CiFILTER_OFFSET * 0), mcp_1.CiFLTOBJ0.byte, &spican1);
//			  h = 26;
//		  }
//		  else if(h == 26)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFLTOBJ + (CiFILTER_OFFSET * 1), mcp_1.CiFLTOBJ1.byte, &spican1);
//			  h = 27;
//		  }
//		  else if(h == 27)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFLTOBJ + (CiFILTER_OFFSET * 2), mcp_1.CiFLTOBJ2.byte, &spican1);
//			  h = 28;
//		  }
//		  else if(h == 28)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFLTOBJ + (CiFILTER_OFFSET * 3), mcp_1.CiFLTOBJ3.byte, &spican1);
//			  h = 29;
//		  }
//		  else if(h == 29)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiMASK + (CiFILTER_OFFSET * 0), mcp_1.CiMASK0.byte, &spican1);
//			  h = 30;
//		  }
//		  else if(h == 30)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiMASK + (CiFILTER_OFFSET * 1), mcp_1.CiMASK1.byte, &spican1);
//			  h = 31;
//		  }
//		  else if(h == 31)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiMASK + (CiFILTER_OFFSET * 2), mcp_1.CiMASK2.byte, &spican1);
//			  h = 32;
//		  }
//		  else if(h == 32)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiMASK + (CiFILTER_OFFSET * 3), mcp_1.CiMASK3.byte, &spican1);
//			  h = 33;
//		  }
//		  else if(h == 33)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CRC, mcp_1.CRC_reg.byte, &spican1);
//			  h = 34;
//		  }
//		  else if(h == 34)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_ECCCON, mcp_1.ECCCON.byte, &spican1);
//			  h = 35;
//		  }
//		  else if(h == 35)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_ECCSTA, mcp_1.ECCSTA.byte, &spican1);
//			  h = 36;
//		  }
//		  else if(h == 36)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTBC, mcp_1.CiTBC.byte, &spican1);
//			  h = 37;
//		  }
//		  else if(h == 37)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiVEC, mcp_1.CiVEC.byte, &spican1);
//			  h = 38;
//		  }
//		  else if(h == 38)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiRXIF, mcp_1.CiRXIF.byte, &spican1);
//			  h = 39;
//		  }
//		  else if(h == 39)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXIF, mcp_1.CiTXIF.byte, &spican1);
//			  h = 40;
//		  }
//		  else if(h == 40)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiRXOVIF, mcp_1.CiRXOVIF.byte, &spican1);
//			  h = 41;
//		  }
//		  else if(h == 41)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXATIF, mcp_1.CiTXATIF.byte, &spican1);
//			  h = 42;
//		  }
//		  else if(h == 42)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXREQ, mcp_1.CiTXREQ.byte, &spican1);
//			  h = 43;
//		  }
//		  else if(h == 43)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTREC, mcp_1.CiTREC.byte, &spican1);
//			  h = 44;
//		  }
//		  else if(h == 44)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTEFSTA, mcp_1.CiTEFSTA.byte, &spican1);
//			  h = 45;
//		  }
//		  else if(h == 45)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTEFUA, mcp_1.CiTEFUA.byte, &spican1);
//			  h = 46;
//		  }
//		  else if(h == 46)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQSTA, mcp_1.CiTXQSTA.byte, &spican1);
//			  h = 47;
//		  }
//		  else if(h == 47)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiTXQUA, mcp_1.CiTXQUA.byte, &spican1);
//			  h = 48;
//		  }
//		  else if(h == 48)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOSTA + (CiFIFO_OFFSET * 1), mcp_1.CiFIFOSTA2.byte, &spican1);
//			  h = 49;
//		  }
//		  else if(h == 49)
//		  {
//			  spican_read32bitReg_withDMA(cREGADDR_CiFIFOUA + (CiFIFO_OFFSET * 1), mcp_1.CiFIFOUA2.byte, &spican1);
//			  h = 50;
//		  }
//		  else if(h == 50)
//		  {
//
//			  h = 1;
//		  }

		  counter_rx = 0;
	  }
	  else
	  {
		  counter_rx++;
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void SPI_TransmitReceive (uint8_t *tx_data, int tx_size, uint8_t * rx_data, int rx_size, SPI_TypeDef * SPIx)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<tx_size)
	{
	   while (!((SPIx->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPIx->DR = tx_data[i];  // load the data into the Data Register
	   i++;
	}
	/*During discontinuous communications, there is a 2 APB clock period delay between the
	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
	data.
	*/
	while (!((SPIx->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty

	while (rx_size)
	{
		while (((SPIx->SR) & (1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPIx->DR = 0;  // send dummy data
		while (!((SPIx->SR) & (1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
		*rx_data++ = (SPIx->DR);
		rx_size--;
	}
	//  Clear the Overrun flag by reading DR and SR
	uint32_t temp = SPIx->SR;

}
void SPI_Transmit (uint8_t *data, int size, SPI_TypeDef * SPIx)
{

	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/

	int i=0;
	while (i<size)
	{
	   while (!((SPIx->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	   SPIx->DR = data[i];  // load the data into the Data Register
	   i++;
	}


/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((SPIx->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPIx->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPIx->DR;
	temp = SPIx->SR;
	return;

}

void SPI_Receive (uint8_t *data, int size, SPI_TypeDef * SPIx)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/

	while (size)
	{
		while (((SPIx->SR) & (1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPIx->DR = 0;  // send dummy data
		while (!((SPIx->SR) & (1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPIx->DR);
		size--;
	}
	//  Clear the Overrun flag by reading DR and SR
	uint32_t temp = SPIx->SR;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
