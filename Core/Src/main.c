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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_canfdspi_register.h"
#include "canfd_stm.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi1_rx_buf[20];
REG_CiCON rx_buf;
uint8_t my_buffer[6] = {0};
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
  /* USER CODE BEGIN 2 */
  //SPI1->CR2 |= (1U<<6);	// Enable Rx buffer not empty interrupt
  SPI1->CR1 |= (1U<<6);	// Enable SPI1

//	DMA configuration for SPI1_Rx

  // Enable DMA2 clocking
  RCC->AHB1ENR |= (1U<<22);
  // Disable the stream
  DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
  while((DMA2_Stream0->CR & (1U<<0))) // Check that stream is disabled, if not disable again
  {
	  DMA2_Stream0->CR &= ~(1U<<0);	// Disable the stream0
  }
  // Clear dedicated stream status bits

  // Set peripheral port register address in the DMA_SxPAR register.
  DMA2_Stream0->PAR = (uint32_t)&(SPI1->DR);
  // Set memory address in the DMA_SxMA0R register.
  DMA2_Stream0->M0AR =(uint32_t)my_buffer;
  // Configure number of items to be transferred in the DMA_SxNDTR register.
  DMA2_Stream0->NDTR = 1;
  //Select channel for a stream
  DMA2_Stream0->CR |= 0x3<<25;	// Select channel 3 for stream 0 by writing to CHSEL[2:0]
  // Set a peripheral controller in DMA_CR register
  DMA2_Stream0->CR |= (1U<<5);	// Set a PFCTRL bit to enable peripheral flow controller
  // Configure stream priority
  DMA2_Stream0->CR |= (0x2U<<16); // Select high priority for this stream
  // Select direction for a stream
  DMA2_Stream0->CR &= ~(0x3<<6);	// Direction: Peripheral-to-memory. Select by writing into DIR[1:0]
  // Enable memory increment
  DMA2_Stream0->CR |= (1U<<10); // Set MINC
  // Configure memory data size
  DMA2_Stream0->CR &= ~(0x3<<13); // Set memory size to byte(8-bit)
  // Enable SPI1 DMA controller
  SPI1->CR2 |= (1U<<0);	// Rx Buffer DMA enable
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t counter_rx = 0;
	  static uint32_t h = 1;
	  if(counter_rx == 10000)
	  {

		  if(h == 1)
		  {
			  spican_read32bitReg_withDMA(cREGADDR_CiCON, my_buffer);
			  h = 2;
		  }
		  else if(h == 2)
		  {
			  spican_writeByte(cREGADDR_CiCON+3, 0x2);
			  h = 3;
		  }
		  else if(h == 3)
		  {
			  spican_read32bitReg_withDMA(cREGADDR_CiCON, my_buffer);
			  h = 4;
		  }
		  else if(h == 4)
		  {
			  spican_writeByte(cREGADDR_CiCON+3, 0x4);
			  h = 1;
		  }

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
