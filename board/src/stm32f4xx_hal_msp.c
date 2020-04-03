/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp_template.c
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    06-May-2016
  * @brief   This file contains the HAL System and Peripheral (PPP) MSP initialization
  *          and de-initialization functions.
  *          It should be copied to the application folder and renamed into 'stm32f4xx_hal_msp.c'.           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "state.h"
#include "nRF24L01.h"

// [ILG]
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#endif

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions HAL MSP Private Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @note   This function is called from HAL_Init() function to perform system
  *         level initialization (GPIOs, clock, DMA, interrupt).
  * @retval None
  */
void HAL_MspInit(void)
{

}

/**
  * @brief  DeInitializes the Global MSP.
  * @note   This functiona is called from HAL_DeInit() function to perform system
  *         level de-initialization (GPIOs, clock, DMA, interrupt).
  * @retval None
  */
void HAL_MspDeInit(void)
{

}

/**
  * @brief  Initializes the PPP MSP.
  * @note   This functiona is called from HAL_PPP_Init() function to perform 
  *         peripheral(PPP) system level initialization (GPIOs, clock, DMA, interrupt)
  * @retval None
  */
void HAL_PPP_MspInit(void)
{

}

/**
  * @brief  DeInitializes the PPP MSP.
  * @note   This functiona is called from HAL_PPP_DeInit() function to perform 
  *         peripheral(PPP) system level de-initialization (GPIOs, clock, DMA, interrupt)
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{

}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{

	if(hi2c->Instance == I2C2)
	{
		__I2C2_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();

		GPIO_InitTypeDef gpiob;
		gpiob.Alternate = GPIO_AF4_I2C2;
		gpiob.Mode = GPIO_MODE_AF_OD;
		gpiob.Pin = GPIO_PIN_10 | GPIO_PIN_11;
		gpiob.Pull = GPIO_NOPULL;
		gpiob.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &gpiob);
	}
	else if (hi2c->Instance == I2C3)
	{
		__I2C3_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();

		GPIO_InitTypeDef gpio;
		gpio.Alternate = GPIO_AF4_I2C3;
		gpio.Mode = GPIO_MODE_AF_OD;
		gpio.Pin = GPIO_PIN_8;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &gpio);

		gpio.Alternate = GPIO_AF4_I2C3;
		gpio.Mode = GPIO_MODE_AF_OD;
		gpio.Pin = GPIO_PIN_9;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &gpio);

	}
	else abort();
}


void HAL_UART_MspInit(UART_HandleTypeDef* husart)
{

}


void HAL_USART_MspInit(USART_HandleTypeDef* husart)
{
	//	Data transfer.
	if(husart->Instance == USART1)
	{
		__USART1_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();

		GPIO_InitTypeDef gpioa;
		gpioa.Alternate = GPIO_AF7_USART1;
		gpioa.Mode = GPIO_MODE_AF_PP;
		gpioa.Pin = GPIO_PIN_9;
		gpioa.Pull = GPIO_NOPULL;
		gpioa.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &gpioa);

		gpioa.Alternate = GPIO_AF7_USART1;
		gpioa.Mode = GPIO_MODE_AF_OD;
		gpioa.Pin = GPIO_PIN_10;
		gpioa.Pull = GPIO_NOPULL;
		gpioa.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &gpioa);
	}
	else abort();
}


void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	if (hspi->Instance == SPI1) {
		__SPI1_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();

		//	nRF24L01
		GPIO_InitTypeDef gpio;
		gpio.Alternate = GPIO_AF5_SPI1;
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pin = nRF24L01_SCK_PIN | nRF24L01_MISO_PIN | nRF24L01_MOSI_PIN;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(nRF24L01_PORT, &gpio);

		gpio.Mode = GPIO_MODE_OUTPUT_PP;
		gpio.Pin = nRF24L01_CS_PIN | nRF24L01_CE_PIN;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(nRF24L01_PORT, &gpio);

		HAL_GPIO_WritePin(nRF24L01_PORT, nRF24L01_CS_PIN, SET);
	}

	// For lsm6ds3
//	else if (hspi->Instance == SPI2) {
//		__SPI2_CLK_ENABLE();
//		__GPIOB_CLK_ENABLE();
//
//		GPIO_InitTypeDef gpio;
//		gpio.Alternate = GPIO_AF5_SPI2;
//		gpio.Mode = GPIO_MODE_AF_PP;
//		gpio.Pin = LSM6DS3_SCK_PIN | LSM6DS3_MISO_PIN | LSM6DS3_MOSI_PIN;
//		gpio.Pull = GPIO_NOPULL;
//		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
//		HAL_GPIO_Init(LSM6DS3_PORT, &gpio);
//
//		gpio.Mode = GPIO_MODE_OUTPUT_PP;
//		gpio.Pin = LSM6DS3_CS_PIN;
//		gpio.Pull = GPIO_NOPULL;
//		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
//		HAL_GPIO_Init(LSM6DS3_PORT, &gpio);
//
//		HAL_GPIO_WritePin(LSM6DS3_PORT, LSM6DS3_CS_PIN, SET);
//	}
	else abort();
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

// [ILG]
#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
