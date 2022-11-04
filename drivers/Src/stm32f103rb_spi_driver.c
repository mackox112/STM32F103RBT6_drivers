/*
 * stm32f103rb_spi_driver.c
 *
 *  Created on: 4 sie 2022
 *      Author: macie
 */

#include "stm32f103rb.h"
#include "stm32f103rb_spi_driver.h"


/***********************************************************************
 * @fn				- SPI_PeriClockControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			-
 *
 * @Note			-
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx==GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx==GPIOG)
		{
			GPIOG_PCLK_EN();
		}
	}else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if (pGPIOx==GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if (pGPIOx==GPIOG)
		{
			GPIOG_PCLK_DI();
		}
	}
}
