/*
 * stm32f103rb_gpio_driver.c
 *
 *  Created on: Sep 29, 2021
 *      Author: macie
 */


#include "stm32f103rb.h"/*MCU specific data*/
#include "stm32f103rb_gpio_driver.h"

//Peripheral Clock Setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
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




//Init and De-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) // TO ReDO - improvement needed
{
	uint8_t temp, temp2 = 0;	//temp. register



	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt mode
		//we need to place the mode setting into appropriate Configuration Register bits

		uint8_t a = 0b1000; //input

		uint8_t b = 0b0000; //input analog mode

		uint8_t c = 0b0010; //push pull output mode low speed
		uint8_t d = 0b0001; //push pull output mode medium speed
		uint8_t e = 0b0011; //push pull output mode high speed

		uint8_t f = 0b0110; //open drain output mode low speed
		uint8_t g = 0b0001; //open drain output mode medium speed
		uint8_t h = 0b0011; //open drain output mode high speed

		uint8_t i = 0b1010; //alternate function output push-pull low speed
		uint8_t j = 0b1001; //alternate function output push-pull medium speed
		uint8_t k = 0b1011; //alternate function output push-pull medium speed

		uint8_t l = 0b1110; //alternate function output open drain low speed
		uint8_t m = 0b1101; //alternate function output open drain medium speed
		uint8_t n = 0b1111; //alternate function output open drain medium speed




		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber <=7) // Port Configuration Register Low CR[0]
		{
			pGPIOHandle->pGPIOx->CR[0] &= ~(0b1111 << (4* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
			pGPIOHandle->pGPIOx->CR[0] |= (e << (4* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//setting
//			//1. Configure the mode of GPIO
//
//			temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//			pGPIOHandle->pGPIOx->CR[0] &= ~(0b11 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
//			pGPIOHandle->pGPIOx->CR[0] |= temp;	//setting
//
//			temp=0;
//
//			//2. Configure the speed
//			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//			pGPIOHandle->pGPIOx->CR[0] &= ~(0b11 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
//			pGPIOHandle->pGPIOx->CR[0] |= temp;
//
//			temp=0;
//
//			//3. Configure pull up pull down settings
//			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//			pGPIOHandle->pGPIOx->CR[0] &= ~(0b11 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
//			pGPIOHandle->pGPIOx->CR[0] |= temp;
//
//			temp=0;
//
//			//4. Configure output type
//			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutputType << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
//			pGPIOHandle->pGPIOx->CR[0] &= ~(0b11 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));	//clearing
//			pGPIOHandle->pGPIOx->CR[0] |= temp;
//
//			temp=0;
//
//			//5. Configure alternate function




		}else
		{
			// Port Configuration Register High CR[1]

			temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

			pGPIOHandle->pGPIOx->CR[1] &= ~(0b1111 << (4* temp2));	//clearing
			pGPIOHandle->pGPIOx->CR[1] |= (a << (4* temp2));	//setting
		}


	}else
	{
			//interrupt mode

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the falling trigger selection register
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. Configure the GPIO port selection in AFIO_EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		AFIO_PCLK_EN();
		AFIO->EXTICR[temp1] = portcode << temp2*4;

		// 3. Enable the exti interrupt delivery using IMR (Interrupt Mask Register)

		EXTI->IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

}



void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx==GPIOA)
			{
				GPIOA_REG_RESET();
			}else if (pGPIOx==GPIOB)
			{
				GPIOB_REG_RESET();
			}else if (pGPIOx==GPIOC)
			{
				GPIOC_REG_RESET();
			}else if (pGPIOx==GPIOD)
			{
				GPIOD_REG_RESET();
			}else if (pGPIOx==GPIOE)
			{
				GPIOE_REG_RESET();
			}else if (pGPIOx==GPIOF)
			{
				GPIOF_REG_RESET();
			}else if (pGPIOx==GPIOG)
			{
				GPIOG_REG_RESET();
			}

}




//Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)

{
	uint8_t value;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001 );
	return value;
}



uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value==GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1<<PinNumber);
	}else
	{
		//write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR	^= (1<< PinNumber);
}

//IRQ Configuration and ISR handling
void GPIO_IRQInterruptConfig (uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE)
	{

		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1<< IRQNumber);

		}else if (IRQNumber >31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1<< (IRQNumber % 32));

		}else if (IRQNumber > 63 && IRQNumber <=95)
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= (1<< (IRQNumber % 64));
		}

	}else
	{
		if (IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1<< IRQNumber);

		}else if (IRQNumber >31 && IRQNumber <= 63)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1<< (IRQNumber % 32));

		}else if (IRQNumber > 63 && IRQNumber <=95)
		{
			//program ICER2 register //64 to 95
			*NVIC_ICER2 |= (1<< (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig (uint8_t IRQNumber , uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = 8*iprx_section + (8 - NO_PRIORITY_BITS_USED);

	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pending register corresponding to the pin number
	if(EXTI->PR & (1<<PinNumber)) // if pending register bit corresponding to this pin number is set (...)
	{
		//clear
		EXTI->PR |= (1<<PinNumber);
	}
}

/***********************************************************************
 * @fn				- GPIO_IRQHandling
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














