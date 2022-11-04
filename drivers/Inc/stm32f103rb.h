/*
 *	stm32f103rb.h
 *
 *	MCU specific header file
 *
 *  Created on: Sep 28, 2021
 *      Author: macie
 */

#ifndef INC_STM32F103RB_H_
#define INC_STM32F103RB_H_

#include <stdint.h>		// for defining uint_t data types

#define __vo					volatile

/******************************************** START: Processor Specific Details *****************************************************
 *
 *  ARM Cortex M3 Processor NVIC ISERx register addresses (Interrupt Set-enable Registers)
 */
#define NVIC_ISER0 				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 				((__vo uint32_t*)0xE000E108)


/*
 *  ARM Cortex M3 Processor NVIC ICERx register addresses (Interrupt Clear-enable Registers)
 */
#define NVIC_ICER0				((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0XE000E188)


/*
 *  ARM Cortex M3 Processor Interrupt Priority Register Address Calculation
 */

#define NVIC_IPR_BASEADDR		((__vo uint32_t*)0xE000E400)

#define NO_PRIORITY_BITS_USED	4	// number of priority bits implemented

/*
 * base addresses of Flash, SRAM and ROM memories
 */
#define FLASH_BASEADDR 			(uint32_t)0x08000000 	//(uint32_t) or U to show unsigned value
#define SRAM 					0x20000000U
#define ROM						0x1FFFF000U 			//System memory


/*
 * AHB and APBx Bus Peripheral base addresses
 */
#define APB1PERIPH_BASE			0x40000000U 			/*		base address of APB bus	 */
#define APB2PERIPH_BASE			0x40010000U
#define AHBPERIPH_BASE			0x40018000U


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all the peripherals
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all the peripherals
 */
#define ADC1_BASEADDR			(APB2PERIPH_BASE + 0x2400)
#define AFIO_BASEADDR			(APB2PERIPH_BASE) /* this is first peripheral of the APB2 Bus*/
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x0400)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x3800)

#define GPIOA_BASEADDR			(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR			(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR 			(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR 			(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR			(APB2PERIPH_BASE + 0x2000)



/*
 * Base addresses of peripherals which are hanging on AHB bus
 * TODO : Complete for all the peripherals
 */
#define DMA1_BASEADDR			(AHBPERIPH_BASE + 0x8000)
#define RCC_BASEADDR			0x40021000U						//(AHBPERIPH_BASE + 0x9000)



/*****************************peripheral register definition structures********************************/

typedef struct
{
	__vo uint32_t CR[2];			/*	Port configuration registers			Address offset:0x00	 */
	__vo uint32_t IDR;				/*	Port input data register				Address offset:0x08	 */
	__vo uint32_t ODR;				/*	Port output data register				Address offset:0x0C	 */
	__vo uint32_t BSRR;				/*	Port bit set/reset register				Address offset:0x10	 */
	__vo uint32_t BRR;				/*	Port bit reset register					Address offset:0x14	 */
	__vo uint32_t LCKR;				/*	Port configuration lock register		Address offset:0x18	 */
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR[2];
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


typedef struct
{
	__vo uint32_t CR;				/*	Clock control register					Address offset:0x00	 */
	__vo uint32_t CFGR;				/*	Clock configuration register			Address offset:0x04	 */
	__vo uint32_t CIR;				/*	Clock interrupt register				Address offset:0x08	 */
	__vo uint32_t APB2RSTR;			/*	APB2 peripheral reset register			Address offset:0x0C	 */
	__vo uint32_t APB1RSTR;			/*	APB1 peripheral reset register			Address offset:0x10	 */
	__vo uint32_t AHBENR;			/*	AHB Peripheral Clock enable register	Address offset:0x14	 */
	__vo uint32_t APB2ENR;			/*	APB2 peripheral clock enable register	Address offset:0x18	 */
	__vo uint32_t APB1ENR;			/*	APB1 peripheral clock enable register	Address offset:0x1C	 */
	__vo uint32_t BDCR;				/*	Backup domain control register			Address offset:0x20	 */
	__vo uint32_t CSR;				/*	Control/status register					Address offset:0x24	 */
	__vo uint32_t AHBSTR;			/*	AHB peripheral clock reset register		Address offset:0x28	 */
	__vo uint32_t CFGR2;			/*	Clock configuration register2			Address offset:0x2C	 */
}RCC_RegDef_t;


typedef struct {
	__vo uint32_t IMR;				/*	Interrupt mask register					Address offset:0x00	 */
	__vo uint32_t EMR;				/*	Event mask register						Address offset:0x04	 */
	__vo uint32_t RTSR;				/*	Rising trigger selection register		Address offset:0x08	 */
	__vo uint32_t FTSR;				/*	Falling trigger selection register		Address offset:0x0C	 */
	__vo uint32_t SWIER;			/*	Software interrupt event register		Address offset:0x10	 */
	__vo uint32_t PR;				/*	Pending register						Address offset:0x14	 */
}EXTI_RegDef_t;


typedef struct {
	__vo uint32_t EVCR;				/*	Event control register							Address offset:0x00	 */
	__vo uint32_t MAPR;				/*	AF remap and debug I/O configuration register	Address offset:0x04	 */
	__vo uint32_t EXTICR[4];		/*	External interrupt configuration register		Address offset:0x08-0x14 */
	__vo uint32_t RESERVED;			/*	Software interrupt event register				Address offset:0x18	 */
	__vo uint32_t MAPR2;			/*	AF remap and debug I/O configuration register	Address offset:0x1C	 */
}AFIO_RegDef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t structure)
 */

#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
//GPIO_RegDef_t *pGPIOA = GPIOA;
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 						((GPIO_RegDef_t*)GPIOG_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define AFIO						((AFIO_RegDef_t*)AFIO_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)



/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()				(RCC->APB2ENR |= (1<<2))
#define GPIOB_PCLK_EN()				(RCC->APB2ENR |= (1<<3))
#define GPIOC_PCLK_EN()				(RCC->APB2ENR |= (1<<4))
#define GPIOD_PCLK_EN()				(RCC->APB2ENR |= (1<<5))
#define GPIOE_PCLK_EN()				(RCC->APB2ENR |= (1<<6))
#define GPIOF_PCLK_EN()				(RCC->APB2ENR |= (1<<7))
#define GPIOG_PCLK_EN()				(RCC->APB2ENR |= (1<<8))


/*
 * Clock Enable Macros for I2C peripherals
 */
#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1<<22))


/*
 * Clock Enable Macros for SPI peripherals
 */
#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1<<15))

/*
 * Clock Enable Macros for USART peripherals
 */
#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1<<14))


/*
 * Clock Enable Macros for AFIO peripherals
 */

#define AFIO_PCLK_EN()				(RCC->APB2ENR |= (1<<0))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()				(RCC->APB2ENR &= ~(1<<2)) //a way to clear a bit
#define GPIOB_PCLK_DI()				(RCC->APB2ENR &= ~(1<<3))
#define GPIOC_PCLK_DI()				(RCC->APB2ENR &= ~(1<<4))
#define GPIOD_PCLK_DI()				(RCC->APB2ENR &= ~(1<<5))
#define GPIOE_PCLK_DI()				(RCC->APB2ENR &= ~(1<<6))
#define GPIOF_PCLK_DI()				(RCC->APB2ENR &= ~(1<<7))
#define GPIOG_PCLK_DI()				(RCC->APB2ENR &= ~(1<<8))


/*
 * Clock Disable Macros for I2C peripherals
 */
#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1<<22))


/*
 * Clock Disable Macros for SPI peripherals
 */
#define SPI1_PCLK_DI()				(RCC->APB2 &= ~(1<<12))


/*
 * Clock Disable Macros for USART peripherals
 */
#define USART1_PCLK_DI()			(RCC->APB2 &= ~(1<<14))


/*
 * Clock Disable Macros for AFIO peripherals
 */

#define AFIO_PCLK_DI()			(RCC->APB2 &= ~(1<<0))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do { (RCC->APB2RSTR |= (1<<2)); (RCC->APB2RSTR &= ~(1<<2)); }while(0)
#define GPIOB_REG_RESET()			do { (RCC->APB2RSTR |= (1<<3)); (RCC->APB2RSTR &= ~(1<<3)); }while(0)
#define GPIOC_REG_RESET()			do { (RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4)); }while(0)
#define GPIOD_REG_RESET()			do { (RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5)); }while(0)
#define GPIOE_REG_RESET()			do { (RCC->APB2RSTR |= (1<<6)); (RCC->APB2RSTR &= ~(1<<6)); }while(0)
#define GPIOF_REG_RESET()			do { (RCC->APB2RSTR |= (1<<7)); (RCC->APB2RSTR &= ~(1<<7)); }while(0)
#define GPIOG_REG_RESET()			do { (RCC->APB2RSTR |= (1<<8)); (RCC->APB2RSTR &= ~(1<<8)); }while(0)


/*
 * Returns port code for given GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)	((	 x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 : 0)


/*
 * IRQ(Interrupt Request) position numbers of STM32F103 MCU
 * NOTE: update these macros with valid values according to your MCU
 */

#define IRQNumber_EXTI0					6
#define IRQNumber_EXTI1					7
#define IRQNumber_EXTI2					8
#define IRQNumber_EXTI3					9
#define IRQNumber_EXTI4					10
#define IRQNumber_EXTI9_5				23
#define IRQNumber_EXTI15_10				40



/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRIO0					0
#define NVIC_IRQ_PRIO1					1
#define NVIC_IRQ_PRIO2					2
#define NVIC_IRQ_PRIO3					3
#define NVIC_IRQ_PRIO4					4
#define NVIC_IRQ_PRIO5					5
#define NVIC_IRQ_PRIO6					6
#define NVIC_IRQ_PRIO7					7
#define NVIC_IRQ_PRIO8					8
#define NVIC_IRQ_PRIO9					9
#define NVIC_IRQ_PRI10					10
#define NVIC_IRQ_PRI11					11
#define NVIC_IRQ_PRI12					12
#define NVIC_IRQ_PRI13					13
#define NVIC_IRQ_PRI14					14
#define NVIC_IRQ_PRI15					15





//Some generic Macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET



#include "stm32f103rb_gpio_driver.h"
#include "stm32f103rb_spi_driver.h"

#endif /* INC_STM32F103RB_H_ */
