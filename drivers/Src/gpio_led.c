

#include <string.h> //memset()

#include "stm32f103rb.h"
#include "stm32f103rb_gpio_driver.h"

void delay(void)
{
	for (uint32_t i=0; i<500000/2; i++);
}

//void dupka();

int main(void)
{
//	dupka(); //to jest test

	GPIO_Handle_t GpioLed;
 	memset(&GpioLed,0,sizeof(GpioLed));// set each and every member element of this structure to 0

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	//PC13 button pulled up externally to VDD (+3.3V)

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP; 	//N/A
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;			// N/A
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);


	/*
	 * GPIO_Init + addition code for interrupt init
	 */
	int ret = GpioButton.GPIO_PinConfig.GPIO_PinNumber % 8;
	uint8_t a = 0b1000;
	GPIOC->CR[1] &= ~((0b1111) << (4*ret));
	GPIOC->CR[1] |= (a << (4* ret));	//setting
	GPIO_Init(&GpioButton); // configure falling edge detection ; configure portcode ; set Interrupt Mask Register

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQNumber_EXTI15_10, 15);
	GPIO_IRQInterruptConfig(40, ENABLE);



	for(int i=0; i<10;i++){
	GPIO_ToggleOutputPin(GPIOA,5);
	delay();
	}

}


void EXTI15_10_IRQHandler (void){ //this is the Interrupt Service Routine
	//handle the interrupt
	GPIO_IRQHandling(GPIO_PIN_NO_13); //clear the pending event from exti line

	for(int i=0	;	i<10	;	i++)
		{
			delay();
		}

}

