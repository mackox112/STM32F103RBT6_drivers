


#include "stm32f103rb.h"
#include "string.h"
#include "stdio.h"
//#include "stm32f103rb_gpio_driver.h"

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW



int main(void)
{


	GPIO_Handle_t GpioLed, GpioButton;

	memset(&GpioLed,0,sizeof(GpioLed)); //set every element of this structure to 0
	memset(&GpioButton,0,sizeof(GpioButton));

	//this is PA5 led gpio configuration for Nucleo board
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	//PC13 button pulled up externally to VDD (+3.3V)

	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber= GPIO_PIN_NO_13;
	GpioButton.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinOutputType=GPIO_OUTPUT_TYPE_PP; 	//N/A
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;			// N/A
	GpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQNumber_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(40, ENABLE);





	return 0;
}


void delay(void)
{
	for (uint32_t i=0; i<500000/2; i++);
}


void EXTI15_10_IRQHandler (void){ //this is the Interrupt Service Routine

	//handle the interrupt
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
	printf("Hello World \n");


}

