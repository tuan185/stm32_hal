/*
 * but_led_interrupt.c
 *
 *  Created on: Jul 25, 2023
 *      Author: Admin
 */
#include "stm32f407xx_gpio_driver.h"

int main(){
	GPIO_Handle_t LED, BUTT;
	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_1;
	LED.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_PP;
	LED.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	LED.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&LED);
	BUTT.pGPIOx = GPIOA;
	BUTT.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_3;
	BUTT.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_IT_RT;
	BUTT.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PU;
	BUTT.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&BUTT);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI3, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI3, 15);
	while(1);
	return 0;
}

void EXTI3_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_3);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
}
