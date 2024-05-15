/*
 * led.c
 *
 *  Created on: Jul 15, 2023
 *      Author: Admin
 */

#include "stm32f407xx_gpio_driver.h"

void delay(void){
	for(int i = 0; i < 30000; i++);
}

int main(){
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_15;
	LED.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_PP;
	LED.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	LED.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&LED);
	while(1){
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		delay();
	}
}
