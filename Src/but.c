/*
 * but.c
 *
 *  Created on: Jul 15, 2023
 *      Author: Admin
 */
#include "stm32f407xx_gpio_driver.h"

void delay(void){
	for(int i = 0; i < 30000; i++);
}

int main(){
	//LED setting
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_14;
	LED.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_PP;
	LED.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	LED.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&LED);
	//Button setting
	GPIO_Handle_t BUTT;
	BUTT.pGPIOx = GPIOB;
	BUTT.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_12;
	BUTT.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_IN;
	BUTT.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PU;
	BUTT.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&BUTT);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == DISABLE){
			GPIO_WriteToOuputPin(GPIOA, GPIO_PIN_NO_14, ENABLE);
		}else{
			GPIO_WriteToOuputPin(GPIOA, GPIO_PIN_NO_14, DISABLE);
		}
	}
}

