/*
 * SPI_test.c
 *
 *  Created on: Sep 16, 2023
 *      Author: Admin
 */
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
/*
 * PA4 - NSS
 * PA5 - SCLK
 * PA6 - MISO
 * PA7 - MOSI
 * ALT mode = 5;
 */
void SPI1_Init();
void SPI1_GPIOInit();

int main(){
	char code[] = "Hello World";
	SPI1_GPIOInit();
	SPI1_Init();
	But_GPIOInit();
	SPI_SSOEControl(SPI1, ENABLE);
	SPI_SendData(SPI1, &code, 2);
	while(!GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_3));
	delay(10);
	SPI_Peri
	return 0;
}

void But_GPIOInit(){
	GPIO_Handle_t BUTT;
	BUTT.pGPIOx = GPIOB;
	BUTT.GPIO_PinConfig.GPIO_pinNumber = GPIO_PIN_NO_3;
	BUTT.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_IN;
	BUTT.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_PU;
	BUTT.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_LOW;
}

void SPI1_GPIOInit(){
	GPIO_Handle_t SPIpins;
	SPIpins.pGPIOx = GPIOB;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	SPIpins.GPIO_PinConfig.GPIO_pinMode = GPIO_MODE_ALTFN;
	SPIpins.GPIO_PinConfig.GPIO_altFuncMode = 5;
	SPIpins.GPIO_PinConfig.GPIO_pinPuPdControl = GPIO_NO_PUPD;
	SPIpins.GPIO_PinConfig.GPIO_pinOPType = GPIO_OP_PP;
	SPIpins.GPIO_PinConfig.GPIO_pinSpeed = GPIO_SPEED_FAST;
	//SCLK
	SPIpins.GPIO_PinConfig.GPIO_pinNumber = 12;
	GPIO_Init(&SPIpins);
	//NSS
	SPIpins.GPIO_PinConfig.GPIO_pinNumber = 13;
	GPIO_Init(&SPIpins);
	//MISO
	SPIpins.GPIO_PinConfig.GPIO_pinNumber = 14;
	GPIO_Init(&SPIpins);
	//MOSI
	SPIpins.GPIO_PinConfig.GPIO_pinNumber = 15;
	GPIO_Init(&SPIpins);
}

void SPI1_Init(){
	SPI_Handle_t SPI1_config;
	SPI1_config.SPIx = SPI2;
	SPI1_config.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST_EDGE;
	SPI1_config.SPI_Config.SPI_CPOL = SPI_CPOL_ZERO_IDLE;
	SPI1_config.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
	SPI1_config.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI1_config.SPI_Config.SPI_busConfig = SPI_BUSCONFIG_FULL_DUP;
	SPI1_config.SPI_Config.SPI_deviceMode = SPI_MODE_MASTER;
	SPI1_config.SPI_Config.SPI_speed = SPI_SPEED_DIV_8;
	SPI_PeriClockControl(SPI1, ENABLE);
	SPI_Init(&SPI1_config);
}
