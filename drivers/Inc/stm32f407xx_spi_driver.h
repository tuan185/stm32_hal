/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jul 22, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "stm32f407xx.h"

/*
 * Configuration structure for SPI pin
 */
typedef struct{
	uint8_t SPI_deviceMode;						// Mode of the SPI (master or slave mode) - reference: @SPI_MODE
	uint8_t SPI_busConfig;						// Configuration for SPI bus (half-duplex or full-duplex) - reference: @SPI_BUSCONFIG
	uint8_t SPI_DFF;							// Data frame format - reference: @SPI_DATA_FRAME_FORMAT
	uint8_t SPI_CPHA;							//Clock phase - reference: @SPI_CPHA
	uint8_t SPI_CPOL;							//Clock polarity - reference: @SPI_CPOL
	uint8_t SPI_SSM;							//Software slave management - reference: @SPI_SSM
	uint8_t SPI_speed;							//Speed of SPI bus - reference: @SPI_SPEED
}SPI_PinConfig_t;

/*
 * macro for SSI
 */



typedef struct
{
	SPI_RegDef_t* SPIx;				//base address of SPIx peripheral
	SPI_PinConfig_t SPI_Config;	//Pin configuration settings for SPIx peripheral
}SPI_Handle_t;
/*
 * @SPI_MODE
 */
#define SPI_MODE_SLAVE					0
#define SPI_MODE_MASTER					1

/*
 * @SPI_BUSCONFIG
 */
#define SPI_BUSCONFIG_FULL_DUP			0
#define SPI_BUSCONFIG_HALF_DUP			1
#define SPI_BUSCONFIG_SIMPLEX_RXONLY	2
/*
 * @SPI_SPEED
 */
#define SPI_SPEED_DIV_2					0
#define SPI_SPEED_DIV_4					1
#define SPI_SPEED_DIV_8					2
#define SPI_SPEED_DIV_16				3
#define SPI_SPEED_DIV_32				4
#define SPI_SPEED_DIV_64				5
#define SPI_SPEED_DIV_128				6

/*
 * @SPI_DATA_FRAME_FORMAT
 */
#define SPI_DFF_8_BITS					0
#define SPI_DFF_16_BITS					1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST_EDGE 			0
#define SPI_CPHA_SECOND_EDGE 			1
/*
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DIS						0
/*
 * @SPI_CPOL
 */
#define SPI_CPOL_ZERO_IDLE				0
#define SPI_CPOL_ONE_IDLE				1


/*******************************************************************
 * 					APIs supported for this driver
 * For more information about APIs check the function definitions
 ******************************************************************/
/*
 * Clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi);
/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t* SPI_Handle);
void SPI_DeInit(SPI_RegDef_t* SPIx);
/*
 * control SSI
 */
void SPI_SSIControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi);
void SPI_SSOEControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi);
uint8_t SPI_GetStatusFlag(SPI_RegDef_t* SPIx, uint32_t flagName);

/*
 * Send and receive data functions
 */
void SPI_SendData(SPI_RegDef_t* SPIx,uint8_t* pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t* SPIx, uint16_t* pTxBuffer, uint32_t length);
/*
 * IRQ configuration and IRQ handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(uint8_t pinNumber);
#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
