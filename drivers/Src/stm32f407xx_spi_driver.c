/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jul 22, 2023
 *      Author: Admin
 */
#include "stm32f407xx_spi_driver.h"
/***********************************************************
 *************** SUPPORT FUNCTION **************************
 **********************************************************/
#define SPI_FRE_FLAG		( 1 << 8 )
#define SPI_BSY_FLAG		( 1 << 7 )
#define SPI_OVR_FLAG		( 1 << 6 )
#define SPI_MODDF_FLAG		( 1 << 5 )
#define SPI_TXE_FLAG		( 1 << 1 )
#define SPI_RXNE_FLAG		( 1 << 0 )
uint8_t SPI_GetStatusFlag(SPI_RegDef_t* SPIx, uint32_t flagName){
	if(SPIx->SPI_SR & flagName) return FLAG_ENABLE;
	return FLAG_DISABLE;
};

/***********************************************************
 *************** MAIN FUNCTION *****************************
 **********************************************************/
/*
 * Clock control
 */

/*
 * Peripheral clock setup
 */

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(SPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(SPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(SPIx == SPI3){
			SPI3_PCLK_EN();
		}else if(SPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else{
		if(SPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(SPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(SPIx == SPI3){
			SPI3_PCLK_DI();
		}else if(SPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
};

/*
 * Init and De-init
 */

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_Init(SPI_Handle_t* SPI_Handle){
	uint32_t tempreg = 0;
	//1. Configure the mode of device
	tempreg |= (SPI_Handle->SPI_Config.SPI_deviceMode << 2);
	//2. Configure the bus
	if(SPI_Handle->SPI_Config.SPI_busConfig == SPI_BUSCONFIG_HALF_DUP){
		tempreg |= (1 << 15);
	}else if(SPI_Handle->SPI_Config.SPI_busConfig == SPI_BUSCONFIG_FULL_DUP){
		tempreg &= ~(1 << 15);
	}else if(SPI_Handle->SPI_Config.SPI_busConfig == SPI_BUSCONFIG_SIMPLEX_RXONLY){
		tempreg |= (1 << 15);
		tempreg |= (1 << 10);
	}
	//3. Set the value of data frame format
	tempreg |= (SPI_Handle->SPI_Config.SPI_DFF << 11);
	//4. Set the divider for baud rate clock
	tempreg |= (SPI_Handle->SPI_Config.SPI_speed << 3);
	//5. Set the clock phase
	tempreg |= (SPI_Handle->SPI_Config.SPI_CPHA << 0);
	//6. Set the clock polarity
	tempreg |= (SPI_Handle->SPI_Config.SPI_CPOL << 1);
	//7. Enable/disable software selection management
	tempreg |= (SPI_Handle->SPI_Config.SPI_SSM << 9);
	SPI_Handle->SPIx->SPI_CR1 = tempreg;
};

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_DeInit(SPI_RegDef_t* SPIx){
	if(SPIx == SPI1){
		SPI1_REG_RESET();
	}else if(SPIx == SPI2){
		SPI2_REG_RESET();
	}else if(SPIx == SPI3){
		SPI3_REG_RESET();
	}else if(SPIx == SPI4){
		SPI4_REG_RESET();
	}
};

void SPI_SSIControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) SPIx->SPI_CR1 |= (1 << 8);
	else if(EnOrDi == DISABLE) SPIx->SPI_CR1 &= ~(1 << 8);
}

void SPI_SSOEControl(SPI_RegDef_t* SPIx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) SPIx->SPI_CR2 |= (1 << 2);
	else if(EnOrDi == DISABLE) SPIx->SPI_CR2 &= ~(1 << 2);
};
/*
 * Send and receive data functions
 */

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_SendData(SPI_RegDef_t* SPIx,uint8_t* pTxBuffer, uint32_t length){
	while(length >0){
		while( SPI_GetStatusFlag(SPIx, SPI_TXE_FLAG) == FLAG_DISABLE );
		if( SPIx->SPI_CR1 & ( 1 << 11 ) ){
			SPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}else{
			SPIx->SPI_DR = *(pTxBuffer);
			length--;
			pTxBuffer++;
		}
	}
};

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_ReceiveData(SPI_RegDef_t* SPIx, uint16_t* pTxBuffer, uint32_t length){

};
/*
 * IRQ configuration and IRQ handling
 */

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/***********************************************************
 * @fn					-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @Note:				-
 *
 */
void SPI_IRQHandling(uint8_t pinNumber);

