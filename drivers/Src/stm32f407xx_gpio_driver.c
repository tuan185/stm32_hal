/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jul 11, 2023
 *      Author: Admin
 */

#include "stm32f407xx_gpio_driver.h"
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
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}else if(enOrDi == DISABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}
;

/*
 * Init and de-init
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0; //temp register
	//1.configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode == GPIO_MODE_IT_RT){
			//1. configure the RTSR & FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode == GPIO_MODE_IT_FT){
			//1. configure the RTSR & FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_pinMode == GPIO_MODE_IT_FRT){
			//1. configure the RTSR & FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		}
		//2. configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t code = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber%4;
		SYSCFG->EXTICR[temp1] |= code << (4 * temp2);

		//3. enable EXTI delivery
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);


	}
	//2.configure the speed
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinSpeed <= GPIO_SPEED_HIGH){
		//non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	}
	//3.configure the pull-up/pull-down
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl <= GPIO_PD){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber));
		pGPIOHandle->pGPIOx->PUPDR |= temp;
	}
	//4.configure the output type
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_pinOPType <= GPIO_OP_OD){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_pinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber);
		pGPIOHandle->pGPIOx->OTYER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber));
		pGPIOHandle->pGPIOx->OTYER |= temp;
	}
	//5.configure the alternative functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_altFuncMode <= 15){
		uint8_t pin = pGPIOHandle->GPIO_PinConfig.GPIO_pinNumber;
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_altFuncMode << (4 * (pin%8));
		pGPIOHandle->pGPIOx->AFR[pin/8] &= ~(0xF << (4 * pin%8));
		pGPIOHandle->pGPIOx->AFR[pin/8] |= temp;
	}
};

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
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
};

/*
 * Read and write data
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
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber){
	uint8_t data = ((pGPIOx->IDR >> pinNumber) & 0x00000001U);
	return data;
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
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx){
	uint16_t data = (uint16_t)pGPIOx->IDR;
	return data;
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
void GPIO_WriteToOuputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value){
	if(value == GPIO_SET_VALUE){
		pGPIOx->ODR |= (1 << pinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << pinNumber);
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
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value){
	pGPIOx->ODR = value;
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
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber){
	pGPIOx->ODR ^= (1 << pinNumber);
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi){
	if(enOrDi == ENABLE){
		if(IRQNumber >= 0 && IRQNumber < 32){
			(*NVIC_ISER0) |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			(*NVIC_ISER1) |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			(*NVIC_ISER2) |= (1 << (IRQNumber % 64));
		}
	}else{
		if(IRQNumber >= 0 && IRQNumber < 32){
			(*NVIC_ICER0) |= (1 << IRQNumber);
		}else if(IRQNumber >= 32 && IRQNumber < 64){
			(*NVIC_ICER1) |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96){
			(*NVIC_ICER2) |= (1 << (IRQNumber % 64));
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t irqx = IRQNumber / 4;
	uint8_t irqx_offset = IRQNumber % 4;
	uint8_t shift_amount = (irqx_offset * 8) + (8 - NON_IMPLEMENT_LO_BITS);
	*(NVIC_IPR_BASEADDR + (irqx * 4)) |= (IRQPriority << shift_amount);
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
void GPIO_IRQHandling(uint8_t pinNumber){
	if(EXTI->PR & (1 << pinNumber)){
		EXTI->PR |= (1 << pinNumber);
	}
};

