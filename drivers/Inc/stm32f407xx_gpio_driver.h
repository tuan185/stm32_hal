/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jul 11, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_
#include "stm32f407xx.h"

/*
 * Configuration structure for GPIOx pin
 */
typedef struct
{
	uint8_t GPIO_pinNumber;				//GPIO number
	uint8_t GPIO_pinMode;				//GPIO mode	 - possible values from @GPIO_PIN_MODES
	uint8_t GPIO_pinSpeed;				//GPIO speed - possible values from @GPIO_PIN_SPEED
	uint8_t GPIO_pinPuPdControl;		//GPIO pull-up/pull-down - possible values from @GPIO_PDPDCONTROL
	uint8_t GPIO_pinOPType;				//GPIO type of output - possible value from @GPIO_PIN_OPTYPE
	uint8_t GPIO_altFuncMode;			//GPIO alternative function mode
}GPIO_PinConfig_t;

/*
 * Handle structure for GPIOx pin
 */
typedef struct
{
	GPIO_RegDef_t* pGPIOx;				//base address of GPIOx peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	//Pin configuration settings for GPIOx peripheral
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0		//input mode
#define GPIO_MODE_OUT		1		//output mode
#define GPIO_MODE_ALTFN		2		//alternative function mode
#define GPIO_MODE_ANALOG	3		//analog mode
#define GPIO_MODE_IT_FT		4		//falling edge mode
#define GPIO_MODE_IT_RT		5		//rising edge mode
#define GPIO_MODE_IT_FRT	6		//falling and rising trigger

/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output mode
 */
#define GPIO_OP_PP			0		//output push-pull
#define GPIO_OP_OD			1		//output open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speed
 */
#define GPIO_SPEED_LOW		0		//Low speed
#define GPIO_SPEED_MEDIUM	1		//medium speed
#define GPIO_SPEED_FAST		2		//high speed
#define GPIO_SPEED_HIGH		3		//very high speed

/*
 * @GPIO_PDPDCONTROL
 * GPIO pin possible pull up/pull down configuration macros
 */
#define GPIO_NO_PUPD		0		//no pull up or pull down
#define GPIO_PU				1		//pull up mode
#define GPIO_PD				2		//pull down mode

/*******************************************************************
 * 					APIs supported for this driver
 * For more information about APIs check the function definitions
 ******************************************************************/

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t enOrDi);

/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOuputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/*
 * IRQ configuration and IRQ handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t enOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
