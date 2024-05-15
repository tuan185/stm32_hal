/*
 * stm32f407xx.h
 *
 *  Created on: Jun 29, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

/*
 * define data type macros
 */
#define __vo volatile								//define the volatile keyword

/*
 * define some boolean data macros
 */
#define SET					1
#define RESET 				0
#define ENABLE				SET
#define DISABLE				RESET
#define GPIO_SET_VALUE		SET
#define GPIO_RESET_VALUE 	RESET
#define FLAG_ENABLE			SET
#define FLAG_DISABLE		RESET

/*
 * return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)(	(x == GPIOA)? 0:\
									(x == GPIOB)? 1:\
									(x == GPIOC)? 2:\
									(x == GPIOD)? 3:\
									(x == GPIOE)? 4:\
									(x == GPIOF)? 5:\
									(x == GPIOF)? 6:\
									(x == GPIOF)? 7:0 )

/*
 * IRQ number of STM32F407xx MCU
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI5_9				23
#define IRQ_NO_EXTI10_15			40

/*define the base address of memory*/
#define FLASH_BASEADDR				0x08000000U 	//base address of flash memory
#define ROM_BASEADDR				0x1FFF0000U 	//base address of ROM
#define SRAM_BASEADDR				0x20000000U 	//base address of SRAM
#define SRAM1_BASEADDR				0x20000000U 	//base address of SRAM1
#define SRAM2_BASEADDR				0x2001C000U 	//base address of SRAM2

/*define the base address of bus*/
#define APB1_PERIPH_BASEADDR		0x40000000U 	//base address of APB1 bus
#define APB2_PERIPH_BASEADDR		0x40010000U		//base address of APB2 bus
#define AHB1_PERIPH_BASEADDR		0x40020000U 	//base address of AHB1 bus
#define AHB2_PERIPH_BASEADDR		0x50000000U 	//base address of AHB2 bus

/*define the base address of AHB1 peripherals*/
#define GPIOA_BASEADDR				0x40020000U 	//base address of GPIOA
#define GPIOB_BASEADDR				0x40020400U 	//base address of GPIOB
#define GPIOC_BASEADDR				0x40020800U 	//base address of GPIOC
#define GPIOD_BASEADDR				0x40020C00U 	//base address of GPIOD
#define GPIOE_BASEADDR				0x40021000U 	//base address of GPIOE
#define GPIOF_BASEADDR				0x40021400U 	//base address of GPIOF
#define GPIOG_BASEADDR				0x40021800U 	//base address of GPIOG
#define GPIOH_BASEADDR				0x40021C00U 	//base address of GPIOH
#define GPIOI_BASEADDR				0x40022000U 	//base address of GPIOI
#define RCC_BASEADDR				0x40023800U		//base address of RCC register

/*define the base address of APB1 peripherals*/
#define I2C1_BASEADDR				0x40005400U		//base address of I2C1
#define I2C2_BASEADDR				0x40005800U		//base address of I2C2
#define I2C3_BASEADDR				0x40005C00U		//base address of I2C3
#define SPI2_BASEADDR				0x40003800U		//base address of SPI2
#define SPI3_BASEADDR				0x40003C00U		//base address of SPI3
#define USART2_BASEADDR				0x40004800U		//base address of USART2
#define USART3_BASEADDR				0x40004400U		//base address of USART3
#define UART4_BASEADDR				0x40004C00U		//base address of UART4
#define UART5_BASEADDR				0x40005000U		//base address of UART5

/*define the base address of APB2 peripherals*/
#define SPI1_BASEADDR				0x40013000U		//base address of SPI1
#define SPI4_BASEADDR				0x40013400U		//base address of SPI4
#define USART1_BASEADDR				0x40011000U		//base address of USART1
#define USART6_BASEADDR				0x40011400U		//base address of USART6
#define EXTI_BASEADDR				0x40013C00U		//base address of EXTI
#define SYSCFG_BASEADDR				0x40013800U		//base address of SYSCFG

/*define the GPIOx register definition structure*/
typedef struct{
	__vo uint32_t MODER;							//GPIOx port mode register											offset 0x00
	__vo uint32_t OTYER;							//GPIOx port output type register									offset 0x04
	__vo uint32_t OSPEEDR;							//GPIOx port output speed register									offset 0x08
	__vo uint32_t PUPDR;							//GPIOx port pull-up/pull-down register								offset 0x0C
	__vo uint32_t IDR;								//GPIOx port input data register									offset 0x10
	__vo uint32_t ODR;								//GPIOx port output data register									offset 0x14
	__vo uint32_t BSRR;								//GPIOx port bit set/reset register									offset 0x18
	__vo uint32_t LCKR;								//GPIOx port configuration lock register							offset 0x1C
	__vo uint32_t AFR[2];							//GPIOx port alternate function low(AFR[0])/high(AFR[1]) register	offset 0x20/0x24
}GPIO_RegDef_t;

/*define the EXTI register definition structure*/
typedef struct{
	__vo uint32_t IMR;								//Interrupt mask register											offset 0x00
	__vo uint32_t EMR;								//Event mask register												offset 0x04
	__vo uint32_t RTSR;								//Rising trigger selection register									offset 0x08
	__vo uint32_t FTSR;								//Falling trigger selection register								offset 0x0C
	__vo uint32_t SWIER;							//Software interrupt event register									offset 0x10
	__vo uint32_t PR;								//Pending register													offset 0x14
}EXTI_RegDef_t;

/*define the SYCFG register definition structure*/
typedef struct{
	__vo uint32_t MEMRMP;							//memory remap register												offset 0x00
	__vo uint32_t PMC;								//peripheral mode configuration register							offset 0x04
	__vo uint32_t EXTICR[4];						//external interrupt configuration register 1->4					offset 0x08->0x14
	__vo uint32_t Reserved[2];						//Reserved															offset 0x18/1C
	__vo uint32_t CMPCR;							//Compensation cell control register								offset 0x20
}SYSCFG_RegDef_t;

/*define the RCC register definition structure*/
typedef struct{
	__vo uint32_t CR;								//RCC clock control register										offset 0x00
	__vo uint32_t PLLCFGR;							//RCC PLL configuration register									offset 0x04
	__vo uint32_t CFGR;								//RCC clock configuration register									offset 0x08
	__vo uint32_t CIR;								//RCC clock interrupt register										offset 0x0C
	__vo uint32_t AHB1RSTR;							//RCC AHB1 peripheral reset register								offset 0x10
	__vo uint32_t AHB2RSTR;							//RCC AHB2 peripheral reset register								offset 0x14
	__vo uint32_t AHB3RSTR;							//RCC AHB3 peripheral reset register								offset 0x18
	__vo uint32_t Reserved_0;						//Reserved															offset 0x1C
	__vo uint32_t APB1RSTR;							//RCC APB1 peripheral reset register								offset 0x20
	__vo uint32_t APB2RSTR;							//RCC APB2 peripheral reset register								offset 0x24
	__vo uint32_t Reserved_1[2];					//Reserved															offset 0x28/2C
	__vo uint32_t AHB1ENR;							//RCC AHB1 peripheral clock enable register							offset 0x30
	__vo uint32_t AHB2ENR;							//RCC AHB2 peripheral clock enable register							offset 0x34
	__vo uint32_t AHB3ENR;							//RCC AHB3 peripheral clock enable register							offset 0x38
	__vo uint32_t Reserved_2;						//Reserved															offset 0x3C
	__vo uint32_t APB1ENR;							//RCC APB1 peripheral clock enable register							offset 0x40
	__vo uint32_t APB2ENR;							//RCC APB2 peripheral clock enable register							offset 0x44
	__vo uint32_t Reserved_3[2];					//Reserved															offset 0x48/4C
	__vo uint32_t AHB1LPENR;						//RCC AHB1 peripheral clock enable in low power mode register		offset 0x50
	__vo uint32_t AHB2LPENR;						//RCC AHB2 peripheral clock enable in low power mode register		offset 0x54
	__vo uint32_t AHB3LPENR;						//RCC AHB3 peripheral clock enable in low power mode register		offset 0x58
	__vo uint32_t Reserved_4;						//Reserved															offset 0x5C
	__vo uint32_t APB1LPENR;						//RCC APB1 peripheral clock enable in low power mode register		offset 0x60
	__vo uint32_t APB2LPENR;						//RCC APB2 peripheral clock enabled in low power mode register		offset 0x64
	__vo uint32_t Reserved_5[2];					//Reserved															offset 0x68/6C
	__vo uint32_t BDCR;								//RCC Backup domain control register								offset 0x70
	__vo uint32_t CSR;								//RCC clock control & status register								offset 0x74
	__vo uint32_t Reserved_6[2];					//Reserved															offset 0x78/7C
	__vo uint32_t SSCGR;							//RCC spread spectrum clock generation register						offset 0x80
	__vo uint32_t PLLI2SCFGR;						//RCC PLLI2S configuration register									offset 0x84
}RCC_RegDef_t;

/*
 * define the SPI registers definition
 */
typedef struct{
	__vo uint32_t SPI_CR1;							//SPI control register 1											offset 0x00
	__vo uint32_t SPI_CR2;							//SPI control register 2											offset 0x04
	__vo uint32_t SPI_SR;							//SPI status register												offset 0x08
	__vo uint32_t SPI_DR;							//SPI data register													offset 0x0C
	__vo uint32_t SPI_CRCPR;						//SPI CRC polynomial register										offset 0x10
	__vo uint32_t SPI_RXCRCR;						//SPI RX CRC register												offset 0x14
	__vo uint32_t SPI_TXCRCR;						//SPI TX CRC register												offset 0x18
	__vo uint32_t SPI_I2SCFGR;						//SPI_I2S configuration register									offset 0x1C
	__vo uint32_t SPI_I2SPR;						//SPI_I2S configuration register									offset 0x20
}SPI_RegDef_t;

/*
 * define the peripheral definitions
 */
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 0 ))
#define GPIOB_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 1 ))
#define GPIOC_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 2 ))
#define GPIOD_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 3 ))
#define GPIOE_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 4 ))
#define GPIOF_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 5 ))
#define GPIOG_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 6 ))
#define GPIOH_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 7 ))
#define GPIOI_PCLK_EN() 	(RCC->AHB1ENR |= ( 1 << 8 ))
/*
 * Clock enable macro for SYSCFG
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= ( 1 << 14 ))

/*
 * macros for GPIOx reset register
 */
#define GPIOA_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET() 	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 21 ))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 22 ))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= ( 1 << 23 ))

/*
 * Clock enable macros for USARTx/UARTx peripherals
 */
#define USART1_PLCK_EN()	(RCC->APB1ENR |= ( 1 << 4))
#define USART2_PLCK_EN()	(RCC->APB1ENR |= ( 1 << 17))
#define USART3_PLCK_EN()	(RCC->APB1ENR |= ( 1 << 18))
#define UART4_PLCK_EN()		(RCC->APB1ENR |= ( 1 << 19))
#define UART5_PLCK_EN()		(RCC->APB1ENR |= ( 1 << 20))
#define USART6_PLCK_EN()	(RCC->APB1ENR |= ( 1 << 5))

/*
 * Clock enable macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()			(RCC->APB2ENR |= (1 << 13))

/*
 * reset macros for SPIx
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2ENR |= (1 << 12)); (RCC->APB2ENR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1ENR |= (1 << 14)); (RCC->APB1ENR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1ENR |= (1 << 15)); (RCC->APB1ENR &= ~(1 << 15)); } while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2ENR |= (1 << 13)); (RCC->APB2ENR &= ~(1 << 13)); } while(0)

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 0 ))
#define GPIOB_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 1 ))
#define GPIOC_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 2 ))
#define GPIOD_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 3 ))
#define GPIOE_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 4 ))
#define GPIOF_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 5 ))
#define GPIOG_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 6 ))
#define GPIOH_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 7 ))
#define GPIOI_PCLK_DI() 	(RCC->AHB1ENR &= ~( 1 << 8 ))

/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~( 1 << 23 ))

/*
 * Clock disable macros for USARTx/UARTx peripherals
 */
#define USART1_PLCK_DI()	(RCC->APB1ENR &= ~( 1 << 4))
#define USART2_PLCK_DI()	(RCC->APB1ENR &= ~( 1 << 17))
#define USART3_PLCK_DI()	(RCC->APB1ENR &= ~( 1 << 18))
#define UART4_PLCK_DI()		(RCC->APB1ENR &= ~( 1 << 19))
#define UART5_PLCK_DI()		(RCC->APB1ENR &= ~( 1 << 20))
#define USART6_PLCK_DI()	(RCC->APB1ENR &= ~( 1 << 5))

/*
 * Clock disable macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 13))
/*
 * Clock disable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 14))


/********************************************************************
 ******* THIS SECTION INCLUDED MACROS FOR PROCESSOR *****************
 ********************** CORTEX-M4 DEVICES****************************
 *******************************************************************/
/*
 * macros for NVIC set enable registers base address
 */
#define NVIC_ISER0					(__vo uint32_t*)0xE000E100
#define NVIC_ISER1					(__vo uint32_t*)0xE000E104
#define NVIC_ISER2					(__vo uint32_t*)0xE000E108
#define NVIC_ISER3					(__vo uint32_t*)0xE000E10C
#define NVIC_ISER4					(__vo uint32_t*)0xE000E110
#define NVIC_ISER5					(__vo uint32_t*)0xE000E114
#define NVIC_ISER6					(__vo uint32_t*)0xE000E118
#define NVIC_ISER7					(__vo uint32_t*)0xE000E11C

/*
 * macros for NVIC clear enable registers base address
 */
#define NVIC_ICER0					(__vo uint32_t*)0XE000E180
#define NVIC_ICER1					(__vo uint32_t*)0XE000E184
#define NVIC_ICER2					(__vo uint32_t*)0XE000E188
#define NVIC_ICER3					(__vo uint32_t*)0XE000E18C
#define NVIC_ICER4					(__vo uint32_t*)0XE000E190
#define NVIC_ICER5					(__vo uint32_t*)0XE000E194
#define NVIC_ICER6					(__vo uint32_t*)0XE000E198
#define NVIC_ICER7					(__vo uint32_t*)0XE000E19C

/*
 * macros for interrupt priority registers base address
 */
#define NVIC_IPR_BASEADDR			(__vo uint32_t*)0xE000E400

#define NON_IMPLEMENT_LO_BITS		4

#endif /* INC_STM32F407XX_H_ */
