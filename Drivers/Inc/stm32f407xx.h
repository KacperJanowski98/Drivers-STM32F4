/*
 * stm32f407xx.h
 *
 *  Created on: Oct 1, 2021
 *      Author: Kacper Janowski
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/********************************************** START:Processor Specific Details *********************************************/
/*
 *  ARM Cortex Mx Processor NVIC ISERx register Addresses
*/

#define NVIC_ISER0 					( (volatile uint32_t*)0xE000E100 )
#define NVIC_ISER1 					( (volatile uint32_t*)0xE000E104 )
#define NVIC_ISER2 					( (volatile uint32_t*)0xE000E108 )
#define NVIC_ISER3 					( (volatile uint32_t*)0xE000E10C )

/*
 *  ARM Cortex Mx Processor NVIC ICERx register Addresses
*/

#define NVIC_ICER0 					( (volatile uint32_t*)0xE000E180 )
#define NVIC_ICER1 					( (volatile uint32_t*)0xE000E184 )
#define NVIC_ICER2 					( (volatile uint32_t*)0xE000E188 )
#define NVIC_ICER3 					( (volatile uint32_t*)0xE000E18C )

/*
 * ARM Cortex Mx Prosessor Priority Register Address Calculation
*/
#define NVIC_PR_BASE_ADDR 			( (volatile uint32_t*)0xE000E400)

/*
 *  ARM Cortex Mx Processor number of priority bits implemented in Priority Register
*/
#define NO_PR_BITS_IMPLEMENTED						4

/*
 *  Base addresses of Flash and SRAM memories.
*/

#define FLASH_BASEADDR						0x08000000U
#define SRAM1_BASEADDR						0x20000000U
#define SRAM2_BASEADDR 						0x20001C00U
#define ROM_BASEADDR 						0x1FFF0000U 		/*!<System memory> */
#define SRAM 								SRAM1_BASEADDR

/*
 *  AHBx and APBx Bus Peripheral base addresses.
*/

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

/*
 *  Base addresses of peripherals which are hanging on AHB1 bus.
*/

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0X2000)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB1 bus.
*/

#define I2C1_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR 					(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR 				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR 					(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR 					(APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus.
*/

#define SPI1_BASEADDR 					(APB2PERIPH_BASEADDR + 0x3000)
#define USART1_BASEADDR 				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR 				(APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR 					(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR 				(APB2PERIPH_BASEADDR + 0x3800)

/*******************************************peripheral register definition structures*****************************************/

typedef struct
{
	volatile uint32_t MODER;			/*!<GPIO port mode register, 							Address offset: 0x00 */
	volatile uint32_t OTYPER;			/*!<GPIO port output type register, 					Address offset: 0x04 */
	volatile uint32_t OSPEEDR;			/*!<GPIO port output speed register,  					Address offset: 0x08 */
	volatile uint32_t PUPDR;			/*!<GPIO port pull-up/pull-down register,				Address offset: 0x0C */
	volatile uint32_t IDR;				/*!<GPIO port input data register,						Address offset: 0x10 */
	volatile uint32_t ODR;				/*!<GPIO port output data register,						Address offset: 0x14 */
	volatile uint32_t BSRR;				/*!<GPIO port bit set/reset register,					Address offset: 0x18 */
	volatile uint32_t LCKR;				/*!<GPIO port configuration lock register,				Address offset: 0x1C */
	volatile uint32_t AFR[2];			/*!<GPIO alternate function low[0]/high[1] register,	Address offset: 0x20 - 0x24 */
}GPIO_RegDef_t;

/*
 *  peripheral register definition structure for RCC
*/

typedef struct
{
	volatile uint32_t CR;				/*!<RCC clock control register, 									Address offset: 0x00 */
	volatile uint32_t PLLCFGR;			/*!<RCC PLL configuration register, 								Address offset: 0x04 */
	volatile uint32_t CFGR;				/*!<RCC clock configuration register, 								Address offset: 0x08 */
	volatile uint32_t CIR;				/*!<RCC clock interrupt register, 									Address offset: 0x0C */
	volatile uint32_t AHB1RSTR;			/*!<RCC AHB1 peripheral reset register, 							Address offset: 0x10 */
	volatile uint32_t AHB2RSTR;			/*!<RCC AHB2 peripheral reset register, 							Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;			/*!<RCC AHB3 peripheral reset register, 							Address offset: 0x18 */
	uint32_t RESERVED0;					/*!<Reserved, 0x18 */
	volatile uint32_t APB1RSTR;			/*!<RCC APB1 peripheral reset register, 							Address offset: 0x20 */
	volatile uint32_t APB2RSTR;			/*!<RCC APB2 peripheral reset register, 							Address offset: 0x24 */
	uint32_t RESERVED1[2];				/*!<Reserved, 0x28 - 0x2C */
	volatile uint32_t AHB1ENR;			/*!<RCC AHB1 peripheral clock enable register, 						Address offset: 0x30 */
	volatile uint32_t AHB2ENR;			/*!<RCC AHB2 peripheral clock enable register, 						Address offset: 0x34 */
	volatile uint32_t AHB3ENR;			/*!<RCC AHB3 peripheral clock enable register, 						Address offset: 0x38 */
	uint32_t RESERVED2;					/*!<Reserved, 0x3C */
	volatile uint32_t APB1ENR;			/*!<RCC APB1 peripheral clock enable register, 						Address offset: 0x40 */
	volatile uint32_t APB2ENR;			/*!<RCC APB2 peripheral clock enable register, 						Address offset: 0x44 */
	uint32_t RESERVED3[2];				/*!<Reserved, 0x48 - 0x4C */
	volatile uint32_t AHB1LPENR;		/*!<RCC AHB1 peripheral clock enable in low power mode register, 	Address offset: 0x50 */
	volatile uint32_t AHB2LPENR;		/*!<RCC AHB2 peripheral clock enable in low power mode register, 	Address offset: 0x54 */
	volatile uint32_t AHB3LPENR;		/*!<RCC AHB3 peripheral clock enable in low power mode register, 	Address offset: 0x58 */
	uint32_t RESERVED4;					/*!<Reserved, 0x5C */
	volatile uint32_t APB1LPENR;		/*!<RCC APB1 peripheral clock enable in low power mode register,	Address offset: 0x60 */
	volatile uint32_t APB2LPENR;		/*!<RCC APB2 peripheral clock enabled in low power mode register, 	Address offset: 0x64 */
	uint32_t RESERVED5[2];				/*!<Reserved, 0x68 - 0x6C */
	volatile uint32_t BDCR;				/*!<RCC Backup domain control register, 							Address offset: 0x70 */
	volatile uint32_t CSR;				/*!<RCC clock control & status register, 							Address offset: 0x74 */
	uint32_t RESERVED6[2];				/*!<Reserved, 0x78 - 0x7C */
	volatile uint32_t SSCGR;			/*!<RCC spread spectrum clock generation register, 					Address offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;		/*!<RCC PLLI2S configuration register, 								Address offset: 0x84 */
}RCC_RegDef_t;

/*
 * peripheral register definition structure for SPI
*/
typedef struct
{
	volatile uint32_t CR1;				/*!<SPI control register 1, 										Address offset: 0x00*/
	volatile uint32_t CR2;				/*!<SPI control register 2, 										Address offset: 0x04*/
	volatile uint32_t SR;				/*!<SPI status register, 											Address offset: 0x08*/
	volatile uint32_t DR;				/*!<SPI data register, 												Address offset: 0x0C*/
	volatile uint32_t CRCPR;			/*!<SPI CRC polynomial register, 									Address offset: 0x10*/
	volatile uint32_t RXCRCR;			/*!<SPI RX CRC register, 											Address offset: 0x14*/
	volatile uint32_t TXCRCR;			/*!<SPI TX CRC register, 											Address offset: 0x18*/
	volatile uint32_t I2SCFGR;			/*!<SPI_i2S configuration register, 								Address offset: 0x1C*/
	volatile uint32_t I2SPR;			/*!<SPI_I2S prescaler register, 									Address offset: 0x20*/
} SPI_RegDef_t;

/*
 *  peripheral register definition structure for SYSCFG
*/
typedef struct
{
	volatile uint32_t MEMRMP;			/*!<SYSCFG memory remap register, 							Address offset: 0x00        */
	volatile uint32_t PMC;				/*!<SYSCFG peripheral mode configuration register, 			Address offset: 0x04        */
	volatile uint32_t EXTICR[4];		/*!<SYSCFG external interrupt configuration register,  		Address offset: 0x08 - 0x14 */
	uint32_t RESERVED1[2];				/*!<Reserved,												Address offset: 0x18 - 0x0C */
	volatile uint32_t CMPCR;			/*!<Compensation cell control register,						Address offset: 0x20        */
	uint32_t RESERVED2[2];				/*!<Reserved,												Address offset: 0x24 - 0x28 */
	volatile uint32_t CFGR;				/*!<														Address offset: 0x12C	    */
}SYSCFG_RegDef_t;


/*
 *  peripheral register definition structure for EXTI
*/
typedef struct
{
	volatile uint32_t IMR;			/*!<Interrupt mask register, 							Address offset: 0x00 */
	volatile uint32_t EMR;			/*!<Event mask register, 								Address offset: 0x04 */
	volatile uint32_t RTSR;			/*!<Rising trigger selection register,  				Address offset: 0x08 */
	volatile uint32_t FTSR;			/*!<Falling trigger selection register,					Address offset: 0x0C */
	volatile uint32_t SWIER;		/*!<Software interrupt event register,					Address offset: 0x10 */
	volatile uint32_t PRR;			/*!<Pending register,									Address offset: 0x14 */

}EXTI_RegDef_t;

/*
 *  Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
*/

#define GPIOA 		( (GPIO_RegDef_t*) GPIOA_BASEADDR )
#define GPIOB 		( (GPIO_RegDef_t*) GPIOB_BASEADDR )
#define GPIOC 		( (GPIO_RegDef_t*) GPIOC_BASEADDR )
#define GPIOD 		( (GPIO_RegDef_t*) GPIOD_BASEADDR )
#define GPIOE 		( (GPIO_RegDef_t*) GPIOE_BASEADDR )
#define GPIOF 		( (GPIO_RegDef_t*) GPIOF_BASEADDR )
#define GPIOG 		( (GPIO_RegDef_t*) GPIOG_BASEADDR )
#define GPIOH 		( (GPIO_RegDef_t*) GPIOH_BASEADDR )
#define GPIOI 		( (GPIO_RegDef_t*) GPIOI_BASEADDR )

#define RCC 		( (RCC_RegDef_t*) RCC_BASEADDR )

#define EXTI		( (EXTI_RegDef_t*) EXTI_BASEADDR )

#define SYSCFG		( (SYSCFG_RegDef_t*) SYSCFG_BASEADDR )

#define SPI1		( (SPI_RegDef_t*)SPI1_BASEADDR )
#define SPI2		( (SPI_RegDef_t*)SPI2_BASEADDR )
#define SPI3		( (SPI_RegDef_t*)SPI3_BASEADDR )

/*
 * Clock Enable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_EN()		( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()		( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()		( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()		( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()		( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()		( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()		( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()		( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()		( RCC->AHB1ENR |= (1 << 8) )

/*
 *  Clock Enable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_EN() 		( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() 		( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN() 		( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx peripherals
*/

#define SPI1_PCLK_EN() 		( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() 		( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() 		( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN() 		( RCC->APB2ENR |= (1 << 13) )

/*
 * Clock Enable Macros for USARTx peripherals
*/

#define USART2_PCLK_EN() 		( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() 		( RCC->APB1ENR |= (1 << 17) )

#define USART1_PCLK_EN() 		( RCC->APB2ENR |= (1 << 4) )
#define USART6_PCLK_EN() 		( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock Enable Macros for UARTx peripherals
*/

#define UART4_PCLK_EN() 		( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN() 		( RCC->APB1ENR |= (1 << 20) )

/*
 * Clock Enable Macros for SYSCFG peripheral
*/

#define SYSCFG_PCLK_EN() 		( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
*/

#define GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()		( RCC->AHB1ENR &= ~(1 << 8) )

/*
 *  Clock Disable Macros for I2Cx peripherals
*/

#define I2C1_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
*/

#define SPI1_PCLK_DI() 		( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI() 		( RCC->APB2ENR &= ~(1 << 13) )

/*
 * Clock Disable Macros for USARTx peripherals
*/

#define USART2_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 17) )

#define USART1_PCLK_DI() 		( RCC->APB2ENR &= ~(1 << 4) )
#define USART6_PCLK_DI() 		( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock Disable Macros for UARTx peripherals
*/

#define UART4_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI() 		( RCC->APB1ENR &= ~(1 << 20) )

/*
 * Clock Disable Macros for SYSCFG peripheral
*/

#define SYSCFG_PCLK_DI() 		( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
*/

#define GPIOA_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 0) );		( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 1) );		( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 2) );		( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 3) );		( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 4) );		( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 5) );		( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 6) );		( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 7) );		( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 8) );		( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)

/*
 *  Returns port code for given GPIOx base address
*/

#define GPIO_BASEADDR_TO_CODE(x)			( (x == GPIOA) ?0:\
												(x == GPIOB) ?1:\
												(x == GPIOC) ?2:\
												(x == GPIOA) ?3:\
												(x == GPIOB) ?4:\
												(x == GPIOC) ?5:\
												(x == GPIOA) ?6:\
												(x == GPIOB) ?7:0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
*/

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 * macros for all the possible priority levels
*/

#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15
// Some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET

#include "gpio_drivers.h"

#endif /* INC_STM32F407XX_H_ */
