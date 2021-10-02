/*
 * gpio_drivers.h
 *
 *  Created on: 2 paź 2021
 *      Author: Kacper Janowski
 */

#ifndef INC_GPIO_DRIVERS_H_
#define INC_GPIO_DRIVERS_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;					/*!< Possible value from @GPIO_PIN_NUMBER > */
	uint8_t GPIO_PinMode; 					/*!< Possible value from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinSpeed;					/*!< Possible value from @GPIO_PIN_SPEED > */
	uint8_t GPIO_PinPuPdControl;			/*!< Possible value from @GPIO_PIN_PULL_UP_DOWN > */
	uint8_t GPIO_PinOPType;					/*!< Possible value from @GPIO_PIN_OUTPUT_PULL_UP_DOWN > */
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/**
 * This is Handle structure for a GPIO pin
*/
typedef struct
{
	GPIO_RegDef_t *pGPIOx;					/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;		/*!< This holds GPIO pin configuration settings >*/
}GPIO_Handle_t;

/*
 *  @GPIO_PIN_NUMBER
 *  GPIO pin numbers
*/

#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/*
 *  @GPIO_PIN_MODES
 *  GPIO pin possible modes
 */

#define GPIO_MODE_IN 				0 // input
#define GPIO_MODE_OUT 				1 // output
#define GPIO_MODE_ALTFN 			2 // alternative function
#define GPIO_MODE_ANALOG 			3 // analog
#define GPIO_MODE_IT_FT				4 // interrupt falling edge trigger
#define GPIO_MODE_IT_RT				5 // interrupt raising edge trigger
#define GPIO_MODE_IT_RFT			6 // interrupt raising/falling edge trigger

/*
 *  @GPIO_PIN_OUTPUT_PULL_UP_DOWN
 *  GPIO pin possible output types
*/

#define GPIO_OP_TYPE_PP  			0 // output push pull
#define GPIO_OP_TYPE_OP  			1 // output open drain

/*
 *  @GPIO_PIN_SPEED
 *  GPIO pin possible output speeds
*/

#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIOM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_HIGH				3

/*
 *   @GPIO_PIN_PULL_UP_DOWN
 *  GPIO pin pull up pull down configuration macros
*/

#define GPIO_NO_PUPD				0 // No pull-up, pull-down
#define GPIO_PIN_PU					1 // pull-up
#define GPIO_PIN_PD					2 // pull-down

/***************************************************************************************************
 * 									APIs supported by this driver
 * 				For more information about the APIs check the function definitions
 ***************************************************************************************************/

/*
 * Peripheral Clock setup
*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *  Data read and write
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_GPIO_DRIVERS_H_ */
