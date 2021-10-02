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
