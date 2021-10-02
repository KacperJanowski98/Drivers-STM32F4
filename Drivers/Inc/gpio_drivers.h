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


#endif /* INC_GPIO_DRIVERS_H_ */
