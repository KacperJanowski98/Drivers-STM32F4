/*
 * i2c_drivers.h
 *
 *  Created on: 8 lis 2021
 *      Author: Kacper
 */

#ifndef INC_I2C_DRIVERS_H_
#define INC_I2C_DRIVERS_H_

#include "stm32f407xx.h"

/**
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;
} I2C_Config_t;

/**
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;

#endif /* INC_I2C_DRIVERS_H_ */
