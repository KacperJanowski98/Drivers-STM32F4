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

/**
 * @I2C_SCLSpeed
 */
#define  I2C_SCL_SPEED_SM		100000
#define  I2C_SCL_SPEED_FM4K		400000
#define  I2C_SCL_SPEED_FM2K		200000

/**
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/**
 * @I2C_FMDutyCycle
 */
#define I2C_SFM_DUTY_2			0
#define I2C_SFM_DUTY_16_9		1

#endif /* INC_I2C_DRIVERS_H_ */
