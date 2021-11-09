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

/****************************************************************************************************************
 *										 APIs supported by this driver
 * 						For more information about the APIs check the function definitions
 * *************************************************************************************************************/

/**
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/**
 * Unit and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/**
 * Data Send and Receive
 */


/**
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);

/**
 * Other Peripheral Control APIs
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/**
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_I2C_DRIVERS_H_ */
