/*
 * gpio_driver.c
 *
 *  Created on: 2 paź 2021
 *      Author: Kacper Janowski
 */

#include "gpio_drivers.h"

/**********************************************************************************************************
 * @fn 				- GPIO_PeriClockControl
 *
 * #brief 			- This function enables or disables peripheral clock for given GPIO port
 * pGPIOx			- Base address of the GPIO peripheral
 * EnorDi 			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/**********************************************************************************************************
 * @fn 				- GPIO_Init
 *
 * #brief 			- This function set specific parameters for GPIOx (Mode, Speed, input/output etc.)
 * pGPIOHandle		- This is structure which contain: base address of the gpio peripheral and pin configuration
 *
 *********************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temp. register

	//1. Configure the mode of GPIO Pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear
		pGPIOHandle->pGPIOx->MODER |= temp; // set
	} else
	{
		// interrupt mode
	}

	temp = 0;

	//2. Configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // set

	temp = 0;

	//3. Configure the pudp settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear
	pGPIOHandle->pGPIOx->PUPDR |= temp; // set

	temp = 0;

	//4. Configure the optype

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear
	pGPIOHandle->pGPIOx->OTYPER |= temp; // set

	temp = 0;

	//5. Configure the alt functionality

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function register.
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		// defined register
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		// defined number of pin
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); 	// clear
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // set
	}

}

/**********************************************************************************************************
 * @fn 				- GPIO_DeInit
 *
 * #brief 			- This function reset specific parameters for GPIOx (Mode, Speed, input/output etc.)
 * pGPIOx		- This is structure which contain: base address of the gpio peripheral and pin configuration
 *
 *********************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

/**********************************************************************************************************
 * @fn 				- GPIO_ReadFromInputPin
 *
 * #brief 			- This function read data from pin
 * pGPIOx			- This is structure which contain: base address of the gpio peripheral and pin configuration
 * PinNumber 		- Pin from we are reading data, this is pin number like in input data register
 *
 * @return 			-	This is 0 or 1

 *
 *********************************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**********************************************************************************************************
 * @fn 				- GPIO_ReadFromInputPort
 *
 * #brief 			- This function read data from entire port
 * pGPIOx			- This is structure which contain: base address of the gpio peripheral and pin configuration
 *
 * @return 			-	This is 0 or 1
 *
 *********************************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/**********************************************************************************************************
 * @fn 				- GPIO_WriteToOutputPin
 *
 * #brief 			- This function write Value in the specific pin (PinNumber)
 * pGPIOx			- Base address of the gpio peripheral
 * PinNumber 		- Pin where we want to write data, this is pin number like in output data register
 * Value 			- Value to write in specific register it can be 0 or 1
 *
 *********************************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else
	{
		// Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**********************************************************************************************************
 * @fn 				- GPIO_WriteToOutputPort
 *
 * #brief 			- This function write the Value in the entire Output data register
 * pGPIOx			- Base address of the gpio peripheral
 * Value			- Value to write in register it can be 0 or 1
 *
 *********************************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**********************************************************************************************************
 * @fn 				- GPIO_ToggleOutputPin
 *
 * #brief 			- This function toggle pin (PinNumer)
 * @pGPIOx			- Base address of the gpio peripheral
 * @PinNumber 		- Pin number
 *
 *********************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


