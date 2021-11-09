/*
 * i2c_drivers.c
 *
 *  Created on: 8 lis 2021
 *      Author: Kacper
 */

#include "i2c_drivers.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecutedAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

/**********************************************************************************************************
 * @fn 				- I2C_GenerateStartCondition
 *
 * #brief 			- This function generate a start condition for master transmission
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/**********************************************************************************************************
 * @fn 				- I2C_ExecutedAddressPhase
 *
 * #brief 			- This function execute address of slave device.
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 * SlaveAddr		- Address of slave device
 *
 *********************************************************************************************************/
static void I2C_ExecutedAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;	// 7 bit address of slave
	SlaveAddr &= ~(1);	// SlaveAdde is Slave address + r/nw bit = 0;
	pI2Cx->DR = SlaveAddr;
}

/**********************************************************************************************************
 * @fn 				- I2C_ClearADDRFlag
 *
 * #brief 			- This function clear the address of slave flag
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

/**********************************************************************************************************
 * @fn 				- I2C_GenerateStopCondition
 *
 * #brief 			- This function generate a stop condition for master transmission
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/**********************************************************************************************************
 * @fn 				- I2C_PeriClockControl
 *
 * #brief 			- This function enables or disables peripheral clock for given I2C port
 * pI2Cx			- Base address of the I2C peripheral
 * EnorDi 			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		} else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else
	{
		if (pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		} else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}

/**********************************************************************************************************
 * @fn 				- RCC_GetPCLK1Value
 *
 * #brief 			- This function to calculated value of PCLK1
 *
 * return			- value of PCLK1
 *
 *********************************************************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;

	uint8_t clcksrc, temp, ahbp, apb1p;

	clcksrc = (RCC->CFGR >> 2) & 0x3;

	if (clcksrc == 0)
	{
		SystemClk = 16000000;
	} else if (clcksrc == 1)
	{
		SystemClk = 8000000;
	} else if (clcksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	// for ahb
	temp = ((RCC->CFGR >> 4) & 0xF);

	if (temp < 8)
	{
		ahbp = 1;
	} else
	{
		ahbp = AHB_PreScaler[temp - 8];
	}

	// for apb1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if (temp < 4)
	{
		apb1p = 1;
	} else
	{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp) / apb1p;

	return pclk1;
}

/**********************************************************************************************************
 * @fn 				- I2C_Init
 *
 * #brief 			- This function initialize structure for specific I2C
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// program the device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	} else
	{
		// mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

/**********************************************************************************************************
 * @fn 				- I2C_DeInit
 *
 * #brief 			- This function reset specific parameters for I2C
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if (pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if (pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_GetFlagStatus
 *
 * #brief 			- This function get a flag form register I2Cx
 * pSPIx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 * FlagName			- Flag name
 *
 *  @return 		- This is SET or RESET macro
 *
 *********************************************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if (pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**********************************************************************************************************
 * @fn 				- I2C_MasterSendData
 *
 * #brief 			- This function to send data by master to slave
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 * pTxbuffer		- Buffer for transmit data
 * Len				- Length of data
 * SlaveAddr		- Address of slave device
 *
 *********************************************************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecutedAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. Send the data until Len becomes 0
	while (Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));	//Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TX=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
