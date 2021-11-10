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
static void I2C_ExecutedAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecutedAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

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
 * @fn 				- I2C_ExecutedAddressPhaseWrite
 *
 * #brief 			- This function execute address of slave device into write mode.
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 * SlaveAddr		- Address of slave device
 *
 *********************************************************************************************************/
static void I2C_ExecutedAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;	// 7 bit address of slave
	SlaveAddr &= ~(1);	// SlaveAdde is Slave address + r/nw bit = 0;
	pI2Cx->DR = SlaveAddr;
}

/**********************************************************************************************************
 * @fn 				- I2C_ExecutedAddressPhaseRead
 *
 * #brief 			- This function execute address of slave device into read moede.
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 * SlaveAddr		- Address of slave device
 *
 *********************************************************************************************************/
static void I2C_ExecutedAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;	// 7 bit address of slave
	SlaveAddr |= 1;	// SlaveAdde is Slave address + r/nw bit = 1;
	pI2Cx->DR = SlaveAddr;
}

/**********************************************************************************************************
 * @fn 				- I2C_ClearADDRFlag
 *
 * #brief 			- This function clear the address of slave flag
 * pI2Cx			- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	// Check for device mode
	if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				// first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// Clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;	// not create an error related with not used variable
			}
		} else
		{
			// Clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;	// not create an error related with not used variable
		}
	}else
	{
		//device is in slave mode
		// Clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;	// not create an error related with not used variable
	}
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

	// TRISE Configuration
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	} else
	{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->CCR = (tempreg & 0x3F);
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
	I2C_ExecutedAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR flag according to its software sequence
	I2C_ClearADDRFlag(pI2CHandle);

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

/**********************************************************************************************************
 * @fn 				- I2C_MasterReceiveData
 *
 * #brief 			- This function to send data by master to slave
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 * pRxbuffer		- Buffer for receive data
 * Len				- Length of data
 * SlaveAddr		- Address of slave device
 *
 *********************************************************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecutedAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	// Procedure to read only 1 byte from slave
	if (Len == 1)
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// Generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

		return;
	}

	// Procedure to read data from slave when Len > 1
	if (Len > 1)
	{
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the data until Len becomes zero
		for (uint32_t i = Len; i > 0; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2)	// if last 2 bytes are remaining
			{
				// Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read the data from data register into buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxbuffer++;	// increment the buffer address
		}
	}

	// re-enable ACKing
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_IRQInterruptConfig
 *
 * #brief 			-
 * IRQNumber		- Number interrupt register which you would like to enable or disable
 * EnorDi	 		- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Program ISER2 register
			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	} else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER3 |= (1 << IRQNumber % 64);
		}
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_IRQPriorityConfig
 *
 * #brief 			-
 * IRQNumber		- Number interrupt register
 * IRQPriority 		- Number of priority
 *
 *********************************************************************************************************/
void I2C_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**********************************************************************************************************
 * @fn 				- I2C_MasterSendDataIT
 *
 * #brief 			- This function to send data by master to slave
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 * pTxbuffer		- Buffer for transmit data
 * Len				- Length of data
 * SlaveAddr		- Address of slave device
 * @return			- Busy state
 *
 *********************************************************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFFEN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**********************************************************************************************************
 * @fn 				- I2C_MasterReceiveDataIT
 *
 * #brief 			- This function to send data by master to slave
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 * pRxbuffer		- Buffer for receive data
 * Len				- Length of data
 * SlaveAddr		- Address of slave device
 * @return			- Busy state
 *
 *********************************************************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// Implement code to generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFFEN);

		// Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/**********************************************************************************************************
 * @fn 				- I2C_MasterHandleTXEInterrupt
 *
 * #brief 			- This function is helper function
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if (pI2CHandle->TxLen > 0)
	{
		//1. Load the data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. Decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_MasterHandleRXNEInterrupt
 *
 * #brief 			- This function is helper function
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	//We have to do the data reception
	if (pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;

		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1)
	{
		if (pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxLen == 0)
	{
		// close the I2C data reception and notify the application

		//1. generate the stop condition
		if (pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. CLose the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_EV_IRQHandling
 *
 * #brief 			- This function to EV handling
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	//1. Handle for interrupt generated by SB event
	//   Note: SB flag is only applicable in Master mode
	if (temp1 && temp3)
	{
		// The interrupt is generated because of SB event
		// This block will not be executed in slave mode because for slave SB is always zero
		// Executed the address phase
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecutedAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecutedAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle for interrupt generated by ADDR event
	//   Note: When master mode: Address is sent
	//		   When slave mode: Address matched with own address
	if (temp1 && temp3)
	{
		// Interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if (temp1 && temp3)
	{
		// BTF flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// make sure that TXE is also set
			if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				//BTF, TXE = 1
				if (pI2CHandle->TxLen == 0)
				{
					//1. Generate the STOP condition
					if (pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		} else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;	// nothing to do
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
	//   Note: Stop detection flag is applicable only slave mode. For master this flag will be not applicable
	if (temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF (i.e 1) read SR1 2) Write the CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle for interrupt generated by TXE event
	if (temp1 && temp2 && temp3)
	{
		// Check for device mode
		if ((pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)))
		{
			// TXE flag is set
			// We have to do the data transmission
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle for interrupt generated by RXNE event
	if (temp1 && temp2 && temp3)
	{
		//check for device mode
		if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
		{
			// RXNE flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_ER_IRQHandling
 *
 * #brief 			- This function to ER handling
 * pI2CHandle		- This is structure which contain: base address of the I2C peripheral and pin configuration
 *
 *********************************************************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVER);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVER);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/**********************************************************************************************************
 * @fn 				- I2C_ManageAcking
 *
 * #brief 			- This function enables or disables acknowledge bit in I2C
 * pI2Cx			- Base address of the I2C peripheral
 * EnorDi 			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		// enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else
	{
		// disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFFEN);

	//Implement the code to display ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFFEN);

	//Implement the code to display ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}
