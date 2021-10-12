/*
 * spi_drivers.c
 *
 *  Created on: 6 paź 2021
 *      Author: Kacper Janowski
 */

#include "spi_drivers.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**********************************************************************************************************
 * @fn 				- SPI_PeriClockControl
 *
 * #brief 			- This function enables or disables peripheral clock for given SPI port
 * pSPIx			- Base address of the SPI peripheral
 * EnorDi 			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_Init
 *
 * #brief 			- This function initialize structure for specific SPI
 * pSPIHandle		- This is structure which contain: base address of the SPI peripheral and pin configuration
 *
 *********************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPI, ENABLE);

	// First configure the SPI_CR1 register
	uint32_t tempreg = 0;
	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	//2. configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CINFIG_FD)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CINFIG_HD)
	{
		// bidi mode should be set
		tempreg |= (1 << 15);

	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CINFIG_SIMPLEX_RXONLY)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY bit must be set
		tempreg |= (1 << 10);
	}

	//3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPI->CR1 = tempreg;
}

/**********************************************************************************************************
 * @fn 				- SPI_DeInit
 *
 * #brief 			- This function reset specific parameters for SPI
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 *
 *********************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	} else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	} else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_GetFlagStatus
 *
 * #brief 			- This function get a flag form register SPIx
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * FlagName			- Flag name
 *
 *  @return 		- This is SET or RESET macro
 *
 *********************************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**********************************************************************************************************
 * @fn 				- SPI_SendData
 *
 * #brief 			- This function send data by specific SPI
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * pTxBuffer		- Pointer to buffer for transmit data
 * Len				- Length of data
 *
 * @Note			- This is blocking call
 *
 *********************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. Wait until TXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			//1. Load data in to the DR
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		} else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_ReceiveData
 *
 * #brief 			- This function receive data by specific SPI
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * pTxBuffer		- Pointer to buffer for receive data
 * Len				- Length of data
 *
 *********************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		//1. Wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			//1. Load data from DR to RxBuffer address
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		} else
		{
			// 8 bit DFF
			*(uint16_t*)pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_SendDataIT
 *
 * #brief 			- This function send data by specific SPI
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * pTxBuffer		- Pointer to buffer for transmit data
 * Len				- Length of data
 *
 * @return 			- SPI application state
 *
 * @Note			- This is interupt call
 *
 *********************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	//4. Data Transmission will be handled by the ISR code

	return state;
}

/**********************************************************************************************************
 * @fn 				- SPI_ReceiveDataIT
 *
 * #brief 			- This function receive data by specific SPI
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * pTxBuffer		- Pointer to buffer for receive data
 * Len				- Length of data
 *
 * @return 			- SPI application state
 *
 *********************************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in transmission so that
		//	 no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPI->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	//4. Data Transmission will be handled by the ISR code

	return state;
}

/**********************************************************************************************************
 * @fn 				- SPI_IRQInterruptConfig
 *
 * #brief 			-
 * IRQNumber		- Number interrupt register which you would like to enable or disable
 * EnorDi	 		- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn 				- SPI_IRQPriorityConfig
 *
 * #brief 			-
 * IRQNumber		- Number interrupt register
 * IRQPriority 		- Number of priority
 *
 *********************************************************************************************************/
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**********************************************************************************************************
 * @fn 				- SPI_IRQHandling
 *
 * #brief 			-
 * PinNumber		- Base address of the spi peripheral
 *
 *********************************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//First check for TXE
	temp1 = pHandle->pSPI->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPI->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE
	temp1 = pHandle->pSPI->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPI->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check for ovr flag
	temp1 = pHandle->pSPI->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPI->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// Some helper function implementations

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPI->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		// 16 bit DFF
		//1. Load data in to the DR
		pSPIHandle->pSPI->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else
	{
		// 8 bit DFF
		pSPIHandle->pSPI->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if (!pSPIHandle->TxLen)
	{
		//TxLen is zero, so close the spi transmission and inform the application that TX is over.
		//This prevents interrupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if (pSPIHandle->pSPI->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		// 16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPI->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer--;
		pSPIHandle->pRxBuffer--;
	} else
	{
		// 8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPI->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if (!pSPIHandle->TxLen)
	{
		//reception is complete
		//turn off the rxneie interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. Clear the ovr flag
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPI->DR;
		temp = pSPIHandle->pSPI->SR;
	}
	(void)temp;
	//2. Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPI->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPI->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//This is a weak implementation. The application may override this function.
}

/**********************************************************************************************************
 * @fn 				- SPI_PeripheralControl
 *
 * #brief 			- This function enable or disable SPE bit in control register 1
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * EnOrDi			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_SSIConfig
 *
 * #brief 			- This function enable or disable SSI bit in control register 1
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * EnOrDi			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	} else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/**********************************************************************************************************
 * @fn 				- SPI_SSOEConfig
 *
 * #brief 			- This function enable or disable SSOE bit in control register 2
 * pSPIx			- This is structure which contain: base address of the SPI peripheral and pin configuration
 * EnOrDi			- ENABLE or DISABLE macros
 *
 *********************************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	} else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
