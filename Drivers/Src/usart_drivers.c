/*
 * usart_drivers.c
 *
 *  Created on: 17 lis 2021
 *      Author: Kacper
 */

#include "usart_drivers.h"


/*********************************************************************
 * @fn      		  - USART_PeriClockControl
 *
 * @brief             - Function to enable or disable peripheral clock for USART
 *
 * @pUSARTx      	  - Structure with configure register USART
 * @EnorDi			  - Enable or disable
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
	} else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		} else if (pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		} else if (pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		} else if (pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		} else if (pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             - Function for initialization USART
 *
 * @pUSARTHandle      - Structure with configure USART and registers
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);
	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}

/*********************************************************************
 * @fn      		  - USART_DeInit
 *
 * @brief             - Function for de initialization USART
 *
 * @pUSARTx     	  - Structure with configure registers USART
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	} else if (pUSARTx == USART2)
	{
		USART2_REG_RESET();
	} else if ( pUSARTx == USART3)
	{
		USART3_REG_RESET();
	} else if (pUSARTx == UART4)
	{
		UART4_REG_RESET();
	} else if (pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - USART_PeripheralControl
 *
 * @brief             -	Function for enable or disable peripheral.
 *
 * @param[in]         - Structure with register for specific USART
 * @param[in]         - Enable or disable
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}

}

/*********************************************************************
 * @fn      		  - USART_GetFlagStatus
 *
 * @brief             - Function to check current status of flag
 *
 * @param[in]         - Structure with registers for specific USART
 * @param[in]         - Name of status flag
 *
 * @return            - Status of flag
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
    if(pUSARTx->SR & StatusFlagName)
    {
    	return SET;
    }

   return RESET;
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - Function to send data by USART
 *
 * @param[in]         - Structure with registers for specific USART and structure with config
 * @param[in]         - Buffer for transmit data
 * @param[in]         - Length of data
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TODO));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             - Function for receive data by USART
 *
 * @param[in]         - Structure with registers for specific USART and structure with config
 * @param[in]         - Buffer for receive data
 * @param[in]         - Length of data
 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < TODO; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		TODO

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == TODO)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)TODO);

				//Now increment the pRxBuffer two times
				TODO
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				TODO
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = TODO;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) TODO

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             - Function for send data by USART with interrupt
 *
 * @param[in]         - Structure with registers for specific USART and structure with config
 * @param[in]         - Buffer for transmit data
 * @param[in]         - Length of data
 *
 * @return            - State of tx bit
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TODO;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TODO = Len;
		pUSARTHandle->pTxBuffer = TODO;
		pUSARTHandle->TxBusyState = TODO;

		//Implement the code to enable interrupt for TXE
		TODO


		//Implement the code to enable interrupt for TC
		TODO


	}

	return txstate;
}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             - Function for receive data by USART with interrupt
 *
 * @param[in]         - Structure with registers for specific USART and structure with config
 * @param[in]         - Buffer for receive data
 * @param[in]         - Length of data
 *
 * @return            - state of rx bit
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->TODO;

	if(rxstate != TODO)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = TODO;
		pUSARTHandle->RxBusyState = TODO;

		//Implement the code to enable interrupt for RXNE
		TODO

	}

	return rxstate;
}
