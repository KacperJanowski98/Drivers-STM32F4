/*
 * spi_drivers.h
 *
 *  Created on: 6 paź 2021
 *      Author: Kacper Janowski
 */

#ifndef INC_SPI_DRIVERS_H_
#define INC_SPI_DRIVERS_H_

/**
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/**
 *  Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPI;								/*!< This holds the base address of SPIx(x:0,1,2) */
	SPI_Config_t SPIConfig;							/*!< This holds SPI pin configuration settings >*/
}SPI_Handle_t;

/****************************************************************************************************************
 *										 APIs supported by this driver
 * 						For more information about the APIs check the function definitions
 * *************************************************************************************************************/

/**
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * Unit and De-init
 */
void SPI_Init(SPI_RegDef_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/**
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/**
 * Other Peripheral Control APIs
 */


#endif /* INC_SPI_DRIVERS_H_ */
