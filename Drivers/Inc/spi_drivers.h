/*
 * spi_drivers.h
 *
 *  Created on: 6 paź 2021
 *      Author: Kacper Janowski
 */

#ifndef INC_SPI_DRIVERS_H_
#define INC_SPI_DRIVERS_H_

#include "stm32f407xx.h"

/**
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;							/*!< @SPI_DeviceMode */
	uint8_t SPI_BusConfig;							/*!< @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;							/*!< @SPI_SclkSpeed */
	uint8_t SPI_DFF;								/*!< @SPI_DFF */
	uint8_t SPI_CPOL;								/*!< @SPI_CPOL */
	uint8_t SPI_CPHA;								/*!< @SPI_CPHA */
	uint8_t SPI_SSM;								/*!< @SPI_SSM */
}SPI_Config_t;

/**
 *  Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPI;								/*!< This holds the base address of SPIx(x:0,1,2) */
	SPI_Config_t SPIConfig;							/*!< This holds SPI pin configuration settings >*/
}SPI_Handle_t;

/**
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER		1
#define SPI_DEVICE_MODE_SLAVE		0

/**
 * @SPI_BusConfig
 */
#define SPI_BUS_CINFIG_FD					1
#define SPI_BUS_CINFIG_HD					2
#define SPI_BUS_CINFIG_SIMPLEX_RXONLY		3

/**
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/**
 * @SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/**
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

/**
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

/**
 * @SPI_SSM
 */
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0

/**
 * SPI related status flags definitions
 */
#define SPI_RXNE_FLAG				( 1 << SPI_SR_RXNE )
#define SPI_TXE_FLAG				( 1 << SPI_SR_TXE )
#define SPI_CHSIDE_FLAG				( 1 << SPI_SR_CHSIDE )
#define SPI_MODF_FLAG				( 1 << SPI_SR_MODF )
#define SPI_OVR_FLAG				( 1 << SPI_SR_OVR )
#define SPI_BUSY_FLAG				( 1 << SPI_SR_BSY )
#define SPI_FRE_FLAg				( 1 << SPI_SR_FRE )


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
void SPI_Init(SPI_Handle_t *pSPIHandle);
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_SPI_DRIVERS_H_ */
