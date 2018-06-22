/**
 *******************************************************************************
 * @file    : tcd1304_port.h
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-22
 * @brief   : Portable layer of the driver for the CCD sensor chip from Toshiba
 *
 * The software module is split into two part.
 * 1) Generic API to implement the necessary functionality to control the sensor
 * 2) Portable layer to configure and control the hardware platform the sensor
 * is connected to. In this way only (2) is needed to be changed if the hardware
 * platform is needed to be replaced.
 *
 *******************************************************************************
 *
 * COPYRIGHT(c) 2018 Dung Do Dang
 *
 *******************************************************************************
 */

/**
 ***************************** Revision History ********************************
 * 2018-06-22 revision 0: Initial version
 *
 *******************************************************************************
 */

#ifndef TCD1304_PORT_H_
#define TCD1304_PORT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "tcd1304_conf.h"
#include "stm32f7xx_hal.h"

/* Exported typedefs ---------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void    TCD_PORT_Run(void);

int32_t TCD_PORT_ConfigMasterClock(const uint32_t freq);
int32_t TCD_PORT_ConfigSHClock(const uint32_t integrationTime);
int32_t TCD_PORT_ConfigICGClock(const uint32_t freq);

int32_t TCD_PORT_InitADC(void);
void    TCD_PORT_ConfigADCTrigger(void);
int32_t TCD_PORT_StartADC(uint16_t *dataBuffer);

/** 
 * This function is implemented with the keyword __weak.
 * The user implements its own function. The linker will place the correct
 * implementation in the binary output.
 */
void TCD_PORT_CCD_ReadCompletedCallback(uint16_t *pSensorDataBuf);

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_PORT_H_ */
