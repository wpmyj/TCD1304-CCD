/**
 *******************************************************************************
 * @file    : tcd1304.h
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-21
 * @brief   : Driver for the CCD sensor chip from Toshiba
 *
 *******************************************************************************
 *
 * COPYRIGHT(c) 2003 - 2018 Dung Do Dang
 *
 *******************************************************************************
 */

/**
 ***************************** Revision History ********************************
 * revision 0:
 *
 *******************************************************************************
 */

#ifndef TCD1304_H_
#define TCD1304_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Exported typedefs ---------------------------------------------------------*/
#define CFG_SENSOR_NUM_PIXELS               (3694U)
#define CFG_ADC_SAMPLINGRATE_HZ             (500000U)
#define CFG_FM_FREQUENCY_HZ                 (2000000U)
#define CFG_DEFAULT_ICG_PERIOD_MS           (50U)
#define CFG_DEFAULT_SH_PERIOD_MS            (10U)
#define CFG_DEFAULT_SH_PULSE_US             (2U)
#define CFG_DEFAULT_ICG_PULSE_US            (5U)
#define CFG_DEFAULT_SH_PULSE_DELAY          (2U)
#define CFG_DEFAULT_ICG_PULSE_DELAY         (0U)

/* Exported defines ----------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void TCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_H_ */
