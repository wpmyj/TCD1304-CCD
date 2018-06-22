/**
 *******************************************************************************
 * @file    : tcd1304_conf.h
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-22
 * @brief   : Configuration file driver for the CCD sensor chip from Toshiba
 *
 * This file configure the behavior of the software module.
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

#ifndef TCD1304_CONF_H_
#define TCD1304_CONF_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported typedefs ---------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define CFG_SENSOR_NUM_PIXELS               (3694U)
#define CFG_ADC_SAMPLINGRATE_HZ             (500000U)
#define CFG_FM_FREQUENCY_HZ                 (2000000U)
#define CFG_DEFAULT_ICG_PERIOD_MS           (50U)
#define CFG_DEFAULT_SH_PERIOD_MS            (10U)
#define CFG_DEFAULT_SH_PULSE_US             (2U)
#define CFG_DEFAULT_ICG_PULSE_US            (5U)
#define CFG_DEFAULT_SH_PULSE_DELAY          (2U)
#define CFG_DEFAULT_ICG_PULSE_DELAY         (0U)

/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_CONF_H_ */
