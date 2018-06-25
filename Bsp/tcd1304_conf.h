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

/**
 *******************************************************************************
 *                         TCD1304 CCD SENSOR DEFINTIONS
 *******************************************************************************
 */
/* The CCD sensor requires a master clock between 0.4 - 4 MHz */
#define CFG_FM_FREQUENCY_HZ                 (2000000U)

/* The total number of AD samples to be acquired with the ADC */
#define CFG_CCD_NUM_PIXELS                  (3694U)
#define CFG_ADC_SAMPLING_RATE_HZ            (CFG_FM_FREQUENCY_HZ / 4U)

/**
 * Electronic shutter.
 * In normal mode the shutter period is equal the ICG period.
 * In electronic shutter mode the period is < ICG period. The period og SH controls
 * the integration time of the CCD.
 *
 * The SH period must conform the criteria:
 * T_ICG = N x T_SH, where N is an integer > 0.
 *
 * Minimum integration time is >= 10 us.
 * Minimum SH pulse width is >= 2 us.
 */
#define CFG_SH_DEFAULT_PERIOD_US            (1000U)
#define CFG_SH_DEFAULT_PULSE_US             (2U)
#define CFG_SH_DEFAULT_PULSE_DELAY_CNT      (1U)

/**
 * The period time of the ICG pulse determines the sensor data readout period.
 * Minimum ICG pulse width is >= 5 us.
 */
#define CFG_ICG_DEFAULT_PERIOD_MS           (100U)
#define CFG_ICG_DEFAULT_FREQ_HZ             (1000U / CFG_ICG_DEFAULT_PERIOD_MS)
#define CFG_ICG_DEFAULT_PULSE_US            (5U)
#define CFG_ICG_DEFAULT_PULSE_DELAY_CNT     (0U)

/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_CONF_H_ */
