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
 * MIT License
 *
 * Copyright (c) 2018 Dung Do Dang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
#include "tcd1304_port.h"

/* Exported typedefs ---------------------------------------------------------*/
typedef struct
{
    uint32_t avg;
    uint32_t f_master;
    uint32_t t_icg_us;
    uint32_t t_int_us;
} TCD_CONFIG_t;

typedef struct
{
    uint16_t SensorData[ CFG_CCD_NUM_PIXELS ];
    uint16_t SensorDataAvg[ CFG_CCD_NUM_PIXELS ];
    uint32_t SensorDataAccu[ CFG_CCD_NUM_PIXELS ];
} TCD_DATA_t;

typedef enum
{
    TCD_OK = 0,

    /* Warnings. Wrong input parameters. Use default values and return these warnings */
    TCD_WARN_FM,
    TCD_WARN_ICG,
    TCD_WARN_SH,
    TCD_WARN_ADC,

    /* Error codes for parameter inputs */
    TCD_ERR_PARAM_OUT_OF_RANGE,
    TCD_ERR_NULL_POINTER,

    /* Error during initialization */
    TCD_ERR_INIT,
    TCD_ERR_FM_INIT,
    TCD_ERR_ICG_INIT,
    TCD_ERR_SH_INIT,
    TCD_ERR_ADC_INIT,

    /* Serious errors */
    TCD_ERR_NOT_INITIALIZED,
    TCD_ERR_ADC_NOT_STARTED,
    TCD_ERR_CRITICAL
} TCD_ERR_t;

/* Exported defines ----------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
TCD_ERR_t TCD_Init(TCD_CONFIG_t *config);
TCD_ERR_t TCD_SetIntTime(TCD_CONFIG_t *config);
TCD_ERR_t TCD_Start(void);
TCD_ERR_t TCD_Stop(void);

TCD_DATA_t* TCD_GetSensorData(void);
uint64_t TCD_GetNumOfSpectrumsAcquired(void);

uint8_t TCD_IsDataReady(void);
void TCD_ClearDataReadyFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_H_ */
