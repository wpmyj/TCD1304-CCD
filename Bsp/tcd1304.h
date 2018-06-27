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
