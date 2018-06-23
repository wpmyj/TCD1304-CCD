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
    uint32_t f_icg;
    uint32_t t_int_us;
} TCD_CONFIG_t;

/* Exported defines ----------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int32_t  TCD_Init(TCD_CONFIG_t *config);
uint32_t TCD_GetNumOfSpectrumsAcquired(void);

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_H_ */
