/**
 *******************************************************************************
 * @file    : cli.h
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-28
 * @brief   : This software module handles the command line interface.
 *
 *******************************************************************************
 *
 * COPYRIGHT(c) 2018 Neo Monitors AS
 *
 *******************************************************************************
 */

#ifndef CLIIAL_TASK_H_
#define CLIIAL_TASK_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Exported typedefs ---------------------------------------------------------*/
typedef enum
{
    CLI_OK = 0,
    CLI_ERROR,

    /* Error codes for parameter inputs */
    CLI_ERR_PARAM_OUT_OF_RANGE,
    CLI_ERR_NULL_POINTER,

    /* Serious errors */
    CLI_ERR_NOT_INITIALIZED,
    CLI_ERR_CRITICAL
} CLI_ERR_t;
/* Exported defines ----------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
CLI_ERR_t CLI_Init(UART_HandleTypeDef *puart);
void CLI_CheckInputBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* CLIIAL_TASK_H_ */
