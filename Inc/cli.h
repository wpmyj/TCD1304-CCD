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

#ifndef CLI_H_
#define CLI_H_

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

#endif /* CLI_H_ */
