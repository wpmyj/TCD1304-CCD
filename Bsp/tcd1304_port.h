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

/**
 *******************************************************************************
 *                         TIMER DEFINITIONS
 *******************************************************************************
 */
#define TCD_MCLK_TIMER                      (TIM13)
#define TCD_ICG_TIMER                       (TIM2)
#define TCD_SH_TIMER                        (TIM14)
#define TCD_ADC_TRIG_TIMER                  (TIM8)

/**
 *******************************************************************************
 *                         INTERRUPT HANDLERS
 *******************************************************************************
 */
#define TCD_ICG_TIMER_INTERRUPT_HANDLER     TIM2_IRQHandler
#define TCD_CCD_ADC_INTERRUPT_HANDLER       DMA2_Stream0_IRQHandler

/**
 *******************************************************************************
 *                         INTERRUPT DEFINITIONS
 *******************************************************************************
 *
 * In STM32 MCU 4 interrupt priority bits are implemented. This means that
 * the lowest (highest value) interrupt priority is 15 (0x0F).
 * We set:
 * TIM_ICG_INTERRUPT_LEVEL to default value = 6.
 * DMA_ADC_INTERRUPT_LEVEL to default value = 5.
 */
#define TIM_ICG_INTERRUPT_LEVEL             (6U)
#define DMA_ADC_INTERRUPT_LEVEL             (5U)

/**
 *******************************************************************************
 *                         LEGACY STM32 HAL DEFINITIONS
 *******************************************************************************
 */
#ifndef GPIO_SPEED_FREQ_VERY_HIGH
    #define GPIO_SPEED_FREQ_VERY_HIGH       GPIO_SPEED_HIGH
#endif

#ifndef GPIO_SPEED_FREQ_LOW
    #define GPIO_SPEED_FREQ_LOW             GPIO_SPEED_LOW
#endif

/* Exported macros -----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void    TCD_PORT_Run(void);
void    TCD_PORT_Stop(void);

int32_t TCD_PORT_ConfigMasterClock(const uint32_t freq);
int32_t TCD_PORT_ConfigSHClock(const uint32_t t_int_us);
int32_t TCD_PORT_ConfigICGClock(const uint32_t t_icg_us);

int32_t TCD_PORT_InitADC(void);
void    TCD_PORT_ConfigADCTrigger(uint32_t Fs);
int32_t TCD_PORT_StartADC(uint16_t *dataBuffer);

/**
 * This function is called when a complete CCD sensor readout is finished.
 * This function is called in the interrupt handler of the portable layer.
 * The tcd1304.c implements what should be done in this function.
 *
 */
void TCD_ReadCompletedCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* TCD1304_PORT_H_ */
