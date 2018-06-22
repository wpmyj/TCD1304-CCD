/**
 *******************************************************************************
 * @file    : tcd1304_port.c
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

/* Includes ------------------------------------------------------------------*/
#include "tcd1304_port.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_adc3;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 *******************************************************************************
 *                        PUBLIC IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigMasterClock(uint32_t freq)
{
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t period = (HAL_RCC_GetSysClockFreq()) / freq - 1U;

    /* Peripheral clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();

    /* TIM13 GPIO Configuration. PF8------> TIM13_CH1 */
    GPIO_InitStruct.Pin = TCD_fM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init( TCD_fM_GPIO_Port, &GPIO_InitStruct );

    htim13.Instance = TIM13;
    htim13.Init.Prescaler = 0;
    htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim13.Init.Period = period;
    htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if ( HAL_TIM_Base_Init( &htim13 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim13 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 2U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim13, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    return err;
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/****************************** END OF FILE ***********************************/
