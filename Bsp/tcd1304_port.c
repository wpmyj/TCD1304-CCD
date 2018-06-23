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

static uint16_t *pSensorData;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 *******************************************************************************
 *                        PUBLIC IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Start to generate ICG pulses and SH pulses
 * @param   None
 * @retval  None
 ******************************************************************************/
void TCD_PORT_Run(void)
{
    TCD_PORT_ICG_SET_DELAY( CFG_ICG_DEFAULT_PULSE_DELAY_CNT );
    TCD_PORT_SH_SET_DELAY( CFG_SH_DEFAULT_PULSE_DELAY_CNT );

    HAL_TIM_PWM_Start_IT( &htim2, TIM_CHANNEL_1 );      /* ICG */
    HAL_TIM_PWM_Start( &htim14, TIM_CHANNEL_1 );        /* SH */
    HAL_TIM_PWM_Start( &htim13, TIM_CHANNEL_1 );        /* fM */
}

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

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigICGClock(const uint32_t freq)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / 2U) / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = CFG_FM_FREQUENCY_HZ / freq - 1U;
    uint32_t pulse = CFG_ICG_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 GPIO Configuration. PA0------> TIM2_CH1 */
    GPIO_InitStruct.Pin = TCD_ICG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(TCD_ICG_GPIO_Port, &GPIO_InitStruct);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = prescaler;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = period;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if ( HAL_TIM_Base_Init( &htim2 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

    if ( HAL_TIM_ConfigClockSource( &htim2, &sClockSourceConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim2 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if ( HAL_TIMEx_MasterConfigSynchronization( &htim2, &sMasterConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim2, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigSHClock(const uint32_t intTime_us)
{
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / 2U) / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = intTime_us * CFG_FM_FREQUENCY_HZ / 1000000U - 1U;
    uint32_t pulse = CFG_SH_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;

    /* Peripheral clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();

    /* TIM14 GPIO Configuration. PF9------> TIM14_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM14;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    htim14.Instance = TIM14;
    htim14.Init.Prescaler = prescaler;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = period;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if ( HAL_TIM_Base_Init( &htim14 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim14 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = pulse;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim14, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
void TCD_PORT_ConfigADCTrigger(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    uint32_t period = 4U * HAL_RCC_GetSysClockFreq() / CFG_FM_FREQUENCY_HZ - 1U;

    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* TIM8 GPIO Configuration. PC6------> TIM8_CH1 */
    GPIO_InitStruct.Pin = ADC_TRIG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init( ADC_TRIG_GPIO_Port, &GPIO_InitStruct );

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = period;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if ( HAL_TIM_Base_Init( &htim8 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if ( HAL_TIM_ConfigClockSource( &htim8, &sClockSourceConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    if ( HAL_TIM_PWM_Init( &htim8 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC1;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;

    if ( HAL_TIMEx_MasterConfigSynchronization( &htim8, &sMasterConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = period / 4U;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if ( HAL_TIM_PWM_ConfigChannel( &htim8, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    if ( HAL_TIMEx_ConfigBreakDeadTime( &htim8, &sBreakDeadTimeConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd( htim8.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE );

    /* Enable the main output */
    __HAL_TIM_MOE_ENABLE( &htim8 );
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_InitADC(void)
{
    ADC_ChannelConfTypeDef sConfig;
    int32_t err = 0;

    /**
     * Configure the global features of the ADC
     * (Clock, Resolution, Data Alignment and number of conversion)
     */
    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DMAContinuousRequests = ENABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;

    if ( HAL_ADC_Init( &hadc3 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /**
     * Configure for the selected ADC regular channel its
     * corresponding rank in the sequencer and its sample time.
     */
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if ( HAL_ADC_ConfigChannel( &hadc3, &sConfig ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_StartADC(uint16_t *dataBuffer)
{
    if(dataBuffer == NULL)
    {
        return -1;
    }
    pSensorData = dataBuffer;

    return (uint32_t ) HAL_ADC_Start_DMA( &hadc3, (uint32_t *) dataBuffer, CFG_CCD_NUM_PIXELS );
}

__weak void TCD_PORT_CCD_ReadCompletedCallback(uint16_t *pSensorDataBuf)
{
    /* The user implements its own functionality */
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/**
 *******************************************************************************
 *                         INTERRUPT HANDLERS
 *******************************************************************************
 */

/**
 * @brief This function handles TIM5 (ICG PULSE) global interrupt.
 */
void TIM2_IRQHandler(void)
{
    TCD_PORT_ENABLE_ADC_TRIGGER();

    HAL_TIM_IRQHandler( &htim2 );
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
    TCD_PORT_DISABLE_ADC_TRIGGER();

    TCD_PORT_CCD_ReadCompletedCallback( pSensorData );

    HAL_DMA_IRQHandler( &hdma_adc3 );
}
/****************************** END OF FILE ***********************************/
