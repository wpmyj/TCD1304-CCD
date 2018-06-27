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
typedef struct
{
    uint32_t f_master;
    uint32_t t_int_us;
    uint32_t t_icg_us;
    uint32_t Fs;
} PORT_TIMER_CONF_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim8;
static TIM_HandleTypeDef htim13;
static TIM_HandleTypeDef htim14;
static ADC_HandleTypeDef hadc3;
static DMA_HandleTypeDef hdma_adc3;

static PORT_TIMER_CONF_t timer_conf;

/* Private function prototypes -----------------------------------------------*/
static void TCD_PORT_EnableADCTrigger(void);
static void TCD_PORT_DisableADCTrigger(void);
static void TCD_PORT_ICG_SetDelay(uint32_t cnt);
static void TCD_PORT_SH_SetDelay(uint32_t cnt);

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
    TCD_PORT_ICG_SetDelay( CFG_ICG_DEFAULT_PULSE_DELAY_CNT );
    TCD_PORT_SH_SetDelay( CFG_SH_DEFAULT_PULSE_DELAY_CNT );
    
    /* Disable the DMA transfer half complete interrupt */
    __HAL_DMA_DISABLE_IT( &hdma_adc3, DMA_IT_HT );

    __HAL_TIM_ENABLE( &htim2 );         /* ICG TIMER */
    __HAL_TIM_ENABLE( &htim14 );        /* SH TIMER */
    __HAL_TIM_ENABLE( &htim13 );        /* fM TIMER */
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
    uint32_t period = (HAL_RCC_GetSysClockFreq() / 2U) / freq - 1U;
    timer_conf.f_master = freq;
    
    /* Peripheral clock enable */
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* TIM13 GPIO Configuration. PF8------> TIM13_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );

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

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim13.Instance, TIM_CHANNEL_1));

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(htim13.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);

    if(IS_TIM_ADVANCED_INSTANCE(htim13.Instance) != RESET)
    {
      /* Enable the main output */
      __HAL_TIM_MOE_ENABLE(&htim13);
    }

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigICGClock(const uint32_t t_icg_us)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / 2U) / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = (uint32_t) ((uint64_t) t_icg_us * CFG_FM_FREQUENCY_HZ / 1000000U) - 1U;
    uint32_t pulse = CFG_ICG_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;
    timer_conf.t_icg_us = t_icg_us;
    
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* TIM2 GPIO Configuration. PA0------> TIM2_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim2, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Set TIM2 interrupt level and enable interrupt for TIM2 */
    HAL_NVIC_SetPriority(TIM2_IRQn, TIM_ICG_INTERRUPT_LEVEL, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim2.Instance, TIM_CHANNEL_1));

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(htim2.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);

    if(IS_TIM_ADVANCED_INSTANCE(htim2.Instance) != RESET)
    {
      /* Enable the main output */
      __HAL_TIM_MOE_ENABLE(&htim2);
    }

    /* Enable the TIM Capture/Compare 1 interrupt */
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
int32_t TCD_PORT_ConfigSHClock(const uint32_t t_int_us)
{
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;
    uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / 2U) / CFG_FM_FREQUENCY_HZ - 1U;
    uint32_t period = t_int_us * CFG_FM_FREQUENCY_HZ / 1000000U - 1U;
    uint32_t pulse = CFG_SH_DEFAULT_PULSE_US * CFG_FM_FREQUENCY_HZ / 1000000U;
    timer_conf.t_int_us = t_int_us;
    
    /* Peripheral clock enable */
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

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
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if ( HAL_TIM_PWM_ConfigChannel( &htim14, &sConfigOC, TIM_CHANNEL_1 ) != HAL_OK )
    {
        _Error_Handler( __FILE__, __LINE__ );
    }

    /* Check the parameters */
    assert_param(IS_TIM_CCX_INSTANCE(htim13.Instance, TIM_CHANNEL_1));

    /* Enable the Capture compare channel */
    TIM_CCxChannelCmd(htim14.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);

    if(IS_TIM_ADVANCED_INSTANCE(htim14.Instance) != RESET)
    {
      /* Enable the main output */
      __HAL_TIM_MOE_ENABLE(&htim14);
    }

    return err;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
void TCD_PORT_ConfigADCTrigger(uint32_t Fs)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    uint32_t period = HAL_RCC_GetSysClockFreq() / Fs - 1U;
    timer_conf.Fs = Fs;
    
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* TIM8 GPIO Configuration. PC6------> TIM8_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init( GPIOC, &GPIO_InitStruct );

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
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

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
    GPIO_InitTypeDef GPIO_InitStruct;
    int32_t err = 0;

    /* Peripheral clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* ADC input pin is connected to PF6 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );

    /* ADC3 DMA Init */
    hdma_adc3.Instance = DMA2_Stream0;
    hdma_adc3.Init.Channel = DMA_CHANNEL_2;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if ( HAL_DMA_Init( &hdma_adc3 ) != HAL_OK )
    {
      _Error_Handler( __FILE__, __LINE__ );
    }

    __HAL_LINKDMA( &hadc3, DMA_Handle, hdma_adc3 );

    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DMA2_Stream0_IRQn, DMA_ADC_INTERRUPT_LEVEL, 0 );
    HAL_NVIC_EnableIRQ( DMA2_Stream0_IRQn );

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
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
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
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

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

    return (int32_t ) HAL_ADC_Start_DMA( &hadc3, (uint32_t *) dataBuffer, CFG_CCD_NUM_PIXELS );
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/**
 * The compiler is smart to inline these functions calls into where it is called
 * to remove function call overheads.
 */
/*******************************************************************************
 * @brief   Enable the timer that generates ADC trigger signal
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_EnableADCTrigger(void)
{
    TCD_ADC_TRIG_TIMER->CR1 = TIM_CR1_CEN;
}

/*******************************************************************************
 * @brief   Disable the timer that generates ADC trigger signal
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_DisableADCTrigger(void)
{
    TCD_ADC_TRIG_TIMER->CR1 &= ~TIM_CR1_CEN;
}

/*******************************************************************************
 * @brief   Set a delay to the ICG pulse with the given timer counter value
 * @param   cnt, uint32_t. Timer counter value to delay
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_ICG_SetDelay(uint32_t cnt)
{
    TCD_ICG_TIMER->CNT = cnt;
}

/*******************************************************************************
 * @brief   Set a delay to the SH pulse with the given timer counter value
 * @param   cnt, uint32_t. Timer counter value to delay
 * @retval  None
 *
 ******************************************************************************/
static void TCD_PORT_SH_SetDelay(uint32_t cnt)
{
    TCD_ICG_TIMER->CNT = cnt;
}

/**
 *******************************************************************************
 *                         INTERRUPT HANDLERS
 *******************************************************************************
 */

/*******************************************************************************
 * @brief   This function handles ICG PULSE interrupt.
 * @param   None
 * @retval  None
 *
 * When the ICG TIMER has generated an ICG pulse to start moving out charges
 * from the CCD, an interrupt request (TIMx_IRQ) is generated.
 *
 * This interrupt handler function is to start the ADC+DMA acquisition of
 * CFG_CCD_NUM_PIXELS ADC samples. This is 3694 for the TCD1304 sensor.
 * When this acquisition is finished, a new interrupt is generated;
 * TCD_CCD_ADC_INTERRUPT_HANDLER().
 *
 ******************************************************************************/
void TCD_ICG_TIMER_INTERRUPT_HANDLER(void)
{
    TCD_PORT_EnableADCTrigger();

    HAL_TIM_IRQHandler( &htim2 );
}

/*******************************************************************************
 * @brief   This function handles ADC+DMA acquisition complete interrupt.
 * @param   None
 * @retval  None
 *
 * The ADC+DMA acquisition of CFG_CCD_NUM_PIXELS samples were started in
 * TCD_ICG_TIMER_INTERRUPT_HANDLER().
 *
 * When the DMA transfer has completed an interrupt request (DMAX_StreamX_IRQ)
 * is generated.
 * This is the interrupt handler for that request. Following is done:
 * 1) Disable the ADC trigger signal (TCD_ADC_TRIG_TIMER).
 * 2) Let the HAL layer handle the DMA interrupt request
 * 3) Call the user callback function to deal with the acquired ADC samples.
 *
 ******************************************************************************/
void TCD_CCD_ADC_INTERRUPT_HANDLER(void)
{
    if ( __HAL_DMA_GET_FLAG( &hdma_adc3, DMA_FLAG_TCIF0_4) )
    {
        TCD_PORT_DisableADCTrigger();
    
        /* Do something with the acquired AD samples in RAM */
        TCD_ReadCompletedCallback();
    }
    
    HAL_DMA_IRQHandler( &hdma_adc3 );
}

/****************************** END OF FILE ***********************************/
