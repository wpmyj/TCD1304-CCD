/**
 *******************************************************************************
 * @file    : tcd1304.c
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

/* Includes ------------------------------------------------------------------*/
#include "tcd1304.h"

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

uint16_t TCD_SensorData[ CFG_CCD_NUM_PIXELS ];

/* Private function prototypes -----------------------------------------------*/
static void TCD_FM_Init(void);
static void TCD_ICG_Init(void);
static void TCD_SH_Init(void);
static void TCD_ADC_Init(void);
static void TCD_ADC_TRIG_Config(void);

/* External functions --------------------------------------------------------*/
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/**
 *******************************************************************************
 *                        PUBLIC IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Init MCU timers and ADC + DMA to start acquisition of sensor data
 * @param
 * @retval
 ******************************************************************************/
void TCD_Init(void)
{
    TCD_FM_Init();
    TCD_ADC_Init();
    TCD_ADC_TRIG_Config();
    TCD_ICG_Init();
    TCD_SH_Init();
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (2 MHz)
 * @param   None
 * @retval  None
 ******************************************************************************/
static void TCD_FM_Init(void)
{
    TCD_PORT_ConfigMasterClock( CFG_FM_FREQUENCY_HZ );

    HAL_TIM_PWM_Start( &htim13, TIM_CHANNEL_1 );
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (2 MHz)
 * @param   None
 * @retval  None
 ******************************************************************************/
static void TCD_ICG_Init(void)
{
    TCD_PORT_ConfigICGClock( CFG_ICG_DEFAULT_FREQ_HZ );

    HAL_TIM_PWM_Start_IT( &htim2, TIM_CHANNEL_1 );
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (2 MHz)
 * @param   None
 * @retval  None
 ******************************************************************************/
static void TCD_SH_Init(void)
{
    TCD_PORT_ConfigSHClock( CFG_SH_DEFAULT_PERIOD_US );

    HAL_TIM_PWM_Start( &htim14, TIM_CHANNEL_1 );
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (2 MHz)
 * @param   None
 * @retval  None
 ******************************************************************************/
static void TCD_ADC_TRIG_Config(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
    uint32_t period = 4U * HAL_RCC_GetSysClockFreq() / CFG_FM_FREQUENCY_HZ - 1U;

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

    HAL_TIM_MspPostInit( &htim8 );
    HAL_TIM_PWM_Start( &htim8, TIM_CHANNEL_1 );
    htim8.Instance->CR1 &= (~TIM_CR1_CEN);

}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (2 MHz)
 * @param   None
 * @retval  None
 ******************************************************************************/
static void TCD_ADC_Init(void)
{
    ADC_ChannelConfTypeDef sConfig;

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

    /* Start the DMA to move data from ADC to RAM */
    HAL_ADC_Start_DMA( &hadc3, (uint32_t *) TCD_SensorData, CFG_CCD_NUM_PIXELS );
}

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
    extern TIM_HandleTypeDef htim8;
    htim8.Instance->CR1 = (TIM_CR1_CEN);

    HAL_TIM_IRQHandler( &htim2 );
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
    extern TIM_HandleTypeDef htim8;
    extern uint32_t TDC_SpectrumsAcquired;
    htim8.Instance->CR1 &= (~TIM_CR1_CEN);
    TDC_SpectrumsAcquired++;

    HAL_DMA_IRQHandler( &hdma_adc3 );

}
/****************************** END OF FILE ***********************************/