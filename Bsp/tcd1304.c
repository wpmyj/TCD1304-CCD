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

uint16_t TCD_SensorData[ CFG_CCD_NUM_PIXELS ];

/* Private function prototypes -----------------------------------------------*/
static void TCD_FM_Init(void);
static void TCD_ICG_Init(void);
static void TCD_SH_Init(void);
static void TCD_ADC_Init(void);

/* External functions --------------------------------------------------------*/

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
    TCD_ADC_Init();
    TCD_FM_Init();
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
static void TCD_ADC_Init(void)
{
    /* Initialize the ADC hardware and DMA */
    TCD_PORT_InitADC();

    /* Initialize the timer used to trigger AD conversion */
    TCD_PORT_ConfigADCTrigger();

    /**
     * Start the DMA to move data from ADC to RAM.
     * From now on the AD conversion is controlled by hardware.
     */
    TCD_PORT_StartADC( TCD_SensorData );
}

/****************************** END OF FILE ***********************************/
