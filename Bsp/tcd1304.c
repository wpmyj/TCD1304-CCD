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
typedef struct
{
    uint16_t SensorData[ CFG_CCD_NUM_PIXELS ];
    uint32_t SensorDataAvg[ CFG_CCD_NUM_PIXELS ];
    uint32_t specIndex;
    uint64_t totalSpectrumsAcquired;
} TCD_PCB_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TCD_CONFIG_t TCD_config;
static TCD_PCB_t TCD_pcb;

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
int32_t TCD_Init(const TCD_CONFIG_t *config)
{
    if ( config == NULL )
    {
        return -1;
    }
    
    TCD_pcb.specIndex = 0U;
    TCD_pcb.totalSpectrumsAcquired = 0U;

    /* Configure and start the ADC + DMA */
    TCD_ADC_Init();

    /* Configure and start the timers */
    TCD_FM_Init();
    TCD_ICG_Init();
    TCD_SH_Init();

    /* Start to generate ICG and SH pulses */
    TCD_PORT_Run();

    return 0;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 * This function is called from the portable layer in interrupt context.
 ******************************************************************************/
void TCD_ReadCompletedCallback(void)
{
    TCD_pcb.totalSpectrumsAcquired++;
    }
    
/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
uint32_t TCD_GetNumOfSpectrumsAcquired(void)
{
    return TCD_pcb.totalSpectrumsAcquired;
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
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (TIM2)
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_ICG_Init(void)
{
    TCD_PORT_ConfigICGClock( CFG_ICG_DEFAULT_FREQ_HZ );
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (TIM14)
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
static void TCD_SH_Init(void)
{
    TCD_PORT_ConfigSHClock( CFG_SH_DEFAULT_PERIOD_US );
}


/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (TIM8)
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
    TCD_PORT_StartADC( TCD_pcb.SensorData );
}

/****************************** END OF FILE ***********************************/
