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
    uint16_t SensorDataAvg[ CFG_CCD_NUM_PIXELS ];
    uint32_t SensorDataAccu[ CFG_CCD_NUM_PIXELS ];
    uint32_t specIndex;
    uint64_t totalSpectrumsAcquired;
} TCD_PCB_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TCD_CONFIG_t *TCD_config;
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
int32_t TCD_Init(TCD_CONFIG_t *config)
{
    if ( config == NULL )
    {
        return -1;
    }
    
    TCD_config = config;
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
 * @brief   Handle sensor data when the ADC+DMA has samples all pixels.
 * @param   pSensorDataBuf, address to the RAM locating
 * @retval
 * This function is called from the ADC DMA transfer complete interrupt handler.
 * The DMA is configured to circular (ring buffer) mode. The interrupt request
 * flag is generated just before the tranfer counter is re-set to the programmed
 * value.
 *
 * NOTE: This function is called from the portable layer in interrupt context.
 ******************************************************************************/
void TCD_ReadCompletedCallback(void)
{
    TCD_pcb.totalSpectrumsAcquired++;
    TCD_pcb.specIndex++;
    
    /* Accumulate the spectrum data vector */
    for ( uint32_t i = 0U; i < CFG_CCD_NUM_PIXELS; i++ )
    {
        TCD_pcb.SensorDataAccu[ i ] += TCD_pcb.SensorData[ i ];
    }
    
    /* Calculate average data vector */
    if ( TCD_pcb.specIndex == (TCD_config->avg - 1U) )
    {
        for ( uint32_t i = 0U; i < CFG_CCD_NUM_PIXELS; i++ )
        {
            TCD_pcb.SensorDataAvg[ i ] = TCD_pcb.SensorDataAccu[ i ] / TCD_config->avg;
            TCD_pcb.SensorDataAccu[ i ] = 0U;
        }
        
        TCD_pcb.specIndex = 0U;
    }
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
