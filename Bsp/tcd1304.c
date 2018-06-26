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
    uint8_t readyToRun;
    volatile uint8_t dataReady;
    uint32_t specIndex;
    uint64_t totalSpectrumsAcquired;
} TCD_PCB_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TCD_CONFIG_t *TCD_config;
static TCD_PCB_t TCD_pcb;

/* Private function prototypes -----------------------------------------------*/
static TCD_ERR_t TCD_FM_Init(void);
static TCD_ERR_t TCD_ICG_Init(void);
static TCD_ERR_t TCD_SH_Init(void);
static TCD_ERR_t TCD_ADC_Init(void);

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
TCD_ERR_t TCD_Init(TCD_CONFIG_t *config)
{
    TCD_ERR_t err;

    if ( config == NULL )
    {
        return TCD_ERR_NULL_POINTER;
    }
    else
    {
        TCD_config = config;
    }

    /* Configure and start the ADC + DMA */
    err = TCD_ADC_Init();
    if ( err != TCD_OK )
    {
        return err;
    }

    /* Configure the master clock timer */
    err = TCD_FM_Init();
    if ( err != TCD_OK )
    {
        return err;
    }

    /* Configure the ICG pulse timer */
    err = TCD_ICG_Init();
    if ( err != TCD_OK )
    {
        return err;
    }

    /* Configure the electronic shutter timer */
    err = TCD_SH_Init();
    if ( err != TCD_OK )
    {
        return err;
    }

    TCD_pcb.specIndex = 0U;
    TCD_pcb.totalSpectrumsAcquired = 0U;
    TCD_pcb.readyToRun = 1U;
    TCD_pcb.dataReady = 0U;

    return err;
}

/*******************************************************************************
 * @brief   Start the timers and data acquisition with ADC+DMA
 * @param   None
 * @retval  TCD_OK on success or TCD_ERR_t code
 *
 ******************************************************************************/
TCD_ERR_t TCD_Start(void)
{
    if ( TCD_pcb.readyToRun == 1U )
    {
        /* Start to generate ICG and SH pulses */
        TCD_PORT_Run();
        return TCD_OK;
    }
    else
    {
        return TCD_ERR_NOT_INITIALIZED;
    }
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
            TCD_pcb.SensorDataAvg[ i ] = (uint16_t) (TCD_pcb.SensorDataAccu[ i ] / TCD_config->avg);
            TCD_pcb.SensorDataAccu[ i ] = 0U;
        }

        TCD_pcb.specIndex = 0U;
        TCD_pcb.dataReady = 1U;
    }
}

/*******************************************************************************
 * @brief   
 * @param   
 * @retval  
 *
 ******************************************************************************/
uint16_t* TCD_GetSensorDataBuffer(void)
{
    return TCD_pcb.SensorDataAvg;
}

/*******************************************************************************
 * @brief   Check if new data is ready
 * @param   None
 * @retval  1U on ready and 0U on not ready
 *
 ******************************************************************************/
uint8_t TCD_IsDataReady(void)
{
    return TCD_pcb.dataReady;
}

/*******************************************************************************
 * @brief   Clear the data ready flag
 * @param   None
 * @retval  None
 *
 ******************************************************************************/
void TCD_ClearDataReadyFlag(void)
{
    TCD_pcb.dataReady = 0U;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
uint64_t TCD_GetNumOfSpectrumsAcquired(void)
{
    return TCD_pcb.totalSpectrumsAcquired;
}

/**
 *******************************************************************************
 *                        PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Generate the Master Clock for the CCD sensor
 * @param   None
 * @retval  None
 * Check that the master clock is within the limits of the sensor; 0.4 - 4 MHz.
 ******************************************************************************/
static TCD_ERR_t TCD_FM_Init(void)
{
    TCD_ERR_t err = TCD_OK;

    if ( (TCD_config->f_master < 4000000U) && (TCD_config->f_master > 400000U) )
    {
        TCD_PORT_ConfigMasterClock( TCD_config->f_master );
    }
    else
    {
        TCD_PORT_ConfigMasterClock( CFG_FM_FREQUENCY_HZ );
        err = TCD_WARN_FM;
    }

    return err;
}

/*******************************************************************************
 * @Brief   Generate the ICG pulses
 * @param   None
 * @retval  None
 * Check that the ICG frequency is within the limits. 0 - 100 Hz
 ******************************************************************************/
static TCD_ERR_t TCD_ICG_Init(void)
{
    TCD_ERR_t err = TCD_OK;

    if ( (TCD_config->t_icg_us > 0U) && (TCD_config->t_icg_us <= CFG_ICG_MAX_PERIOD_US) )
    {
        TCD_PORT_ConfigICGClock( TCD_config->t_icg_us );
    }
    else
    {
        TCD_PORT_ConfigICGClock( CFG_ICG_DEFAULT_PERIOD_US );
        err = TCD_WARN_ICG;
    }

    return err;
}

/*******************************************************************************
 * @Brief   Generate the electronic shutter (SH) pulses
 * @param   None
 * @retval  None
 * Check that the SH period is within the limits. >=10 us and < ICG period.
 * In addition the SH and the ICG pulses MUST overlap. This is fullfilled if
 * this relationship is held:
 * P_ICG = N x P_SH, where N is an integer. 
 ******************************************************************************/
static TCD_ERR_t TCD_SH_Init(void)
{
    TCD_ERR_t err = TCD_OK;

    /* Check that P_ICG = N x P_SH, where N is an integer */
    if ( TCD_config->t_icg_us % TCD_config->t_int_us)
    {
        return TCD_ERR_SH_INIT;
    }
    
    if ( (TCD_config->t_int_us >= 10U) && (TCD_config->t_int_us <= TCD_config->t_icg_us) )
    {
        TCD_PORT_ConfigSHClock( TCD_config->t_int_us );
    }
    else
    {
        TCD_PORT_ConfigSHClock( CFG_SH_DEFAULT_PERIOD_US );
        err = TCD_WARN_SH;
    }

    return err;
}

/*******************************************************************************
 * @Brief   Generate the Master Clock to run at CFG_FM_FREQUENCY_HZ (TIM8)
 * @param   None
 * @retval  None
 ******************************************************************************/
static TCD_ERR_t TCD_ADC_Init(void)
{
    /* Initialize the ADC hardware and DMA */
    if ( TCD_PORT_InitADC() != 0 )
    {
        return TCD_ERR_ADC_INIT;
    }

    /* Initialize the timer used to trigger AD conversion */
    TCD_PORT_ConfigADCTrigger();

    /* Start the DMA transfer */
    if ( TCD_PORT_StartADC( TCD_pcb.SensorData ) == 0 )
    {
        return TCD_OK;
    }
    else
    {
        return TCD_ERR_ADC_NOT_STARTED;
    }
    /**
     * Start the DMA to move data from ADC to RAM.
     * From now on the AD conversion is controlled by hardware.
     */
}

/****************************** END OF FILE ***********************************/
