/**
 *******************************************************************************
 * @file    : cli.c
 * @author  : Dung Do Dang
 * @version : V1.0.0
 * @date    : 2018-06-28
 * @brief   : This software module handles the command line interface.
 *
 *******************************************************************************
 *
 * COPYRIGHT(c) 2018 Neo Monitors AS
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
#include <string.h>
#include <stdlib.h>
#include "cli.h"
#include "tcd1304.h"

/* Private defines -----------------------------------------------------------*/
#define RING_BUFFER_SIZE                ((uint32_t) 12U)
#define CMD_BUFFER_SIZE                 ((uint32_t) 10U)
#define PARAM_BUFFER_SIZE               ((uint32_t) 20U)
#define COMMAND_BUFFER_SIZE             ((uint32_t) CMD_BUFFER_SIZE + PARAM_BUFFER_SIZE + 2U)

/* Private typedefs ----------------------------------------------------------*/
typedef struct
{
    char serialDataBuffer[ RING_BUFFER_SIZE ];
    uint32_t head;
    uint32_t tail;
    uint32_t errors;
    uint32_t packets;
} CLI_RING_BUFFER_t;

typedef struct
{
    char buffer[ COMMAND_BUFFER_SIZE ];
    char cmd[ CMD_BUFFER_SIZE ];
    char param[ PARAM_BUFFER_SIZE ];
    uint8_t pos;
} CLI_PCB_t;

/* Private macros ------------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CLI_RING_BUFFER_t ringBuffer;
static UART_HandleTypeDef *CLI_uart;
static CLI_PCB_t pcb;

/* Private function prototypes -----------------------------------------------*/
static void CLI_ClearCommand(void);
static CLI_ERR_t CLI_GetCommand(void);
static CLI_ERR_t CLI_ProcessCommand(char byte);
static CLI_ERR_t CLI_IF_Init(void);

extern void _Error_Handler(char *, int);

/**
 *******************************************************************************
 *                          PUBLIC IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @Brief   Initialize the CLI
 * @param   None
 * @retval  pointer to the osThreadId object
 ******************************************************************************/
CLI_ERR_t CLI_Init(UART_HandleTypeDef *puart)
{
    if ( puart == NULL )
    {
        return CLI_ERR_NULL_POINTER;
    }

    CLI_uart = puart;
    CLI_ClearCommand();

    return CLI_IF_Init();
}

/*******************************************************************************
 * @brief   Check the data buffer for new data and process it if new data is available
 * @param   None
 * @retval  None
 * The DMA transfers data from RS485 interface to a RAM ring buffer.
 * This function makes sure the tail is equal the head.
 *
 ******************************************************************************/
void CLI_CheckInputBuffer(void)
{
    char byte;
    ringBuffer.head = RING_BUFFER_SIZE - CLI_uart->hdmarx->Instance->NDTR;

    if ( ringBuffer.tail < ringBuffer.head )
    {
        while ( ringBuffer.tail < ringBuffer.head )
        {
            byte = ringBuffer.serialDataBuffer[ ringBuffer.tail++ ];
            CLI_ProcessCommand( byte );
        }
    }
    else if ( ringBuffer.tail > ringBuffer.head )
    {
        while ( ringBuffer.tail < sizeof(ringBuffer.serialDataBuffer) )
        {
            byte = ringBuffer.serialDataBuffer[ ringBuffer.tail++ ];
            CLI_ProcessCommand( byte );
        }
        ringBuffer.tail = 0U;

        /* Catch up the head that has wrapped around zero */
        while ( ringBuffer.tail < ringBuffer.head )
        {
            byte = ringBuffer.serialDataBuffer[ ringBuffer.tail++ ];
            CLI_ProcessCommand( byte );
        }
    }
    else
    {
        /* head == tail = 0 means no new data */
    }
}

/**
 *******************************************************************************
 *                          PRIVATE IMPLEMENTATION SECTION
 *******************************************************************************
 */

/*******************************************************************************
 * @brief   This function initialize the serial interface and starts the DMA
 * @param   None
 * @retval  HAL_StatusTypeDef code
 *
 ******************************************************************************/
static CLI_ERR_t CLI_IF_Init(void)
{
    CLI_ERR_t status = CLI_OK;
    uint16_t TxSize = (uint16_t) sizeof(ringBuffer.serialDataBuffer);
    uint8_t *RxBufAddr = (uint8_t *) ringBuffer.serialDataBuffer;

    if ( HAL_UART_Receive_DMA( CLI_uart, RxBufAddr, TxSize ) != HAL_OK )
    {
        status = CLI_ERR_NOT_INITIALIZED;
    }
    return status;
}

/*******************************************************************************
 * @brief   This function process the data in the RAM ring buffer
 * @param   data, uint8_t ASCII data from the RS485 interface.
 * @retval  None
 * This function builds the command and parameter based on the stream of data
 * bytes in the RAM ring buffer.
 *
 ******************************************************************************/
static CLI_ERR_t CLI_ProcessCommand(char byte)
{
    if ( (byte != ';') && (byte != ' ') && (byte != '\r') && (byte != '\n') )
    {
        if ( pcb.pos < sizeof(pcb.buffer) )
        {
            pcb.buffer[ pcb.pos++ ] = byte;
        }
        else
        {
            CLI_ClearCommand();
            return CLI_ERROR;
        }
    }
    else if ( (byte == ' ') || (byte == '\r') || (byte == '\n') )
    {
        /* Ignore space, carriage return and newline characters */
    }
    else
    {
        pcb.buffer[ pcb.pos ] = '\0'; /* Terminate the string */

        if ( CLI_GetCommand() != CLI_OK )
        {
            CLI_ClearCommand();
            return CLI_ERROR;
        }

        char *cmd = pcb.cmd;
        char *param = pcb.param;
        char ack[ 32 ];

        /**
         * Process the command with correct actions.
         */
        if ( strcmp( cmd, "SH=" ) == 0 )
        {
            uint32_t t_sh_us = atoi( param );
            extern TCD_CONFIG_t sensor_config;
            sensor_config.t_int_us = t_sh_us;
            TCD_SetIntTime( &sensor_config );

            sprintf( ack, "SH = %d\r\n", t_sh_us );
            HAL_UART_Transmit( CLI_uart, (uint8_t *) ack, strlen( ack ), 1000U );
        }

        else if ( strcmp( cmd, "ICG=" ) == 0 )
        {
            uint32_t t_icg_us = atoi( param );
            extern TCD_CONFIG_t sensor_config;
            sensor_config.t_icg_us = t_icg_us;

            sprintf( ack, "ICG = %d\r\n", t_icg_us );
            HAL_UART_Transmit( CLI_uart, (uint8_t *) ack, strlen( ack ), 1000U );
        }

        else if ( strcmp( cmd, "AVG=" ) == 0 )
        {
            uint32_t avg = atoi( param );
            extern TCD_CONFIG_t sensor_config;
            sensor_config.avg = avg;

            sprintf( ack, "AVG = %d\r\n", avg );
            HAL_UART_Transmit( CLI_uart, (uint8_t *) ack, strlen( ack ), 1000U );
        }

        else if ( strcmp( cmd, "DATA" ) == 0 )
        {
            extern volatile uint8_t requestToSendFlag;
            requestToSendFlag = 1U;
        }

        else if ( strcmp( cmd, "RUN" ) == 0 )
        {
            TCD_Start();
            sprintf( ack, "RUN()\r\n" );
            HAL_UART_Transmit( CLI_uart, (uint8_t *) ack, strlen( ack ), 1000U );
        }

        else if ( strcmp( cmd, "STOP" ) == 0 )
        {
            TCD_Stop();
            sprintf( ack, "STOP()\r\n" );
            HAL_UART_Transmit( CLI_uart, (uint8_t *) ack, strlen( ack ), 1000U );
        }

        else
        {
            /* No command found */
        }

        CLI_ClearCommand();
    }

    return CLI_OK;
}

/*******************************************************************************
 * @brief   Iterate through the buffer and find the command and parameter string.
 * @param   None
 * @retval  CLI_OK on success or error codes.
 * PH0=100.0;
 ******************************************************************************/
static CLI_ERR_t CLI_GetCommand(void)
{
    uint32_t idx = 0U;
    uint32_t len;

    while ( (pcb.buffer[ idx ] != '=') && (pcb.buffer[ idx ] != '\0') )
    {
        idx++;
        if ( idx >= (sizeof(pcb.cmd) - 1U) )
        {
            return CLI_ERR_CRITICAL;
        }
    }
    len = idx + 1U;
    memcpy( pcb.cmd, pcb.buffer, len );
    strcpy( pcb.param, &pcb.buffer[ len ] );
    pcb.cmd[ len ] = 0;

    return CLI_OK;
}

/*******************************************************************************
 * @brief
 * @param
 * @retval
 *
 ******************************************************************************/
static void CLI_ClearCommand(void)
{
    pcb.cmd[ 0 ] = 0U;
    pcb.param[ 0 ] = 0U;
    pcb.buffer[ 0 ] = 0U;
    pcb.pos = 0U;
}
/****************************** END OF FILE ***********************************/
