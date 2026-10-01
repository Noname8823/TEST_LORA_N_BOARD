#include "usart_if.h"


/*
 * USART2 is reserved exclusively for RS485.
 *
 * Advanced Trace is intentionally muted.
 */


static void
    (*TraceTxCpltCallback)(void *) =
        NULL;


/* =========================================================
 * Driver expected by STM32 Advanced Trace
 * ========================================================= */

const UTIL_ADV_TRACE_Driver_s
UTIL_TraceDriver =
{
    vcom_Init,
    vcom_DeInit,
    vcom_ReceiveInit,
    vcom_Trace_DMA,
};


/* =========================================================
 * Init
 * ========================================================= */

UTIL_ADV_TRACE_Status_t
vcom_Init(
    void (*cb)(void *))
{
    /*
     * Save callback only.
     *
     * DO NOT:
     *
     * MX_USART2_UART_Init()
     * MX_DMA_Init()
     * HAL_UART_Receive_IT()
     */

    TraceTxCpltCallback =
        cb;


    return UTIL_ADV_TRACE_OK;
}


/* =========================================================
 * DeInit
 * ========================================================= */

UTIL_ADV_TRACE_Status_t
vcom_DeInit(void)
{
    /*
     * Do not reset USART2.
     */

    return UTIL_ADV_TRACE_OK;
}


/* =========================================================
 * Polling trace
 *
 * Discard debug output.
 * ========================================================= */

void vcom_Trace(
    uint8_t *p_data,
    uint16_t size)
{
    (void)p_data;
    (void)size;
}


/* =========================================================
 * DMA trace
 *
 * Discard debug output, but immediately report
 * completion so Advanced Trace does not block.
 * ========================================================= */

UTIL_ADV_TRACE_Status_t
vcom_Trace_DMA(
    uint8_t *p_data,
    uint16_t size)
{
    (void)p_data;
    (void)size;


    if (TraceTxCpltCallback != NULL)
    {
        TraceTxCpltCallback(
            NULL);
    }


    return UTIL_ADV_TRACE_OK;
}


/* =========================================================
 * Trace receive
 *
 * Disabled.
 * ========================================================= */

UTIL_ADV_TRACE_Status_t
vcom_ReceiveInit(
    void (*RxCb)(
        uint8_t *rxChar,
        uint16_t size,
        uint8_t error))
{
    (void)RxCb;


    return UTIL_ADV_TRACE_OK;
}


/* =========================================================
 * Resume
 * ========================================================= */

void vcom_Resume(void)
{
    /*
     * USART2 belongs to RS485.
     */
}


/* =========================================================
 * IRQ placeholders
 * ========================================================= */

void vcom_IRQHandler(void)
{
}


void vcom_DMA_TX_IRQHandler(void)
{
}


/* =========================================================
 * UART ERROR CALLBACK
 *
 * Recover USART2 RX after UART errors.
 * ========================================================= */

