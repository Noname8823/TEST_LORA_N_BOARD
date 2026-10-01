
#include "rs485.h"

#include "usart.h"
#include "usart_if.h"

#include "board_config.h"
#include "subghz_phy_app.h"

#include <string.h>

#define UART_FIFO_SIZE 256U

/* ================= DEBUG VARIABLES ================= */

volatile uint32_t g_rs485_rx_bytes = 0U;
volatile uint32_t g_rs485_messages = 0U;
volatile uint32_t g_rs485_overflow = 0U;
volatile uint32_t g_rs485_errors = 0U;

volatile uint8_t g_rs485_last_byte = 0U;

#if (BOARD_ROLE == BOARD_1_TX)

/* ================= UART FIFO ================= */

static uint8_t uart_fifo[UART_FIFO_SIZE];

static volatile uint16_t fifo_head = 0U;
static volatile uint16_t fifo_tail = 0U;

/* ================= SERIAL MESSAGE ================= */

static uint8_t serial_buffer[RS485_DATA_MAX];

static uint8_t serial_length = 0U;
static uint8_t serial_ready = 0U;

static uint32_t last_byte_time = 0U;

/* =========================================================
 * UART RECEIVE CALLBACK
 * Registered through existing usart_if.c
 * ========================================================= */

static void RS485_RxChar(
    uint8_t *data,
    uint16_t size,
    uint8_t error)
{
    if (error != 0U)
    {
        g_rs485_errors++;
        return;
    }

    if ((data == NULL) || (size == 0U))
    {
        return;
    }

    for (uint16_t i = 0U; i < size; i++)
    {
        g_rs485_rx_bytes++;
        g_rs485_last_byte = data[i];

        uint16_t next =
            (uint16_t)((fifo_head + 1U) % UART_FIFO_SIZE);

        if (next == fifo_tail)
        {
            g_rs485_overflow++;
            continue;
        }

        uart_fifo[fifo_head] = data[i];
        fifo_head = next;
    }
}

/* =========================================================
 * READ FIFO
 * ========================================================= */

static uint8_t RS485_PopByte(uint8_t *data)
{
    if (fifo_head == fifo_tail)
    {
        return 0U;
    }

    *data = uart_fifo[fifo_tail];

    fifo_tail =
        (uint16_t)((fifo_tail + 1U) % UART_FIFO_SIZE);

    return 1U;
}

/* =========================================================
 * SUBMIT SERIAL MESSAGE TO LORA
 * ========================================================= */

static uint8_t RS485_SubmitSerial(void)
{
    if (serial_length == 0U)
    {
        serial_ready = 0U;
        return 1U;
    }

    if (SubghzApp_QueueSerial(
            serial_buffer,
            serial_length) == 0U)
    {
        /* LoRa pending slot is busy */
        return 0U;
    }

    g_rs485_messages++;

    serial_length = 0U;
    serial_ready = 0U;

    return 1U;
}

#endif

/* =========================================================
 * RS485 INITIALIZATION
 * ========================================================= */

void RS485_Init(void)
{
    /*
     * PA4 LOW:
     * MAX3485 receiver enabled.
     */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);

#if (BOARD_ROLE == BOARD_1_TX)

    fifo_head = 0U;
    fifo_tail = 0U;

    serial_length = 0U;
    serial_ready = 0U;

    last_byte_time = HAL_GetTick();

    /*
     * Use the UART callback already provided
     * by the original ST project.
     */
    if (vcom_ReceiveInit(RS485_RxChar)
        != UTIL_ADV_TRACE_OK)
    {
        Error_Handler();
    }

#endif
}

/* =========================================================
 * RS485 TRANSMIT
 *
 * Used by Board 2.
 * ========================================================= */

uint8_t RS485_Send(
    const uint8_t *data,
    uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    /* Enable MAX3485 transmitter */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_SET);

    HAL_StatusTypeDef status =
        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)data,
            length,
            1000U);

    /* Wait for complete UART transmission */
    if (status == HAL_OK)
    {
        while (__HAL_UART_GET_FLAG(
                   &huart2,
                   UART_FLAG_TC) == RESET)
        {
        }
    }

    /* Return MAX3485 to RX mode */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);

    return (status == HAL_OK) ? 1U : 0U;
}

/* =========================================================
 * RS485 MAIN TASK
 * ========================================================= */

void RS485_Task(void)
{
#if (BOARD_ROLE == BOARD_1_TX)

    uint8_t data;

    /*
     * Retry submission if previous message
     * could not be queued.
     */
    if (serial_ready != 0U)
    {
        if (RS485_SubmitSerial() == 0U)
        {
            return;
        }
    }

    /*
     * Read received UART data.
     */
    while (RS485_PopByte(&data) != 0U)
    {
        serial_buffer[serial_length] = data;
        serial_length++;

        last_byte_time = HAL_GetTick();

        /*
         * Message completion:
         * - Newline received.
         * - Maximum message size reached.
         */
        if ((data == '\n') ||
            (serial_length >= RS485_DATA_MAX))
        {
            serial_ready = 1U;

            if (RS485_SubmitSerial() == 0U)
            {
                return;
            }
        }
    }

    /*
     * Also support messages without newline.
     *
     * 25 ms without new UART bytes indicates
     * the current message is complete.
     */
    if (serial_length > 0U)
    {
        uint32_t now = HAL_GetTick();

        if ((uint32_t)(now - last_byte_time)
            >= RS485_GAP_MS)
        {
            serial_ready = 1U;

            (void)RS485_SubmitSerial();
        }
    }

#else

    /* Board 2 only forwards received LoRa data to RS485. */

#endif
}

/* =========================================================
 * UART ERROR RECOVERY
 *
 * usart_if.c owns HAL_UART_RxCpltCallback().
 * This file only provides the UART error callback.
 * ========================================================= */
