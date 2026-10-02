#include "rs485.h"
#include "board_config.h"
#include "subghz_phy_app.h"
#include "usart.h"

/* UART RX FIFO */
static volatile uint8_t fifo_data[RS485_FIFO_SIZE];
static volatile uint32_t fifo_time[RS485_FIFO_SIZE];

static volatile uint16_t fifo_head;
static volatile uint16_t fifo_tail;

static uint8_t rx_byte;
static volatile uint8_t tx_active;

/* Pending UART data */
static uint8_t chunk[RS485_DATA_MAX];
static uint8_t chunk_len;
static uint32_t last_chunk_byte_ms;

/* Debug counters */
volatile uint32_t g_rs485_rx_bytes;
volatile uint32_t g_rs485_messages;
volatile uint32_t g_rs485_overflow;
volatile uint32_t g_rs485_errors;
volatile uint32_t g_rs485_rearm_fail;
volatile uint32_t g_rs485_tx_errors;

volatile uint8_t g_rs485_last_byte;
volatile uint8_t g_rs485_raw_ring[32];
volatile uint8_t g_rs485_raw_wr;

/* =========================================
 * UART RX REARM
 * ========================================= */

static void RS485_Rearm(void)
{
    if (HAL_UART_Receive_IT(
            &huart2,
            &rx_byte,
            1U) != HAL_OK)
    {
        g_rs485_rearm_fail++;
    }
}

/* =========================================
 * INIT
 * ========================================= */

void RS485_Init(void)
{
    fifo_head = 0U;
    fifo_tail = 0U;

    chunk_len = 0U;
    tx_active = 0U;

    /* MAX3485 RX mode */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET
    );

    if (HAL_UART_Receive_IT(
            &huart2,
            &rx_byte,
            1U) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================
 * UART RX INTERRUPT
 * ========================================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART2)
        return;

    if (!tx_active)
    {
        g_rs485_rx_bytes++;
        g_rs485_last_byte = rx_byte;

        g_rs485_raw_ring[
            g_rs485_raw_wr++ & 31U
        ] = rx_byte;

        next = (uint16_t)(
            (fifo_head + 1U) &
            (RS485_FIFO_SIZE - 1U)
        );

        if (next != fifo_tail)
        {
            fifo_data[fifo_head] = rx_byte;
            fifo_time[fifo_head] = HAL_GetTick();

            fifo_head = next;
        }
        else
        {
            g_rs485_overflow++;
        }
    }

    RS485_Rearm();
}

/* =========================================
 * UART ERROR
 * ========================================= */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
        return;

    g_rs485_errors++;

    __HAL_UART_CLEAR_FLAG(
        huart,
        UART_CLEAR_OREF |
        UART_CLEAR_FEF |
        UART_CLEAR_NEF |
        UART_CLEAR_PEF
    );

    RS485_Rearm();
}

/* =========================================
 * READ FIFO
 * ========================================= */

uint8_t RS485_ReadByte(uint8_t *data)
{
    if ((data == NULL) ||
        (fifo_tail == fifo_head))
    {
        return 0U;
    }

    *data = fifo_data[fifo_tail];

    fifo_tail = (uint16_t)(
        (fifo_tail + 1U) &
        (RS485_FIFO_SIZE - 1U)
    );

    return 1U;
}

/* =========================================
 * FLUSH FIFO
 * ========================================= */

void RS485_FlushRx(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    fifo_tail = fifo_head;
    chunk_len = 0U;

    if (!primask)
    {
        __enable_irq();
    }
}

/* =========================================
 * TRANSMIT RAW DATA TO RS485
 * ========================================= */

uint8_t RS485_Send(
    const uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
        return 0U;

    tx_active = 1U;

    /* Switch MAX3485 to TX */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_SET
    );

    status = HAL_UART_Transmit(
        &huart2,
        (uint8_t *)data,
        length,
        1000U
    );

    /* HAL_UART_Transmit waits for UART TC */

    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET
    );

    tx_active = 0U;

    if (status != HAL_OK)
        g_rs485_tx_errors++;

    return (status == HAL_OK) ? 1U : 0U;
}

/* =========================================
 * SUBMIT UART DATA TO LORA
 * ========================================= */

static uint8_t RS485_SubmitChunk(void)
{
    if (chunk_len == 0U)
        return 1U;

    /* Preserve data if LoRa queue is occupied */
    if (!SubghzApp_QueueSerial(
            chunk,
            chunk_len))
    {
        return 0U;
    }

    g_rs485_messages++;

    chunk_len = 0U;

    return 1U;
}

/* =========================================
 * MAIN RS485 TASK
 * ========================================= */

void RS485_Task(void)
{
    uint8_t b;
    uint32_t t;

    while (fifo_tail != fifo_head)
    {
        /*
         * Peek before consuming.
         * Do not lose data when LoRa is busy.
         */
        b = fifo_data[fifo_tail];
        t = fifo_time[fifo_tail];

        /*
         * Detect inter-frame gap using the
         * original byte reception timestamps.
         */
        if ((chunk_len != 0U) &&
            ((uint32_t)(t - last_chunk_byte_ms)
              >= RS485_GAP_MS))
        {
            if (!RS485_SubmitChunk())
                return;
        }

        if (chunk_len == RS485_DATA_MAX)
        {
            if (!RS485_SubmitChunk())
                return;
        }

        /* Consume UART byte */
        fifo_tail = (uint16_t)(
            (fifo_tail + 1U) &
            (RS485_FIFO_SIZE - 1U)
        );

        chunk[chunk_len++] = b;
        last_chunk_byte_ms = t;

        /* Packet full */
        if (chunk_len == RS485_DATA_MAX)
        {
            if (!RS485_SubmitChunk())
                return;
        }
    }

    /* End of UART burst */
    if ((chunk_len != 0U) &&
        ((uint32_t)(
            HAL_GetTick() - last_chunk_byte_ms
        ) >= RS485_GAP_MS))
    {
        (void)RS485_SubmitChunk();
    }
}
