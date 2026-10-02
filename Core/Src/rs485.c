#include "rs485.h"
#include "usart.h"
#include "board_config.h"
#include "subghz_phy_app.h"
#include "wind_sensor.h"

#define UART_FIFO_SIZE 256U

/* Debug counters */

volatile uint32_t g_rs485_rx_bytes = 0U;
volatile uint32_t g_rs485_messages = 0U;
volatile uint32_t g_rs485_overflow = 0U;
volatile uint32_t g_rs485_errors = 0U;

volatile uint8_t g_rs485_last_byte = 0U;


#if (BOARD_ROLE == BOARD_1_TX)

static uint8_t rx_byte;


#if (RS485_MODE == RS485_MODE_RAW)

/* Legacy RAW mode */

static uint8_t uart_fifo[UART_FIFO_SIZE];

static volatile uint16_t fifo_head = 0U;
static volatile uint16_t fifo_tail = 0U;

static uint8_t serial_buffer[RS485_DATA_MAX];

static uint8_t serial_length = 0U;
static uint8_t serial_ready = 0U;

static uint32_t last_byte_time = 0U;


/* Push UART byte to FIFO */

static void RS485_PushByte(uint8_t b)
{
    uint16_t next =
        (uint16_t)((fifo_head + 1U) % UART_FIFO_SIZE);

    if (next == fifo_tail)
    {
        g_rs485_overflow++;
        return;
    }

    uart_fifo[fifo_head] = b;

    fifo_head = next;
}


/* Pop byte from FIFO */

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


/* Send RAW bytes to LoRa */

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
        return 0U;
    }

    g_rs485_messages++;

    serial_length = 0U;

    serial_ready = 0U;

    return 1U;
}

#endif /* RS485_MODE_RAW */


/* =====================================================
 * UART RX CALLBACK
 * ===================================================== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    g_rs485_rx_bytes++;

    g_rs485_last_byte = rx_byte;


#if (RS485_MODE == RS485_MODE_RK100_02)

    /* Pass received byte to Modbus parser */

    Wind_OnByte(rx_byte);

#else

    RS485_PushByte(rx_byte);

#endif

    /* Continue receiving */

    (void)HAL_UART_Receive_IT(
        huart,
        &rx_byte,
        1U
    );
}


/* =====================================================
 * UART ERROR CALLBACK
 * ===================================================== */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2)
    {
        return;
    }

    g_rs485_errors++;


#if (RS485_MODE == RS485_MODE_RK100_02)

    Wind_OnUartError();

#endif

    __HAL_UART_CLEAR_FLAG(
        huart,
        UART_CLEAR_OREF |
        UART_CLEAR_FEF |
        UART_CLEAR_NEF |
        UART_CLEAR_PEF
    );

    (void)HAL_UART_Receive_IT(
        huart,
        &rx_byte,
        1U
    );
}

#endif /* BOARD_1_TX */


/* =====================================================
 * RS485 INIT
 * ===================================================== */

void RS485_Init(void)
{
    /*
     * MAX3485:
     *
     * PA4 LOW  = RX mode
     * PA4 HIGH = TX mode
     */

    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET
    );


#if (BOARD_ROLE == BOARD_1_TX)

#if (RS485_MODE == RS485_MODE_RAW)

    fifo_head = 0U;

    fifo_tail = 0U;

    serial_length = 0U;

    serial_ready = 0U;

    last_byte_time = HAL_GetTick();

#endif

    /* Enable UART receive interrupt */

    if (HAL_UART_Receive_IT(
            &huart2,
            &rx_byte,
            1U) != HAL_OK)
    {
        Error_Handler();
    }

#endif
}


/* =====================================================
 * RS485 TRANSMIT
 * ===================================================== */

uint8_t RS485_Send(
    const uint8_t *data,
    uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((data == NULL) || (length == 0U))
    {
        return 0U;
    }

    /* Switch MAX3485 to TX mode */

    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_SET
    );

    /* Blocking UART transmit */

    status = HAL_UART_Transmit(
        &huart2,
        (uint8_t *)data,
        length,
        1000U
    );

    /*
     * HAL_UART_Transmit waits until
     * transmission is completed.
     *
     * Switch back to RX mode.
     */

    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET
    );

    return (status == HAL_OK) ? 1U : 0U;
}


/* =====================================================
 * RS485 MAIN TASK
 * ===================================================== */

void RS485_Task(void)
{

#if ((BOARD_ROLE == BOARD_1_TX) && \
     (RS485_MODE == RS485_MODE_RAW))

    uint8_t data;

    if ((serial_ready != 0U) &&
        (RS485_SubmitSerial() == 0U))
    {
        return;
    }

    while (RS485_PopByte(&data) != 0U)
    {
        serial_buffer[serial_length++] = data;

        last_byte_time = HAL_GetTick();

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

    /* Detect end of RAW message */

    if ((serial_length > 0U) &&
        ((uint32_t)(HAL_GetTick() - last_byte_time)
          >= RS485_GAP_MS))
    {
        serial_ready = 1U;

        (void)RS485_SubmitSerial();
    }

#else

    /*
     * Sensor mode:
     *
     * Wind_Task() handles polling,
     * Modbus RX and LoRa submission.
     *
     * Board 2:
     * LoRa receive handler forwards data.
     */

#endif
}
