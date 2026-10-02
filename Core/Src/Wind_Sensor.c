#include "wind_sensor.h"
#include "board_config.h"

/* Debug variables */

volatile uint32_t g_wind_requests = 0U;
volatile uint32_t g_wind_ok = 0U;
volatile uint32_t g_wind_timeout = 0U;
volatile uint32_t g_wind_crc_error = 0U;
volatile uint32_t g_wind_frame_error = 0U;
volatile uint32_t g_wind_exception = 0U;

volatile uint8_t g_wind_exception_code = 0U;

volatile uint32_t g_wind_uart_error = 0U;
volatile uint32_t g_wind_queue_wait = 0U;

volatile uint16_t g_wind_speed_x10 = 0U;
volatile uint8_t g_wind_valid = 0U;

volatile uint32_t g_wind_last_update_ms = 0U;

volatile uint8_t g_wind_last_response[7] = {0U};


#if ((BOARD_ROLE == BOARD_1_TX) && \
     (RS485_MODE == RS485_MODE_RK100_02))

#include "main.h"
#include "rs485.h"
#include "protocol.h"
#include "subghz_phy_app.h"

/*
 * RK100-02 request:
 *
 * Slave    = 01
 * Function = 03
 * Register = 0000
 * Quantity = 0001
 * CRC      = 840A (low byte first)
 */

static const uint8_t wind_request[8] =
{
    WIND_SLAVE_ID,
    0x03U,
    0x00U,
    0x00U,
    0x00U,
    0x01U,
    0x84U,
    0x0AU
};

#define WIND_NORMAL_LEN      7U
#define WIND_EXCEPTION_LEN   5U

/* UART RX buffer */

static volatile uint8_t rx_buf[WIND_NORMAL_LEN];

static volatile uint8_t rx_len = 0U;

static volatile uint8_t rx_waiting = 0U;

static volatile uint8_t rx_complete = 0U;

static volatile uint8_t rx_uart_fault = 0U;

/* Pending data for LoRa */

static uint8_t pending_frame[WIND_NORMAL_LEN];

static uint8_t pending_valid = 0U;

static uint32_t next_poll_ms = 0U;

static uint32_t request_started_ms = 0U;


/* =====================================================
 * WIND INIT
 * ===================================================== */

void Wind_Init(void)
{
    rx_len = 0U;

    rx_waiting = 0U;

    rx_complete = 0U;

    rx_uart_fault = 0U;

    pending_valid = 0U;

    g_wind_valid = 0U;

    /* First request after 200 ms */
    next_poll_ms = HAL_GetTick() + 200U;
}


/* =====================================================
 * RECEIVE ONE UART BYTE
 *
 * Called from UART RX interrupt.
 * Do not perform blocking operations here.
 * ===================================================== */

void Wind_OnByte(uint8_t b)
{
    uint8_t expected;

    if ((rx_waiting == 0U) ||
        (rx_complete != 0U))
    {
        return;
    }

    /* Check Slave ID */

    if ((rx_len == 0U) &&
        (b != WIND_SLAVE_ID))
    {
        return;
    }

    /* Check function code */

    if ((rx_len == 1U) &&
        (b != 0x03U) &&
        (b != 0x83U))
    {
        rx_len = (b == WIND_SLAVE_ID) ? 1U : 0U;

        return;
    }

    /* Normal response byte count must be 2 */

    if ((rx_len == 2U) &&
        (rx_buf[1] == 0x03U) &&
        (b != 0x02U))
    {
        rx_len = 0U;

        return;
    }

    if (rx_len >= WIND_NORMAL_LEN)
    {
        return;
    }

    rx_buf[rx_len] = b;

    rx_len++;

    /* Exception response has 5 bytes */

    expected = (rx_buf[1] == 0x83U)
             ? WIND_EXCEPTION_LEN
             : WIND_NORMAL_LEN;

    if (rx_len == expected)
    {
        rx_waiting = 0U;

        rx_complete = 1U;
    }
}


/* =====================================================
 * UART ERROR CALLBACK
 * ===================================================== */

void Wind_OnUartError(void)
{
    if (rx_waiting != 0U)
    {
        rx_uart_fault = 1U;
    }
}


/* =====================================================
 * MAIN WIND SENSOR TASK
 * ===================================================== */

void Wind_Task(void)
{
    uint8_t response[WIND_NORMAL_LEN];

    uint8_t len;

    uint16_t calculated_crc;

    uint16_t received_crc;

    uint32_t now = HAL_GetTick();


    /* -----------------------------------------------
     * 1. PROCESS RECEIVED MODBUS FRAME
     * ----------------------------------------------- */

    if (rx_complete != 0U)
    {
        len = rx_len;

        for (uint8_t i = 0U; i < len; ++i)
        {
            response[i] = rx_buf[i];
        }

        rx_complete = 0U;

        rx_len = 0U;


        /* Calculate Modbus CRC */

        calculated_crc = Protocol_CRC16(
            response,
            (uint16_t)(len - 2U)
        );

        received_crc =
            (uint16_t)response[len - 2U] |
            (uint16_t)(
                (uint16_t)response[len - 1U] << 8U
            );


        /* Invalid CRC */

        if (calculated_crc != received_crc)
        {
            g_wind_crc_error++;

            g_wind_valid = 0U;
        }

        /* Modbus exception */

        else if ((len == WIND_EXCEPTION_LEN) &&
                 (response[1] == 0x83U))
        {
            g_wind_exception++;

            g_wind_exception_code = response[2];

            g_wind_valid = 0U;
        }

        /* Valid wind speed response */

        else if ((len == WIND_NORMAL_LEN) &&
                 (response[0] == WIND_SLAVE_ID) &&
                 (response[1] == 0x03U) &&
                 (response[2] == 0x02U))
        {
            /*
             * Response example:
             *
             * 01 03 02 00 B4 B8 33
             *
             * Data = 0x00B4 = 180
             *
             * Wind speed = 180 / 10 = 18.0 m/s
             */

            g_wind_speed_x10 =
                (uint16_t)(
                    ((uint16_t)response[3] << 8U) |
                    response[4]
                );

            g_wind_last_update_ms = now;

            g_wind_valid = 1U;

            g_wind_ok++;


            /* Save original Modbus response */

            for (uint8_t i = 0U;
                 i < WIND_NORMAL_LEN;
                 ++i)
            {
                g_wind_last_response[i] = response[i];

                pending_frame[i] = response[i];
            }

            pending_valid = 1U;
        }

        else
        {
            g_wind_frame_error++;

            g_wind_valid = 0U;
        }
    }


    /* -----------------------------------------------
     * 2. UART ERROR
     * ----------------------------------------------- */

    if (rx_uart_fault != 0U)
    {
        rx_waiting = 0U;

        rx_uart_fault = 0U;

        rx_len = 0U;

        g_wind_uart_error++;

        g_wind_valid = 0U;
    }


    /* -----------------------------------------------
     * 3. RESPONSE TIMEOUT
     * ----------------------------------------------- */

    if ((rx_waiting != 0U) &&
        ((uint32_t)(now - request_started_ms)
          >= WIND_RESPONSE_TIMEOUT_MS))
    {
        rx_waiting = 0U;

        rx_len = 0U;

        g_wind_timeout++;

        g_wind_valid = 0U;
    }


    /* -----------------------------------------------
     * 4. SEND VALID SENSOR DATA TO LORA QUEUE
     * ----------------------------------------------- */

    if (pending_valid != 0U)
    {
        if (SubghzApp_QueueSerial(
                pending_frame,
                WIND_NORMAL_LEN) != 0U)
        {
            pending_valid = 0U;
        }
        else
        {
            g_wind_queue_wait++;
        }
    }


    /* -----------------------------------------------
     * 5. CHECK IF READY FOR NEXT POLL
     * ----------------------------------------------- */

    if ((rx_waiting != 0U) ||
        (rx_complete != 0U) ||
        (pending_valid != 0U))
    {
        return;
    }

    if ((int32_t)(now - next_poll_ms) < 0)
    {
        return;
    }


    /* -----------------------------------------------
     * 6. PREPARE RECEIVER
     * ----------------------------------------------- */

    rx_len = 0U;

    rx_complete = 0U;

    rx_uart_fault = 0U;

    rx_waiting = 1U;

    request_started_ms = now;

    next_poll_ms = now + WIND_POLL_PERIOD_MS;


    /* -----------------------------------------------
     * 7. SEND MODBUS REQUEST TO SENSOR
     * ----------------------------------------------- */

    if (RS485_Send(
            wind_request,
            sizeof(wind_request)) == 0U)
    {
        rx_waiting = 0U;

        g_wind_uart_error++;

        g_wind_valid = 0U;
    }
    else
    {
        g_wind_requests++;
    }
}

#else

/* Disable sensor driver on Board 2 or RAW mode */

void Wind_Init(void)
{
}

void Wind_Task(void)
{
}

void Wind_OnByte(uint8_t b)
{
    (void)b;
}

void Wind_OnUartError(void)
{
}

#endif
