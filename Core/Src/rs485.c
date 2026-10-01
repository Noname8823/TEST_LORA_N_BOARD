#include "rs485.h"

#include "usart.h"
#include "protocol.h"
#include "app_inputs.h"

#include <string.h>


#define RS485_RX_BUFFER_SIZE    64U
#define RS485_TX_BUFFER_SIZE    64U


/* =========================================================
 * Private variables
 * ========================================================= */

static uint8_t uart_rx_byte;


/*
 * RX buffer is filled from UART interrupt.
 * When one complete packet is received:
 *
 *      rx_frame_ready = 1
 *
 * Main loop will process it in RS485_Task().
 */
static volatile uint8_t  rx_frame_ready = 0U;

static uint8_t rx_buffer[RS485_RX_BUFFER_SIZE];

static volatile uint16_t rx_index           = 0U;
static volatile uint16_t rx_expected_length = 0U;
static volatile uint16_t rx_frame_length    = 0U;


/* =========================================================
 * RX parser state
 * ========================================================= */

typedef enum
{
    RX_WAIT_SOF1 = 0,
    RX_WAIT_SOF2,
    RX_RECEIVING

} RS485_RX_State;


static volatile RS485_RX_State rx_state = RX_WAIT_SOF1;


/* =========================================================
 * Private functions
 * ========================================================= */

static void RS485_ResetParser(void)
{
    rx_state = RX_WAIT_SOF1;

    rx_index = 0U;

    rx_expected_length = 0U;
}


static void RS485_ParseByte(uint8_t data)
{
    /*
     * One complete frame has already been received.
     *
     * Wait until RS485_Task() copies it before accepting
     * another frame.
     */
    if (rx_frame_ready != 0U)
    {
        return;
    }


    switch (rx_state)
    {
        /* -------------------------------------------------
         * Wait first start byte: 0xAA
         * ------------------------------------------------- */
        case RX_WAIT_SOF1:
        {
            if (data == PROTO_SOF1)
            {
                rx_buffer[0] = data;

                rx_index = 1U;

                rx_state = RX_WAIT_SOF2;
            }

            break;
        }


        /* -------------------------------------------------
         * Wait second start byte: 0x55
         * ------------------------------------------------- */
        case RX_WAIT_SOF2:
        {
            if (data == PROTO_SOF2)
            {
                rx_buffer[1] = data;

                rx_index = 2U;

                rx_expected_length = 0U;

                rx_state = RX_RECEIVING;
            }
            /*
             * Example:
             *
             * AA AA 55 ...
             *
             * Second AA can be treated as a new SOF1.
             */
            else if (data == PROTO_SOF1)
            {
                rx_buffer[0] = data;

                rx_index = 1U;

                rx_state = RX_WAIT_SOF2;
            }
            else
            {
                RS485_ResetParser();
            }

            break;
        }


        /* -------------------------------------------------
         * Receive remaining packet
         * ------------------------------------------------- */
        case RX_RECEIVING:
        {
            /*
             * Protect RX buffer overflow.
             */
            if (rx_index >= RS485_RX_BUFFER_SIZE)
            {
                RS485_ResetParser();

                break;
            }


            rx_buffer[rx_index] = data;

            rx_index++;


            /*
             * Packet:
             *
             *  AA 55 DST SRC CMD SEQ LEN DATA... CRC_L CRC_H
             *
             *  0  1   2   3   4   5   6
             *
             * When rx_index == 7, LEN has just been received.
             */
            if (rx_index == 7U)
            {
                uint8_t payload_len = rx_buffer[6];


                /*
                 * Reject invalid payload length.
                 */
                if (payload_len > PROTO_MAX_PAYLOAD)
                {
                    RS485_ResetParser();

                    break;
                }


                /*
                 * Total packet:
                 *
                 * SOF      = 2 bytes
                 * header   = 5 bytes
                 * payload  = LEN bytes
                 * CRC      = 2 bytes
                 *
                 * Total = 9 + LEN
                 */
                rx_expected_length =
                    (uint16_t)(9U + payload_len);


                /*
                 * Also protect our physical RX buffer.
                 */
                if (rx_expected_length > RS485_RX_BUFFER_SIZE)
                {
                    RS485_ResetParser();

                    break;
                }
            }


            /*
             * Complete packet received.
             */
            if ((rx_expected_length > 0U) &&
                (rx_index >= rx_expected_length))
            {
                rx_frame_length = rx_expected_length;

                rx_frame_ready = 1U;

                /*
                 * Prepare parser state.
                 *
                 * But rx_buffer remains unchanged until
                 * RS485_Task() copies the complete frame.
                 */
                rx_state = RX_WAIT_SOF1;

                rx_index = 0U;

                rx_expected_length = 0U;
            }

            break;
        }


        default:
        {
            RS485_ResetParser();

            break;
        }
    }
}


/* =========================================================
 * Public functions
 * ========================================================= */

void RS485_Init(void)
{
    /*
     * MAX485 / RS485 direction:
     *
     * DIR = 0 -> RX
     * DIR = 1 -> TX
     *
     * This assumes DE and /RE are controlled together.
     */

    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);


    /* Reset RX state */
    rx_frame_ready = 0U;

    rx_frame_length = 0U;

    RS485_ResetParser();


    /*
     * Start receiving one byte using USART2 interrupt.
     */
    if (HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1U) != HAL_OK)
    {
        Error_Handler();
    }
}


void RS485_Send(uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }


    /*
     * Switch RS485 transceiver to transmit mode.
     */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_SET);


    /*
     * Send packet through USART2.
     */
    if (HAL_UART_Transmit(
            &huart2,
            data,
            length,
            1000U) != HAL_OK)
    {
        /*
         * Return transceiver to RX even if TX failed.
         */
        HAL_GPIO_WritePin(
            RS485_DR_GPIO_Port,
            RS485_DR_Pin,
            GPIO_PIN_RESET);

        return;
    }


    /*
     * HAL_UART_Transmit() normally waits until TC,
     * but keep this check to make absolutely sure
     * the final stop bit has left the UART before
     * disabling the RS485 transmitter.
     */
    while (__HAL_UART_GET_FLAG(
               &huart2,
               UART_FLAG_TC) == RESET)
    {
    }


    /*
     * Back to receive mode.
     */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);
}


void RS485_Task(void)
{
    ProtocolFrame frame;

    uint8_t local_rx_buffer[RS485_RX_BUFFER_SIZE];

    uint8_t tx_buffer[RS485_TX_BUFFER_SIZE];

    uint8_t payload[PROTO_MAX_PAYLOAD];

    uint16_t length;


    /*
     * No complete packet yet.
     */
    if (rx_frame_ready == 0U)
    {
        return;
    }


    /*
     * While rx_frame_ready == 1,
     * RS485_ParseByte() will not modify rx_buffer.
     *
     * Copy complete packet into local buffer first.
     */
    length = rx_frame_length;


    if ((length == 0U) ||
        (length > RS485_RX_BUFFER_SIZE))
    {
        rx_frame_ready = 0U;

        return;
    }


    memcpy(
        local_rx_buffer,
        rx_buffer,
        length);


    /*
     * Allow ISR to start receiving next packet.
     *
     * Processing below uses local_rx_buffer,
     * therefore next packet cannot overwrite the
     * packet currently being decoded.
     */
    rx_frame_ready = 0U;


    /*
     * Decode + CRC check.
     */
    if (Protocol_DecodeFrame(
            local_rx_buffer,
            length,
            &frame) == 0U)
    {
        return;
    }


    /*
     * Check device address.
     *
     * Ignore packet if it is not sent to STM32.
     */
    if (frame.dst != DEVICE_ADDR_STM32)
    {
        return;
    }


    /*
     * Process command.
     */
    switch (frame.cmd)
    {
        /* =================================================
         * PING command
         * ================================================= */
        case CMD_PING:
        {
            /*
             * Response payload:
             *
             * "OK"
             */

            payload[0] = 'O';
            payload[1] = 'K';


            uint16_t tx_len =
                Protocol_BuildFrame(
                    tx_buffer,

                    /* Destination = original sender */
                    frame.src,

                    /* Source = STM32 */
                    DEVICE_ADDR_STM32,

                    CMD_PING_RESPONSE,

                    /* Return same sequence number */
                    frame.seq,

                    payload,

                    2U);


            if (tx_len > 0U)
            {
                RS485_Send(
                    tx_buffer,
                    tx_len);
            }

            break;
        }


        /* =================================================
         * GET INPUTS command
         * ================================================= */
        case CMD_GET_INPUTS:
        {
            /*
             * Payload byte:
             *
             * bit 0 = IN1
             * bit 1 = IN2
             * bit 2 = IN3
             * bit 3 = IN4
             *
             * bit = 1 -> input ACTIVE
             */

            payload[0] = Inputs_GetMask();


            uint16_t tx_len =
                Protocol_BuildFrame(
                    tx_buffer,

                    /* Destination = Master */
                    frame.src,

                    /* Source = STM32 */
                    DEVICE_ADDR_STM32,

                    CMD_INPUTS_RESPONSE,

                    frame.seq,

                    payload,

                    1U);


            if (tx_len > 0U)
            {
                RS485_Send(
                    tx_buffer,
                    tx_len);
            }

            break;
        }


        default:
        {
            /*
             * Unknown command.
             * Ignore for now.
             */

            break;
        }
    }
}


/* =========================================================
 * STM32 HAL UART callbacks
 * ========================================================= */

void RS485_UART_RxCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RS485_ParseByte(uart_rx_byte);

        HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /*
         * Typical RS485/UART errors:
         *
         * ORE = overrun
         * FE  = framing error
         * NE  = noise error
         */

        RS485_ResetParser();

        rx_frame_ready = 0U;

        rx_frame_length = 0U;


        /*
         * Return transceiver to receive mode.
         */
        HAL_GPIO_WritePin(
            RS485_DR_GPIO_Port,
            RS485_DR_Pin,
            GPIO_PIN_RESET);


        /*
         * Restart UART reception.
         */
        HAL_UART_Receive_IT(
            &huart2,
            &uart_rx_byte,
            1U);
    }
}
