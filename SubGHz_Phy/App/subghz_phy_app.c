#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"

#include "main.h"
#include "board_config.h"
#include "app_inputs.h"
#include "rs485.h"
#include "protocol.h"

#include <string.h>
#include <stdbool.h>

/* =========================================================
 * CONFIGURATION
 * ========================================================= */

#define RX_TIMEOUT_VALUE      3000U
#define TX_TIMEOUT_VALUE      3000U

#define MAX_APP_BUFFER_SIZE   255U

#define RX_TIME_MARGIN        200U

#define FSK_AFC_BANDWIDTH     83333U

/* Board 1: number of resend attempts when ACK is lost */
#define LINK_MAX_RETRY        3U

#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error "Invalid LoRa payload size"
#endif

/* =========================================================
 * RADIO EVENTS
 * ========================================================= */

typedef enum
{
    EVENT_NONE = 0,

    EVENT_RX_DONE,
    EVENT_RX_TIMEOUT,
    EVENT_RX_ERROR,

    EVENT_TX_DONE,
    EVENT_TX_TIMEOUT

} RadioEventType;

/* =========================================================
 * PRIVATE VARIABLES
 * ========================================================= */

static RadioEvents_t RadioEvents;

static volatile RadioEventType radio_event = EVENT_NONE;

/* LoRa buffers */
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

static uint16_t RxBufferSize = 0U;

/* Radio information */
int8_t RssiValue = 0;
int8_t SnrValue = 0;


static volatile int16_t g_rx_rssi_dbm = 0;
static volatile int8_t  g_rx_snr_db = 0;
/* Radio state */
static uint8_t radio_tx_active = 0U;

/* LED PA11 timer */
static UTIL_TIMER_Object_t timerLed;

/* =========================================================
 * DEBUG VARIABLES
 *
 * Can be viewed using STM32CubeIDE Expressions.
 * ========================================================= */

volatile uint32_t g_lora_tx_start = 0U;
volatile uint32_t g_lora_tx_done = 0U;
volatile uint32_t g_lora_rx_done = 0U;
volatile uint32_t g_lora_tx_timeout = 0U;
volatile uint32_t g_lora_rx_error = 0U;

volatile uint32_t g_lora_retry = 0U;
volatile uint32_t g_lora_drop = 0U;

#if (BOARD_ROLE == BOARD_1_TX)

/* Heartbeat timer */
static UTIL_TIMER_Object_t timerHeartbeat;

static volatile uint8_t heartbeat_due = 0U;

/* Pending RS485 data (kept until ACK is received) */
static uint8_t serial_pending[RS485_DATA_MAX];

static uint8_t serial_pending_length = 0U;
static uint8_t serial_pending_valid = 0U;

/* Packet sequence */
static uint8_t tx_sequence = 0U;

static uint8_t waiting_sequence = 0U;
static uint8_t waiting_ack = 0U;

/* Retry handling */
static uint8_t tx_carries_serial = 0U;  /* in-flight frame contains RS485 data */
static uint8_t retry_count = 0U;
static uint8_t resend_pending = 0U;     /* resend with the same sequence */

#else

/* Duplicate packet protection */
static uint8_t last_rx_sequence = 0U;

static uint8_t last_rx_sequence_valid = 0U;

#endif

/* =========================================================
 * FUNCTION PROTOTYPES
 * ========================================================= */

static void OnTxDone(void);

static void OnRxDone(
    uint8_t *payload,
    uint16_t size,
    int16_t rssi,
    int8_t LoraSnr_FskCfo);

static void OnTxTimeout(void);
static void OnRxTimeout(void);
static void OnRxError(void);

static void LoRa_Process(void);

static void LoRa_StartTx(uint16_t length);
static void LoRa_StartRx(void);

static void OnledEvent(void *context);

#if (BOARD_ROLE == BOARD_1_TX)

static void OnHeartbeatEvent(void *context);

static void Board1_TransmitPending(void);
static void Board1_ProcessReceived(void);

#else

static void Board2_ProcessReceived(void);

#endif

/* =========================================================
 * PA11 LED CONTROL
 *
 * HIGH = LED ON
 * LOW  = LED OFF
 *
 * Never use HAL_GPIO_TogglePin() here.
 * ========================================================= */

static void LED_TxOn(void)
{
    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_SET);
}

static void LED_TxOff(void)
{
    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET);
}

/* =========================================================
 * LED TIMER CALLBACK
 *
 * Automatically turn PA11 OFF after TX indication.
 * ========================================================= */

static void OnledEvent(void *context)
{
    (void)context;

    LED_TxOff();
}

/* =========================================================
 * START LORA TRANSMISSION
 * ========================================================= */

static void LoRa_StartTx(uint16_t length)
{
    if ((length == 0U) ||
        (length > MAX_APP_BUFFER_SIZE))
    {
        return;
    }

    /* Stop previous LED OFF timer */
    UTIL_TIMER_Stop(&timerLed);

    /* Stop radio RX */
    Radio.Sleep();

    radio_tx_active = 1U;

    /* Debug */
    g_lora_tx_start++;

    /* PA11 HIGH: LED ON when LoRa transmission starts */
    LED_TxOn();

    /* Start sending packet */
    Radio.Send(BufferTx, length);
}

/* =========================================================
 * START LORA RECEPTION
 * ========================================================= */

static void LoRa_StartRx(void)
{
    Radio.Rx(RX_TIMEOUT_VALUE);
}

/* =========================================================
 * QUEUE RS485 DATA
 *
 * Called from RS485_Task() on Board 1.
 * ========================================================= */

uint8_t SubghzApp_QueueSerial(
    const uint8_t *data,
    uint8_t length)
{
#if (BOARD_ROLE == BOARD_1_TX)

    if ((data == NULL) ||
        (length == 0U) ||
        (length > RS485_DATA_MAX))
    {
        return 0U;
    }

    /* Only one pending serial message */
    if (serial_pending_valid != 0U)
    {
        return 0U;
    }

    memcpy(
        serial_pending,
        data,
        length);

    serial_pending_length = length;

    serial_pending_valid = 1U;

    /* Wake LoRa processing task */
    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);

    return 1U;

#else

    (void)data;
    (void)length;

    return 0U;

#endif
}

/* =========================================================
 * LORA INITIALIZATION
 * ========================================================= */

void SubghzApp_Init(void)
{
    memset(
        BufferRx,
        0,
        sizeof(BufferRx));

    memset(
        BufferTx,
        0,
        sizeof(BufferTx));

    RxBufferSize = 0U;

    radio_event = EVENT_NONE;

    radio_tx_active = 0U;

    /* Always start with PA11 LOW */
    LED_TxOff();

    /* LED timer: one-shot, started only after TX done */
    UTIL_TIMER_Create(
        &timerLed,
        LED_TX_HOLD_MS,
        UTIL_TIMER_ONESHOT,
        OnledEvent,
        NULL);

#if (BOARD_ROLE == BOARD_1_TX)

    /* =====================================================
     * BOARD 1 INITIALIZATION
     * ===================================================== */

    heartbeat_due = 0U;

    serial_pending_valid = 0U;
    serial_pending_length = 0U;

    tx_sequence = 0U;

    waiting_ack = 0U;
    waiting_sequence = 0U;

    tx_carries_serial = 0U;
    retry_count = 0U;
    resend_pending = 0U;

    UTIL_TIMER_Create(
        &timerHeartbeat,
        HEARTBEAT_PERIOD_MS,
        UTIL_TIMER_PERIODIC,
        OnHeartbeatEvent,
        NULL);

#else

    /* =====================================================
     * BOARD 2 INITIALIZATION
     * ===================================================== */

    last_rx_sequence = 0U;
    last_rx_sequence_valid = 0U;

#endif

    /* =====================================================
     * ORIGINAL RADIO CALLBACKS
     * ===================================================== */

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;

    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

    /* =====================================================
     * ORIGINAL LORA / FSK CONFIGURATION
     * ===================================================== */

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

    Radio.SetTxConfig(
        MODEM_LORA,
        TX_OUTPUT_POWER,
        0,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        true,
        0,
        0,
        LORA_IQ_INVERSION_ON,
        TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(
        MODEM_LORA,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODINGRATE,
        0,
        LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD_ON,
        0,
        true,
        0,
        0,
        LORA_IQ_INVERSION_ON,
        true);

    Radio.SetMaxPayloadLength(
        MODEM_LORA,
        MAX_APP_BUFFER_SIZE);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

    Radio.SetTxConfig(
        MODEM_FSK,
        TX_OUTPUT_POWER,
        FSK_FDEV,
        0,
        FSK_DATARATE,
        0,
        FSK_PREAMBLE_LENGTH,
        FSK_FIX_LENGTH_PAYLOAD_ON,
        true,
        0,
        0,
        0,
        TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(
        MODEM_FSK,
        FSK_BANDWIDTH,
        FSK_DATARATE,
        0,
        FSK_AFC_BANDWIDTH,
        FSK_PREAMBLE_LENGTH,
        0,
        FSK_FIX_LENGTH_PAYLOAD_ON,
        0,
        true,
        0,
        0,
        false,
        true);

    Radio.SetMaxPayloadLength(
        MODEM_FSK,
        MAX_APP_BUFFER_SIZE);

#else

#error "Invalid modem configuration"

#endif

    /* =====================================================
     * REGISTER LORA TASK
     * ===================================================== */

    UTIL_SEQ_RegTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        UTIL_SEQ_RFU,
        LoRa_Process);

#if (BOARD_ROLE == BOARD_1_TX)

    /* Start periodic heartbeat */
    UTIL_TIMER_Start(&timerHeartbeat);

#endif

    /* Both boards start in RX mode */
    LoRa_StartRx();
}

/* =========================================================
 * RADIO TX DONE CALLBACK
 * ========================================================= */

static void OnTxDone(void)
{
    g_lora_tx_done++;

    radio_event = EVENT_TX_DONE;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * RADIO RX DONE CALLBACK
 * ========================================================= */

static void OnRxDone(
    uint8_t *payload,
    uint16_t size,
    int16_t rssi,
    int8_t LoraSnr_FskCfo)
{
	g_rx_rssi_dbm = rssi;
	g_rx_snr_db = LoraSnr_FskCfo;
    g_lora_rx_done++;

    RssiValue = (int8_t)rssi;

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
    SnrValue = LoraSnr_FskCfo;
#else
    SnrValue = 0;
#endif

    if ((payload == NULL) ||
        (size == 0U) ||
        (size > MAX_APP_BUFFER_SIZE))
    {
        RxBufferSize = 0U;

        radio_event = EVENT_RX_ERROR;
    }
    else
    {
        memcpy(
            BufferRx,
            payload,
            size);

        RxBufferSize = size;

        radio_event = EVENT_RX_DONE;
    }

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * RADIO TX TIMEOUT CALLBACK
 * ========================================================= */

static void OnTxTimeout(void)
{
    g_lora_tx_timeout++;

    radio_event = EVENT_TX_TIMEOUT;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * RADIO RX TIMEOUT CALLBACK
 * ========================================================= */

static void OnRxTimeout(void)
{
    radio_event = EVENT_RX_TIMEOUT;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * RADIO RX ERROR CALLBACK
 * ========================================================= */

static void OnRxError(void)
{
    g_lora_rx_error++;

    radio_event = EVENT_RX_ERROR;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * BOARD 1 - HEARTBEAT TIMER
 * ========================================================= */

#if (BOARD_ROLE == BOARD_1_TX)

static void OnHeartbeatEvent(void *context)
{
    (void)context;

    heartbeat_due = 1U;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0);
}

/* =========================================================
 * BOARD 1 - BUILD AND SEND DATA PACKET
 *
 * Payload:
 * [0]    = GPIO mask
 * [1...] = Received RS485 data
 *
 * The RS485 data stays in serial_pending until the ACK
 * arrives (see Board1_ProcessReceived), so it can be
 * resent with the same sequence number after a timeout.
 * ========================================================= */

static void Board1_TransmitPending(void)
{
    uint8_t payload[PROTO_MAX_PAYLOAD];

    uint8_t payload_length = 1U;

    uint8_t sequence;

    uint16_t tx_length;

    /* Wait until previous transmission has finished */
    if ((radio_tx_active != 0U) ||
        (waiting_ack != 0U))
    {
        return;
    }

    /* No serial data and no heartbeat */
    if ((serial_pending_valid == 0U) &&
        (heartbeat_due == 0U))
    {
        return;
    }

    memset(
        payload,
        0,
        sizeof(payload));

    /* Read four opto states */
    payload[0] = Inputs_GetMask() & 0x0FU;

    /* If RS485 data exists: append it after GPIO mask */
    if (serial_pending_valid != 0U)
    {
        payload_length =
            (uint8_t)(1U + serial_pending_length);

        memcpy(
            &payload[1],
            serial_pending,
            serial_pending_length);
    }

    /* Resend uses the same sequence so Board 2 can drop duplicates */
    if (resend_pending != 0U)
    {
        sequence = waiting_sequence;
        resend_pending = 0U;
    }
    else
    {
        sequence = tx_sequence++;
    }

    /* Build AA 55 protocol frame */
    tx_length = Protocol_BuildFrame(
        BufferTx,

        DEVICE_ADDR_BOARD2,
        DEVICE_ADDR_BOARD1,

        CMD_LINK_DATA,

        sequence,

        payload,
        payload_length);

    if (tx_length == 0U)
    {
        return;
    }

    waiting_sequence = sequence;

    /* Remember whether this frame carries RS485 data */
    tx_carries_serial =
        (serial_pending_valid != 0U) ? 1U : 0U;

    heartbeat_due = 0U;

    /* PA11 turns ON inside this function */
    LoRa_StartTx(tx_length);
}

/* =========================================================
 * BOARD 1 - PROCESS ACK
 * ========================================================= */

static void Board1_ProcessReceived(void)
{
    ProtocolFrame frame;

    if (Protocol_DecodeFrame(
            BufferRx,
            RxBufferSize,
            &frame) == 0U)
    {
        return;
    }

    if ((waiting_ack != 0U) &&
        (frame.dst == DEVICE_ADDR_BOARD1) &&
        (frame.src == DEVICE_ADDR_BOARD2) &&
        (frame.cmd == CMD_LINK_ACK) &&
        (frame.len == 0U) &&
        (frame.seq == waiting_sequence))
    {
        /* ACK received */
        waiting_ack = 0U;

        /* Release RS485 data only if this frame carried it */
        if (tx_carries_serial != 0U)
        {
            serial_pending_valid = 0U;
            serial_pending_length = 0U;

            tx_carries_serial = 0U;
        }

        retry_count = 0U;
    }
}

#endif

/* =========================================================
 * BOARD 2 - PROCESS RECEIVED DATA
 * ========================================================= */

#if (BOARD_ROLE == BOARD_2_RX)


static void Board2_ProcessReceived(void)
{
    ProtocolFrame frame;

    uint16_t ack_length;
    uint16_t rf_length;

    /* RF frame: 9-byte overhead + 3-byte payload */
    uint8_t rf_packet[12];
    uint8_t rf_payload[3];

    int16_t rssi;

    /* =====================================================
     * 1. Decode received LoRa packet and check CRC
     * ===================================================== */

    if (Protocol_DecodeFrame(
            BufferRx,
            RxBufferSize,
            &frame) == 0U)
    {
        LoRa_StartRx();
        return;
    }

    /* Accept only DATA from Board 1 */
    if ((frame.dst != DEVICE_ADDR_BOARD2) ||
        (frame.src != DEVICE_ADDR_BOARD1) ||
        (frame.cmd != CMD_LINK_DATA) ||
        (frame.len < 1U))
    {
        LoRa_StartRx();
        return;
    }

    /* =====================================================
     * 2. Forward valid packet to RS485
     *
     * Duplicate packets are not forwarded twice.
     * ===================================================== */

    if ((last_rx_sequence_valid == 0U) ||
        (last_rx_sequence != frame.seq))
    {
        /* =================================================
         * 3. Build RF signal information
         *
         * RF Payload:
         *
         * Byte 0: RSSI LOW
         * Byte 1: RSSI HIGH
         * Byte 2: SNR
         *
         * Command = 0x21
         * Sequence = Same as DATA packet
         * ================================================= */

        rssi = g_rx_rssi_dbm;

        rf_payload[0] =
            (uint8_t)((uint16_t)rssi & 0xFFU);

        rf_payload[1] =
            (uint8_t)(((uint16_t)rssi >> 8U) & 0xFFU);

        rf_payload[2] =
            (uint8_t)g_rx_snr_db;

        rf_length = Protocol_BuildFrame(
            rf_packet,

            DEVICE_ADDR_MASTER,
            DEVICE_ADDR_BOARD2,

            CMD_RF_STATS,

            frame.seq,

            rf_payload,
            sizeof(rf_payload));

        if (rf_length == 0U)
        {
            LoRa_StartRx();
            return;
        }

        /* =================================================
         * 4. Send RF information to PC via RS485
         * ================================================= */

        if (RS485_Send(
                rf_packet,
                rf_length) == 0U)
        {
            LoRa_StartRx();
            return;
        }

        /* =================================================
         * 5. Send COMPLETE original DATA frame
         *
         * IMPORTANT:
         *
         * Keep the complete frame:
         *
         * AA 55 DST SRC CMD SEQ LEN PAYLOAD CRC
         *
         * payload[0]  = GPIO MASK
         * payload[1..] = Temperature / RS485 data
         *
         * Also forward heartbeat packets (LEN = 1).
         * ================================================= */

        if (RS485_Send(
                BufferRx,
                RxBufferSize) == 0U)
        {
            LoRa_StartRx();
            return;
        }

        /* Remember successfully forwarded packet */
        last_rx_sequence = frame.seq;

        last_rx_sequence_valid = 1U;
    }

    /* =====================================================
     * 6. Build ACK
     *
     * ACK is also transmitted for duplicate packets.
     * ===================================================== */

    ack_length = Protocol_BuildFrame(
        BufferTx,

        DEVICE_ADDR_BOARD1,
        DEVICE_ADDR_BOARD2,

        CMD_LINK_ACK,

        frame.seq,

        NULL,
        0U);

    if (ack_length == 0U)
    {
        LoRa_StartRx();
        return;
    }

    /* =====================================================
     * 7. Allow Board 1 to switch from TX to RX
     * ===================================================== */

    HAL_Delay(
        Radio.GetWakeupTime() + RX_TIME_MARGIN);

    /* =====================================================
     * 8. Transmit ACK
     *
     * PA11 turns ON during ACK transmission.
     * ===================================================== */

    LoRa_StartTx(ack_length);
}

#endif

/* =========================================================
 * MAIN LORA STATE MACHINE
 * ========================================================= */

static void LoRa_Process(void)
{
    RadioEventType event = radio_event;

    radio_event = EVENT_NONE;

    switch (event)
    {
        /* ================================================
         * RX DONE
         * ================================================ */

        case EVENT_RX_DONE:
        {
            Radio.Sleep();

#if (BOARD_ROLE == BOARD_1_TX)

            /* Board 1 receives ACK */
            Board1_ProcessReceived();

            LoRa_StartRx();

#else

            /* Board 2 receives DATA */
            Board2_ProcessReceived();

#endif

            break;
        }

        /* ================================================
         * TX DONE
         * ================================================ */

        case EVENT_TX_DONE:
        {
            radio_tx_active = 0U;

            /*
             * Keep LED ON for another LED_TX_HOLD_MS,
             * then timerLed will turn it OFF.
             */
            UTIL_TIMER_Start(&timerLed);

#if (BOARD_ROLE == BOARD_1_TX)

            waiting_ack = 1U;

#endif

            /* Switch radio back to RX */
            Radio.Sleep();

            LoRa_StartRx();

            break;
        }

        /* ================================================
         * RX TIMEOUT
         * ================================================ */

        case EVENT_RX_TIMEOUT:
        {
            Radio.Sleep();

#if (BOARD_ROLE == BOARD_1_TX)

            if (waiting_ack != 0U)
            {
                /* ACK timeout */
                waiting_ack = 0U;

                if (tx_carries_serial != 0U)
                {
                    retry_count++;

                    if (retry_count > LINK_MAX_RETRY)
                    {
                        /* Give up: drop RS485 data */
                        g_lora_drop++;

                        serial_pending_valid = 0U;
                        serial_pending_length = 0U;

                        tx_carries_serial = 0U;
                        retry_count = 0U;
                    }
                    else
                    {
                        /* Resend the same data, same sequence */
                        g_lora_retry++;

                        resend_pending = 1U;
                    }
                }
            }

#endif

            LoRa_StartRx();

            break;
        }

        /* ================================================
         * RX ERROR
         * ================================================ */

        case EVENT_RX_ERROR:
        {
            Radio.Sleep();

            LoRa_StartRx();

            break;
        }

        /* ================================================
         * TX TIMEOUT
         * ================================================ */

        case EVENT_TX_TIMEOUT:
        {
            Radio.Sleep();

            radio_tx_active = 0U;

            UTIL_TIMER_Stop(&timerLed);

            /* Force PA11 OFF on TX error */
            LED_TxOff();

#if (BOARD_ROLE == BOARD_1_TX)

            waiting_ack = 0U;

            /* Radio never sent it: try again with the same sequence */
            if (tx_carries_serial != 0U)
            {
                resend_pending = 1U;
            }

#endif

            LoRa_StartRx();

            break;
        }

        case EVENT_NONE:
        default:
        {
            break;
        }
    }

    /* Board 1 sends the next queued packet */
#if (BOARD_ROLE == BOARD_1_TX)

    Board1_TransmitPending();

#endif
}
