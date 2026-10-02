#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"

#include "main.h"
#include "board_config.h"
#include "protocol.h"
#include "rs485.h"

#include <string.h>

/* Point-to-point: exactly two identical nodes */
#define BRIDGE_DST             0xFFU
#define BRIDGE_SRC             0x42U

#define RX_TIMEOUT_VALUE       3000U
#define TX_TIMEOUT_VALUE       3000U

#define MAX_APP_BUFFER_SIZE    255U
#define BRIDGE_DUP_WINDOW_MS   10000U

#if (RS485_DATA_MAX > PROTO_MAX_PAYLOAD)
#error "RS485 payload too large"
#endif

/* =========================================
 * EVENT DEFINITIONS
 * ========================================= */

typedef enum
{
    EVENT_NONE = 0,

    EVENT_RX_DONE,
    EVENT_RX_TIMEOUT,
    EVENT_RX_ERROR,

    EVENT_TX_DONE,
    EVENT_TX_TIMEOUT

} RadioEventType;

typedef enum
{
    TX_KIND_NONE = 0,
    TX_KIND_DATA,
    TX_KIND_ACK

} TxKind;

/* =========================================
 * RADIO VARIABLES
 * ========================================= */

static RadioEvents_t RadioEvents;

static volatile RadioEventType radio_event;

static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

static uint16_t RxBufferSize;

static uint8_t radio_tx_active;
static TxKind current_tx_kind;

static UTIL_TIMER_Object_t timerLed;
static UTIL_TIMER_Object_t timerBridge;

static uint8_t bridge_timer_active;

/* =========================================
 * OUTGOING DATA
 * ========================================= */

static uint8_t outgoing[RS485_DATA_MAX];

static uint8_t outgoing_len;
static uint8_t outgoing_valid;

static uint8_t outgoing_seq;
static uint8_t next_seq;

static uint8_t send_attempts;
static uint8_t waiting_ack;

static uint32_t ack_deadline_ms;
static uint32_t next_data_ms;

/* =========================================
 * ACK STATE
 * ========================================= */

static uint8_t ack_pending;
static uint8_t ack_seq;

static uint32_t ack_due_ms;

/* Duplicate protection */

static uint8_t last_rx_valid;
static uint8_t last_rx_seq;

static uint32_t last_rx_ms;

/* =========================================
 * DEBUG
 * ========================================= */

int8_t RssiValue;
int8_t SnrValue;

volatile uint32_t g_lora_tx_start;
volatile uint32_t g_lora_tx_done;
volatile uint32_t g_lora_rx_done;

volatile uint32_t g_lora_tx_timeout;
volatile uint32_t g_lora_rx_error;

volatile uint32_t g_lora_retry;
volatile uint32_t g_lora_drop;

volatile uint32_t g_lora_ack_rx;
volatile uint32_t g_lora_dup_rx;

volatile uint32_t g_lora_uart_forward;

/* =========================================
 * PROTOTYPES
 * ========================================= */

static void OnTxDone(void);

static void OnRxDone(
    uint8_t *payload,
    uint16_t size,
    int16_t rssi,
    int8_t LoraSnr_FskCfo
);

static void OnTxTimeout(void);
static void OnRxTimeout(void);
static void OnRxError(void);

static void LoRa_Process(void);

/* =========================================
 * BRIDGE TIMER
 * ========================================= */

static void Bridge_Tick(void *context)
{
    (void)context;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

static void Bridge_StartTick(void)
{
    if (!bridge_timer_active)
    {
        bridge_timer_active = 1U;

        UTIL_TIMER_Start(&timerBridge);
    }
}

/* =========================================
 * TIME CHECK
 * ========================================= */

static uint8_t TimeReached(uint32_t deadline)
{
    return (
        (int32_t)(HAL_GetTick() - deadline) >= 0
    ) ? 1U : 0U;
}

/* =========================================
 * RETRY BACKOFF
 * ========================================= */

static uint32_t RetryJitter(void)
{
    /*
     * Use chip UID to reduce the chance
     * of both boards starting TX together.
     *
     * This does not guarantee collision-free TX.
     */

    uint32_t mix =
        HAL_GetUIDw0() ^
        HAL_GetUIDw1() ^
        HAL_GetUIDw2();

    mix ^= HAL_GetTick() +
        ((uint32_t)send_attempts * 0x9E3779B9UL);

    mix ^= mix >> 16;

    mix *= 0x7FEB352DUL;

    mix ^= mix >> 15;

    return 35U + (mix % 180U);
}

/* =========================================
 * PA11 TX LED
 * ========================================= */

static void LED_Off(void *context)
{
    (void)context;

    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET
    );
}

/* =========================================
 * START RX
 * ========================================= */

static void LoRa_StartRx(void)
{
    Radio.Rx(RX_TIMEOUT_VALUE);
}

/* =========================================
 * START TX
 * ========================================= */

static void LoRa_StartTx(
    uint16_t length,
    TxKind kind)
{
    if ((length == 0U) ||
        (length > MAX_APP_BUFFER_SIZE))
    {
        return;
    }

    UTIL_TIMER_Stop(&timerLed);

    Radio.Sleep();

    radio_tx_active = 1U;
    current_tx_kind = kind;

    g_lora_tx_start++;

    /* PA11 HIGH during TX */
    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_SET
    );

    Radio.Send(BufferTx, length);
}

/* =========================================
 * RETRY OR DROP
 * ========================================= */

static void RetryOrDrop(void)
{
    waiting_ack = 0U;

    if (send_attempts >= BRIDGE_MAX_ATTEMPTS)
    {
        g_lora_drop++;

        outgoing_valid = 0U;
        outgoing_len = 0U;

        send_attempts = 0U;
    }
    else
    {
        g_lora_retry++;

        next_data_ms =
            HAL_GetTick() + RetryJitter();
    }
}

/* =========================================
 * QUEUE RAW RS485 DATA
 * ========================================= */

uint8_t SubghzApp_QueueSerial(
    const uint8_t *data,
    uint8_t length)
{
    if ((data == NULL) ||
        (length == 0U) ||
        (length > RS485_DATA_MAX) ||
        outgoing_valid)
    {
        return 0U;
    }

    memcpy(outgoing, data, length);

    outgoing_len = length;
    outgoing_valid = 1U;

    outgoing_seq = next_seq++;

    send_attempts = 0U;

    /* Initial backoff */
    next_data_ms =
        HAL_GetTick() + RetryJitter();

    Bridge_StartTick();

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );

    return 1U;
}

/* =========================================
 * BRIDGE PERIODIC TASK
 * ========================================= */

void SubghzApp_Task(void)
{
    uint8_t wake = 0U;

    if (ack_pending &&
        TimeReached(ack_due_ms))
    {
        wake = 1U;
    }

    if (waiting_ack &&
        TimeReached(ack_deadline_ms))
    {
        wake = 1U;
    }

    if (outgoing_valid &&
        !waiting_ack &&
        TimeReached(next_data_ms))
    {
        wake = 1U;
    }

    if (wake)
    {
        UTIL_SEQ_SetTask(
            (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
            CFG_SEQ_Prio_0
        );
    }
}

/* =========================================
 * LORA INIT
 * ========================================= */

void SubghzApp_Init(void)
{
    memset(BufferRx, 0, sizeof(BufferRx));
    memset(BufferTx, 0, sizeof(BufferTx));

    RxBufferSize = 0U;

    radio_event = EVENT_NONE;

    radio_tx_active = 0U;
    current_tx_kind = TX_KIND_NONE;

    bridge_timer_active = 0U;

    outgoing_valid = 0U;
    outgoing_len = 0U;

    next_seq = 0U;

    waiting_ack = 0U;
    send_attempts = 0U;

    ack_pending = 0U;
    last_rx_valid = 0U;

    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET
    );

    UTIL_TIMER_Create(
        &timerLed,
        LED_TX_HOLD_MS,
        UTIL_TIMER_ONESHOT,
        LED_Off,
        NULL
    );

    /*
     * Keeps ACK and retry deadlines alive
     * while STM32 sequencer is idle.
     */
    UTIL_TIMER_Create(
        &timerBridge,
        10U,
        UTIL_TIMER_PERIODIC,
        Bridge_Tick,
        NULL
    );

    /* Radio callbacks */
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;

    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

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
        TX_TIMEOUT_VALUE
    );

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
        true
    );

    Radio.SetMaxPayloadLength(
        MODEM_LORA,
        MAX_APP_BUFFER_SIZE
    );

#else
#error "This bridge implementation is LoRa-only"
#endif

    UTIL_SEQ_RegTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        UTIL_SEQ_RFU,
        LoRa_Process
    );

    /* Both boards start in RX */
    LoRa_StartRx();
}

/* =========================================
 * RADIO CALLBACKS
 * ========================================= */

static void OnTxDone(void)
{
    g_lora_tx_done++;

    radio_event = EVENT_TX_DONE;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

static void OnRxDone(
    uint8_t *payload,
    uint16_t size,
    int16_t rssi,
    int8_t LoraSnr_FskCfo)
{
    RssiValue = (int8_t)rssi;
    SnrValue = LoraSnr_FskCfo;

    g_lora_rx_done++;

    if ((payload == NULL) ||
        (size == 0U) ||
        (size > sizeof(BufferRx)))
    {
        RxBufferSize = 0U;
        radio_event = EVENT_RX_ERROR;
    }
    else
    {
        memcpy(BufferRx, payload, size);

        RxBufferSize = size;

        radio_event = EVENT_RX_DONE;
    }

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

static void OnTxTimeout(void)
{
    g_lora_tx_timeout++;

    radio_event = EVENT_TX_TIMEOUT;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

static void OnRxTimeout(void)
{
    radio_event = EVENT_RX_TIMEOUT;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

static void OnRxError(void)
{
    g_lora_rx_error++;

    radio_event = EVENT_RX_ERROR;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}

/* =========================================
 * PROCESS RECEIVED LORA DATA
 * ========================================= */

static void ProcessReceived(void)
{
    ProtocolFrame frame;

    uint32_t now = HAL_GetTick();

    if (!Protocol_DecodeFrame(
            BufferRx,
            RxBufferSize,
            &frame))
    {
        return;
    }

    /* Accept only paired bridge packets */
    if ((frame.dst != BRIDGE_DST) ||
        (frame.src != BRIDGE_SRC))
    {
        return;
    }

    /* -------------------------------------
     * Received ACK
     * ------------------------------------- */

    if (frame.cmd == CMD_LINK_ACK)
    {
        if ((frame.len == 0U) &&
            waiting_ack &&
            outgoing_valid &&
            (frame.seq == outgoing_seq))
        {
            waiting_ack = 0U;

            outgoing_valid = 0U;
            outgoing_len = 0U;

            send_attempts = 0U;

            g_lora_ack_rx++;
        }

        return;
    }

    /* Accept only DATA packets */
    if ((frame.cmd != CMD_LINK_DATA) ||
        (frame.len == 0U))
    {
        return;
    }

    /* -------------------------------------
     * Duplicate detection
     * ------------------------------------- */

    if (last_rx_valid &&
        (last_rx_seq == frame.seq) &&
        ((uint32_t)(now - last_rx_ms)
          < BRIDGE_DUP_WINDOW_MS))
    {
        g_lora_dup_rx++;
    }
    else
    {
        /*
         * Output ONLY raw payload bytes.
         *
         * Never forward:
         * - AA 55
         * - LoRa packet header
         * - Internal CRC
         * - ACK
         * - RF statistics
         */

        if (!RS485_Send(
                frame.payload,
                frame.len))
        {
            /* Failed UART transmission: do not ACK */
            return;
        }

        last_rx_seq = frame.seq;
        last_rx_ms = HAL_GetTick();

        last_rx_valid = 1U;

        g_lora_uart_forward++;
    }

    /* -------------------------------------
     * Schedule ACK
     * ------------------------------------- */

    ack_seq = frame.seq;
    ack_pending = 1U;

    ack_due_ms =
        HAL_GetTick() + BRIDGE_ACK_DELAY_MS;

    Bridge_StartTick();

    if (waiting_ack)
    {
        ack_deadline_ms =
            HAL_GetTick() + BRIDGE_ACK_TIMEOUT_MS;
    }
}

/* =========================================
 * TRANSMISSION STATE MACHINE
 * ========================================= */

static void Bridge_Service(void)
{
    uint16_t length;

    if (radio_tx_active)
        return;

    /*
     * ACK has priority over new UART traffic.
     */
    if (ack_pending)
    {
        if (!TimeReached(ack_due_ms))
            return;

        length = Protocol_BuildFrame(
            BufferTx,
            BRIDGE_DST,
            BRIDGE_SRC,
            CMD_LINK_ACK,
            ack_seq,
            NULL,
            0U
        );

        if (!length)
            return;

        ack_pending = 0U;

        LoRa_StartTx(length, TX_KIND_ACK);

        return;
    }

    /* -------------------------------------
     * ACK timeout
     * ------------------------------------- */

    if (waiting_ack)
    {
        if (!TimeReached(ack_deadline_ms))
            return;

        RetryOrDrop();
    }

    /* -------------------------------------
     * No outgoing data or still waiting
     * ------------------------------------- */

    if (!outgoing_valid ||
        waiting_ack ||
        !TimeReached(next_data_ms))
    {
        return;
    }

    /* -------------------------------------
     * Build internal DATA packet
     * ------------------------------------- */

    length = Protocol_BuildFrame(
        BufferTx,
        BRIDGE_DST,
        BRIDGE_SRC,
        CMD_LINK_DATA,
        outgoing_seq,
        outgoing,
        outgoing_len
    );

    if (!length)
    {
        g_lora_drop++;

        outgoing_valid = 0U;

        return;
    }

    send_attempts++;

    LoRa_StartTx(length, TX_KIND_DATA);
}

/* =========================================
 * MAIN RADIO PROCESS
 * ========================================= */

static void LoRa_Process(void)
{
    RadioEventType event = radio_event;

    TxKind finished_kind;

    radio_event = EVENT_NONE;

    switch (event)
    {
        /* ---------------------------------
         * RX DONE
         * --------------------------------- */

        case EVENT_RX_DONE:
        {
            Radio.Sleep();

            ProcessReceived();

            LoRa_StartRx();

            break;
        }

        /* ---------------------------------
         * TX DONE
         * --------------------------------- */

        case EVENT_TX_DONE:
        {
            radio_tx_active = 0U;

            finished_kind = current_tx_kind;
            current_tx_kind = TX_KIND_NONE;

            UTIL_TIMER_Start(&timerLed);

            if (finished_kind == TX_KIND_DATA)
            {
                waiting_ack = 1U;

                ack_deadline_ms =
                    HAL_GetTick() +
                    BRIDGE_ACK_TIMEOUT_MS;
            }
            else if ((finished_kind == TX_KIND_ACK) &&
                     waiting_ack)
            {
                ack_deadline_ms =
                    HAL_GetTick() +
                    BRIDGE_ACK_TIMEOUT_MS;
            }

            Radio.Sleep();

            LoRa_StartRx();

            break;
        }

        /* ---------------------------------
         * TX TIMEOUT
         * --------------------------------- */

        case EVENT_TX_TIMEOUT:
        {
            radio_tx_active = 0U;

            finished_kind = current_tx_kind;
            current_tx_kind = TX_KIND_NONE;

            UTIL_TIMER_Stop(&timerLed);

            HAL_GPIO_WritePin(
                Led_Signal_GPIO_Port,
                Led_Signal_Pin,
                GPIO_PIN_RESET
            );

            Radio.Sleep();

            LoRa_StartRx();

            if (finished_kind == TX_KIND_DATA)
            {
                RetryOrDrop();
            }
            else if (finished_kind == TX_KIND_ACK)
            {
                ack_pending = 1U;

                ack_due_ms =
                    HAL_GetTick() +
                    BRIDGE_ACK_DELAY_MS;
            }

            break;
        }

        /* ---------------------------------
         * RX TIMEOUT / RX ERROR
         * --------------------------------- */

        case EVENT_RX_TIMEOUT:
        case EVENT_RX_ERROR:
        {
            Radio.Sleep();

            LoRa_StartRx();

            break;
        }

        case EVENT_NONE:
        default:
        {
            break;
        }
    }

    /* Process pending ACK / DATA / retries */
    Bridge_Service();

    /* Stop timer when the bridge is idle */
    if (bridge_timer_active &&
        !outgoing_valid &&
        !waiting_ack &&
        !ack_pending &&
        !radio_tx_active)
    {
        UTIL_TIMER_Stop(&timerBridge);

        bridge_timer_active = 0U;
    }
}
