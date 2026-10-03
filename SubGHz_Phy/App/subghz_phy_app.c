
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"
#include "bridge_config.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"

#include "main.h"
#include "board_config.h"
#include "protocol.h"
#include "rs485.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* =========================================================
 * BRIDGE CONFIGURATION
 * ========================================================= */

/*
 * Current point-to-point protocol identifiers.
 * Keep these values unchanged for this latency fix.
 */
#define BRIDGE_DST             0xFFU
#define BRIDGE_SRC             0x42U

#define RX_TIMEOUT_VALUE       3000U
#define TX_TIMEOUT_VALUE       3000U

#define MAX_APP_BUFFER_SIZE    255U
#define BRIDGE_DUP_WINDOW_MS   10000U

/*
 * IMPORTANT:
 *
 * Bridge timer runs continuously.
 * It is independent from LoRa RX timeout.
 */
#define BRIDGE_TICK_MS         10U

#if (RS485_DATA_MAX > PROTO_MAX_PAYLOAD)
#error "RS485 payload too large"
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


typedef enum
{
    TX_KIND_NONE = 0,

    TX_KIND_DATA,
    TX_KIND_ACK

} TxKind;


/* =========================================================
 * RADIO VARIABLES
 * ========================================================= */

static RadioEvents_t RadioEvents;

static volatile RadioEventType radio_event;

static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

static uint16_t RxBufferSize;

static uint8_t radio_tx_active;
static TxKind current_tx_kind;


/* =========================================================
 * TIMERS
 * ========================================================= */

static UTIL_TIMER_Object_t timerLed;
static UTIL_TIMER_Object_t timerBridge;

static uint8_t bridge_timer_active;


/* =========================================================
 * OUTGOING RS485 DATA
 * ========================================================= */

static uint8_t outgoing[RS485_DATA_MAX];

static uint8_t outgoing_len;
static uint8_t outgoing_valid;

static uint8_t outgoing_seq;
static uint8_t next_seq;

static uint8_t send_attempts;
static uint8_t waiting_ack;

static uint32_t ack_deadline_ms;
static uint32_t next_data_ms;


/* =========================================================
 * ACK STATE
 * ========================================================= */

static uint8_t ack_pending;
static uint8_t ack_seq;

static uint32_t ack_due_ms;


/* =========================================================
 * DUPLICATE PROTECTION
 * ========================================================= */

static uint8_t last_rx_valid;
static uint8_t last_rx_seq;

static uint32_t last_rx_ms;


/* =========================================================
 * DEBUG VARIABLES
 * ========================================================= */

int8_t RssiValue;
int8_t SnrValue;

/* Actual configuration passed to Radio */
volatile uint32_t g_rf_applied_freq = 0U;
volatile uint8_t  g_rf_applied_sf   = 0U;
volatile uint8_t  g_rf_applied_bw   = 0U;

/* LoRa debug */
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

/*
 * NEW DEBUG VARIABLES:
 *
 * Verify that the periodic bridge timer
 * is running even while LoRa is idle.
 */
volatile uint32_t g_bridge_tick_count = 0U;
volatile uint32_t g_bridge_process_count = 0U;
volatile uint32_t g_bridge_last_process_ms = 0U;


/* =========================================================
 * FUNCTION PROTOTYPES
 * ========================================================= */

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
static void Bridge_Service(void);


/* =========================================================
 * BRIDGE TIMER CALLBACK
 * ========================================================= */

/*
 * This timer wakes up the STM32 sequencer
 * periodically, independently of radio events.
 *
 * Do not call blocking UART transmit here.
 */

static void Bridge_Tick(void *context)
{
    (void)context;

    g_bridge_tick_count++;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
}


/* =========================================================
 * START BRIDGE TIMER
 * ========================================================= */

static void Bridge_StartTick(void)
{
    /*
     * Normally already running since
     * SubghzApp_Init().
     *
     * Keep this helper for compatibility
     * with existing TX / ACK code.
     */

    if (bridge_timer_active == 0U)
    {
        bridge_timer_active = 1U;

        UTIL_TIMER_Start(&timerBridge);
    }
}


/* =========================================================
 * TIME CHECK
 * ========================================================= */

static uint8_t TimeReached(uint32_t deadline)
{
    return (
        (int32_t)(HAL_GetTick() - deadline) >= 0
    ) ? 1U : 0U;
}


/* =========================================================
 * RETRY JITTER
 * ========================================================= */

static uint32_t RetryJitter(void)
{
    /*
     * Use STM32 unique ID to reduce
     * the probability of simultaneous TX.
     *
     * This does not guarantee
     * collision-free communication.
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


/* =========================================================
 * PA11 TX LED
 * ========================================================= */

static void LED_Off(void *context)
{
    (void)context;

    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET
    );
}


/* =========================================================
 * START LORA RX
 * ========================================================= */

static void LoRa_StartRx(void)
{
    /*
     * RX timeout is independent of UART
     * processing because timerBridge now
     * runs every 10 ms.
     */

    Radio.Rx(RX_TIMEOUT_VALUE);
}


/* =========================================================
 * START LORA TX
 * ========================================================= */

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

    /* PA11 HIGH during LoRa TX */

    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_SET
    );

    Radio.Send(BufferTx, length);
}


/* =========================================================
 * RETRY OR DROP
 * ========================================================= */

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


/* =========================================================
 * QUEUE RAW RS485 DATA
 * ========================================================= */

uint8_t SubghzApp_QueueSerial(
    const uint8_t *data,
    uint8_t length)
{
    if ((data == NULL) ||
        (length == 0U) ||
        (length > RS485_DATA_MAX) ||
        (outgoing_valid != 0U))
    {
        return 0U;
    }

    /* Copy raw UART bytes */

    memcpy(
        outgoing,
        data,
        length
    );

    outgoing_len = length;
    outgoing_valid = 1U;

    /* Sequence number */

    outgoing_seq = next_seq++;

    send_attempts = 0U;

    /* Initial transmission backoff */

    next_data_ms =
        HAL_GetTick() + RetryJitter();

    /*
     * Timer is continuously running.
     * This also handles any legacy stop state.
     */

    Bridge_StartTick();

    /* Request immediate sequencer processing */

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );

    return 1U;
}


/* =========================================================
 * BRIDGE PERIODIC TASK
 * ========================================================= */

void SubghzApp_Task(void)
{
    uint8_t wake = 0U;

    /* Pending ACK */

    if ((ack_pending != 0U) &&
        TimeReached(ack_due_ms))
    {
        wake = 1U;
    }

    /* ACK timeout */

    if ((waiting_ack != 0U) &&
        TimeReached(ack_deadline_ms))
    {
        wake = 1U;
    }

    /* Pending outgoing DATA */

    if ((outgoing_valid != 0U) &&
        (waiting_ack == 0U) &&
        TimeReached(next_data_ms))
    {
        wake = 1U;
    }

    if (wake != 0U)
    {
        UTIL_SEQ_SetTask(
            (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
            CFG_SEQ_Prio_0
        );
    }
}


/* =========================================================
 * LORA INITIALIZATION
 * ========================================================= */

void SubghzApp_Init(void)
{
    /*
     * Load ACTIVE configuration.
     *
     * BridgeConfig_Init() must already
     * have been called in main.c.
     */

    const BridgeSettings *cfg = BridgeConfig_Get();


    /* -----------------------------------------------------
     * Reset radio buffers
     * ----------------------------------------------------- */

    memset(
        BufferRx,
        0,
        sizeof(BufferRx)
    );

    memset(
        BufferTx,
        0,
        sizeof(BufferTx)
    );

    RxBufferSize = 0U;

    radio_event = EVENT_NONE;

    radio_tx_active = 0U;
    current_tx_kind = TX_KIND_NONE;


    /* -----------------------------------------------------
     * Reset bridge state
     * ----------------------------------------------------- */

    bridge_timer_active = 0U;

    outgoing_valid = 0U;
    outgoing_len = 0U;

    next_seq = 0U;

    waiting_ack = 0U;
    send_attempts = 0U;

    ack_pending = 0U;

    last_rx_valid = 0U;

    g_bridge_tick_count = 0U;
    g_bridge_process_count = 0U;
    g_bridge_last_process_ms = 0U;


    /* -----------------------------------------------------
     * PA11 LED OFF
     * ----------------------------------------------------- */

    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET
    );


    /* -----------------------------------------------------
     * Create TX LED timer
     * ----------------------------------------------------- */

    UTIL_TIMER_Create(
        &timerLed,
        LED_TX_HOLD_MS,
        UTIL_TIMER_ONESHOT,
        LED_Off,
        NULL
    );


    /* -----------------------------------------------------
     * Create periodic Bridge timer
     *
     * IMPORTANT:
     *
     * This timer is never stopped when
     * the application becomes idle.
     * ----------------------------------------------------- */

    UTIL_TIMER_Create(
        &timerBridge,
        BRIDGE_TICK_MS,
        UTIL_TIMER_PERIODIC,
        Bridge_Tick,
        NULL
    );


    /* -----------------------------------------------------
     * Register radio callbacks
     * ----------------------------------------------------- */

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;

    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);


    /* -----------------------------------------------------
     * Record applied RF configuration
     * ----------------------------------------------------- */

    g_rf_applied_freq = cfg->frequency;

    g_rf_applied_sf = cfg->sf;

    g_rf_applied_bw = cfg->bandwidth;


    /* -----------------------------------------------------
     * Apply RF frequency loaded from Flash
     * ----------------------------------------------------- */

    Radio.SetChannel(cfg->frequency);


#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

    /* -----------------------------------------------------
     * LoRa TX configuration
     * ----------------------------------------------------- */

    Radio.SetTxConfig(
        MODEM_LORA,

        cfg->power,

        0,

        cfg->bandwidth,

        cfg->sf,

        cfg->cr,

        LORA_PREAMBLE_LENGTH,

        LORA_FIX_LENGTH_PAYLOAD_ON,

        true,

        0,

        0,

        LORA_IQ_INVERSION_ON,

        TX_TIMEOUT_VALUE
    );


    /* -----------------------------------------------------
     * LoRa RX configuration
     * ----------------------------------------------------- */

    Radio.SetRxConfig(
        MODEM_LORA,

        cfg->bandwidth,

        cfg->sf,

        cfg->cr,

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


    /* -----------------------------------------------------
     * Register application task
     * ----------------------------------------------------- */

    UTIL_SEQ_RegTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),

        UTIL_SEQ_RFU,

        LoRa_Process
    );


    /* -----------------------------------------------------
     * Start in RX mode
     * ----------------------------------------------------- */

    LoRa_StartRx();


    /* =====================================================
     * IMPORTANT FIX
     *
     * Start continuous 10 ms bridge timer.
     *
     * Do this AFTER registering LoRa_Process().
     * ===================================================== */

    bridge_timer_active = 1U;

    UTIL_TIMER_Start(&timerBridge);
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
        CFG_SEQ_Prio_0
    );
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
        memcpy(
            BufferRx,
            payload,
            size
        );

        RxBufferSize = size;

        radio_event = EVENT_RX_DONE;
    }

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
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
        CFG_SEQ_Prio_0
    );
}


/* =========================================================
 * RADIO RX TIMEOUT CALLBACK
 * ========================================================= */

static void OnRxTimeout(void)
{
    radio_event = EVENT_RX_TIMEOUT;

    UTIL_SEQ_SetTask(
        (1UL << CFG_SEQ_Task_SubGHz_Phy_App_Process),
        CFG_SEQ_Prio_0
    );
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
        CFG_SEQ_Prio_0
    );
}


/* =========================================================
 * PROCESS RECEIVED LORA DATA
 * ========================================================= */

static void ProcessReceived(void)
{
    ProtocolFrame frame;

    uint32_t now = HAL_GetTick();


    /* -----------------------------------------------------
     * Decode internal LoRa frame
     * ----------------------------------------------------- */

    if (!Protocol_DecodeFrame(
            BufferRx,
            RxBufferSize,
            &frame))
    {
        return;
    }


    /* -----------------------------------------------------
     * Keep original point-to-point filtering
     * ----------------------------------------------------- */

    if ((frame.dst != BRIDGE_DST) ||
        (frame.src != BRIDGE_SRC))
    {
        return;
    }


    /* -----------------------------------------------------
     * Received ACK
     * ----------------------------------------------------- */

    if (frame.cmd == CMD_LINK_ACK)
    {
        if ((frame.len == 0U) &&
            (waiting_ack != 0U) &&
            (outgoing_valid != 0U) &&
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


    /* -----------------------------------------------------
     * Accept only DATA
     * ----------------------------------------------------- */

    if ((frame.cmd != CMD_LINK_DATA) ||
        (frame.len == 0U) ||
        (frame.len > RS485_DATA_MAX))
    {
        return;
    }


    /* -----------------------------------------------------
     * Duplicate detection
     * ----------------------------------------------------- */

    if ((last_rx_valid != 0U) &&
        (last_rx_seq == frame.seq) &&
        ((uint32_t)(now - last_rx_ms)
          < BRIDGE_DUP_WINDOW_MS))
    {
        g_lora_dup_rx++;
    }
    else
    {
        /*
         * Forward ONLY raw payload bytes.
         *
         * Do not output:
         *
         * - LoRa header
         * - Node address
         * - Internal CRC
         * - RF statistics
         * - ACK
         */

        if (RS485_Send(
                frame.payload,
                frame.len) == 0U)
        {
            /*
             * UART TX failed.
             * Do not acknowledge this packet.
             */

            return;
        }


        /* Update duplicate protection */

        last_rx_seq = frame.seq;

        last_rx_ms = HAL_GetTick();

        last_rx_valid = 1U;

        g_lora_uart_forward++;
    }


    /* -----------------------------------------------------
     * Schedule ACK
     * ----------------------------------------------------- */

    ack_seq = frame.seq;

    ack_pending = 1U;

    ack_due_ms =
        HAL_GetTick() + BRIDGE_ACK_DELAY_MS;


    Bridge_StartTick();


    if (waiting_ack != 0U)
    {
        ack_deadline_ms =
            HAL_GetTick() + BRIDGE_ACK_TIMEOUT_MS;
    }
}


/* =========================================================
 * BRIDGE TRANSMISSION STATE MACHINE
 * ========================================================= */

static void Bridge_Service(void)
{
    uint16_t length;


    /* Radio is currently transmitting */

    if (radio_tx_active != 0U)
    {
        return;
    }


    /* -----------------------------------------------------
     * 1. ACK has priority
     * ----------------------------------------------------- */

    if (ack_pending != 0U)
    {
        if (!TimeReached(ack_due_ms))
        {
            return;
        }


        length = Protocol_BuildFrame(
            BufferTx,

            BRIDGE_DST,
            BRIDGE_SRC,

            CMD_LINK_ACK,

            ack_seq,

            NULL,
            0U
        );


        if (length == 0U)
        {
            return;
        }


        ack_pending = 0U;

        LoRa_StartTx(
            length,
            TX_KIND_ACK
        );

        return;
    }


    /* -----------------------------------------------------
     * 2. ACK timeout
     * ----------------------------------------------------- */

    if (waiting_ack != 0U)
    {
        if (!TimeReached(ack_deadline_ms))
        {
            return;
        }

        RetryOrDrop();
    }


    /* -----------------------------------------------------
     * 3. Check outgoing data
     * ----------------------------------------------------- */

    if ((outgoing_valid == 0U) ||
        (waiting_ack != 0U) ||
        (!TimeReached(next_data_ms)))
    {
        return;
    }


    /* -----------------------------------------------------
     * 4. Build DATA frame
     * ----------------------------------------------------- */

    length = Protocol_BuildFrame(
        BufferTx,

        BRIDGE_DST,
        BRIDGE_SRC,

        CMD_LINK_DATA,

        outgoing_seq,

        outgoing,
        outgoing_len
    );


    if (length == 0U)
    {
        g_lora_drop++;

        outgoing_valid = 0U;
        outgoing_len = 0U;

        return;
    }


    /* -----------------------------------------------------
     * 5. Start LoRa TX
     * ----------------------------------------------------- */

    send_attempts++;

    LoRa_StartTx(
        length,
        TX_KIND_DATA
    );
}


/* =========================================================
 * MAIN RADIO PROCESS
 * ========================================================= */

static void LoRa_Process(void)
{
    RadioEventType event = radio_event;

    TxKind finished_kind;

    radio_event = EVENT_NONE;


    /* =====================================================
     * IMPORTANT FIX:
     *
     * Called periodically every 10 ms.
     *
     * Handle incoming RS485 data without waiting
     * for the next LoRa RX timeout.
     * ===================================================== */

    g_bridge_process_count++;

    g_bridge_last_process_ms = HAL_GetTick();

    RS485_Task();


    /* =====================================================
     * PROCESS RADIO EVENTS
     * ===================================================== */

    switch (event)
    {
        /* -------------------------------------------------
         * RX DONE
         * ------------------------------------------------- */

        case EVENT_RX_DONE:
        {
            Radio.Sleep();

            ProcessReceived();

            LoRa_StartRx();

            break;
        }


        /* -------------------------------------------------
         * TX DONE
         * ------------------------------------------------- */

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
                     (waiting_ack != 0U))
            {
                ack_deadline_ms =
                    HAL_GetTick() +
                    BRIDGE_ACK_TIMEOUT_MS;
            }


            Radio.Sleep();

            LoRa_StartRx();

            break;
        }


        /* -------------------------------------------------
         * TX TIMEOUT
         * ------------------------------------------------- */

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


        /* -------------------------------------------------
         * RX TIMEOUT / RX ERROR
         * ------------------------------------------------- */

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


    /* =====================================================
     * PROCESS PENDING DATA / ACK / RETRY
     * ===================================================== */

    Bridge_Service();


    /*
     * IMPORTANT:
     *
     * Do not stop timerBridge here.
     *
     * The 10 ms timer must keep running even when:
     *
     * - No LoRa packet is being transmitted
     * - No ACK is pending
     * - No UART data is waiting
     *
     * Otherwise local WinForms commands can suffer
     * long processing delays again.
     */
}
