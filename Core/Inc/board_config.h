#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Board role */
#define BOARD_1_TX  1U
#define BOARD_2_RX  2U

#ifndef BOARD_ROLE
#define BOARD_ROLE BOARD_2_RX
#endif

#if ((BOARD_ROLE != BOARD_1_TX) && \
     (BOARD_ROLE != BOARD_2_RX))
#error "Invalid BOARD_ROLE"
#endif

/* RS485 modes */
#define RS485_MODE_RAW         0U
#define RS485_MODE_RK100_02    1U

#ifndef RS485_MODE
#define RS485_MODE RS485_MODE_RK100_02
#endif

#if ((RS485_MODE != RS485_MODE_RAW) && \
     (RS485_MODE != RS485_MODE_RK100_02))
#error "Invalid RS485_MODE"
#endif

/* UART baudrate */
#if ((BOARD_ROLE == BOARD_1_TX) && \
     (RS485_MODE == RS485_MODE_RK100_02))

#define RS485_BAUDRATE 9600U

#else

#define RS485_BAUDRATE 115200U

#endif

#define RS485_GAP_MS       25U
#define RS485_DATA_MAX     47U

/* RK100-02 configuration */
#define WIND_SLAVE_ID             0x01U
#define WIND_POLL_PERIOD_MS       3000U
#define WIND_RESPONSE_TIMEOUT_MS  500U

/* LoRa heartbeat */
#define HEARTBEAT_PERIOD_MS 5000U

#define LED_TX_HOLD_MS 150U

#endif
