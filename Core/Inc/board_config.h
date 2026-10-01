
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ================= BOARD ROLE ================= */

#define BOARD_1_TX  1U
#define BOARD_2_RX  2U

/*
 * BOARD 1: BOARD_1_TX
 * BOARD 2: BOARD_2_RX
 */
#define BOARD_ROLE BOARD_2_RX

#if ((BOARD_ROLE != BOARD_1_TX) && \
     (BOARD_ROLE != BOARD_2_RX))
#error "Invalid BOARD_ROLE"
#endif

/* ================= RS485 ================= */

#define RS485_BAUDRATE       115200U
#define RS485_GAP_MS         25U

#define RS485_DATA_MAX       47U

/* ================= LORA ================= */

#define HEARTBEAT_PERIOD_MS  3000U

/* Hold TX LED after transmission to make it visible */
#define LED_TX_HOLD_MS       150U

#endif
