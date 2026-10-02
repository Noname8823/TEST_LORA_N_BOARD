#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Same firmware for both boards */

#define RS485_BAUDRATE           9600U

/* UART buffering */
#define RS485_DATA_MAX          48U
#define RS485_FIFO_SIZE         256U
#define RS485_GAP_MS            6U

/* LoRa reliability */
#define BRIDGE_ACK_DELAY_MS     200U
#define BRIDGE_ACK_TIMEOUT_MS   1200U
#define BRIDGE_MAX_ATTEMPTS     4U

/* PA11 TX LED */
#define LED_TX_HOLD_MS          150U

#endif
