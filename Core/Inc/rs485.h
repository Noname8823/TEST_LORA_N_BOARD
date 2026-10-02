#ifndef RS485_H
#define RS485_H

#include "main.h"
#include <stdint.h>

void RS485_Init(void);
void RS485_Task(void);

uint8_t RS485_Send(
    const uint8_t *data,
    uint16_t length
);

uint8_t RS485_ReadByte(uint8_t *data);
void RS485_FlushRx(void);

/* Debug counters */
extern volatile uint32_t g_rs485_rx_bytes;
extern volatile uint32_t g_rs485_messages;
extern volatile uint32_t g_rs485_overflow;
extern volatile uint32_t g_rs485_errors;
extern volatile uint32_t g_rs485_rearm_fail;
extern volatile uint32_t g_rs485_tx_errors;

extern volatile uint8_t g_rs485_last_byte;
extern volatile uint8_t g_rs485_raw_ring[32];
extern volatile uint8_t g_rs485_raw_wr;

#endif
