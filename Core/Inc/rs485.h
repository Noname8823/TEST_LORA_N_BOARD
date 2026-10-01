
#ifndef RS485_H
#define RS485_H

#include "main.h"
#include <stdint.h>

void RS485_Init(void);

void RS485_Task(void);

uint8_t RS485_Send(
    const uint8_t *data,
    uint16_t length);

#endif
