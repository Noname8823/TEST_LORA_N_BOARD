#ifndef RS485_H
#define RS485_H

#include "main.h"
#include <stdint.h>


void RS485_Init(void);

void RS485_Task(void);

void RS485_Send(uint8_t *data, uint16_t length);


#endif /* RS485_H */
