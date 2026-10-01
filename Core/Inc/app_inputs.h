#ifndef APP_INPUTS_H
#define APP_INPUTS_H

#include "main.h"

#include <stdint.h>


void Inputs_Init(void);

void Inputs_Task(void);


uint8_t Inputs_GetRawMask(void);

uint8_t Inputs_GetMask(void);


#endif /* APP_INPUTS_H */
