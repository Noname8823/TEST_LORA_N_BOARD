#ifndef WIND_SENSOR_H
#define WIND_SENSOR_H

#include <stdint.h>

/*
 * RK100-02
 *
 * Interface : RS485
 * Protocol  : Modbus RTU
 * Slave ID  : 1
 * Baudrate  : 9600
 * UART      : 8N1
 *
 * Read:
 * Function  : 0x03
 * Register  : 0x0000
 * Quantity  : 1
 */

void Wind_Init(void);

void Wind_Task(void);

/* Called from UART interrupt */
void Wind_OnByte(uint8_t b);

void Wind_OnUartError(void);

/* Debug variables */
extern volatile uint32_t g_wind_requests;

extern volatile uint32_t g_wind_ok;

extern volatile uint32_t g_wind_timeout;

extern volatile uint32_t g_wind_crc_error;

extern volatile uint32_t g_wind_frame_error;

extern volatile uint32_t g_wind_exception;

extern volatile uint8_t g_wind_exception_code;

extern volatile uint32_t g_wind_uart_error;

extern volatile uint32_t g_wind_queue_wait;

/* Wind speed multiplied by 10 */
extern volatile uint16_t g_wind_speed_x10;

extern volatile uint8_t g_wind_valid;

extern volatile uint32_t g_wind_last_update_ms;

extern volatile uint8_t g_wind_last_response[7];

#endif
