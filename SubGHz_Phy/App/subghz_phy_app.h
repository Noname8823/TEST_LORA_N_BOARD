#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#include <stdint.h>
#include <stdbool.h>

/* Radio mode */
#define USE_MODEM_LORA 1
#define USE_MODEM_FSK  0

/* RF frequency */
#define RF_FREQUENCY 433000000U

#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER 14
#endif

/* LoRa configuration */
#define LORA_BANDWIDTH                0
#define LORA_SPREADING_FACTOR         7
#define LORA_CODINGRATE               1
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT          5

#define LORA_FIX_LENGTH_PAYLOAD_ON    false
#define LORA_IQ_INVERSION_ON          false

#define PAYLOAD_LEN                   64U

/* Application API */
void SubghzApp_Init(void);
void SubghzApp_Task(void);

uint8_t SubghzApp_QueueSerial(
    const uint8_t *data,
    uint8_t length
);

#endif
