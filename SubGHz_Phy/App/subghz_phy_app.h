
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ================= RADIO MODE ================= */

#define USE_MODEM_LORA 1
#define USE_MODEM_FSK  0

/* ================= RADIO FREQUENCY ================= */

#define RF_FREQUENCY 917300000U

#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER 14
#endif

/* ================= LORA CONFIGURATION ================= */

#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))

#define LORA_BANDWIDTH                0
#define LORA_SPREADING_FACTOR         7
#define LORA_CODINGRATE               1
#define LORA_PREAMBLE_LENGTH          8
#define LORA_SYMBOL_TIMEOUT          5

#define LORA_FIX_LENGTH_PAYLOAD_ON    false
#define LORA_IQ_INVERSION_ON          false

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))

#define FSK_FDEV                      25000
#define FSK_DATARATE                  50000
#define FSK_BANDWIDTH                 50000
#define FSK_PREAMBLE_LENGTH           5

#define FSK_FIX_LENGTH_PAYLOAD_ON     false

#else

#error "Invalid radio configuration"

#endif

#define PAYLOAD_LEN 64U

/* ================= FUNCTIONS ================= */

void SubghzApp_Init(void);

uint8_t SubghzApp_QueueSerial(
    const uint8_t *data,
    uint8_t length);

#ifdef __cplusplus
}
#endif

#endif
