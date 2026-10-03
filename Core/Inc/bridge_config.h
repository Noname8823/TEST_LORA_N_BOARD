
#ifndef BRIDGE_CONFIG_H
#define BRIDGE_CONFIG_H

#include <stdint.h>

/*
 * Module configuration.
 *
 * Node ID : 1..254
 * Dest ID : 1..254
 *
 * All boards run the same firmware.
 */

typedef struct
{
    uint8_t node_id;

    uint8_t dest_id;

    uint32_t frequency;

    /* 0=125kHz, 1=250kHz, 2=500kHz */
    uint8_t bandwidth;

    /* SF7..SF12 */
    uint8_t sf;

    /* 1=4/5, 2=4/6, 3=4/7, 4=4/8 */
    uint8_t cr;

    /* TX power in dBm */
    int8_t power;

} BridgeSettings;


/* Load configuration from Flash */
void BridgeConfig_Init(void);

/* Active configuration loaded at startup */
const BridgeSettings *BridgeConfig_Get(void);

/* Pending configuration */
const BridgeSettings *BridgeConfig_GetPending(void);

/* Set pending configuration in RAM */
uint8_t BridgeConfig_Set(
    const BridgeSettings *cfg
);

/* Save pending configuration to Flash */
uint8_t BridgeConfig_Save(void);

#endif
