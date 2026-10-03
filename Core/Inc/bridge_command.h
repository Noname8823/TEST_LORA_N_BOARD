
#ifndef BRIDGE_COMMAND_H
#define BRIDGE_COMMAND_H

#include <stdint.h>

/*
 * Process local configuration command.
 *
 * Return:
 *
 * 1 = Valid configuration command,
 *     handled locally.
 *
 * 0 = Not a valid configuration frame.
 *     Forward normally through LoRa.
 */

uint8_t BridgeCommand_TryHandle(
    const uint8_t *bytes,
    uint8_t length
);

#endif
