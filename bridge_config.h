#define BRIDGE_ROLE BRIDGE_ROLE_RS485_TO_LORA


#ifndef BRIDGE_CONFIG_H
#define BRIDGE_CONFIG_H


/* =========================================================
 * Bridge role
 * ========================================================= */

#define BRIDGE_ROLE_RS485_TO_LORA    1U
#define BRIDGE_ROLE_LORA_TO_RS485    2U


/*
 * =========================================================
 * CHANGE ONLY THIS LINE
 * =========================================================
 *
 * BOARD 1:
 *
 * #define BRIDGE_ROLE BRIDGE_ROLE_RS485_TO_LORA
 *
 * BOARD 2:
 *
 * #define BRIDGE_ROLE BRIDGE_ROLE_LORA_TO_RS485
 */

#define BRIDGE_ROLE BRIDGE_ROLE_RS485_TO_LORA


/* =========================================================
 * Node address
 * ========================================================= */

#if (BRIDGE_ROLE == BRIDGE_ROLE_RS485_TO_LORA)

#define BRIDGE_LOCAL_ID     0x01U
#define BRIDGE_REMOTE_ID    0x02U

#else

#define BRIDGE_LOCAL_ID     0x02U
#define BRIDGE_REMOTE_ID    0x01U

#endif


/* =========================================================
 * Maximum RS485 frame
 * ========================================================= */

#define BRIDGE_RS485_MAX_FRAME       64U

#define BRIDGE_LORA_MAX_PACKET       80U


#endif /* BRIDGE_CONFIG_H */
