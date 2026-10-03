
#include "bridge_command.h"
#include "bridge_config.h"

#include "app_inputs.h"
#include "protocol.h"
#include "rs485.h"
#include "main.h"

#include <string.h>


/* =========================================
 * COMMAND DEFINITIONS
 * ========================================= */

#define CFG_PING       0x01U
#define CFG_GET        0x02U
#define CFG_SET        0x03U
#define CFG_SAVE       0x04U
#define CFG_INPUTS     0x05U
#define CFG_REBOOT     0x06U


/* =========================================
 * STATUS DEFINITIONS
 * ========================================= */

#define CFG_OK         0x00U
#define CFG_BAD_ARG    0x01U
#define CFG_FLASH_ERR  0x02U
#define CFG_UNKNOWN    0x03U


/* =========================================
 * UINT32 LITTLE ENDIAN
 * ========================================= */

static uint32_t Get32(const uint8_t *p)
{
    return
        (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}


static void Put32(
    uint8_t *p,
    uint32_t v)
{
    p[0] = (uint8_t)v;

    p[1] = (uint8_t)(v >> 8);

    p[2] = (uint8_t)(v >> 16);

    p[3] = (uint8_t)(v >> 24);
}


/* =========================================
 * SEND CONFIGURATION RESPONSE
 * ========================================= */

static void Reply(
    uint8_t cmd,
    const uint8_t *p,
    uint8_t len)
{
    uint8_t b[40];

    uint16_t crc;

    if (len > 32U)
    {
        return;
    }


    /* Magic header */

    b[0] = 0xC3U;
    b[1] = 0x3CU;
    b[2] = 0xA5U;
    b[3] = 0x5AU;


    /* Response command */

    b[4] = (uint8_t)(cmd | 0x80U);

    b[5] = len;


    /* Copy payload */

    if (len > 0U)
    {
        memcpy(
            &b[6],
            p,
            len
        );
    }


    /* CRC over CMD + LEN + PAYLOAD */

    crc = Protocol_CRC16(
        &b[4],
        (uint16_t)(2U + len)
    );


    /* CRC low byte first */

    b[6U + len] = (uint8_t)crc;

    b[7U + len] = (uint8_t)(crc >> 8);


    /* Send response through local RS485 */

    (void)RS485_Send(
        b,
        (uint16_t)(8U + len)
    );
}


/* =========================================
 * HANDLE CONFIGURATION COMMAND
 * ========================================= */

uint8_t BridgeCommand_TryHandle(
    const uint8_t *b,
    uint8_t n)
{
    uint8_t cmd;

    uint8_t len;

    uint8_t reply[16] = {0U};

    uint8_t reply_len = 1U;

    uint16_t recv_crc;

    uint16_t calc_crc;

    const BridgeSettings *cfg;

    BridgeSettings update;


    /* -------------------------------------
     * 1. Check magic header
     * ------------------------------------- */

    if ((b == NULL) ||
        (n < 8U) ||
        (b[0] != 0xC3U) ||
        (b[1] != 0x3CU) ||
        (b[2] != 0xA5U) ||
        (b[3] != 0x5AU))
    {
        return 0U;
    }


    /* -------------------------------------
     * 2. Validate frame length
     * ------------------------------------- */

    len = b[5];

    if ((len > 32U) ||
        (n != (uint8_t)(8U + len)))
    {
        return 0U;
    }


    /* -------------------------------------
     * 3. Verify CRC16
     * ------------------------------------- */

    recv_crc =
        (uint16_t)b[6U + len] |
        ((uint16_t)b[7U + len] << 8);

    calc_crc = Protocol_CRC16(
        &b[4],
        (uint16_t)(2U + len)
    );

    if (recv_crc != calc_crc)
    {
        return 0U;
    }


    /* -------------------------------------
     * 4. Decode command
     * ------------------------------------- */

    cmd = b[4];

    cfg = BridgeConfig_GetPending();


    switch (cmd)
    {
        /* =================================
         * PING
         * ================================= */

        case CFG_PING:
        {
            if (len != 0U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            reply[0] = CFG_OK;

            memcpy(
                &reply[1],
                "PONG",
                4U
            );

            reply_len = 5U;

            break;
        }


        /* =================================
         * GET CONFIG
         * ================================= */

        case CFG_GET:
        {
            if (len != 0U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            /*
             * Response payload:
             *
             * [0]    Status
             * [1]    Node ID
             * [2]    Destination ID
             * [3:6]  Frequency LE32
             * [7]    Bandwidth
             * [8]    SF
             * [9]    CR
             * [10]   Power
             */

            reply[0] = CFG_OK;

            reply[1] = cfg->node_id;

            reply[2] = cfg->dest_id;

            Put32(
                &reply[3],
                cfg->frequency
            );

            reply[7] = cfg->bandwidth;

            reply[8] = cfg->sf;

            reply[9] = cfg->cr;

            reply[10] = (uint8_t)cfg->power;

            reply_len = 11U;

            break;
        }


        /* =================================
         * SET CONFIG
         * ================================= */

        case CFG_SET:
        {
            /*
             * Request payload:
             *
             * [0]    Node ID
             * [1]    Destination ID
             * [2:5]  Frequency LE32
             * [6]    Bandwidth
             * [7]    SF
             * [8]    CR
             * [9]    TX Power
             */

            if (len != 10U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            update.node_id = b[6];

            update.dest_id = b[7];

            update.frequency = Get32(&b[8]);

            update.bandwidth = b[12];

            update.sf = b[13];

            update.cr = b[14];

            update.power = (int8_t)b[15];


            /* Validate and stage configuration */

            if (!BridgeConfig_Set(&update))
            {
                reply[0] = CFG_BAD_ARG;
            }
            else
            {
                reply[0] = CFG_OK;
            }

            break;
        }


        /* =================================
         * SAVE TO FLASH
         * ================================= */

        case CFG_SAVE:
        {
            if (len != 0U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            if (!BridgeConfig_Save())
            {
                reply[0] = CFG_FLASH_ERR;
            }
            else
            {
                reply[0] = CFG_OK;
            }

            break;
        }


        /* =================================
         * GET DIGITAL INPUTS
         * ================================= */

        case CFG_INPUTS:
        {
            if (len != 0U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            /*
             * Bit 0: IN1
             * Bit 1: IN2
             * Bit 2: IN3
             * Bit 3: IN4
             */

            reply[0] = CFG_OK;

            reply[1] = Inputs_GetMask();

            reply_len = 2U;

            break;
        }


        /* =================================
         * REBOOT STM32
         * ================================= */

        case CFG_REBOOT:
        {
            if (len != 0U)
            {
                reply[0] = CFG_BAD_ARG;
                break;
            }

            reply[0] = CFG_OK;

            Reply(
                cmd,
                reply,
                reply_len
            );

            HAL_Delay(100U);

            NVIC_SystemReset();

            return 1U;
        }


        /* =================================
         * UNKNOWN COMMAND
         * ================================= */

        default:
        {
            reply[0] = CFG_UNKNOWN;

            break;
        }
    }


    /* Send response */

    Reply(
        cmd,
        reply,
        reply_len
    );

    return 1U;
}
