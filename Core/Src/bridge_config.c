
#include "bridge_config.h"
#include "main.h"
#include "protocol.h"

#include <string.h>

/* =========================================
 * FLASH CONFIGURATION
 * ========================================= */

/*
 * STM32WLE5CCU6
 * Flash size : 256 KB
 * Page size  : 2 KB
 *
 * Last Flash page:
 * 0x0803F800 - 0x0803FFFF
 *
 */



volatile uint32_t g_flash_stage = 0U;

volatile uint32_t g_flash_hal_status = 0U;

volatile uint32_t g_flash_error = 0U;

volatile uint32_t g_flash_page_error = 0xFFFFFFFFU;

volatile uint32_t g_flash_failed_addr = 0U;

volatile uint32_t g_flash_verify_offset = 0xFFFFFFFFU;

#define CONFIG_FLASH_ADDR  0x0803F800UL

#define CONFIG_MAGIC       0x3143524CUL

#define RECORD_SIZE        24U


/* =========================================
 * CONFIGURATION VARIABLES
 * ========================================= */

/* Configuration currently used by LoRa */
static BridgeSettings current_cfg;

/* Configuration waiting to be saved */
static BridgeSettings staged_cfg;


/* =========================================
 * UINT32 LITTLE ENDIAN
 * ========================================= */

static void Put32(
    uint8_t *p,
    uint32_t v)
{
    p[0] = (uint8_t)v;

    p[1] = (uint8_t)(v >> 8);

    p[2] = (uint8_t)(v >> 16);

    p[3] = (uint8_t)(v >> 24);
}


static uint32_t Get32(const uint8_t *p)
{
    return
        (uint32_t)p[0] |
        ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24);
}


/* =========================================
 * VALIDATE CONFIGURATION
 * ========================================= */

static uint8_t Valid(const BridgeSettings *c)
{
    if (c == NULL)
    {
        return 0U;
    }

    /* Node ID: 1 - 254 */

    if ((c->node_id < 1U) ||
        (c->node_id > 254U))
    {
        return 0U;
    }

    /*
     * Broadcast mode.
     *
     * Destination must be 0xFF.
     */

    if (c->dest_id != 0xFFU)
    {
        return 0U;
    }

    /* Frequency */

    if ((c->frequency != 433000000UL) &&
        (c->frequency != 470000000UL) &&
        (c->frequency != 868000000UL) &&
        (c->frequency != 915000000UL))
    {
        return 0U;
    }

    /* Bandwidth: 0, 1, 2 */

    if (c->bandwidth > 2U)
    {
        return 0U;
    }

    /* Spreading Factor: SF7 - SF12 */

    if ((c->sf < 7U) ||
        (c->sf > 12U))
    {
        return 0U;
    }

    /* Coding Rate: 1 - 4 */

    if ((c->cr < 1U) ||
        (c->cr > 4U))
    {
        return 0U;
    }

    /* TX Power */

    if ((c->power < -9) ||
        (c->power > 14))
    {
        return 0U;
    }

    return 1U;
}
/* =========================================
 * SERIALIZE CONFIGURATION
 * ========================================= */

static void Serialize(
    uint8_t b[RECORD_SIZE],
    const BridgeSettings *c)
{
    uint16_t crc;

    memset(
        b,
        0xFF,
        RECORD_SIZE
    );

    Put32(
        &b[0],
        CONFIG_MAGIC
    );

    b[4] = c->node_id;

    b[5] = c->dest_id;

    b[6] = c->bandwidth;

    b[7] = c->sf;

    b[8] = c->cr;

    b[9] = (uint8_t)c->power;

    Put32(
        &b[12],
        c->frequency
    );

    /* CRC16 over first 16 bytes */

    crc = Protocol_CRC16(
        b,
        16U
    );

    b[16] = (uint8_t)crc;

    b[17] = (uint8_t)(crc >> 8);
}


/* =========================================
 * INITIALIZATION
 * ========================================= */

void BridgeConfig_Init(void)
{
    uint8_t b[RECORD_SIZE];

    BridgeSettings loaded;

    uint16_t crc;

    /* Default configuration */

    current_cfg.node_id = 1U;

    current_cfg.dest_id = 0xFFU;

    current_cfg.frequency = 433000000UL;

    current_cfg.bandwidth = 0U;

    current_cfg.sf = 7U;

    current_cfg.cr = 1U;

    current_cfg.power = 14;

    /* Read saved configuration from Flash */

    memcpy(
        b,
        (const void *)CONFIG_FLASH_ADDR,
        RECORD_SIZE
    );

    if (Get32(&b[0]) != CONFIG_MAGIC)
    {
        staged_cfg = current_cfg;
        return;
    }

    crc = Protocol_CRC16(b, 16U);

    if ((b[16] != (uint8_t)crc) ||
        (b[17] != (uint8_t)(crc >> 8)))
    {
        staged_cfg = current_cfg;
        return;
    }

    loaded.node_id = b[4];
    loaded.dest_id = 0xFFU;

    loaded.bandwidth = b[6];
    loaded.sf = b[7];
    loaded.cr = b[8];
    loaded.power = (int8_t)b[9];

    loaded.frequency = Get32(&b[12]);

    if (Valid(&loaded))
    {
        current_cfg = loaded;
    }

    staged_cfg = current_cfg;
}

/* =========================================
 * GET ACTIVE CONFIGURATION
 * ========================================= */

const BridgeSettings *BridgeConfig_Get(void)
{
    return &current_cfg;
}


/* =========================================
 * GET PENDING CONFIGURATION
 * ========================================= */

const BridgeSettings *BridgeConfig_GetPending(void)
{
    return &staged_cfg;
}


/* =========================================
 * UPDATE CONFIGURATION IN RAM
 * ========================================= */

uint8_t BridgeConfig_Set(
    const BridgeSettings *cfg)
{
    BridgeSettings temp;

    if (cfg == NULL)
    {
        return 0U;
    }

    /* Copy configuration received from WinForms */

    temp = *cfg;

    /*
     * Always use Broadcast.
     * Destination ID is not user configurable.
     */

    temp.dest_id = 0xFFU;

    /* Validate all parameters */

    if (Valid(&temp) == 0U)
    {
        return 0U;
    }

    /* Store new configuration in RAM */

    staged_cfg = temp;

    return 1U;
}
/* =========================================
 * SAVE CONFIGURATION TO FLASH
 * ========================================= */
uint8_t BridgeConfig_Save(void)
{
    uint8_t b[RECORD_SIZE];

    uint32_t page_error = 0xFFFFFFFFU;

    FLASH_EraseInitTypeDef erase = {0};

    HAL_StatusTypeDef status;

    uint32_t i;

    uint64_t doubleword;

    volatile const uint8_t *flash_data =
        (volatile const uint8_t *)CONFIG_FLASH_ADDR;


    /* =========================================
     * RESET DEBUG VARIABLES
     * ========================================= */

    g_flash_stage = 0U;

    g_flash_hal_status = 0U;

    g_flash_error = 0U;

    g_flash_page_error = 0xFFFFFFFFU;

    g_flash_failed_addr = 0U;

    g_flash_verify_offset = 0xFFFFFFFFU;


    /* =========================================
     * STEP 1: VALIDATE CONFIGURATION
     * ========================================= */

    if (!Valid(&staged_cfg))
    {
        g_flash_stage = 1U;

        return 0U;
    }


    Serialize(
        b,
        &staged_cfg
    );


    /* =========================================
     * STEP 2: UNLOCK FLASH
     * ========================================= */

    status = HAL_FLASH_Unlock();

    g_flash_hal_status = (uint32_t)status;

    if (status != HAL_OK)
    {
        g_flash_stage = 2U;

        g_flash_error = HAL_FLASH_GetError();

        return 0U;
    }


    /* =========================================
     * STEP 3: ERASE CONFIGURATION PAGE
     * ========================================= */

    erase.TypeErase = FLASH_TYPEERASE_PAGES;

    erase.Page =
        (CONFIG_FLASH_ADDR - FLASH_BASE) /
        FLASH_PAGE_SIZE;

    erase.NbPages = 1U;


    status = HAL_FLASHEx_Erase(
        &erase,
        &page_error
    );

    g_flash_hal_status = (uint32_t)status;

    g_flash_page_error = page_error;


    if (status != HAL_OK)
    {
        g_flash_stage = 3U;

        g_flash_error = HAL_FLASH_GetError();

        (void)HAL_FLASH_Lock();

        return 0U;
    }


    /* =========================================
     * STEP 4: PROGRAM FLASH
     * ========================================= */

    for (i = 0U; i < RECORD_SIZE; i += 8U)
    {
        doubleword = 0U;

        memcpy(
            &doubleword,
            &b[i],
            8U
        );

        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_DOUBLEWORD,
            CONFIG_FLASH_ADDR + i,
            doubleword
        );

        g_flash_hal_status = (uint32_t)status;


        if (status != HAL_OK)
        {
            g_flash_stage = 4U;

            g_flash_error =
                HAL_FLASH_GetError();

            g_flash_failed_addr =
                CONFIG_FLASH_ADDR + i;

            (void)HAL_FLASH_Lock();

            return 0U;
        }
    }


    /* =========================================
     * STEP 5: LOCK FLASH
     * ========================================= */

    status = HAL_FLASH_Lock();

    g_flash_hal_status = (uint32_t)status;

    if (status != HAL_OK)
    {
        g_flash_stage = 5U;

        g_flash_error =
            HAL_FLASH_GetError();

        return 0U;
    }


    /* =========================================
     * STEP 6: VERIFY FLASH DATA
     * ========================================= */

    for (i = 0U; i < RECORD_SIZE; i++)
    {
        if (flash_data[i] != b[i])
        {
            g_flash_stage = 6U;

            g_flash_verify_offset = i;

            return 0U;
        }
    }


    /* =========================================
     * SAVE COMPLETED SUCCESSFULLY
     * ========================================= */

    g_flash_stage = 7U;

    return 1U;
}
