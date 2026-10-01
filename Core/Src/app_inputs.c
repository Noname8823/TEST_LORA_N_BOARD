#include "app_inputs.h"

#define INPUT_DEBOUNCE_COUNT    3U
#define INPUT_SAMPLE_TIME_MS   10U

static uint8_t stable_mask = 0;
static uint8_t last_raw_mask = 0;
static uint8_t debounce_count = 0;

static uint32_t last_sample_time = 0;


uint8_t Inputs_GetRawMask(void)
{
    uint8_t mask = 0;

    /*
     * EL817 output:
     *
     * Input OFF -> transistor OFF -> pull-up -> GPIO HIGH
     * Input ON  -> transistor ON  -> GPIO LOW
     *
     * Packet definition:
     * bit = 1 means input ACTIVE.
     */

    if (HAL_GPIO_ReadPin(IN_GPIO1_GPIO_Port,
                         IN_GPIO1_Pin) == GPIO_PIN_RESET)
    {
        mask |= (1U << 0);
    }

    if (HAL_GPIO_ReadPin(IN_GPIO2_GPIO_Port,
                         IN_GPIO2_Pin) == GPIO_PIN_RESET)
    {
        mask |= (1U << 1);
    }

    if (HAL_GPIO_ReadPin(IN_GPIO3_GPIO_Port,
                         IN_GPIO3_Pin) == GPIO_PIN_RESET)
    {
        mask |= (1U << 2);
    }

    if (HAL_GPIO_ReadPin(IN_GPIO4_GPIO_Port,
                         IN_GPIO4_Pin) == GPIO_PIN_RESET)
    {
        mask |= (1U << 3);
    }

    return mask;
}


void Inputs_Init(void)
{
    last_raw_mask = Inputs_GetRawMask();
    stable_mask   = last_raw_mask;

    debounce_count = 0;

    last_sample_time = HAL_GetTick();
}


void Inputs_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - last_sample_time) < INPUT_SAMPLE_TIME_MS)
    {
        return;
    }

    last_sample_time = now;

    uint8_t raw = Inputs_GetRawMask();

    if (raw == last_raw_mask)
    {
        if (debounce_count < INPUT_DEBOUNCE_COUNT)
        {
            debounce_count++;
        }

        if (debounce_count >= INPUT_DEBOUNCE_COUNT)
        {
            stable_mask = raw;
        }
    }
    else
    {
        last_raw_mask = raw;
        debounce_count = 0;
    }
}


uint8_t Inputs_GetMask(void)
{
    return stable_mask;
}
