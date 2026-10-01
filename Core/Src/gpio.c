
#include "gpio.h"

/* =========================================================
 * GPIO INITIALIZATION
 * ========================================================= */

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* =====================================================
     * OUTPUT INITIAL STATES
     * ===================================================== */

    /*
     * PA4 - RS485_DR
     *
     * LOW = Receive mode.
     */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);

    /*
     * PA11 - LED SIGNAL
     *
     * LOW = LED OFF.
     */
    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET);

    /*
     * PC13 - RF SWITCH
     *
     * Keep original initialization.
     * Radio driver controls it during operation.
     */
    HAL_GPIO_WritePin(
        RF_SW_CTRL_GPIO_Port,
        RF_SW_CTRL_Pin,
        GPIO_PIN_RESET);

    /* =====================================================
     * OPTO INPUTS + PB12
     * ===================================================== */

    GPIO_InitStruct.Pin =
        IN_GPIO1_Pin |
        IN_GPIO2_Pin |
        IN_GPIO3_Pin |
        IN_GPIO4_Pin |
        FREQ_HIGH_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct);

    /* =====================================================
     * PA4 + PA11 OUTPUTS
     * ===================================================== */

    GPIO_InitStruct.Pin =
        RS485_DR_Pin |
        Led_Signal_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        GPIOA,
        &GPIO_InitStruct);

    /* =====================================================
     * PC13 RF SWITCH
     * ===================================================== */

    GPIO_InitStruct.Pin = RF_SW_CTRL_Pin;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        RF_SW_CTRL_GPIO_Port,
        &GPIO_InitStruct);
}
