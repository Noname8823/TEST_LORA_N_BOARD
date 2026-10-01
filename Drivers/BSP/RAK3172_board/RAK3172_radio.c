/**
 ******************************************************************************
 * @file    stm32wlxx_nucleo_radio.c
 * @author  MCD Application Team
 * @brief   RF switch control for custom STM32WLE5CCU6 board
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32wlxx_nucleo_radio.h"

/**
 * @brief  Init Radio Switch
 * @retval BSP status
 */
int32_t BSP_RADIO_Init(void)
{
    GPIO_InitTypeDef gpio_init_structure = {0};

    RF_SW_CTRL2_GPIO_CLK_ENABLE();

    gpio_init_structure.Pin   = RF_SW_CTRL2_PIN;
    gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull  = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(RF_SW_CTRL2_GPIO_PORT,
                  &gpio_init_structure);

    /* Default = RX */
    HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                      RF_SW_CTRL2_PIN,
                      GPIO_PIN_RESET);

    return BSP_ERROR_NONE;
}


/**
 * @brief  DeInit Radio Switch
 * @retval BSP status
 */
int32_t BSP_RADIO_DeInit(void)
{
    /*
     * Return RF switch to default state
     */
    HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                      RF_SW_CTRL2_PIN,
                      GPIO_PIN_RESET);

    /*
     * DeInit PC13
     */
    HAL_GPIO_DeInit(RF_SW_CTRL2_GPIO_PORT,
                    RF_SW_CTRL2_PIN);

    return BSP_ERROR_NONE;
}


/**
 * @brief Configure Radio Switch
 *
 * @param Config
 *        RADIO_SWITCH_OFF
 *        RADIO_SWITCH_RX
 *        RADIO_SWITCH_RFO_LP
 *        RADIO_SWITCH_RFO_HP
 *
 * @retval BSP status
 */
int32_t BSP_RADIO_ConfigRFSwitch(BSP_RADIO_Switch_TypeDef Config)
{
    switch (Config)
    {
        /*
         * BGS12SN6 is SPDT.
         * It does not have a real OFF state.
         *
         * Therefore OFF is mapped to RX/default path.
         */
        case RADIO_SWITCH_OFF:
        {
            HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                              RF_SW_CTRL2_PIN,
                              GPIO_PIN_RESET);

            break;
        }


        /*
         * RX
         *
         * PC13 = 0
         *
         * Assumption:
         * RFIN -> RF1 -> STM32 RFI
         */
        case RADIO_SWITCH_RX:
        {
            HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                              RF_SW_CTRL2_PIN,
                              GPIO_PIN_RESET);

            break;
        }


        /*
         * TX Low Power
         *
         * PC13 = 1
         *
         * This board currently uses the same antenna
         * switch position for TX LP / TX HP.
         */
        case RADIO_SWITCH_RFO_LP:
        {
            HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                              RF_SW_CTRL2_PIN,
                              GPIO_PIN_SET);

            break;
        }


        /*
         * TX High Power
         *
         * PC13 = 1
         *
         * Assumption:
         * STM32 RFO_HP -> RF2 -> RFIN -> Antenna
         */
        case RADIO_SWITCH_RFO_HP:
        {
            HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT,
                              RF_SW_CTRL2_PIN,
                              GPIO_PIN_SET);

            break;
        }


        default:
        {
            break;
        }
    }

    return BSP_ERROR_NONE;
}


/**
 * @brief  Return Board TX Configuration
 *
 * @retval RADIO_CONF_RFO_HP
 */
int32_t BSP_RADIO_GetTxConfig(void)
{
    /*
     * Custom board uses STM32WLE5 RFO_HP output.
     */
    return RADIO_CONF_RFO_HP;
}


/**
 * @brief  Check if TCXO is present
 *
 * @retval RADIO_CONF_TCXO_NOT_SUPPORTED
 */
int32_t BSP_RADIO_IsTCXO(void)
{
    /*
     * Board uses 32 MHz crystal,
     * not TCXO.
     */
    return RADIO_CONF_TCXO_NOT_SUPPORTED;
}


/**
 * @brief Check if DCDC is supported
 *
 * @retval RADIO_CONF_DCDC_SUPPORTED
 */
int32_t BSP_RADIO_IsDCDC(void)
{
    /*
     * Keep this if your STM32WLE5 power circuit
     * supports the internal SMPS/DCDC configuration.
     */
    return RADIO_CONF_DCDC_SUPPORTED;
}


/**
 * @brief Return RF Output Max Power Configuration
 *
 * @param Config
 *        RADIO_RFO_LP_MAXPOWER
 *        RADIO_RFO_HP_MAXPOWER
 *
 * @retval maximum output configuration
 */
int32_t BSP_RADIO_GetRFOMaxPowerConfig(
        BSP_RADIO_RFOMaxPowerConfig_TypeDef Config)
{
    int32_t ret;

    if (Config == RADIO_RFO_LP_MAXPOWER)
    {
        ret = RADIO_CONF_RFO_LP_MAX_15_dBm;
    }
    else
    {
        ret = RADIO_CONF_RFO_HP_MAX_22_dBm;
    }

    return ret;
}
