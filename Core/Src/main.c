
#include "main.h"

#include "app_subghz_phy.h"

#include "gpio.h"
#include "usart.h"

#include "app_inputs.h"
#include "rs485.h"

#include "board_config.h"

/* =========================================================
 * FUNCTION PROTOTYPES
 * ========================================================= */

void SystemClock_Config(void);

/* =========================================================
 * MAIN
 * ========================================================= */

int main(void)
{
    /* Initialize HAL */
    HAL_Init();

    /* System clock */
    SystemClock_Config();

    /* Initialize GPIO */
    MX_GPIO_Init();

    /*
     * Force PA11 OFF immediately after GPIO init.
     *
     * PA11 LOW = LED OFF.
     */
    HAL_GPIO_WritePin(
        Led_Signal_GPIO_Port,
        Led_Signal_Pin,
        GPIO_PIN_RESET);

    /* Default RS485 direction = RX */
    HAL_GPIO_WritePin(
        RS485_DR_GPIO_Port,
        RS485_DR_Pin,
        GPIO_PIN_RESET);

    /* Initialize USART2 */
    MX_USART2_UART_Init();

    /* Initialize four opto inputs */
    Inputs_Init();

    /*
     * Initialize LoRa middleware.
     *
     * This also initializes the RTC timer,
     * radio driver and sequencer.
     */
    MX_SubGHz_Phy_Init();

    /*
     * Initialize RS485 after the original
     * middleware UART initialization.
     */
    RS485_Init();

    /* =====================================================
     * MAIN LOOP
     * ===================================================== */

    while (1)
    {
        /* Update opto input status */
        Inputs_Task();

        /*
         * Board 1:
         * Process incoming RS485 data.
         *
         * Board 2:
         * RS485 data is forwarded directly
         * from the LoRa processing task.
         */
        RS485_Task();

        /* Process LoRa radio events */
        MX_SubGHz_Phy_Process();
    }
}

/* =========================================================
 * SYSTEM CLOCK CONFIGURATION
 * ========================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure LSE drive capability */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSEDRIVE_CONFIG(
        RCC_LSEDRIVE_LOW);

    /* Configure main regulator */
    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Oscillator configuration */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_LSE |
        RCC_OSCILLATORTYPE_MSI;

    RCC_OscInitStruct.LSEState = RCC_LSE_ON;

    RCC_OscInitStruct.MSIState = RCC_MSI_ON;

    RCC_OscInitStruct.MSICalibrationValue =
        RCC_MSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.MSIClockRange =
        RCC_MSIRANGE_11;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* CPU / AHB / APB clock configuration */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK3 |
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_MSI;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;

    RCC_ClkInitStruct.AHBCLK3Divider =
        RCC_SYSCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================================================
 * ERROR HANDLER
 * ========================================================= */

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line)
{
    (void)file;
    (void)line;
}

#endif
