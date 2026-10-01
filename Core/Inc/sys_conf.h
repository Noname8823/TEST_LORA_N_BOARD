
#ifndef __SYS_CONF_H__
#define __SYS_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * TRACE CONFIGURATION
 * ========================================================= */

#define VERBOSE_LEVEL                        VLEVEL_M

/*
 * USART2 is dedicated to MAX3485.
 * Disable application debug logs.
 */
#define APP_LOG_ENABLED                      0

/* =========================================================
 * RF DEBUG CONFIGURATION
 * ========================================================= */

#define DEBUG_SUBGHZSPI_MONITORING_ENABLED   0

#define DEBUG_RF_NRESET_ENABLED              0

#define DEBUG_RF_HSE32RDY_ENABLED            0

#define DEBUG_RF_SMPSRDY_ENABLED             0

#define DEBUG_RF_LDORDY_ENABLED              0

#define DEBUG_RF_DTB1_ENABLED                0

#define DEBUG_RF_BUSY_ENABLED                0

/* =========================================================
 * DEBUGGER
 * ========================================================= */

#define DEBUGGER_ENABLED                     1

/* =========================================================
 * LOW POWER CONFIGURATION
 *
 * 1 = Disable Stop2.
 * ========================================================= */

#define LOW_POWER_DISABLE                    1

#ifdef __cplusplus
}
#endif

#endif
