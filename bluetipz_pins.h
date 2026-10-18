/***************************************************************************//**
 * @file bluetipz_pins.h
 * @brief BlueTipz Rx board GPIO assignments (EFR32BG21A020F512IM32).
 *
 * Port note (SiSDK 2026.6.1): these definitions used to come from the
 * "CUSTOM_PIN_NAME" section that the Studio 5 Pin Tool generated into
 * config/pin_config.h. The application now owns them, so the build no longer
 * depends on Pin Tool output. Values are identical to the 4.3.1 project.
 * Each pair is guarded in case pin_config.h (which the ported .pintool may
 * regenerate with the same names) was included first.
 ******************************************************************************/
#ifndef BLUETIPZ_PINS_H
#define BLUETIPZ_PINS_H

#include "em_gpio.h"

// Buzzer enable                         PA04
#ifndef BUZZ_EN_PORT
#define BUZZ_EN_PORT                     gpioPortA
#define BUZZ_EN_PIN                      4
#endif

// Debug UART (USART0, not instantiated) PA05 TX / PA06 RX
#ifndef dbg_tx_PORT
#define dbg_tx_PORT                      gpioPortA
#define dbg_tx_PIN                       5
#endif
#ifndef dbg_rx_PORT
#define dbg_rx_PORT                      gpioPortA
#define dbg_rx_PIN                       6
#endif

// NTC thermistor supply enable          PC00
#ifndef NTC_SENSOR_EN_PORT
#define NTC_SENSOR_EN_PORT               gpioPortC
#define NTC_SENSOR_EN_PIN                0
#endif

// NTC thermistor analog input           PC01 (IADC0 iadcPosInputPortCPin1)
#ifndef NTC_SENSOR_PORT
#define NTC_SENSOR_PORT                  gpioPortC
#define NTC_SENSOR_PIN                   1
#endif

// LED enables                           PC03 / PC04
#ifndef LED2_EN_PORT
#define LED2_EN_PORT                     gpioPortC
#define LED2_EN_PIN                      3
#endif
#ifndef LED1_EN_PORT
#define LED1_EN_PORT                     gpioPortC
#define LED1_EN_PIN                      4
#endif

// Power button (EM4 wake-up EM4WU7)     PC05
#ifndef PWR_BTN_PORT
#define PWR_BTN_PORT                     gpioPortC
#define PWR_BTN_PIN                      5
#endif

// Vibration motor driver enable         PD04
#ifndef VIB_EN_PORT
#define VIB_EN_PORT                      gpioPortD
#define VIB_EN_PIN                       4
#endif

#endif // BLUETIPZ_PINS_H
