/*
 * Copyright (C) 2016-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_common_nucleo64 STM32 Nucleo-64
 * @ingroup     boards
 * @brief       Support for STM32 Nucleo-64 boards
 * @{
 *
 * @file
 * @brief       Common pin definitions and board configuration options
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Sebastian Meiling <s@mlng.net>
 */

#ifndef BOARD_H
#define BOARD_H

#include "board_nucleo.h"
#include "arduino_pinmap.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LED pin definitions and handlers
 * @{
 */
#if defined(CPU_MODEL_STM32F302R8) || defined(CPU_MODEL_STM32L433RC)
#define LED0_PIN_NUM        13
#define LED0_PORT_NUM       PORT_B
#else
#define LED0_PIN_NUM        5
#define LED0_PORT_NUM       PORT_A
#endif
/** @} */

/**
 * @name    User button
 * @{
 */
#define BTN0_PIN            GPIO_PIN(PORT_C, 13)
#if defined(CPU_MODEL_STM32L433RC) || defined(CPU_MODEL_STM32G474RE) || \
    defined(CPU_MODEL_STM32G431RB)
#define BTN0_MODE           GPIO_IN_PD
#else
#define BTN0_MODE           GPIO_IN_PU
#endif
/** @} */

/**
 * @name Describe MRF24J40 radio
 * @{
 */
#ifndef MRF24J40_PARAM_SPI
#define MRF24J40_PARAM_SPI      SPI_DEV(0)
#endif

#ifndef MRF24J40_PARAM_SPI_CLK
#define MRF24J40_PARAM_SPI_CLK  SPI_CLK_5MHZ
#endif

#ifndef MRF24J40_PARAM_CS
#define MRF24J40_PARAM_CS       ARDUINO_PIN_10
#endif

#ifndef MRF24J40_PARAM_INT
#define MRF24J40_PARAM_INT      ARDUINO_PIN_7
#endif

#ifndef MRF24J40_PARAM_RESET
#define MRF24J40_PARAM_RESET    ARDUINO_PIN_5
#endif

/** @} */

#ifdef __cplusplus
}
#endif

#include "stm32_leds.h"

#endif /* BOARD_H */
/** @} */
