/*
 * Copyright (C) 2018 Gilles DOFFE <g.doffe@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_motor_driver DC Motor Driver
 * @ingroup     drivers_actuators
 * @brief       High-level driver for DC motors
 *
 * This API aims to handle DC motor analogic driver.
 * Driver boards using serial communication protocols (I2C, UART, etc...) are not in the
 * scope of this driver.
 * Mainly designed for H-bridge, it could also drive some brushless drivers.
 *
 * Some H-bridge driver circuits handle several motors.
 * Maximum motor number by H-bridge is set to 2 with CONFIG_MOTOR_DRIVER_MAX macro.
 * This macro can be overridden to support H-bridge drivers with more outputs.
 * However, CONFIG_MOTOR_DRIVER_MAX should not exceed PWM channels number.
 *
 * motor_driver_t structure represents an H-bridge.
 * As several H-bridge can share a same PWM device, motor_driver_t can
 * represent a group of H-bridge.
 *
 * Most of H-bridge boards uses the following I/Os for each motor :
 * - Enable/disable GPIO
 * - One or two direction GPIOs
 * - A PWM signal
 *
 * @verbatim
 *
 * Each motor direction is controlled (assuming it is enabled) according to
 * the following truth table :
 *  __________________________
 * | DIR0 | DIR1 |  BEHAVIOR  |
 * |--------------------------|
 * |  0   |  0   | BRAKE LOW  |
 * |  0   |  1   |     CW     |
 * |  1   |  0   |     CCW    |
 * |  1   |  1   | BRAKE HIGH |
 * |______|______|____________|
 *
 * In case of single GPIO for direction, only DIR0 is used without brake
 * capability :
 *  ___________________
 * | DIR0 |  BEHAVIOR  |
 * |-------------------|
 * |   0  |     CW     |
 * |   1  |     CCW    |
 * |______|____________|
 *
 * Some boards add a brake pin with single direction GPIO :
 *  ________________________
 * | DIR | BRAKE | BEHAVIOR |
 * |------------------------|
 * |  0  |   0   |    CW    |
 * |  0  |   1   |   BRAKE  |
 * |  1  |   0   |    CCW   |
 * |  1  |   1   |   BRAKE  |
 * |_____|_______|__________|
 *
 * @endverbatim
 *
 * From this truth tables we can extract two direction states :
 * - CW (ClockWise)
 * - CCW (Counter ClockWise)
 * and a brake capability
 *
 * BRAKE LOW is functionally the same than BRAKE HIGH but some H-bridge only
 * brake on BRAKE HIGH due to hardware.
 * In case of single direction GPIO, there is no BRAKE.
 *
 * In case of brake, PWM duty cycle is always set to 0.

 * @{
 * @file
 * @brief       High-level driver for DC motors
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "periph/pwm.h"
#include "periph/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup drivers_motor_driver_config     motor_driver driver build configuration
 * @ingroup config_drivers_actuators
 * @{
 */
/**
 * @brief Maximum number of motors by motor driver
 */
#ifndef CONFIG_MOTOR_DRIVER_MAX
#define CONFIG_MOTOR_DRIVER_MAX    (2)
#endif
/** @} */

/**
 * @brief Describe DC motor driver modes
 */
typedef enum {
    MOTOR_DRIVER_2_DIRS         = 0,            /**< 2 GPIOs for direction, \
                                                     handling brake */
    MOTOR_DRIVER_1_DIR          = 1,            /**< single GPIO for direction, \
                                                     no brake */
    MOTOR_DRIVER_1_DIR_BRAKE    = 2             /**< single GPIO for direction, \
                                                     single GPIO for brake */
} motor_driver_mode_t;

/**
 * @brief Describe DC motor driver brake levels
 */
typedef enum {
    MOTOR_BRAKE_LOW     = 0,        /**< low level brake */
    MOTOR_BRAKE_HIGH    = 1,        /**< high level brake */
} motor_driver_brake_level_t;

/**
 * @brief Describe DC motor driver enable levels
 */
typedef enum {
    MOTOR_ENABLE_LOW     = 0,        /**< low level enable */
    MOTOR_ENABLE_HIGH    = 1,        /**< high level enable */
} motor_driver_enable_level_t;

/**
 * @brief Describe DC motor direction states
 */
typedef enum {
    MOTOR_CW    = 0,            /**< clockwise */
    MOTOR_CCW   = 1,            /**< counter clockwise */
} motor_direction_t;

/**
 * @brief Describe DC motor with PWM channel and GPIOs
 */
typedef struct {
    int pwm_channel;            /**< PWM channel the motor is connected to */
    gpio_t gpio_enable;         /**< GPIO to enable/disable motor */
    gpio_t gpio_dir0;           /**< GPIO to control rotation direction */
    gpio_t gpio_dir1_or_brake;  /**< GPIO to control rotation direction */
    uint8_t gpio_dir_reverse;   /**< flag to reverse direction */
} motor_t;

/**
 * @brief   Motor driver
 */
typedef struct _motor_driver_t motor_driver_t;

/**
 * @brief   Motor callback. It is called at end of motor_set()
 */
typedef void (*motor_set_post_cb_t)(const motor_driver_t *motor_driver,
                                  uint8_t motor_id,
                                  int32_t pwm_duty_cycle);

/**
 * @brief   Motor set callback. Called to set motor speed.
 */
typedef void (*motor_set_cb_t)(const motor_t *motor,
                               motor_direction_t direction);

/**
 * @brief   Motor brake callback. Called to brake a motor.
 */
typedef void (*motor_brake_cb_t)(const motor_t *motor,
                                 motor_driver_brake_level_t brake);

/**
 * @brief Describe DC motor driver with PWM device and motors array
 */
typedef struct {
    motor_driver_mode_t mode;                   /**< driver mode */
    pwm_t pwm_dev;                              /**< PWM device driving motors */
    pwm_mode_t pwm_mode;                        /**< PWM mode */
    uint32_t pwm_frequency;                     /**< PWM device frequency */
    uint32_t pwm_resolution;                    /**< PWM device resolution */
    motor_driver_brake_level_t brake_level;     /**< driver brake mode */
    motor_driver_enable_level_t enable_level;   /**< driver brake mode */
    uint8_t nb_motors;                          /**< number of motors */
    motor_t motors[CONFIG_MOTOR_DRIVER_MAX];    /**< motors array */
    motor_set_post_cb_t motor_set_post_cb;      /**< callback post to motor_set */
} motor_driver_params_t;

/**
 * @brief   Motor driver
 */
struct _motor_driver_t {
    const motor_driver_params_t *params;        /**< parameters */
    motor_set_cb_t motor_set_cb;                /**< callback used by motor_set() to set direction */
    motor_brake_cb_t motor_brake_cb;            /**< callback used by motor_brake() */
};

/**
 * @brief Initialize DC motor driver board
 *
 * @param[out]  motor_driver        motor driver to initialize
 * @param[in]   motor_driver_params motor driver parameters
 *
 * @return                          0 on success
 * @return                          -EINVAL on bad parameter value
 * @return                          -EIO on failed GPIO init
 */
int motor_driver_init(motor_driver_t *motor_driver, const motor_driver_params_t *motor_driver_params);

/**
 * @brief Set motor speed and direction
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 * @param[in] pwm_duty_cycle    signed PWM duty_cycle to set motor speed and direction
 *
 * @return                      0 on success
 * @return                      -EINVAL on bad motor ID
 */
int motor_set(const motor_driver_t *motor_driver, uint8_t motor_id, \
              int32_t pwm_duty_cycle);

/**
 * @brief Brake the motor of a given motor driver
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 *
 * @return                      0 on success
 * @return                      -EINVAL on bad motor ID
 */
int motor_brake(const motor_driver_t *motor_driver, uint8_t motor_id);

/**
 * @brief Enable a motor of a given motor driver
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 *
 * @return
 */
void motor_enable(const motor_driver_t *motor_driver, uint8_t motor_id);

/**
 * @brief Disable a motor of a given motor driver
 *
 * @param[in] motor_driver      motor driver to which motor is attached
 * @param[in] motor_id          motor ID on driver
 *
 * @return
 */
void motor_disable(const motor_driver_t *motor_driver, uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */
/** @} */
