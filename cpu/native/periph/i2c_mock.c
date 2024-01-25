/*
 * Copyright (C) 2024 COGIP Robotics association
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_native
 * @ingroup     drivers_periph_i2c
 * @{
 *
 * @file
 * @brief       empty I2C implementation
 *
 * @author      Gilles DOFFE <g.doffe@gmail.com>
 */

#include "periph/i2c.h"

__attribute__((weak)) void i2c_init(i2c_t dev)
{
    (void)dev;
}

__attribute__((weak)) void i2c_acquire(i2c_t dev)
{
    (void)dev;
}

__attribute__((weak)) void i2c_release(i2c_t dev)
{
    (void)dev;
}

__attribute__((weak)) int i2c_read_bytes(i2c_t dev, uint16_t addr,
                   void *data, size_t len, uint8_t flags)
{
    (void)dev;
    (void)addr;
    (void)data;
    (void)len;
    (void)flags;

    return 0;
}

__attribute__((weak)) int i2c_write_bytes(i2c_t dev, uint16_t addr, const void *data,
                    size_t len, uint8_t flags)
{
    (void)dev;
    (void)addr;
    (void)data;
    (void)len;
    (void)flags;

    return 0;
}
