/*
 * Copyright (C)  2020 Benjamin Valentin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_serpente
 * @{
 *
 * @file
 * @brief       Board specific implementations for the Serpente board
 *
 * @author      Benjamin Valentin <benpicco@googlemail.com>
 *
 * @}
 */

#include "cpu.h"
#include "board.h"
#include "mtd.h"
#include "mtd_spi_nor.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "timex.h"

#ifdef MODULE_MTD
/* GD25Q32C */
static const mtd_spi_nor_params_t _serpente_nor_params = {
    .opcode = &mtd_spi_nor_opcode_default,
    .wait_chip_erase = 15LU * US_PER_SEC,
    .wait_32k_erase = 250LU * US_PER_MS,
    .wait_sector_erase = 50LU * US_PER_MS,
    .wait_chip_wake_up = 1LU * US_PER_MS,
    .clk = SERPENTE_NOR_SPI_CLK,
    .flag = SERPENTE_NOR_FLAGS,
    .spi = SERPENTE_NOR_SPI_DEV,
    .mode = SERPENTE_NOR_SPI_MODE,
    .cs = SERPENTE_NOR_SPI_CS,
    .wp = GPIO_UNDEF,
    .hold = GPIO_UNDEF,
};

static mtd_spi_nor_t serpente_nor_dev = {
    .base = {
        .driver = &mtd_spi_nor_driver,
        .page_size = SERPENTE_NOR_PAGE_SIZE,
        .pages_per_sector = SERPENTE_NOR_PAGES_PER_SECTOR,
        .sector_count = SERPENTE_NOR_SECTOR_COUNT,
    },
    .params = &_serpente_nor_params,
};

MTD_XFA_ADD(serpente_nor_dev, 0);
#endif /* MODULE_MTD */
