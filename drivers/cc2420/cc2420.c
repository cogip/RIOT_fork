/*
 * Copyright (C) 2015-2016 Freie Universität Berlin
 *               2016 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_cc2420
 * @{
 *
 * @file
 * @brief       Implementation of public functions for cc2420 driver
 *
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "byteorder.h"
#include "net/ieee802154.h"
#include "net/gnrc.h"

#include "cc2420_internal.h"
#include "cc2420_netdev.h"
#include "cc2420_registers.h"

#define ENABLE_DEBUG 0
#include "debug.h"

void cc2420_setup(cc2420_t * dev, const cc2420_params_t *params, uint8_t index)
{
    netdev_t *netdev = &dev->netdev.netdev;

    /* set pointer to the devices netdev functions */
    netdev->driver = &cc2420_driver;
    /* pull in device configuration parameters */
    dev->params = *params;
    dev->state = CC2420_STATE_IDLE;
    /* reset device descriptor fields */
    dev->options = 0;

    netdev_register(netdev, NETDEV_CC2420, index);
    netdev_ieee802154_setup(&dev->netdev);
}

int cc2420_init(cc2420_t *dev)
{
    uint16_t reg;

    netdev_ieee802154_reset(&dev->netdev);

    cc2420_set_addr_short(dev, dev->netdev.short_addr);
    cc2420_set_addr_long(dev, dev->netdev.long_addr);
    cc2420_set_chan(dev, CC2420_CHAN_DEFAULT);
    cc2420_set_txpower(dev, CC2420_TXPOWER_DEFAULT);

    /* set default options */
    cc2420_set_option(dev, CC2420_OPT_AUTOACK, true);
    cc2420_set_option(dev, CC2420_OPT_CSMA, true);

    /* change default RX bandpass filter to 1.3uA (as recommended) */
    reg = cc2420_reg_read(dev, CC2420_REG_RXCTRL1);
    reg |= CC2420_RXCTRL1_RXBPF_LOCUR;
    cc2420_reg_write(dev, CC2420_REG_RXCTRL1, reg);

    /* set the FIFOP threshold to maximum. */
    cc2420_reg_write(dev, CC2420_REG_IOCFG0, CC2420_PKT_MAXLEN);

    /* turn off "Security enable" (page 33). */
    reg = cc2420_reg_read(dev, CC2420_REG_SECCTRL0);
    reg &= ~CC2420_SECCTRL0_RXFIFO_PROT;
    cc2420_reg_write(dev, CC2420_REG_SECCTRL0, reg);

    /* set preamble length to 3 leading zero byte */
    /* and turn on hardware CRC generation */
    reg = cc2420_reg_read(dev, CC2420_REG_MDMCTRL0);
    reg &= ~(CC2420_MDMCTRL0_PREAMBLE_M);
    reg |= CC2420_MDMCTRL0_PREAMBLE_3B;
    reg |= CC2420_MDMCTRL0_AUTOCRC;
    cc2420_reg_write(dev, CC2420_REG_MDMCTRL0, reg);

    /* go into RX state */
    cc2420_set_state(dev, NETOPT_STATE_IDLE);

    return 0;
}

bool cc2420_cca(cc2420_t *dev)
{
    while (!(cc2420_status(dev) & CC2420_STATUS_RSSI_VALID)) {}
    return gpio_read(dev->params.pin_cca);
}

size_t cc2420_send(cc2420_t *dev, const iolist_t *iolist)
{
    size_t n = cc2420_tx_prepare(dev, iolist);

    if ((n > 0) && !(dev->options & CC2420_OPT_PRELOADING)) {
        cc2420_tx_exec(dev);
    }

    return n;
}

size_t cc2420_tx_prepare(cc2420_t *dev, const iolist_t *iolist)
{
    size_t pkt_len = 2;     /* include the FCS (frame check sequence) */

    /* wait for any ongoing transmissions to be finished */
    DEBUG("cc2420: tx_exec: waiting for any ongoing transmission\n");
    while (cc2420_get_state(dev) & NETOPT_STATE_TX) {}

    /* get and check the length of the packet */
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        pkt_len += iol->iol_len;
    }
    if (pkt_len >= CC2420_PKT_MAXLEN) {
        DEBUG("cc2420: tx_prep: unable to send, pkt too large\n");
        return 0;
    }

    /* flush TX FIFO and write new packet to it */
    cc2420_strobe(dev, CC2420_STROBE_FLUSHTX);
    /* push packet length to TX FIFO */
    cc2420_fifo_write(dev, (uint8_t *)&pkt_len, 1);
    /* push packet to TX FIFO, only if iol->iol_len > 0 */
    for (const iolist_t *iol = iolist; iol; iol = iol->iol_next) {
        if (iol->iol_len > 0) {
            cc2420_fifo_write(dev, iol->iol_base, iol->iol_len);
        }
    }
    DEBUG("cc2420: tx_prep: loaded %" PRIuSIZE " byte into the TX FIFO\n", pkt_len);

    return pkt_len;
}

void cc2420_tx_exec(cc2420_t *dev)
{
    /* trigger the transmission */
    if (dev->netdev.netdev.event_callback) {
        dev->netdev.netdev.event_callback(&dev->netdev.netdev,
                                          NETDEV_EVENT_TX_STARTED);
    }
    DEBUG("cc2420: tx_exec: TX_START\n");
    if (dev->options & CC2420_OPT_CSMA) {
        DEBUG("cc2420: tx_exec: triggering TX with CCA\n");
        cc2420_strobe(dev, CC2420_STROBE_TXONCCA);
    }
    else {
        DEBUG("cc2420: tx_exec: triggering TX without CCA\n");
        cc2420_strobe(dev, CC2420_STROBE_TXON);
    }
}

static inline void _flush_rx_fifo(cc2420_t *dev)
{
    /* as stated in the CC2420 datasheet (section 14.3), the SFLUSHRX command
     * strobe should be issued twice to ensure that the SFD pin goes back to its
     * idle state */
    cc2420_strobe(dev, CC2420_STROBE_FLUSHRX);
    cc2420_strobe(dev, CC2420_STROBE_FLUSHRX);
}

int cc2420_rx(cc2420_t *dev, uint8_t *buf, size_t max_len, void *info)
{
    (void)info;

    uint8_t len;
    uint8_t crc_corr;

    /* without a provided buffer, only readout the length and return it */
    if (buf == NULL) {
        /* get the packet length (without dropping it) (first byte in RX FIFO) */
        cc2420_ram_read(dev, CC2420_RAM_RXFIFO, &len, 1);
        len -= 2;   /* subtract RSSI and FCF */
        DEBUG("cc2420: recv: packet of length %i in RX FIFO\n", (int)len);
        if (max_len != 0) {
            DEBUG("cc2420: recv: Dropping frame as requested\n");
            _flush_rx_fifo(dev);
        }
    }
    else {
        /* read length byte */
        cc2420_fifo_read(dev, &len, 1);
        len -= 2;   /* subtract RSSI and FCF */

        if (len > max_len) {
            DEBUG("cc2420: recv: Supplied buffer to small\n");
            _flush_rx_fifo(dev);
            return -ENOBUFS;
        }

        /* read fifo contents */
        DEBUG("cc2420: recv: reading %i byte of the packet\n", (int)len);
        cc2420_fifo_read(dev, buf, len);

        int8_t rssi;
        cc2420_fifo_read(dev, (uint8_t*)&rssi, 1);
        DEBUG("cc2420: recv: RSSI is %i\n", (int)rssi);

        /* fetch and check if CRC_OK bit (MSB) is set */
        cc2420_fifo_read(dev, &crc_corr, 1);
        if (!(crc_corr & 0x80)) {
            DEBUG("cc2420: recv: CRC_OK bit not set, dropping packet\n");
            /* drop the corrupted frame from the RXFIFO */
            len = 0;
        }
        if (info != NULL) {
            netdev_ieee802154_rx_info_t *radio_info = info;
            radio_info->rssi = CC2420_RSSI_OFFSET + rssi;
            radio_info->lqi = crc_corr & CC2420_CRCCOR_COR_MASK;
        }

        /* finally flush the FIFO */
        _flush_rx_fifo(dev);
    }

    return (int)len;
}
