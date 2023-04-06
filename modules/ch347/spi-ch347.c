/*
 * Driver for the CH347 USB to I2C/SPI/GPIO adapter
 *
 * Copyright (c) 2023 Seeed Studio
 * Hongtai Liu <lht856@foxmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "core.h"

#define ch347_spi_maser_to_dev(m) *((struct ch347_device **)spi_master_get_devdata(m))

static struct spi_board_info ch347_spi_device_template = {
    .modalias = "spidev",
    .max_speed_hz = CH347_SPI_MAX_FREQ,
    .bus_num = 0,
    .chip_select = 0,
    .mode = SPI_MODE_0,
};

static int ch347_spi_get_cfg(struct ch347_device *dev)
{
    int retval;
    int offset;

    mutex_lock(&dev->io_mutex);

    offset = 0;

    dev->bulk_out_buffer[offset++] = CH347_CMD_SPI_CONFIG_R;
    dev->bulk_out_buffer[offset++] = 1;
    dev->bulk_out_buffer[offset++] = 0;
    dev->bulk_out_buffer[offset++] = 1;

    retval = ch347_usb_xfer(dev, offset, sizeof(dev->spi_reg) + CH347_CMD_SPI_HEADER_LEN, 1000);

    if (retval < 0)
    {
        dev_dbg(&dev->intf->dev, "SPI config get failed.");
        goto done;
    }

    memcpy(&dev->spi_reg, dev->bulk_in_buffer + CH347_CMD_SPI_HEADER_LEN, sizeof(dev->spi_reg));

done:
    mutex_unlock(&dev->io_mutex);
    return retval;
}

static int ch347_spi_set_cfg(struct ch347_device *dev)
{
    int retval;
    int offset;
    mutex_lock(&dev->io_mutex);

    dev->bulk_out_buffer[offset++] = CH347_CMD_SPI_CONGIG_W;
    dev->bulk_out_buffer[offset++] = (u8)(sizeof(dev->spi_reg) & 0xFF);
    dev->bulk_out_buffer[offset++] = (u8)((sizeof(dev->spi_reg) >> 8) & 0xFF);
    memcpy(dev->bulk_out_buffer + offset, &dev->spi_reg, sizeof(dev->spi_reg));
    offset += sizeof(dev->spi_reg);

    retval = ch347_usb_xfer(dev, offset, CH347_CMD_GPIO_HEADER_LEN + 1, 1000);

    if (retval < 0)
    {
        dev_dbg(&dev->intf->dev, "SPI config set failed.");
        goto done;
    }

    if (dev->bulk_in_buffer[0] != CH347_CMD_SPI_CONGIG_W)
    {
        dev_dbg(&dev->intf->dev, "SPI config set failed.");
        retval = -EIO;
        goto done;
    }

    if (dev->bulk_in_buffer[CH347_CMD_SPI_HEADER_LEN] != 0)
    {
        dev_dbg(&dev->intf->dev, "SPI config set failed.");
        retval = -EIO;
        goto done;
    }

done:
    mutex_unlock(&dev->io_mutex);
    return retval;
}

static int ch347_spi_setup(struct spi_device *spi)
{
    struct spi_master *master = spi->master;
    struct ch347_device *dev = ch347_spi_maser_to_dev(master);
    int retval;
    u8 scale;
    u8 clkdiv;

    if (spi->max_speed_hz > master->max_speed_hz || spi->max_speed_hz < master->min_speed_hz)
    {
        dev_err(&dev->intf->dev, "SPI speed out of range.");
        retval = -EINVAL;
        goto done;
    }

    if (spi->bits_per_word != 8)
    {
        dev_err(&dev->intf->dev, "SPI bits per word not supported.");
        retval = -EINVAL;
        goto done;
    }

    ch347_spi_get_cfg(dev);

    // setup bit order
    dev->spi_reg.bitoder = spi->mode & SPI_LSB_FIRST ? 0x80 : 0x00; // 0x80: LSB first, 0x00: MSB first

    // setup clock scale
    scale = spi->max_speed_hz / CH347_SPI_MIN_FREQ;
    clkdiv = 7;
    while (scale > 1)
    {
        scale >>= 1;
        clkdiv--;
    }
    dev->spi_reg.scale = __cpu_to_le16(clkdiv * 8);

    // setup clock polarity and phase
    switch (spi->mode & (SPI_CPOL | SPI_CPHA))
    {
    case SPI_MODE_0:
        dev->spi_reg.cpha = CH347_SPI_CPHA_1EDGE;
        dev->spi_reg.cpol = CH347_SPI_CPOL_LOW;
        break;
    case SPI_MODE_1:
        dev->spi_reg.cpha = CH347_SPI_CPHA_2EDGE;
        dev->spi_reg.cpol = CH347_SPI_CPOL_LOW;
        break;
    case SPI_MODE_2:
        dev->spi_reg.cpha = CH347_SPI_CPHA_1EDGE;
        dev->spi_reg.cpol = CH347_SPI_CPOL_HIGH;
        break;
    case SPI_MODE_3:
        dev->spi_reg.cpha = CH347_SPI_CPHA_2EDGE;
        dev->spi_reg.cpol = CH347_SPI_CPOL_HIGH;
        break;
    default:
        dev_err(&dev->intf->dev, "SPI mode not supported.");
        retval = -EINVAL;
        goto done;
    }

    dev->spi_reg.cpha = __cpu_to_le16(dev->spi_reg.cpha);
    dev->spi_reg.cpol = __cpu_to_le16(dev->spi_reg.cpol);

    // setup cs polarity
    if (spi->chip_select == 0)
    {
        if (spi->mode & SPI_CS_HIGH)
            dev->spi_reg.cs_polarity |= 0x80;
        else
            dev->spi_reg.cs_polarity &= ~0x80;
    }
    else
    {
        if (spi->mode & SPI_CS_HIGH)
            dev->spi_reg.cs_polarity |= 0x40;
        else
            dev->spi_reg.cs_polarity &= ~0x40;
    }

    // setup dummy and interval
    // TODO get dummy and interval from device tree
    dev->spi_reg.dummy = 0xFF;
    dev->spi_reg.interval = 0x00;

    retval = ch347_spi_set_cfg(dev);
    if (retval < 0)
    {
        dev_err(&dev->intf->dev, "SPI config set failed.");
        goto done;
    }

done:
    return retval;
}

static int ch347_spi_transfer_one_message(struct spi_master *master,
                                          struct spi_message *msg)
{
    struct ch347_device *dev = ch347_spi_maser_to_dev(master);
    struct spi_transfer *t;
    int ret;

    return 0;
}

int ch347_spi_probe(struct ch347_device *dev)
{
    struct platform_device *pdev;
    struct spi_master *master;
    struct spi_device *spi_device;
    int retval;
    int i;

    /* Register SPI Platform Device */
    pdev = platform_device_alloc("ch347_spi", 0);
    if (!pdev)
    {
        dev_err(&dev->intf->dev, "platform_device_alloc failed.");
        retval = -ENOMEM;
        goto err_spi_platform;
    }

    pdev->dev.parent = &dev->intf->dev;
    pdev->id = dev->id;

    retval = platform_device_add_data(pdev, NULL, 0);
    if (retval < 0)
    {
        dev_err(&dev->intf->dev, "platform_device_add_data failed.");
        goto err_spi_platform;
    }

    retval = platform_device_add(pdev);
    if (retval)
    {
        dev_err(&dev->intf->dev, "platform_device_add failed.");
        goto err_spi_platform;
    }

    dev->spi_pdev = pdev;

    /* Register SPI Master */
    master = spi_alloc_master(&dev->intf->dev, sizeof(struct ch34x_device *));

    if (!master)
    {
        dev_err(&dev->intf->dev, "spi_alloc_master failed.");
        retval = -ENOMEM;
        goto err_spi_master;
    }

    platform_set_drvdata(dev->spi_pdev, master);

    ch347_spi_maser_to_dev(master) = dev;

    master->bus_num = -1;
    master->num_chipselect = 2;
    master->mode_bits = SPI_MODE_3 | SPI_LSB_FIRST | SPI_CS_HIGH;
    master->bits_per_word_mask = SPI_BPW_MASK(8);
    master->transfer_one_message = ch347_spi_transfer_one_message;
    master->setup = ch347_spi_setup;
    master->max_speed_hz = CH347_SPI_MAX_FREQ;
    master->min_speed_hz = CH347_SPI_MIN_FREQ;

    retval = spi_register_master(master);
    if (retval)
    {
        dev_err(&dev->intf->dev, "spi_register_master failed.");
        goto err_spi_master;
    }

    dev->spi_master = master;

    /* Register SPI Device */
    for (i = 0; i < master->num_chipselect; i++)
    {
        spi_device = NULL;
        dev->spi_dev[i] = NULL;
        dev->spi_board_info[i] = ch347_spi_device_template;
        dev->spi_board_info[i].bus_num = master->bus_num;
        dev->spi_board_info[i].chip_select = i;
        spi_device = spi_new_device(master, &dev->spi_board_info[i]);
        if (!spi_device)
        {
            dev_err(&dev->intf->dev, "spi_new_device failed.");
            goto err_spi_device;
        }
        dev->spi_dev[i] = spi_device;
        dev_info(&dev->intf->dev, "spi device /dev/spidev%d.%d registered.", master->bus_num, spi_device->chip_select);
    }

    dev_dbg(&dev->intf->dev, "spi probe success.");

    return 0;

err_spi_device:
    for (i = 0; i < master->num_chipselect; i++)
    {
        if (dev->spi_dev[i])
        {
            spi_unregister_device(dev->spi_dev[i]);
            dev->spi_dev[i] = NULL;
        }
    }
err_spi_master:
    if (master)
    {
        spi_master_put(master);
        dev->spi_master = NULL;
    }
err_spi_platform:
    if (pdev)
    {
        platform_device_put(pdev);
        dev->spi_pdev = NULL;
    }
    return retval;
}

void ch347_spi_remove(struct ch347_device *dev)
{
    int i;
    if (dev->spi_master)
    {
        for (i = 0; i < dev->spi_master->num_chipselect; i++)
        {
            if (dev->spi_dev[i])
                spi_unregister_device(dev->spi_dev[i]);
        }
        spi_unregister_master(dev->spi_master);
    }
    if (dev->spi_pdev)
    {
        platform_device_unregister(dev->spi_pdev);
        dev->spi_pdev = NULL;
    }
}