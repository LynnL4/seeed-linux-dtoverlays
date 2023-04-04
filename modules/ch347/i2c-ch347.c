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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>

#define CH347_I2C_WRITE_LENGTH_CHECK(len) \
    (len / CH347_I2C_WRITE_MAX_LENGTH * 3 + len + 6 > CH347_USB_MAX_BULK_SIZE) ? 0 : 1

#define CH347_I2C_READ_LENGTH_CHECK(len) \
    (len / CH347_I2C_READ_MAX_LENGTH * 3 + len + 6 > CH347_I2C_READ_MAX_LENGTH) ? 0 : 1

int ch347_i2c_read(struct ch347_device *dev, u8 addr, u8 *buf, int len)
{
    int retval;
    int avail_len = len;
    int xfer_len = 0;
    int offset = 0;
    uint8_t first_frame = 1;

    dev_dbg(&dev->intf->dev, "read %d bytes from 0x%02x", len, addr);

    while (avail_len)
    {
        offset = 0;
        if (avail_len > CH347_I2C_READ_MAX_LENGTH - first_frame)
        {
            xfer_len = CH347_I2C_READ_MAX_LENGTH - first_frame;
        }
        else
        {
            xfer_len = avail_len;
        }

        dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STREAM;
        // first frame
        if (first_frame)
        {
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_STA;
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_OUT | 1;
            dev->bulk_out_buffer[offset++] = addr << 1 | 1;
        }
        
        dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_IN | (xfer_len - 1);

        avail_len -= xfer_len;
        // last frame
        if (avail_len == 0)
        {
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_IN;
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_STO;
        }
        dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_END;

        retval = ch347_usb_xfer(dev, offset, (xfer_len -1) + first_frame, 1000);

        if (dev->bulk_in_buffer[0] != 0x01)
        {
            return -EIO;
        }

        if (retval < 0)
        {
            return retval;
        }

        memcpy(buf + (len - avail_len - xfer_len), dev->bulk_in_buffer + first_frame, xfer_len);

        // first frame is seny
        first_frame = first_frame ? 0 : 1;
    }

    return len;
}

int ch347_i2c_write(struct ch347_device *dev, u8 addr, u8 *buf, int len)
{
    int retval;
    int avail_len = len;
    int xfer_len = 0;
    int ack_len = 0;
    int offset = 0;
    int i = 0;
    uint8_t first_frame = 1;

    dev_dbg(&dev->intf->dev, "i2c write: %d bytes to 0x%02x", len, addr);

    do
    {
        offset = 0;
        if (avail_len > CH347_I2C_WRITE_MAX_LENGTH - first_frame)
        {
            xfer_len = CH347_I2C_WRITE_MAX_LENGTH - first_frame;
        }
        else
        {
            xfer_len = avail_len;
        }

        dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STREAM;
        // first frame
        if (first_frame)
        {
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_STA;
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_OUT | (xfer_len + 1);
            dev->bulk_out_buffer[offset++] = addr << 1;
            ack_len = xfer_len + 1;
        }
        else
        {
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_OUT | xfer_len;
            ack_len = xfer_len;
        }
        memcpy(dev->bulk_out_buffer + offset, buf + (len - avail_len), xfer_len);
        offset += xfer_len;
        avail_len -= xfer_len;
        // last frame
        if (avail_len == 0)
        {
            dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_STO;
        }
        dev->bulk_out_buffer[offset++] = CH347_CMD_I2C_STM_END;
        retval = ch347_usb_xfer(dev, offset, ack_len, 1000);
        if (retval < 0)
        {
            return retval;
        }

        for (i = 0; i < ack_len; i++)
        {
            if (!(dev->bulk_in_buffer[i] & 0x01))
            {
                return -EIO;
            }
        }

        // first frame is sent
        first_frame = first_frame ? 0 : 1;

    } while (avail_len);

    return retval;
}

static int ch347_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
    struct ch347_device *dev = i2c_get_adapdata(adapter);
    int i, ret;

    dev_dbg(&dev->intf->dev, "i2c xfer num: %d", num);

    if (num > 2)
    {
        return -EINVAL;
    }

    for (i = 0; i < num; i++)
    {
        if (msgs[i].flags & I2C_M_RD)
        {
            ret = ch347_i2c_read(dev, msgs[i].addr, msgs[i].buf, msgs[i].len);
        }
        else
        {
            ret = ch347_i2c_write(dev, msgs[i].addr, msgs[i].buf, msgs[i].len);
        }
    }

    return ret < 0 ? ret : num;
}

static u32 ch347_i2c_func(struct i2c_adapter *adapter)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static int ch34x_i2c_set_speed(struct ch347_device *dev, int speed)
{
    int retval;
    dev->bulk_out_buffer[0] = CH347_CMD_I2C_STREAM;
    dev->bulk_out_buffer[2] = CH347_CMD_I2C_STM_END;
    if (speed <= 20000) // 20K
    {
        dev->bulk_out_buffer[1] = CH347_CMD_I2C_STM_SET | CH347_I2C_LOW_SPEED;
    }
    else if (speed <= 100000) // 100K
    {
        dev->bulk_out_buffer[1] = CH347_CMD_I2C_STM_SET | CH347_I2C_STANDARD_SPEED;
    }
    else if (speed <= 400000) // 400K
    {
        dev->bulk_out_buffer[1] = CH347_CMD_I2C_STM_SET | CH347_I2C_FAST_SPEED;
    }
    else if (speed <= 3400000) // 3.4M
    {
        dev->bulk_out_buffer[1] = CH347_CMD_I2C_STM_SET | CH347_I2C_HIGH_SPEED;
    }
    else
    {
        return -EINVAL;
    }
    retval = ch347_usb_xfer(dev, 3, 0, 1000);

    return retval;
}

static const struct i2c_algorithm ch347_i2c_algorithm = {
    .master_xfer = ch347_i2c_xfer,
    .functionality = ch347_i2c_func,
};

int ch347_i2c_probe(struct ch347_device *dev)
{
    int retval = 0;
    struct i2c_adapter *adapter = NULL;

    dev->adapter = NULL;

    adapter = devm_kzalloc(&dev->intf->dev, sizeof(struct i2c_adapter), GFP_KERNEL);
    if (!adapter)
    {
        return -ENOMEM;
    }

    adapter->owner = THIS_MODULE;
    adapter->class = I2C_CLASS_HWMON;
    adapter->algo = &ch347_i2c_algorithm;
    adapter->dev.parent = &dev->intf->dev;

    snprintf(adapter->name, sizeof(adapter->name), "ch347-i2c at bus %03d device %03d",
             dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

    dev_dbg(&dev->intf->dev, "i2c adapter name: %s", adapter->name);

    retval = i2c_add_adapter(adapter);
    if (retval)
    {
        devm_kfree(&dev->intf->dev, adapter);
        return retval;
    }
    /* set ch34x i2c speed */
    retval = ch34x_i2c_set_speed(dev, 100000);
    if (retval < 0)
    {
        i2c_del_adapter(adapter);
        devm_kfree(&dev->intf->dev, adapter);
        return retval;
    }

    i2c_set_adapdata(adapter, dev);

    dev->adapter = adapter;

    dev_dbg(&dev->intf->dev, "ch347 i2c adapter attached at %d", dev->adapter->nr);

    return retval;
}

void ch347_i2c_remove(struct ch347_device *dev)
{
    if (dev->adapter)
    {
        i2c_del_adapter(dev->adapter);
        devm_kfree(&dev->intf->dev, dev->adapter);
    }
}