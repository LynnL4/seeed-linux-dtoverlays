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

static DEFINE_IDA(ch347_devid_ida);

int ch347_usb_xfer(struct ch347_device *dev, int tx_len, int rx_len, int timeout)
{
    int retval = 0;
    int actual_length;
    int i = 0;

    dev_dbg(&dev->intf->dev, "tx_len: %d, rx_len: %d, timeout: %d", tx_len, rx_len, timeout);

    if (tx_len > 0)
    {
#ifdef DEBUG
        dev_dbg(&dev->intf->dev, "tx_len: %d, tx_buf: ", tx_len);
        for (i = 0; i < tx_len; i++)
        {
            printk("%02x ", dev->bulk_out_buffer[i]);
        }
        printk("\r\n");
#endif
        retval = usb_bulk_msg(dev->usb_dev, usb_sndbulkpipe(dev->usb_dev, dev->bulk_out->bEndpointAddress),
                              dev->bulk_out_buffer, tx_len, &actual_length, timeout);
        if (retval)
        {
            dev_err(&dev->intf->dev, "usb_bulk_msg() failed: %d", retval);
            goto error;
        }
        if (actual_length != tx_len)
        {
            dev_err(&dev->intf->dev, "usb_bulk_msg() sent %d bytes instead of %d", actual_length, tx_len);
            retval = -EIO;
            goto error;
        }
    }

    if (rx_len > 0)
    {
        memset(dev->bulk_in_buffer, 0, rx_len);
        retval = usb_bulk_msg(dev->usb_dev, usb_rcvbulkpipe(dev->usb_dev, dev->bulk_in->bEndpointAddress),
                              dev->bulk_in_buffer, rx_len, &actual_length, timeout);
        if (retval)
        {
            dev_err(&dev->intf->dev, "usb_bulk_msg() failed: %d", retval);
            goto error;
        }
        if (actual_length != rx_len)
        {
            dev_err(&dev->intf->dev, "usb_bulk_msg() received %d bytes instead of %d", actual_length, rx_len);
            retval = -EIO;
            goto error;
        }
    }

    return 0;

error:
    return retval;
}

static int ch347_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_device *udev = usb_get_dev(interface_to_usbdev(interface));
    struct ch347_device *ch347_dev;
    struct usb_host_interface *iface_desc;
    struct usb_endpoint_descriptor *endpoint;
    int i;
    int retval = 0;

    /* allocate memory for our device state and initialize it */
    ch347_dev = kzalloc(sizeof(*ch347_dev), GFP_KERNEL);
    if (!ch347_dev)
    {
        dev_err(&interface->dev, "Out of memory");
        return -ENOMEM;
    }

    ch347_dev->usb_dev = udev;
    ch347_dev->intf = interface;

    ch347_dev->bulk_in_buffer = NULL;
    ch347_dev->bulk_out_buffer = NULL;
    ch347_dev->intr_in_buffer = NULL;

    iface_desc = interface->cur_altsetting;

    DEV_DBG(CH347_USBDEV, "bNumEndpoints=%d", iface_desc->desc.bNumEndpoints);

    /* set up the endpoint information */
    /* use only the first bulk-in and bulk-out endpoints */
    for (i = 0; i < iface_desc->desc.bNumEndpoints; i++)
    {
        endpoint = &iface_desc->endpoint[i].desc;
        DEV_DBG(CH347_USBDEV, "  endpoint=%d type=%d dir=%d addr=%0x", i, usb_endpoint_type(endpoint),
                usb_endpoint_dir_in(endpoint), usb_endpoint_num(endpoint));

        if (usb_endpoint_is_bulk_in(endpoint))
        {
            /* we found a bulk in endpoint */
            ch347_dev->bulk_in = endpoint;
            ch347_dev->bulk_in_size = usb_endpoint_maxp(endpoint);
            ch347_dev->bulk_in_buffer = kmalloc(ch347_dev->bulk_in_size, GFP_KERNEL);
            if (!ch347_dev->bulk_in_buffer)
            {
                dev_err(&interface->dev, "Could not allocate bulk_in_buffer");
                retval = -ENOMEM;
                goto error;
            }
            DEV_DBG(CH347_USBDEV, "  bulk_in_size=%d", ch347_dev->bulk_in_size);
        }

        if (usb_endpoint_is_bulk_out(endpoint))
        {
            /* we found a bulk out endpoint */
            ch347_dev->bulk_out = endpoint;
            ch347_dev->bulk_out_size = usb_endpoint_maxp(endpoint);
            ch347_dev->bulk_out_buffer = kmalloc(ch347_dev->bulk_out_size, GFP_KERNEL);
            if (!ch347_dev->bulk_out_buffer)
            {
                dev_err(&interface->dev, "Could not allocate bulk_out_buffer");
                retval = -ENOMEM;
                goto error;
            }
            DEV_DBG(CH347_USBDEV, "  bulk_out_size=%d", ch347_dev->bulk_out_size);
        }

        if (usb_endpoint_xfer_int(endpoint))
        {
            /* we found a interrupt in endpoint */
            ch347_dev->intr_in = endpoint;
            ch347_dev->intr_in_buffer = kmalloc(usb_endpoint_maxp(endpoint), GFP_KERNEL);
            ch347_dev->intr_in_size = usb_endpoint_maxp(endpoint);
            if (!ch347_dev->intr_in_buffer)
            {
                dev_err(&interface->dev, "Could not allocate intr_in_buffer");
                retval = -ENOMEM;
                goto error;
            }
            DEV_DBG(CH347_USBDEV, "  intr_in_size=%d", ch347_dev->intr_in_size);
        }
    }

    if (ch347_dev->bulk_in == NULL || ch347_dev->bulk_out == NULL || ch347_dev->intr_in == NULL)
    {
        dev_err(&interface->dev, "Could not find all endpoints");
        retval = -ENODEV;
        goto error;
    }
    /* save the pointer to the new ch347_device in USB interface device data */
    usb_set_intfdata(interface, ch347_dev);
    mutex_init(&ch347_dev->mutex);
    ch347_dev->usb_id = ida_simple_get(&ch347_devid_ida, 0, 0, GFP_KERNEL);
    if (ch347_dev->usb_id < 0)
    {
        retval = ch347_dev->usb_id;
        goto error;
    }
    /* register gpio device */
    // ch347_dev->gpio = NULL;
    // ch347_dev->gpio_irq = NULL;
    // retval = ch347_gpio_probe(ch347_dev);
    // if (retval < 0)
    //     goto error;
    retval = ch347_i2c_probe(ch347_dev);
    if (retval < 0)
    {
        dev_dbg(&interface->dev, "ch347_i2c_probe failed");
        goto error;
    }
    /* let the user know what node this device is now attached to */
    dev_info(&interface->dev, "USB CH347 device now attached to ch347-%d", interface->minor);

    return 0;

error:
    // ch347_gpio_remove(ch347_dev);
    if (ch347_dev->bulk_in_buffer)
        kfree(ch347_dev->bulk_in_buffer);
    if (ch347_dev->bulk_out_buffer)
        kfree(ch347_dev->bulk_out_buffer);
    if (ch347_dev->intr_in_buffer)
        kfree(ch347_dev->intr_in_buffer);
    kfree(ch347_dev);
    return retval;
}

static void ch341_usb_disconnect(struct usb_interface *usb_if)
{
    struct ch347_device *dev = usb_get_intfdata(usb_if);

    ch347_i2c_remove(dev);

    usb_set_intfdata(usb_if, NULL);
    usb_put_dev(dev->usb_dev);

    if (dev->bulk_in_buffer)
        kfree(dev->bulk_in_buffer);
    if (dev->bulk_out_buffer)
        kfree(dev->bulk_out_buffer);
    if (dev->intr_in_buffer)
        kfree(dev->intr_in_buffer);

    kfree(dev);
}

/* table of devices that work with this driver */
static const struct usb_device_id ch347_usb_table[] = {
    {USB_DEVICE_INTERFACE_NUMBER(0x1a86, 0x55db, 0x02)}, /* CH347 Mode1 SPI+IIC+UART */
    {}                                                   /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, ch347_usb_table);

static struct usb_driver ch347_usb_driver = {
    .name = "ch347 gadget ",
    .id_table = ch347_usb_table,
    .probe = ch347_usb_probe,
    .disconnect = ch341_usb_disconnect,
};

module_usb_driver(ch347_usb_driver);
MODULE_AUTHOR("Various");
MODULE_DESCRIPTION("ch347 USB to I2C/SPI/GPIO adapter");
MODULE_LICENSE("GPL v2");
