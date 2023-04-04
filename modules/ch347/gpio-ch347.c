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

static int ch347_gpio_regs_get(struct gpio_chip *gpio)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;

    mutex_lock(&dev->mutex);

    memset(dev->bulk_out_buffer, 0, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN);

    dev->bulk_out_buffer[0] = CH347_CMD_GPIO_START;
    dev->bulk_out_buffer[1] = CH347_CMD_GPIO_COUNT;
    dev->bulk_out_buffer[2] = CH347_CMD_GPIO_END;

    ret = ch347_usb_xfer(dev, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, 1000);
    if (ret < 0)
    {
        goto error;
    }

    memcpy(&dev->gpio_regs, dev->bulk_in_buffer + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT);

    dev_dbg(&dev->intf->dev, "gpio regs: %02X %02X %02X %02X %02X %02X %02X %02X",
            dev->gpio_regs.pin[0].reg, dev->gpio_regs.pin[1].reg, dev->gpio_regs.pin[2].reg, dev->gpio_regs.pin[3].reg,
            dev->gpio_regs.pin[4].reg, dev->gpio_regs.pin[5].reg, dev->gpio_regs.pin[6].reg, dev->gpio_regs.pin[7].reg);

error:
    mutex_unlock(&dev->mutex);
    return ret;
}

static int ch347_gpio_regs_set(struct gpio_chip *gpio)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;

    mutex_lock(&dev->mutex);

    memset(dev->bulk_out_buffer, 0, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN);

    dev->bulk_out_buffer[0] = CH347_CMD_GPIO_START;
    dev->bulk_out_buffer[1] = CH347_CMD_GPIO_COUNT;
    dev->bulk_out_buffer[2] = CH347_CMD_GPIO_END;

    memcpy(dev->bulk_out_buffer + CH347_CMD_GPIO_HEADER_LEN, &dev->gpio_regs, CH347_CMD_GPIO_COUNT);

    ret = ch347_usb_xfer(dev, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, 1000);
    if (ret < 0)
    {
        goto error;
    }

    memcpy(&dev->gpio_regs, dev->bulk_in_buffer + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT);

    dev_dbg(&dev->intf->dev, "gpio regs: %02X %02X %02X %02X %02X %02X %02X %02X",
            dev->gpio_regs.pin[0].reg, dev->gpio_regs.pin[1].reg, dev->gpio_regs.pin[2].reg, dev->gpio_regs.pin[3].reg,
            dev->gpio_regs.pin[4].reg, dev->gpio_regs.pin[5].reg, dev->gpio_regs.pin[6].reg, dev->gpio_regs.pin[7].reg);

error:
    mutex_unlock(&dev->mutex);
    return ret;
}

static int ch347_gpio_get_direction(struct gpio_chip *gpio, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;
    u8 index;

    memset(dev->bulk_out_buffer, 0, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN);

    dev->bulk_out_buffer[0] = CH347_CMD_GPIO_START;
    dev->bulk_out_buffer[1] = CH347_CMD_GPIO_COUNT;
    dev->bulk_out_buffer[2] = CH347_CMD_GPIO_END;

    ret = ch347_usb_xfer(dev, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, 1000);

    memcpy(&dev->gpio_regs, dev->bulk_in_buffer + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT);

    if (ret < 0)
    {
        return ret;
    }

    return dev->bulk_in_buffer[CH347_CMD_GPIO_HEADER_LEN + index] & 0x80 ? GPIO_LINE_DIRECTION_OUT : GPIO_LINE_DIRECTION_IN;
}

static int ch347_gpio_direction_input(struct gpio_chip *gpio, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;
    u8 index = offset;

    dev->gpio_regs.pin[index].bits.direction = 0;

    ret = ch347_gpio_regs_set(gpio);

    return ret;
}

static int ch347_gpio_direction_output(struct gpio_chip *gpio, unsigned offset, int value)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;
    int index = offset;

    dev->gpio_regs.pin[index].bits.direction = 1;
    ret = ch347_gpio_regs_set(gpio);

    return ret;
}

static void ch347_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
    struct ch347_device *dev = gpiochip_get_data(chip);
    int index = offset;

    dev->gpio_regs.pin[index].bits.mode = value;
    dev->gpio_regs.pin[index].bits.value = value;

    ch347_gpio_regs_set(chip);
}

static int ch347_gpio_get(struct gpio_chip *chip, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(chip);
    int ret;

    ret = ch347_gpio_regs_get(chip);

    return dev->gpio_regs.pin[offset].bits.value;
}

int ch347_gpio_probe(struct ch347_device *dev)
{
    struct gpio_chip *gpio;
    struct gpio_irq_chip *gpio_irq;
    int ret;
    int i;

    gpio = devm_kzalloc(&dev->intf->dev, sizeof(*gpio), GFP_KERNEL);
    if (!gpio)
    {
        dev_err(&dev->intf->dev, "Failed to allocate memory for gpio");
        return -ENOMEM;
    }

    // gpio_irq = devm_kzalloc(&dev->intf->dev, sizeof(*gpio_irq), GFP_KERNEL);
    // if (!gpio_irq)
    // {
    //     dev_err(&dev->intf->dev, "Failed to allocate memory for gpio_irq");
    //     ret = -ENOMEM;
    //     goto error;
    // }

    gpio->label = "ch347-gpio";
    gpio->owner = THIS_MODULE;
    gpio->parent = &dev->intf->dev;
    gpio->base = -1;
    gpio->ngpio = 8;
    gpio->can_sleep = true;
    gpio->request = NULL;
    gpio->get_direction = ch347_gpio_get_direction;
    gpio->direction_input = ch347_gpio_direction_input;
    gpio->direction_output = ch347_gpio_direction_output;
    gpio->get = ch347_gpio_get;
    gpio->set = ch347_gpio_set;

    gpio->of_node = of_find_compatible_node(NULL, NULL, "ch347-gpio");
    // if (!gpio->of_node)
    // {
    //     ch347_gpio_property_set()
    // }

    if (ret < 0)
    {
        dev_err(&dev->intf->dev, "Failed to set gpio registers: %d", ret);
        goto error;
    }

    ret = gpiochip_add_data(gpio, dev);
    if (ret)
    {
        dev_err(&dev->intf->dev, "Failed to add gpiochip: %d", ret);
        gpio->base = -1;
        goto error;
    }

    // for (i = 0; i < gpio->ngpio; i++)
    // {
    //     dev->gpio_regs.pin[i].bits.enable = 1;
    //     dev->gpio_regs.pin[i].bits.direction = 0;
    // }

    // ret = ch347_gpio_regs_set(gpio);

    if (ret < 0)
    {
        dev_err(&dev->intf->dev, "Failed to set gpio registers: %d", ret);
        goto error;
    }

    dev_dbg(&dev->intf->dev, "Registered GPIOs %d..%d", gpio->base, gpio->base + gpio->ngpio - 1);
    dev->gpio = gpio;

    return 0;

error:
    if (gpio_irq)
    {
        kfree(gpio_irq);
    }
    if (gpio)
        kfree(gpio);
    return ret;
}

int ch347_gpio_remove(struct ch347_device *dev)
{
    int i;

    if (dev->gpio && dev->gpio->base > 0)
    {
        for (i = 0; i < dev->gpio->ngpio; i++)
            gpio_free(dev->gpio->base + i);

        gpiochip_remove(dev->gpio);
    }

    if (dev->gpio_irq)
        kfree(dev->gpio_irq);
    if (dev->gpio)
        kfree(dev->gpio);

    return 0;
}
