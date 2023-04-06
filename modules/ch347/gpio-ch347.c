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

static int ch347_gpio_regs_get(struct gpio_chip *gpio)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;

    mutex_lock(&dev->io_mutex);

    dev->bulk_out_buffer[0] = CH347_CMD_GPIO_START;
    dev->bulk_out_buffer[1] = CH347_CMD_GPIO_COUNT;
    dev->bulk_out_buffer[2] = CH347_CMD_GPIO_END;

    ret = ch347_usb_xfer(dev, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, 1000);
    if (ret < 0)
    {
        goto error;
    }

    memcpy(&dev->gpio_reg, dev->bulk_in_buffer + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT);

error:
    mutex_unlock(&dev->io_mutex);
    return ret;
}

static int ch347_gpio_regs_set(struct gpio_chip *gpio)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;

    mutex_lock(&dev->io_mutex);

    memset(dev->bulk_out_buffer, 0, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN);

    dev->bulk_out_buffer[0] = CH347_CMD_GPIO_START;
    dev->bulk_out_buffer[1] = CH347_CMD_GPIO_COUNT;
    dev->bulk_out_buffer[2] = CH347_CMD_GPIO_END;

    memcpy(dev->bulk_out_buffer + CH347_CMD_GPIO_HEADER_LEN, &dev->gpio_reg, CH347_CMD_GPIO_COUNT);

    ret = ch347_usb_xfer(dev, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT + CH347_CMD_GPIO_HEADER_LEN, 1000);
    if (ret < 0)
    {
        goto error;
    }

    memcpy(&dev->gpio_reg, dev->bulk_in_buffer + CH347_CMD_GPIO_HEADER_LEN, CH347_CMD_GPIO_COUNT);

error:
    mutex_unlock(&dev->io_mutex);
    return ret;
}

static int ch347_gpio_get_direction(struct gpio_chip *gpio, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    u8 index = offset;

    ch347_gpio_regs_get(gpio);

    return dev->gpio_reg.pin[index] == 0x80 ? 0 : 1;
}

static int ch347_gpio_direction_input(struct gpio_chip *gpio, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;
    u8 index = offset;

    memset(&dev->gpio_reg, 0, sizeof(dev->gpio_reg));

    dev->gpio_reg.pin[index] |= CH347_GPIO_ENABLE_BIT_MASK;
    dev->gpio_reg.pin[index] &= ~CH347_GPIO_MODE_BIT_MASK;
    dev->gpio_reg.pin[index] &= ~CH347_GPIO_DIRECTION_BIT_MASK;

    ret = ch347_gpio_regs_set(gpio);

    return ret;
}

static int ch347_gpio_direction_output(struct gpio_chip *gpio, unsigned offset, int value)
{
    struct ch347_device *dev = gpiochip_get_data(gpio);
    int ret;
    int index = offset;

    memset(&dev->gpio_reg, 0, sizeof(dev->gpio_reg));

    dev->gpio_reg.pin[index] |= CH347_GPIO_ENABLE_BIT_MASK;
    dev->gpio_reg.pin[index] |= CH347_GPIO_DIRECTION_BIT_MASK;
    dev->gpio_reg.pin[index] |= CH347_GPIO_MODE_BIT_MASK;
    if (value)
        dev->gpio_reg.pin[index] |= CH347_GPIO_VALUE_BIT_MASK;
    else
        dev->gpio_reg.pin[index] &= ~CH347_GPIO_VALUE_BIT_MASK;

    ret = ch347_gpio_regs_set(gpio);

    return ret;
}

static void ch347_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
    struct ch347_device *dev = gpiochip_get_data(chip);
    int index = offset;

    memset(&dev->gpio_reg, 0, sizeof(dev->gpio_reg));

    dev->gpio_reg.pin[index] |= CH347_GPIO_ENABLE_BIT_MASK;
    dev->gpio_reg.pin[index] |= CH347_GPIO_DIRECTION_BIT_MASK;
    dev->gpio_reg.pin[index] |= CH347_GPIO_MODE_BIT_MASK;
    dev->gpio_reg.pin[index] |= value ? CH347_GPIO_VALUE_BIT_MASK : 0;

    ch347_gpio_regs_set(chip);
}

static int ch347_gpio_get(struct gpio_chip *chip, unsigned offset)
{
    struct ch347_device *dev = gpiochip_get_data(chip);
    int ret;

    ch347_gpio_regs_get(chip);

    return (dev->gpio_reg.pin[offset] == 0x40) ? 1 : 0;
}

int ch347_gpio_probe(struct ch347_device *dev)
{
    struct gpio_chip *gpio = &dev->gpio;
    int retval;

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

    retval = gpiochip_add_data(gpio, dev);
    if (retval)
    {
        dev_err(&dev->intf->dev, "Failed to add gpiochip: %d", retval);
        gpio->base = -1;
        goto error;
    }

    retval = ch347_gpio_regs_set(gpio);

    if (retval < 0)
    {
        dev_err(&dev->intf->dev, "Failed to set gpio registers: %d", retval);
        gpiochip_remove(gpio);
        goto error;
    }

    memset(&dev->gpio_reg, 0, sizeof(dev->gpio_reg));

    dev_dbg(&dev->intf->dev, "Registered GPIOs %d..%d", gpio->base, gpio->base + gpio->ngpio - 1);

    return 0;

error:
    return retval;
}

void ch347_gpio_remove(struct ch347_device *dev)
{

    if (dev->gpio.base != -1)
    {
        gpiochip_remove(&dev->gpio);
    }
}
