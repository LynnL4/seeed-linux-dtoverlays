/*
 * WCH CH347 USB to SPI/I2C/GPIO converter driver
 *
 *
 * Copyright (c) 2023 Seeed Studio
 * Hongtai Liu <lht856@foxmail.com>
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef _CH34X_MPSI_H
#define _CH34X_MPSI_H

#define DEBUG
#define VERBOSE_DEBUG

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#define CH347_USBDEV (&(ch347_dev->intf->dev))
#define DEV_ERR(d, f, ...) dev_err(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_DBG(d, f, ...) dev_dbg(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)
#define DEV_INFO(d, f, ...) dev_info(d, "%s: " f "\n", __FUNCTION__, ##__VA_ARGS__)

#define CH347_USB_MAX_BULK_SIZE 510
#define CH347_USB_MAX_INTR_SIZE 512
#define CH347_MAX_BUFFER_LENGTH 4096

/******************************************************/

#define CH347_GPIO_NUM_PINS 8

#define CH347_GPIO_MODE_INPUT 0x00
#define CH347_GPIO_MODE_OUTPUT 0x01

#define CH347_CMD_GPIO_HEADER_LEN 3
#define CH347_CMD_GPIO_START 0xCC
#define CH347_CMD_GPIO_COUNT 0x08
#define CH347_CMD_GPIO_END 0x00

/********************************************************/
#define CH347_I2C_LOW_SPEED 0	   /* low rate 20KHz */
#define CH347_I2C_STANDARD_SPEED 1 /* standard rate 100KHz */
#define CH347_I2C_FAST_SPEED 2	   /* fast rate 400KHz */
#define CH347_I2C_HIGH_SPEED 3	   /* high rate 750KHz */

#define CH347_I2C_WRITE_MAX_LENGTH 0x3F
#define CH347_I2C_READ_MAX_LENGTH 0x3F

#define CH347_CMD_I2C_STREAM 0xAA
#define CH347_CMD_I2C_STM_STA 0x74
#define CH347_CMD_I2C_STM_STO 0x75
#define CH347_CMD_I2C_STM_OUT 0x80
#define CH347_CMD_I2C_STM_IN 0xC0
#define CH347_CMD_I2C_STM_SET 0x60
#define CH347_CMD_I2C_STM_END 0x00

#ifndef USB_DEVICE_INTERFACE_NUMBER
#define USB_DEVICE_INTERFACE_NUMBER(vend, prod, num)                                                \
	.match_flags = USB_DEVICE_ID_MATCH_DEVICE | USB_DEVICE_ID_MATCH_INT_NUMBER, .idVendor = (vend), \
	.idProduct = (prod), .bInterfaceNumber = (num)
#endif

#pragma pack(1)

struct ch347_gpio_regs
{
	union gpio_pin_reg
	{
		u8 reg;
		struct
		{
			u8 enable : 2;
			u8 direction : 1;
			u8 mode : 1;
			u8 value : 1;
			u8 irq_enable : 1;
			u8 irq_type : 2;
		} bits;
	} pin[CH347_GPIO_NUM_PINS];
};

/* device specific structure */
struct ch347_device
{
	/* usb */
	int usb_id;					/* usb id */
	struct usb_device *usb_dev; /* usb device */
	struct usb_interface *intf; /* usb interface */
	struct mutex mutex;			/* mutex for usb operations */

	struct usb_endpoint_descriptor *bulk_in;  /* usb endpoint bulk in */
	struct usb_endpoint_descriptor *bulk_out; /* usb endpoint bulk out */
	struct usb_endpoint_descriptor *intr_in;  /* usb endpoint interrupt in */

	int bulk_in_size;  /* bulk in size */
	int bulk_out_size; /* bulk out size */
	int intr_in_size;  /* interrupt in size */

	u8 *bulk_in_buffer;	 /* buffer for bulk in */
	u8 *bulk_out_buffer; /* buffer for bulk out */
	u8 *intr_in_buffer;	 /* buffer for interrupt in */

	struct urb *intr_urb;

	/* gpio */
	struct ch347_gpio_regs gpio_regs;
	struct gpio_chip *gpio;
	struct gpio_irq_chip *gpio_irq;

	/* i2c */
	struct i2c_adapter *adapter;

	spinlock_t irq_lock;
};

#pragma pack()

int ch347_usb_xfer(struct ch347_device *dev, int tx_len, int rx_len, int timeout);
int ch347_gpio_probe(struct ch347_device *ch347_dev);
int ch347_gpio_remove(struct ch347_device *ch347_dev);
int ch347_i2c_probe(struct ch347_device *ch347_dev);
void ch347_i2c_remove(struct ch347_device *dev);

#endif
