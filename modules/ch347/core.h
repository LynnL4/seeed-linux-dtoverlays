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
#include <linux/spi/spi.h>
#include <linux/platform_device.h>

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

#define CH347_I2C_WRITE_MAX_LENGTH 0x20
#define CH347_I2C_READ_MAX_LENGTH 0x20

#define CH347_CMD_I2C_STREAM 0xAA
#define CH347_CMD_I2C_STM_STA 0x74
#define CH347_CMD_I2C_STM_STO 0x75
#define CH347_CMD_I2C_STM_OUT 0x80
#define CH347_CMD_I2C_STM_IN 0xC0
#define CH347_CMD_I2C_STM_SET 0x60
#define CH347_CMD_I2C_STM_END 0x00

/********************************************************/
#define CH347_SPI_MAX_NUM_DEVICES 2

#define CH347_SPI_MAX_FREQ 60e6
#define CH347_SPI_MIN_FREQ 468750
#define CH347_SPI_MIN_BITS_PER_WORD 4
#define CH347_SPI_MAX_BITS_PER_WORD 32

#define CH347_CMD_SPI_HEADER_LEN 3
#define CH347_CMD_SPI_STREAM 0xAB
#define CH347_CMD_SPI_CONGIG_W 0xC0
#define CH347_CMD_SPI_CONFIG_R 0xCA
#define CH347_CMD_SPI_CTRL 0xC1
#define CH347_CMD_SPI_RW 0xC2
#define CH347_CMD_SPI_BURST_R 0xC3
#define CH347_CMD_SPI_BURST_W 0xC4

#define CH347_SPI_SET_CS 0
#define CH347_SPI_CLR_CS 1
#define CH347_SPI_CS_ACTIVE 0x00
#define CH347_SPI_CS_DEACTIVE 0x01

/* SPI_Clock_Polarity */
#define CH347_SPI_CPOL_LOW ((u16)0x0000)
#define CH347_SPI_CPOL_HIGH ((u16)0x0002)

/* SPI_Clock_Phase */
#define CH347_SPI_CPHA_1EDGE ((u16)0x0000)
#define CH347_SPI_CPHA_2EDGE ((u16)0x0001)

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

struct ch347_spi_reg
{
	u16 direction;	/* Specifies the SPI unidirectional or bidirectional data mode. */
	u16 mode;		/* Specifies the SPI operating mode. */
	u16 data_size;	/* Specifies the SPI data size. */
	u16 cpol;		/* Specifies the serial clock steady state. */
	u16 cpha;		/* Specifies the clock active edge for the bit capture. */
	u16 nss;		/* Specifies whether the NSS signal is managed by hardware (NSS pin) or by software. */
	u16 scale;		/* Specifies the SPI baud rate prescaler. */
	u16 bitoder;	/* Specifies whether data transfers start from MSB or LSB bit. */
	u16 crc_poly;	/* Specifies the polynomial used for the CRC calculation. */
	u16 interval;	/* Specifies the interval between two consecutive data transfers in unit of us. */
	u8 dummy;		/* Specifies the dummy data sent to the SPI slave device in bidirectional mode. */
	u8 cs_polarity; /* Specifies the miscellaneous control bits.
				   BIT7: CS0 polar control, 0：low active, 1：high active
				   BIT6：CS1 polar control, 0：low active, 1：high active
				   BIT5：I2C clock stretch control, 0：disable 1: enable
				   BIT4：generates NACK or not when read the last byte for I2C operation
				   BIT3-0：reserved
				*/
	u8 reserved[4]; /* Reserved, must be set to 0.*/
};

#pragma pack()

/* device specific structure */
struct ch347_device
{
	/* usb */
	int id;						/* usb id */
	struct usb_device *usb_dev; /* usb device */
	struct usb_interface *intf; /* usb interface */
	struct mutex io_mutex;		/* mutex for usb operations */

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

	/* spi */
	struct ch347_spi_reg spi_reg;
	struct platform_device *spi_pdev;
	struct spi_master *spi_master;
	struct spi_device *spi_dev[CH347_SPI_MAX_NUM_DEVICES];
	struct spi_board_info spi_board_info[CH347_SPI_MAX_NUM_DEVICES];

	spinlock_t irq_lock;
};

int ch347_usb_xfer(struct ch347_device *dev, int tx_len, int rx_len, int timeout);
int ch347_gpio_probe(struct ch347_device *ch347_dev);
int ch347_gpio_remove(struct ch347_device *ch347_dev);
int ch347_i2c_probe(struct ch347_device *ch347_dev);
void ch347_i2c_remove(struct ch347_device *dev);
int ch347_spi_probe(struct ch347_device *ch347_dev);
void ch347_spi_remove(struct ch347_device *dev);

#endif
