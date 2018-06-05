/*
 * SC16IS740/750/760 tty serial driver - Copyright (C) 2014 Lancer
 * Author: James Martinez <jamesm@xxxxxxxxxxxxx>
 *
 * Single UART with I2C-bus/SPI interface,64 bytes of transmit and receive FIFOs, IrDA SIR built-in support
 * http://www.nxp.com/documents/data_sheet/SC16IS740_750_760.pdf
 * 
 * 
 * 
 *  Based on max3100.c, by Christian Pellegrin <chripell@evolware.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

/* SC16IS740/750/760 register set */
#define RHR 		0x00
#define THR 		0x00
#define IER 		0x08
#define FCR 		0x10
#define IIR 		0x10
#define LCR 		0x18
#define MCR 		0x20
#define LSR 		0x28
#define MSR 		0x30
#define SPR 		0x38
#define TCR 		0x40
#define TLR 		0x48
#define TXLVL		0x50
#define RXLVL		0x58
#define IODIR		0x60
#define IOST	 	0x68
#define IOINTEN 	0x70
#define IOCTRL 		0x80
#define EFCR 		0x88
/* SC16IS740/750/760 special register set */
#define DLL 		0x00
#define DLH 		0x08
/* SC16IS740/750/760 enhanced register set */
#define EFR 		0x10
#define XON1 		0x20
#define XON2		0x28
#define XOFF1		0x30
#define XOFF2		0x38



static const struct i2c_device_id sc16is740_750_760_id[] = {
	{ "sc16is740_750_760", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sc16is740_750_760_id);

static int sc16is740_750_760_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
                return -ENODEV;
	printk(KERN_INFO "sc16is740_750_760 probe\n");
		i2c_smbus_write_byte_data(client,LCR,0x80);
		i2c_smbus_write_byte_data(client,DLL,0x14);
		i2c_smbus_write_byte_data(client,DLH,0x00);

	return 0;
}

static int sc16is740_750_760_read(void) {
	return 0;
}

static int sc16is740_750_760__write(void) {
	return 0;
}

static int __devexit sc16is740_750_760_remove(struct i2c_client *client)
{
	printk(KERN_INFO "sc16is740_750_760 remove\n");
	return 0;
}

static unsigned short normal_i2c[] = { 0x4d, I2C_CLIENT_END };

static struct i2c_driver sc16is740_750_760_driver = {
	.driver.owner	= THIS_MODULE, 
	.driver.name	= "sc16is740_750_760",
	.driver.bus	= &i2c_bus_type,
	.probe		= sc16is740_750_760_probe,
	.remove		= __devexit_p(sc16is740_750_760_remove),
	.id_table	= sc16is740_750_760_id,
	.address_list   = normal_i2c,
};

static struct uart_driver sc16is740_750_760_uart_driver = {
	.owner	= THIS_MODULE,
	.driver_name = "ttySC",
	.dev_name = "ttySC",
};
static int uart_driver_registered;


static int __init  sc16is740_750_760_init(void)
{
	printk(KERN_INFO "sc16is740_750_760 loaded\n");
	return i2c_add_driver(&sc16is740_750_760_driver);
}

static void __exit sc16is740_750_760_exit(void)
{
	printk(KERN_INFO "sc16is740_750_760 unloaded\n");
	i2c_del_driver(&sc16is740_750_760_driver);
}

module_init(sc16is740_750_760_init);
module_exit(sc16is740_750_760_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Martinez <jmartinez@xxxxxxxxxxxxx>");
MODULE_DESCRIPTION("SC16IS740/750/760 i2c to serial driver");
MODULE_ALIAS("i2c:sc16is740_750_760");
