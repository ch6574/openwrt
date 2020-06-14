// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPLD driver for the MikroTik RouterBoard 4xx series
 *
 * This driver provides access to a CPLD that interfaces between the SoC SPI bus
 * and other devices. Behind the CPLD there is a NAND flash chip and five LEDs.
 *
 * The CPLD supports SPI two-wire mode, in which two bits are transferred per
 * SPI clock cycle. The second bit is transmitted with the SoC's CS2 pin.
 *
 * The CPLD also acts as a GPIO expander.
 *
 * Copyright (C) 2008-2011 Gabor Juhos <juhosg@openwrt.org>
 * Copyright (C) 2008 Imre Kaloz <kaloz@openwrt.org>
 * Copyright (C) 2015 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2020 Christopher Hill <ch6574@gmail.com>
 *
 * This file was based on the driver for Linux 2.6.22 published by
 * MikroTik for their RouterBoard 4xx series devices.
*/
#include <linux/mfd/core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/module.h>
#include <linux/of_platform.h>

#include <mfd/rb4xx-cpld.h>

/* CPLD commands */
#define CPLD_CMD_WRITE_NAND	0x08	/* send cmd, n x send data, send idle */
#define CPLD_CMD_WRITE_CFG	0x09	/* send cmd, n x send cfg */
#define CPLD_CMD_READ_NAND	0x0a	/* send cmd, send idle, n x read data */
//#define CPLD_CMD_READ_FAST	0x0b	/* send cmd, 3 x addr, send idle, n x read data */
#define CPLD_CMD_GPIO8_HIGH	0x0c	/* send cmd */
#define CPLD_CMD_GPIO8_LOW	0x0d	/* send cmd */
#define CPLD_CMD_SDPWR_ON	0x0e	/* send cmd */
#define CPLD_CMD_SDPWR_OFF	0x0f	/* send cmd */
#define CPLD_CMD_MON_VOLTAGE	0x11	/* send cmd */
#define CPLD_CMD_MON_TEMP	0x12	/* send cmd */
#define CPLD_CMD_READ_VOLTS	0x1a	/* send cmd, 3 x read data (LSB 1st) */

static int rb4xx_cpld_write_nand(struct rb4xx_cpld *cpld, const void *tx_buf,
				 unsigned int len)
{
	struct spi_message m;
	static const u8 cmd = CPLD_CMD_WRITE_NAND;
	struct spi_transfer t[3] = {
		{
			.tx_buf = &cmd,
			.len = sizeof(cmd),
		}, {
			.tx_buf = tx_buf,
			.len = len,
			.tx_nbits = SPI_NBITS_DUAL,
		}, {
			.len = 1,
			.tx_nbits = SPI_NBITS_DUAL,
		},
	};

	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	spi_message_add_tail(&t[2], &m);
	return spi_sync(cpld->spi, &m);
}

static int rb4xx_cpld_read_nand(struct rb4xx_cpld *cpld, void *rx_buf,
				unsigned int len)
{
	struct spi_controller *ctlr = cpld->spi->controller;
	struct spi_message m;
	static const u8 cmd[2] = {
		CPLD_CMD_READ_NAND, 0
	};
	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = sizeof(cmd),
		}, {
			.rx_buf = rx_buf,
			.len = len,
		},
	};
	int ret = 1;
	unsigned int size32 = len & ~31;	// TODO needed?

	if (cpld->foo && ctlr->mem_ops && size32 > 32) {
		// Do it the new way
		// printk_ratelimited("RB4XX CPLD (mem) reading %d bytes\n", len);

		struct spi_mem_op op =
			SPI_MEM_OP(SPI_MEM_OP_CMD(0xff, 1), // 0xff magic opcode
				   SPI_MEM_OP_NO_ADDR,
				   SPI_MEM_OP_NO_DUMMY,
				   SPI_MEM_OP_DATA_IN(len, rx_buf, 1));

		// can't use this as it has NOR fallback builtin.
		// ret = spi_mem_exec_op(cpld->spimem, &op);
		ret = ctlr->mem_ops->exec_op(cpld->spimem, &op);

		t[1].len    -= size32;
		t[1].rx_buf += size32;
	}

	if (t[1].len > 0) {
		// Do the old way
		// printk_ratelimited("RB4XX CPLD (spi) reading %d bytes\n", t[1].len);

		spi_message_init(&m);
		spi_message_add_tail(&t[0], &m);
		spi_message_add_tail(&t[1], &m);
		ret = spi_sync(cpld->spi, &m);
	}

	// Hexdump a bit of what we read
	/*
	if (printk_ratelimit()) {
		print_hex_dump(KERN_ALERT, "RB4XX CPLD: ", DUMP_PREFIX_ADDRESS,
			       16, 1, rx_buf, min(len, 256), 1);
		printk("--------------------------------------------------\n");
	}
	*/

	return ret;
}

static int rb4xx_cpld_cmd(struct rb4xx_cpld *cpld, const void *tx_buf,
			  unsigned int len)
{
	struct spi_message m;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.len = len,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(cpld->spi, &m);
}

static int rb4xx_cpld_gpio_set_0_7(struct rb4xx_cpld *cpld, u8 values)
{
	/* GPIO 0-7 change can be sent via command + bitfield */
	u8 cmd[2] = {
		CPLD_CMD_WRITE_CFG, values
	};
	return rb4xx_cpld_cmd(cpld, &cmd, 2);
}

static int rb4xx_cpld_gpio_set_8(struct rb4xx_cpld *cpld, u8 value)
{
	/* GPIO 8 uses dedicated high/low commands */
	u8 cmd = CPLD_CMD_GPIO8_HIGH | !!(value);
	return rb4xx_cpld_cmd(cpld, &cmd, 1);
}



// TODO testing toggle
static ssize_t rb4xx_cpld_foo_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rb4xx_cpld *cpld = dev_get_drvdata(dev);
	return sprintf(buf, "%ld\n", cpld->foo);
}
static ssize_t rb4xx_cpld_foo_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rb4xx_cpld *cpld = dev_get_drvdata(dev);
	kstrtol(buf, 10, &cpld->foo);
	return count;
}

static DEVICE_ATTR(foo, S_IRUGO | S_IWUSR,
		   rb4xx_cpld_foo_show, rb4xx_cpld_foo_store);

static struct attribute *rb4xx_cpld_attrs[] = {
	&dev_attr_foo.attr,
	NULL
};
ATTRIBUTE_GROUPS(rb4xx_cpld);
// TODO testing toggle



static const struct mfd_cell rb4xx_cpld_cells[] = {
	{
		.name = "mikrotik,rb4xx-gpio",
		.of_compatible = "mikrotik,rb4xx-gpio",
	}, {
		.name = "mikrotik,rb4xx-nand",
		.of_compatible = "mikrotik,rb4xx-nand",
	},
};

static int rb4xx_cpld_probe(struct spi_mem *spimem)
{
	struct spi_device *spi = spimem->spi;
	struct device *dev = &spi->dev;
	struct rb4xx_cpld *cpld;
	int ret;

	cpld = devm_kzalloc(dev, sizeof(*cpld), GFP_KERNEL);
	if (!cpld)
		return -ENOMEM;

	dev_set_drvdata(dev, cpld);

	cpld->spimem		= spimem;
	cpld->spi		= spi;
	cpld->write_nand	= rb4xx_cpld_write_nand;
	cpld->read_nand		= rb4xx_cpld_read_nand;
	cpld->gpio_set_0_7	= rb4xx_cpld_gpio_set_0_7;
	cpld->gpio_set_8	= rb4xx_cpld_gpio_set_8;

	spi->mode = SPI_MODE_0 | SPI_TX_DUAL;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	// TODO testing toggle
	cpld->foo = 0;
	ret = sysfs_create_group(&dev->kobj, &rb4xx_cpld_group);
	if (ret) {
		dev_err(dev, "sysfs creation failed\n");
		return ret;
	}

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE,
				    rb4xx_cpld_cells,
				    ARRAY_SIZE(rb4xx_cpld_cells),
				    NULL, 0, NULL);
}

static int rb4xx_cpld_remove(struct spi_mem *spimem)
{
	return 0;
}

static const struct of_device_id rb4xx_cpld_dt_match[] = {
	{ .compatible = "mikrotik,rb4xx-cpld", },
	{ },
};
MODULE_DEVICE_TABLE(of, rb4xx_cpld_dt_match);

static struct spi_mem_driver rb4xx_cpld_driver = {
	.probe = rb4xx_cpld_probe,
	.remove = rb4xx_cpld_remove,
	.spidrv = {
		.driver = {
			.name = "rb4xx-cpld",
//			.bus = &spi_bus_type,
			.of_match_table = of_match_ptr(rb4xx_cpld_dt_match),
		},
	},
};

module_spi_mem_driver(rb4xx_cpld_driver);

MODULE_DESCRIPTION("Mikrotik RB4xx CPLD driver");
MODULE_AUTHOR("Gabor Juhos <juhosg@openwrt.org>");
MODULE_AUTHOR("Imre Kaloz <kaloz@openwrt.org>");
MODULE_AUTHOR("Bert Vermeulen <bert@biot.com>");
MODULE_AUTHOR("Christopher Hill <ch6574@gmail.com");
MODULE_LICENSE("GPL v2");
