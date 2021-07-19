/*
 * dmec-core: Data Modul AG mfd embedded controller driver
 *
 * Zahari Doychev  <zahari.doychev@linux.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/acpi.h>
#include <linux/nls.h>
#include "dmec.h"

#define DMEC_LBAR			0x00 /* Index addressing 4 bytes */
#define DMEC_ECVER0			0x04
#define DMEC_ECVER1			0x05
#define DMEC_ECFTR0			0x07
#define DMEC_ECFTR1			0x08
#define DMEC_FPGAVER0			0x0c
#define DMEC_FPGAVER1			0x0d
#define DMEC_FPGABLD0			0x0e
#define DMEC_FPGABLD1			0x0f
#define DMEC_IRQCFG0			0x10
#define DMEC_IRQCFG1			0x11
#define DMEC_RTM_START			0x60
#define DMEC_RTM_END			0x6e
#define DMEC_PWM_START			0x80
#define DMEC_PWM_END			0x88

#define DMEC_MAX_GPIO_CHIPS		2

#define DMEC_VERSION_LEN		32

#define DMEC_FEATURE_BIT_I2C		BIT(0)
#define DMEC_FEATURE_BIT_WDT		BIT(4)
#define DMEC_FEATURE_BIT_GPIO		(3 << 6)

#define DMEC_REG_MAX		0x88
#define DMEC_MAX_DEVS		ARRAY_SIZE(dmec_dev_names)
#define DMEC_MAX_IO_RES		2
#define DMEC_STR_SZ		128

#define FTRPWM				(1 << 5)   /* PWM support flag */

static bool i_addr;
module_param(i_addr, bool, 0644);
MODULE_PARM_DESC(i_addr, "Enable register index addressing usage");

static const char * const fw_types[] = {"release", "custom",
					"debug", "reserved"};

enum dmec_cells {
	DMEC_I2C = 0,
	DMEC_GPIOA,
	DMEC_GPIOB,
	DMEC_WDT,
	DMEC_RTM,
	DMEC_ACPI,
	DMEC_PWM
};

struct dmec_features {
	unsigned int i2c_buses:2;
	unsigned int uarts:2;
	unsigned int wdt:1;
	unsigned int rsvd1:1;
	unsigned int gpio_chips:2;
	unsigned int spi_buses:2;
	unsigned int can_buses:2;
	unsigned int rsvd2:1;
	unsigned int nmi:1;
	unsigned int sci:1;
	unsigned int smi:1;
};

struct dmec_info {
	unsigned int ec_ver:4;
	unsigned int ec_rev:4;
	unsigned int ec_num:2;
	unsigned int ec_type:2;
	unsigned int ec_dbg:4;
	u16 fpga_ver;
	u16 fpga_bld;
	char version[DMEC_VERSION_LEN];
};

struct dmec_device_data {
	void __iomem *io_base;
	void __iomem *io_index;
	void __iomem *io_data;
	union {
		u16 feature_mask;
		struct dmec_features ftr;
	} u;
	struct device *dev;
	struct dmec_info info;
	struct regmap *regmap;
	/* use index addressing for register access if set*/
	bool i_addr;
};

struct dmec_platform_data {
	int (*get_info)(struct dmec_device_data *);
	int (*register_cells)(struct dmec_device_data *);
};

static struct dmec_i2c_platform_data dmec_i2c_data = {
	.clock_khz	= 50000,	/* input clock of 50MHz */
};

/* The gpio block can use up to DMEC_GPIO_MAX_IRQS APIC irqs */
static struct resource dmec_gpio_irq_resources[DMEC_MAX_GPIO_CHIPS];
static struct resource dmec_wdt_irq_resource;
static struct resource dmec_i2c_irq_resource;

static const char *dmec_dev_names[] = {
	[DMEC_I2C]   = "dmec-i2c",
	[DMEC_GPIOA] = "dmec-gpio",
	[DMEC_GPIOB] = "dmec-gpio",
	[DMEC_WDT]   = "dmec-wdt",
	[DMEC_RTM]   = "dmec-rtm",
	[DMEC_ACPI]  = "dmec-acpi",
	[DMEC_PWM]   = "dmec-pwm",
};

static void dmec_get_gpio_irqs(struct dmec_device_data *ec)
{
	unsigned int irq, val;

	regmap_read(ec->regmap, DMEC_IRQCFG1, &val);
	irq = (val >> 4) & 0xf;
	dmec_gpio_irq_resources[0].start = irq;
	dmec_gpio_irq_resources[0].end = irq;
	dmec_gpio_irq_resources[0].flags = IORESOURCE_IRQ;
	irq = val & 0xf;
	dmec_gpio_irq_resources[1].start = irq;
	dmec_gpio_irq_resources[1].end = irq;
	dmec_gpio_irq_resources[1].flags = IORESOURCE_IRQ;
}

static void dmec_get_wdt_irq(struct dmec_device_data *ec)
{
	unsigned int irq, val;

	regmap_read(ec->regmap, DMEC_IRQCFG0, &val);
	irq = val & 0xf;
	dmec_wdt_irq_resource.start = irq;
	dmec_wdt_irq_resource.end = irq;
	dmec_wdt_irq_resource.flags = IORESOURCE_IRQ;
}

static void dmec_get_i2c_irq(struct dmec_device_data *ec)
{
	unsigned int irq, val;

	regmap_read(ec->regmap, DMEC_IRQCFG0, &val);
	irq = (val >> 4) & 0xf;
	dmec_i2c_irq_resource.start = irq;
	dmec_i2c_irq_resource.end = irq;
	dmec_i2c_irq_resource.flags = IORESOURCE_IRQ;
}

static int dmec_rtm_detect(struct dmec_device_data *ec)
{
	unsigned int val, n;
	int ret = 0;

	for (n = DMEC_RTM_START; n <= DMEC_RTM_END; n++) {
		ret = regmap_read(ec->regmap, n, &val);
		if (val != 0xff && ret == 0)
			return 1;
	}
	return 0;
}

static int dmec_pwm_detect(struct dmec_device_data *ec)
{
	unsigned int val = 0, n;
	int ret = 0;

	/* check PWM support */
	ret = regmap_read(ec->regmap, DMEC_ECFTR0, &val);
	if(ret < 0)
	{
		dev_err(ec->dev, "Cannot read Embedded Controller Information\n");
		return -EPERM;
	}
	if(0 == (val & FTRPWM))
	{
		dev_err(ec->dev, "PWM is not supported\n");
		return -ENODEV;
	}

	for (n = DMEC_PWM_START; n <= DMEC_PWM_END; n++) {
		ret = regmap_read(ec->regmap, n, &val);
		if (val != 0xff && ret == 0)
			return 1;
	}
	return 0;
}

static int dmec_register_cells(struct dmec_device_data *ec)
{
	struct mfd_cell cells[DMEC_MAX_DEVS] = {};
	u8 n_dev = 0;

	if (ec->u.feature_mask & DMEC_FEATURE_BIT_I2C) {
		dmec_get_i2c_irq(ec);
		cells[n_dev].name = dmec_dev_names[DMEC_I2C];
		cells[n_dev].platform_data = &dmec_i2c_data;
		cells[n_dev].pdata_size = sizeof(dmec_i2c_data);
		cells[n_dev].resources = &dmec_i2c_irq_resource;
		cells[n_dev].num_resources = 1;
		cells[n_dev].id = 0;
	}

	if (ec->u.feature_mask & DMEC_FEATURE_BIT_GPIO) {
		n_dev++;
		dmec_get_gpio_irqs(ec);
		cells[n_dev].name = dmec_dev_names[DMEC_GPIOA];
		if (dmec_gpio_irq_resources[0].start != 0) {
			cells[n_dev].resources = &dmec_gpio_irq_resources[0];
			cells[n_dev].num_resources = 1;
		}		cells[n_dev].id = 0;
		if (ec->u.ftr.gpio_chips > 1) {
			n_dev++;
			cells[n_dev].name = dmec_dev_names[DMEC_GPIOB];
			if (dmec_gpio_irq_resources[1].start != 0) { 
				cells[n_dev].resources = &dmec_gpio_irq_resources[1];
				cells[n_dev].num_resources = 1;
			}
			cells[n_dev].id = 1;
		}
	}

	if (ec->u.feature_mask & DMEC_FEATURE_BIT_WDT) {
		n_dev++;
		dmec_get_wdt_irq(ec);
		cells[n_dev].name = dmec_dev_names[DMEC_WDT];
		cells[n_dev].resources = &dmec_wdt_irq_resource;
		cells[n_dev].num_resources = 1;
		cells[n_dev].id = 0;
	}

	if (dmec_rtm_detect(ec)) {
		n_dev++;
		cells[n_dev].name = dmec_dev_names[DMEC_RTM];
		cells[n_dev].id = 0;
	}

	if (dmec_pwm_detect(ec)) {
		n_dev++;
		cells[n_dev].name = dmec_dev_names[DMEC_PWM];
		cells[n_dev].id = 0;
	}
	
	n_dev++;
	cells[n_dev].name = dmec_dev_names[DMEC_ACPI];
	cells[n_dev].id = 0;

	return devm_mfd_add_devices(ec->dev, 0,
				    cells, n_dev, NULL, 0, NULL);
}

static int dmec_read16(struct dmec_device_data *ec, u8 reg)
{
	unsigned int lsb, msb;
	int ret;

	ret = regmap_read(ec->regmap, reg, &lsb);
	ret = regmap_read(ec->regmap, reg + 0x1, &msb);

	return (msb << 8) | lsb;
}

static int dmec_get_info(struct dmec_device_data *ec)
{
	unsigned int ver0, ver1;

	regmap_read(ec->regmap, DMEC_ECVER0, &ver0);
	regmap_read(ec->regmap, DMEC_ECVER1, &ver1);
	if (ver0 == 0xff && ver1 == 0xff)
		return -ENODEV;

	ec->u.feature_mask = dmec_read16(ec, DMEC_ECFTR0);

	ec->info.ec_ver = (ver0 >> 4) & 0xf;
	ec->info.ec_rev = ver0 & 0xf;
	ec->info.ec_num = ver1 & 0x3;
	ec->info.ec_type = (ver1 >> 2) & 0x3;
	ec->info.ec_dbg = (ver1 >> 4) & 0xf;

	ec->info.fpga_ver = dmec_read16(ec, DMEC_FPGAVER0);
	ec->info.fpga_bld = dmec_read16(ec, DMEC_FPGABLD0);

	return 0;
}

static int dmec_regmap_reg_read(void *context,
				unsigned int reg, unsigned int *val)
{
	struct dmec_device_data *ec = context;

	if (ec->i_addr) {
		iowrite8(reg, ec->io_index);
		*val = ioread8(ec->io_data);
	} else {
		*val = ioread8(ec->io_base + reg);
	}

	return 0;
}

static int dmec_regmap_reg_write(void *context,
				 unsigned int reg, unsigned int val)
{
	struct dmec_device_data *ec = context;

	if (ec->i_addr) {
		iowrite8(reg, ec->io_index);
		iowrite8(val, ec->io_data);
	} else {
		iowrite8(val, ec->io_base + reg);
	}

	return 0;
}

struct regmap *dmec_get_regmap(struct device *dev)
{
	struct dmec_device_data *ec = dev_get_drvdata(dev);

	return ec->regmap;
}
EXPORT_SYMBOL(dmec_get_regmap);

static ssize_t dmec_version_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dmec_device_data *ec = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", ec->info.version);
}

static ssize_t dmec_fw_type_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct dmec_device_data *ec = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE,
			"ver:        %u.%u\n"
			"ec:         %u\n"
			"type:       %s\n"
			"debug:      %u\n"
			"fpga ver:   %x\n"
			"fpga build: %x\n",
			ec->info.ec_ver,
			ec->info.ec_rev,
			ec->info.ec_num,
			fw_types[ec->info.ec_type % (ARRAY_SIZE(fw_types) - 1)],
			ec->info.ec_dbg,
			ec->info.fpga_ver,
			ec->info.fpga_bld);
}

static ssize_t dmec_features_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct dmec_device_data *ec = dev_get_drvdata(dev);
	struct dmec_features *ftr = &ec->u.ftr;

	return scnprintf(buf, PAGE_SIZE,
			 "i2c buses:    %2u\n"
			 "uarts:        %2u\n"
			 "watchdog:     %2u\n"
			 "gpio chips:   %2u\n"
			 "spi buses:    %2u\n"
			 "can buses:    %2u\n"
			 "nmi:          %2u\n"
			 "sci:          %2u\n"
			 "smi:          %2u\n",
			 ftr->i2c_buses, ftr->uarts, ftr->wdt,
			 ftr->gpio_chips, ftr->spi_buses, ftr->can_buses,
			 ftr->nmi, ftr->sci, ftr->smi);
}

static DEVICE_ATTR(version, 0444, dmec_version_show, NULL);
static DEVICE_ATTR(fw_type, 0444, dmec_fw_type_show, NULL);
static DEVICE_ATTR(features, 0444, dmec_features_show, NULL);

static struct attribute *dmec_attributes[] = {
	&dev_attr_version.attr,
	&dev_attr_fw_type.attr,
	&dev_attr_features.attr,
	NULL
};

static const struct attribute_group ec_attr_group = {
	.attrs = dmec_attributes,
};

static int dmec_detect_device(struct dmec_device_data *ec)
{
	int ret;

	ret = dmec_get_info(ec);
	if (ret)
		return ret;

	ret = scnprintf(ec->info.version, sizeof(ec->info.version),
			"%u.%u",
			ec->info.ec_ver, ec->info.ec_rev);
	if (ret < 0)
		return ret;

	dev_info(ec->dev, "v%s (%s) features: %#x\n",
		 ec->info.version,
		 fw_types[ec->info.ec_type % (ARRAY_SIZE(fw_types) - 1)],
		 ec->u.feature_mask);

	ret = dmec_register_cells(ec);
	if (ret)
		return ret;

	return sysfs_create_group(&ec->dev->kobj, &ec_attr_group);
}

static const struct regmap_config dmec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_stride = 1,
	.max_register =	DMEC_REG_MAX,
	.reg_write = dmec_regmap_reg_write,
	.reg_read = dmec_regmap_reg_read,
	.cache_type = REGCACHE_NONE,
	.fast_io = true,
};

static int dmec_mfd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dmec_device_data *ec;
	struct resource *io_idx, *io;
	int ret;

	ret = platform_device_add_data(pdev, NULL, 0);
	if (ret)
		return ret;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	io_idx = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!io_idx)
		return -EINVAL;

	io = platform_get_resource(pdev, IORESOURCE_IO, 1);
	if (!io) {
		dev_info(dev, "falling back to index addressing\n");
		ec->io_base = devm_ioport_map(dev, io_idx->start,
					      io_idx->end - io_idx->start);
		i_addr = true;
	} else
		ec->io_base = devm_ioport_map(dev, io->start,
					      io->end - io->start);

	if (IS_ERR(ec->io_base))
		return PTR_ERR(ec->io_base);

	/* In index mode registers @ 0x0 and 0x2 are used by the BIOS */
	ec->i_addr = i_addr;
	ec->io_index = ec->io_base + 1;
	ec->io_data = ec->io_base + 3;
	ec->dev = dev;

	ec->regmap = devm_regmap_init(dev, NULL, ec, &dmec_regmap_config);
	if (IS_ERR(ec->regmap))
		return PTR_ERR(ec->regmap);
	regcache_cache_bypass(ec->regmap, true);

	platform_set_drvdata(pdev, ec);

	return dmec_detect_device(ec);
}

static int dmec_mfd_remove(struct platform_device *pdev)
{
	struct dmec_device_data *ec = platform_get_drvdata(pdev);

	sysfs_remove_group(&ec->dev->kobj, &ec_attr_group);

	return 0;
}

static const struct acpi_device_id dmec_acpi_ids[] = {
	{"DMEC0001", 0},
	{"", 0},
};
MODULE_DEVICE_TABLE(acpi, dmec_acpi_ids);

static struct platform_driver dmec_driver = {
	.probe = dmec_mfd_probe,
	.remove = dmec_mfd_remove,
	.driver = {
		.name = "dmec",
		.acpi_match_table = ACPI_PTR(dmec_acpi_ids)
	},
};

module_platform_driver(dmec_driver);

MODULE_DESCRIPTION("DMO embedded controller core driver");
MODULE_AUTHOR("Zahari Doychev <zahari.doychev@linux.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-core");
