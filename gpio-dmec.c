/*
 * GPIO driver for Data Modul AG Embedded Controller
 *
 * Copyright (C) 2016 Data Modul AG
 *
 * Authors: Zahari Doychev <zahari.doychev@linux.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio/driver.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include "dmec.h"

#define DMEC_GPIO_BANKS			2
#define DMEC_GPIO_MAX_NUM		8
#define DMEC_GPIO_BASE(x)		(0x40 + 0x10 * ((x)->chip_num))
#define DMEC_GPIO_SET_OFFSET(x)		(DMEC_GPIO_BASE(x) + 0x1)
#define DMEC_GPIO_GET_OFFSET(x)		(DMEC_GPIO_BASE(x) + 0x1)
#define DMEC_GPIO_CLR_OFFSET(x)		(DMEC_GPIO_BASE(x) + 0x2)
#define DMEC_GPIO_VER_OFFSET(x)		(DMEC_GPIO_BASE(x) + 0x2)
#define DMEC_GPIO_DIR_OFFSET(x)		(DMEC_GPIO_BASE(x) + 0x3)
#define DMEC_GPIO_IRQTYPE_OFFSET(x)	(DMEC_GPIO_BASE(x) + 0x4)
#define DMEC_GPIO_EVTSTA_OFFSET(x)	(DMEC_GPIO_BASE(x) + 0x6)
#define DMEC_GPIO_IRQCFG_OFFSET(x)	(DMEC_GPIO_BASE(x) + 0x8)
#define DMEC_GPIO_NOPS_OFFSET(x)	(DMEC_GPIO_BASE(x) + 0xa)
#define DMEC_GPIO_IRQSTA_OFFSET(x)	(DMEC_GPIO_BASE(x) + 0xb)

struct dmec_reg_ctx {
	u32 dat;
	u32 dir;
	u32 icfg[2];
	u32 emask[2];
};

struct dmec_gpio_priv {
	struct regmap *regmap;
	struct gpio_chip gpio_chip;
	struct irq_chip irq_chip;
	unsigned int chip_num;
	unsigned int irq;
	u8 ver;
	raw_spinlock_t lock;
	struct dmec_reg_ctx regs;
};

static int dmec_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned int val;

	/* read get register */
	regmap_read(regmap, DMEC_GPIO_GET_OFFSET(priv), &val);

	return !!(val & BIT(offset));
}

static void dmec_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;

	if (value)
		regmap_write(regmap, DMEC_GPIO_SET_OFFSET(priv), BIT(offset));
	else
		regmap_write(regmap, DMEC_GPIO_CLR_OFFSET(priv), BIT(offset));
}

static int dmec_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);
	/* set pin as input */
	regmap_update_bits(regmap, DMEC_GPIO_DIR_OFFSET(priv), BIT(offset), 0);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int dmec_gpio_direction_output(struct gpio_chip *gc, unsigned int offset,
				      int value)
{
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned int val = BIT(offset);
	unsigned long flags;

	raw_spin_lock_irqsave(&priv->lock, flags);
	if (value)
		regmap_write(regmap, DMEC_GPIO_SET_OFFSET(priv), val);
	else
		regmap_write(regmap, DMEC_GPIO_CLR_OFFSET(priv), val);

	/* set pin as output */
	regmap_update_bits(regmap, DMEC_GPIO_DIR_OFFSET(priv), val, val);

	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int dmec_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned int val;

	regmap_read(regmap, DMEC_GPIO_DIR_OFFSET(priv), &val);

	return !(val & BIT(offset));
}

static int dmec_gpio_pincount(struct dmec_gpio_priv *priv)
{
	struct regmap *regmap = priv->regmap;
	unsigned int val;

	regmap_read(regmap, DMEC_GPIO_NOPS_OFFSET(priv), &val);

	/* number of pins is val + 1 */
	return val == 0xff ? 0 : (val & 7) + 1;
}

static int dmec_gpio_get_version(struct gpio_chip *gc)
{
	struct device *dev = gc->parent;
	struct dmec_gpio_priv *p = gpiochip_get_data(gc);
	unsigned int v;

	regmap_read(p->regmap, DMEC_GPIO_VER_OFFSET(p), &v);
	p->ver = v;
	dev_info(dev, "chip%u v%u.%u\n", p->chip_num, (v >> 4) & 0xf, v & 0xf);

	return 0;
}

static void dmec_gpio_irq_enable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned long flags;
	int offset, mask;

	raw_spin_lock_irqsave(&priv->lock, flags);
	offset = DMEC_GPIO_IRQCFG_OFFSET(priv) + (d->hwirq >> 2);
	mask = BIT((d->hwirq & 3) << 1);

	regmap_update_bits(regmap, offset, mask, mask);
	raw_spin_unlock_irqrestore(&priv->lock, flags);
}

static void dmec_gpio_irq_disable(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned long flags;
	int offset, mask;

	raw_spin_lock_irqsave(&priv->lock, flags);
	offset = DMEC_GPIO_IRQCFG_OFFSET(priv) + (d->hwirq >> 2);
	mask = 3 << ((d->hwirq & 3) << 1);

	regmap_update_bits(regmap, offset, mask, 0);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

}

static int dmec_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct dmec_gpio_priv *priv = gpiochip_get_data(gc);
	struct regmap *regmap = priv->regmap;
	unsigned int offset, mask, val;
	unsigned long flags;

	offset = DMEC_GPIO_IRQTYPE_OFFSET(priv) + (d->hwirq >> 2);
	mask = ((d->hwirq & 3) << 1);

	raw_spin_lock_irqsave(&priv->lock, flags);
	regmap_read(regmap, offset, &val);

	val &= ~(3 << mask);
	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_LEVEL_LOW:
		irq_set_handler_locked(d, handle_level_irq);
		break;
	case IRQ_TYPE_EDGE_RISING:
		val |= (1 << mask);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val |= (2 << mask);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		val |= (3 << mask);
		irq_set_handler_locked(d, handle_edge_irq);
		break;
	default:
		return -EINVAL;
	}

	regmap_write(regmap, offset, val);
	raw_spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void dmec_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct dmec_gpio_priv *p = gpiochip_get_data(gc);
	u8 mask = BIT(irqd_to_hwirq(d) % DMEC_GPIO_MAX_NUM);

	regmap_write(p->regmap, DMEC_GPIO_EVTSTA_OFFSET(p), mask);
}

static irqreturn_t dmec_gpio_irq_handler(int irq, void *dev_id)
{
	struct dmec_gpio_priv *p = dev_id;
	struct irq_domain *d = p->gpio_chip.irq.domain;
	unsigned int stat, pending;
	unsigned long flags;

	raw_spin_lock_irqsave(&p->lock, flags);
	do {
		regmap_read(p->regmap, DMEC_GPIO_IRQSTA_OFFSET(p), &stat);
		pending = stat;
		while (pending) {
			int offset = __ffs(pending);

			generic_handle_irq(irq_find_mapping(d, offset));
			pending &= ~BIT(offset);
		}
	} while (stat);
	raw_spin_unlock_irqrestore(&p->lock, flags);

	return IRQ_HANDLED;
}

static int dmec_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dmec_gpio_priv *priv;
	struct gpio_chip *gpio_chip;
	struct irq_chip *irq_chip;
	int ret = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = dmec_get_regmap(pdev->dev.parent);
	priv->chip_num = pdev->id;

	gpio_chip = &priv->gpio_chip;
	gpio_chip->owner = THIS_MODULE;
	gpio_chip->parent = dev;
	gpio_chip->label = dev_name(dev);
	gpio_chip->base = -1;

	gpio_chip->direction_input = dmec_gpio_direction_input;
	gpio_chip->direction_output = dmec_gpio_direction_output;
	gpio_chip->get_direction = dmec_gpio_get_direction;
	gpio_chip->get = dmec_gpio_get;
	gpio_chip->set = dmec_gpio_set;
	gpio_chip->ngpio = dmec_gpio_pincount(priv);
	if (gpio_chip->ngpio == 0) {
		dev_err(dev, "No GPIOs detected\n");
		return -ENODEV;
	}

	raw_spin_lock_init(&priv->lock);
	irq_chip = &priv->irq_chip;
	irq_chip->name = dev_name(dev);
	irq_chip->irq_mask = dmec_gpio_irq_disable;
	irq_chip->irq_unmask = dmec_gpio_irq_enable;
	irq_chip->irq_set_type = dmec_gpio_irq_set_type;
	irq_chip->irq_ack = dmec_gpio_irq_ack;

	ret = devm_gpiochip_add_data(&pdev->dev, gpio_chip, priv);
	if (ret) {
		dev_err(dev, "Could not register GPIO chip\n");
		return ret;
	}

	ret = gpiochip_irqchip_add(gpio_chip, irq_chip, 0,
				   handle_bad_irq, IRQ_TYPE_NONE);
	if (ret) {
		dev_err(dev, "cannot add irqchip\n");
		return ret;
	}

	priv->irq = platform_get_irq(pdev, 0);
	gpiochip_set_chained_irqchip(gpio_chip, irq_chip,
				     priv->irq, NULL);

	ret = devm_request_irq(dev, priv->irq, dmec_gpio_irq_handler,
			       IRQF_ONESHOT | IRQF_SHARED,
			       dev_name(dev), priv);
	if (ret) {
		dev_err(dev, "unable to get irq: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, priv);

	dmec_gpio_get_version(gpio_chip);

	return 0;
}

static int dmec_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_PM
static int dmec_gpio_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dmec_gpio_priv *p = platform_get_drvdata(pdev);
	struct regmap *regmap = p->regmap;
	struct dmec_reg_ctx *ctx = &p->regs;

	regmap_read(regmap, DMEC_GPIO_BASE(p), &ctx->dat);
	regmap_read(regmap, DMEC_GPIO_DIR_OFFSET(p), &ctx->dir);
	regmap_read(regmap, DMEC_GPIO_IRQTYPE_OFFSET(p), &ctx->emask[0]);
	regmap_read(regmap, DMEC_GPIO_IRQTYPE_OFFSET(p) + 1, &ctx->emask[1]);
	regmap_read(regmap, DMEC_GPIO_IRQCFG_OFFSET(p), &ctx->icfg[0]);
	regmap_read(regmap, DMEC_GPIO_IRQCFG_OFFSET(p) + 1, &ctx->icfg[1]);

	return 0;
}

static int dmec_gpio_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dmec_gpio_priv *p = platform_get_drvdata(pdev);
	struct regmap *regmap = p->regmap;
	struct dmec_reg_ctx *ctx = &p->regs;

	regmap_write(regmap, DMEC_GPIO_BASE(p), ctx->dat);
	regmap_write(regmap, DMEC_GPIO_DIR_OFFSET(p), ctx->dir);
	regmap_write(regmap, DMEC_GPIO_IRQTYPE_OFFSET(p), ctx->emask[0]);
	regmap_write(regmap, DMEC_GPIO_IRQTYPE_OFFSET(p) + 1, ctx->emask[1]);
	regmap_write(regmap, DMEC_GPIO_IRQCFG_OFFSET(p), ctx->icfg[0]);
	regmap_write(regmap, DMEC_GPIO_IRQCFG_OFFSET(p) + 1, ctx->icfg[1]);
	regmap_write(regmap, DMEC_GPIO_EVTSTA_OFFSET(p), 0xff);

	return 0;
}

static SIMPLE_DEV_PM_OPS(dmec_gpio_pm, dmec_gpio_suspend, dmec_gpio_resume);

#define DMEC_GPIO_PM	(&dmec_gpio_pm)
#else
#define DMEC_GPIO_PM	NULL
#endif

static struct platform_driver dmec_gpio_driver = {
	.driver = {
		.name = "dmec-gpio",
		.owner = THIS_MODULE,
		.pm = DMEC_GPIO_PM,
	},
	.probe = dmec_gpio_probe,
	.remove	= dmec_gpio_remove,
};

module_platform_driver(dmec_gpio_driver);

MODULE_DESCRIPTION("dmec gpio driver");
MODULE_AUTHOR("Zahari Doychev <zahari.doychev@linux.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-gpio");
