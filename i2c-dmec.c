/*
 * i2c-dmec.c: I2C bus driver for OpenCores I2C controller
 * (http://www.opencores.org/projects.cgi/web/i2c/overview).
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * Support for the GRLIB port of the controller by
 * Andreas Larsson <andreas@gaisler.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include "dmec.h"

/* registers */
#define DMECI2C_PRELOW		0
#define DMECI2C_PREHIGH		1
#define DMECI2C_CONTROL		2
#define DMECI2C_DATA		3
#define DMECI2C_CMD		4 /* write only */
#define DMECI2C_STATUS		4 /* read only, same address as DMECI2C_CMD */
#define DMECI2C_VER		5
#define DMECI2C_MUX		6 /* i2c demultiplezer */

#define DMECI2C_CTRL_EN		BIT(7)
#define DMECI2C_CTRL_IEN	BIT(6)
#define DMECI2C_CTRL_VSSCL	BIT(5)
#define DMECI2C_CTRL_MABCLR	BIT(4)
#define DMECI2C_CTRL_MMDIS	BIT(3)
#define DMECI2C_CTRL_VSSDA	BIT(2)

#define DMECI2C_CMD_START	0x91
#define DMECI2C_CMD_STOP	0x41
#define DMECI2C_CMD_READ	0x21
#define DMECI2C_CMD_WRITE	0x11
#define DMECI2C_CMD_READ_ACK	0x21
#define DMECI2C_CMD_READ_NACK	0x29
#define DMECI2C_CMD_IACK	0x01

#define DMECI2C_STAT_IF		0x01
#define DMECI2C_STAT_TIP	0x02
#define DMECI2C_STAT_ARBLOST	0x20
#define DMECI2C_STAT_BUSY	0x40
#define DMECI2C_STAT_NACK	0x80

#define STATE_DONE		0
#define STATE_START		1
#define STATE_WRITE		2
#define STATE_READ		3
#define STATE_ERROR		4

#define TYPE_OCORES		0
#define TYPE_GRLIB		1

#define DMEC_I2C_OFFSET		0x20
#define DMEC_I2C_MAX_BUS_NUM	3

#define DMEC_I2C_IP_CLK		50000 /* kHz */
#define DMEC_I2C_BUS_CLK	100   /* kHz */

struct dmec_i2c {
	struct device *dev;
	void __iomem *base;
	wait_queue_head_t wait;
	struct i2c_adapter adap;
	struct i2c_mux_core *mux;
	struct i2c_msg *msg;
	int pos;
	int nmsgs;
	int state; /* see STATE_ */
	struct clk *clk;
	int ip_clock_khz;
	int bus_clock_khz;

	u8 irq;
	struct regmap *regmap;
};

static int flags;
module_param(flags, int, 0644);
MODULE_PARM_DESC(flags, "additional flags for the i2c bus configuration");

static inline void dmec_i2c_setreg(struct dmec_i2c *i2c, int reg, u8 value)
{
	struct regmap *regmap = i2c->regmap;

	regmap_write(regmap, DMEC_I2C_OFFSET + reg, value);
}

static inline u8 dmec_i2c_getreg(struct dmec_i2c *i2c, int reg)
{
	struct regmap *regmap = i2c->regmap;
	unsigned int val;

	regmap_read(regmap, DMEC_I2C_OFFSET + reg, &val);

	return val;
}

static int dmec_i2c_dmx_select(struct i2c_mux_core *mux, u32 chan)
{
	struct dmec_i2c *i2c = mux->priv;
	u8 bus = chan & 0x3;

	dmec_i2c_setreg(i2c, DMECI2C_MUX, bus);

	return 0;
}

static void dmec_i2c_dmx_del(struct dmec_i2c *i2c)
{
	if (i2c->mux)
		i2c_mux_del_adapters(i2c->mux);
}

static int dmec_i2c_dmx_add(struct dmec_i2c *i2c)
{
	u8 bus_mask;
	int i, ret = 0;

	bus_mask = dmec_i2c_getreg(i2c, DMECI2C_MUX);
	bus_mask = (bus_mask & 0x70) >> 4;

	i2c->mux = i2c_mux_alloc(&i2c->adap,
				 i2c->dev,
				 DMEC_I2C_MAX_BUS_NUM, 0, 0,
				 dmec_i2c_dmx_select,
				 NULL);
	if (!i2c->mux)
		return -ENOMEM;

	i2c->mux->priv = i2c;

	for (i = 0; i < DMEC_I2C_MAX_BUS_NUM; i++) {
		if (!(bus_mask & (i + 1)))
			/* bus is not present so skip */
			continue;
		ret = i2c_mux_add_adapter(i2c->mux, 0, i, 0);
		if (ret) {
			ret = -ENODEV;
			dev_err(i2c->dev,
				"i2c dmx failed to register adapter %d\n", i);
			goto dmec_i2c_dmx_add_failed;
		}
	}

	return 0;

dmec_i2c_dmx_add_failed:
	dmec_i2c_dmx_del(i2c);
	return ret;
}

static void dmec_i2c_process(struct dmec_i2c *i2c)
{
	struct i2c_msg *msg = i2c->msg;
	u8 stat = dmec_i2c_getreg(i2c, DMECI2C_STATUS);

	if ((i2c->state == STATE_DONE) || (i2c->state == STATE_ERROR)) {
		/* stop has been sent */
		dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_IACK);
		wake_up(&i2c->wait);
		return;
	}

	/* error? */
	if (stat & DMECI2C_STAT_ARBLOST) {
		i2c->state = STATE_ERROR;
		dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_STOP);
		return;
	}

	if ((i2c->state == STATE_START) || (i2c->state == STATE_WRITE)) {
		i2c->state =
			(msg->flags & I2C_M_RD) ? STATE_READ : STATE_WRITE;

		if (stat & DMECI2C_STAT_NACK) {
			i2c->state = STATE_ERROR;
			dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_STOP);
			return;
		}
	} else {
		msg->buf[i2c->pos++] = dmec_i2c_getreg(i2c, DMECI2C_DATA);
	}

	/* end of msg? */
	if (i2c->pos == msg->len) {
		i2c->nmsgs--;
		i2c->msg++;
		i2c->pos = 0;
		msg = i2c->msg;

		if (i2c->nmsgs) {	/* end? */
			/* send start? */
			if (!(msg->flags & I2C_M_NOSTART)) {
				u8 addr = (msg->addr << 1);

				if (msg->flags & I2C_M_RD)
					addr |= 1;

				i2c->state = STATE_START;

				dmec_i2c_setreg(i2c, DMECI2C_DATA, addr);
				dmec_i2c_setreg(i2c, DMECI2C_CMD,
						DMECI2C_CMD_START);
				return;
			}
			i2c->state = (msg->flags & I2C_M_RD)
					? STATE_READ : STATE_WRITE;
		} else {
			i2c->state = STATE_DONE;
			dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_STOP);
			return;
		}
	}

	if (i2c->state == STATE_READ) {
		dmec_i2c_setreg(i2c, DMECI2C_CMD, i2c->pos == (msg->len - 1) ?
			  DMECI2C_CMD_READ_NACK : DMECI2C_CMD_READ_ACK);
	} else {
		dmec_i2c_setreg(i2c, DMECI2C_DATA, msg->buf[i2c->pos++]);
		dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_WRITE);
	}
}

static irqreturn_t dmec_i2c_isr(int irq, void *dev_id)
{
	struct dmec_i2c *i2c = dev_id;

	dmec_i2c_process(i2c);

	return IRQ_HANDLED;
}

static int dmec_i2c_xfer(struct i2c_adapter *adap,
			 struct i2c_msg *msgs, int num)
{
	struct dmec_i2c *i2c = i2c_get_adapdata(adap);

	i2c->msg = msgs;
	i2c->pos = 0;
	i2c->nmsgs = num;
	i2c->state = STATE_START;

	dmec_i2c_setreg(i2c, DMECI2C_DATA,
			(i2c->msg->addr << 1) |
			((i2c->msg->flags & I2C_M_RD) ? 1 : 0));

	dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_START);

	if (wait_event_timeout(i2c->wait, (i2c->state == STATE_ERROR) ||
			       (i2c->state == STATE_DONE), HZ))
		return (i2c->state == STATE_DONE) ? num : -EIO;
	else
		return -ETIMEDOUT;
}

static int dmec_i2c_init(struct device *dev, struct dmec_i2c *i2c)
{
	int prescale;
	int diff;
	u8 stat;
	u8 ctrl = dmec_i2c_getreg(i2c, DMECI2C_CONTROL);

	/* make sure the device is disabled */
	ctrl &= ~(DMECI2C_CTRL_EN | DMECI2C_CTRL_IEN |
		  DMECI2C_CTRL_VSSCL | DMECI2C_CTRL_VSSDA |
		  DMECI2C_CTRL_MABCLR | DMECI2C_CTRL_MMDIS);
	dmec_i2c_setreg(i2c, DMECI2C_CONTROL, ctrl);

	prescale = (i2c->ip_clock_khz / (8 * i2c->bus_clock_khz)) - 2;
	prescale = clamp(prescale, 0, 0xffff);

	diff = i2c->ip_clock_khz / (8 * (prescale + 2)) - i2c->bus_clock_khz;
	if (abs(diff) > i2c->bus_clock_khz / 10) {
		dev_err(dev,
			"Unsupported clock: core: %d KHz, bus: %d KHz\n",
			i2c->ip_clock_khz, i2c->bus_clock_khz);
		return -EINVAL;
	}

	dmec_i2c_setreg(i2c, DMECI2C_PRELOW, prescale & 0xff);
	dmec_i2c_setreg(i2c, DMECI2C_PREHIGH, prescale >> 8);

	/* default to first bus */
	dmec_i2c_setreg(i2c, DMECI2C_MUX, 0x0);

	/* Init the device */
	dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_IACK);
	ctrl |= DMECI2C_CTRL_EN;
	if (!flags)
		ctrl |= DMECI2C_CTRL_MABCLR | DMECI2C_CTRL_MMDIS;
	else
		ctrl |= (flags & 0x3c);
	dmec_i2c_setreg(i2c, DMECI2C_CONTROL, ctrl);

	stat = dmec_i2c_getreg(i2c, DMECI2C_STATUS);
	if (stat & DMECI2C_STAT_BUSY) {
		dev_warn(dev,
			 "I2C bus is busy - generating stop signal\n");
		dmec_i2c_setreg(i2c, DMECI2C_CMD, DMECI2C_CMD_STOP);
	}

	stat = dmec_i2c_getreg(i2c, DMECI2C_VER);
	dev_info(dev, "v%u.%u\n", (stat >> 4) & 0xf, stat & 0xf);

	return 0;
}

static int dmec_i2c_irq_enable(struct dmec_i2c *i2c)
{
	u8 ctrl;
	unsigned int irq;
	int ret;

	irq = i2c->irq;

	/* Initialize interrupt handlers if not already done */
	init_waitqueue_head(&i2c->wait);

	ret = devm_request_threaded_irq(i2c->dev, irq, NULL, dmec_i2c_isr,
					IRQF_ONESHOT | IRQF_SHARED,
					i2c->adap.name, i2c);
	if (ret) {
		dev_err(i2c->dev,
			"Unable to claim IRQ\n");
		return ret;
	}

	ctrl = dmec_i2c_getreg(i2c, DMECI2C_CONTROL);
	ctrl |= DMECI2C_CTRL_IEN;
	dmec_i2c_setreg(i2c, DMECI2C_CONTROL, ctrl);

	return 0;
}

static u32 dmec_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm dmec_algorithm = {
	.master_xfer = dmec_i2c_xfer,
	.functionality = dmec_i2c_func,
};

static struct i2c_adapter dmec_adapter = {
	.owner = THIS_MODULE,
	.name = "i2c-dmec",
	.class = I2C_CLASS_DEPRECATED,
	.algo = &dmec_algorithm,
};

#define dmec_i2c_of_probe(pdev, i2c) -ENODEV

static int dmec_i2c_probe(struct platform_device *pdev)
{
	struct dmec_i2c *i2c;
	struct dmec_i2c_platform_data *pdata;
	int ret;

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->dev = &pdev->dev;
	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		i2c->ip_clock_khz = pdata->clock_khz;
		i2c->bus_clock_khz = DMEC_I2C_BUS_CLK;
	}
	i2c->regmap = dmec_get_regmap(pdev->dev.parent);

	i2c->irq = platform_get_irq(pdev, 0);

	ret = dmec_i2c_init(&pdev->dev, i2c);
	if (ret)
		return ret;

	/* hook up driver to tree */
	platform_set_drvdata(pdev, i2c);
	i2c->adap = dmec_adapter;
	i2c_set_adapdata(&i2c->adap, i2c);
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = pdev->dev.of_node;

	if (dmec_i2c_irq_enable(i2c)) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		return ret;
	}

	/* add i2c adapter to i2c tree */
	ret = i2c_add_adapter(&i2c->adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add adapter\n");
		return ret;
	}

	ret = dmec_i2c_dmx_add(i2c);

	return ret;
}

static int dmec_i2c_remove(struct platform_device *pdev)
{
	struct dmec_i2c *i2c = platform_get_drvdata(pdev);

	/* disable i2c logic */
	dmec_i2c_setreg(i2c, DMECI2C_CONTROL,
			dmec_i2c_getreg(i2c, DMECI2C_CONTROL)
			& ~(DMECI2C_CTRL_EN | DMECI2C_CTRL_IEN));

	/* remove adapter & data */
	dmec_i2c_dmx_del(i2c);
	i2c_del_adapter(&i2c->adap);

	if (!IS_ERR(i2c->clk))
		clk_disable_unprepare(i2c->clk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dmec_i2c_suspend(struct device *dev)
{
	struct dmec_i2c *i2c = dev_get_drvdata(dev);

	/* make sure the device is disabled */
	dmec_i2c_setreg(i2c, DMECI2C_CONTROL,
			dmec_i2c_getreg(i2c, DMECI2C_CONTROL)
			& ~(DMECI2C_CTRL_EN | DMECI2C_CTRL_IEN));

	if (!IS_ERR(i2c->clk))
		clk_disable_unprepare(i2c->clk);
	return 0;
}

static int dmec_i2c_resume(struct device *dev)
{
	struct dmec_i2c *i2c = dev_get_drvdata(dev);
	int ret;

	if (!IS_ERR(i2c->clk)) {
		unsigned long rate;

		ret = clk_prepare_enable(i2c->clk);

		if (ret) {
			dev_err(dev,
				"clk_prepare_enable failed: %d\n", ret);
			return ret;
		}
		rate = clk_get_rate(i2c->clk) / 1000;
		if (rate)
			i2c->ip_clock_khz = rate;
	}


	ret = dmec_i2c_init(dev, i2c);
	if (ret)
		return ret;

	dmec_i2c_setreg(i2c, DMECI2C_CONTROL,
			dmec_i2c_getreg(i2c, DMECI2C_CONTROL)
			| DMECI2C_CTRL_IEN);

	return 0;
}

static SIMPLE_DEV_PM_OPS(dmec_i2c_pm, dmec_i2c_suspend, dmec_i2c_resume);
#define DMEC_I2C_PM	(&dmec_i2c_pm)
#else
#define DMEC_I2C_PM	NULL
#endif

static struct platform_driver dmec_i2c_driver = {
	.probe   = dmec_i2c_probe,
	.remove  = dmec_i2c_remove,
	.driver  = {
		.name = "dmec-i2c",
		.pm = DMEC_I2C_PM,
	},
};

module_platform_driver(dmec_i2c_driver);

MODULE_AUTHOR("Peter Korsgaard <jacmet@sunsite.dk>");
MODULE_AUTHOR("Zahari Doychev <zahari.doychev@linux.com>");
MODULE_DESCRIPTION("DMO OpenCores based I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-i2c");
