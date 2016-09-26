/*
 * Watchdog driver for Data Modul AG Embedded Controller
 *
 * Copyright (C) 2016 Zahari Doychev, Data Modul AG
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/bitops.h>

#include "dmec.h"

#define DMEC_WDT_VER		0x30
#define DMEC_WDT_SRV		0x30
#define DMEC_WDT_CFG		0x31
#define DMEC_WDT_S0CFG		0x32
#define DMEC_WDT_S0MOD0		0x33
#define DMEC_WDT_S0MOD1		0x34
#define DMEC_WDT_S1CFG		(DMEC_WDT_S0CFG + 3)
#define DMEC_WDT_S1MOD0		(DMEC_WDT_S0MOD0 + 3)
#define DMEC_WDT_S1MOD1		(DMEC_WDT_S0MOD1 + 3)
#define DMEC_WDT_S2CFG		(DMEC_WDT_S1CFG + 3)
#define DMEC_WDT_S2MOD0		(DMEC_WDT_S1MOD0 + 3)
#define DMEC_WDT_S2MOD1		(DMEC_WDT_S1MOD1 + 3)

#define DMEC_WDT_EN		BIT(0)
#define DMEC_WDT_LOCK		BIT(1)
#define DMEC_WDT_WIN_MODE	BIT(2)
#define DMEC_WDT_AL		BIT(3)

#define DMEC_WDT_PRESCALER	BIT(4)
#define DMEC_WDT_WDTEN		BIT(3)
#define DMEC_WDT_WDSTS		BIT(5)

#define DMEC_WDT_TIMEOUT_MIN	1 /* s */
#define DMEC_WDT_TIMEOUT_MAX	(2 * 3600) /* s */

#define DMEC_WDT_TIME_MAX	(65 * 1000)

/* S0 used only during boot */
#define DEFAULT_S0_TIMEOUT	0
#define DEFAULT_S1_TIMEOUT	3
#define DEFAULT_S2_TIMEOUT	5

enum wdt_actions {
	WDT_DISABLE = 0,
	WDT_DELAY,
	WDT_RESET,
	WDT_SYSIRQ0,
	WDT_SYSIRQ1,
	WDT_SYSIRQ2,
	WDT_IRQ,
	WDT_RESERVED
};

enum wdt_stages {
	S0,
	S1,
	S2
};

static unsigned int s1_timeout = DEFAULT_S1_TIMEOUT;
module_param(s1_timeout, uint, 0);
MODULE_PARM_DESC(s1_timeout,
		 "Watchdog stage 1 timeout in [s], default=3");

static unsigned int s2_timeout = DEFAULT_S2_TIMEOUT;
module_param(s2_timeout, uint, 0);
MODULE_PARM_DESC(s2_timeout,
		 "Watchdog stage 2 timeout in [s], default=5");

static enum wdt_actions action = WDT_DELAY;
module_param(action, uint, 0);
MODULE_PARM_DESC(action,
		 "Watchdog action for stage 1, default=1 (delay)");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static bool win_mode;
module_param(win_mode, bool, 0);
MODULE_PARM_DESC(win_mode, "Use watchdog window mode, default=0");

static bool stop_on_reboot;
module_param(stop_on_reboot, bool, 0);
MODULE_PARM_DESC(stop_on_reboot, "stop watchdog on system reboot, default=0");

struct dmec_wdt_data {
	struct watchdog_device	wdd;
	struct regmap *regmap;
	unsigned int s1_time;
	unsigned int s2_time;
	unsigned int status;
	unsigned int boot_cfg;
	bool boot_mode;
};

static int dmec_wdt_set_stage_action(struct dmec_wdt_data *wdat,
				     unsigned int stage,
				     enum wdt_actions action)
{
	unsigned int offset = DMEC_WDT_S0CFG + (3 * stage);
	unsigned int val;

	regmap_read(wdat->regmap, offset, &val);
	val &= ~DMEC_WDT_WDSTS;
	val |= action | DMEC_WDT_WDTEN;
	regmap_write(wdat->regmap, offset, val);

	return 0;
}

static int dmec_wdt_clear_action(struct dmec_wdt_data *wdat, int stage)
{
	unsigned int val;

	regmap_read(wdat->regmap, DMEC_WDT_S0CFG + 3 * stage, &val);
	val |= DMEC_WDT_WDSTS;
	val &= ~0xf;
	regmap_write(wdat->regmap, DMEC_WDT_S0CFG + 3 * stage, val);

	return 0;
}

static unsigned int dmec_wdt_get_stage_timeout(struct dmec_wdt_data *wdat,
					       unsigned int stage)
{
	unsigned int val, timeout = 0, cfg;

	regmap_read(wdat->regmap, DMEC_WDT_S0CFG + 3 * stage, &cfg);
	regmap_read(wdat->regmap, DMEC_WDT_S0MOD0 + 3 * stage, &val);
	timeout = val;
	regmap_read(wdat->regmap, DMEC_WDT_S0MOD1 + 3 * stage, &val);
	timeout |= (val << 8);

	if (cfg & DMEC_WDT_PRESCALER)
		timeout <<= 7;

	return timeout / 1000;
}

static int dmec_wdt_set_stage_timeout(struct dmec_wdt_data *wdat,
				      unsigned int stage,
				      unsigned int timeout)
{
	unsigned int val;

	timeout *= 1000;
	if (timeout > DMEC_WDT_TIME_MAX) {
		/* enable prescaler */
		regmap_read(wdat->regmap, DMEC_WDT_S0CFG + (3 * stage), &val);
		val |= DMEC_WDT_PRESCALER;
		regmap_write(wdat->regmap, DMEC_WDT_S0CFG + (3 * stage), val);
		timeout >>= 7;
	} else {
		regmap_read(wdat->regmap, DMEC_WDT_S0CFG + (3 * stage), &val);
		val &= ~DMEC_WDT_PRESCALER;
		regmap_write(wdat->regmap, DMEC_WDT_S0CFG + (3 * stage), val);
	}

	val = timeout & 0xff;
	regmap_write(wdat->regmap, DMEC_WDT_S0MOD0 + (3 * stage), val);
	val = (timeout >> 8) & 0xff;
	regmap_write(wdat->regmap, DMEC_WDT_S0MOD1 + (3 * stage), val);

	return 0;
}

static int dmec_wdt_set_timeouts(struct dmec_wdt_data *wdat)
{
	dmec_wdt_clear_action(wdat, S0);
	dmec_wdt_clear_action(wdat, S1);
	dmec_wdt_clear_action(wdat, S2);

	wdat->s1_time = s1_timeout;
	wdat->s2_time = s2_timeout;
	dmec_wdt_set_stage_timeout(wdat, S1, s1_timeout);
	dmec_wdt_set_stage_timeout(wdat, S2, s2_timeout);
	dmec_wdt_set_stage_action(wdat, S2, WDT_RESET);

	return 0;
}

static int dmec_wdt_stop(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	regmap_update_bits(wdat->regmap, DMEC_WDT_CFG, DMEC_WDT_EN, 0);

	return 0;
}

static int dmec_wdt_start(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	regmap_update_bits(wdat->regmap, DMEC_WDT_CFG,
			   DMEC_WDT_EN, DMEC_WDT_EN);

	wdat->boot_mode = false;

	return 0;
}

static int dmec_wdt_ping(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);
	unsigned int val;

	/* We should try avoiding resets in window mode until the watchdog
	 * daemon takes control
	 */
	if (win_mode && wdat->boot_mode) {
		regmap_read(wdat->regmap, DMEC_WDT_S1CFG, &val);
		if (!(val & DMEC_WDT_WDSTS))
			return 0;
		val |= DMEC_WDT_WDSTS;
		regmap_write(wdat->regmap, DMEC_WDT_S1CFG, val);
	}

	regmap_write(wdat->regmap, DMEC_WDT_SRV, 0xff);

	return 0;
}

static int dmec_wdt_set_win_mode(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	regmap_update_bits(wdat->regmap, DMEC_WDT_CFG,
			   DMEC_WDT_WIN_MODE, DMEC_WDT_WIN_MODE);

	return 0;
}

static int dmec_wdt_set_std_mode(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	regmap_update_bits(wdat->regmap, DMEC_WDT_CFG, DMEC_WDT_WIN_MODE, 0);

	return 0;
}

static int dmec_wdt_set_timeout(struct watchdog_device *wdd,
				unsigned int timeout)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	wdd->timeout = timeout;
	wdat->boot_mode = false;
	dmec_wdt_stop(wdd);
	dmec_wdt_clear_action(wdat, S0);

	if (win_mode) {
		dmec_wdt_clear_action(wdat, S1);
		dmec_wdt_set_stage_timeout(wdat, S1, wdat->s1_time);
		dmec_wdt_set_stage_action(wdat, S1, WDT_DELAY);
		dmec_wdt_set_win_mode(wdd);
	} else {
		dmec_wdt_set_std_mode(wdd);
	}

	wdat->s2_time = timeout;
	dmec_wdt_clear_action(wdat, S2);
	dmec_wdt_set_stage_timeout(wdat, S2, timeout);
	dmec_wdt_set_stage_action(wdat, S2, WDT_RESET);
	dmec_wdt_start(wdd);

	return 0;
}

static long dmec_wdt_ioctl(struct watchdog_device *wdd, unsigned int cmd,
			   unsigned long arg)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);
	void __user *argp = (void __user *)arg;
	int ret = -ENOIOCTLCMD;
	int __user *p = argp;
	int val;

	switch (cmd) {
	case WDIOC_SETPRETIMEOUT:
		if (get_user(val, p) && val < 0)
			return -EFAULT;
		dmec_wdt_clear_action(wdat, S1);
		if (val > 0) {
			dmec_wdt_set_stage_timeout(wdat, S1, val);
			dmec_wdt_set_stage_action(wdat, S1, action);
		}
		wdat->s1_time = val;
		wdat->boot_mode = false;
		ret = 0;
		if (!win_mode)
			ret = dmec_wdt_ping(wdd);
		break;
	case WDIOC_GETPRETIMEOUT:
		ret = put_user(wdat->s1_time, (int __user *)arg);
		break;
	}

	return ret;
}

static unsigned int dmec_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);

	return dmec_wdt_get_stage_timeout(wdat, 2);
}

static unsigned int dmec_wdt_status(struct watchdog_device *wdd)
{
	struct dmec_wdt_data *wdat = watchdog_get_drvdata(wdd);
	unsigned int status;

	regmap_read(wdat->regmap, DMEC_WDT_CFG, &status);

	return status;
}

static struct watchdog_info dmec_wdt_info = {
	.options		= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
				  WDIOF_MAGICCLOSE | WDIOF_PRETIMEOUT,
	.identity		= "DMEC WDT",
};

static const struct watchdog_ops dmec_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= dmec_wdt_start,
	.stop		= dmec_wdt_stop,
	.ping		= dmec_wdt_ping,
	.set_timeout	= dmec_wdt_set_timeout,
	.status		= dmec_wdt_status,
	.ioctl		= dmec_wdt_ioctl,
	.get_timeleft	= dmec_wdt_get_timeleft,
};

static int dmec_wdt_get_hw_ping_time(struct dmec_wdt_data *wdat)
{
	struct watchdog_device *wdd = &wdat->wdd;
	unsigned int val, t;

	regmap_read(wdat->regmap, DMEC_WDT_S0CFG, &val);
	t = dmec_wdt_get_stage_timeout(wdat, S0);
	if (t && (val & 0x7) > 1)
		dmec_wdt_clear_action(wdat, S0);

	wdat->s1_time = dmec_wdt_get_stage_timeout(wdat, S1);
	wdat->s2_time = dmec_wdt_get_stage_timeout(wdat, S2);

	regmap_read(wdat->regmap, DMEC_WDT_S1CFG, &val);
	if (wdat->s1_time && (val & 0x7) > 1 && !(val & DMEC_WDT_WDSTS)) {
		wdd->timeout = wdat->s1_time;
		return 1;
	}

	regmap_read(wdat->regmap, DMEC_WDT_S2CFG, &val);
	if (wdat->s2_time && (val & 0x7) > 1 && !(val & DMEC_WDT_WDSTS)) {
		wdd->timeout = wdat->s2_time;
		return 2;
	}

	return -1;
}

static int dmec_wdt_config_win_mode(struct dmec_wdt_data *wdat)
{
	struct watchdog_device *wdd = &wdat->wdd;
	unsigned int t;

	/* The timeout should be ok until the watchdog daemon appears */
	t = (min_t(unsigned int, wdat->s1_time, wdat->s2_time) / 2);
	wdd->timeout = t;

	return 0;
}

static int dmec_wdt_config_mode(struct dmec_wdt_data *wdat)
{
	if (!(wdat->boot_cfg & DMEC_WDT_EN))
		return -1;

	if (dmec_wdt_get_hw_ping_time(wdat) < 0)
		return -1;

	if (wdat->boot_cfg & DMEC_WDT_WIN_MODE)
		dmec_wdt_config_win_mode(wdat);

	wdat->boot_mode = true;

	return 0;
}

static int dmec_wdt_setup(struct dmec_wdt_data *wdat)
{
	struct watchdog_device *wdd = &wdat->wdd;
	int ret = 0;

	regmap_read(wdat->regmap, DMEC_WDT_CFG, &wdat->boot_cfg);

	ret = dmec_wdt_config_mode(wdat);

	if (ret < 0)
		dmec_wdt_set_timeouts(wdat);

	if (wdat->boot_cfg & DMEC_WDT_LOCK && !nowayout) {
		dev_info(wdd->parent, "watchdog lock is enabled.\n");
		nowayout = true;
		return 0;
	}

	return 0;
}

static int dmec_wdt_probe(struct platform_device *pdev)
{
	struct dmec_wdt_data *wdat;
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	int ret = 0;

	wdat = devm_kzalloc(dev, sizeof(*wdat), GFP_KERNEL);
	if (!wdat)
		return -ENOMEM;

	wdat->regmap = dmec_get_regmap(pdev->dev.parent);
	wdd = &wdat->wdd;
	wdd->parent = dev;

	wdd->info = &dmec_wdt_info;
	wdd->ops = &dmec_wdt_ops;
	wdd->min_timeout = DMEC_WDT_TIMEOUT_MIN;
	wdd->max_timeout = DMEC_WDT_TIMEOUT_MAX;

	wdat->s1_time = s1_timeout;
	wdat->s2_time = s2_timeout;

	watchdog_set_drvdata(wdd, wdat);
	platform_set_drvdata(pdev, wdat);

	dmec_wdt_setup(wdat);

	watchdog_set_nowayout(wdd, nowayout);

	ret = watchdog_register_device(wdd);
	if (ret)
		return ret;

	regmap_read(wdat->regmap, DMEC_WDT_VER,
		    &dmec_wdt_info.firmware_version);

	dev_info(dev, "registered. v%u.%u sta: %#lx mode:%d\n",
		 (dmec_wdt_info.firmware_version >> 4) & 0xf,
		 dmec_wdt_info.firmware_version & 0xf,
		 wdd->status, win_mode);

	return 0;
}

static int dmec_wdt_remove(struct platform_device *pdev)
{
	struct dmec_wdt_data *wdat = platform_get_drvdata(pdev);

	dmec_wdt_stop(&wdat->wdd);
	watchdog_unregister_device(&wdat->wdd);

	return 0;
}

static void dmec_wdt_shutdown(struct platform_device *pdev)
{
	struct dmec_wdt_data *wdat = platform_get_drvdata(pdev);

	dmec_wdt_stop(&wdat->wdd);
}

#ifdef CONFIG_PM
/* Disable watchdog if it is active during suspend */
static int dmec_wdt_suspend(struct platform_device *pdev,
			    pm_message_t message)
{
	struct dmec_wdt_data *wdat = platform_get_drvdata(pdev);
	struct watchdog_device *wdd = &wdat->wdd;

	regmap_read(wdat->regmap, DMEC_WDT_CFG, &wdat->status);

	if (wdat->status & DMEC_WDT_EN)
		return dmec_wdt_stop(wdd);

	return 0;
}

/* Enable watchdog and configure it if necessary */
static int dmec_wdt_resume(struct platform_device *pdev)
{
	struct dmec_wdt_data *wdat = platform_get_drvdata(pdev);
	struct watchdog_device *wdd = &wdat->wdd;

	if (!win_mode && wdat->status & DMEC_WDT_EN) {
		dmec_wdt_stop(wdd);
		dmec_wdt_clear_action(wdat, S0);
		dmec_wdt_clear_action(wdat, S1);
		dmec_wdt_clear_action(wdat, S2);

		if (wdat->s1_time) {
			dmec_wdt_set_stage_timeout(wdat, S1, wdat->s1_time);
			dmec_wdt_set_stage_action(wdat, S1, WDT_DELAY);
		}
		dmec_wdt_set_stage_timeout(wdat, S2, wdat->s2_time);
		dmec_wdt_set_stage_action(wdat, S2, WDT_RESET);
		dmec_wdt_get_timeleft(wdd);
		return dmec_wdt_start(wdd);
	}

	return dmec_wdt_stop(wdd);
}
#else
#define dmec_wdt_suspend NULL
#define dmec_wdt_resume	NULL
#endif

static struct platform_driver dmec_wdt_driver = {
	.driver = {
		.name = "dmec-wdt",
		.owner = THIS_MODULE,
	},
	.probe = dmec_wdt_probe,
	.remove = dmec_wdt_remove,
	.shutdown = dmec_wdt_shutdown,
	.suspend = dmec_wdt_suspend,
	.resume = dmec_wdt_resume,
};

module_platform_driver(dmec_wdt_driver);

MODULE_DESCRIPTION("dmec watchdog driver");
MODULE_AUTHOR("Zahari Doychev <zahari.doychev@linux.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-wdt");
