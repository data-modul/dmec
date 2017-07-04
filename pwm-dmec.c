/*
 * Pulse Width Modulation(PWM) for Data Modul AG Embedded Controller
 *
 * Copyright (C) 2017 Data Modul AG
 *
 * Author: Reyhaneh Yazdani <reyhane.y84@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>

#include "dmec.h"


#define   PWMREV              0x080       /* PWM version/revision register */
#define   PWMA_BASE           0x081       /* PWM controller A register base */
#define   PWMB_BASE           0x085       /* PWM controller B register base */
#define   PWMCFG              0x000       /* PWM configuration register */
  #define   PWMEN             (1 << 7)    /* PWM enable */
  #define   PWMBUSY           (1 << 6)    /* PWM busy flag */
  #define   PWM16             (1 << 5)    /* PWM 16Bit mode enable, only available in PWM Channel A */
  #define   PWMCA             (1 << 4)    /* PWM alignment, 0=left, 1=center */
  #define   PWMPOL            (1 << 3)    /* PWM polarity, 0=low, 1=high*/
  #define   PWMCLK_BITS       0           /* PWM base clock bits offset */
  #define   PWMCLK_MASK       7           /* PWM base clock mask*/
#define   PWMSCL              0x001       /* PWM channel pre-scaler register */
#define   PWMPER              0x002       /* PWM period register */
#define   PWMDTY              0x003       /* PWM duty cycle register */

#define PWM_SRC_CLK           50000000    /* default input clock is 50MHz */
#define PWM_BASE_CLK(a)       ((UINT64)(PWM_BASE_CLK / (1 << a)))

#define DMEC_PWM_EN_ON        (1 << 7)
#define DMEC_PWM_EN_OFF       0
#define DMEC_PWM_PWMDTYA_OFFSET 0x84

#define   DMEC_NUM_PWMS     2

#define DMEC_PWM_PAR1			0x071      /* Pin assignment register 1 */
#define DMEC_PWM_CHANNEL0_GPIO_OFF	0          /* GPIO Port A pin 4 assignment */
#define DMEC_PWM_CHANNEL1_GPIO_OFF	2          /* GPIO Port A pin 5 assignment */
#define PAX_MASK			0x03       /* Pin assignment mask */
#define DMEC_PWM_GPIO_FCNT		2

static bool GpioConfigured_0 = 0, GpioConfigured_1 = 0;
static struct regmap *generalRegmap;
struct dmec_pwm_chip {
	struct pwm_chip chip;
	struct device *dev;
	struct mutex mutex;
	struct regmap *regmap;
	unsigned int mode;
	unsigned int alignmentA;
	unsigned int alignmentB;
	unsigned int clkA;
	unsigned int clkB;
	unsigned int scalerA;
	unsigned int scalerB;
};

static inline struct dmec_pwm_chip *to_dmec(struct pwm_chip *chip)
{
	return container_of(chip, struct dmec_pwm_chip, chip);
}

/*static int dmec_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
					enum pwm_polarity polarity)
{
	struct dmec_pwm_device *dmecPwm = pwm_get_chip_data(pwm);
	printk("inside set polarity\n");
}
*/
/*static int dmec_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
				int duty_ns, int period_ns)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	unsigned int prd, dty;
	unsigned long div;
	unsigned int pres = 0;
	unsigned int val = 0;
	int ret = 0;

printk("config------------1   %d    %d\n", period_ns, duty_ns);

	if (pwm_is_enabled(pwm) && (period_ns != pwm_get_period(pwm)))
	{
		dev_err(chip->dev, "cannot change PWM period while enabled\n");
		return -EBUSY;
	}

	if (pwm->hwpwm == 0) 
		currentBaseReg = PWMA_BASE;
	else
		currentBaseReg = PWMB_BASE;
*/
	/*
	 * write period if the channel is disabled.
	 */
/*	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
	if (ret < 0)
	{
		printk("config****2\n");
		dev_err(chip->dev, "%s: Cannot read PWM enable status\n", pwm->label);
		printk("config****3\n");
		return -EPERM;
	}

	if(val & PWMEN)
	{
		dev_err(chip->dev, "%s: Cannot change PWM period while enabled\n", pwm->label);
		return -EBUSY;
	}
	else
	{
		mutex_lock(&dmecPwm->mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMPER, period_ns);
		mutex_unlock(&dmecPwm->mutex);
	}
*/
	/*
	 * write duty if the it is not greater than period.
	 */
/*	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMPER, &val);
	if(val >= duty_ns)
	{
		dev_err(chip->dev, "%s: Cannot change PWM duty while greater that period\n", pwm->label);
		return -EPERM;
	}
	else
	{
		mutex_lock(&dmecPwm->mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMDTY, duty_ns);
		mutex_unlock(&dmecPwm->mutex);
	}

	return ret;
}
*/
/*static int dmec_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	int ret = 0;
	unsigned int val;
	unsigned int currentBaseReg;

printk("enable------------1\n");
	if (pwm->hwpwm == 1)
		currentBaseReg = PWMB_BASE;
	else if (pwm->hwpwm == 0 && GpioConfigured_0) 
		currentBaseReg = PWMA_BASE;
	else 
		currentBaseReg = PWMB_BASE;

	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWMEN\n", pwm->label);
		return -EPERM;
	}

	val &= ~PWMEN;
	val |= DMEC_PWM_EN_ON;

	mutex_lock(&dmecPwm->mutex); 
	ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
	if (ret < 0)
		dev_err(chip->dev, "%s: Failed to enable PWM\n", pwm->label);
	mutex_unlock(&dmecPwm->mutex);

	return ret;
}

static void dmec_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	int ret = 0;
	unsigned int val;
	unsigned int currentBaseReg;
printk("disable------------1\n");

	if (pwm->hwpwm == 1) 
		currentBaseReg = PWMB_BASE;
	else if (pwm->hwpwm == 0 && GpioConfigured_0) 
		currentBaseReg = PWMA_BASE;
	else 
		currentBaseReg = PWMB_BASE;

	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWMEN\n", pwm->label);
		return;
	}

	val &= ~PWMEN;
	val |= DMEC_PWM_EN_OFF;

	mutex_lock(&dmecPwm->mutex);
	ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
	if (ret < 0)
		dev_err(chip->dev, "%s: Failed to enable PWM\n", pwm->label);
	mutex_unlock(&dmecPwm->mutex);

	return;
}*/

static void dmec_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	int ret = 0;
	unsigned int val;
	unsigned int currentBaseReg;

printk("get state------------1\n");
	if (pwm->hwpwm == 1) /* channel 1 */
		currentBaseReg = PWMB_BASE;
	else if (pwm->hwpwm == 0 && GpioConfigured_0) /* channel 0*/
		currentBaseReg = PWMA_BASE;
	else /*channel 1*/
		currentBaseReg = PWMB_BASE;

	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM config\n", pwm->label);
		return;
	}

	/* mode : 1= 16bit, 0=8bit*/
	if(currentBaseReg == PWMA_BASE) /*mode bit is available only in ch1*/
		dmecPwm->mode = (val & PWM16) ? 1 : 0;
	else
		dmecPwm->mode = 0;

	/* enable state*/
	state->enabled = (val & PWMEN) ? true : false;

	/* alignment */
	if(currentBaseReg == PWMA_BASE) /*alignment ch1*/
		dmecPwm->alignmentA = (val & PWMCA) ? 1 : 0;
	else
		dmecPwm->alignmentB = (val & PWMCA) ? 1 : 0;

	/* polarity state*/
	state->polarity = (val & PWMPOL) ? PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;

	/* clk */
	if(currentBaseReg == PWMA_BASE) /*clk ch1*/
		dmecPwm->clkA = val & PWMCLK_MASK;
	else
		dmecPwm->clkB = val & PWMCLK_MASK;

	/* scaler register */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMSCL, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM scaler\n", pwm->label);
		return;
	}
	if(currentBaseReg == PWMA_BASE) /*scaler ch1*/
		dmecPwm->scalerA = val;
	else
		dmecPwm->scalerB = val;

	/* period state */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMPER, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM period\n", pwm->label);
		return;
	}
	state->period = val;

	/* duty state */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMDTY, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM duty-cycle\n", pwm->label);
		return;
	}
	state->duty_cycle = val;

	/* 16bit only supported witch channel 0*/
	if(dmecPwm->mode)
	{
		/* period state */
		ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMPER, &val);
		state->period |= val << 8;
		
		/* duty state */
		ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMDTY, &val);
		state->duty_cycle |= val << 8;
	}
}

static int dmec_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm, 
				struct pwm_state *state)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	struct pwm_state cstate;
	unsigned int val;
	int ret = 0;
	unsigned int currentBaseReg;

	printk("inside apply***\n");

	pwm_get_state(pwm, &cstate);

	if (pwm->hwpwm == 1) /* channel 1 */
		currentBaseReg = PWMB_BASE;
	else if (pwm->hwpwm == 0 && GpioConfigured_0) /* channel 0*/
		currentBaseReg = PWMA_BASE;
	else /*channel 1*/
		currentBaseReg = PWMB_BASE;

	/* check enable request*/
	if(cstate.enabled != state->enabled)
	{
		ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
		if (ret < 0)
		{
			dev_err(chip->dev, "%s: Failed to read PWMEN\n", 
					pwm->label);
			return -EPERM;
		}

		val &= ~PWMEN;
		if(state->enabled)
			val |= DMEC_PWM_EN_ON;
		else
			val |= DMEC_PWM_EN_OFF;

		mutex_lock(&dmecPwm->mutex); 
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
		if (ret < 0)
			dev_err(chip->dev, "%s: Failed to enablei/disable PWM\n", 
					pwm->label);
		mutex_unlock(&dmecPwm->mutex);

//		if(state->enabled)
//			pwm_enable(pwm);
//		else
//			pwm_disable(pwm);
	}

	/* check period request */
	if(cstate.period != state->period)
	{
		if (cstate.enabled) /*channel is enable*/
		{
			dev_err(chip->dev, "%s: Cannot change PWM period while enabled\n", 
					pwm->label);
			return -EBUSY;
		}
		mutex_lock(&dmecPwm->mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMPER, 
				(uint8_t)state->period & 0xFF);
		if(dmecPwm->mode) /*16bit*/
			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMPER, 
					(uint8_t)((state->period >> 8) & 0xFF));
		mutex_unlock(&dmecPwm->mutex);
		
		pwm_set_period(pwm, state->period);
	}

	/* check duty-cycle */
	if(cstate.duty_cycle != state->duty_cycle)
	{
		mutex_lock(&dmecPwm->mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMDTY, 
				(uint8_t)(state->duty_cycle & 0xFF));
		if(dmecPwm->mode) /* 16bit*/
			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMDTY, 
					(uint8_t)((state->duty_cycle >> 8) & 0xFF));
		mutex_unlock(&dmecPwm->mutex);
		
		pwm_set_duty_cycle(pwm, state->duty_cycle);
	}

	/* check polarity */
	if(cstate.polarity != state->polarity)
	{
		if (cstate.enabled) /*channel is enable*/
		{
			dev_err(chip->dev, "%s: Cannot change PWM polarity while enabled\n", pwm->label);
			return -EBUSY;
		}
		ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
		if (ret < 0)
		{
			dev_err(chip->dev, "%s: Failed to read PWMConfig\n", pwm->label);
			return -EPERM;
		}
		val &= ~PWMPOL;
		if(state->polarity == PWM_POLARITY_NORMAL)
			val |= DMEC_PWM_EN_ON;
		else
			val |= DMEC_PWM_EN_OFF;

		mutex_lock(&dmecPwm->mutex); 
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
		if (ret < 0)
			dev_err(chip->dev, "%s: Failed to enablei/disable PWM\n", 
					pwm->label);
		mutex_unlock(&dmecPwm->mutex);

		ret = pwm_set_polarity(pwm, state->polarity);
	}
	return ret;
}

//////////////////////////////////////////////////////////////////////

static ssize_t dmec_pwm_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;
	regmap_read(generalRegmap, PWMREV, &val);

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n", (val >> 4) & 0xf, val & 0xf);
}

static ssize_t dmec_pwm_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;

	regmap_read(generalRegmap, PWMA_BASE + PWMCFG, &val);
	return scnprintf(buf, PAGE_SIZE, "%u\n", val & PWM16);
}
/*
static ssize_t dmec_pwm_chA_alignment_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0 ;

	val = dmec_pwm_get_config(DMEC_PWM_PWMCFGA_OFFSET, DMEC_PWM_CA, dev);
	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t dmec_pwm_pwm16_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	long ret = -EPERM;

	ret = dmec_pwm_set_config(DMEC_PWM_PWMCFGA_OFFSET, DMEC_PWM_16, dev, buf, count);

	return ret;
}

static ssize_t dmec_pwm_chA_alignment_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	long ret = -EPERM;

	ret = dmec_pwm_set_config(DMEC_PWM_PWMCFGA_OFFSET, DMEC_PWM_CA, dev, buf, count);

	return ret;
}

static ssize_t dmec_pwm_chA_scaler_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;

	regmap_read(regmap, DMEC_PWM_PWMSCLA_OFFSET, &val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}

static long dmec_pwm_chA_scaler_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	long val = 0, ret = -EPERM;

	mutex_lock(&pwm_lock);
	if (!kstrtol(buf, 10, &val)) {
		regmap_write(regmap, DMEC_PWM_PWMSCLA_OFFSET, val);
		ret = count;
	}
	mutex_unlock(&pwm_lock);
	return ret;
}
*/

//static DEVICE_ATTR(version, S_IRUGO,dmec_pwm_version_show, NULL );

//static struct attribute *pwm_attribute[]= {
//	&dev_attr_version.attr,
//	NULL
//};

//static const struct attribute_group pwm_group = {
//	.attrs = pwm_attribute,
//};

static const struct pwm_ops dmec_pwm_ops = {
	.get_state = dmec_pwm_get_state,
	.apply = dmec_pwm_apply,
	.owner = THIS_MODULE,
};

static int dmec_pwm_probe(struct platform_device *pdev)
{
	struct dmec_pwm_chip *dmecPwm;
	unsigned int val;
	int ret;

	dmecPwm = devm_kzalloc(&pdev->dev, sizeof(*dmecPwm), GFP_KERNEL);
	if (!dmecPwm){
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	dmecPwm->regmap = dmec_get_regmap(pdev->dev.parent);
	generalRegmap = dmec_get_regmap(pdev->dev.parent);

	/* check pwm is set in corresponding GPIOA-4 and GPIOA-5 pin in BIOS */
	ret = regmap_read(dmecPwm->regmap, DMEC_PWM_PAR1, &val);
	if(((val >> DMEC_PWM_CHANNEL0_GPIO_OFF) & PAX_MASK) == DMEC_PWM_GPIO_FCNT)
		GpioConfigured_0 = 1;
	if (((val >> DMEC_PWM_CHANNEL1_GPIO_OFF) & PAX_MASK) == DMEC_PWM_GPIO_FCNT)
		GpioConfigured_1 = 1;

	if ((GpioConfigured_0 == 0) && (GpioConfigured_1 == 0))
	{
		dev_err(&pdev->dev, "PWM is not configured in BIOS.\n");
		devm_kfree(&pdev->dev, dmecPwm);
		return -ENODEV;
	}

	dmecPwm->chip.ops = &dmec_pwm_ops;

	dmecPwm->chip.npwm = 0;
	if (GpioConfigured_0 == 1)
		dmecPwm->chip.npwm += 1;
	if (GpioConfigured_1 == 1)
		dmecPwm->chip.npwm += 1;

	dmecPwm->chip.dev = &pdev->dev;
	dmecPwm->chip.base = -1;

	mutex_init(&dmecPwm->mutex);

	ret = pwmchip_add(&dmecPwm->chip);
	if (ret < 0)
	{
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		devm_kfree(&pdev->dev, dmecPwm);
		return ret;
	}

	platform_set_drvdata(pdev, dmecPwm);
//	ret = sysfs_create_group(&pdev->dev.kobj, &pwm_group);
	return 0;
}

static int dmec_pwm_remove(struct platform_device *pdev)
{
	struct dmec_pwm_chip *dmecPwm = platform_get_drvdata(pdev);
	return pwmchip_remove(&dmecPwm->chip);
}

static const struct of_device_id dmecPwm_of_match[] = {
	{ .compatible = "dmec,dmec-pwm" },
	{},
};
MODULE_DEVICE_TABLE(of, dmecPwm_of_match);

static struct platform_driver dmec_pwm_driver = {
	.driver = {
			.name = "dmec-pwm",
			.of_match_table = dmecPwm_of_match,
			.owner = THIS_MODULE,
	},
	.probe = dmec_pwm_probe,
	.remove = dmec_pwm_remove,
};

module_platform_driver(dmec_pwm_driver);

MODULE_DESCRIPTION("dmec pwm driver");
MODULE_AUTHOR("Reyhaneh Yazdani <reyhane.y84@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-pwm");
