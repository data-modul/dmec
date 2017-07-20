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

#define DMEC_PWM_EN_ON        (1 << 7)
#define DMEC_PWM_EN_OFF       0
#define DMEC_PWM_16_BIT      (1 << 5)
#define DMEC_PWM_8_BIT        0
#define DMEC_PWM_CA_CENTER (1 << 4)
#define DMEC_PWM_CA_LEFT      0

#define DMEC_PWM_PAR1			0x071      /* Pin assignment register 1 */
#define DMEC_PWM_CHANNEL0_GPIO_OFF	0          /* GPIO Port A pin 4 assignment */
#define DMEC_PWM_CHANNEL1_GPIO_OFF	2          /* GPIO Port A pin 5 assignment */
#define PAX_MASK			0x03       /* Pin assignment mask */
#define DMEC_PWM_GPIO_FCNT		2

static bool GpioConfigured_0 = 0, GpioConfigured_1 = 0;

struct dmec_pwm_chip {
	struct pwm_chip chip;
	struct regmap *regmap;
};

static struct mutex mutex;

typedef struct _dmec_pwm_channel {
	struct pwm_state state;
	uint8_t mode;
	uint8_t preScaler;
	uint8_t scaler;
	uint8_t alignment;
}dmec_pwm_channel;

static dmec_pwm_channel channels[2];

static inline struct dmec_pwm_chip *to_dmec(struct pwm_chip *chip)
{
	return container_of(chip, struct dmec_pwm_chip, chip);
}

static void dmec_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	dmec_pwm_channel *channel;
	int ret = 0;
	unsigned int val;
	unsigned int currentBaseReg;
	unsigned int index = 0;

	if ((pwm->hwpwm == 1) && (channels[0].mode)) /* channel 1 in 16-bit mode is not active */
	{
		dev_info(chip->dev, "second PWM channel is not active in 16-bit mode\n");
		return;
	}

	if (pwm->hwpwm == 1) /* channel 1 */
	{
		currentBaseReg = PWMB_BASE;
		index = 1;
	}
	else if (pwm->hwpwm == 0 && GpioConfigured_0) /* channel 0*/
	{
		currentBaseReg = PWMA_BASE;
		index = 0;
	}
	else /*channel 1*/
	{
		currentBaseReg = PWMB_BASE;
		index = 1;
	}

	channel = &channels[index];

	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMCFG, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM config\n", pwm->label);
		return;
	}

	/* mode : 1= 16bit, 0=8bit*/
	if(currentBaseReg == PWMA_BASE) /*mode bit is available only in ch1*/
		channel->mode = (val & PWM16) ? 1 : 0;
	else
		channel->mode = 0;

	/* enable state*/
	channel->state.enabled = (val & PWMEN) ? true : false;

	/* alignment */
	channel->alignment = (val & PWMCA) ? 1 : 0;

	/* polarity state*/
	channel->state.polarity = (val & PWMPOL) ? PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;
	
	/* clk */
	channel->preScaler = val & PWMCLK_MASK;

	/* scaler register */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMSCL, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM scaler\n", pwm->label);
		return;
	}
	channel->scaler = val;

	/* period state */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMPER, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM period\n", pwm->label);
		return;
	}
	channel->state.period = val;

	/* duty state */
	ret = regmap_read(dmecPwm->regmap, currentBaseReg + PWMDTY, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM duty-cycle\n", pwm->label);
		return;
	}
	channel->state.duty_cycle = val;

	/* 16bit only supported witch channel 0*/
	if(channels[0].mode)
	{
		/* period state */
		ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMPER, &val);
		channel->state.period |= val << 8;
		
		/* duty state */
		ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMDTY, &val);
		channel->state.duty_cycle |= val << 8;
	}

	*state = channel->state;
}

static int dmec_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm, 
				struct pwm_state *state)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	dmec_pwm_channel *channel;
	unsigned int val;
	int ret = 0;
	unsigned int currentBaseReg;
	int index =0;
	
	if (!state)
		return -EINVAL;

	if ((pwm->hwpwm == 1) && (channels[0].mode)) /* channel 1 in 16-bit mode is not active */
	{
		dev_info(chip->dev, "second PWM channel is not active in 16-bit mode\n");
		return -EPERM;
	}

	if (pwm->hwpwm == 1) /* channel 1 */
	{
		currentBaseReg = PWMB_BASE;
		index = 1;
	}
	else if (pwm->hwpwm == 0 && GpioConfigured_0) /* channel 0*/
	{
		currentBaseReg = PWMA_BASE;
		index = 0;
	}
	else /*channel 1*/
	{
		currentBaseReg = PWMB_BASE;
		index = 1;
	}
	channel = &channels[index];

	/* check enable request*/
	if(channel->state.enabled != state->enabled)
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

		mutex_lock(&mutex); 
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
		if (ret < 0)
			dev_err(chip->dev, "%s: Failed to enable/disable PWM\n", 
					pwm->label);
		mutex_unlock(&mutex);
		channel->state.enabled = state->enabled;
	}

	/* check period request */
	if(channels[0].mode)
		state->period = ((uint16_t)(state->period)) % 0x10000; /*mod 65536*/
	else
		state->period = ((uint8_t)(state->period)) % 0x100; /* mod 256 */

	if(channel->state.period != state->period)
	{
		if (channel->state.enabled) /*channel is enable*/
		{
			dev_err(chip->dev, "%s: Cannot change PWM period while enabled\n", 
					pwm->label);
			return -EBUSY;
		}

		mutex_lock(&mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMPER, 
				(uint8_t)state->period & 0xFF);
		if(channels[0].mode) /*16bit*/
			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMPER, 
					(uint8_t)((state->period >> 8) & 0xFF));
		mutex_unlock(&mutex);
		channel->state.period = state->period;

		pwm_set_period(pwm, state->period);
	}

	/* check duty-cycle */
	if(channels[0].mode)
		state->duty_cycle = ((uint16_t)(state->duty_cycle)) % 0x10000;
	else
		state->duty_cycle = ((uint8_t)(state->duty_cycle)) % 0x100;

	if(channel->state.duty_cycle != state->duty_cycle)
	{
		mutex_lock(&mutex);
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMDTY, 
				(uint8_t)(state->duty_cycle & 0xFF));
		if(channels[0].mode) /* 16bit*/
			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMDTY, 
					(uint8_t)((state->duty_cycle >> 8) & 0xFF));
		mutex_unlock(&mutex);
		channel->state.duty_cycle = state->duty_cycle;
		pwm_set_duty_cycle(pwm, state->duty_cycle);
	}

	/* check polarity */
	if(channel->state.polarity != state->polarity)
	{
		if (channel->state.enabled) /*channel is enable*/
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
			val |= PWMPOL;
		else
			val |= 0;

		mutex_lock(&mutex); 
		ret = regmap_write(dmecPwm->regmap, currentBaseReg + PWMCFG, val);
		if (ret < 0)
			dev_err(chip->dev, "%s: Failed to change PWM polarity\n", 
					pwm->label);
		mutex_unlock(&mutex);
		channel->state.polarity = state->polarity;
	}
	
	return ret;
}

static ssize_t dmec_pwm_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	regmap_read(dmecPwm->regmap, PWMREV, &val);

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n", (val >> 4) & 0xf, val & 0xf);
}

static ssize_t dmec_pwm_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].mode);
}

static ssize_t dmec_pwm_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0, valB = 0;
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);
	struct pwm_device *pwm0 = &dmecPwm->chip.pwms[0];
	struct pwm_device *pwm1 = &dmecPwm->chip.pwms[1];
	bool flag = 0;

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[0].mode != convertedValue) && 
		    (convertedValue == 0 || convertedValue == 1)) /*mode change request*/
		{
			ret = regmap_read(dmecPwm->regmap, PWMA_BASE + PWMCFG, &val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to read PWM Mode\n");
				return -EPERM;
			}
			val &= ~PWM16;

			if(convertedValue)/* 16-bit */
				val |= DMEC_PWM_16_BIT;
			else /* 8-bit */
				val |= DMEC_PWM_8_BIT;

			/* mode changes can be applied only in disable channel*/
			if(channels[0].state.enabled)
			{
				val &= ~PWMEN;
				val |= DMEC_PWM_EN_OFF;
			}

			if(channels[1].state.enabled)
			{
				ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMCFG, &valB);
				valB &= ~PWMEN;
				valB |= DMEC_PWM_EN_OFF;

				mutex_lock(&mutex); 
				ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMCFG, valB);
				if (ret < 0)
				{
					dev_err(dev, "Failed to disable second PWM\n");
					flag = 1;
					goto unlock;
				}
				mutex_unlock(&mutex);
				channels[1].state.enabled = false;
				pwm_disable(pwm1);
			}

			mutex_lock(&mutex); 
			ret = regmap_write(dmecPwm->regmap, PWMA_BASE + PWMCFG, val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM mode\n");
				flag=  1;
				goto unlock;
			}
			mutex_unlock(&mutex);

			channels[0].mode = convertedValue;

			pwm_disable(pwm0);
			channels[0].state.enabled = false;
		}
		ret = count;
	}
unlock:
	if (flag)
		mutex_unlock(&mutex);
	return ret;
}

static ssize_t dmec_pwm_preScaler0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].preScaler);
}

static ssize_t dmec_pwm_preScaler0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[0].preScaler != convertedValue) && 
		    (convertedValue >= 0 &&  convertedValue <= 7)) /*preScaler change request*/
		{
			ret = regmap_read(dmecPwm->regmap, PWMA_BASE + PWMCFG, &val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to read PWM preScaler\n");
				return -EPERM;
			}
			val &= ~PWMCLK_MASK;
			val |= (uint8_t)(convertedValue);

			mutex_lock(&mutex);
			ret = regmap_write(dmecPwm->regmap, PWMA_BASE + PWMCFG, val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM preScaler\n");
				goto unlock;
			}
			channels[0].preScaler = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_preScaler1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[1].preScaler);
}

static ssize_t dmec_pwm_preScaler1_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[1].preScaler != convertedValue) && 
		    (convertedValue >= 0 &&  convertedValue <= 7)) /*preScaler change request*/
		{
			ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMCFG, &val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to read PWM preScaler\n");
				return -EPERM;
			}
			val &= ~PWMCLK_MASK;
			val |= (uint8_t)(convertedValue);

			mutex_lock(&mutex); 

			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMCFG, val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM preScaler\n");
				goto unlock;
			}
			channels[1].preScaler = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_alignment0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].alignment);
}

static ssize_t dmec_pwm_alignment0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[0].alignment != convertedValue) && 
		    (convertedValue == 0 ||  convertedValue == 1)) /*alignment change request*/
		{
			ret = regmap_read(dmecPwm->regmap, PWMA_BASE + PWMCFG, &val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to read PWM alignment\n");
				return -EPERM;
			}
			val &= ~PWMCA;
			if(convertedValue)
				val |= DMEC_PWM_CA_CENTER;
			else
				val |= DMEC_PWM_CA_LEFT;

			mutex_lock(&mutex); 

			ret = regmap_write(dmecPwm->regmap, PWMA_BASE + PWMCFG, val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM alignment\n");
				goto unlock;
			}
			channels[0].alignment = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_alignment1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[1].alignment);
}

static ssize_t dmec_pwm_alignment1_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	unsigned int val = 0;
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[1].alignment != convertedValue) && 
		    (convertedValue == 0 ||  convertedValue == 1)) /*alignment change request*/
		{
			ret = regmap_read(dmecPwm->regmap, PWMB_BASE + PWMCFG, &val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to read PWM alignment\n");
				return -EPERM;
			}
			val &= ~PWMCA;
			if(convertedValue)
				val |= DMEC_PWM_CA_CENTER;
			else
				val |= DMEC_PWM_CA_LEFT;

			mutex_lock(&mutex); 

			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMCFG, val);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM alignment\n");
				goto unlock;
			}
			channels[1].alignment = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_scaler0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].scaler);
}

static ssize_t dmec_pwm_scaler0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[0].scaler != convertedValue) && 
		    (convertedValue >= 0 &&  convertedValue <= 255)) /*scaler change request*/
		{

			mutex_lock(&mutex); 

			ret = regmap_write(dmecPwm->regmap, PWMA_BASE + PWMSCL, convertedValue);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM scaler\n");
				goto unlock;
			}
			channels[0].scaler = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_scaler1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[1].scaler);
}

static ssize_t dmec_pwm_scaler1_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret = 0;
	long convertedValue;
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[1].scaler != convertedValue) && 
		    (convertedValue >= 0 &&  convertedValue <= 255)) /*scaler change request*/
		{

			mutex_lock(&mutex); 

			ret = regmap_write(dmecPwm->regmap, PWMB_BASE + PWMSCL, convertedValue);
			if (ret < 0)
			{
				dev_err(dev, "Failed to change PWM scaler\n");
				goto unlock;
			}
			channels[1].scaler = convertedValue;
unlock:
			mutex_unlock(&mutex);
		}
		ret =count;
	}
	return ret;
}

static DEVICE_ATTR(version, S_IRUGO,dmec_pwm_version_show, NULL );
static DEVICE_ATTR(mode,    S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_mode_show, dmec_pwm_mode_store );
static DEVICE_ATTR(preScaler0,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_preScaler0_show, dmec_pwm_preScaler0_store );
static DEVICE_ATTR(preScaler1,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_preScaler1_show, dmec_pwm_preScaler1_store );
static DEVICE_ATTR(alignment0,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_alignment0_show, dmec_pwm_alignment0_store );
static DEVICE_ATTR(alignment1,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_alignment1_show, dmec_pwm_alignment1_store );
static DEVICE_ATTR(scaler0,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_scaler0_show, dmec_pwm_scaler0_store );
static DEVICE_ATTR(scaler1,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_scaler1_show, dmec_pwm_scaler1_store );

static struct attribute *pwm_attributeAll[]= {
	&dev_attr_version.attr,
	&dev_attr_mode.attr,
	&dev_attr_preScaler0.attr,
	&dev_attr_preScaler1.attr,
	&dev_attr_alignment0.attr,
	&dev_attr_alignment1.attr,
	&dev_attr_scaler0.attr,
	&dev_attr_scaler1.attr,
	NULL
};

static struct attribute *pwm_attributeA[]= {
	&dev_attr_version.attr,
	&dev_attr_preScaler0.attr,
	&dev_attr_alignment0.attr,
	&dev_attr_scaler0.attr,
	NULL
};

static struct attribute *pwm_attributeB[]= {
	&dev_attr_version.attr,
	&dev_attr_preScaler1.attr,
	&dev_attr_alignment1.attr,
	&dev_attr_scaler1.attr,
	NULL
};

static const struct attribute_group pwm_groupAll = {
	.attrs = pwm_attributeAll,
};

static const struct attribute_group pwm_groupA = {
	.attrs = pwm_attributeA,
};

static const struct attribute_group pwm_groupB = {
	.attrs = pwm_attributeB,
};

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

	ret = regmap_read(dmecPwm->regmap, PWMA_BASE + PWMCFG, &val);
	channels[0].mode = (val & PWM16) ? 1 : 0;
	channels[1].mode = 0;

	dmecPwm->chip.ops = &dmec_pwm_ops;

	dmecPwm->chip.npwm = 0;
	if (GpioConfigured_0 == 1)
		dmecPwm->chip.npwm += 1;
	if (GpioConfigured_1 == 1)
		dmecPwm->chip.npwm += 1;

	dmecPwm->chip.dev = &pdev->dev;
	dmecPwm->chip.base = -1;

	mutex_init(&mutex);

	ret = pwmchip_add(&dmecPwm->chip);
	if (ret < 0)
	{
		dev_err(&pdev->dev, "pwmchip_add() failed: %d\n", ret);
		devm_kfree(&pdev->dev, dmecPwm);
		return ret;
	}

	platform_set_drvdata(pdev, dmecPwm);
	if((GpioConfigured_0 == 1) && (GpioConfigured_1 == 1)) /* both channels*/
		ret = sysfs_create_group(&pdev->dev.kobj, &pwm_groupAll);
	else if((GpioConfigured_0 == 1) && (GpioConfigured_1 == 0)) /* only channel A*/
		ret = sysfs_create_group(&pdev->dev.kobj, &pwm_groupA);
	else if((GpioConfigured_0 == 0) && (GpioConfigured_1 == 1)) /* only channel B*/
		ret = sysfs_create_group(&pdev->dev.kobj, &pwm_groupB);

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
