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

#ifndef UINT32_MAX
	#define UINT32_MAX 0xFFFFFFFF
#endif

#ifndef UINT16_MAX
	#define UINT16_MAX 0xFFFF
#endif

#ifndef UINT8_MAX
	#define UINT8_MAX 0xFF
#endif

#define PWM_SRC_CLK           50000000    /* default input clock is 50MHz */
#define DMEC_PWM_CLK_GRANULARITY        (uint32_t)(1000000000/PWM_SRC_CLK)
#define DMEC_PWM_GRANULARITY(p, s, a)   (uint32_t)(DMEC_PWM_CLK_GRANULARITY * (uint32_t)(1 << p) * (uint32_t)(s + 1) * (a?2:1))

#define DMEC_PWM_MIN_STEPS              8
#define DMEC_PWM_MAX_STEPS_8BIT         (uint32_t)UINT8_MAX
#define DMEC_PWM_MAX_STEPS_16BIT        (uint32_t)UINT16_MAX

#define DMEC_PWM_MODE_8BIT              0
#define DMEC_PWM_MODE_16BIT             1

#define DMEC_PWM_MIN_SCALER             0
#define DMEC_PWM_MAX_SCALER             UINT8_MAX

#define DMEC_PWM_MIN_PRESCALER_VALUE    0
#define DMEC_PWM_MIN_PRESCALER          (1 << DMEC_PWM_MIN_PRESCALER_VALUE) 
#define DMEC_PWM_MAX_PRESCALER_VALUE    7
#define DMEC_PWM_MAX_PRESCALER          (1 << DMEC_PWM_MAX_PRESCALER_VALUE)

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

enum pin {PIN0,PIN1,PIN01,NOPIN};

static struct regmap *regmap;
static struct mutex mutex;

struct dmec_pwm_chip {
	struct pwm_chip chip;
	enum pin pin_value;
	uint32_t minSteps;
	uint32_t maxSteps;
};

typedef struct _dmec_pwm_channel {
	struct pwm_state state;
	uint8_t mode;
	uint8_t preScaler;
	uint8_t scaler;
	uint8_t alignment;
	uint32_t granularity;
}dmec_pwm_channel;

static dmec_pwm_channel channels[2];

/**
  
  Calculate best match PWM parameters.

  @param[in]    Period        Desired Period in ns.
  @param[in]    MinSteps      Minimum number of steps, 0 means auto.
  @param[in]    MaxSteps      Maximum number of steps, 0 means auto.
  @param[in]    Align         0 = Left Aligned, 1 = Center aligned.
  @param[in]    Mode          0 = 8Bit, 1 = 16Bit.
  @param[out]   PreScaler     Returns calculated prescaler.
  @param[out]   Scaler        Returns calculated channel scaler.
  @param[out]   PeriodReg     Returns calculated period.
  
**/
static int
PwmCalcParms (uint32_t  Period,
		uint32_t  MinSteps,
		uint32_t  MaxSteps,
		uint8_t   Align,
		uint8_t   Mode,
		uint8_t   *PreScaler,
		uint8_t   *Scaler,
		uint16_t  *PeriodReg
)
{
  uint32_t PwmClk, PwmScl, PwmPer;
  uint8_t PwmClkMatch = 0;
  uint8_t  PwmSclMatch = 0;
  uint16_t PwmPerMatch = 0;
  uint32_t u32Deviation;
  uint32_t u32lPeriod;
  uint32_t u32LastDeviation = Period;
  uint32_t TotalGranularity;
  uint32_t MaxPeriod;
  uint32_t MinPeriod;
  uint32_t lMinSteps = MinSteps;
  uint32_t lMaxSteps = MaxSteps;
  uint8_t AlignMultiplier = Align?2:1;
  uint8_t FoundParms = 0;
  uint32_t Factor, Steps, TotalScaler;

  /* some sanity checking */
  if (lMinSteps == 0) {
    lMinSteps = DMEC_PWM_MIN_STEPS;
  }
  
  if (((Mode == DMEC_PWM_MODE_8BIT) && (lMinSteps > DMEC_PWM_MAX_STEPS_8BIT)) || 
      ((Mode == DMEC_PWM_MODE_16BIT) && (lMinSteps > DMEC_PWM_MAX_STEPS_16BIT))) {
    return -1;
  }
  if (lMaxSteps == 0) {
    if (Mode == DMEC_PWM_MODE_8BIT) {
      lMaxSteps = DMEC_PWM_MAX_STEPS_8BIT;
    } else {
      lMaxSteps = DMEC_PWM_MAX_STEPS_16BIT;
    }
  }
  
  /* min/max period bounds-check for the requested configuration */
  MaxPeriod = DMEC_PWM_CLK_GRANULARITY * (uint32_t)AlignMultiplier * lMaxSteps * (DMEC_PWM_MAX_SCALER + 1) * DMEC_PWM_MAX_PRESCALER;
  MinPeriod = DMEC_PWM_CLK_GRANULARITY * (uint32_t)AlignMultiplier * lMinSteps * (DMEC_PWM_MIN_SCALER + 1) * DMEC_PWM_MIN_PRESCALER;

  if ((MaxPeriod < Period) || (MinPeriod > Period)) {
    return -1;
  }    

  /* iterate through all possible values and record the parameter 
     combination with the least deviation
 
    -> min(|Period - ((1 << PWMCLKx) * (PWMSCLx + 1) * PWMPERx / 50.000.000Hz)|)
  
  */
  TotalGranularity = DMEC_PWM_CLK_GRANULARITY * AlignMultiplier;
  Factor = Period/TotalGranularity; 
  for (PwmClk = DMEC_PWM_MIN_PRESCALER_VALUE; PwmClk <= DMEC_PWM_MAX_PRESCALER_VALUE; PwmClk++) {
    for (PwmScl = DMEC_PWM_MIN_SCALER; PwmScl <= DMEC_PWM_MAX_SCALER; PwmScl++) {
      if (FoundParms) break;
      TotalScaler = (PwmScl+1)*(1 << PwmClk);
      Steps = Factor/TotalScaler;
      /* quick check: skip period register loop if out of range */
      if ((Steps > lMaxSteps) || (Steps < lMinSteps)) 
          continue;
      for (PwmPer = lMaxSteps; PwmPer >= lMinSteps; PwmPer--) {
        if (FoundParms) break;
        //check for UINT32 overflow, skip larger values
        if (PwmPer > (UINT32_MAX / (TotalGranularity * TotalScaler))) {
          PwmPer =  (UINT32_MAX / (TotalGranularity * TotalScaler)) + 1;
          continue;
        }
        u32lPeriod = TotalGranularity * PwmPer * TotalScaler;
        if (Period < u32lPeriod) {
          u32Deviation = u32lPeriod - Period;
        } else {
          u32Deviation = Period - u32lPeriod;
        }          
        if (u32Deviation < u32LastDeviation) {
          PwmClkMatch = (uint8_t)PwmClk;
          PwmSclMatch = (uint8_t)PwmScl;
          PwmPerMatch = (uint16_t)PwmPer;
          u32LastDeviation = u32Deviation;
          if (u32LastDeviation == 0) FoundParms = 1;
        }
      }
    }
  }

  if (PreScaler != NULL) *PreScaler = (uint8_t)PwmClkMatch;
  if (Scaler != NULL) *Scaler = (uint8_t)PwmSclMatch;
  if (PreScaler != NULL) *PeriodReg = (uint16_t)PwmPerMatch;

  return 0;
}


/**

  Calculate duty cycle with remainder check.
      
  @param[in]        Duty          Duty cycle in ns
  @param[in]        Granularity   PWM Granularity in ns
  
  @return           Duty Cycle value.

**/
uint16_t CalcDuty (uint32_t Duty, uint32_t Granularity
    )
{
  if ((Duty % Granularity) > (Granularity/2)) return ((uint16_t)(1 + (Duty/Granularity)));
  return ((uint16_t)(Duty/Granularity));
}


static int pwmWriteReg (uint8_t base, uint8_t reg, uint8_t value)
{
	int ret = 0;

	mutex_lock(&mutex);
	ret = regmap_write(regmap, base + reg, value);
	mutex_unlock(&mutex);
	return ret;
}

static int pwmConfigureWriteReg (uint8_t base, uint8_t reg, uint8_t value, uint8_t mask)
{
	int ret = 0;
	unsigned int val = 0;

	ret = regmap_read(regmap, base + PWMCFG, &val);
	if (ret < 0)
		return -EPERM;

	val &= ~mask;
	val |= (uint8_t)(value);
	
	ret = pwmWriteReg (base, PWMCFG, val);
	return ret;
}

static inline struct dmec_pwm_chip *to_dmec(struct pwm_chip *chip)
{
	return container_of(chip, struct dmec_pwm_chip, chip);
}

static void dmec_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
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

	ret = regmap_read(regmap, currentBaseReg + PWMCFG, &val);
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
	ret = regmap_read(regmap, currentBaseReg + PWMSCL, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM scaler\n", pwm->label);
		return;
	}
	channel->scaler = val;

	/* period state */
	ret = regmap_read(regmap, currentBaseReg + PWMPER, &val);
	if (ret < 0)
	{
		dev_err(chip->dev, "%s: Failed to read PWM period\n", pwm->label);
		return;
	}
	channel->state.period = val;

	/* duty state */
	ret = regmap_read(regmap, currentBaseReg + PWMDTY, &val);
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
		ret = regmap_read(regmap, PWMB_BASE + PWMPER, &val);
		channel->state.period |= val << 8;
		
		/* duty state */
		ret = regmap_read(regmap, PWMB_BASE + PWMDTY, &val);
		channel->state.duty_cycle |= val << 8;
	}

	*state = channel->state;
}

static int dmec_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm, 
				struct pwm_state *state)
{
	struct dmec_pwm_chip *dmecPwm = to_dmec(chip);
	dmec_pwm_channel *channel;
	int ret = 0;
	unsigned int currentBaseReg;
	int index =0;
	uint8_t p,s;
	uint16_t r;
	uint16_t  tempduty;

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
		if(state->enabled)
			ret = pwmConfigureWriteReg (currentBaseReg, PWMCFG, DMEC_PWM_EN_ON, PWMEN);
		else
			ret = pwmConfigureWriteReg (currentBaseReg, PWMCFG, DMEC_PWM_EN_OFF, PWMEN);
		if (ret < 0)
			dev_err(chip->dev, "Failed to to enable/disable PWM\n");
		channel->state.enabled = state->enabled;
	}

	/* check period request */
	if(channel->state.period != state->period)
	{
		if (channel->state.enabled) /*channel is enable*/
		{
			dev_err(chip->dev, "%s: Cannot change PWM period while enabled\n", 
					pwm->label);
			return -EBUSY;
		}

		ret = PwmCalcParms(state->period,
				   dmecPwm->minSteps,
				   dmecPwm->maxSteps,
				   channel->alignment,
				   channels[0].mode,
				   &p,
				   &s,
				   &r);
		if (ret < 0)
		{
			dev_err(chip->dev, "%s: Cannot change PWM period(value is unsupported)\n", 
					pwm->label);
			return -EPERM;
		}

		channel->granularity = DMEC_PWM_GRANULARITY(p, s, channel->alignment);
		ret = pwmWriteReg (currentBaseReg, PWMPER, (uint8_t)r & 0xFF);
		if(channels[0].mode) /*16bit*/
			ret = pwmWriteReg (PWMB_BASE, PWMPER,(uint8_t)((r >> 8) & 0xFF));
		if (ret < 0)
		{
			dev_err(chip->dev, "%s: Cannot change PWM period\n", 
					pwm->label);
			return -EPERM;
		}

		channel->state.period = r * channel->granularity;
		state->period = channel->state.period;
		pwm_set_period(pwm, channel->state.period);

		/* change pre scale*/
		ret = pwmConfigureWriteReg (currentBaseReg, PWMCFG, p, PWMCLK_MASK);
		if (ret < 0)
			dev_err(chip->dev, "Failed to change PWM preScaler\n");
		channel->preScaler = p;

		/* change scaler*/
		ret = pwmWriteReg (currentBaseReg, PWMSCL, s);
		if (ret < 0)
			dev_err(chip->dev, "Failed to change PWM Scaler\n");
		channel->scaler = s;

		/* duty cycle*/
		tempduty = CalcDuty(channel->state.duty_cycle, channel->granularity);
		ret = pwmWriteReg (currentBaseReg, PWMDTY, (uint8_t)(tempduty & 0xFF));
		if(channels[0].mode) /* 16bit*/
			ret = pwmWriteReg (PWMB_BASE, PWMDTY, (uint8_t)((tempduty >> 8)& 0xFF));
		if (ret < 0)
			dev_err(chip->dev, "Failed to change PWM duty cycle\n");
		channel->state.duty_cycle = tempduty * channel->granularity ;
		state->duty_cycle = channel->state.duty_cycle;
	}

	/* check duty-cycle */
	if(channel->state.duty_cycle != state->duty_cycle)
	{
		channel->granularity = DMEC_PWM_GRANULARITY(channel->preScaler, channel->scaler, channel->alignment);

		if(state->duty_cycle > channel->state.period)
			state->duty_cycle = channel->state.period;

		tempduty = (uint16_t)CalcDuty(state->duty_cycle, channel->granularity);
		ret = pwmWriteReg (currentBaseReg, PWMDTY, (uint8_t)(tempduty & 0xFF));
		if(channels[0].mode) /* 16bit*/
			ret = pwmWriteReg (PWMB_BASE, PWMDTY, (uint8_t)((tempduty >> 8)& 0xFF));
		if (ret < 0)
			dev_err(chip->dev, "Failed to change PWM duty cycle\n");
		channel->state.duty_cycle = tempduty * channel->granularity ;
		state->duty_cycle = channel->state.duty_cycle;
	}

	/* check polarity */
	if(channel->state.polarity != state->polarity)
	{
		if(state->polarity == PWM_POLARITY_NORMAL)
			ret = pwmConfigureWriteReg (currentBaseReg, PWMCFG, PWMPOL, PWMPOL);
		else
			ret = pwmConfigureWriteReg (currentBaseReg, PWMCFG, 0, PWMPOL);
		if (ret < 0)
			dev_err(chip->dev, "Failed to change PWM preScaler\n");
		channel->state.polarity = state->polarity;
	}
	
	return ret;
}

static ssize_t dmec_pwm_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned int val = 0;

	regmap_read(regmap, PWMREV, &val);

	return scnprintf(buf, PAGE_SIZE, "%u.%u\n", (val >> 4) & 0xf, val & 0xf);
}

static ssize_t dmec_pwm_pin_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	if(dmecPwm->pin_value == PIN0)
		return scnprintf(buf, PAGE_SIZE, "pin0\n");
	else if(dmecPwm->pin_value == PIN1)
		return scnprintf(buf, PAGE_SIZE, "pin1\n");
	else if(dmecPwm->pin_value == PIN01)
		return scnprintf(buf, PAGE_SIZE, "pin01\n");
	else
		return scnprintf(buf, PAGE_SIZE, "noPin\n");
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
			ret = regmap_read(regmap, PWMA_BASE + PWMCFG, &val);
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
				ret = regmap_read(regmap, PWMB_BASE + PWMCFG, &valB);
				valB &= ~PWMEN;
				valB |= DMEC_PWM_EN_OFF;

				mutex_lock(&mutex); 
				ret = regmap_write(regmap, PWMB_BASE + PWMCFG, valB);
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
			ret = regmap_write(regmap, PWMA_BASE + PWMCFG, val);
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

static ssize_t dmec_pwm_alignment0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].alignment);
}

static ssize_t dmec_pwm_alignment0_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret = 0;
	long convertedValue;

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[0].alignment != convertedValue) && 
		    (convertedValue == 0 ||  convertedValue == 1)) /*alignment change request*/
		{
			if(convertedValue)
				ret = pwmConfigureWriteReg (PWMA_BASE, PWMCFG, DMEC_PWM_CA_CENTER, PWMCA);
			else
				ret = pwmConfigureWriteReg (PWMA_BASE, PWMCFG, DMEC_PWM_CA_LEFT, PWMCA);

			if(ret < 0)
				dev_err(dev, "Failed to change PWM alignment\n");
			else
				channels[0].alignment = convertedValue;
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
	int ret = 0;
	long convertedValue;

	if (!kstrtol(buf, 10, &convertedValue)) {
		if ((channels[1].alignment != convertedValue) && 
		    (convertedValue == 0 ||  convertedValue == 1)) /*alignment change request*/
		{
			if(convertedValue)
				ret = pwmConfigureWriteReg (PWMB_BASE, PWMCFG, DMEC_PWM_CA_CENTER, PWMCA);
			else
				ret = pwmConfigureWriteReg (PWMB_BASE, PWMCFG, DMEC_PWM_CA_LEFT, PWMCA);

			if(ret < 0)
				dev_err(dev, "Failed to change PWM alignment\n");
			else
				channels[1].alignment = convertedValue;
		}
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_minSteps_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", dmecPwm->minSteps);
}

static ssize_t dmec_pwm_minSteps_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);
	int ret = 0;
	long convertedValue;

	if (!kstrtol(buf, 10, &convertedValue)) {
		dmecPwm->minSteps = (uint32_t)convertedValue;
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_maxSteps_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", dmecPwm->maxSteps);
}

static ssize_t dmec_pwm_maxSteps_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct dmec_pwm_chip *dmecPwm = dev_get_drvdata(dev);
	int ret = 0;
	long convertedValue;

	if (!kstrtol(buf, 10, &convertedValue)) {
		dmecPwm->maxSteps = (uint32_t)convertedValue;
		ret = count;
	}
	return ret;
}

static ssize_t dmec_pwm_granularity0_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[0].granularity);
}

static ssize_t dmec_pwm_granularity1_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n", channels[1].granularity);
}

static DEVICE_ATTR(version, S_IRUGO,dmec_pwm_version_show, NULL );
static DEVICE_ATTR(pin, S_IRUGO,dmec_pwm_pin_show, NULL );
static DEVICE_ATTR(mode,    S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_mode_show, dmec_pwm_mode_store );
static DEVICE_ATTR(alignment0,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_alignment0_show, dmec_pwm_alignment0_store );
static DEVICE_ATTR(alignment1,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_alignment1_show, dmec_pwm_alignment1_store );
static DEVICE_ATTR(minSteps,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_minSteps_show, dmec_pwm_minSteps_store );
static DEVICE_ATTR(maxSteps,  S_IRUGO|S_IWUSR|S_IWGRP,dmec_pwm_maxSteps_show, dmec_pwm_maxSteps_store );
static DEVICE_ATTR(granularity0, S_IRUGO,dmec_pwm_granularity0_show, NULL );
static DEVICE_ATTR(granularity1, S_IRUGO,dmec_pwm_granularity1_show, NULL );

static struct attribute *pwm_attributeAll[]= {
	&dev_attr_version.attr,
	&dev_attr_pin.attr,
	&dev_attr_mode.attr,
	&dev_attr_alignment0.attr,
	&dev_attr_alignment1.attr,
	&dev_attr_minSteps.attr,
	&dev_attr_maxSteps.attr,
	&dev_attr_granularity0.attr,
	&dev_attr_granularity1.attr,
	NULL
};

static struct attribute *pwm_attributeA[]= {
	&dev_attr_version.attr,
	&dev_attr_pin.attr,
	&dev_attr_alignment0.attr,
	&dev_attr_minSteps.attr,
	&dev_attr_maxSteps.attr,
	&dev_attr_granularity0.attr,
	NULL
};

static struct attribute *pwm_attributeB[]= {
	&dev_attr_version.attr,
	&dev_attr_pin.attr,
	&dev_attr_alignment1.attr,
	&dev_attr_minSteps.attr,
	&dev_attr_maxSteps.attr,
	&dev_attr_granularity1.attr,
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

	regmap = dmec_get_regmap(pdev->dev.parent);

	dmecPwm->minSteps = 0;
	dmecPwm->maxSteps = 0;

	/* check pwm is set in corresponding GPIOA-4 and GPIOA-5 pin in BIOS */
	ret = regmap_read(regmap, DMEC_PWM_PAR1, &val);
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
	if ((GpioConfigured_0 == 1) && (GpioConfigured_1 == 1))
		dmecPwm->pin_value = PIN01;
	else if (GpioConfigured_0 == 1)
		dmecPwm->pin_value = PIN0;
	else if (GpioConfigured_1 == 1)
		dmecPwm->pin_value = PIN1;
	else
		dmecPwm->pin_value = NOPIN;

	ret = regmap_read(regmap, PWMA_BASE + PWMCFG, &val);
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
	int err;

	err = pwmchip_remove(&dmecPwm->chip);
	if(err < 0)
	{
		dev_err(&pdev->dev, "remove pwm driver failed: %d\n", err); 
		return err;
	}
	dev_dbg(&pdev->dev, "pwm driver removed.");
	return 0;
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
