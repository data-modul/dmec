#ifndef _LINUX_MFD_DMEC_H
#define _LINUX_MFD_DMEC_H

struct dmec_i2c_platform_data {
	u32 clock_khz; /* input clock in kHz */
};

struct regmap *dmec_get_regmap(struct device *dev);

#endif
