#ifndef _LINUX_MFD_DMEC_H
#define _LINUX_MFD_DMEC_H

struct dmec_gpio_platform_data {
	int gpio_base;
	int chip_num;
};

struct dmec_i2c_platform_data {
	u32 reg_shift; /* register offset shift value */
	u32 reg_io_width; /* register io read/write width */
	u32 clock_khz; /* input clock in kHz */
	bool big_endian; /* registers are big endian */
	u8 num_devices; /* number of devices in the devices list */
	struct i2c_board_info const *devices; /* devices connected to the bus */
};

struct regmap *dmec_get_regmap(struct device *dev);

#endif
