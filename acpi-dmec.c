/*
 * ACPI-HWMON table  for Data Modul AG Embedded Controller
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
#include <linux/acpi.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "dmec.h"

static struct regmap *regmap;

static unsigned long long dmec_acpi_get_data(char* input, int inparam)
{
	acpi_status status;
	acpi_handle handle;
	union acpi_object arg;
	union acpi_object *result_buffer;
	struct acpi_object_list list_arg;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL};

	status = acpi_get_handle(NULL, (acpi_string) input, &handle);
	if(ACPI_FAILURE(status))
		return  0xFFFFFFFF;

	arg.type = ACPI_TYPE_INTEGER;
	arg.integer.value = inparam;

	list_arg.count = 1;
	list_arg.pointer = &arg;

	status = acpi_evaluate_object(handle, NULL, &list_arg, &buffer);
	if(ACPI_FAILURE(status))
		return 0xFFFFFFFF;

	result_buffer = buffer.pointer;
	kfree(buffer.pointer);
	return result_buffer->integer.value;
}

static ssize_t dmec_acpi_cpu_fan(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.FSPW", 0x00);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_sys_fan(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.FSPW", 0x01);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_cpu_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.TMPW", 0x00);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_chipset_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.TMPW", 0x01);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_system_temperature(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.TMPW", 0x02);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_vcore_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x00);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_2v5_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x01);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_3v3_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x02);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_vbat_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x03);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_5v_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x04);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_5vsb_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x05);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static ssize_t dmec_acpi_12v_voltage(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	unsigned long long val = 0;
	val = dmec_acpi_get_data("\\_SB.PCI0.LPCB.DMEC.HVTW", 0x06);

	return scnprintf(buf, PAGE_SIZE, "%llu\n", val);
}

static DEVICE_ATTR(acpi_fan_cpu, S_IRUGO, dmec_acpi_cpu_fan, NULL);
static DEVICE_ATTR(acpi_fan_sys, S_IRUGO, dmec_acpi_sys_fan, NULL);
static DEVICE_ATTR(acpi_temp_cpu, S_IRUGO, dmec_acpi_cpu_temperature, NULL);
static DEVICE_ATTR(acpi_temp_chipset, S_IRUGO, dmec_acpi_chipset_temperature, NULL);
static DEVICE_ATTR(acpi_temp_sys, S_IRUGO, dmec_acpi_system_temperature, NULL);
static DEVICE_ATTR(acpi_vol_vcore, S_IRUGO, dmec_acpi_vcore_voltage, NULL);
static DEVICE_ATTR(acpi_vol_2v5, S_IRUGO, dmec_acpi_2v5_voltage, NULL);
static DEVICE_ATTR(acpi_vol_3v3, S_IRUGO, dmec_acpi_3v3_voltage, NULL);
static DEVICE_ATTR(acpi_vol_vbat, S_IRUGO, dmec_acpi_vbat_voltage, NULL);
static DEVICE_ATTR(acpi_vol_5v, S_IRUGO, dmec_acpi_5v_voltage, NULL);
static DEVICE_ATTR(acpi_vol_5vsb, S_IRUGO, dmec_acpi_5vsb_voltage, NULL);
static DEVICE_ATTR(acpi_vol_12v, S_IRUGO, dmec_acpi_12v_voltage, NULL);

static struct attribute *acpi_attribute[] = {
	&dev_attr_acpi_fan_cpu.attr,
	&dev_attr_acpi_fan_sys.attr,
	&dev_attr_acpi_temp_cpu.attr,
	&dev_attr_acpi_temp_chipset.attr,
	&dev_attr_acpi_temp_sys.attr,
	&dev_attr_acpi_vol_vcore.attr,
	&dev_attr_acpi_vol_2v5.attr,
	&dev_attr_acpi_vol_3v3.attr,
	&dev_attr_acpi_vol_vbat.attr,
	&dev_attr_acpi_vol_5v.attr,
	&dev_attr_acpi_vol_5vsb.attr,
	&dev_attr_acpi_vol_12v.attr,
	NULL
};

static const struct attribute_group acpi_attr_group = {
	.attrs = acpi_attribute,
};

static int dmec_acpi_probe(struct platform_device *pdev)
{
	int ret = 0;

	regmap = dmec_get_regmap(pdev->dev.parent);

	ret = sysfs_create_group(&pdev->dev.kobj, &acpi_attr_group);
	if (ret)
		return ret;
	return ret;
}

static int dmec_acpi_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &acpi_attr_group);

	return 0;
}

static struct platform_driver dmec_acpi_driver = {
	.driver = {
			.name = "dmec-acpi",
			.owner = THIS_MODULE,
	},
	.probe = dmec_acpi_probe,
	.remove = dmec_acpi_remove,
};

module_platform_driver(dmec_acpi_driver);

MODULE_DESCRIPTION("dmec acpi driver");
MODULE_AUTHOR("Reyhaneh Yazdani <reyhane.y84@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:dmec-acpi");

