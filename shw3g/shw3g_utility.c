/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[shw3g]:" fmt

#define SENSOR_ADDRESS 		(0x36)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "inc/shw3g_setting.h"
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

static int32_t poc_addr = 0xff;

int32_t write_register(uint32_t bus, uint32_t i2c_addr, int32_t reg_width,
					   int32_t setting_size, uint32_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int32_t ret = RET_OK;
	int i = 0, k, al, dl;
	uint32_t read_value;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = (uint32_t)reg_width;

	while (i < setting_size) {
		if ((cam_setting[i] == DELAY_FLAG || cam_setting[i] == DELAY1_FLAG) &&
			(i + 1) < setting_size) {
			uint32_t delay_ms = cam_setting[i + 1];
			usleep(delay_ms * 1000u);
			cam_dbg("Delay %ums executed.\n", delay_ms);
			i += 2;
			continue;
		}
		i2c_cfg.reg = cam_setting[i];
		i2c_cfg.data = cam_setting[i + 1];
		uint32_t expected_value = i2c_cfg.data;

		k = CAM_I2C_RETRY_MAX;
		int write_success = 0;
		int read_success = 0;

		do {
			/* 写入操作 */
			if (i2c_cfg.reg_size == REG16_VAL16) {
				al = 16;
				dl = 16;
				ret = camera_i2c_write_reg16_data16(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint16_t)i2c_cfg.data);
			} else if (i2c_cfg.reg_size == REG16_VAL8) {
				al = 16;
				dl = 8;
				ret = camera_i2c_write_reg16_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			} else {
				al = 8;
				dl = 8;
				ret = camera_i2c_write_reg8_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			}

			if (ret != RET_OK) {
				cam_warn("Write failed: W%d %d@0x%02x: 0x%0*x = 0x%0*x retry %d\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, i2c_cfg.data,
					CAM_I2C_RETRY_MAX + 1 - k);
				camera_sys_msleep(20);
				continue;
			}

			write_success = 1;
			cam_dbg("Write success: W%d %d@0x%02x: 0x%0*x = 0x%0*x\n",
				al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, i2c_cfg.data);

#ifdef ENABLE_READBACK_VERIFICATION
			int32_t read_ret;
			if (i2c_cfg.reg_size == REG16_VAL16) {
				read_ret = camera_i2c_read_reg16_data16(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			} else if (i2c_cfg.reg_size == REG16_VAL8) {
				read_ret = camera_i2c_read_reg16_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			} else {
				read_ret = camera_i2c_read_reg8_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			}

			if (ret != RET_OK) {
				cam_warn("Read failed: R%d %d@0x%02x: 0x%0*x retry %d\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg,
					CAM_I2C_RETRY_MAX + 1 - k);
				camera_sys_msleep(10);
				continue;
			}

			if (read_value == expected_value) {
				read_success = 1;
				cam_dbg("Verify success: R%d %d@0x%02x: 0x%0*x = 0x%0*x\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, read_value);
				break;
			} else {
				cam_warn("Verify mismatch: R%d %d@0x%02x: 0x%0*x = 0x%0*x (expected 0x%0*x)\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, read_value, dl / 4, expected_value);
				read_success = 1;
				break;
			}
#else
			read_success = 1;
			break;
#endif
		} while (k-- > 0);

		if (!write_success || !read_success) {
			cam_dbg("Operation failed: %s %d@0x%02x: 0x%0*x = 0x%0*x\n",
				(write_success ? "Read" : "Write"), bus, i2c_addr,
				al / 4, i2c_cfg.reg, dl / 4, expected_value);
			ret = RET_ERROR;
			break;
		}

		i += 2;
	}
	return ret;
}

int32_t serdes_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	static int32_t bus;

	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	bus = sensor_info->bus_num;


	struct {
		uint32_t dev_addr;
		uint32_t *data;
		size_t size;
		const char *err_msg;
	} configs[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_step1,
		  ARRAY_SIZE(max96712_init_setting_step1), "max96712 step1" },

		{ sensor_info->serial_addr, max9295_init_setting,
		  ARRAY_SIZE(max9295_init_setting), "max9295" },

		{ sensor_info->sensor_addr, sensor_init_setting,
		  ARRAY_SIZE(sensor_init_setting), "sensor init" },

		{ deserial_if->deserial_addr, max96712_init_setting_step2,
		  ARRAY_SIZE(max96712_init_setting_step2), "max96712 step2" }
	};

	for (int i = 0; i < ARRAY_SIZE(configs); i++) {
		ret = write_register(bus, configs[i].dev_addr, REG16_VAL8,
							 configs[i].size, configs[i].data);
		if (ret < 0) {
			vin_err("Write %s register failed at address 0x%02X\n",
					configs[i].err_msg, configs[i].dev_addr);
			return ret;
		}
	}

	return RET_OK;
}


int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
							 sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
							  1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);
			}
		}
	}
	return ret;
}

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			vin_info("linear mode Reserved interface\n");
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	return ret;
}


int sensor_init(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size=0;

	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}

	/* 1.power on*/
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	if(sensor_info->sen_devfd <= 0) {
		char str[24] = {0};

		snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	vin_dbg("/dev/port_%d success sensor_info->sen_devfd %d===\n",
			 sensor_info->dev_port, sensor_info->sen_devfd);

	/* serdes init */
	ret = serdes_init(sensor_info);
	if (ret < 0) {
		vin_err("serdes_init fail\n");
		return ret;
	}

	/* 3. linear mode config */
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}

	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
  	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t)/2;
	vin_info("mipi_stream_on_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max96712_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s max96712 fail\n", sensor_info->sensor_name);
		return -1;
	}
	usleep(10*1000);
	setting_size = sizeof(sensor_stream_on_setting) / sizeof(uint32_t)/2;
	vin_info("sensor_stream_on_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sensor_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s sensor fail\n", sensor_info->sensor_name);
		return -1;
	}

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;

 	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	setting_size = sizeof(max96712_stream_off_setting) / sizeof(uint32_t)/2;

	vin_info("mipi_stream_off_setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max96712_stream_off_setting);
	if(ret < 0) {
		vin_err("stop %s max96712 fail\n", sensor_info->sensor_name);
		return -1;
	}
	usleep(10*1000);
	setting_size = sizeof(sensor_stream_off_setting) / sizeof(uint32_t)/2;
	vin_info("sensor_stream_off_setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sensor_stream_off_setting);
	if(ret < 0) {
		vin_err("start %s sensor fail\n", sensor_info->sensor_name);
		return -1;
	}
	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
							 sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return ret;
}

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}


#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(shw3g, CAM_MODULE_FLAG_A16D16);
sensor_module_t shw3g = {
	.module = SENSOR_MNAME(shw3g),
#else
sensor_module_t shw3g = {
	.module = "shw3g",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};



