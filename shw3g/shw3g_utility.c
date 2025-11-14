/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[shw3g]:" fmt

#define SENSOR_ADDRESS 		(0x36)
#define AE_DBG 				1

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

void shw3g_commmon_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	// common data
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
			sizeof(turning_data->sensor_name));
}

void shw3g_param_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{

	// int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX219_FRM_LENGTH_HI);
	// int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX219_FRM_LENGTH_LO);
	// uint32_t vts = vts_hi;
	// vts = vts << 8 | vts_lo;
	// pr_info("IMX219: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);
	uint32_t vts = 2877;

	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;
	// turning sensor_data
	turning_data->sensor_data.conversion = 1;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.lines_per_second = vts * sensor_info->fps;
	turning_data->sensor_data.exposure_time_max = vts;
	turning_data->sensor_data.exposure_time_long_max = vts;
	turning_data->sensor_data.analog_gain_max = 255;
	turning_data->sensor_data.digital_gain_max = 0;
	turning_data->sensor_data.exposure_time_min = 1;
}


static int shw3g_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	shw3g_commmon_data_init(sensor_info,&turning_data);
	shw3g_param_data_init(sensor_info,&turning_data);

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 12, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sensor_stream_on_setting)) {
		memcpy(stream_on, sensor_stream_on_setting, sizeof(sensor_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sensor_stream_off_setting)) {
		memcpy(stream_off, sensor_stream_off_setting, sizeof(sensor_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, shw3g_gain_lut,
			sizeof(shw3g_gain_lut));
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}

	if (ret < 0) {
		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}

	return ret;
}


int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			shw3g_linear_data_init(sensor_info);
			vin_info("linear master mode Reserved interface\n");
			break;
		case SLAVE_M:
			shw3g_linear_data_init(sensor_info);
			vin_info("linear slave mode Reserved interface\n");
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

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
		printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif

	const uint16_t AGAIN_L = 0x3514;
		const uint16_t AGAIN_H = 0x3515;
	char again_reg_value_L = 0;
	char again_reg_value_H = 0;
	int gain_index = 0;

		if (mode == NORMAL_M || mode == SLAVE_M) {
			if (again[0] >= shw3g_gain_lut[sizeof(shw3g_gain_lut)/sizeof(uint32_t)-1])
			gain_index = shw3g_gain_lut[sizeof(shw3g_gain_lut)/sizeof(uint32_t) - 1];
		else
			gain_index = again[0];

		again_reg_value_L = shw3g_gain_lut[gain_index] & 0xFF;
		again_reg_value_H = (shw3g_gain_lut[gain_index] >> 8) & 0x01;
#ifdef AE_DBG
				printf("%s, gain_index: %d, again_l: 0x3514 = 0x%x, again_h: 0x3515 = 0x%x\n",
				__FUNCTION__, gain_index, again_reg_value_L, again_reg_value_H);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_L, again_reg_value_L);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_H, again_reg_value_H);
	} else	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
		printf("%s, line mode %d,   line[0]: %d, line_num: %d \n", __func__, mode,  line[0], line_num);
#endif

		char temp0 = 0, temp1 = 0, temp2 = 0;
		uint32_t shs;
		uint32_t val = line[0];

		if (mode == NORMAL_M || mode == SLAVE_M) {
				shs = IMX900_EXPOSURE_LINE - val ;

				if (shs > IMX900_MAX_SHS)
						shs = IMX900_MAX_SHS;
				else if (shs < IMX900_MIN_SHS)
						shs = IMX900_MIN_SHS;

				temp0 = shs & 0xff;
				vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3240, temp0);
				temp1 = (shs >> 8) & 0xFF;
				vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3241, temp1);
				temp2 = (shs >> 16) & 0xFF;
				vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3242, temp2);

#ifdef AE_DBG
				printf("%s, write sline = %d, 0x3240 = 0x%x, 0x3241 = 0x%x, 0x3242 = 0x%x \n",
								__func__, shs, temp0, temp1, temp2);
#endif
		} else {
				vin_err(" unsupport mode %d\n", mode);
		}

		return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	// *enable = 0;	//imx415 use kernel space gain contrl and line control
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
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
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.userspace_control = sensor_userspace_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};



