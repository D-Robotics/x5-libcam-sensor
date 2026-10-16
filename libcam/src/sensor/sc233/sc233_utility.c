/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[sc233]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/sc233_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH	2	/* reg16 data8 */
#define SC233_VTS_HI (0x320E)
#define SC233_VTS_LO (0x320F)

/* 16'h3e03[3]=1: again/dgain independent control */
#define SC233_AGAIN_C (0x3e08)
#define SC233_AGAIN_F (0x3e09)
#define SC233_DGAIN_C (0x3e06)
#define SC233_DGAIN_F (0x3e07)

static int sc233_linear_data_init(sensor_info_t *sensor_info);

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if (sensor_info->gpio_num > 0) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWEROFF_FAIL;
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

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	vin_dbg("%s gpio_num = %d \n", sensor_info->sensor_name, sensor_info->gpio_num);
	if (sensor_info->gpio_num > 0) {
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			vin_dbg("%s gpio_pin[%d] = %d \n", sensor_info->sensor_name,
				gpio, sensor_info->gpio_pin[gpio]);
			if (sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
					1 - sensor_info->gpio_level[gpio]);
				if (ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);
			}
		}
	}

	return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			__LINE__, sensor_info->sensor_name);
		return ret;
	}

	switch (sensor_info->sensor_mode) {
	case NORMAL_M:
		vin_info("sc233 in normal/linear mode\n");
		vin_info("bus_num = %d, sensor_addr = 0x%0x, fps = %d\n",
			sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps);
		if (sensor_info->fps != 120) {
			vin_err("sc233 only support 120fps, got %d\n", sensor_info->fps);
			return -HB_CAM_INIT_FAIL;
		}
		/* native MIPI output 1920x1080@120fps (window crop from 1200) */
		setting_size = sizeof(sc233_24Minput_4lane_810Mbps_10bit_1920x1080_120fps_setting)
			/ sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			setting_size, sc233_24Minput_4lane_810Mbps_10bit_1920x1080_120fps_setting);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return -HB_CAM_I2C_WRITE_FAIL;
		}
		ret = sc233_linear_data_init(sensor_info);
		if (ret < 0) {
			vin_err("%d : linear data init %s fail\n",
				__LINE__, sensor_info->sensor_name);
			return -HB_CAM_INIT_FAIL;
		}
		break;
	default:
		vin_err("not support mode %d\n", sensor_info->sensor_mode);
		ret = -RET_ERROR;
		break;
	}
	vin_info("sc233 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(sc233_stream_on_setting) / sizeof(uint32_t) / 2;
	vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			setting_size, sc233_stream_on_setting);
	if (ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(sc233_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, sc233_stream_off_setting);
	if (ret < 0) {
		vin_err("stop %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0) {
		vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode,
	uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	char again_reg_value_h = 0, again_reg_value_l = 0;
	char dgain_reg_value_h = 0, dgain_reg_value_l = 0;
	int gain_index = 0;

	(void)dgain;
	(void)gain_num;

	if (mode == NORMAL_M || mode == SLAVE_M) {
		if (again[0] >= sizeof(sc233_gain_lut) / sizeof(uint32_t))
			gain_index = sizeof(sc233_gain_lut) / sizeof(uint32_t) - 1;
		else
			gain_index = again[0];

		dgain_reg_value_h = (sc233_gain_lut[gain_index] >> 24) & 0x000000FF;
		dgain_reg_value_l = (sc233_gain_lut[gain_index] >> 16) & 0x000000FF;
		again_reg_value_h = (sc233_gain_lut[gain_index] >> 8) & 0x000000FF;
		again_reg_value_l = (sc233_gain_lut[gain_index]) & 0x000000FF;

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_AGAIN_C, again_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_AGAIN_F, again_reg_value_l);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_DGAIN_C, dgain_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, SC233_DGAIN_F, dgain_reg_value_l);
	} else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode,
	uint32_t *line, uint32_t line_num)
{
	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	char temp0 = 0, temp1 = 0, temp2 = 0;

	(void)line_num;

	if (mode == NORMAL_M || mode == SLAVE_M) {
		uint32_t sline = line[0];

		/* VTS=1250 for 120fps, exposure max = VTS - 6 */
		if (sline > 1244)
			sline = 1244;
		if (sline < 1)
			sline = 1;

		temp0 = (sline >> 12) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline >> 4) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
		temp2 = (sline & 0x0F) << 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
	} else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sc233_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;
	int vts_hi, vts_lo;
	uint32_t vts;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));

	/* sensor native output 1920x1080 */
	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;

	vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, SC233_VTS_HI);
	vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, SC233_VTS_LO);
	vts = ((uint32_t)vts_hi << 8) | (uint32_t)vts_lo;
	vin_info("sc233: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);

	turning_data.sensor_data.conversion = 1;
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = vts * sensor_info->fps; /* 1250*120=150000 */
	turning_data.sensor_data.exposure_time_max = (vts > 6) ? (vts - 6) : 1;
	turning_data.sensor_data.exposure_time_long_max = turning_data.sensor_data.exposure_time_max;
	turning_data.sensor_data.analog_gain_max = 192 * 8192;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;

	sensor_data_bayer_fill(&turning_data.sensor_data, 10,
			(uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;

	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc233_stream_on_setting)) {
		memcpy(stream_on, sc233_stream_on_setting, sizeof(sc233_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc233_stream_off_setting)) {
		memcpy(stream_off, sc233_stream_off_setting, sizeof(sc233_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc233_gain_lut,
			sizeof(sc233_gain_lut));
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

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	(void)port;
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc233, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc233 = {
	.module = SENSOR_MNAME(sc233),
#else
sensor_module_t sc233 = {
	.module = "sc233",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.userspace_control = sensor_userspace_control,
};
