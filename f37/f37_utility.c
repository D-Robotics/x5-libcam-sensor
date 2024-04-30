/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[f37]:" fmt

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
#include "inc/f37_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH	1	//8 bit
#define F37_PROGRAM_GAIN    (0x00)
#define F37_EXP_LINE	    (0x01)
#define F37_DOL2_SHORT_EXP_LINE     (0x05)


static int f37_linear_data_init(sensor_info_t *sensor_info);
static int f37_dol2_data_init(sensor_info_t *sensor_info);

static int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWEROFF_FAIL;
				}
			}
		}
	}

	return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	vin_dbg("%s gpio_num = %d \n", sensor_info->sensor_name, sensor_info->gpio_num);
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			vin_dbg("%s gpio_pin[%d] = %d \n", sensor_info->sensor_name, gpio, sensor_info->gpio_pin[gpio]);
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
					sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
					1 - sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);  //100ms
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
		return ret;  //-HB_CAM_SENSOR_POWERON_FAIL
	}

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:	  // 1: normal
			vin_info("f37 in normal/linear mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);

			setting_size = sizeof(f37_linear_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
						    setting_size, f37_linear_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			ret = f37_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_INIT_FAIL;
			}
			break;
		case DOL2_M:
			vin_info("f37 in dol2/hdr mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);

			setting_size = sizeof(f37_HDR_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
						    setting_size, f37_linear_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			ret = f37_dol2_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : hdr data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_INIT_FAIL;
			}
			break;
		default:
			vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
			ret = -HB_CAM_INIT_FAIL;
			break;
	}
	vin_info("f37 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			setting_size = sizeof(f37_linear_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, f37_linear_stream_on_setting);
			if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			break;
		case DOL2_M:
			setting_size = sizeof(f37_hdr_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info("%s start dol2/hdr mode\n", sensor_info->sensor_name);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, f37_hdr_stream_on_setting);
			if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			break;
		default:
			vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
			ret = -HB_CAM_START_FAIL;
			break;
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	// linear and hdr mode use one stream_off setting
	setting_size =
		sizeof(f37_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, f37_stream_off_setting);
	if (ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
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
		return ret; //-HB_CAM_SENSOR_POWEROFF_FAIL
	}
	return ret;
}

static int f37_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;

	turning_data.sensor_data.lines_per_second = 33783;
	turning_data.sensor_data.exposure_time_max = 1088;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 4000;	//linear not use
	turning_data.sensor_data.analog_gain_max = 255;
	turning_data.sensor_data.digital_gain_max = 0;

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//this sensor will set line and gain in kernel driver.
	//we need set those value

	/* input_line <= max
	 * out_line = offset + ((ratio * input_line) >> 8);
	 * */
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 1088;

	turning_data.normal.s_line = F37_EXP_LINE;	//f37 write line i2c reg addr: 0x01
	/* in kernel driver, we will use 2 output value, so we need s_line_length=2
	 *	output[0] = (uint8_t)(input[0] & 0xffu);
		output[1] = (uint8_t)((input[0] >> 8) & 0xffu);
		output[2] = (uint8_t)((input[0] >> 16) & 0x03u);
	 * */
	turning_data.normal.s_line_length = 2;		//f37 write line i2c count = 1(reg_addr) + length
	turning_data.normal.again_control_num = 1;	//linear mode, one contrl num
	turning_data.normal.again_control[0] = F37_PROGRAM_GAIN;	//f37 write gain i2c reg addr: 0x00
	turning_data.normal.again_control_length[0] = 1;	// f37 gain value is small, gain only need 1 output value

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_linear_stream_on_setting)) {
		memcpy(stream_on, f37_linear_stream_on_setting, sizeof(f37_linear_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(f37_stream_off_setting)) {
		memcpy(stream_off, f37_stream_off_setting, sizeof(f37_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, f37_gain_lut,
			sizeof(f37_gain_lut));
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

static int f37_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;

	//FIXME
	turning_data.sensor_data.lines_per_second = 33783;
	turning_data.sensor_data.exposure_time_max = 1088;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 4000;	//linear not use
	turning_data.sensor_data.analog_gain_max = 255;
	turning_data.sensor_data.digital_gain_max = 0;

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(f37_linear_stream_on_setting)) {
		memcpy(stream_on, f37_linear_stream_on_setting, sizeof(f37_linear_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(f37_stream_off_setting)) {
		memcpy(stream_off, f37_stream_off_setting, sizeof(f37_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, f37_gain_lut,
			sizeof(f37_gain_lut));
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
	vin_info("enable userspace gain control and line control\n");
	*enable = 0;	//f37 use kernel space gain contrl and line control
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(f37, CAM_MODULE_FLAG_A8D8);
sensor_module_t f37 = {
	.module = SENSOR_MNAME(f37),
#else
sensor_module_t f37 = {
	.module = "f37",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.userspace_control = sensor_userspace_control,
};
