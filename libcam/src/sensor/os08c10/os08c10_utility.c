/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2024 D-Robotics.
 * All rights reserved.
 ***************************************************************************/
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
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "inc/os08c10_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "hb_camera_data_config.h"

#define OS08C10_AGAIN_HIGH_BYTE  0x3508
#define OS08C10_AGAIN_LOW_BYTE  0x3509
#define OS08C10_VTS_HI  0x380e
#define OS08C10_VTS_LO  0x380f
#define OS08C10_EXP_HIGH_BYTE  0x3501
#define OS08C10_EXP_LOW_BYTE  0x3502

#define USE_DAG_HDR 1

static int os08c10_linear_data_init(sensor_info_t *sensor_info);
int sensor_poweron(sensor_info_t *sensor_info)
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

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	int group_id = 0;
	uint8_t val = 0;
	int i;

	pr_debug("os08c10 sensor_init \n");
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
			pr_err("%d : sensor reset %s fail\n",
						__LINE__, sensor_info->sensor_name);
			return ret;
	}

	// set resolution and format
	if (sensor_info->resolution == 2160) {
			pr_debug("os08c10 resolution is 2160 \n");
#if USE_DAG_HDR
			setting_size =
					sizeof(os08c10_3840x2160_30fps_27MHz_linear_12bit_1620Mbps_2lane) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num,
									sensor_info->sensor_addr, 2,
									setting_size, os08c10_3840x2160_30fps_27MHz_linear_12bit_1620Mbps_2lane);
#else
			setting_size =
					sizeof(os08c10_3840x2160_30fps_27MHz_linear_12bit_1701Mbps_2lane) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num,
									sensor_info->sensor_addr, 2,
									setting_size, os08c10_3840x2160_30fps_27MHz_linear_12bit_1701Mbps_2lane);
#endif
			if (ret < 0) {
				pr_err("%d : init %s fail\n",
						__LINE__, sensor_info->sensor_name);
				return ret;
			}
	} else {
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}

	ret = os08c10_linear_data_init(sensor_info);
	if (ret < 0) {
		pr_err("%d : turning data init %s fail\n",
							__LINE__, sensor_info->sensor_name);
		return ret;
	}

return ret;
}

// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	int group_id = 0;

	pr_debug("os08c10 sensor start\n");
	setting_size = sizeof(os08c10_2lane_stream_on_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, os08c10_2lane_stream_on_setting);
	if (ret < 0) {
		pr_err("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	printf("os08c10 sensor stop \n");
	setting_size = sizeof(os08c10_2lane_stream_off_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, os08c10_2lane_stream_off_setting);
	if (ret < 0) {
		pr_err("stop %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->gpio_num > 0) {
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

	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0)
	{
			pr_err("%d : deinit %s fail\n",
						__LINE__, sensor_info->sensor_name);
			return ret;
	}
	return ret;
}


void os08c10_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
					sizeof(turning_data->sensor_name));
	return;
}

void os08c10_normal_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;
	// turning sensor_data
	int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OS08C10_VTS_HI);
	int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OS08C10_VTS_LO);
	uint32_t vts = vts_hi;
	vts = vts << 8 | vts_lo;
	printf("vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi,vts_lo, vts);//0x90a
	turning_data->sensor_data.conversion = 1;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.lines_per_second = vts * sensor_info->fps;//2314*30=69420
	turning_data->sensor_data.exposure_time_max = vts;
	printf("exposure_time_max: 0x%x\n",turning_data->sensor_data.exposure_time_max);
	turning_data->sensor_data.exposure_time_long_max = vts;
	turning_data->sensor_data.analog_gain_max = 191;
	turning_data->sensor_data.digital_gain_max = 0;
	turning_data->sensor_data.exposure_time_min = 1;
}

// turning data init
static int os08c10_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	os08c10_common_data_init(sensor_info, &turning_data);
	os08c10_normal_data_init(sensor_info, &turning_data);

	sensor_data_bayer_fill(&turning_data.sensor_data, 12, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	// setting stream ctrl
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(os08c10_2lane_stream_on_setting)) {
			memcpy(stream_on, os08c10_2lane_stream_on_setting, sizeof(os08c10_2lane_stream_on_setting));
	} else {
			pr_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
	}

	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(os08c10_2lane_stream_off_setting)) {
			memcpy(stream_off, os08c10_2lane_stream_off_setting, sizeof(os08c10_2lane_stream_off_setting));
	}
	else {
			pr_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
	}
	// look-up table
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
			memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
			memcpy(turning_data.normal.again_lut, os08c10_gain_lut,
						sizeof(os08c10_gain_lut));
	}
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
			free(turning_data.normal.again_lut);
			turning_data.normal.again_lut = NULL;
	}
	if (ret < 0) {
			pr_err("sensor_%s ioctl fail %d\n", sensor_info->sensor_name, ret);
			return -RET_ERROR;
	}

	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
	printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	/*
	Customer Requirements: 
	The gain needs to be set to both high gain and low gain. 
	The range for high gain is 8x-16x, and the range for low gain is 1x-2x, 
	with the condition that high gain / low gain = 8.
	*/
	//short frame
	const uint16_t AGAIN_H = 0x3548;//again
	const uint16_t AGAIN_L = 0x3549;
	//long frame
	const uint16_t DAG_AGAIN_H = 0x3508;
	const uint16_t DAG_AGAIN_L = 0x3509;

	char again_reg_value_h = 0,again_reg_value_l = 0;
	int gain_index = 0;

	if (mode == NORMAL_M) {
		if (again[0] >= sizeof(os08c10_gain_lut)/sizeof(uint32_t))
			gain_index = sizeof(os08c10_gain_lut)/sizeof(uint32_t) - 1;
		else
			gain_index = again[0];
#ifdef AE_DBG
		printf("%s, gain_index: %d, again_h:0x3548 = 0x%x, again_l:0x3549 = 0x%x\n",
		__FUNCTION__, gain_index, again_reg_value_h,again_reg_value_l);
#endif

#if USE_DAG_HDR
		//限制1x-2x
		if(gain_index < 0){
			gain_index = 0;
		}
		else if(gain_index > 32){
			gain_index = 32;
		}else{

		}
		//high gain
		again_reg_value_h = (os08c10_gain_lut[gain_index] >> 8) & 0x7F;
		again_reg_value_l = (os08c10_gain_lut[gain_index]) & 0xFE;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_H, again_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_L, again_reg_value_l);

		//low gain
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DAG_AGAIN_H, ((again_reg_value_h<<3)+(again_reg_value_l>>5)));
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DAG_AGAIN_L, (again_reg_value_l<<3));

#else
		again_reg_value_h = (os08c10_gain_lut[gain_index] >> 8) & 0x7F;
		again_reg_value_l = (os08c10_gain_lut[gain_index]) & 0xFE;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_H, again_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_L, again_reg_value_l);
#endif
	} else	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

/* input value:
 * line: exposure time value
 * line_num: linear mode: 1; dol2 mode: 2
 * */

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	printf("line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
	const uint16_t EXP_LINE0 = 0x3500;
	const uint16_t EXP_LINE1 = 0x3501;
	const uint16_t EXP_LINE2 = 0x3502;

	char temp0 = 0, temp1 = 0, temp2 = 0;
	if (mode == NORMAL_M) {
		uint32_t sline =  line[0];
		if ( sline > 2314) {
			sline = 2314;
		}
		if (sline < 8) {
			sline = 8;
		}

		temp0 = (sline >> 16) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
#if USE_DAG_HDR
		const uint16_t DAG_EXP_LINE0 = 0x3540;//Always set 3540 = 3500
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DAG_EXP_LINE0, temp0);
#endif
		temp1 = (sline >> 8) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
#if USE_DAG_HDR
		const uint16_t DAG_EXP_LINE1 = 0x3541;//Always set 3541 = 3501
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DAG_EXP_LINE1, temp1);
#endif
		temp2 = (sline) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
#if USE_DAG_HDR
		const uint16_t DAG_EXP_LINE2 = 0x3542;//Always set 3542 = 3502
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DAG_EXP_LINE2, temp2);
#endif

#ifdef AE_DBG
		printf("write sline = %d, 0x3500 = 0x%x, 0x3501 = 0x%x,0x3502 = 0x%x\n",
						sline, temp0, temp1,temp2);
#endif
		} else {
			vin_err(" unsupport mode %d\n", mode);
		}

	return 0;
}


static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL| HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(os08c10, CAM_MODULE_FLAG_A16D8);
sensor_module_t os08c10 = {
	.module = SENSOR_MNAME(os08c10),
#else
sensor_module_t os08c10 = {
	.module = "os08c10",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.aexp_line_control = sensor_aexp_line_control,
	.aexp_gain_control = sensor_aexp_gain_control,
	.userspace_control = sensor_userspace_control,
};
