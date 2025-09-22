/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc850sl]:" fmt

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
#include "inc/sc850sl_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

int sc850sl_linear_data_init(sensor_info_t *sensor_info);


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

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			__LINE__, sensor_info->sensor_name);
		return ret;  //-HB_CAM_SENSOR_POWERON_FAIL
	}

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  // 1: normal
			vin_info("sc8505 in normal linear mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);

			setting_size = sizeof(sc850sl_linear_30fps_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							setting_size, sc850sl_linear_30fps_init_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			ret = sc850sl_linear_data_init(sensor_info);
			if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
			}
			break;

		default:
			vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
			ret = -HB_CAM_INIT_FAIL;
			break;
	}
	vin_info("sc8505 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
		case SLAVE_M:
			setting_size = sizeof(sc850sl_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info("%s start normal / slave linear mode\n", sensor_info->sensor_name);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, sc850sl_stream_on_setting);
			if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			break;
		case DOL2_M:
		default:
			vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
			ret = -HB_CAM_START_FAIL;
			break;
	}
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size =
		sizeof(sc850sl_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, 2,
			setting_size, sc850sl_stream_off_setting);
	if (ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}

	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0) {
		vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret; //-HB_CAM_SENSOR_POWEROFF_FAIL
	}
	return ret;
}

int sc850sl_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	uint16_t VTS_HIGH;
	uint16_t VTS_LOW;
	uint32_t VTS_VALUE;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = NORMAL_M;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = 3840;
	turning_data.sensor_data.active_height = 2160;

	//read vts = frame_length = HMAX, default value is 0x0465 = 1125
	VTS_HIGH = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320e);
	VTS_HIGH = VTS_HIGH & 0x7f; //0x320e[0:6]
	VTS_LOW = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320f);
	VTS_VALUE = (VTS_HIGH << 8 | VTS_LOW);
#ifdef AE_DBG
	printf("%s read VTS_HIGH = 0x%x, VTS_LOW = 0x%x, VTS = 0x%x \n",
		__FUNCTION__, VTS_HIGH, VTS_LOW, VTS_VALUE);
#endif
	turning_data.sensor_data.lines_per_second = VTS_VALUE * sensor_info->fps;//67500vts * fps, should be fixed = 1125 * 30
	vin_info("%s set lines_per_second = %d\n",
		sensor_info->sensor_name, turning_data.sensor_data.lines_per_second);
	//lines_per_second/fps = vts
	// from customer, max 10ms
	// 1000ms -lines_per_second - 33750
	// 10ms - 337, 33ms - 1125
	turning_data.sensor_data.exposure_time_max = 675; //from customer, max 10ms

	turning_data.sensor_data.exposure_time_long_max = 2 * VTS_VALUE - 8;  //2*frame_length - 8  //linear not use
	turning_data.sensor_data.analog_gain_max = 205; //we use again + dig fine gain
	turning_data.sensor_data.digital_gain_max = 255;
	turning_data.sensor_data.exposure_time_min = 1;

	turning_data.sensor_data.exposure_time_step = 2;	//hdr exposure_time_step, from spec

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	/* line and gain userspace control donnot need set those params */
#if 0
	turning_data.sensor_data.turning_type = 6;  //FIXME should be tuning_type ???
	turning_data.sensor_data.conversion = 1;
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 968;
#endif
	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc850sl_stream_on_setting)) {
		memcpy(stream_on, sc850sl_stream_on_setting, sizeof(sc850sl_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc850sl_stream_off_setting)) {
		memcpy(stream_off, sc850sl_stream_off_setting, sizeof(sc850sl_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc850sl_gain_lut,
			sizeof(sc850sl_gain_lut));
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



static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	//vin_info("%s %s mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
	const uint16_t AGAIN_LOW = 0x3e08;	//for linear or dol2 long frame
	const uint16_t AGAIN_HIGH = 0x3e09;
	const uint16_t DGAIN_LOW = 0x3e06;
	const uint16_t DGAIN_HIGH = 0x3e07;
	const uint16_t S_AGAIN_LOW = 0x3e12;	//for dol2 short frame
	const uint16_t S_AGAIN_HIGH = 0x3e13;
	const uint16_t S_DGAIN_LOW = 0x3e10;
	const uint16_t S_DGAIN_HIGH = 0x3e11;
	char lower_again_reg_value = 0, high_again_reg_value = 0;
	char lower_dgain_reg_value = 0, high_dgain_reg_value = 0;
	char s_lower_again_reg_value = 0, s_high_again_reg_value = 0;
	char s_lower_dgain_reg_value = 0, s_high_dgain_reg_value = 0;

	int again_index = 0, dgain_index = 0;
	int s_again_index = 0, s_dgain_index = 0;
	if (mode == NORMAL_M) {
		if (again[0] >= sizeof(sc850sl_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(sc850sl_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(sc850sl_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(sc850sl_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		lower_again_reg_value = sc850sl_gain_lut[again_index] & 0x000000FF;
		high_again_reg_value = (sc850sl_gain_lut[again_index] >> 8) & 0x000000FF;
		lower_dgain_reg_value = sc850sl_dgain_lut[dgain_index] & 0x000000FF;
		high_dgain_reg_value = (sc850sl_dgain_lut[dgain_index] >> 8) & 0x000000FF;
		//vin_info("%s again(0x3e08/0x3e09):%x,%x; dgain(0x3e06x3e07):%x,%x\n",
		//		__FUNCTION__, lower_again_reg_value, high_again_reg_value, lower_dgain_reg_value,
		//		high_dgain_reg_value);

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_LOW, lower_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_HIGH, high_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_LOW, lower_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_HIGH, high_dgain_reg_value);
	} else if (mode == DOL2_M){
		if (again[0] >= sizeof(sc850sl_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(sc850sl_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (again[1] >= sizeof(sc850sl_gain_lut)/sizeof(uint32_t))
			s_again_index = sizeof(sc850sl_gain_lut)/sizeof(uint32_t) - 1;
		else
			s_again_index = again[1];

		if (dgain[0] >= sizeof(sc850sl_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(sc850sl_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		if (dgain[1] >= sizeof(sc850sl_dgain_lut)/sizeof(uint32_t))
			s_dgain_index = sizeof(sc850sl_dgain_lut)/sizeof(uint32_t) - 1;
		else
			s_dgain_index = dgain[1];


		lower_again_reg_value = sc850sl_gain_lut[again_index] & 0x000000FF;
		high_again_reg_value = (sc850sl_gain_lut[again_index] >> 8) & 0x000000FF;
		lower_dgain_reg_value = sc850sl_dgain_lut[dgain_index] & 0x000000FF;
		high_dgain_reg_value = (sc850sl_dgain_lut[dgain_index] >> 8) & 0x000000FF;

		s_lower_again_reg_value = sc850sl_gain_lut[s_again_index] & 0x000000FF;
		s_high_again_reg_value = (sc850sl_gain_lut[s_again_index] >> 8) & 0x000000FF;
		s_lower_dgain_reg_value = sc850sl_dgain_lut[s_dgain_index] & 0x000000FF;
		s_high_dgain_reg_value = (sc850sl_dgain_lut[s_dgain_index] >> 8) & 0x000000FF;

		//vin_info("%s again_index = %d, dgain_index = %d, s_again_index = %d, s_dgain_index = %d \n",
		//	__FUNCTION__, again_index, dgain_index, s_again_index, s_dgain_index);

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_LOW, lower_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_HIGH, high_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_LOW, lower_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_HIGH, high_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_AGAIN_LOW, s_lower_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_AGAIN_HIGH, s_high_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_DGAIN_LOW, s_lower_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_DGAIN_HIGH, s_high_dgain_reg_value);

	} else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
	//vin_info(" line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	const uint16_t S_EXP_LINE0 = 0x3e04;
	const uint16_t S_EXP_LINE1 = 0x3e05;
	char temp0 = 0, temp1 = 0, temp2 = 0;


		if (mode == NORMAL_M) {
		uint32_t sline = line[0];
		if ( sline > 1046){
			sline = 1046;
		}

		temp0 = (sline & 0xF000) >> 12;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline & 0xFF0) >> 4;
				vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
				temp2 = (sline & 0x0F) << 4;
				vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
	} else if (mode == DOL2_M) {
		//printf(" line mode %d, line0 %u , line1: %u, line_num:%d \n", mode, line[0], line[1], line_num);
		uint32_t lline = line[0];	//long frame  20ms
		if ( lline > 1500) {
			lline = 1500;
		}
		temp0 = (lline & 0xF000) >> 12;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (lline & 0xFF0) >> 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
		temp2 = (lline & 0x0F) << 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);

		uint32_t sline = line[1];	//short frame 7.17ms
		if ( sline > 538) {
			sline = 538;
		}
		temp0 = (sline & 0xFF0) >> 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_EXP_LINE0, temp0);
		temp1 = (sline & 0x0F) << 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, S_EXP_LINE1, temp1);
	} else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	//*enable = 0;
	return 0;
}


#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc850sl, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc850sl = {
	.module = SENSOR_MNAME(sc850sl),
#else
sensor_module_t sc850sl = {
	.module = "sc850sl",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.aexp_gain_control = sensor_aexp_gain_control,
	.aexp_line_control = sensor_aexp_line_control,
	.userspace_control = sensor_userspace_control,
};
