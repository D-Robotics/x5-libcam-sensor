/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[cv4006]:" fmt

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
#include "inc/cv4006_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH	2	//reg16 data8

//sensor register address.
#define  CV4006_REG_GAIN            0x3114   
#define  CV4006_REG_SHUTTER0_LOW    0x3048
#define  CV4006_REG_SHUTTER0_MID    0x3049
#define  CV4006_REG_SHUTTER0_HIGH   0x304A
#define  CV4006_REG_FRAME_LENGTH_LOW    0x301C  
#define  CV4006_REG_FRAME_LENGTH_MID    0x301D
#define  CV4006_REG_FRAME_LENGTH_HIGH   0x301E

//sensor value.
#define  CV4006_FRAME_LENGTH 0x954


static int cv4006_linear_data_init(sensor_info_t *sensor_info);
// static int cv4006_dol2_data_init(sensor_info_t *sensor_info);

static uint32_t cv4006_max(int32_t a, int32_t b)
{
    if (a > b) return a;
    else  return b;
}

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
			vin_info("cv4006 in normal/linear mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x, fps = %d, config_index = %d\n",
				sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps, sensor_info->config_index);
			if (sensor_info->fps == 30) {
				if(sensor_info->config_index == 0){ // 0: 2lane (default is 0)
					setting_size = sizeof(cv4006_init_1280x720_2lane_linear_setting) / sizeof(uint32_t) / 2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
						setting_size, cv4006_init_1280x720_2lane_linear_setting);
				}else{// 1: 4lane
				//	setting_size = sizeof(cv4006_init_3840x2160_4lane_linear_setting) / sizeof(uint32_t) / 2;
				//	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
				//		setting_size, cv4006_init_3840x2160_4lane_linear_setting);
				}

				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
				ret = cv4006_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
				}
			} else if (sensor_info->fps == 60) {
				setting_size = sizeof(cv4006_init_1280x720_2lane_linear_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, cv4006_init_1280x720_2lane_linear_setting);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
				ret = cv4006_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
				}
			}
			break;
		// case DOL2_M:
		// 	vin_info("cv4006 in dol2/hdr mode\n");
		// 	vin_info("bus_num = %d, sensor_addr = 0x%0x fps = %d\n",
		// 		sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps);

		// 	setting_size = sizeof(cv4006_init_3840x2160_dol2_setting) / sizeof(uint32_t) / 2;
		// 	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
		// 				    setting_size, cv4006_init_3840x2160_dol2_setting);
		// 	if (ret < 0) {
		// 		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		// 		return -HB_CAM_I2C_WRITE_FAIL;
		// 	}
		// 	ret = cv4006_dol2_data_init(sensor_info);
		// 	if (ret < 0) {
		// 		vin_err("%d : hdr data init %s fail\n", __LINE__, sensor_info->sensor_name);
		// 		return -HB_CAM_INIT_FAIL;
		// 	}
		// 	break;
		default:
			vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
			ret = -HB_CAM_INIT_FAIL;
			break;
	}
	vin_info("cv4006 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(cv4006_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			setting_size, cv4006_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
		return -HB_CAM_I2C_WRITE_FAIL;
	}
	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	// linear and hdr mode use one stream_off setting
	setting_size =
		sizeof(cv4006_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, cv4006_stream_off_setting);
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


static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
        printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
    const uint16_t AGAIN = CV4006_REG_GAIN;
	char again_reg_value = 0;
	int gain_index = 0;

        if (mode == NORMAL_M) {
	        if (again[0] >= sizeof(cv4006_gain_lut)/sizeof(uint32_t))
			gain_index = sizeof(cv4006_gain_lut)/sizeof(uint32_t) - 1;
		else
			gain_index = again[0];

		again_reg_value = (cv4006_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
                printf("%s, gain_index: %d, again:0x3114 = 0x%x\n",
				__FUNCTION__, gain_index, again_reg_value);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN, again_reg_value);
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
    const uint16_t EXP_LINE0 = CV4006_REG_SHUTTER0_LOW;
	const uint16_t EXP_LINE1 = CV4006_REG_SHUTTER0_MID;
	const uint16_t EXP_LINE2 = CV4006_REG_SHUTTER0_HIGH;
	char temp0 = 0, temp1 = 0, temp2 = 0;

        if (mode == NORMAL_M) {
			uint32_t tmp =  line[0];
			// uint32_t sline = CV4006_FRAME_LENGTH - (uint32_t)(tmp/1.2);
			uint32_t shutter0 = CV4006_FRAME_LENGTH - (uint32_t)(tmp);
			if ( shutter0 > CV4006_FRAME_LENGTH -1 ) {
				shutter0 = CV4006_FRAME_LENGTH -1;
			}
			if (shutter0 < 5) {
				shutter0 = 5;
			}

			temp2 = (shutter0 >> 16) & 0x0F;
			vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
			temp1 = (shutter0 >> 8) & 0xFF;
			vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
			temp0 = (shutter0 & 0xFF);
			vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);

	#ifdef AE_DBG
			printf("write shutter0 = %d, 0x3048 = 0x%x, 0x3049 = 0x%x,0x304a = 0x%x\n",
							shutter0, temp0, temp1,temp2);
	#endif
        } else {
			vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int cv4006_linear_data_init(sensor_info_t *sensor_info)
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

	turning_data.sensor_data.active_width = 1280;
	turning_data.sensor_data.active_height = 720;

	turning_data.sensor_data.lines_per_second = CV4006_FRAME_LENGTH * 60; //shaokc...
	turning_data.sensor_data.exposure_time_max = CV4006_FRAME_LENGTH - 5;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.analog_gain_max = 255;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.analog_gain_init = 32;
	turning_data.sensor_data.digital_gain_init = 0;
	turning_data.sensor_data.exposure_time_init = CV4006_FRAME_LENGTH - 5;


	//sensor bit && bayer
	//sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_GB, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(cv4006_stream_on_setting)) {
		memcpy(stream_on, cv4006_stream_on_setting, sizeof(cv4006_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(cv4006_stream_off_setting)) {
		memcpy(stream_off, cv4006_stream_off_setting, sizeof(cv4006_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, cv4006_gain_lut,
			sizeof(cv4006_gain_lut));
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

// static int cv4006_dol2_data_init(sensor_info_t *sensor_info)
// {
// 	int ret = RET_OK;
// 	uint32_t  open_cnt = 0;
// 	sensor_turning_data_t turning_data;
// 	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
// 	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

// 	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

// 	// common data
// 	turning_data.bus_num = sensor_info->bus_num;
// 	turning_data.bus_type = sensor_info->bus_type;
// 	turning_data.port = sensor_info->port;
// 	turning_data.reg_width = sensor_info->reg_width;
// 	turning_data.mode = sensor_info->sensor_mode;
// 	turning_data.sensor_addr = sensor_info->sensor_addr;
// 	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
// 		sizeof(turning_data.sensor_name));

// 	turning_data.sensor_data.active_width = 3840;
// 	turning_data.sensor_data.active_height = 2160;

// 	turning_data.sensor_data.lines_per_second = 81000;
// 	turning_data.sensor_data.exposure_time_max = 2430;
// 	turning_data.sensor_data.exposure_time_min = 8;
// 	turning_data.sensor_data.analog_gain_max = 255;
// 	turning_data.sensor_data.digital_gain_max = 0;


// 	//sensor bit && bayer
// 	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_GB, (uint32_t)BAYER_PATTERN_RGGB);
// 	// sensor exposure_max_bit, maybe not used ?  //FIXME
// 	sensor_data_bits_fill(&turning_data.sensor_data, 12);

// 	//some stress test case, we need kernel stream_ctrl.
// 	turning_data.stream_ctrl.data_length = 1;

// 	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(cv4006_stream_on_setting)) {
// 		memcpy(stream_on, cv4006_stream_on_setting, sizeof(cv4006_stream_on_setting));
// 	} else {
// 		vin_err("Number of registers on stream over 10\n");
// 		return -RET_ERROR;
// 	}
// 	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(cv4006_stream_off_setting)) {
// 		memcpy(stream_off, cv4006_stream_off_setting, sizeof(cv4006_stream_off_setting));
// 	} else {
// 		vin_err("Number of registers on stream over 10\n");
// 		return -RET_ERROR;
// 	}

// 	// sync gain lut to kernel driver.
// 	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
// 	if (turning_data.normal.again_lut != NULL) {
// 		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
// 		memcpy(turning_data.normal.again_lut, cv4006_gain_lut,
// 			sizeof(cv4006_gain_lut));
// 	}

// 	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

// 	if (turning_data.normal.again_lut) {
// 		free(turning_data.normal.again_lut);
// 		turning_data.normal.again_lut = NULL;
// 	}

// 	if (ret < 0) {
// 		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
// 		return -RET_ERROR;
// 	}

// 	return ret;
// }

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	// *enable = 0;	//cv4006 use kernel space gain contrl and line control
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(cv4006, CAM_MODULE_FLAG_A16D8);
sensor_module_t cv4006 = {
	.module = SENSOR_MNAME(cv4006),
#else
sensor_module_t cv4006 = {
	.module = "cv4006",
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
