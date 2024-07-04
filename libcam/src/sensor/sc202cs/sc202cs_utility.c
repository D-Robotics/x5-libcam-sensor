/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc202cs]:" fmt

//#define AE_DBG 1
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
#include "inc/sc202cs_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define MCLK (24000000)



#define SC202CS_PROGRAM_GAIN	(0x3e09)
#define SC202CS_DIGITAL_GAIN	(0x3e06)
#define SC202CS_EXP_LINE		(0x3e00)
#define SC202CS_DOL2_SHORT_EXP_LINE		(0x3e04)

// turning data init
int sc202cs_linear_data_init_1600x1200(sensor_info_t *sensor_info)
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
	// turning sensor_data
	// lines_per_second = fps * vts, vts = {16‘h320e,16’h320f} = 1400
	// If trigger is enabled, the configuration before trigger will still be used.
	turning_data.sensor_data.lines_per_second = 37500;
	// form customer, exposure time max = 10ms
	turning_data.sensor_data.exposure_time_max = 1125;

	turning_data.sensor_data.active_width = 1600;
	turning_data.sensor_data.active_height = 1200;

	turning_data.sensor_data.analog_gain_max = 190;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;

	// No setting is required in linear mode
	turning_data.sensor_data.exposure_time_long_max = 4000;

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc202cs_stream_on_setting)) {
		memcpy(stream_on, sc202cs_stream_on_setting, sizeof(sc202cs_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc202cs_stream_off_setting)) {
		memcpy(stream_off, sc202cs_stream_off_setting, sizeof(sc202cs_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc202cs_gain_lut,
			sizeof(sc202cs_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc202cs_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut) {
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
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

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio]);
				usleep(1 * 100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
						1 - sensor_info->gpio_level[gpio]);
				usleep(1 * 100 * 1000);  //100ms
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio]);
				usleep(1 * 100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
						1 - sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);  //100ms
			}
		}
	}

	return ret;
}

static int sensor_configure(sensor_info_t *sensor_info, const uint32_t *setting, int setting_size)
{
	int ret = RET_OK;

	vin_info("sensor %s enable fps: %d, setting_size = %d, bus_num = %d, sensor_addr = 0x%0x\n",
		sensor_info->sensor_name, sensor_info->fps, setting_size,
		sensor_info->bus_num, sensor_info->sensor_addr);

	return vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size, setting);
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}

	if (sensor_info->width == 1600 && sensor_info->height == 1200) {
		switch(sensor_info->sensor_mode) {
		case NORMAL_M:	  // 1: normal
			vin_info("sc202cs in normal mode\n");
			setting_size = sizeof(sc202cs_linear_init_1600x1200_30fps_setting) / sizeof(uint32_t) / 2;
			ret = sensor_configure(sensor_info, sc202cs_linear_init_1600x1200_30fps_setting, setting_size);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc202cs_linear_data_init_1600x1200(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
		}
	}
	vin_info("sc202cs config success under %d mode\n\n", sensor_info->sensor_mode);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
	case NORMAL_M:
		setting_size = sizeof(sc202cs_stream_on_setting)/sizeof(uint32_t)/2;
		vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, sc202cs_stream_on_setting);
		if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return ret;
		}
		break;
	case DOL2_M:
		setting_size = sizeof(sc202cs_stream_on_setting)/sizeof(uint32_t)/2;
		vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, sc202cs_stream_on_setting);
		if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return ret;
		}
		break;
	}

	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

		setting_size =
				sizeof(sc202cs_stream_off_setting) / sizeof(uint32_t) / 2;
		vin_info("sensor stop sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, sc202cs_stream_off_setting);
		if (ret < 0)
		{
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return ret;
		}

		return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0)
	{
		vin_err("%d : deinit %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
        printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
    const uint16_t AGAIN = 0x3e09;
	const uint16_t DGAIN = 0x3e06;
	const uint16_t DFINE_GAIN = 0x3e07;
	char again_reg_value = 0;
	char dgain_reg_value = 0, d_fine_gain_reg_value = 0;
	int gain_index = 0;

        if (mode == NORMAL_M) {
	        if (again[0] >= sizeof(sc202cs_gain_lut)/sizeof(uint32_t))
			gain_index = sizeof(sc202cs_gain_lut)/sizeof(uint32_t) - 1;
		else
			gain_index = again[0];

		again_reg_value = (sc202cs_gain_lut[gain_index] >> 16) & 0x000000FF;
		dgain_reg_value = (sc202cs_gain_lut[gain_index] >> 8) & 0x000000FF;
		d_fine_gain_reg_value = sc202cs_gain_lut[gain_index] & 0x000000FF;
#ifdef AE_DBG
                printf("%s, gain_index: %d, 0x3e09 = 0x%x dgain: 0x3e06 = 0x%x dig fine gain: 0x3e07 = 0x%x\n",
				__FUNCTION__, gain_index, again_reg_value, dgain_reg_value, d_fine_gain_reg_value);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN, again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN, dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DFINE_GAIN, d_fine_gain_reg_value);
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
    const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	char temp0 = 0, temp1 = 0, temp2 = 0;

        if (mode == NORMAL_M) {
		uint32_t sline =  line[0];
                /*
                 * NOTICE: sensor exposure half line, so sline = 2 * line(from isp)
                 * exposure line max, from customer:
                 * exposure_time_max = 10ms, line = 337, result = 337 * 2 = 674
                 * form spec:
                 * exposure_time_max = 2 * VTS - 8, 10fps, result = 11250 * 2 - 8
                 * so, we should limit sline = 674
                 */
		if ( sline > 1792) {
			sline = 1792;
		}

		temp0 = (sline >> 12) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline >> 4) & 0xFF;
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
        temp2 = (sline & 0x0F) << 4;
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
#ifdef AE_DBG
        printf("write sline = %d, 0x3e00 = 0x%x, 0x3e01 = 0x%x, 0x3e02 = 0x%x \n",
                        sline, temp0, temp1, temp2);
#endif

        } else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}
static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc202cs, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc202cs = {
		.module = SENSOR_MNAME(sc202cs),
#else
sensor_module_t sc202cs = {
		.module = "sc202cs",
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
