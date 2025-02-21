/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc1336]:" fmt

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
#include "inc/sc1336_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"
#include "../serial/max_serial.h"
#include <math.h>

#define MCLK (24000000)

#define S1336_PROGRAM_GAIN	(0x3e08)
#define S1336_DIGITAL_GAIN	(0x3e06)
#define S1336_EXP_LINE		(0x3e00)
#define SC1336_DOL2_SHORT_EXP_LINE		(0x3e04)


static int power_ref;

emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SC1336] = {
		.serial_addr = 0x00,		// serial i2c addr, dummy
		.sensor_addr = 0x30,		// sensor i2c addr
		.eeprom_addr = 0x00,		// eeprom i2c addr, dummy
		.serial_rclk_out = 0,		// 0: reserved
		.rclk_mfp = 0,                // 0: reserved
	},
	[SC1336] = {
		.serial_addr = 0x00,		// serial i2c addr, dummy
		.sensor_addr = 0x30,		// sensor i2c addr
		.eeprom_addr = 0x00,		// eeprom i2c addr, dummy
		.serial_rclk_out = 0,		// 0: reserved
		.rclk_mfp = 0,                // 0: reserved
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SC1336, "0.0.1", "sc1336", "0.1.0.0", &emode_data[SC1336]),
	SENSOR_EMEND(),
};

int sc1336_linear_data_init(sensor_info_t *sensor_info);

int sensor_poweroff(sensor_info_t *sensor_info)
{
    int gpio, ret = RET_OK;

	if(sensor_info->gpio_pin[0] == -1) {
		vin_err("SC1336 power_off GPIO control not enabled.\n");
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}
	usleep(10 * 1000);
	//power down
	ret = vin_power_ctrl(sensor_info->gpio_pin[0], 0);
	if(ret < 0) {
		vin_err("SC1336 vin_poweroff_down_ctrl fail\n");
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}
	return ret;
}

int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->gpio_pin[0] == -1) {
		vin_err("SC1336 power_on GPIO control not enabled.\n");
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}

	ret = vin_power_ctrl(sensor_info->gpio_pin[0], 0);
	if(ret < 0) {
		vin_err("SC1336 vin_poweron_down_ctrl fail\n");
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}
	usleep(10 * 1000);
	// Wrokoing
	ret = vin_power_ctrl(sensor_info->gpio_pin[0], 1);
	if(ret < 0) {
		vin_err("SC1336 vin_power_up_ctrl fail\n");
		return -HB_CAM_SENSOR_POWEROFF_FAIL;
	}

    return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	const uint32_t *sc1336_linear_init_setting;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}
        switch(sensor_info->sensor_mode) {
		case NORMAL_M:// 1: normal
			vin_err("sc1336 in normal mode\n");
			if (sensor_info->fps == 15) {
				sc1336_linear_init_setting = sc1336_linear_init_15fps_setting;
				setting_size = ARRAY_SIZE(sc1336_linear_init_15fps_setting) / 2;
			}
			else {
				vin_err("unsupported fps setting\n");
				return -RET_ERROR;
			}
			vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
						vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc1336_linear_init_setting);

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc1336_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;

		case SLAVE_M:// 6
			vin_err("sc1336 in slave mode is %d\n" ,sensor_info->sensor_mode );
			if (sensor_info->fps == 15 || sensor_info->fps == 10) {
				sc1336_linear_init_setting = sc1336_linear_init_15fps_slave_setting;
				setting_size = ARRAY_SIZE(sc1336_linear_init_15fps_slave_setting) / 2;
			}
			else {
				vin_err("unsupported fps setting\n");
				return -RET_ERROR;
			}
			vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
						vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc1336_linear_init_15fps_slave_setting);

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc1336_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;

		default:
			vin_err("sc1336 in mode is %d\n" ,sensor_info->sensor_mode );
			if (sensor_info->fps == 15 || sensor_info->fps == 10) {
				sc1336_linear_init_setting = sc1336_linear_init_15fps_slave_setting;
				setting_size = ARRAY_SIZE(sc1336_linear_init_15fps_slave_setting) / 2;
			}
			else {
				vin_err("unsupported fps setting\n");
				return -RET_ERROR;
			}
			vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
						vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc1336_linear_init_15fps_slave_setting);

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc1336_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			// vin_err("not support mode %d\n", sensor_info->sensor_mode);
			// ret = -RET_ERROR;
			break;
	}
	vin_info("sc1336 config success under %d mode\n\n", sensor_info->sensor_mode);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

        switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			setting_size = sizeof(sc1336_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, sc1336_stream_on_setting);
			if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			vin_err("sc1336_master start \n");
			break;
		case SLAVE_M:
			setting_size = sizeof(sc1336_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, sc1336_stream_on_setting);
			if(ret < 0) {
					vin_err("start %s fail\n", sensor_info->sensor_name);
					return ret;
			}
			vin_err("sc1336_slave start \n");
			break;
        }
	vin_err("sc1336 start \n");
	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size =
			sizeof(sc1336_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("sensor stop sensor_name %s, setting_size = %d\n",
			sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, sc1336_stream_off_setting);
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

// turning data init
int sc1336_linear_data_init(sensor_info_t *sensor_info)
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
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = NORMAL_M;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 22500; //vts * fps = 1500 * 15
	turning_data.sensor_data.exposure_time_max = 1500;

	turning_data.sensor_data.active_width = 1280;
	turning_data.sensor_data.active_height = 720;
	// turning_data.sensor_data.gain_max = 128;
	turning_data.sensor_data.analog_gain_max = 255;   //205qq


	turning_data.sensor_data.digital_gain_max = 255;   //255
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 4000;
	// turning_data.sensor_data.conversion = 1;

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	// turning normal
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 968;

    // 设置下面的参数，底层驱动将会更新参数, 包括 line、agin，dgain
    // 不设置的话，则从上层hbre 生效，即在该文件中，去写寄存器
/*
        turning_data.normal.s_line = S1336_EXP_LINE;
        turning_data.normal.s_line_length = 2;
	turning_data.normal.again_control_num = 0;
	turning_data.normal.again_control[0] = S1336_PROGRAM_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = S1336_DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
*/
	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc1336_stream_on_setting)) {
		memcpy(stream_on, sc1336_stream_on_setting, sizeof(sc1336_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc1336_stream_off_setting)) {
		memcpy(stream_off, sc1336_stream_off_setting, sizeof(sc1336_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc1336_gain_lut,
			sizeof(sc1336_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc1336_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}

	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}


static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
    vin_err("%s %s mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
	const uint16_t AGAIN_3E08 = 0x3e08;	//for linear or dol2 long frame
	const uint16_t AGAIN_3E09 = 0x3e09;
	const uint16_t DGAIN = 0x3e06;
	const uint16_t DGAIN_FINE = 0x3e07;

	char again_3e09_value = 0, again_3e08_value = 0;
	char dgain_value = 0, dfine_gain_value = 0;

	double gain = 0;
	int again_index = 0, dgain_index = 0;
	if (mode == NORMAL_M) {
		if (again[0] >= sizeof(sc1336_gain_lut)/sizeof(uint32_t) * 6)
			again_index = sizeof(sc1336_gain_lut)/sizeof(uint32_t) * 6 - 1;
		else
			again_index = again[0];
		if (again_index < 32) {
			again_3e08_value = 0x1F;
			again_3e09_value = 0x00;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index];
		} else if (again_index < 64) {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x00;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 32];
		} else if (again_index < 96) {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x08;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 64];
		} else if (again_index < 128) {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x09;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 96];
		} else if (again_index < 160) {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x0b;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 128];
		} else if (again_index < 192) {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x0f;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 160];
		} else {
			again_3e08_value = 0x3F;
			again_3e09_value = 0x1F;
			dgain_value = 0;
			dfine_gain_value = sc1336_gain_lut[again_index - 192];
		}
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_3E08,
					again_3e08_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_3E09,
					again_3e09_value);
		char again_read_h = vin_i2c_read8(info->bus_num, 16, info->sensor_addr, AGAIN_3E08);
		char again_read_l = vin_i2c_read8(info->bus_num, 16, info->sensor_addr, AGAIN_3E09);

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN,
						dgain_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_FINE,
						dfine_gain_value);
		char dgain_read_h = vin_i2c_read8(info->bus_num, 16, info->sensor_addr, DGAIN);
		char dgain_read_l= vin_i2c_read8(info->bus_num, 16, info->sensor_addr, DGAIN_FINE);

		// printf("sc1336debug:%s %s index:%d gain:%f again_h:%x again_l:%x dgain_h:%x dgain_l:%x\n", __FILE__,
		// __FUNCTION__, again_index, pow(2.0, again_index/32.0), again_read_h,again_read_l, dgain_read_h, dgain_read_l);
		// vin_err("sc1336debug:%s %s index:%d gain:%f again_h:%x again_l:%x dgain_h:%x dgain_l:%x\n", __FILE__,
		// __FUNCTION__, again_index, pow(2.0, again_index/32.0), again_read_h,again_read_l, dgain_read_h, dgain_read_l);
	}
	else {
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
		uint32_t sline =  2 * line[0];
		if ( sline > 1350){
			sline = 1350;
		}

		temp0 = (sline & 0xF000) >> 12;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline & 0xFF0) >> 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
		temp2 = (sline & 0x0F) << 4;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);

		char exp_h = vin_i2c_read8(info->bus_num, 16, info->sensor_addr, EXP_LINE1);
		char exp_l = vin_i2c_read8(info->bus_num, 16, info->sensor_addr, EXP_LINE2);
		// printf("sc1336debug:%s %s exp_h:%x exp_l:%x\n", __FILE__,
		// __FUNCTION__, exp_h, exp_l);
		// vin_err("sc1336debug:%s %s exp_h:%x exp_l:%x\n", __FILE__,
		// __FUNCTION__, exp_h, exp_l);
	}
	else {
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
SENSOR_MODULE_EF(sc1336, sensor_emode, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc1336 = {
        .module = SENSOR_MNAME(sc1336),
#else
sensor_module_t sc1336 = {
        .module = "sc1336",
	.emode = sensor_emode,
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
