/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[sc1330t]:" fmt

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
#include "inc/sc1330t_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"
#include "../serial/max_serial.h"

#define MCLK (24000000)

#define S1330T_PROGRAM_GAIN	(0x3e08)
#define S1330T_DIGITAL_GAIN	(0x3e06)
#define S1330T_EXP_LINE		(0x3e00)
#define SC1330T_DOL2_SHORT_EXP_LINE		(0x3e04)

int sc1330t_dol2_data_init(sensor_info_t *sensor_info);

static int power_ref;

emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SC1330T] = {
		.serial_addr = 0x00,		// serial i2c addr, dummy
		.sensor_addr = 0x30,		// sensor i2c addr
		.eeprom_addr = 0x00,		// eeprom i2c addr, dummy
		.serial_rclk_out = 0,		// 0: reserved
		.rclk_mfp = 0,                // 0: reserved
	},
	[SC1330T_HDR] = {
		.serial_addr = 0x00,		// serial i2c addr, dummy
		.sensor_addr = 0x30,		// sensor i2c addr
		.eeprom_addr = 0x00,		// eeprom i2c addr, dummy
		.serial_rclk_out = 0,		// 0: reserved
		.rclk_mfp = 0,                // 0: reserved
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SC1330T, "0.0.1", "sc1330t_tuning.json", "0.1.0.0", &emode_data[SC1330T]),
	SENSOR_EMADD(SC1330T_HDR, "0.0.1", "sc1330t_hdr_tuning.json", "0.1.0.0", &emode_data[SC1330T_HDR]),

	SENSOR_EMEND(),
};

int sc1330t_linear_data_init(sensor_info_t *sensor_info);

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

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
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
				usleep(1 * 1000 * 1000);  //1000ms
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

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	const uint32_t *sc1330t_linear_init_setting;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			   __LINE__, sensor_info->sensor_name);
		return ret;
	}

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:	  // 1: normal
                        vin_info("sc1330 in normal mode\n");
			if (sensor_info->fps == 30) {
				sc1330t_linear_init_setting = sc1330t_linear_init_30fps_setting;
				setting_size = ARRAY_SIZE(sc1330t_linear_init_30fps_setting) / 2;
			} else if (sensor_info->fps == 60) {
				sc1330t_linear_init_setting = sc1330t_linear_init_60fps_setting;
				setting_size = ARRAY_SIZE(sc1330t_linear_init_60fps_setting) / 2;
			} else {
				vin_err("unsupported fps setting\n");
				return -RET_ERROR;
			}
			vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
                        vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc1330t_linear_init_setting);

                        if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc1330t_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
                case DOL2_M:  // 2: DOl2
                        vin_info("sc1330t in dol2 mode\n");
			setting_size = sizeof(sc1330t_hdr_init_setting) / sizeof(uint32_t) / 2;
			vin_dbg("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sc1330t_hdr_init_setting);

                        if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = sc1330t_dol2_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : dol2 data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	vin_info("sc1330t config success under %d mode\n\n", sensor_info->sensor_mode);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

        switch(sensor_info->sensor_mode) {
                case NORMAL_M:
                setting_size = sizeof(sc1330t_stream_on_setting)/sizeof(uint32_t)/2;
                vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
                ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                        setting_size, sc1330t_stream_on_setting);
                if(ret < 0) {
                        vin_err("start %s fail\n", sensor_info->sensor_name);
                        return ret;
                }
                break;
        case DOL2_M:
                setting_size = sizeof(sc1330t_stream_on_setting)/sizeof(uint32_t)/2;
                vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
                ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
                        setting_size, sc1330t_stream_on_setting);
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
                sizeof(sc1330t_stream_off_setting) / sizeof(uint32_t) / 2;
        vin_info("sensor stop sensor_name %s, setting_size = %d\n",
                sensor_info->sensor_name, setting_size);
        ret = vin_write_array(sensor_info->bus_num,
                             sensor_info->sensor_addr, 2,
                             setting_size, sc1330t_stream_off_setting);
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
int sc1330t_linear_data_init(sensor_info_t *sensor_info)
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
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 31500;
	turning_data.sensor_data.exposure_time_max = 968;

	turning_data.sensor_data.active_width = 1280;
	turning_data.sensor_data.active_height = 960;
	// turning_data.sensor_data.gain_max = 128;
	turning_data.sensor_data.analog_gain_max = 205;
	turning_data.sensor_data.digital_gain_max = 255;   //159
	turning_data.sensor_data.exposure_time_min = 8;
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
        turning_data.normal.s_line = S1330T_EXP_LINE;
        turning_data.normal.s_line_length = 2;
	turning_data.normal.again_control_num = 0;
	turning_data.normal.again_control[0] = S1330T_PROGRAM_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = S1330T_DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
*/
	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc1330t_stream_on_setting)) {
		memcpy(stream_on, sc1330t_stream_on_setting, sizeof(sc1330t_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc1330t_stream_off_setting)) {
		memcpy(stream_off, sc1330t_stream_off_setting, sizeof(sc1330t_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc1330t_gain_lut,
			sizeof(sc1330t_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc1330t_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, sc1330t_dgain_lut,
			sizeof(sc1330t_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc1330t_dgain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
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

int sc1330t_dol2_data_init(sensor_info_t *sensor_info)
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
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 75000;	//vts*fps=2500*30=75000
	turning_data.sensor_data.exposure_time_max = 538;	//short frame: max 7.17ms
	turning_data.sensor_data.exposure_time_long_max = 1500;	//long frame: 20ms

	turning_data.sensor_data.active_width = 1280;
	turning_data.sensor_data.active_height = 960;

	turning_data.sensor_data.analog_gain_max = 205;		//gain lut index
	turning_data.sensor_data.digital_gain_max = 255;	//gain lut index
	turning_data.sensor_data.exposure_time_min = 1;

	turning_data.sensor_data.exposure_time_step = 2;	//hdr exposure_time_step, from spec

        // raw10
        sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
        sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc1330t_stream_on_setting)) {
		memcpy(stream_on, sc1330t_stream_on_setting, sizeof(sc1330t_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc1330t_stream_off_setting)) {
		memcpy(stream_off, sc1330t_stream_off_setting, sizeof(sc1330t_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.dol2.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, sc1330t_gain_lut,
			sizeof(sc1330t_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc1330t_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.dol2.again_lut[open_cnt], 2);
		}
	}

	turning_data.dol2.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.dol2.dgain_lut != NULL) {
		memset(turning_data.dol2.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.dol2.dgain_lut, sc1330t_dgain_lut,
			sizeof(sc1330t_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc1330t_dgain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.dol2.dgain_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	if (turning_data.dol2.dgain_lut) {
		free(turning_data.dol2.dgain_lut);
		turning_data.dol2.dgain_lut = NULL;
	}
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
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
		if (again[0] >= sizeof(sc1330t_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(sc1330t_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(sc1330t_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(sc1330t_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		lower_again_reg_value = sc1330t_gain_lut[again_index] & 0x000000FF;
		high_again_reg_value = (sc1330t_gain_lut[again_index] >> 8) & 0x000000FF;
		lower_dgain_reg_value = sc1330t_dgain_lut[dgain_index] & 0x000000FF;
		high_dgain_reg_value = (sc1330t_dgain_lut[dgain_index] >> 8) & 0x000000FF;
		//vin_info("%s again(0x3e08/0x3e09):%x,%x; dgain(0x3e06x3e07):%x,%x\n",
		//		__FUNCTION__, lower_again_reg_value, high_again_reg_value, lower_dgain_reg_value,
		//		high_dgain_reg_value);

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_LOW, lower_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_HIGH, high_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_LOW, lower_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_HIGH, high_dgain_reg_value);
	} else if (mode == DOL2_M){
		if (again[0] >= sizeof(sc1330t_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(sc1330t_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (again[1] >= sizeof(sc1330t_gain_lut)/sizeof(uint32_t))
			s_again_index = sizeof(sc1330t_gain_lut)/sizeof(uint32_t) - 1;
		else
			s_again_index = again[1];

		if (dgain[0] >= sizeof(sc1330t_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(sc1330t_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		if (dgain[1] >= sizeof(sc1330t_dgain_lut)/sizeof(uint32_t))
			s_dgain_index = sizeof(sc1330t_dgain_lut)/sizeof(uint32_t) - 1;
		else
			s_dgain_index = dgain[1];


		lower_again_reg_value = sc1330t_gain_lut[again_index] & 0x000000FF;
		high_again_reg_value = (sc1330t_gain_lut[again_index] >> 8) & 0x000000FF;
		lower_dgain_reg_value = sc1330t_dgain_lut[dgain_index] & 0x000000FF;
		high_dgain_reg_value = (sc1330t_dgain_lut[dgain_index] >> 8) & 0x000000FF;

		s_lower_again_reg_value = sc1330t_gain_lut[s_again_index] & 0x000000FF;
		s_high_again_reg_value = (sc1330t_gain_lut[s_again_index] >> 8) & 0x000000FF;
		s_lower_dgain_reg_value = sc1330t_dgain_lut[s_dgain_index] & 0x000000FF;
		s_high_dgain_reg_value = (sc1330t_dgain_lut[s_dgain_index] >> 8) & 0x000000FF;

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
SENSOR_MODULE_EF(sc1330t, sensor_emode, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc1330t = {
        .module = SENSOR_MNAME(sc1330t),
#else
sensor_module_t sc1330t = {
        .module = "sc1330t",
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
