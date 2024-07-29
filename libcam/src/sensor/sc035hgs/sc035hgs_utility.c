/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt) "[sc035hgs]:" fmt

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
#include "inc/sc035hgs_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

// #define AE_DBG

#define MCLK (24000000)

#define SC035HGS_PROGRAM_GAIN (0x3e08)
#define SC035HGS_DIGITAL_GAIN (0x3e06)
#define SC035HGS_EXP_LINE (0x3e00)
#define SC035HGS_DOL2_SHORT_EXP_LINE (0x3e04)

int sc035hgs_dol2_data_init(sensor_info_t *sensor_info);

static int power_ref;
int sc035hgs_linear_data_init(sensor_info_t *sensor_info);

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if (sensor_info->gpio_num > 0)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									 sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
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
	if (sensor_info->gpio_num > 0)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									 sensor_info->gpio_level[gpio]);
				usleep(1 * 100 * 1000); // 100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
									  1 - sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000); // 100ms
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
	if (ret < 0)
	{
		vin_err("%d : sensor reset %s fail\n",
				__LINE__, sensor_info->sensor_name);
		return ret;
	}

	switch (sensor_info->sensor_mode)
	{
	case NORMAL_M: // 1: normal
		vin_info("sc035hgs in normal/master mode\n");
		setting_size = sizeof(sc035hgs_linear_init_master_setting) / sizeof(uint32_t) / 2;
		vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, sc035hgs_linear_init_master_setting);

		if (ret < 0)
		{
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}

		ret = sc035hgs_linear_data_init(sensor_info);
		if (ret < 0)
		{
			vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		break;
	case SLAVE_M: // 6: slave
		vin_info("sc035hgs in slave mode\n");
		setting_size = sizeof(sc035hgs_linear_init_slave_setting) / sizeof(uint32_t) / 2;
		vin_info("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		vin_info("bus_num = %d, sensor_addr = 0x%0x \n", sensor_info->bus_num, sensor_info->sensor_addr);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, sc035hgs_linear_init_slave_setting);

		if (ret < 0)
		{
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}

		ret = sc035hgs_linear_data_init(sensor_info);
		if (ret < 0)
		{
			vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		break;
	case DOL2_M: // 2: DOl2
		vin_info("sc035hgs in dol2 mode\n");
		setting_size = sizeof(sc035hgs_hdr_init_setting) / sizeof(uint32_t) / 2;
		vin_dbg("sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, sc035hgs_hdr_init_setting);
		if (ret < 0)
		{
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		ret = sc035hgs_dol2_data_init(sensor_info);
		if (ret < 0)
		{
			vin_err("%d : dol2 data init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		break;
	default:
		vin_err("not support mode %d\n", sensor_info->sensor_mode);
		ret = -RET_ERROR;
		break;
	}
	vin_info("sc035hgs config success under %d mode\n\n", sensor_info->sensor_mode);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch (sensor_info->sensor_mode)
	{
	case NORMAL_M:
	case SLAVE_M:
		if (sensor_info->sensor_addr == 0x30)
		{
			// set vc = 0
			ret = vin_i2c_write8(sensor_info->bus_num, sensor_info->reg_width,
				sensor_info->sensor_addr, 0x4816, 0x1);
		}
		else if (sensor_info->sensor_addr == 0x31)
		{
			// set vc = 1
			ret = vin_i2c_write8(sensor_info->bus_num, sensor_info->reg_width,
				sensor_info->sensor_addr, 0x4816, 0x5);
		}
		if (ret < 0)
		{
			vin_err("set sensor %s vc fail\n", sensor_info->sensor_name);
			return ret;
		}
		setting_size = sizeof(sc035hgs_stream_on_setting) / sizeof(uint32_t) / 2;
		vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
								setting_size, sc035hgs_stream_on_setting);
		if (ret < 0)
		{
			vin_err("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
		break;
	case DOL2_M:
		setting_size = sizeof(sc035hgs_stream_on_setting) / sizeof(uint32_t) / 2;
		vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, sc035hgs_stream_on_setting);
		if (ret < 0)
		{
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
		sizeof(sc035hgs_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("sensor stop sensor_name %s, setting_size = %d\n",
			 sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num,
						  sensor_info->sensor_addr, 2,
						  setting_size, sc035hgs_stream_off_setting);
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
int sc035hgs_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

#ifdef AE_DBG
	uint16_t temp;
	uint16_t HTS_HIGH, HTS_LOW;
	uint32_t HTS_VALUE;
#endif
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	//SLAVE MODE is just for custom, kernel should be keep same with NORMAL_M
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = NORMAL_M;  //NOTICE
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));

	turning_data.sensor_data.active_width = 640;
	turning_data.sensor_data.active_height = 480;

#ifdef AE_DBG
	// read hts = line_length, trigger mode: default value is 0x640
	HTS_HIGH = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320c);
	HTS_LOW = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320d);
	HTS_VALUE = (HTS_HIGH << 8 | HTS_LOW);
	printf("%s read HTS_HIGH = 0x%x, HTS_LOW = 0x%x, HTS = 0x%x \n",
		   __FUNCTION__, HTS_HIGH, HTS_LOW, HTS_VALUE);
#endif

	/* from spec: one line exposure time = (1/(vts * hts * fps))*hts = 1/(vts * fps)
	 * trigger mode, we should use previous value: 1250, not 0x3fff
	 * */
	turning_data.sensor_data.lines_per_second = 37500; // fps * vts = 30 * 1250 = 37500
	// from customer, exposure max = 10ms, exposure_time_max = lines_per_second / 100 = 375
	turning_data.sensor_data.exposure_time_max = 375; // from customer
	turning_data.sensor_data.exposure_time_min = 8;	  // trigger mode, value read from 0x3226
													  // sensor AGC decided by 0x3e03
#ifdef AE_DBG
	temp = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3e03);
	printf("%s read AGC 0x3e03 = 0x%x \n", __FUNCTION__, temp);
#endif
	turning_data.sensor_data.analog_gain_max = 63; // from sensor fae, gain lut index
	turning_data.sensor_data.digital_gain_max = 0; // from sensor fae, gain lut index

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc035hgs_stream_on_setting))
	{
		memcpy(stream_on, sc035hgs_stream_on_setting, sizeof(sc035hgs_stream_on_setting));
	}
	else
	{
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc035hgs_stream_off_setting))
	{
		memcpy(stream_off, sc035hgs_stream_off_setting, sizeof(sc035hgs_stream_off_setting));
	}
	else
	{
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL)
	{
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc035hgs_again_lut0,
			   sizeof(sc035hgs_again_lut0));
		for (open_cnt = 0; open_cnt <
						   sizeof(sc035hgs_again_lut0) / sizeof(uint32_t);
			 open_cnt++)
		{
			// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	turning_data.normal.dgain_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL)
	{
		memset(turning_data.normal.dgain_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, sc035hgs_dgain_lut0,
			   sizeof(sc035hgs_dgain_lut0));
		for (open_cnt = 0; open_cnt <
						   sizeof(sc035hgs_dgain_lut0) / sizeof(uint32_t);
			 open_cnt++)
		{
			// DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut)
	{
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut)
	{
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0)
	{
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sc035hgs_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
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
	//SLAVE MODE is just for custom, kernel should be keep same with DOL2_M
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = DOL2_M;  //NOTICE
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));

	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 60240;
	turning_data.sensor_data.exposure_time_max = 968;

	turning_data.sensor_data.active_width = 1088;
	turning_data.sensor_data.active_height = 1280;
	// turning_data.sensor_data.gain_max = 128 * 8192;
	turning_data.sensor_data.analog_gain_max = 205 * 8192;
	turning_data.sensor_data.digital_gain_max = 159 * 8192;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 2176;
	// turning_data.sensor_data.conversion = 1;

	// turning normal
	// // // short frame
	// turning_data.dol2.s_line = SC1330T_DOL2_SHORT_EXP_LINE;
	// turning_data.dol2.s_line_length = 2;
	// // long frame
	// turning_data.dol2.m_line = S1330T_EXP_LINE;
	// turning_data.dol2.m_line_length = 2;

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.dol2.line_p[0].ratio = 1 << 8;
	turning_data.dol2.line_p[0].offset = 0;
	turning_data.dol2.line_p[0].max = 66;
	turning_data.dol2.line_p[1].ratio = 1 << 8;
	turning_data.dol2.line_p[1].offset = 0;
	turning_data.dol2.line_p[1].max = 2176;

#if 0
	turning_data.dol2.again_control_num = 1;
	turning_data.dol2.again_control[0] = SC035HGS_PROGRAM_GAIN;
	turning_data.dol2.again_control_length[0] = 2;
	turning_data.dol2.dgain_control_num = 1;
	turning_data.dol2.dgain_control_length[0] = 2;
	turning_data.dol2.dgain_control[0] = SC035HGS_DIGITAL_GAIN;
#endif

	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc035hgs_stream_on_setting))
	{
		memcpy(stream_on, sc035hgs_stream_on_setting, sizeof(sc035hgs_stream_on_setting));
	}
	else
	{
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc035hgs_stream_off_setting))
	{
		memcpy(stream_off, sc035hgs_stream_off_setting, sizeof(sc035hgs_stream_off_setting));
	}
	else
	{
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.dol2.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL)
	{
		memset(turning_data.dol2.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, sc035hgs_again_lut0,
			   sizeof(sc035hgs_again_lut0));
		for (open_cnt = 0; open_cnt <
						   sizeof(sc035hgs_again_lut0) / sizeof(uint32_t);
			 open_cnt++)
		{
			// DOFFSET(&turning_data.dol2.again_lut[open_cnt], 2);
		}
	}

	turning_data.dol2.dgain_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.dol2.dgain_lut != NULL)
	{
		memset(turning_data.dol2.dgain_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.dol2.dgain_lut, sc035hgs_dgain_lut0,
			   sizeof(sc035hgs_dgain_lut0));
		for (open_cnt = 0; open_cnt <
						   sizeof(sc035hgs_dgain_lut0) / sizeof(uint32_t);
			 open_cnt++)
		{
			// DOFFSET(&turning_data.dol2.dgain_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.dol2.again_lut)
	{
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}
	if (turning_data.dol2.dgain_lut)
	{
		free(turning_data.dol2.dgain_lut);
		turning_data.dol2.dgain_lut = NULL;
	}
	if (ret < 0)
	{
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
	printf("%s %s mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	const uint16_t AGAIN_LOW = 0x3e08;
	const uint16_t AGAIN_HIGH = 0x3e09;
	const uint16_t DGAIN_LOW = 0x3e06;
	const uint16_t DGAIN_HIGH = 0x3e07;
	char lower_again_reg_value = 0, high_again_reg_value = 0;
	char lower_dgain_reg_value = 0, high_dgain_reg_value = 0;
	int again_index = 0, dgain_index = 0;
	if (mode == NORMAL_M || mode == DOL2_M || mode == SLAVE_M)
	{
		if (again[0] >= 127)
			again_index = 127;
		else
			again_index = again[0];

		if (dgain[0] >= 95)
			dgain_index = 95;
		else
			dgain_index = dgain[0];

		lower_again_reg_value = (sc035hgs_again_lut0[again_index] << 2) & 0x000000FF;
		high_again_reg_value = sc035hgs_again_lut1[again_index] & 0x000000FF;
		lower_dgain_reg_value = sc035hgs_dgain_lut0[dgain_index] & 0x000000FF;
		high_dgain_reg_value = sc035hgs_dgain_lut1[dgain_index] & 0x000000FF;
#ifdef AE_DBG
		printf("%s again(0x3e08/0x3e09):%x,%x; dgain(0x3e06x3e07):%x,%x\n",
			   __FUNCTION__, lower_again_reg_value, high_again_reg_value, lower_dgain_reg_value,
			   high_dgain_reg_value);
#endif

		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_LOW, lower_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_HIGH, high_again_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_LOW, lower_dgain_reg_value);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_HIGH, high_dgain_reg_value);
	}
	else
	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

/* input value:
 * line: exposure line number
 * line_num: linear mode: 1; dol2 mode: 2
 * */
static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	printf("%s line mode %d, --line %d , line_num:%d \n", __FUNCTION__, mode, line[0], line_num);
#endif
	const uint16_t EXP_LINE0 = 0x3e01;
	const uint16_t EXP_LINE1 = 0x3e02;
	char temp0 = 0, temp1 = 0;
	if (mode == NORMAL_M || mode == SLAVE_M)
	{
		uint32_t sline = line[0];
		/*
		 * NOTICE: trigger mode: sline = line(from isp)
		 * from sensor fae:
		 * exposure_time_max =  1/fps - readout(480line) = 33ms- 480line = lines_per_second/30 - 480 = 1250 - 480 = 770
		 * from customer, exposure time max is 10ms，sline = exposure_time_max = 375
		 */
		if (sline < 8)
		{
			sline = 8;
		}
		else if (sline > 375)
		{
			sline = 375;
		}
		temp0 = (sline >> 4) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline & 0x0F) << 4; // bit[7:4]
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
#ifdef AE_DBG
		printf("write sline = %d, 0x3e01 = 0x%x, 0x3e02 = 0x%x \n",
			   sline, temp0, temp1);
#endif
	}
	else
	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	// *enable = 0;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc035hgs, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc035hgs = {
	.module = SENSOR_MNAME(sc035hgs),
#else
sensor_module_t sc035hgs = {
	.module = "sc035hgs",
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
