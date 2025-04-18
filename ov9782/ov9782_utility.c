/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024-2025, D-Robotics Co., Ltd.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[ov9782]:" fmt

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
#include "inc/ov9782_setting.h"
#include "hb_camera_data_config.h"


#define REG_WIDTH	2	//reg16 data8
#define OV9782_LINE_LENGTH_HI 0x380C
#define OV9782_LINE_LENGTH_LO 0x380D
#define OV9782_FRM_LENGTH_HI 0x380E
#define OV9782_FRM_LENGTH_LO 0x380F
static int ov9782_linear_data_init(sensor_info_t *sensor_info);

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
			vin_info("ov9782 in normal/linear mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x, fps = %d\n",
				sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps);
			if (sensor_info->resolution == 720 && sensor_info->fps == 120) {
				setting_size = sizeof(ov9782_init_1280x720_linear_setting_120fps) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, ov9782_init_1280x720_linear_setting_120fps);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
			}
			else if (sensor_info->resolution == 720 && sensor_info->fps == 30) {
				setting_size = sizeof(ov9782_init_1280x720_linear_setting_30fps) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, ov9782_init_1280x720_linear_setting_30fps);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
			}
			else if (sensor_info->resolution == 360 && sensor_info->fps == 200) {
				setting_size = sizeof(ov9782_init_640x360_linear_setting_200fps) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, ov9782_init_640x360_linear_setting_200fps);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
			}
			else {
				vin_err("%d : please choice 1280x720@120fps, 1280x720@30fps, 640x360@200fps \n",
					__LINE__);
				return -HB_CAM_INIT_FAIL;
			}

			ret = ov9782_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_INIT_FAIL;
			}
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	vin_info("ov9782 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	setting_size = sizeof(ov9782_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
			setting_size, ov9782_stream_on_setting);
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
		sizeof(ov9782_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, ov9782_stream_off_setting);
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

void ov9782_commmon_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
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

void ov9782_param_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{

	int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV9782_FRM_LENGTH_HI);
	int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV9782_FRM_LENGTH_LO);
	uint32_t vts = vts_hi;
	vts = vts << 8 | vts_lo;	//default 0x038e
	pr_info("ov9782: vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi, vts_lo, vts);
	int hts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV9782_LINE_LENGTH_HI);
	int hts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV9782_LINE_LENGTH_LO);
	uint32_t hts = hts_hi;
	hts = hts << 8 | hts_lo;	//default 0x02d8
	pr_info("ov9782: hts_hi:0x%x,hts_lo:0x%x,hts:0x%x\n", hts_hi, hts_lo, hts);

	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;
	// turning sensor_data
	turning_data->sensor_data.conversion = 1;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.lines_per_second = vts * sensor_info->fps;
	turning_data->sensor_data.exposure_time_max = (vts - 25);
	turning_data->sensor_data.exposure_time_long_max = (vts - 25);
	turning_data->sensor_data.analog_gain_max = 128;	//only analog gain index
	turning_data->sensor_data.digital_gain_max = 0;
	turning_data->sensor_data.exposure_time_min = 1;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
		printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	const uint16_t AEC_MANUAL= 0x3503;
	const uint16_t GAIN = 0x3509;
	int gain_index = 0;
	char again_reg_value = 0;

	if (mode == NORMAL_M) {
		if (again[0] >= sizeof(ov9782_gain_lut)/sizeof(uint32_t))
		gain_index = sizeof(ov9782_gain_lut)/sizeof(uint32_t) - 1;
	else
		gain_index = again[0];

		again_reg_value = (ov9782_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
				printf("%s, gain_index:%d, 0x3509 = 0x%x\n",
			__FUNCTION__, gain_index, again_reg_value);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, GAIN, again_reg_value);
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
	const uint16_t EXP_LINE0 = 0x3500;	//bit[3:0] -> exposure[19:16]
	const uint16_t EXP_LINE1 = 0x3501;	//bit[7:0] -> exposure[15:8]
	const uint16_t EXP_LINE2 = 0x3502;	//bit[7:0] -> exposure[7:0]
						//NOTICE 0x3501 bit[0:3] is fraction bits
	char temp0 = 0, temp1 = 0, temp2 = 0;

		if (mode == NORMAL_M) {
		uint32_t sline =  (line[0] << 4) & 0x0FFFFF;

		temp0 = (sline >> 16) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
		temp1 = (sline >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
		temp2 = sline & 0xF0;	//NOTICE don't write low 4 bits
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);
#ifdef AE_DBG
		printf("write sline = %d, 0x3500 = 0x%x, 0x3501 = 0x%x, 0x3502 = 0x%x\n",
						sline, temp0, temp1, temp2);
#endif
		} else {
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

static int ov9782_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	ov9782_commmon_data_init(sensor_info,&turning_data);
	ov9782_param_data_init(sensor_info,&turning_data);

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ov9782_stream_on_setting)) {
		memcpy(stream_on, ov9782_stream_on_setting, sizeof(ov9782_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ov9782_stream_off_setting)) {
		memcpy(stream_off, ov9782_stream_off_setting, sizeof(ov9782_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, ov9782_gain_lut,
			sizeof(ov9782_gain_lut));
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
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ov9782, CAM_MODULE_FLAG_A16D8);
sensor_module_t ov9782 = {
	.module = SENSOR_MNAME(ov9782),
#else
sensor_module_t ov9782 = {
	.module = "ov9782",
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
