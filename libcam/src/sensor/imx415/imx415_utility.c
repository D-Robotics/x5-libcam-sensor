/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[imx415]:" fmt

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
#include "inc/imx415_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define REG_WIDTH	2	//reg16 data8

static int imx415_linear_data_init(sensor_info_t *sensor_info);
static int imx415_dol2_data_init(sensor_info_t *sensor_info);

static uint32_t imx415_max(int32_t a, int32_t b)
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
			vin_info("imx415 in normal/linear mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x, fps = %d, config_index = %d\n",
				sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps, sensor_info->config_index);
			if (sensor_info->fps == 30) {
				if(sensor_info->config_index == 0){ // 0: 2lane (default is 0)
					setting_size = sizeof(imx415_init_3840x2160_2lane_linear_setting) / sizeof(uint32_t) / 2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
						setting_size, imx415_init_3840x2160_2lane_linear_setting);
				}else{// 1: 4lane
					setting_size = sizeof(imx415_init_3840x2160_4lane_linear_setting) / sizeof(uint32_t) / 2;
					ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
						setting_size, imx415_init_3840x2160_4lane_linear_setting);
				}

				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
				ret = imx415_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
				}
			} else if (sensor_info->fps == 60) {
				setting_size = sizeof(imx415_init_3840x2160_60_fps_linear_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
					setting_size, imx415_init_3840x2160_60_fps_linear_setting);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_I2C_WRITE_FAIL;
				}
				ret = imx415_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
					return -HB_CAM_INIT_FAIL;
				}
			}
			break;
		case DOL2_M:
			vin_info("imx415 in dol2/hdr mode\n");
			vin_info("bus_num = %d, sensor_addr = 0x%0x fps = %d\n",
				sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps);

			setting_size = sizeof(imx415_init_3840x2160_dol2_4lane_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
							setting_size, imx415_init_3840x2160_dol2_4lane_setting);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return -HB_CAM_I2C_WRITE_FAIL;
			}
			ret = imx415_dol2_data_init(sensor_info);
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
	vin_info("imx415 config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch (sensor_info->sensor_mode)
	{
	case NORMAL_M:
		setting_size = sizeof(imx415_stream_on_setting) / sizeof(uint32_t) / 2;
		vin_info(" start linear mode, sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, imx415_stream_on_setting);
		if (ret < 0)
		{
			vin_err("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
		break;
	case DOL2_M:
		setting_size = sizeof(imx415_stream_on_setting) / sizeof(uint32_t) / 2;
		vin_info("start hdr mode, sensor_name %s, setting_size = %d\n",
				 sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							  setting_size, imx415_stream_on_setting);
		if (ret < 0)
		{
			vin_err("start %s fail\n", sensor_info->sensor_name);
			return ret;
		}
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
		sizeof(imx415_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor stop\n", sensor_info->sensor_name);
	ret = vin_write_array(sensor_info->bus_num,
			sensor_info->sensor_addr, REG_WIDTH,
			setting_size, imx415_stream_off_setting);
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
/* Helper function: Set gain register */
static void set_gain_registers(hal_control_info_t *info, uint16_t again_reg,char again_val)
{
	vin_i2c_write8(info->bus_num, 16, info->sensor_addr, again_reg, again_val);
}
#define AE_DBG
static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
		printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	const uint16_t AGAIN = GAIN_PGC0;       //LEF:  Long Frame Gain register
	const uint16_t S_AGAIN = GAIN_PGC1;     //SEF1: Short Gain register
	char again_reg_value = 0, s_again_reg_value = 0;
	int again_index = 0, s_again_index = 0;

	if (mode == NORMAL_M ) {

		if (again[0] >= sizeof(imx415_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(imx415_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		again_reg_value = (imx415_gain_lut[again_index] >> 0) & 0xFF;
		 // Set Long Exposure Gain
		 set_gain_registers(info, AGAIN,again_reg_value);
#ifdef AE_DBG
				printf("%s, again_index: %d, again:0x3090 = 0x%x\n",
				__FUNCTION__, again_index, again_reg_value);
#endif
	} else if (mode == DOL2_M){

		if (again[0] >= sizeof(imx415_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(imx415_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (again[1] >= sizeof(imx415_gain_lut)/sizeof(uint32_t))
			s_again_index = sizeof(imx415_gain_lut)/sizeof(uint32_t) - 1;
		else
			s_again_index = again[1];

		again_reg_value = (imx415_gain_lut[again_index] >> 0) & 0xFF;
		s_again_reg_value = (imx415_gain_lut[s_again_index] >> 0) & 0xFF;
#ifdef AE_DBG
		printf("%s, again_index: %d, again:0x3090 = 0x%x, s_again:0x3092 = 0x%x \n",
			__FUNCTION__, again_index, again_reg_value, s_again_reg_value);
#endif
		 // Set Long Exposure Gain
		 set_gain_registers(info, AGAIN,again_reg_value);
		 // Set Short Exposure Gain
		 set_gain_registers(info, S_AGAIN,s_again_reg_value);

	}else{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

/* input value:
 * line: exposure time value (in lines)
 * line_num: linear mode: 1; dol2 mode: 2
 *
 * IMX415 Linear mode exposure formula (from Datasheet):
 *   t_exp = VMAX - SHR0,  so SHR0 = VMAX - line[0]
 * Constraints:
 *   8 <= SHR0 <= (VMAX - 4)
 *
 * IMX415 DOL2 exposure formula (from AppNote DOL Rev6.0):
 *   LEF:  t_LEF  = FSC  - SHR0,  so SHR0 = FSC  - line[0]
 *   SEF1: t_SEF1 = RHS1 - SHR1,  so SHR1 = RHS1 - line[1]
 *   FSC = VMAX * 2
 * Constraints:
 *   SHR0: 2n (even),     (RHS1 + 9) <= SHR0 <= (FSC - 8)
 *   SHR1: 2n+1 (odd),    9 <= SHR1 <= (RHS1 - 8)
 */

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	printf("line mode %d, --line[0] %d, line[1] %d, line_num:%d\n", mode, line[0],
		(line_num > 1) ? line[1] : 0, line_num);
#endif
	uint32_t tmp = 0;
	char temp0 = 0, temp1 = 0, temp2 = 0;
	int val_l = 0, val_m = 0, val_h = 0;
	uint32_t Vmax = 0;

	val_l = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_VMAX);
	val_m = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_VMAX + 1);
	val_h = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_VMAX + 2);
	Vmax = ((val_h & 0x0F) << 16) | (val_m << 8) | val_l;

	if ((mode == NORMAL_M) || (line_num == 1)) {
		int shr0 = Vmax - line[0];

		if (shr0 < 8)
			shr0 = 8;
		if (shr0 > (int)(Vmax - 4))
			shr0 = Vmax - 4;

		tmp = shr0;
		temp2 = (tmp >> 16) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0 + 2, temp2);
		temp1 = (tmp >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0 + 1, temp1);
		temp0 = (tmp & 0xFF);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0, temp0);

#ifdef AE_DBG
		printf("SHR0 = %d, 0x3052 = 0x%x, 0x3051 = 0x%x, 0x3050 = 0x%x\n",
			shr0, temp2, temp1, temp0);
#endif
	} else if (mode == DOL2_M) {
		uint32_t rhs1 = 0;
		uint32_t fsc = 0;
		int shr0 = 0, shr1 = 0;

#ifdef AE_DBG
		printf("line[0] = %d, line[1] = %d\n", line[0], line[1]);
#endif
		val_l = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_RHS1);
		val_m = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_RHS1 + 1);
		val_h = hb_vin_i2c_read_reg16_data8(info->bus_num, info->sensor_addr, IMX415_RHS1 + 2);
		rhs1 = ((val_h & 0x0F) << 16) | (val_m << 8) | val_l;

		fsc = Vmax * 2;

		shr0 = fsc - line[0];
		shr1 = rhs1 - line[1];

		if (shr1 < 9)
			shr1 = 9;
		if (shr1 > (int)(rhs1 - 8))
			shr1 = rhs1 - 8;
		if (shr0 < (int)(rhs1 + 9))
			shr0 = rhs1 + 9;
		if (shr0 > (int)(fsc - 8))
			shr0 = fsc - 8;

		shr1 |= 0x01; 			//SHR1: 2n+1 (odd)
		shr0 = (shr0 >> 1) << 1;//SHR0: 2n (even)

#ifdef AE_DBG
		printf("shr0 = 0x%x, shr1 = 0x%x, rhs1 = 0x%x, Vmax = 0x%x, fsc = 0x%x\n",
			shr0, shr1, rhs1, Vmax, fsc);
#endif
		tmp = shr0;
		temp2 = (tmp >> 16) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0 + 2, temp2);
		temp1 = (tmp >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0 + 1, temp1);
		temp0 = (tmp & 0xFF);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR0, temp0);

#ifdef AE_DBG
		printf("SHR0: 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
			IMX415_SHR0 + 2, temp2, IMX415_SHR0 + 1, temp1, IMX415_SHR0, temp0);
#endif
		tmp = shr1;
		temp2 = (tmp >> 16) & 0x0F;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR1 + 2, temp2);
		temp1 = (tmp >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR1 + 1, temp1);
		temp0 = (tmp & 0xFF);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX415_SHR1, temp0);

#ifdef AE_DBG
		printf("SHR1: 0x%x = 0x%x, 0x%x = 0x%x, 0x%x = 0x%x\n",
			IMX415_SHR1 + 2, temp2, IMX415_SHR1 + 1, temp1, IMX415_SHR1, temp0);
#endif
	} else {
		vin_err(" unsupport mode %d\n", mode);
		return -1;
	}

	return 0;
}

static int imx415_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	int val_l = 0, val_m = 0, val_h = 0;
	uint32_t Vmax = 0;
	//IMX415_VMAX [19:0]
	val_l = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX);
	val_m = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX + 1);
	val_h = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX + 2);
	Vmax = ((val_h & 0x0F) << 16) | (val_m << 8) | val_l;

	vin_info("Vmax = 0x%x\n", Vmax);

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

	turning_data.sensor_data.active_width = sensor_info->width;
	turning_data.sensor_data.active_height = sensor_info->height;

	turning_data.sensor_data.lines_per_second = Vmax * sensor_info->fps;
	turning_data.sensor_data.exposure_time_max = Vmax - 4;
	turning_data.sensor_data.exposure_time_min = 8;
	turning_data.sensor_data.analog_gain_max = 255;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.analog_gain_init = 32;
	turning_data.sensor_data.digital_gain_init = 0;
	turning_data.sensor_data.exposure_time_init = 2430;


	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_GB, (uint32_t)BAYER_PATTERN_RGGB);
	// sensor exposure_max_bit, maybe not used ?  //FIXME
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	//some stress test case, we need kernel stream_ctrl.
	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx415_stream_on_setting)) {
		memcpy(stream_on, imx415_stream_on_setting, sizeof(imx415_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx415_stream_off_setting)) {
		memcpy(stream_off, imx415_stream_off_setting, sizeof(imx415_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, imx415_gain_lut,
			sizeof(imx415_gain_lut));
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

static int imx415_dol2_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	int val_l = 0, val_m = 0, val_h = 0;
	uint32_t Vmax = 0;
	uint32_t rhs1 = 0;
	uint32_t fsc = 0;
	//IMX415_VMAX [19:0]
	val_l = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX);
	val_m = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX + 1);
	val_h = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_VMAX + 2);
	Vmax = ((val_h & 0x0F) << 16) | (val_m << 8) | val_l;

	val_l = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_RHS1);
	val_m = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_RHS1 + 1);
	val_h = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, IMX415_RHS1 + 2);
	rhs1 = ((val_h & 0x0F) << 16) | (val_m << 8) | val_l;

	fsc = Vmax * 2;
	vin_info("Vmax = 0x%x, RHS1 = 0x%x, FSC = 0x%x\n", Vmax, rhs1, fsc);

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
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.active_width = sensor_info->width;
	turning_data.sensor_data.active_height = sensor_info->height;

	turning_data.sensor_data.lines_per_second = Vmax * 2 * sensor_info->fps;
	turning_data.sensor_data.exposure_time_long_max = fsc - (rhs1 + 9);
	turning_data.sensor_data.exposure_time_max = rhs1 - 9;
	turning_data.sensor_data.exposure_time_min = 8;
	turning_data.sensor_data.analog_gain_max = 255;
	turning_data.sensor_data.digital_gain_max = 0;


	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_GB, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx415_stream_on_setting)) {
		memcpy(stream_on, imx415_stream_on_setting, sizeof(imx415_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx415_stream_off_setting)) {
		memcpy(stream_off, imx415_stream_off_setting, sizeof(imx415_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// sync gain lut to kernel driver.
	turning_data.dol2.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut, imx415_gain_lut,
			sizeof(imx415_gain_lut));
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

	if (turning_data.dol2.again_lut) {
		free(turning_data.dol2.again_lut);
		turning_data.dol2.again_lut = NULL;
	}

	if (ret < 0) {
		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}
	vin_info("imx415_dol2_data_init success\n");

	return ret;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(imx415, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx415 = {
	.module = SENSOR_MNAME(imx415),
#else
sensor_module_t imx415 = {
	.module = "imx415",
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
