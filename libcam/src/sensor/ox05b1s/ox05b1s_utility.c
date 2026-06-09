/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)             "[ox05b1s]:" fmt

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
#include "inc/ox05b1s_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define MCLK (24000000)
//#define AE_DBG

// AEC (exposure time control)
#define OX05B1S_EXP_LINE0 0x3501  // 0x3501[7:0] = AEC[15:8]
#define OX05B1S_EXP_LINE1 0x3502  // 0x3502[7:0] = AEC[7:0]
#define OX05B1S_FPS 10
#define OX05B1S_IMAGE_WIDTH 2592
#define OX05B1S_IMAGE_HEIGHT 1944
#define OX05B1S_FRAME_LENTH 2128  // VTS
#define OX05B1S_LINE_LENTH  752   // HTS
#define OX05B1S_PLL2_SA1_CLK  48  // 48MHz
#define OX05B1S_LINE_ROW_PERIODS ((float)OX05B1S_LINE_LENTH / OX05B1S_PLL2_SA1_CLK)   // 15.67us .Forced conversion to floating point retains data precision
#define OX05B1S_MAX_EXPOSURE_TIME ((int)(OX05B1S_FRAME_LENTH - 30))  // 2128 - 30 * 15.67 = 1658
#define OX05B1S_MIN_EXPOSURE_TIME 6  // According to actual testing, the value obtained is 6 lines.
#define OX05B1S_LINES_PER_SECOND (OX05B1S_FRAME_LENTH * OX05B1S_FPS)  // 2128 * 30 = 63840
#define SENSOR0_I2C_BUS 4
#define SENSOR1_I2C_BUS 2

// AGC\DGC (gain control)
#define OX05B1S_AGIN_REG0 0x3508     // 0x3508[3:0] = real_gain[7:4]
#define OX05B1S_AGIN_REG1 0x3509     // 0x3509[7:4] = real_gain[3:0]
#define OX05B1S_AGIN_REG0_MASK 0xF0  // [7:4]
#define OX05B1S_AGIN_REG1_MASK 0xF   // [3:0]

#define OX05B1S_DGIN_REG0 0x350A       // 0x350A[3:0] = real_Dgain[14:11]
#define OX05B1S_DGIN_REG1 0x350B       // 0x350B[7:0] = real_Dgain[10:2]
#define OX05B1S_DGIN_REG2 0x350C       // 0x350C[7:6] = real_Dgain[1:0]
#define OX05B1S_DGIN_REG0_MASK 0x3C00  // [14:11]
#define OX05B1S_DGIN_REG1_MASK 0x3FC   // [10:2]
#define OX05B1S_DGIN_REG2_MASK 0x3     // [1:0]
#define OX05B1S_AGIN_MIN 1             // 1, (0x3508 = 0x01, 0x3509 = 0x00)/16
#define OX05B1S_AGIN_MAX 126
#define OX05B1S_DGIN_MIN 1             // 1,(0x350A = 0x01, 0x350B = 0x00, 0x350C = 0x00)/1024
#define OX05B1S_DGIN_MAX 128

const char* sensor_mode_strings[] = {
    "Unknown Mode",  // Index 0, not used
    "NORMAL_M",
    "DOL2_M",
    "DOL3_M",
    "DOL4_M",
    "PWL_M",
    "SLAVE_M",
    "MONO_M",
    "INVALID_MOD"
};

// turning data init
int ox05b1s_linear_data_init_2592x1944(sensor_info_t *sensor_info)
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
	// lines_per_second = fps * vts, vts = {16‘h320e,16 ’ h320f} = 1400
	// If trigger is enabled, the configuration before trigger will still be used.
	turning_data.sensor_data.lines_per_second = OX05B1S_LINES_PER_SECOND;
	// form OX05B1S, exposure time max = 30ms
	turning_data.sensor_data.exposure_time_max = OX05B1S_MAX_EXPOSURE_TIME;

	turning_data.sensor_data.active_width = OX05B1S_IMAGE_WIDTH;
	turning_data.sensor_data.active_height = OX05B1S_IMAGE_HEIGHT;
	turning_data.sensor_data.analog_gain_max = OX05B1S_AGIN_MAX;	//16
	turning_data.sensor_data.digital_gain_max = OX05B1S_DGIN_MAX;   //16
	turning_data.sensor_data.exposure_time_min = OX05B1S_MIN_EXPOSURE_TIME;
	// No setting is required in linear mode
	// turning_data.sensor_data.exposure_time_long_max = 2079;  // DOL2 only
	turning_data.sensor_data.analog_gain_init = 1;
	turning_data.sensor_data.digital_gain_init = 1;
	turning_data.sensor_data.exposure_time_init = 638;  // 638/15.67 = 10ms
	turning_data.sensor_data.delta_time = 2;  // Sensor's base exposure time, default is 0 line

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_RGBIR_4x4_START_BGGIR, (uint32_t)BAYER_PATTERN_GRBIR_4X4);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ox05b1s_stream_on_setting)) {
		memcpy(stream_on, ox05b1s_stream_on_setting, sizeof(ox05b1s_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ox05b1s_stream_off_setting)) {
		memcpy(stream_off, ox05b1s_stream_off_setting, sizeof(ox05b1s_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, ox05b1s_gain_lut,
			sizeof(ox05b1s_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ox05b1s_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, ox05b1s_dgain_lut,
			sizeof(ox05b1s_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ox05b1s_dgain_lut)/sizeof(uint32_t); open_cnt++) {
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

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:	  // 1: normal
			vin_info("ox05b1s is in normal mode.\n");
			#if 1  // 2 lane
			vin_info("ox05b1s is in 2 lane config.\n");
			setting_size = sizeof(ox05b1s_linear_init_2592x1944_10fps_setting_master_2lane) / sizeof(uint32_t) / 2;
			ret = sensor_configure(sensor_info, ox05b1s_linear_init_2592x1944_10fps_setting_master_2lane, setting_size);
			#else  // 4 lane
			vin_info("ox05b1s is in 4 lane config\n");
			setting_size = sizeof(ox05b1s_linear_init_2592x1944_30fps_setting_master_4lane) / sizeof(uint32_t) / 2;
			ret = sensor_configure(sensor_info, ox05b1s_linear_init_2592x1944_30fps_setting_master_4lane, setting_size);
			#endif

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}

			ret = ox05b1s_linear_data_init_2592x1944(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			break;
		case SLAVE_M:	  // 6: slave
			vin_info("ox05b1s is in slave mode.\n");
			#if 1 // 2 lane
			vin_info("ox05b1s is in 2 lane config\n");
			setting_size = sizeof(ox05b1s_linear_init_2592x1944_10fps_setting_slave_2lane) / sizeof(uint32_t) / 2;
			ret = sensor_configure(sensor_info, ox05b1s_linear_init_2592x1944_10fps_setting_slave_2lane, setting_size);
			#else  // 4 lane
			vin_info("ox05b1s is in 4 lane config\n");
			setting_size = sizeof(ox05b1s_linear_init_2592x1944_30fps_setting_slave_4lane) / sizeof(uint32_t) / 2;
			ret = sensor_configure(sensor_info, ox05b1s_linear_init_2592x1944_30fps_setting_slave_4lane, setting_size);
			#endif

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			ret = ox05b1s_linear_data_init_2592x1944(sensor_info);
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

	vin_info("ox05b1s config success under %s mode\n\n", sensor_mode_strings[sensor_info->sensor_mode]);

	return ret;
}
// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			setting_size = sizeof(ox05b1s_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ox05b1s_stream_on_setting);
			if(ret < 0) {
					vin_err("start %s fail\n", sensor_info->sensor_name);
					return ret;
			}
			break;
		case SLAVE_M:
			setting_size = sizeof(ox05b1s_stream_on_setting)/sizeof(uint32_t)/2;
			vin_info(" start linear mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ox05b1s_stream_on_setting);
			if(ret < 0) {
					vin_err("start %s fail\n", sensor_info->sensor_name);
					return ret;
			}
			break;
		case DOL2_M:
			// setting_size = sizeof(ox05b1s_stream_on_setting)/sizeof(uint32_t)/2;
			// vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
			// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
			// 		setting_size, ox05b1s_stream_on_setting);
			// if(ret < 0) {
			// 		vin_err("start %s fail\n", sensor_info->sensor_name);
			// 		return ret;
			// }
			break;
	}

	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;

		setting_size =
				sizeof(ox05b1s_stream_off_setting) / sizeof(uint32_t) / 2;
		vin_info("sensor stop sensor_name %s, setting_size = %d\n",
				sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num,
							 sensor_info->sensor_addr, 2,
							 setting_size, ox05b1s_stream_off_setting);
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
	printf("%s %s mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	char ana_gain = 0, ana_fine_gain = 0;
	char dig_gain = 0, dig_fine_gain = 0;
	int again_index = 0, dgain_index = 0;
	int i = 0;
	char again_value_0 = 0, again_value_1 = 0;
	char dgain_value_0 = 0, dgain_value_1 = 0, dgain_value_2 = 0;

	if(info->bus_num != SENSOR0_I2C_BUS) {
		vin_info("The sensor in bus %d is SENSOR_1, which needs to be controlled simultaneously \
				when SENSOR_0 is controlled.\n", info->bus_num);
		return 0;
	}

	vin_info("Current sensor is SENSOR_0 in bus %d,need to control SENSOR_1 at the same time.\n",
			info->bus_num);

	if ((mode == NORMAL_M) || (mode == DOL2_M) || (mode == SLAVE_M)) {

		if (again[0] >= sizeof(ox05b1s_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(ox05b1s_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(ox05b1s_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(ox05b1s_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		again_value_0 = (ox05b1s_gain_lut[again_index] & OX05B1S_AGIN_REG0_MASK) >> 4;
		again_value_1 = (ox05b1s_gain_lut[again_index] & OX05B1S_AGIN_REG1_MASK) << 4;
		dgain_value_0 = (ox05b1s_dgain_lut[dgain_index] & OX05B1S_DGIN_REG0_MASK) >> 10;
		dgain_value_1 = (ox05b1s_dgain_lut[dgain_index] & OX05B1S_DGIN_REG1_MASK) >> 2;
		dgain_value_2 = ox05b1s_dgain_lut[dgain_index] & OX05B1S_DGIN_REG2_MASK;

		vin_info("ox05b1s_gain_lut[%d] = 0x%x ,again_value_0 = 0x%x, again_value_1 = 0x%x\n ox05b1s_dgain_lut[%d] = 0x%x, dgain_value_0 = 0x%x, dgain_value_1 = 0x%x, dgain_value_2 = 0x%x\n",
			again_index, ox05b1s_gain_lut[again_index], again_value_0, again_value_1, dgain_index, ox05b1s_dgain_lut[dgain_index], dgain_value_0, dgain_value_1, dgain_value_2);
#ifdef AE_DBG
		vin_info("%s again(0x3e08/0x3e09):%x,%x; dgain(0x3e06x3e07):%x,%x\n",
				__FUNCTION__, ana_gain, ana_fine_gain, dig_gain, dig_fine_gain);
#endif

		// vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OX05B1S_AGIN_REG0, again_value_0);
		// vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OX05B1S_AGIN_REG1, again_value_1);
		// vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OX05B1S_DGIN_REG0, dgain_value_0);
		// vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OX05B1S_DGIN_REG1, dgain_value_1);
		// vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OX05B1S_DGIN_REG2, dgain_value_2);

		// set sensor0 gain
		vin_i2c_write8(SENSOR0_I2C_BUS, 16, info->sensor_addr, OX05B1S_AGIN_REG0, again_value_0);
		vin_i2c_write8(SENSOR0_I2C_BUS, 16, info->sensor_addr, OX05B1S_AGIN_REG1, again_value_1);
		vin_i2c_write8(SENSOR0_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG0, dgain_value_0);
		vin_i2c_write8(SENSOR0_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG1, dgain_value_1);
		vin_i2c_write8(SENSOR0_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG2, dgain_value_2);

		// set sensor1 gain
		vin_i2c_write8(SENSOR1_I2C_BUS, 16, info->sensor_addr, OX05B1S_AGIN_REG0, again_value_0);
		vin_i2c_write8(SENSOR1_I2C_BUS, 16, info->sensor_addr, OX05B1S_AGIN_REG1, again_value_1);
		vin_i2c_write8(SENSOR1_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG0, dgain_value_0);
		vin_i2c_write8(SENSOR1_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG1, dgain_value_1);
		vin_i2c_write8(SENSOR1_I2C_BUS, 16, info->sensor_addr, OX05B1S_DGIN_REG2, dgain_value_2);

		// if (mode == DOL2_M) {
		// 	vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3e12, ana_gain);
		// 	vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3e13, ana_fine_gain);
		// 	vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3e10, dig_gain);
		// 	vin_i2c_write8(info->bus_num, 16, info->sensor_addr, 0x3e11, dig_fine_gain);
		// }

	} else	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

/**
 * @brief: set exposure time.Control Sensor0 and Sensor1 to synchronous exposure.
 * @param bus: i2c bus number
 * @param addr: sensor address
 * @param line: exposure time
 * 
 * AEC register: 16-bit value
 * 0x3501[7:0] Save high bytes, AEC[15:8]
 * 0x3502[7:0] Save low bytes, AEC[7:0]
 */
static int ox05b1s_ae_set(uint32_t bus, uint32_t addr, uint32_t line)
{
	char temp0 = 0, temp1 = 0, temp2 = 0;

	uint32_t sline = line;

	if(bus != SENSOR0_I2C_BUS) {
		vin_info("The sensor in bus %d is SENSOR_1, which needs to be controlled simultaneously when SENSOR_0 is controlled.\n", bus);
		return 0;
	}

	vin_info("Current sensor is SENSOR_0 in bus %d,need to control SENSOR_1 at the same time.\n", bus);
	if (sline >= OX05B1S_MAX_EXPOSURE_TIME) {
		sline = OX05B1S_MAX_EXPOSURE_TIME;
	} else if (sline <= OX05B1S_MIN_EXPOSURE_TIME)
	{
		sline = OX05B1S_MIN_EXPOSURE_TIME;
	}

	temp0 = (sline & 0xFF00) >> 8;
	temp1 = (sline & 0xFF);

	// vin_i2c_write8(bus, 16, addr, OX05B1S_EXP_LINE0, temp0);
	// vin_i2c_write8(bus, 16, addr, OX05B1S_EXP_LINE1, temp1);

	// set sensor0 exposure time
	vin_i2c_write8(SENSOR0_I2C_BUS, 16, addr, OX05B1S_EXP_LINE0, temp0);
	vin_i2c_write8(SENSOR0_I2C_BUS, 16, addr, OX05B1S_EXP_LINE1, temp1);
	// set sensor1 exposure time
	vin_i2c_write8(SENSOR1_I2C_BUS, 16, addr, OX05B1S_EXP_LINE0, temp0);
	vin_i2c_write8(SENSOR1_I2C_BUS, 16, addr, OX05B1S_EXP_LINE1, temp1);

#ifdef AE_DBG
	printf("%s sline = %d, OX05B1S_EXP_LINE0 = %x, OX05B1S_EXP_LINE1 = %x\n",
		__FUNCTION__, sline, temp0, temp1);
#endif

	return 0;
}

#define SAMPLECNT 8

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	printf(" line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
	uint32_t val;


	if ((mode == NORMAL_M) || (mode == SLAVE_M)) {
		val = line[0];
		ox05b1s_ae_set(info->bus_num, info->sensor_addr, val);
	} else if (mode == DOL2_M) {
		//todo
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
SENSOR_MODULE_F(ox05b1s, CAM_MODULE_FLAG_A16D8);
sensor_module_t ox05b1s = {
		.module = SENSOR_MNAME(ox05b1s),
#else
sensor_module_t ox05b1s = {
		.module = "ox05b1s",
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
