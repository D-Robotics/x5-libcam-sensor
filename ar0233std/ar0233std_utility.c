/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0233std]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/shm.h>

#include "inc/hb_vin.h"
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "inc/ar0233std_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define VERSION_SENSING "0.1.0"
#define VERSION_WISSEN  "0.1.0"
#define FPS_HTS
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)

#define CONFIG_INDEX_ALL ( \
		TEST_PATTERN | \
		TRIG_STANDARD | \
		TRIG_SHUTTER_SYNC \
		)
emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SENSING_M27F216D12_S0R0T7] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M27F100D12_S0R0T7] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_M25F100D12_S2R7T8] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_M25F123D12_S2R5T8] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_M27F100D12_S0R0T7, "0.0.1", "lib_ar0233RGGB_Pwl12_SY_Fov216.so", "0.21.8.26", &emode_data[SENSING_M27F100D12_S0R0T7]),
	SENSOR_EMADD(SENSING_M27F216D12_S0R0T7, "0.0.1", "lib_ar0233RGGB_Pwl12_SY_Fov216.so", "0.21.8.26", &emode_data[SENSING_M27F216D12_S0R0T7]),
	SENSOR_EMADD(SUNNY_M25F100D12_S2R7T8, "0.0.1", "lib_ar0233RGGB_Pwl12_WS_Fov100.so", "0.22.9.13", &emode_data[SUNNY_M25F100D12_S2R7T8]),
	SENSOR_EMADD(SUNNY_M25F123D12_S2R5T8, "0.0.1", "lib_CA23GB_pwl12_OF_Fov123.so", "0.22.08.31", &emode_data[SUNNY_M25F123D12_S2R5T8]),

	SENSOR_EMEND(),
};

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);
static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_TEST_PATTERN] = sensor_config_index_test_pattern,
	[B_TRIG_STANDARD] = sensor_config_index_trig_mode,
	[B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
};

int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	vin_dbg("ar0233_test_pattern 0x%04x\n", ar0233_test_pattern[1]);
	setting_size = sizeof(ar0233_test_pattern)/sizeof(uint32_t)/2;
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						    sensor_addr,
						    ar0233_test_pattern[i*2],
						    ar0233_test_pattern[i*2 + 1]);
		if (ret < 0) {
			vin_err("write ar0233_test_pattern error\n");
		}
	}
	return ret;
}

int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(((deserial_module_t *)(deserial_if->deserial_ops))->ops.max);
#endif
	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ret;
	}
	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_id = 1;
	else
		gpio_id = sensor_info->deserial_port;

	ret = maxops->mfp_cfg(deserial_if, GPIO_TX_GMSL, gpio_id, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("%s trig_pin config fail\n", deserial_if->deserial_name);
		return -1;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	return  ret;
}

int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(((deserial_module_t *)(deserial_if->deserial_ops))->ops.max);
#endif
	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ret;
	}
	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_id = 1;
	else
		gpio_id = sensor_info->deserial_port;

	ret = maxops->mfp_cfg(deserial_if, GPIO_TX_GMSL, gpio_id, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("%s trig_pin config fail\n", deserial_if->deserial_name);
		return -1;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	return  ret;
}

int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	uint32_t gpio;
	int32_t ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
						     sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret = (int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
						1-sensor_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);
			}
		}
	}
	return ret;
}

void sensor_common_data_init(sensor_info_t *sensor_info,
		sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
		sizeof(turning_data->sensor_name) - 1);
	return;
}

/**
 * @brief sensor_param_init : read sensor VTS, HTS, X/Y_START/END reg
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 * @param [in] turning_data : store sensor reg
 *
 * @return ret
 */
int32_t sensor_param_init(sensor_info_t *sensor_info,
			  sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	char init_d[3];
	uint32_t x0, m_y0, x1, m_y1, width, height;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_VTS, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) |
					  (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_HTS, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) |
					 (uint32_t)init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_X_START, init_d, 2);
	x0 = init_d[0];
	x0 = (x0 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_Y_START, init_d, 2);
	m_y0 = init_d[0];
	m_y0 = (m_y0 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_X_END, init_d, 2);
	x1 = init_d[0];
	x1 = (x1 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  AR0233_Y_END, init_d, 2);
	m_y1 = init_d[0];
	m_y1 = (m_y1 << 8) | (uint32_t)init_d[1];
	width = x1 - x0 + 1;
	height = m_y1 - m_y0 + 1;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;
	turning_data->sensor_data.gain_max = 128 * 8192;
	turning_data->sensor_data.analog_gain_max = 128*8192;
	turning_data->sensor_data.digital_gain_max = 64*8192;
	turning_data->sensor_data.exposure_time_min = 50;
	turning_data->sensor_data.exposure_time_max = 4000;
	turning_data->sensor_data.exposure_time_long_max = 4000;
	if ((sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS == 1) {/* zu3 2fps */
		turning_data->sensor_data.lines_per_second = 12752;
	} else {
		turning_data->sensor_data.lines_per_second = sensor_info->fps *
							     turning_data->sensor_data.VMAX;
	}
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;

	sensor_data_bayer_fill(&turning_data->sensor_data, 12,
			       (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data->sensor_data, 20);

	return ret;
}

/**
 * @brief sensor_stream_control_set : store stream on setting
 *
 * @param [in] turning_data : store sensor reg
 *
 * @return ret
 */
static int32_t sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ar0233_stream_on_setting)) {
		memcpy(stream_on, ar0233_stream_on_setting, sizeof(ar0233_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");/* PRQA S 2880 */
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ar0233_stream_off_setting)) {
		memcpy(stream_off, ar0233_stream_off_setting, sizeof(ar0233_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");		/* PRQA S 2880 */
		return -RET_ERROR;
	}
	return ret;
}

int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t open_cnt;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}

	turning_data.normal.param_hold = AR0233_PARAM_HOLD;
	turning_data.normal.param_hold_length = 2;
	turning_data.normal.s_line = AR0233_LINE;
	turning_data.normal.s_line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.normal.line_p.ratio = 1 << 8;
	turning_data.normal.line_p.offset = 0;
	turning_data.normal.line_p.max = 4000;
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = AR0233_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = AR0233_DGAIN;
	turning_data.normal.dgain_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));	/* PRQA S 5118 */
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, ar0233_again_lut,
			sizeof(ar0233_again_lut));
		for (open_cnt =0; open_cnt < sizeof(ar0233_again_lut)/
			sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));	/* PRQA S 5118 */
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut,
			ar0233_dgain_lut, sizeof(ar0233_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ar0233_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", sensor_info->port, ret);
		return -RET_ERROR;
	}
	if (turning_data.normal.again_lut)
		free(turning_data.normal.again_lut);	/* PRQA S 5118 */
	if (turning_data.normal.dgain_lut)
		free(turning_data.normal.dgain_lut);	/* PRQA S 5118 */

	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
#ifdef COMP_XJ3_CAM
	sensor_turning_data_ex_t turning_data_ex;
#endif
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
#ifdef COMP_XJ3_CAM
	memset(&turning_data_ex, 0, sizeof(sensor_turning_data_ex_t));
#endif
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}
	turning_data.pwl.param_hold = AR0233_PARAM_HOLD;
	turning_data.pwl.param_hold_length = 1;
	turning_data.pwl.line = AR0233_LINE;
	turning_data.pwl.line_length = 2;

	turning_data.pwl.line_ext[0] = AR0233_LINE;
	turning_data.pwl.line_length_ext[0] = 2;
	turning_data.pwl.line_ext[1] = AR0233_LINE2;
	turning_data.pwl.line_length_ext[1] = 2;
	turning_data.pwl.l_s_mode = 0;
	turning_data.pwl.line_num = 0;
	turning_data.pwl.line_p_ext[0].ratio = 1 << 16;
	turning_data.pwl.line_p_ext[0].offset = 0;
	turning_data.pwl.line_p_ext[0].max = 4000;
	turning_data.pwl.line_p_ext[0].min = 40;
	turning_data.pwl.line_p_ext[1].ratio = 655;
	turning_data.pwl.line_p_ext[1].offset = 0;
	turning_data.pwl.line_p_ext[1].max = 4000;
	turning_data.pwl.line_p_ext[1].min = 5;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
		turning_data.pwl.line_p.ratio = 1 << 8;
		turning_data.pwl.line_p.offset = 0;
		turning_data.pwl.line_p.max = 4000;

		turning_data.pwl.again_control_num = 1;
		turning_data.pwl.again_control[0] = AR0233_GAIN;
		if (sensor_info->extra_mode == SUNNY_M25F123D12_S2R5T8) {
			turning_data.pwl.again_control_length[0] = 0;
		} else {
			turning_data.pwl.again_control_length[0] = 2;
		}
		turning_data.pwl.dgain_control_num = 1;
		turning_data.pwl.dgain_control[0] = AR0233_DGAIN;
		turning_data.pwl.dgain_control_length[0] = 2;
		turning_data.pwl.again_lut = malloc(256*1*sizeof(uint32_t));	/* PRQA S 5118 */
		if (turning_data.pwl.again_lut != NULL) {
			memset(turning_data.pwl.again_lut, 0xff, 256*1*sizeof(uint32_t));
			if (sensor_info->extra_mode == SUNNY_M25F123D12_S2R5T8) {
				memcpy(turning_data.pwl.again_lut, ar0233_again_lut_ga,
				       sizeof(ar0233_again_lut_ga));
				for (open_cnt =0; open_cnt <
				     sizeof(ar0233_again_lut_ga)/sizeof(uint32_t); open_cnt++) {
					VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
				}
			} else {
				memcpy(turning_data.pwl.again_lut, ar0233_again_lut,
				       sizeof(ar0233_again_lut));
				for (open_cnt =0; open_cnt <
				     sizeof(ar0233_again_lut)/sizeof(uint32_t); open_cnt++) {
					VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
				}
			}
		}
		turning_data.pwl.dgain_lut = malloc(256*1*sizeof(uint32_t));	/* PRQA S 5118 */
		if (turning_data.pwl.dgain_lut != NULL) {
			memset(turning_data.pwl.dgain_lut, 0xff, 256*1*sizeof(uint32_t));

			memcpy(turning_data.pwl.dgain_lut, ar0233_dgain_lut,
				sizeof(ar0233_dgain_lut));
			for (open_cnt =0; open_cnt <
				sizeof(ar0233_dgain_lut)/sizeof(uint32_t); open_cnt++) {
				VIN_DOFFSET(&turning_data.pwl.dgain_lut[open_cnt], 2);
			}
		}

		turning_data.sensor_awb.bgain_addr[0] = 0x3058;
		turning_data.sensor_awb.bgain_length[0] = 2;
		turning_data.sensor_awb.rgain_addr[0] = 0x305a;
		turning_data.sensor_awb.rgain_length[0] = 2;
		turning_data.sensor_awb.grgain_addr[0] = 0x3056;
		turning_data.sensor_awb.grgain_length[0] = 2;
		turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
		turning_data.sensor_awb.gbgain_length[0] = 2;

		turning_data.sensor_awb.rb_prec = 7;

		/* awb and dgain share the gain logic for ga0233 */
		if (sensor_info->extra_mode == SUNNY_M25F123D12_S2R5T8) {
			turning_data.sensor_awb.apply_lut_gain = 1;
		} else {
			turning_data.sensor_awb.apply_lut_gain = 0;
		}

#endif
#ifdef COMP_XJ3_CAM
	if (turning_data_ex.ratio_en == 1) {
		ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM_EX, &turning_data_ex);
		if (ret < 0) {
			vin_err("SENSOR_TURNING_PARAM_EX ioctl fail %d\n", ret);
			return -RET_ERROR;
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("SENSOR_TURNING_PARAM ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);	/* PRQA S 5118 */
		turning_data.pwl.again_lut = NULL;
	}
	if (turning_data.pwl.dgain_lut) {
		free(turning_data.pwl.dgain_lut);	/* PRQA S 5118 */
		turning_data.pwl.dgain_lut = NULL;
	}

	return ret;
}

static int32_t sensor_0233_res_fix(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size;
	uint32_t bus = sensor_info->bus_num;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	if (sensor_info->width !=0) {
		ar0233_width_1920_init_setting[1] = (uint16_t)(1032 - (sensor_info->width / 2));
		ar0233_width_1920_init_setting[3] = (uint16_t)(1032 + (sensor_info->width / 2) - 1);

		vin_dbg("%s width %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,		/*PRQA S 1860*/
			ar0233_width_1920_init_setting[3] - ar0233_width_1920_init_setting[1] + 1,
			ar0233_width_1920_init_setting[1], ar0233_width_1920_init_setting[3]);
		setting_size = sizeof(ar0233_width_1920_init_setting)/sizeof(uint32_t)/2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr,
							    ar0233_width_1920_init_setting[i*2],
							    ar0233_width_1920_init_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("write ar0233_width_1920_init_setting error\n");
			}
		}
	}

	if (sensor_info->height !=0) {
		ar0233_height_1080_init_setting[1] = (uint16_t)(644 - (sensor_info->height / 2));
		ar0233_height_1080_init_setting[3] = (uint16_t)(644 + (sensor_info->height / 2) - 1);
		vin_dbg("%s height %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,		/*PRQA S 1860*/
			ar0233_height_1080_init_setting[3] - ar0233_height_1080_init_setting[1] + 1,
			ar0233_height_1080_init_setting[1], ar0233_height_1080_init_setting[3]);
		setting_size = sizeof(ar0233_height_1080_init_setting)/sizeof(uint32_t)/2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr,
							    ar0233_height_1080_init_setting[i*2],
							    ar0233_height_1080_init_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("write ar0233_height_1080_init_setting error\n");
			}
		}
	}
	return ret;
}

/**
 * @brief sensor_0233_linear_init : sensor linear mode
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_0233_linear_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size, tmp = 0;

	setting_size = sizeof(ar0233_x3_30fps_linear_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("x3 setting_size %d\n", setting_size);
	for(i = 0; i < setting_size; i++) {
		if ((sensor_info->extra_mode & EXT_MASK) == 0) { /* xj3 / j5 */
#ifdef FPS_HTS
			if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_HTS
				&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
				ar0233_x3_30fps_linear_init_setting[i*2 + 1] = (uint16_t)(50160 / sensor_info->fps);
#else
			if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_VTS
				&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
				ar0233_x3_30fps_linear_init_setting[i*2 + 1] = 40380 / sensor_info->fps;
#endif
		} else if (((sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS) == 1) { /* zu3 2fps */
			if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_HTS)
				ar0233_x3_30fps_linear_init_setting[i*2 + 1] = 5293;
			else if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_VTS)
				ar0233_x3_30fps_linear_init_setting[i*2 + 1] = 6375;
		} else if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_VTS) {
			tmp = ((uint32_t)sensor_info->extra_mode) >> 16;
			vin_dbg("%s vts=%d(0x%04x)\n", sensor_info->sensor_name, tmp, tmp);
			ar0233_x3_30fps_linear_init_setting[i*2 + 1] = (uint16_t)tmp;
		} else if (ar0233_x3_30fps_linear_init_setting[i*2] == (uint16_t)AR0233_HTS) {
			tmp = ((uint32_t)sensor_info->extra_mode) & 0xffff;
			vin_dbg("%s hts=%d(0x%04x)\n", sensor_info->sensor_name, tmp, tmp);
			ar0233_x3_30fps_linear_init_setting[i*2 + 1] = (uint16_t)tmp;
		}
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				ar0233_x3_30fps_linear_init_setting[i*2], ar0233_x3_30fps_linear_init_setting[i*2 + 1]);
		if (ret < 0) {
			tmp++;
			if (tmp < 10) {
				i--;
				usleep(10*1000);
				continue;
			}
			vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__, sensor_info->sensor_name,
				sensor_info->bus_num, sensor_info->sensor_addr,
				i, ar0233_x3_30fps_linear_init_setting[i*2], ar0233_x3_30fps_linear_init_setting[i*2 + 1]);
			return ret;
		}
		if((i == 1) || (i == 1070))
			usleep(200*1000);
		tmp = 0;
	}

	ret = sensor_0233_res_fix(sensor_info);
	if (ret < 0)
		return ret;

	ret = sensor_linear_data_init(sensor_info);
	if (ret < 0)
		return ret;

	vin_dbg("sensor_0233_linear_init OK!\n");
	return ret;
}

/**
 * @brief sensor_0233_pwl_init : write sy/ws 0233 pwl setting
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_0233_pwl_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t setting_size, tmp = 0;

	usleep(100*1000);
	setting_size = sizeof(ar0233_init_setting_soft_reset_0)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
						  REG16_VAL16, setting_size,
						  ar0233_init_setting_soft_reset_0);
	if (ret < 0) {
		vin_err("senor %s soft reset_0 setting error\n", sensor_info->sensor_name);
		return ret;
	}
	usleep(100*1000);
	setting_size = sizeof(ar0233_init_setting_soft_reset_1)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
						  REG16_VAL16, setting_size,
						  ar0233_init_setting_soft_reset_1);
	if (ret < 0) {
		vin_err("senor %s soft reset_1 setting error\n", sensor_info->sensor_name);
		return ret;
	}
	if (sensor_info->resolution == 1080 && sensor_info->fps == 60) {
		setting_size = sizeof(ar0233_init_setting_1080p_60fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1080p_60fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1280 && sensor_info->fps == 45) {
		setting_size = sizeof(ar0233_init_setting_1280p_45fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1280p_45fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1280 && sensor_info->fps == 30) {
		if (deserial_if->lane_speed <= 1200) {
			vin_dbg("setting for des low speed\n");
			setting_size = sizeof(ar0233_init_setting_1280p_30fps_low)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
								  REG16_VAL16, setting_size,
								  ar0233_init_setting_1280p_30fps_low);
		} else {
			setting_size = sizeof(ar0233_init_setting_1280p_30fps)/sizeof(uint32_t)/2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
								  REG16_VAL16, setting_size,
								  ar0233_init_setting_1280p_30fps);
		}
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1280 && sensor_info->fps == 25) {
		setting_size = sizeof(ar0233_init_setting_1280p_25fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1280p_25fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1280 && sensor_info->fps == 20) {
		setting_size = sizeof(ar0233_init_setting_1280p_20fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1280p_20fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1280 && sensor_info->fps == 15) {
		setting_size = sizeof(ar0233_init_setting_1280p_15fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1280p_15fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->resolution == 1080 && sensor_info->fps == 30) {
		setting_size = sizeof(ar0233_init_setting_1080p_30fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_1080p_30fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else {
			vin_err("senor %s write resolution=%d--fps=%d setting not supported\n",
			sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
	}
	if (sensor_info->extra_mode == (uint32_t)SENSING_M27F100D12_S0R0T7 ||
		sensor_info->extra_mode == (uint32_t)SENSING_M27F216D12_S0R0T7) {
		setting_size = sizeof(ar0233_init_setting_27M)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_27M);
		if (ret < 0) {
			vin_err("senor %s write 27M pll setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->extra_mode == (uint32_t)SUNNY_M25F123D12_S2R5T8 ||
			   sensor_info->extra_mode == (uint32_t)SUNNY_M25F100D12_S2R7T8) {
		setting_size = sizeof(ar0233_init_setting_25M)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
							  REG16_VAL16, setting_size,
							  ar0233_init_setting_25M);
		if (ret < 0) {
			vin_err("senor %s write 25M pll setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
	}
	setting_size = sizeof(ar0233_base_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
						  REG16_VAL16, setting_size,
						  ar0233_base_init_setting);
	if (ret < 0) {
		vin_err("sensor %s write base setting error\n", sensor_info->sensor_name);
		return ret;
	}
	ret = sensor_0233_res_fix(sensor_info);
	if (ret < 0)
		return ret;

	ret = sensor_pwl_data_init(sensor_info);
	if(ret < 0) {
		vin_err("sensor_pwl_data_init %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	vin_dbg("sensor_0233_pwl_init OK!\n");

	return ret;
}

static int32_t sensor_config_special_timing(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t bus, sensor_addr;
	sensor_addr = sensor_info->sensor_addr;
	bus = sensor_info->bus_num;
	double ratio = 1.0;
	int32_t set = 0;
	uint8_t reg_vb[2];
	uint32_t reg_v, reg_vs;

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_hts_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, AR0233_HTS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts(0x%04x) %d to %d\n",
			ratio, AR0233_HTS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, AR0233_HTS, reg_vs);
		if (ret < 0) {
			vin_err("write AR0233_HTS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_vts_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, AR0233_VTS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("vts ratio %.2f setting: vts(0x%04x) %d to %d\n",
			ratio, AR0233_VTS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, AR0233_VTS, reg_vs);
		if (ret < 0) {
			vin_err("write AR0233_VTS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_pll_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, PLL_MULTIPLIER, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("pll ratio %.2f setting: pll_mult(0x%04x) %d to %d\n",
			ratio, PLL_MULTIPLIER, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, PLL_MULTIPLIER, reg_vs);
		if (ret < 0) {
			vin_err("write PLL_MULTIPLIER register error\n");
			return ret;
		}
		set++;
	}

	if (set == 0)
		vin_dbg("no special_timing prase\n");

	return RET_OK;
}

/**
 * @brief sensor_mode_config_init : use pwl mode by default
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;

	switch(sensor_info->sensor_mode) {
		case (uint32_t)NORMAL_M:  //  normal
			ret = sensor_0233_linear_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_0233_linear_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case (uint32_t)PWL_M:
			ret = sensor_0233_pwl_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_0233_pwl_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
		    vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}

	// test pattern enable

#if 0
	// fps div.
	if(sensor_info->config_index & FPS_DIV) {
		char init_d[3];
		uint32_t vts_v;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_VTS, init_d, 2);
		vts_v = ((uint32_t)init_d[0] << 8) | (uint32_t)init_d[1];
		vin_dbg("%dfps settint, vts %d to %d!\n", sensor_info->fps / 2, vts_v, vts_v * 2);
		vts_v *= 2;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_VTS, (uint16_t)vts_v);
		if (ret < 0)
			vin_err("write register error\n");
	}
#endif

	ret = sensor_config_special_timing(sensor_info);
	if (ret < 0) {
		vin_err("%s sensor special timing error\n", sensor_info->sensor_name);
		return ret;
	}

	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);
	if (ret < 0) {
		vin_err("sensor config_index do fail!!!\n");
		return ret;
	}
	return ret;
}

int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info)
{
		int32_t ret = RET_OK;

		switch(sensor_info->sensor_mode) {
			case (uint32_t)NORMAL_M:  //  normal
				ret = sensor_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("sensor_linear_data_init fail\n");
					return ret;
				}
				break;
			case (uint32_t)PWL_M:
				ret = sensor_pwl_data_init(sensor_info);
				if (ret < 0) {
					vin_err("sensor_dol2_update_notify_driver fail\n");
					return ret;
				}
				break;
			default:
				break;
		}
		return ret;
}

static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int32_t ret = RET_OK;
	int32_t xts;

	if (fps < 1 || sensor_info->fps > 30) {
		vin_err("%s %s %dfps not support\n", __func__, sensor_info->sensor_name, fps);
		return -RET_ERROR;
	}
	vin_dbg("%s %s %dfps\n", __func__, sensor_info->sensor_name, fps);
#ifdef FPS_HTS
	switch (sensor_info->sensor_mode) {
		case (uint32_t)NORMAL_M:
			xts = 50160 / fps;
			break;
		case (uint32_t)PWL_M:
			xts = 60000 / fps;
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
			AR0233_HTS, (uint16_t)xts);
#else
	switch (sensor_info->sensor_mode) {
		case NORMAL_M:
			xts = 40380 / fps;
			break;
		case PWL_M:
			xts = 51000 / fps;
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
			AR0233_VTS, xts);
#endif
	if(ret < 0) {
		vin_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
		return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
	}
	sensor_info->fps = fps;
	sensor_update_fps_notify_driver(sensor_info);
	vin_dbg("dynamic_switch to %dfps success\n", fps);
	return RET_OK;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t entry_num = sensor_info->entry_num;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port, drop\n", __func__);
	} else {
		if(sensor_info->sen_devfd <= 0) {
			char str[24] = {0};

			snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
			if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
				vin_err("port%d: %s open fail\n", sensor_info->port, str);
				return -RET_ERROR;
			}
		}
		vin_dbg("/dev/port_%d success sensor_info->sen_devfd %d===\n",
				sensor_info->dev_port, sensor_info->sen_devfd);
	}

	ret = max_serial_init(sensor_info);
	if (ret < 0) {
		vin_err("max_serial_init fail\n");
		return ret;
	}
	vin_dbg("0233 serializer init done\n");
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t setting_size = 0, i, req;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t ret = RET_OK, tmp = 0;
	int32_t entry_num = sensor_info->entry_num;

	if ((sensor_info->config_index & TRIG_STANDARD) ||
			(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if (sensor_info->start_state == CAM_STOP) {
			if (sensor_info->config_index & TRIG_STANDARD) {
				setting_size = sizeof(ar0233_sync_standard_restart_setting)/
					sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_sync_standard_restart_setting);
				if (ret < 0) {
					vin_err("senor %s write trigger mode setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
			} else {
				setting_size = sizeof(ar0233_sync_shutter_restart_setting)/
							sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_sync_shutter_restart_setting);
				if (ret < 0) {
					vin_err("senor %s write trigger mode setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
			}
			vin_dbg("senor %s restart sucessfully\n", sensor_info->sensor_name);
		} else {
			if (sensor_info->config_index & TRIG_STANDARD) {
				setting_size = sizeof(ar0233_trigger_standard_mode_setting)/
					sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_trigger_standard_mode_setting);
				if (ret < 0) {
					vin_err("senor %s write trigger mode setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->config_index & TRIG_SHUTTER_SYNC) {
				setting_size = sizeof(ar0233_trigger_shuttersync_mode_setting)/
					sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_trigger_shuttersync_mode_setting);
				if (ret < 0) {
					vin_err("senor %s write TRIG_SHUTTER_SYNC mode setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
				ret = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, AR0233_VTS);
				if (ret < 0) {
					vin_err("senor %s read VTS error\n", sensor_info->sensor_name);
					return ret;
				}
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, AR0233_VTS, (uint16_t)(ret - 1));
				if (ret < 0) {
					vin_err("senor %s write VTS error\n", sensor_info->sensor_name);
					return ret;
				}
			}
			if (sensor_info->extra_mode == (uint32_t)SENSING_M27F100D12_S0R0T7 ||
				sensor_info->extra_mode == (uint32_t)SENSING_M27F216D12_S0R0T7) {
				setting_size = sizeof(ar0233_trigger_gpio3_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_trigger_gpio3_setting);
				if (ret < 0) {
					vin_err("senor %s write trigger gpio3 setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
			} else if (sensor_info->extra_mode == (uint32_t)SUNNY_M25F100D12_S2R7T8 ||
					   sensor_info->extra_mode == (uint32_t)SUNNY_M25F123D12_S2R5T8) {
				setting_size = sizeof(ar0233_trigger_gpio1_setting)/sizeof(uint32_t)/2;
				ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
									  REG16_VAL16, setting_size,
									  ar0233_trigger_gpio1_setting);
				if (ret < 0) {
					vin_err("senor %s write trigger gpio1 setting error\n",
							sensor_info->sensor_name);
					return ret;
				}
			}
		}
	} else {
		setting_size = sizeof(ar0233_stream_on_setting)/sizeof(uint32_t)/2;
		vin_dbg("%s sensor_start setting_size %d\n",
				sensor_info->sensor_name, setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
					(uint8_t)sensor_info->sensor_addr, (uint16_t)ar0233_stream_on_setting[i*2],
					(uint16_t)ar0233_stream_on_setting[i*2 + 1]);
			if (ret < 0) {
				tmp++;
				if (tmp < 10) {
					i--;
					usleep(10*1000);
					continue;
				}
				vin_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
				return ret;
			}
			tmp = 0;
		}
	}
	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size = 0, i;
	uint8_t value;


		if ((sensor_info->config_index & TRIG_STANDARD) ||
			(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
			setting_size = sizeof(ar0233_sync_stream_off_setting)/sizeof(uint32_t)/2;
			vin_dbg("%s sensor_stop setting_size %d\n",
					sensor_info->sensor_name, setting_size);
			for(i = 0; i < setting_size; i++) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, (uint16_t)ar0233_sync_stream_off_setting[i*2],
						(uint16_t)ar0233_sync_stream_off_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
		} else {
			setting_size = sizeof(ar0233_stream_off_setting)/sizeof(uint32_t)/2;
			vin_dbg("%s sensor_stop setting_size %d\n",
					sensor_info->sensor_name, setting_size);
			for(i = 0; i < setting_size; i++) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, (uint16_t)ar0233_stream_off_setting[i*2],
						(uint16_t)ar0233_stream_off_setting[i*2 + 1]);
				if (ret < 0) {
					vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
					return ret;
				}
			}
			/* Increase the delay to ensure that there is enough time for the next stream on */
			usleep(50 * 1000);
		}
	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	uint32_t gpio;
	int32_t ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
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

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	return ret;
}

int get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	int ret = RET_OK;
	uint32_t x0, m_y0, x1, m_y1;
	int vt_pix_clk_div, vt_sys_clk_div, pre_pll_clk_div, pll_multiplier;
	uint64_t extclk = 27000000;

	if (!sp || !si) {
		vin_err("input sp|si is null!\n");
		return -RET_ERROR;
	}

	uint32_t i2c_num = si->bus_num;
	uint8_t i2c_addr = (uint8_t)(si->sensor_addr);
	sp->frame_length = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			AR0233_VTS);
	sp->line_length = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			AR0233_HTS);

	x0 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0233_X_START);
	m_y0 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0233_Y_START);
	x1 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0233_X_END);
	m_y1 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0233_Y_END);

	sp->width = x1 - x0 + 1;
	sp->height = m_y1 - m_y0 + 1;

	vt_pix_clk_div = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_PIX_CLK_DIV);
	vt_sys_clk_div = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_SYS_CLK_DIV) & 0x1F;
	pre_pll_clk_div = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PRE_PLL_CLK_DIV);
	pll_multiplier = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PLL_MULTIPLIER);

	if (si->extra_mode == (uint32_t)SENSING_M27F100D12_S0R0T7 ||
		si->extra_mode == (uint32_t)SENSING_M27F216D12_S0R0T7) {
		extclk = 27000000;
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));	/* PRQA S 2845 */
	} else {
		extclk = 25000000;
		strncpy(sp->version, VERSION_WISSEN, sizeof(sp->version));	/* PRQA S 2845 */
	}

	sp->pclk = (uint32_t)((extclk * (uint64_t)pll_multiplier) / ((uint64_t)pre_pll_clk_div *
			(uint64_t)vt_sys_clk_div * (uint64_t)vt_pix_clk_div));

	sp->fps = ((float)sp->pclk) / (sp->frame_length * sp->line_length);

	sp->exp_num = ((uint32_t)(hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
				REG_EXP_NUM) & 0xC) >> 2) + 1;

	sp->lines_per_second = (uint32_t)(sp->frame_length * sp->fps);

	return ret;
}

static uint8_t e2prom_i2c_addr;
int32_t hb_e2prom_read_data(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint64_t *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = hb_vin_i2c_read_reg16_data8((uint32_t)i2c_num, e2prom_i2c_addr,
				(uint16_t)(base_addr + i));
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val = (int32_t)((uint32_t)val & 0xff);
		ret <<= 8;
		ret |= (uint64_t)val;
	}
	*data = ret;
	return RET_OK;
}

int32_t hb_e2prom_read_double(int32_t i2c_num, int32_t base_addr, double *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 7; i >= 0; i --) {
		val = hb_vin_i2c_read_reg16_data8((uint32_t)i2c_num, e2prom_i2c_addr,
				(uint16_t)(base_addr + i));
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val = (int32_t)((uint32_t)val & 0xff);
		ret <<= 8;
		ret |= (uint64_t)val;
	}
	*data = *((double*)&ret);
	return RET_OK;
}

int32_t hb_e2prom_read_img_info(int32_t i2c_num, int32_t base_addr, uint64_t *data)
{
	int32_t val, ret = 0;

	val = hb_vin_i2c_read_reg16_data8((uint32_t)i2c_num, e2prom_i2c_addr,
			(uint16_t)base_addr);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr, val);
		return -RET_ERROR;
	}
	val = (int32_t)((uint32_t)val & 0xff);
	ret = val;

	val = hb_vin_i2c_read_reg16_data8((uint32_t)i2c_num, e2prom_i2c_addr,
			(uint16_t)(base_addr + 1));
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr + 1, val);
		return -RET_ERROR;
	}
	ret *= 100;
	ret += val;

	*data = (uint64_t)ret;

	return RET_OK;
}

int32_t hb_e2prom_read_array(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint8_t *data)
{
	int32_t i, val, ret = 0;
	for (i = 0; i < i2c_num - 1; i ++) {
		val = hb_vin_i2c_read_reg16_data8((uint32_t)i2c_num, e2prom_i2c_addr,
				(uint16_t)(base_addr + i));
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		data[i] = (uint8_t)val;
	}
	return RET_OK;
}

int32_t get_intrinsic_params(sensor_info_t *si,
		sensor_intrinsic_parameter_t *sip)
{
	int32_t i2c_num;
	uint64_t data;
	uint8_t serial_num[40] = {0};
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if;
	uint8_t eeprom_addr_alias_id;

	if (!sip || !si) {
		vin_err("input sip|si is null!\n");
		return -RET_ERROR;
	}
	i2c_num = si->bus_num;
	deserial_if = si->deserial_info;
	/* need set eeprom addr */
	eeprom_addr_alias_id = (uint8_t)(EEPROM_I2C_ADDR_ALIAS_ID + si->port);
	if (si->eeprom_addr == 0) {
		e2prom_i2c_addr = eeprom_addr_alias_id;
	} else {
		e2prom_i2c_addr = (uint8_t)(si->eeprom_addr);
	}
	if (e2prom_i2c_addr != eeprom_addr_alias_id)
		vin_warn("The eeprom_addr is not default (0x%x)\n", e2prom_i2c_addr);

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));

	if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_WIDTH_ADDR, &data)) < 0)
		return ret;
	sip->image_width = (uint16_t)data;

	if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_HEIGHT_ADDR, &data)) < 0)
		return ret;
	sip->image_height = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MAJOR_VERSION_ADDR, &data)) < 0)
		return ret;
	sip->major_version = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MINOR_VERSION_ADDR, &data)) < 0)
		return ret;
	sip->minor_version = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, VENDOR_ID_ADDR, &data)) < 0)
		return ret;
	sip->vendor_id = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, MODULE_ID_ADDR, &data)) < 0)
		return ret;
	sip->module_id = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 4, MODULE_SERIAL_ADDR, &data)) < 0)
		return ret;
	sip->module_serial = (uint32_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 2, YEAR_ADDR, &data)) < 0)
		return ret;
	sip->year = (uint16_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MONTH_ADDR, &data)) < 0)
		return ret;
	sip->month = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DAY_ADDR, &data)) < 0)
		return ret;
	sip->day = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR, &data)) < 0)
		return ret;
	sip->cam_type = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, MODULE_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->module_falg = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, EFL_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->efl_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, COD_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->cod_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, PP_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->pp_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORTION_FLAG_ADDR, &data)) < 0)
		return ret;
	sip->distortion_flag = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_PARAMS_ADDR, &data)) < 0)
		return ret;
	sip->distort_params = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_MODEL_TYPE_ADDR, &data)) < 0)
		return ret;
	sip->distort_model_type = (uint8_t)data;

	if ((ret = hb_e2prom_read_data(i2c_num, 4, CRC32_1_ADDR, &data)) < 0)
		return ret;
	sip->crc32_1 = (uint32_t)data;

	if ((ret = hb_e2prom_read_double(i2c_num, COD_X_ADDR, &sip->center_u)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_ADDR, &sip->center_v)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_ADDR, &sip->focal_u)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_ADDR, &sip->focal_v)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, FOV_ADDR, &sip->hfov)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, PP_X_ADDR, &sip->pp_x)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, PP_Y_ADDR, &sip->pp_y)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, CAM_SKEW_ADDR, &sip->cam_skew)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K1_ADDR, &sip->k1)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K2_ADDR, &sip->k2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, P1_ADDR, &sip->p1)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, P2_ADDR, &sip->p2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K3_ADDR, &sip->k3)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K4_ADDR, &sip->k4)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K5_ADDR, &sip->k5)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K6_ADDR, &sip->k6)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K7_ADDR, &sip->k7)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K8_ADDR, &sip->k8)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K9_ADDR, &sip->k9)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K10_ADDR, &sip->k10)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K11_ADDR, &sip->k11)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K12_ADDR, &sip->k12)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K13_ADDR, &sip->k13)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K14_ADDR, &sip->k14)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_2_ADDR, &sip->focal_u_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_2_ADDR, &sip->focal_v_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_X_2_ADDR, &sip->center_u_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_2_ADDR, &sip->center_v_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K1_2_ADDR, &sip->k1_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K2_2_ADDR, &sip->k2_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K3_2_ADDR, &sip->k3_2)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_double(i2c_num, K4_2_ADDR, &sip->k4_2)) < 0)
		return ret;

	if ((ret = hb_e2prom_read_array(i2c_num, 40, SERIAL_NUM_ADDR, sip->serial_num)) < 0)
		return ret;
	if ((ret = hb_e2prom_read_data(i2c_num, 4, CRC32_GROUP1_ADDR, &data)) < 0)
		return ret;
	sip->crc_group1 = (uint32_t)data;

	return RET_OK;
}

int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
{
	int32_t ret = RET_OK;

	switch (type) {
	case 0:
		ret = get_sensor_info(si, &csp->sns_param);
		break;
	case 1:
		ret = get_intrinsic_params(si, &csp->sns_intrinsic_param);
		break;
	case 3:
		ret = get_sensor_info(si, &csp->sns_param);
		if (ret == RET_OK)
			ret = get_intrinsic_params(si, &csp->sns_intrinsic_param);
		break;
	default:
		vin_err("ar0233 param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

#ifdef CAM_DIAG
static int32_t camera_ar0233_diag_voltage(diag_node_info_t *node)
{
	uint32_t vol = 0;
	int32_t fault = 0;
	uint8_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t addr = node->diag_info.t.reg.reg_dev;
	uint16_t reg_addr = node->diag_info.t.reg.reg_addr & 0xFFFF;
	vol = hb_vin_i2c_read_reg16_data16(bus, addr, reg_addr);
	if (vol > node->diag_info.t.reg.reg_max) {
		vin_err("%d@0x%x reg 0x%x vol high:%d\n",
			bus, addr, node->diag_info.t.reg.reg_addr, vol);
		fault = 1;
		return fault;
	} else if (vol < node->diag_info.t.reg.reg_min) {
		vin_err("%d@0x%x reg 0x%x vol low:%d\n",
			bus, addr, node->diag_info.t.reg.reg_addr, vol);
		fault = 1;
		return fault;
	}
	return fault;
}

static int32_t camera_ar0233_diag_voltage_fault_inject(diag_node_info_t *node, int32_t inject)
{
	uint32_t vol = 0;
	int32_t ret = RET_OK;

	if (inject == 1) {	  // Fault injection
		node->diag_info.t.reg.reg_max = node->diag_info.t.reg.reg_min;
	} else {		  // Fault clearance
		switch (node->diag_info.t.reg.reg_addr) {
		case DVDD_REGADDR :
			node->diag_info.t.reg.reg_max = DVDD_MAX;
			node->diag_info.t.reg.reg_min = DVDD_MIN;
			break;
		case VAA_REGADDR :
			node->diag_info.t.reg.reg_max = VAA_MAX;
			node->diag_info.t.reg.reg_min = VAA_MIN;
			break;
		case VAAPIX_REGADDR :
			node->diag_info.t.reg.reg_max = VAAPIX_MAX;
			node->diag_info.t.reg.reg_min = VAAPIX_MIN;
			break;
		case VDDIO_REGADDR :
			node->diag_info.t.reg.reg_max = VDDIO_MAX;
			node->diag_info.t.reg.reg_min = VDDIO_MIN;
			break;
		default:
			break;
		}
	}
	return ret;
}

int32_t sensor_voltage_nodes_register(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t port;
	diag_node_info_t *reg_node = NULL;

	port = sensor_info->port + 1;
	reg_node = cam_diag_get_nodes(4);
	if (reg_node == NULL) {
		vin_err("cam_diag_get_nodes failed\n");
		return -1;
	}
	// voltage reg VAA adccode
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0233_VAA_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = VAA_REGADDR;
	reg_node->diag_info.t.reg.reg_max = VAA_MAX;
	reg_node->diag_info.t.reg.reg_min = VAA_MIN;
	reg_node->fault_judging = camera_ar0233_diag_voltage;
	reg_node->test_fault_inject = camera_ar0233_diag_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VAAPIX
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0233_VAAPIX_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = VAAPIX_REGADDR;
	reg_node->diag_info.t.reg.reg_max = VAAPIX_MAX;
	reg_node->diag_info.t.reg.reg_min = VAAPIX_MIN;
	reg_node->fault_judging = camera_ar0233_diag_voltage;
	reg_node->test_fault_inject = camera_ar0233_diag_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VDD
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0233_DVDD_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = DVDD_REGADDR;
	reg_node->diag_info.t.reg.reg_max = DVDD_MAX;
	reg_node->diag_info.t.reg.reg_min = DVDD_MIN;
	reg_node->fault_judging = camera_ar0233_diag_voltage;
	reg_node->test_fault_inject = camera_ar0233_diag_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VDDIO
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0233_VDDIO_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = VDDIO_REGADDR;
	reg_node->diag_info.t.reg.reg_max = VDDIO_MAX;
	reg_node->diag_info.t.reg.reg_min = VDDIO_MIN;
	reg_node->fault_judging = camera_ar0233_diag_voltage;
	reg_node->test_fault_inject = camera_ar0233_diag_voltage_fault_inject;
	cam_diag_node_register(reg_node);

	vin_info("ar0233 diag reg_node init done\n");
	return 0;
}

int32_t sensor_diag_nodes_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	ret = sensor_voltage_nodes_register(sensor_info);
	vin_info("ar0233 diag node init done\n");
	return ret;
}
#endif  //  #ifdef CAM_DIAG

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(ar0233std, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D16);
sensor_module_t ar0233std = {
	.module = SENSOR_MNAME(ar0233std),
#else
sensor_module_t ar0233std = {
	.module = "ar0233std",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.get_sns_params = get_sns_info,
#ifdef CAM_DIAG
	.diag_nodes_init = sensor_diag_nodes_init,
#endif
};

