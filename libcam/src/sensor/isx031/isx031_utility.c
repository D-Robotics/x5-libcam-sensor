/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[isx031]:" fmt


//#define AE_DBG 				1

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "inc/isx031_setting.h"
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

#define SYNC_TRIGGER_FLAG 	(1 << 0)
#define PORTB_FLAG		(1 << 1)

static int32_t poc_addr = 0xff;
// static ISX031_RES_MODE_E current_res_mode = ISX031_RES_MODE_1536P;

int32_t write_register(uint32_t bus, uint32_t i2c_addr, int32_t reg_width,
					   int32_t setting_size, uint32_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int32_t ret = RET_OK;
	int i = 0, k, al, dl;
	uint32_t read_value;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = (uint32_t)reg_width;

	while (i < setting_size) {
		if ((cam_setting[i] == DELAY_FLAG || cam_setting[i] == DELAY1_FLAG) &&
			(i + 1) < setting_size) {
			uint32_t delay_ms = cam_setting[i + 1];
			usleep(delay_ms * 1000u);
			cam_dbg("Delay %ums executed.\n", delay_ms);
			i += 2;
			continue;
		}
		i2c_cfg.reg = cam_setting[i];
		i2c_cfg.data = cam_setting[i + 1];
		uint32_t expected_value = i2c_cfg.data;

		k = CAM_I2C_RETRY_MAX;
		int write_success = 0;
		int read_success = 0;

		do {
			/* 写入操作 */
			if (i2c_cfg.reg_size == REG16_VAL16) {
				al = 16;
				dl = 16;
				ret = camera_i2c_write_reg16_data16(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint16_t)i2c_cfg.data);
			} else if (i2c_cfg.reg_size == REG16_VAL8) {
				al = 16;
				dl = 8;
				ret = camera_i2c_write_reg16_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			} else {
				al = 8;
				dl = 8;
				ret = camera_i2c_write_reg8_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
			}

			if (ret != RET_OK) {
				cam_warn("Write failed: W%d %d@0x%02x: 0x%0*x = 0x%0*x retry %d\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, i2c_cfg.data,
					CAM_I2C_RETRY_MAX + 1 - k);
				camera_sys_msleep(20);
				continue;
			}

			write_success = 1;
			cam_dbg("Write success: W%d %d@0x%02x: 0x%0*x = 0x%0*x\n",
				al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, i2c_cfg.data);

#ifdef ENABLE_READBACK_VERIFICATION
			int32_t read_ret;
			if (i2c_cfg.reg_size == REG16_VAL16) {
				read_ret = camera_i2c_read_reg16_data16(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			} else if (i2c_cfg.reg_size == REG16_VAL8) {
				read_ret = camera_i2c_read_reg16_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			} else {
				read_ret = camera_i2c_read_reg8_data8(
					bus, (uint8_t)i2c_cfg.i2c_addr, (uint16_t)i2c_cfg.reg);
				if (read_ret >= 0) {
					read_value = (uint32_t)read_ret;
					ret = RET_OK;
				} else {
					ret = read_ret;
				}
			}

			if (ret != RET_OK) {
				cam_warn("Read failed: R%d %d@0x%02x: 0x%0*x retry %d\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg,
					CAM_I2C_RETRY_MAX + 1 - k);
				camera_sys_msleep(10);
				continue;
			}

			if (read_value == expected_value) {
				read_success = 1;
				cam_dbg("Verify success: R%d %d@0x%02x: 0x%0*x = 0x%0*x\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, read_value);
				break;
			} else {
				cam_warn("Verify mismatch: R%d %d@0x%02x: 0x%0*x = 0x%0*x (expected 0x%0*x)\n",
					al, bus, i2c_addr, al / 4, i2c_cfg.reg, dl / 4, read_value, dl / 4, expected_value);
				read_success = 1;
				break;
			}
#else
			read_success = 1;
			break;
#endif
		} while (k-- > 0);

		if (!write_success || !read_success) {
			cam_dbg("Operation failed: %s %d@0x%02x: 0x%0*x = 0x%0*x\n",
				(write_success ? "Read" : "Write"), bus, i2c_addr,
				al / 4, i2c_cfg.reg, dl / 4, expected_value);
			ret = RET_ERROR;
			break;
		}

		i += 2;
	}
	return ret;
}



int32_t sensor_poweron(sensor_info_t *sensor_info);

int32_t serdes_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	static int32_t bus;
	int32_t val_linka, val_linkb, val_linkc, val_linkd;
	int32_t val_addr_a, val_addr_b, val_addr_c, val_addr_d;

	vin_info("linsh %s\n", __func__);

	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	bus = sensor_info->bus_num;
	struct gmsl_config {
		uint32_t dev_addr;
		uint32_t *data;
		size_t size;
		const char *err_msg;
	};
	struct gmsl_config *configs;

	struct gmsl_config configs_linkA_auto[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe0_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe0_step1), "max96712 pipe0 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkA_step2,
		  ARRAY_SIZE(max96712_init_setting_linkA_step2), "max96712 linkA step2" },

		{ 0x40, max96717_init_0x40_linkA_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkA_addr_setting), "max96717 linkA addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x41_linkA_auto_setting,
		  ARRAY_SIZE(max96717_init_0x41_linkA_auto_setting), "max96717 linkA auto" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkA[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe0_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe0_step1), "max96712 pipe0 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkA_step2,
		  ARRAY_SIZE(max96712_init_setting_linkA_step2), "max96712 linkA step2" },

		{ 0x40, max96717_init_0x40_linkA_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkA_addr_setting), "max96717 linkA addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x41_linkA_sync_setting,
		  ARRAY_SIZE(max96717_init_0x41_linkA_sync_setting), "max96717 linkA" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkB_auto[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe1_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe1_step1), "max96712 pipe1 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkB_step2,
		  ARRAY_SIZE(max96712_init_setting_linkB_step2), "max96712 linkB step2" },

		{ 0x40, max96717_init_0x40_linkB_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkB_addr_setting), "max96717 linkB addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x42_linkB_auto_setting,
		  ARRAY_SIZE(max96717_init_0x42_linkB_auto_setting), "max96717 linkB auto" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkB[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe1_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe1_step1), "max96712 pipe1 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkB_step2,
		  ARRAY_SIZE(max96712_init_setting_linkB_step2), "max96712 linkB step2" },

		{ 0x40, max96717_init_0x40_linkB_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkB_addr_setting), "max96717 linkB addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x42_linkB_sync_setting,
		  ARRAY_SIZE(max96717_init_0x42_linkB_sync_setting), "max96717 linkB sync" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkC_auto[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe2_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe2_step1), "max96712 pipe2 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkC_step2,
		  ARRAY_SIZE(max96712_init_setting_linkC_step2), "max96712 linkC step2" },

		{ 0x40, max96717_init_0x40_linkC_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkC_addr_setting), "max96717 linkC addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x43_linkC_auto_setting,
		  ARRAY_SIZE(max96717_init_0x43_linkC_auto_setting), "max96717 linkC auto" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkC[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe2_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe2_step1), "max96712 pipe2 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkC_step2,
		  ARRAY_SIZE(max96712_init_setting_linkC_step2), "max96712 linkC step2" },

		{ 0x40, max96717_init_0x40_linkC_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkC_addr_setting), "max96717 linkC addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x43_linkC_sync_setting,
		  ARRAY_SIZE(max96717_init_0x43_linkC_sync_setting), "max96717 linkC sync" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkD_auto[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe3_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe3_step1), "max96712 pipe3 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkD_step2,
		  ARRAY_SIZE(max96712_init_setting_linkD_step2), "max96712 linkD step2" },

		{ 0x40, max96717_init_0x40_linkD_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkD_addr_setting), "max96717 linkD addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x44_linkD_auto_setting,
		  ARRAY_SIZE(max96717_init_0x44_linkD_auto_setting), "max96717 linkD auto" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	struct gmsl_config configs_linkD[] = {

		{ deserial_if->deserial_addr, max96712_init_setting_before_step1,
		  ARRAY_SIZE(max96712_init_setting_before_step1), "max96712 before step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_pipe3_step1,
		  ARRAY_SIZE(max96712_init_setting_pipe3_step1), "max96712 pipe3 step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_after_step1,
		  ARRAY_SIZE(max96712_init_setting_after_step1), "max96712 after step1" },

		{ deserial_if->deserial_addr, max96712_init_setting_linkD_step2,
		  ARRAY_SIZE(max96712_init_setting_linkD_step2), "max96712 linkD step2" },

		{ 0x40, max96717_init_0x40_linkD_addr_setting,
		  ARRAY_SIZE(max96717_init_0x40_linkD_addr_setting), "max96717 linkD addr" },

		{ deserial_if->deserial_addr, max96712_init_setting_step3,
		  ARRAY_SIZE(max96712_init_setting_step3), "max96712 step3" },

		{ sensor_info->serial_addr, max96717_init_0x44_linkD_sync_setting,
		  ARRAY_SIZE(max96717_init_0x44_linkD_sync_setting), "max96717 linkD sync" },

		{ deserial_if->deserial_addr, max96712_init_setting_step4,
		  ARRAY_SIZE(max96712_init_setting_step4), "max96712 step4" }
	};

	if (sensor_info->serial_addr == 0x41) {

		if (sensor_info->sensor_mode == 6) {
			vin_info("%s: enter fsync mode for link A \n", __func__);
			configs = configs_linkA;
		} else {
			configs = configs_linkA_auto;
		}
	} else if (sensor_info->serial_addr == 0x42) {
		vin_info("%s: enter fsync mode for link B \n", __func__);
		if (sensor_info->sensor_mode == 6) {
			configs = configs_linkB;
		} else {
			configs = configs_linkB_auto;
		}
	} else if (sensor_info->serial_addr == 0x43) {
		vin_info("%s: enter fsync mode for link C \n", __func__);
		if (sensor_info->sensor_mode == 6) {
			configs = configs_linkC;
		} else {
			configs = configs_linkC_auto;
		}
	} else if (sensor_info->serial_addr == 0x44) {
		vin_info("%s: enter fsync mode for link D \n", __func__);
		if (sensor_info->sensor_mode == 6) {
			configs = configs_linkD;
		} else {
			configs = configs_linkD_auto;
		}
	}

	for (int i = 0; i < ARRAY_SIZE(configs_linkA); i++) {

		ret = write_register(bus, configs[i].dev_addr, REG16_VAL8,
							 configs[i].size, configs[i].data);
		vin_info("Write %s register  at address 0x%02X\n",
					configs[i].err_msg, configs[i].dev_addr); 
		if (ret < 0) {
			vin_err("Write %s register failed at address 0x%02X\n",
					configs[i].err_msg, configs[i].dev_addr);
			return ret;
		}
	}

	return RET_OK;
}


int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	vin_info("linsh %s\n", __func__);

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
							 sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
							  1-sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);
			}
		}
	}

	return ret;
}

void isx031_commmon_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
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




static int isx031_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	isx031_commmon_data_init(sensor_info,&turning_data);

	turning_data.stream_ctrl.data_length = 2;

	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sensor_stream_on_setting)) {
		memcpy(stream_on, sensor_stream_on_setting, sizeof(sensor_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sensor_stream_off_setting)) {
		memcpy(stream_off, sensor_stream_off_setting, sizeof(sensor_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	return ret;
}


int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			isx031_linear_data_init(sensor_info);
			vin_info("linear master mode Reserved interface\n");
			break;
		case SLAVE_M:
			isx031_linear_data_init(sensor_info);
			vin_info("linear slave mode Reserved interface\n");
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	return ret;
}


int sensor_init(sensor_info_t *sensor_info)
{
	int ret = 0;

	vin_info("linsh %s\n", __func__);

	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}

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
	 // serial init setting, i2cmap
	// ret = max_serial_init(sensor_info);
	// if (ret < 0) {
	// 	vin_err("max serial init fail!\n");
	// 	return ret;
	// }
	// ret = sensor_mode_config_init(sensor_info);
	// if (ret < 0) {
	// 	vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	// 	return ret;
	// }

	/* serdes init */
	ret = serdes_init(sensor_info);

	if (ret < 0) {
		vin_err("serdes_init fail\n");
		return ret;
	}
	/* 3. linear mode config */
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}

	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
	vin_info("linsh %s\n", __func__);

  	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	setting_size = sizeof(max96712_enable_clock_setting) / sizeof(uint32_t)/2;
	vin_info("max96712_enable_clock_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max96712_enable_clock_setting);
	if(ret < 0) {
		vin_err("start %s max96712 fail\n", sensor_info->sensor_name);
		return -1;
	}

	setting_size = sizeof(max96717_stream_on_setting) / sizeof(uint32_t)/2;
	vin_info("max96717_stream_on_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
						setting_size, max96717_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s max96717 fail\n", sensor_info->sensor_name);
		return -1;
	}

	setting_size = sizeof(sensor_stream_on_setting) / sizeof(uint32_t)/2;
	vin_info("sensor_stream_on_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sensor_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s sensor fail\n", sensor_info->sensor_name);
		return -1;
	}

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;

	vin_info("linsh %s\n", __func__);

	setting_size = sizeof(sensor_stream_off_setting) / sizeof(uint32_t)/2;
	vin_info("sensor_stream_off_setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, sensor_stream_off_setting);
	if(ret < 0) {
		vin_err("stop %s sensor fail\n", sensor_info->sensor_name);
		return -1;
	}

	setting_size = sizeof(max96717_stream_off_setting) / sizeof(uint32_t)/2;
	vin_info("max96717_stream_off_setting_size = %d\n", setting_size);

	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
						setting_size, max96717_stream_off_setting);
	if(ret < 0) {
		vin_err("stop %s max96717 fail\n", sensor_info->sensor_name);
		return -1;
	}

	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	vin_info("linsh %s\n", __func__);

	/*
	if(sensor_info->power_mode) {
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
	*/

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return ret;
}




int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	vin_info("linsh %s\n", __func__);

	ret = sensor_deinit(sensor_info);
	return ret;
}


#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(isx031, CAM_MODULE_FLAG_A16D16);
sensor_module_t isx031 = {
	.module = SENSOR_MNAME(isx031),
#else
sensor_module_t isx031 = {
	.module = "isx031",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};



