/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0233]:" fmt

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
#include <sys/mman.h>

#include "inc/hb_vin.h"
#include "hb_cam_utility.h"
#include "hb_i2c.h"
#include "inc/ar0233_setting.h"
//#include "inc/sensor_effect_common.h"
#include "../../inc/hb_vin.h"

#include "inc/ds960_setting.h"
#include "inc/ds954_setting.h"
#include "inc/ds953_setting.h"
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
#define WAIT_LINK_LOOP			3000
#define SER_LINKED_STATUS 		(0xda)
#define DEFAULT_MAX96712_ADDR		(0x29)
#define DEFAULT_MAX9296_ADDR		(0x48)
#define INVALID_CAM_ADDR		(0xFF)

enum MODE_TYPE {
	DEFAULT_4LANE,
	DEFAULT_2LANE,
	DEFAULT_960_DUAL,
	SENSING_27M,
	SENSING_27M_DUAL,
	SENSING_27M_QUAD,
	WISSEN_25M,
	WISSEN_25M_QUAD,
	WISSEN_25M_DUAL,
	WISSEN_25M_TRIP,
	SENSING_27M_TRIP,
	GA0233,
	GA0233_DUAL,
	GA0233_TRIP,
	GA0233_QUAD,
	WISSEN_SY0820,
	WISSEN_WS0820,
	GA0233_WS0820,
	GA0233_WITH_WS0820,
	GA0233_SEPA_WS0820,
	GA0233_WITH_GA0820,
	GA0233_SEPA_GA0820,
};

int32_t poc_linked_first(uint32_t bus, uint8_t poc_addr)
{
	int32_t ret = 0, i, val;
	for (i = 0; i < 4; i++) {
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, (uint16_t)0x6 + (uint16_t)i);
		usleep(2000);
		/* read again to get current state */
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, (uint16_t)0x6 + (uint16_t)i);
		if (val < 0)
			break;
		/* linked on if > 150ma */
		if (val > 0x5) {
			ret = i + 1;
			break;
		}
	}
	return ret;
}

void setting_modify(uint8_t *pdata, int32_t size, uint8_t addr, uint16_t reg, uint16_t v)
{
	int32_t i, len;
	uint16_t r;

	for (i = 0; i < size;) {
		len = pdata[i];
		if (len == 0) {
			i += 2;
			continue;
		}
		if (pdata[i + 1] == (addr << 1)) {
			switch (len) {
				case 4:
				case 5:
					r = (uint16_t)(((uint16_t)pdata[i + 2] << 8) | (pdata[i + 3]));
					break;
				case 3:
					r = pdata[i + 2];
					break;
				case 0:
				default:
					r = 0xffff;
					break;
			}
			if (r == reg) {
				switch (len) {
					case 5:
						pdata[i + 4] = (uint8_t)(v >> 8);
						pdata[i + 5] = (uint8_t)(v & (uint8_t)0xff);
						break;
					case 4:
						pdata[i + 4] = (uint8_t)(v & (uint8_t)0xff);
						break;
					case 3:
						pdata[i + 3] = (uint8_t)(v & (uint8_t)0xff);
						break;
					case 0:
					default:
						break;
				}
				break;
			}
		}
		i += (len + 1);
	}
}

int32_t sensor_setting_array(int32_t bus, uint32_t i2c_addr, int32_t reg_width,
                         int32_t setting_size, uint16_t *cam_setting)
{
	x2_camera_i2c_t i2c_cfg;
	int32_t ret = RET_OK, i, k;

	i2c_cfg.i2c_addr = i2c_addr;
	i2c_cfg.reg_size = (uint32_t)reg_width;

	for(i = 0; i < setting_size; i++) {
		i2c_cfg.reg = cam_setting[2 * i];
		i2c_cfg.data = cam_setting[2 * i + 1];
		if (i2c_cfg.reg_size == 2)
			ret = hb_vin_i2c_write_reg16_data16((uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr,
				(uint16_t)i2c_cfg.reg, (uint16_t)i2c_cfg.data);
		else
			ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr,
				(uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
		k = 10;
		while (ret < 0 && k--) {
			if (k % 10 == 9)
				usleep(200 * 1000);
			if (i2c_cfg.reg_size == 2)
				ret = hb_vin_i2c_write_reg16_data16((uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr,
					(uint16_t)i2c_cfg.reg, (uint16_t)i2c_cfg.data);
			else
				ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, (uint8_t)i2c_cfg.i2c_addr,
					(uint16_t)i2c_cfg.reg, (uint8_t)i2c_cfg.data);
		}
		if (ret < 0) {
			vin_err("camera write 0x%2x fail \n", i2c_cfg.reg);
			break;
		}
	}
	return ret;
}

int32_t write_register(int32_t bus, int32_t deserial_addr, int32_t poc_addr, int32_t serial_addr,
			int32_t sensor_addr, uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = RET_OK;
	uint8_t i2c_slave, failed_dev = INVALID_CAM_ADDR;
	uint16_t reg_addr, value, delay;
	int32_t i, len, k;
	uint16_t link = 0, failed_link = 0;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 5) {
			i2c_slave = pdata[i + 1] >> 1;
			if (sensor_addr != 0 && i2c_slave == (uint8_t)DEFAULT_SENSOR_ADDR)
				i2c_slave = (uint8_t)sensor_addr;
			reg_addr = (uint16_t)(((uint16_t)pdata[i + 2] << 8) | pdata[i + 3]);
			value = (uint16_t)(((uint16_t)pdata[i + 4] << 8) | pdata[i + 5]);
			ret = hb_vin_i2c_write_reg16_data16((uint32_t)bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write ar0233 %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// 	usleep(5*1000);
			i = i + len + 1;
			vin_dbg("write ar0233 %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (uint16_t)(((uint16_t)pdata[i + 2] << 8) | pdata[i + 3]);
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == (int32_t)DEFAULT_DESERIAL_ADDR) {
				i2c_slave = (uint8_t)deserial_addr;
			// } else if (serial_addr != 0 && i2c_slave == DEFAULT_SERIAL_ADDR) {
				// i2c_slave = serial_addr;
			}

			// MAX96712
			if (i2c_slave == DEFAULT_MAX96712_ADDR && reg_addr == 0x6) {
				link = value & 0xF;
				value &= (~failed_link);
				vin_info("reg_addr 0x%x value 0x%x link 0x%x, failed_link %x\n",
								reg_addr, value, link, failed_link);
			}
			// MAX9296
			if (i2c_slave == DEFAULT_MAX9296_ADDR && reg_addr == 0x10) {
				link = value & 0x3;
				value &= (~failed_link);
				vin_info("reg_addr 0x%x value 0x%x link 0x%x, failed_link %x\n",
								reg_addr, value, link, failed_link);
			}
			if ((failed_dev != INVALID_CAM_ADDR && i2c_slave == failed_dev)) {
				vin_warn("skip write failed_dev 0x%x value 0x%x\n", i2c_slave, value);
				i = i + len + 1;
				continue;
			}

			k = 10;
			ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, reg_addr, (uint8_t)value);
			while (ret < 0 && k--) {
				vin_warn("write serdes %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, reg_addr, (uint8_t)value);
			}
			if (ret < 0) {
				vin_err("write serdes %d@0x%02x: 0x%04x=0x%02x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				if (i2c_slave == DEFAULT_SERIAL_ADDR && reg_addr == 0x00) {
					failed_dev = value >> 1;
				} else {
					failed_dev = i2c_slave;
				}
				failed_link |= link;
				if (failed_dev == deserial_addr) {
					return ret;
				}
				ret = 0;
			}
			// usleep(100*1000);
			i = i + len + 1;
			vin_dbg("write serdes %d@0x%02x: 0x%04x=0x%02x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 3) {
			if (poc_addr != INVALID_POC_ADDR) {
				i2c_slave = pdata[i + 1] >> 1;
				reg_addr = pdata[i + 2];
				value = pdata[i + 3];
				if (poc_addr != 0 && i2c_slave == (uint16_t)DEFAULT_POC_ADDR)
					i2c_slave = (uint8_t)poc_addr;
				ret = hb_vin_i2c_write_reg8_data8((uint32_t)bus, i2c_slave, reg_addr, (uint8_t)value);
				if (ret < 0) {
					vin_err("write poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				// usleep(100*1000);
				vin_dbg("write poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
			}
			i = i + len + 1;
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep((uint32_t)delay * 1000u);
			i = i + 2;
		}
	}
	return ret;
}

#if 0
/**
 * @brief write_register : write sensor and serdes reg
 *
 * @param [in] bus : i2c num
 * @param [in] pdata : setting need to write
 * @param [in] setting_size : setting num
 *
 * @return ret
 */
int32_t write_register(int32_t bus, uint8_t *pdata, int32_t setting_size)
{
	int32_t ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int32_t i, len, k = 10;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			while (ret < 0 && k--) {
				vin_err("write serdes %d@0x%02x: 0x%04x=0x%02x err %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				if (k % 10 == 9) {
					usleep(20 * 1000);
					ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
				}
			}
			if (ret < 0) {
				vin_err("write serdes %d@0x%02x: 0x%04x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
				return ret;
			}
			i = i + len + 1;
			vin_dbg("write serdes %d@0x%02x: 0x%04x=0x%02x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
			i = i + 2;
		}
	}
	return ret;
}
#endif

int32_t link_switch(sensor_info_t *sensor_info, int32_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint8_t  val = 0;
	uint8_t  val_read = 0;

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		reg = REG_LINK_SET_9296;
		if (link_port < DES_PORT_NUM_MAX - 2) {
			val = (uint8_t)((uint8_t)LINK_NONE_9296 | ((uint8_t)1 << (uint32_t)link_port));
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_9296;
		} else {
			vin_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
				deserial_if->deserial_name);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		reg = REG_LINK_SET_96712;
		if (link_port < DES_PORT_NUM_MAX) {
			val = (uint8_t)((uint8_t)LINK_NONE_96712 & (~((uint8_t)1 << (2 * (uint32_t)link_port))));
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_96712;
		} else {
			vin_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
				deserial_if->deserial_name);
			return -1;
		}
	} else {
		vin_info("%s not supported des-%s, drop\n", __func__, deserial_if->deserial_name);
		return 0;
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
		reg, val);
	if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
			deserial_if->deserial_name);
		return -1;
	}
	vin_dbg("%s switch to port 0x%x successfully for des-%s!\n",
		__func__, link_port, deserial_if->deserial_name);
	usleep(20 * 1000);
	return 0;
}

static int32_t des_low_speed_fix(sensor_info_t *sensor_info, uint8_t *pdata, int32_t setting_size)
{
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	if ((sensor_info->config_index & DES_LOW_SPEED) &&
		(!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722"))) {
		setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0x415, 0x2c);
		setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0x418, 0x2c);
		setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0x41b, 0x2c);
		setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0x41e, 0x2c);
	}

	return 0;
}

#ifdef POC_RETRY_POLICY
static int32_t set_gpio(deserial_info_t *deserial_info)
{
	uint32_t gpio;
	int32_t ret = -HB_CAM_SENSOR_POWERON_FAIL;

	for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
		vin_dbg("Set gpio level is %d for gpio%d\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
		if(deserial_info->gpio_pin[gpio] != -1) {
			ret = vin_power_ctrl((uint32_t)deserial_info->gpio_pin[gpio],
					deserial_info->gpio_level[gpio]);
			usleep(100 *1000);
			ret = (int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)deserial_info->gpio_pin[gpio],
					1-deserial_info->gpio_level[gpio]));
			if(ret < 0) {
				vin_err("Set gpio level is %d for gpio%d failed\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
				return -HB_CAM_SENSOR_POWERON_FAIL;
			}
			usleep(100*1000);
		}
	}
	return ret;
}

static int32_t poc_power_reset(sensor_info_t *sensor_info)
{
	int32_t retry_poc_times = 0;
	int32_t setting_size = 0;
	int32_t ret = RET_OK;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	setting_size = 1;
	poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
	for (retry_poc_times = 0; retry_poc_times < RETRY_POC_TIMES; retry_poc_times++) {
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, (uint8_t)poc_addr, 0x01, 0x00);
		if (ret < 0) {
			vin_warn("write %d@0x%02x=0x00 failed, try to reset the poc, %d times\n", \
				deserial_if->bus_num, poc_addr, retry_poc_times);
			ret = set_gpio(deserial_if);
			if (ret < 0) {
				vin_err("set_gpio fail\n");
				return ret;
			}
			continue;
		}
		usleep(10*1000);
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, (uint8_t)poc_addr, 0x01, 0x1f);
		if (ret < 0) {
			vin_warn("write %d@0x%02x=0x1f failed, try to reset the poc, %d times\n", \
				deserial_if->bus_num, poc_addr, retry_poc_times);
			ret = set_gpio(deserial_if);
			if (ret < 0) {
				vin_err("set_gpio fail\n");
				return ret;
			}
			continue;
		}
		usleep(50*1000);
		return ret;
	}
	return -RET_ERROR;
}

#endif
/**
 * @brief sensor_ar0233_des_init : write deserial init setting,
 *                                 including 965, 9296, 96712, etc.
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_ar0233_des_init(sensor_info_t *sensor_info)	 /*PRQA S 2775*/
{
	int32_t ret = RET_OK;
	uint32_t setting_size = 0;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;

	if (!strcmp(deserial_if->deserial_name, "s954")) {
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_4LANE) {
			setting_size = sizeof(ds954_ar0233_x3_4lane_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds954_ar0233_x3_4lane_init_setting);
			if (ret < 0) {
				vin_err("write ds954_ar0233_init_4lane_setting error\n");
				return ret;
			}
	    } else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_2LANE) {
			setting_size = sizeof(ds954_ar0233_x3_2lane_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds954_ar0233_x3_2lane_init_setting);
			if (ret < 0) {
				vin_err("write ds954_ar0233_init_2lane_setting error\n");
				return ret;
			}
		} else if (sensor_info->extra_mode & EXT_MASK) { /* zu3 */
			setting_size = sizeof(ds954_ar0233_zu3_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds954_ar0233_zu3_init_setting);
			if (ret < 0) {
				vin_err("write ds954_ar0233_zu3_init_setting error\n");
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "s960")) {
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_4LANE) {
			setting_size = sizeof(ds960_ar0233_x3_4lane_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds960_ar0233_x3_4lane_init_setting);
			if (ret < 0) {
				vin_err("write ds960_ar0233_x3_4lane_init_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_2LANE) {
			setting_size = sizeof(ds960_ar0233_x3_2lane_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds960_ar0233_x3_2lane_init_setting);
			if (ret < 0) {
				vin_err("write ds960_ar0233_x3_2lane_init_setting error\n");
				return ret;
			}
		} else if (sensor_info->extra_mode & EXT_MASK) { /* zu3 */
			setting_size = sizeof(ds960_ar0233_zu3_init_setting)/sizeof(uint32_t)/2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 1,
					setting_size, ds960_ar0233_zu3_init_setting);
			if (ret < 0) {
				vin_err("write ds960_ar0233_zu3_init_setting error\n");
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296") ||
			!strcmp(deserial_if->deserial_name, "max96718")) {
		if (poc_addr != INVALID_POC_ADDR) {
#ifdef POC_RETRY_POLICY
			ret = poc_power_reset(sensor_info);
			if (ret < 0) {
				vin_err("poc_power_reset fail\n");
				return ret;
			}
#else
			setting_size = 1;
			poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
			ret = vin_write_array(deserial_if->bus_num, poc_addr, 1,
									 setting_size, poc_init_setting);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
			usleep(10 * 1000);
			ret = vin_write_array(deserial_if->bus_num, poc_addr, 1,
									 setting_size, poc_init_setting + 2);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
#endif
		} else if (!sensor_info->power_mode) {
			/* reset all serials replace to poc off */
			int32_t i2c_slave;
			for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
				vin_dbg("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
				hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)i2c_slave, 0x0010, 0xf1);
			}
		}

		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) {
			setting_size = 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_init_setting);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			usleep(10 * 1000);
			setting_size = sizeof(max9296_init_setting) / sizeof(uint32_t) / 2 - 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_init_setting + 2);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			pipe_96718 = 0x09;
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233) {
			setting_size = 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_init_setting_ws);
			if (ret < 0) {
				vin_err("write max9296_init_setting_ws error\n");
				return ret;
			}
			usleep(10 * 1000);
			setting_size = sizeof(max9296_init_setting_ws) / sizeof(uint32_t) / 2 - 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_init_setting_ws + 2);
			if (ret < 0) {
				vin_err("write max9296_init_setting_ws error\n");
				return ret;
			}
			// only effect 9296
                        ret = link_switch(sensor_info, sensor_info->deserial_port);
                        if (ret < 0) {
                                vin_err("switch to link all failed for port%d\n",
                                        sensor_info->port);
                        }
			usleep(100 * 1000);
			if (sensor_info->deserial_port)
				pipe_96718 = 0x36;
			else
				pipe_96718 = 0x12;
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL) {
			pdata = max9296_max9295_dual_init_setting;
			setting_size = sizeof(max9296_max9295_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max9295 dual error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL) {
			pdata = max9296_max96717_dual_init_setting;
			setting_size = sizeof(max9296_max96717_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max96717 dual error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else {
			vin_err("%s not support extra_mode %d\n", deserial_if->deserial_name,
					sensor_info->extra_mode);
			return -1;
		}
		if (!strcmp(deserial_if->deserial_name, "max96718")) {
			max9296_add_max96718_init_setting[1] = pipe_96718;
			setting_size = sizeof(max9296_add_max96718_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
					setting_size, max9296_add_max96718_init_setting);
			if (ret < 0) {
				vin_err("write max9296_add_max96718_init_setting error\n");
				return ret;
			}
		}
		if (((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) ||
			((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M) ||
			((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233)) {
			if (sensor_info->config_index & DPHY_COPY) {
				setting_size = sizeof(max9296_phy_portall_init_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max9296_phy_portall_init_setting);
				if (ret < 0) {
					vin_err("write max9296_phy_portall_init_setting error\n");
					return ret;
				}
			} else if (sensor_info->config_index & DPHY_PORTB) {
				setting_size = sizeof(max9296_phy_portb_init_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max9296_phy_portb_init_setting);
				if (ret < 0) {
					vin_err("write max9296_phy_portall_init_setting error\n");
					return ret;
				}
			}
		}
		if ((sensor_info->config_index & TRIG_STANDARD) ||
		    (sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		    if (deserial_if->mfp_index > MAX9296_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max9296_trig_setting MFP index error\n");
				return ret;
			}
			setting_size = 0;
			uint16_t regaddr = 0, offset = 0;
			uint16_t *trigger_reg;

			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max9296_trigger_mfp5;
				setting_size = sizeof(max9296_trigger_mfp5) / sizeof(uint16_t) / 2;
				offset = 0;
			} else {
				trigger_reg = max9296_trigger_mfp;
				setting_size = sizeof(max9296_trigger_mfp) / sizeof(uint16_t) / 2;
				offset = (uint16_t)(deserial_if->mfp_index * MAX9296_MFP_OFFSET);
			}

			for (uint32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i] + offset;
				vin_dbg("write mfp: w%d@0x%02x 0x%04x=0x%02x\n", deserial_if->bus_num,	/*PRQA S 1861*/
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i+1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      (uint8_t)deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i+1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write max9296_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			vin_dbg("%s testpattern extra_mode=%d %dx%d init\n",
					deserial_if->deserial_name, sensor_info->extra_mode,
					(sensor_info->width) ? sensor_info->width : 2048,
					(sensor_info->height) ? sensor_info->height : 1280);
			pdata = max96712_testpattern_quad_init_setting;
			setting_size = sizeof(max96712_testpattern_quad_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_testpattern register error\n");
				return ret;
			}
			switch (sensor_info->extra_mode & EXT_MODE) {
			case (uint32_t)WISSEN_25M:
			case (uint32_t)SENSING_27M:
			case (uint32_t)GA0233:
				ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						(uint8_t)deserial_if->deserial_addr, 0x096D, 0x00));
				/* no break */
			case (uint32_t)WISSEN_25M_DUAL:
			case (uint32_t)SENSING_27M_DUAL:
			case (uint32_t)GA0233_DUAL:
				ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						(uint8_t)deserial_if->deserial_addr, 0x09AD, 0x00));
				/* no break */
			case (uint32_t)WISSEN_25M_TRIP:
			case (uint32_t)SENSING_27M_TRIP:
			case (uint32_t)GA0233_TRIP:
				ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						(uint8_t)deserial_if->deserial_addr, 0x09ED, 0x00));
				/* no break */
				break;
			default:
				break;
			}
			if (sensor_info->width) {
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x0167, (uint8_t)(sensor_info->width >> 8)));
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x0168, (uint8_t)(sensor_info->width & 0xFF)));
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x0169, (uint8_t)((4200 - sensor_info->width) >> 8)));
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x016A, (uint8_t)((4200 - sensor_info->width) & 0xfF)));
			}
			if (sensor_info->height) {
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x016B, (uint8_t)(sensor_info->height >> 8)));
					ret = (int32_t)((uint32_t)ret | (uint32_t)hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							(uint8_t)deserial_if->deserial_addr, 0x016C, (uint8_t)(sensor_info->height & 0xFF)));
			}
			if (ret < 0) {
				vin_err("write max96712_testpattern res register error\n");
				return ret;
			}
		} else {
			if (poc_addr != INVALID_POC_ADDR) {
#ifdef POC_RETRY_POLICY
				ret = poc_power_reset(sensor_info);
				if (ret < 0) {
					vin_err("poc_power_reset fail\n");
					return ret;
				}
#else
				setting_size = 1;
				poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
				ret = vin_write_array(deserial_if->bus_num, poc_addr, 1,
										 setting_size, poc_init_setting);
				if (ret < 0) {
					vin_err("write poc 0x%02x init setting error\n", poc_addr);
					return ret;
				}
				usleep(10 * 1000);
				ret = vin_write_array(deserial_if->bus_num, poc_addr, 1,
									 setting_size, poc_init_setting + 2);
				if (ret < 0) {
					vin_err("write poc 0x%02x init setting error\n", poc_addr);
					return ret;
				}
#endif

			} else if (!sensor_info->power_mode) {
				/* reset all serials replace to poc off */
				int32_t i2c_slave;
				for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
					vin_dbg("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
					hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)i2c_slave, 0x0010, 0xf1);
				}
			}

			if (((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) ||
				((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M) ||
				((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233)) {
				uint8_t poc_first = 0, des_port = 0, des_link_en = 0, des_pipe0_sel = 0;
				if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) {
					pdata = max96712_max9295_init_setting;
					setting_size = sizeof(max96712_max9295_init_setting)/sizeof(uint8_t);
				} else {
					pdata = max96712_max96717_init_setting;
					setting_size = sizeof(max96712_max96717_init_setting)/sizeof(uint8_t);
				}
				/* auto detect for one link on 96712 */
				if ((poc_addr != INVALID_POC_ADDR) && (sensor_info->deserial_port == 0)) {
					poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
					poc_first = (uint8_t)(poc_linked_first(sensor_info->bus_num, (uint8_t)poc_addr));
					if (poc_first >= (uint8_t)3)
						poc_first = ((uint8_t)7 - poc_first);
				} else {
					poc_first = (uint8_t)(sensor_info->deserial_port + 1);
				}
				switch (poc_first) {
					case 4:
						des_port = 'D';
						des_link_en = 0xF8;
						des_pipe0_sel = 0x0e;
						break;
					case 3:
						des_port = 'C';
						des_link_en = 0xF4;
						des_pipe0_sel = 0x0a;
						break;
					case 2:
						des_port = 'B';
						des_link_en = 0xF2;
						des_pipe0_sel = 0x06;
						break;
					case 1:
						des_port = 'A';
						des_link_en = 0xF1;
						des_pipe0_sel = 0x02;
						break;
					case 0:
					default:
						break;
				}
				if (poc_first) {
					vin_dbg("%s on port-%c config mode!\n", sensor_info->sensor_name, des_port);
					setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0x6, des_link_en);
					setting_modify(pdata, setting_size, (uint8_t)deserial_if->deserial_addr, 0xf0, des_pipe0_sel);
				} else {
					vin_dbg("%s config mode!\n", sensor_info->sensor_name);
				}
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write %s 1 port register error\n", sensor_info->sensor_name);
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD) {
				pdata = max96712_max9295_quad_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_quad_init_setting_4lane)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 quad register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL) {
				pdata = max96712_max9295_dual_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_dual_init_setting_4lane)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 dual register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP) {
				pdata = max96712_max9295_trip_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_trip_init_setting_4lane)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 trip register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD) {
				pdata = max96712_max96717_quad_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_quad_init_setting_4lane)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 quad register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL) {
				pdata = max96712_max96717_dual_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_dual_init_setting_4lane)/
				               sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 dual register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP) {
				pdata = max96712_max96717_trip_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_trip_init_setting_4lane)/
				               sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 trip register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820) {
				pdata = sensing_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(sensing_max96712_max9295_max96717_init_setting)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 sy0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820) {
				pdata = weisen_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(weisen_max96712_max9295_max96717_init_setting)/
					sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 ws0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820) {
				pdata = galaxy_with_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_with_max96712_max9295_max96717_init_setting)/
				               sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 with ga0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
					(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
				pdata = galaxy_sepa_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_sepa_max96712_max9295_max96717_init_setting)/
				               sizeof(uint8_t);
				des_low_speed_fix(sensor_info, pdata, setting_size);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 sepa ga0820 register error\n");
					return ret;
				}
			} else {
				vin_err("extra_mode %d not supported\n", sensor_info->extra_mode);
				return -1;
			}
			/* i2c addr remap for galaxy */
			if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820) {
				pdata = galaxy_maxser_sensor_i2cmap_setting;
				setting_size = sizeof(galaxy_maxser_sensor_i2cmap_setting)/
					sizeof(uint8_t);
				pdata[1] = (uint8_t)((sensor_info->serial_addr - 1) << 1);
				pdata[6] = (uint8_t)((sensor_info->serial_addr - 1) << 1);
				vin_dbg("map ar0820 0x%02x as 0x%02x for galaxy\n", pdata[9] >> 1, pdata[4] >> 1);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
									 sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write galaxy_maxser_sensor_i2cmap_setting register error\n");
					return ret;
				}
			}
		}
		if (sensor_info->config_index & DPHY_PORTB) {
			pdata = max96712_phy_portb_init_setting;
			setting_size = sizeof(max96712_phy_portb_init_setting)/
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_phy_portb register error\n");
				return ret;
			}
			if (sensor_info->config_index & DPHY_COPY) {
				pdata = max96712_phy_cpB2A_init_setting;
				setting_size = sizeof(max96712_phy_cpB2A_init_setting)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_phy_cpB2A register error\n");
					return ret;
				}
			}
		} else {
			if (sensor_info->config_index & DPHY_COPY) {
				pdata = max96712_phy_cpA2B_init_setting;
				setting_size = sizeof(max96712_phy_cpA2B_init_setting)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_phy_cpA2B register error\n");
					return ret;
				}
			}
		}
		if ((sensor_info->config_index & TRIG_STANDARD) ||
		    (sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		    if (deserial_if->mfp_index > MAX96712_MFP_NUM && deserial_if->mfp_index != 0xffff) {
				vin_err("max96712_trig_setting MFP index error\n");
				return ret;
			}
			setting_size = sizeof(max96712_trigger_setting_mfp) /
			                        sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
			uint16_t offset = 0, size = 0, regaddr = 0;
			uint16_t *trigger_reg;

			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max96712_trigger_setting_mfp14;
				setting_size = sizeof(max96712_trigger_setting_mfp14) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
			} else {
				trigger_reg = max96712_trigger_setting_mfp;
				setting_size = sizeof(max96712_trigger_setting_mfp) / sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
				offset = (uint16_t)(deserial_if->mfp_index % MAX96712_MFP_LOOP * setting_size);
				size = (uint16_t)(deserial_if->mfp_index / MAX96712_MFP_LOOP * MAX96712_MFP_OFFSET);
			}

			for (uint32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i + 2*(uint32_t)offset] + size;
				vin_dbg("write mfp: w%d@0x%02x 0x%04x=0x%02x\n", deserial_if->bus_num,	/*PRQA S 1860,1861,1891*/
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      (uint8_t)deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i + 2*(uint32_t)offset + 1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write max96712_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	deserial_if->init_state = 1;
	vin_dbg("deserial %s init done\n", deserial_if->deserial_name);
	return ret;
}

#ifdef FPGA_AR0233

#define GPIO_BASE     0x34120000
int32_t sensor_poweron(sensor_info_t *sensor_info)
{
        uint32_t gpio_base, gpio_val, gpio_curr_val;
        int fd;
        unsigned char *gpio_addr;
        int32_t sensor_index = 0;

        printf("FPGA ar0233 sensor poweron \n");

        /* we enable csi0-csi4 all gpio pin in x5 fpga */
        for (int i = 0; i < 5; i ++) {

                sensor_index = i;

                gpio_base = 1 << (11 + sensor_index) | (1 << (sensor_index * 2));
                gpio_val  = 1 << (11 + sensor_index);

                fd = open("/dev/mem", O_RDWR);
                if (fd < 0)
                printf("open /dev/mem failed\n");

                gpio_addr = (unsigned char *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);

                gpio_curr_val = *(unsigned int *)(gpio_addr + 0x04);
                *(unsigned int *)(gpio_addr + 0x04) = gpio_base | gpio_curr_val;

                usleep(200 * 1000);

                gpio_curr_val = *(unsigned int *)gpio_addr;
                *(unsigned int *)gpio_addr = (gpio_val | gpio_curr_val) | (1 << (sensor_index * 2));

                usleep(200 * 1000);

                if (fd)
                    close(fd);

                munmap(gpio_addr, 0x1000);

                usleep(200 * 1000);
        }

        return 0;
}
#else
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
#endif


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
		sizeof(turning_data->sensor_name)-1);
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

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_VTS, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_HTS, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | (uint32_t)init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_X_START, init_d, 2);
	x0 = init_d[0];
	x0 = (x0 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_Y_START, init_d, 2);
	m_y0 = init_d[0];
	m_y0 = (m_y0 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0233_X_END, init_d, 2);
	x1 = init_d[0];
	x1 = (x1 << 8) | (uint32_t)init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
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
	if ((sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS == 1) /* zu3 2fps */
		turning_data->sensor_data.lines_per_second = 12752;
	else
		turning_data->sensor_data.lines_per_second = sensor_info->fps * turning_data->sensor_data.VMAX;
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;

	sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
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

#ifdef COMP_XJ3_CAM
	if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_25M ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_25M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_25M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_25M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_SY0820 ||
		(sensor_info->extra_mode & EXT_MODE) == WISSEN_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_WITH_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_SEPA_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_WITH_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GA0233_SEPA_GA0820) {
		turning_data_ex.l_line = AR0233_LINE2;
		turning_data_ex.l_line_length = 2;
		turning_data_ex.ratio_value = AR0233_RATIO_FATOR;   //  T2 = T1/100
		turning_data_ex.ratio_en = 1;   //  T2 = T1/100
		turning_data_ex.lexposure_time_min = 3;
	}
#endif
	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	if ((sensor_info->extra_mode & EXT_MODE) < (uint32_t)SENSING_27M) {
		turning_data.pwl.line_p.ratio = 1 << 8;
		turning_data.pwl.line_p.offset = 0;
		turning_data.pwl.line_p.max = 4000;

		turning_data.pwl.again_control_num = 1;
		turning_data.pwl.again_control[0] = AR0233_GAIN;
		turning_data.pwl.again_control_length[0] = 2;
		turning_data.pwl.dgain_control_num = 1;
		turning_data.pwl.dgain_control[0] = AR0233_DGAIN;
		turning_data.pwl.dgain_control_length[0] = 2;
		turning_data.pwl.again_lut = malloc(256*1*sizeof(uint32_t));	/* PRQA S 5118 */
		if (turning_data.pwl.again_lut != NULL) {
			memset(turning_data.pwl.again_lut, 0xff, 256*1*sizeof(uint32_t));

			memcpy(turning_data.pwl.again_lut, ar0233_again_lut,
				sizeof(ar0233_again_lut));
			for (open_cnt =0; open_cnt <
				sizeof(ar0233_again_lut)/sizeof(uint32_t); open_cnt++) {
				VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
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
	} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
		turning_data.pwl.line_p.ratio = 1 << 8;
		turning_data.pwl.line_p.offset = 0;
		turning_data.pwl.line_p.max = 4000;

		turning_data.pwl.again_control_num = 1;
		turning_data.pwl.again_control[0] = AR0233_GAIN;
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
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
			if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
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
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
			turning_data.sensor_awb.apply_lut_gain = 1;
		} else {
			turning_data.sensor_awb.apply_lut_gain = 0;
		}
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

	if (sensor_info->config_index & RES_WIDTH_1920 || sensor_info->width != 0) {
		if (sensor_info->width !=0) {
			ar0233_width_1920_init_setting[1] = (uint16_t)(1032 - (sensor_info->width / 2));
			ar0233_width_1920_init_setting[3] = (uint16_t)(1032 + (sensor_info->width / 2) - 1);
		}
		vin_dbg("%s width %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,		/*PRQA S 1860*/
				ar0233_width_1920_init_setting[3] - ar0233_width_1920_init_setting[1] + 1,
				ar0233_width_1920_init_setting[1], ar0233_width_1920_init_setting[3]);
		setting_size = sizeof(ar0233_width_1920_init_setting)/sizeof(uint16_t)/2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, ar0233_width_1920_init_setting[i*2],
						ar0233_width_1920_init_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("write ar0233_width_1920_init_setting error\n");
			}
		}
	}

	if (sensor_info->config_index & RES_HEIGHT_1080 || sensor_info->height != 0) {
		if (sensor_info->height !=0) {
			ar0233_height_1080_init_setting[1] = (uint16_t)(644 - (sensor_info->height / 2));
			ar0233_height_1080_init_setting[3] = (uint16_t)(644 + (sensor_info->height / 2) - 1);
		}
		vin_dbg("%s height %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,		/*PRQA S 1860*/
				ar0233_height_1080_init_setting[3] - ar0233_height_1080_init_setting[1] + 1,
				ar0233_height_1080_init_setting[1], ar0233_height_1080_init_setting[3]);
		setting_size = sizeof(ar0233_height_1080_init_setting)/sizeof(uint16_t)/2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
						(uint8_t)sensor_info->sensor_addr, ar0233_height_1080_init_setting[i*2],
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

	setting_size = sizeof(ar0233_x3_30fps_linear_init_setting)/sizeof(uint16_t)/2;
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
	int32_t setting_size, tmp = 0;

	if ((sensor_info->extra_mode & EXT_MODE) < (uint32_t)SENSING_27M) {
		setting_size = sizeof(ar0233_x3_30fps_pwl_init_setting)/sizeof(uint16_t)/2;
		vin_dbg("x3 setting_size %d\n", setting_size);
		for (i = 0; i < setting_size; i++) {
			if (((sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS) <= 1) {
#ifdef FPS_HTS
				if (ar0233_x3_30fps_pwl_init_setting[i*2] == (uint16_t)AR0233_HTS
					&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
					ar0233_x3_30fps_pwl_init_setting[i*2 + 1] = (uint16_t)(60000 / sensor_info->fps);
#else
				if (ar0233_x3_30fps_pwl_init_setting[i*2] == (uint16_t)AR0233_VTS
					&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
					ar0233_x3_30fps_pwl_init_setting[i*2 + 1] = 51000 / sensor_info->fps;
#endif
			} else if (ar0233_x3_30fps_pwl_init_setting[i*2] == (uint16_t)AR0233_VTS) {
				tmp = ((uint32_t)sensor_info->extra_mode) >> 16;
				vin_dbg("%s vts=%d(0x%04x)\n", sensor_info->sensor_name, tmp);
				ar0233_x3_30fps_pwl_init_setting[i*2 + 1] = (uint16_t)tmp;
			} else if (ar0233_x3_30fps_pwl_init_setting[i*2] == (uint16_t)AR0233_HTS) {
				tmp = ((uint32_t)sensor_info->extra_mode) & 0xffff;
				vin_dbg("%s hts=%d(0x%04x)\n", sensor_info->sensor_name, tmp);
				ar0233_x3_30fps_pwl_init_setting[i*2 + 1] = (uint16_t)tmp;
			}
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
											ar0233_x3_30fps_pwl_init_setting[i*2], ar0233_x3_30fps_pwl_init_setting[i*2 + 1]);
			if (ret < 0) {
				tmp++;
				if (tmp < 10) {
					i--;
					usleep(10*1000);
					continue;
				}
				vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__, sensor_info->sensor_name,
					   sensor_info->bus_num, sensor_info->sensor_addr,
					   i, ar0233_x3_30fps_pwl_init_setting[i*2], ar0233_x3_30fps_pwl_init_setting[i*2 + 1]);
				return ret;
			}
			if((i == 0) || (i == 1) || (i == 1151))
				usleep(200*1000);
			tmp = 0;
		}
	} else {
		usleep(100*1000);
		setting_size = sizeof(ar0233_init_setting_soft_reset_0)/sizeof(uint16_t)/2;
		ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, ar0233_init_setting_soft_reset_0);
		if (ret < 0) {
			vin_err("senor %s soft reset_0 setting error\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(100*1000);
		setting_size = sizeof(ar0233_init_setting_soft_reset_1)/sizeof(uint16_t)/2;
		ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, ar0233_init_setting_soft_reset_1);
		if (ret < 0) {
			vin_err("senor %s soft reset_1 setting error\n", sensor_info->sensor_name);
			return ret;
		}
		if (sensor_info->resolution == 1080 && sensor_info->fps == 60) {
			setting_size = sizeof(ar0233_init_setting_1080p_60fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1080p_60fps);
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1280 && sensor_info->fps == 45) {
			setting_size = sizeof(ar0233_init_setting_1280p_45fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1280p_45fps);
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1280 && sensor_info->fps == 30) {
			if (sensor_info->config_index & DES_LOW_SPEED) {
				vin_dbg("setting for des low speed\n");
				setting_size = sizeof(ar0233_init_setting_1280p_30fps_low)/sizeof(uint16_t)/2;
				ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, ar0233_init_setting_1280p_30fps_low);
			} else {
				setting_size = sizeof(ar0233_init_setting_1280p_30fps)/sizeof(uint16_t)/2;
				ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, ar0233_init_setting_1280p_30fps);
			}
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1280 && sensor_info->fps == 25) {
			setting_size = sizeof(ar0233_init_setting_1280p_25fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1280p_25fps);
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1280 && sensor_info->fps == 20) {
			setting_size = sizeof(ar0233_init_setting_1280p_20fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1280p_20fps);
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1280 && sensor_info->fps == 15) {
			setting_size = sizeof(ar0233_init_setting_1280p_15fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1280p_15fps);
			if (ret < 0) {
				vin_err("senor %s write resolution=%d--fps=%d setting error\n",
					sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
				return ret;
			}
		} else if (sensor_info->resolution == 1080 && sensor_info->fps == 30) {
			setting_size = sizeof(ar0233_init_setting_1080p_30fps)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_1080p_30fps);
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
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP) {
			setting_size = sizeof(ar0233_init_setting_27M)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_27M);
			if (ret < 0) {
				vin_err("senor %s write 27M pll setting error\n",
					sensor_info->sensor_name);
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
			setting_size = sizeof(ar0233_init_setting_25M)/sizeof(uint16_t)/2;
			ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
					setting_size, ar0233_init_setting_25M);
			if (ret < 0) {
				vin_err("senor %s write 25M pll setting error\n",
					sensor_info->sensor_name);
				return ret;
			}
		}
		setting_size = sizeof(ar0233_base_init_setting)/sizeof(uint16_t)/2;
		ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, ar0233_base_init_setting);
		if (ret < 0) {
			vin_err("senor %s write base setting error\n", sensor_info->sensor_name);
			return ret;
		}
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
	if (sensor_info->config_index & TEST_PATTERN) {
		vin_dbg("ar0233_test_pattern 0x%04x\n", ar0233_test_pattern[1]);
		setting_size = sizeof(ar0233_test_pattern)/sizeof(uint16_t)/2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
					(uint8_t)sensor_info->sensor_addr, ar0233_test_pattern[i*2],
					ar0233_test_pattern[i*2 + 1]);
			if (ret < 0) {
				vin_err("write ar0233_test_pattern error\n");
			}
		}
	}

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

int32_t wait_serializer_9295A_96717_link_locked(sensor_info_t *sensor_info)
{
	int32_t i = 0, val = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	while ((val != SER_LINKED_STATUS) && (i < WAIT_LINK_LOOP)) {
		/* Get link locked register status */
		val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr, 0x13);
		vin_dbg("Check %s link status: i2c%d-0x%x 0x13=0x%x, Currently waiting for %dms\n", \
			deserial_if->deserial_name, deserial_if->bus_num, deserial_if->deserial_addr, val, i);
		i++;
		usleep(1*1000);
		if (100 == i) {
			vin_warn("Wait for the link to exceed 100ms\n");
		}
	}
	if (i < WAIT_LINK_LOOP) {
		return RET_OK;
	} else {
		return -RET_ERROR;
	}
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

	if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_4LANE ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)DEFAULT_2LANE) {  /*ar0233 4lane and 2lane config*/
                req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		if (req == 0) {
			vin_dbg("0233 serdes start init \n");
			ret = sensor_ar0233_des_init(sensor_info);
			hb_vin_mipi_pre_result(sensor_info->entry_num, 0, (uint32_t)ret);	/* PRQA S 2897 */
			if (ret < 0) {
				vin_err("sensor_ar0233_des_init fail\n");
				return ret;
			}
		}
	} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820 ||
		(sensor_info->config_index & TEST_PATTERN_SERDES)) {
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
			entry_num = deserial_if->physical_entry;
			vin_dbg("sepa config use physical_entry %d\n", entry_num);
		}
		req = hb_vin_mipi_pre_request((uint32_t)entry_num, 0, 0);
		if (req == 0) {
			vin_dbg("0233 serdes start init \n");
			ret = sensor_ar0233_des_init(sensor_info);
			hb_vin_mipi_pre_result((uint32_t)entry_num, 0, (uint32_t)ret);	/* PRQA S 2897 */
			if (ret < 0) {
				vin_err("sensor_ar0233_des_init fail\n");
				return ret;
			}
		}
	}

	if (sensor_info->config_index & TEST_PATTERN_SERDES)
		return ret;

	if ((sensor_info->extra_mode & EXT_MODE) < (uint32_t)SENSING_27M) {
		usleep(5000);
		vin_dbg("0233 953 start init \n");
		setting_size = sizeof(ds953_ar0233_x3_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1, 1, ds953_ar0233_x3_init_setting);
		usleep(5000);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 1,
				setting_size - 1, &(ds953_ar0233_x3_init_setting[2]));
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296") ||
			   !strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233) {
			/* support i2c address remap for single config */
			if (sensor_info->serial_addr != DEFAULT_SERIAL_ADDR) {
				vin_dbg("i2c remap serial %s as 0x%02x\n",
						deserial_if->deserial_name, sensor_info->serial_addr);
				single_maxser_i2cmap_setting[1] = (sensor_info->serial_addr << 1);
				setting_size = sizeof(single_maxser_i2cmap_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(sensor_info->bus_num, DEFAULT_SERIAL_ADDR, 2,
						setting_size, single_maxser_i2cmap_setting);
				if (ret < 0) {
					/* retry with mapped addr */
					ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
							setting_size, single_maxser_i2cmap_setting);
					if (ret < 0) {
						vin_err("write single_maxser_i2cmap_setting error\n");
						return ret;
					}
				}
			}
		}
		if (sensor_info->sensor_addr != DEFAULT_SENSOR_ADDR) {
			vin_dbg("i2c remap sensor %s as 0x%02x\n",
				sensor_info->sensor_name, sensor_info->sensor_addr);
			single_sensor_i2cmap_setting[1] = (sensor_info->sensor_addr << 1);
			setting_size = sizeof(single_sensor_i2cmap_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, single_sensor_i2cmap_setting);
			if (ret < 0) {
				vin_err("write single_sensor_i2cmap_setting error\n");
				return ret;
			}
		}
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) {
			vin_dbg("0233 9295 start init \n");
			setting_size = sizeof(max9295_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, max9295_init_setting);
			if (ret < 0) {
				vin_err("write max9295_init_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233) {
			if (wait_serializer_9295A_96717_link_locked(sensor_info) != 0) {
				vin_err("Waiting for the deserializer link to time out\n");
				return -RET_ERROR;
			} else {
				vin_info("The deserializer link is successful\n");
			}
			vin_dbg("0233 96717 start init \n");
			setting_size = sizeof(max96717_init_setting_ws) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, max96717_init_setting_ws);
			if (ret < 0) {
				vin_err("write max96717_init_setting_ws error\n");
				return ret;
			}
		}
	}

	if ((sensor_info->config_index & TRIG_STANDARD) ||
	    (sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if (((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M) ||
		    ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL) ||
			((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD) ||
			((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP)) {
			setting_size = sizeof(max9295_trigger_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, max9295_trigger_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
		           (sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
		           (sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
		           (sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
			setting_size = sizeof(max96717_trigger_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max96717 trig\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, max96717_trigger_setting);
			if (ret < 0) {
				vin_err("write max96717_trig_setting error\n");
			}
		}
	}
	/* max9295 need enable LDO */
	if (((sensor_info->extra_mode & 0xff) == SENSING_27M) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_DUAL) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_QUAD) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_TRIP)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}
	vin_dbg("0233 serializer init done\n");
	/* According to the timing requirements of ar0233, the sensor needs to be configured after a delay after reset */
	usleep(20 * 1000);
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	return ret;
}

int32_t sensor_ar0233_serdes_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}

	if ((!strcmp(deserial_if->deserial_name, "s954")) ||
		(!strcmp(deserial_if->deserial_name, "s960"))) {
		ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr, 0x20);
		if (ret < 0) {
			vin_err("serdes start read %s failed\n", deserial_if->deserial_name);
			goto unlock;
		}
		value = (uint8_t)ret;

		if (!strcmp(deserial_if->deserial_name, "s954")) {
			vin_dbg("serdes start read ds954 0x20 value:%02x\n deserial_port %d\n",
				value, sensor_info->deserial_port);
			switch(sensor_info->deserial_port) {
				case 0:
					value &= ~((uint8_t)1 << 4);     //   enable port0
					break;
				case 1:
					value &= ~((uint8_t)1 << 5);     //   enable port1
					break;
			}
		} else if (!strcmp(deserial_if->deserial_name, "s960")) {
			vin_dbg("serdes start read ds960 0x20 value:%02x  deserial_port %d\n",
				value, sensor_info->deserial_port);
			switch(sensor_info->deserial_port) {
				case 0:
					value &= ~((uint8_t)1 << 4);     //   enable port0
					break;
				case 1:
					value &= ~((uint8_t)1 << 5);     //   enable port1
					break;
				case 2:
					value &= ~((uint8_t)1 << 6);     //   enable port2
					break;
				case 3:
					value &= ~((uint8_t)1 << 7);     //   enable port3
					break;
			}
		}

		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr, 0x20, value);
		if (ret < 0) {
			vin_err("write %s failed\n", deserial_if->deserial_name);
			goto unlock;
		}
		vin_dbg("sensor_start write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	} else if (!strcmp(deserial_if->deserial_name, "max9296") ||
			   !strcmp(deserial_if->deserial_name, "max96718")) {
		uint32_t setting_size = sizeof(max9296_start_setting) / sizeof(uint16_t) / 2;
		for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
				max9296_start_setting[2*i], (uint8_t)(max9296_start_setting[2*i+1] & (uint8_t)0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_dbg("sensor_start write %s successfully\n", deserial_if->deserial_name);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			vin_dbg("%s testpattern start\n", deserial_if->deserial_name);
                        uint32_t setting_size = sizeof(max96712_tp_start_setting) / sizeof(uint16_t) / 2;
			for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
						max96712_tp_start_setting[2*i], (uint8_t)(max96712_tp_start_setting[2*i+1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		} else {
			uint32_t setting_size = sizeof(max96712_start_setting) / sizeof(uint16_t) / 2;
			for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
						max96712_start_setting[2*i], (uint8_t)(max96712_start_setting[2*i+1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
                        vin_dbg("sensor_start write %s successfully\n", deserial_if->deserial_name);
		}
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}
unlock:
	return ret;
}

int32_t sensor_ar0233_serdes_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if ((!strcmp(deserial_if->deserial_name, "s954")) ||
		(!strcmp(deserial_if->deserial_name, "s960"))) {
		ret = hb_vin_i2c_read_reg8_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr, 0x20);
		if (ret < 0) {
			vin_err("ov10635 start read ds960 failed\n");
			goto unlock;
		}
		value = (uint8_t)ret;
		if (!strcmp(deserial_if->deserial_name, "s954")) {
			vin_dbg("sensor_stop read s954 0x20 value:%02x deserial_port %d\n",
				value, sensor_info->deserial_port);
			switch(sensor_info->deserial_port) {
				case 0:
					value = value | (uint8_t)0x10;	 	//	 disable port0
					break;
				case 1:
					value = value | (uint8_t)0x20;	 	//	 disable port1
					break;
			}
		}  else if (!strcmp(deserial_if->deserial_name, "s960")) {
			vin_dbg("sensor_stop read s960 0x20 value:%02x deserial_port %d\n",
				value, sensor_info->deserial_port);
			switch(sensor_info->deserial_port) {
				case 0:
					value = value | (uint8_t)0x10;	  //	  disable port0
					break;
				case 1:
					value = value | (uint8_t)0x20;	  //	  disable port1
					break;
				case 2:
					value = value | (uint8_t)0x40;	   //    disable port2
					break;
				case 3:
					value = value | (uint8_t)0x80;	   //    disable port3
					break;
			}
		}

		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr, 0x20, value);
		if (ret < 0) {
			vin_err("write %s failed\n", deserial_if->deserial_name);
			goto unlock;
		}
		vin_dbg("sensor_stop write %s 0x20 value:%02x\n", deserial_if->deserial_name, value);
	} else if (!strcmp(deserial_if->deserial_name, "max9296") ||
			   !strcmp(deserial_if->deserial_name, "max96718")) {
		uint32_t setting_size = sizeof(max9296_stop_setting) / sizeof(uint16_t) / 2;
		for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
				max9296_stop_setting[2*i], (uint8_t)(max9296_stop_setting[2*i+1] & (uint8_t)0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_dbg("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			uint32_t setting_size = sizeof(max96712_tp_stop_setting) / sizeof(uint16_t) / 2;
			for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
						max96712_tp_stop_setting[2*i], (uint8_t)(max96712_tp_stop_setting[2*i+1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		} else {
			uint32_t setting_size = sizeof(max96712_stop_setting) / sizeof(uint16_t) / 2;
			for (uint32_t i = 0; i < setting_size; ++i) {	/* PRQA S 2877 */
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
						max96712_stop_setting[2*i], (uint8_t)(max96712_stop_setting[2*i+1] & (uint8_t)0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		}
		vin_dbg("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}

unlock:
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

	if (((deserial_if != NULL) &&
		(!strcmp(deserial_if->deserial_name, "max96712") ||
		 !strcmp(deserial_if->deserial_name, "max96722"))) &&
		((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
		 (sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820)) {
		bus = deserial_if->bus_num;
		deserial_addr = deserial_if->deserial_addr;
		pdata = galaxy_sepa_max96712_csib_reset;
		setting_size = sizeof(galaxy_sepa_max96712_csib_reset)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	if (!deserial_if ||
		((strcmp(deserial_if->deserial_name, "max96712") ||
		strcmp(deserial_if->deserial_name, "max96722")) &&
		!(sensor_info->config_index & TEST_PATTERN_SERDES))) {
		if ((sensor_info->config_index & TRIG_STANDARD) ||
				(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
			if (sensor_info->start_state == CAM_STOP) {
				if (sensor_info->config_index & TRIG_STANDARD) {
					setting_size = sizeof(ar0233_sync_standard_restart_setting)/
						sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr,
							2, setting_size, ar0233_sync_standard_restart_setting);
					if (ret < 0) {
						vin_err("senor %s write trigger mode setting error\n",
								sensor_info->sensor_name);
						return ret;
					}
				} else {
					setting_size = sizeof(ar0233_sync_shutter_restart_setting)/
								sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr,
							2, setting_size, ar0233_sync_shutter_restart_setting);
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
						sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr,
							2, setting_size, ar0233_trigger_standard_mode_setting);
					if (ret < 0) {
						vin_err("senor %s write trigger mode setting error\n",
								sensor_info->sensor_name);
						return ret;
					}
				} else if (sensor_info->config_index & TRIG_SHUTTER_SYNC) {
					setting_size = sizeof(ar0233_trigger_shuttersync_mode_setting)/
						sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr,
							2, setting_size, ar0233_trigger_shuttersync_mode_setting);
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
				if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP) {
					setting_size = sizeof(ar0233_trigger_gpio3_setting)/sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							setting_size, ar0233_trigger_gpio3_setting);
					if (ret < 0) {
						vin_err("senor %s write trigger gpio3 setting error\n",
								sensor_info->sensor_name);
						return ret;
					}
				} else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_QUAD ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_DUAL ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_25M_TRIP ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_QUAD ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_DUAL ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_TRIP ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_SY0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)WISSEN_WS0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WS0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_WS0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_WITH_GA0820 ||
						(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
					setting_size = sizeof(ar0233_trigger_gpio1_setting)/sizeof(uint16_t)/2;
					ret = sensor_setting_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							setting_size, ar0233_trigger_gpio1_setting);
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
	}

	if (deserial_if) {
		if (!strcmp(deserial_if->deserial_name, "s954") ||
			!strcmp(deserial_if->deserial_name, "s960") ||
			(sensor_info->config_index & DES_STREAMOFF)) {
			ret = sensor_ar0233_serdes_stream_on(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
				vin_err("%d : %s sensor_ar0233_serdes_stream_on fail\n",
						__LINE__, sensor_info->sensor_name);
			}
		} else {
			if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == (uint32_t)GA0233_SEPA_GA0820) {
				entry_num = deserial_if->physical_entry;
				vin_dbg("sepa config use physical_entry %d\n", entry_num);
			}
			req = hb_vin_mipi_pre_request((uint32_t)entry_num, 1, 0);
			if (req == 0) {
				ret = sensor_ar0233_serdes_stream_on(sensor_info);
				if (ret < 0) {
					ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
					vin_err("%d : %s sensor_ar0233_serdes_stream_on fail\n",
							__LINE__, sensor_info->sensor_name);
					return ret;
				}
				hb_vin_mipi_pre_result((uint32_t)entry_num, 1, (uint32_t)ret);
			}
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

	if (!deserial_if ||
		((strcmp(deserial_if->deserial_name, "max96712") ||
		strcmp(deserial_if->deserial_name, "max96722")) &&
		!(sensor_info->config_index & TEST_PATTERN_SERDES))) {
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
		}
		/* Increase the delay to ensure that there is enough time for the next stream on */
		usleep(50 * 1000);
	}
	if (deserial_if) {
		if (!strcmp(deserial_if->deserial_name, "s954") ||
			!strcmp(deserial_if->deserial_name, "s960") ||
			(sensor_info->config_index & DES_STREAMOFF)) {
			ret = sensor_ar0233_serdes_stream_off(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_OFF_FAIL;
				vin_err("%d : %s sensor_ar0233_serdes_stream_off fail\n",
						__LINE__, sensor_info->sensor_name);
			}
		}
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

	if ((si->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M ||
		(si->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_DUAL ||
		(si->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_QUAD ||
		(si->extra_mode & EXT_MODE) == (uint32_t)SENSING_27M_TRIP) {
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

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ar0233, CAM_MODULE_FLAG_A16D16);
sensor_module_t ar0233 = {
	.module = SENSOR_MNAME(ar0233),
#else
sensor_module_t ar0233 = {
	.module = "ar0233",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.get_sns_params = get_sns_info,
};

