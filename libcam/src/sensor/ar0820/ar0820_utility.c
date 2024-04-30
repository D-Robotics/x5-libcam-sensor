/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0820]:" fmt

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
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ar0820_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x18)
#define DEFAULT_GALAXY_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR_A	(0x62)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR		(0x48)
#define DEFAULT_MAX96712_ADDR		(0x29)
#define DEFAULT_MAX9296_ADDR		(0x48)
#define INVALID_CAM_ADDR		(0xFF)

#define WEISEN_LINES_PER_SECOND             (11764)
#define INCEPTIO_LINES_PER_SECOND           (8784)
#define GALAXY_LINES_PER_SECOND             (8784)
#define GAHDR4_LINES_PER_SECOND             (8832)
#define STOP_DELAY_TIME_US	(1800)
#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"

int32_t poc_linked_first(int32_t bus, int32_t poc_addr)
{
	int32_t ret = 0, i, val;
	for (i = 0; i < 4; i++) {
		val = hb_vin_i2c_read_reg8_data8((uint32_t)bus, (uint8_t)poc_addr, (uint16_t)0x6 + (uint16_t)i);
		usleep(2000);
		/* read again to get current state */
		val = hb_vin_i2c_read_reg8_data8((uint32_t)bus, (uint8_t)poc_addr, (uint16_t)0x6 + (uint16_t)i);
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
					r = ((uint16_t)pdata[i + 2] << 8) | (pdata[i + 3]);
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
			reg_addr = ((uint16_t)pdata[i + 2] << 8) | pdata[i + 3];
			value = ((uint16_t)pdata[i + 4] << 8) | pdata[i + 5];
			ret = hb_vin_i2c_write_reg16_data16((uint32_t)bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write ar0820 %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// 	usleep(5*1000);
			i = i + len + 1;
			vin_dbg("write ar0820 %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = ((uint16_t)pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == (uint8_t)DEFAULT_DESERIAL_ADDR) {
				i2c_slave = (uint8_t)deserial_addr;
			} else if (serial_addr != 0 && i2c_slave == (uint8_t)DEFAULT_SERIAL_ADDR_A) {
				i2c_slave = (uint8_t)serial_addr;
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

			ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, reg_addr, (uint8_t)value);
			k = 10;
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
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = pdata[i + 2];
			value = pdata[i + 3];
			if (poc_addr != INVALID_POC_ADDR) {
				if (poc_addr != 0 && i2c_slave == (uint8_t)DEFAULT_POC_ADDR)
					i2c_slave = (uint8_t)poc_addr;
				ret = hb_vin_i2c_write_reg8_data8((uint32_t)bus, i2c_slave, reg_addr, (uint8_t)value);
				if (ret < 0) {
					vin_err("write poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				// usleep(100*1000);
				vin_dbg("write poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
			} else {
				if (reg_addr == (uint16_t)0x01 && value == (uint16_t)0x00) {
					/* reset all serials replace to poc off */
					for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (uint8_t)(DEFAULT_SERIAL_ADDR + 2); i2c_slave++) {
						vin_dbg("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
						hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, 0x0010, 0xf1);
					}
				}
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
		usleep(200*1000);
		return ret;
	}
	return -RET_ERROR;
}

#endif
int32_t deserial_serial_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t setting_size = 0;
	uint8_t *pdata = NULL;
	uint8_t i2c_slave;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;
	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		switch (sensor_info->config_index & 0xff) {
			case (uint32_t)INCEPTIO:
			case (uint32_t)SENSING:
				if (sensor_info->sensor_mode == (uint32_t)DOL2_M) {
					pdata = inceptio_max9296_max9295_2vc_init_setting;
					setting_size = sizeof(inceptio_max9296_max9295_2vc_init_setting)/sizeof(uint8_t);
					pipe_96718 = 0x11;
				} else {
					pdata = inceptio_max9296_max9295_init_setting;
					setting_size = sizeof(inceptio_max9296_max9295_init_setting)/sizeof(uint8_t);
					pipe_96718 = 0x09;
				}
				break;
			case (uint32_t)SENSING_RAW12_EMB:
				pdata = inceptio_max9296_max9295_init_setting_raw12_emb;
				setting_size = sizeof(inceptio_max9296_max9295_init_setting_raw12_emb)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case (uint32_t)SENSING_RAW16_EMB:
				pdata = inceptio_max9296_max9295_init_setting_raw16_emb;
				setting_size = sizeof(inceptio_max9296_max9295_init_setting_raw16_emb)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case (uint32_t)WEISEN:
			case (uint32_t)GALAXY:
				if (sensor_info->sensor_mode == (uint32_t)DOL2_M) {
					pdata = weisen_max9296_max9295_2vc_init_setting;
					setting_size = sizeof(weisen_max9296_max9295_2vc_init_setting)/sizeof(uint8_t);
					pipe_96718 = 0x11;
				} else {
					pdata = weisen_max9296_max9295_init_setting;
					setting_size = sizeof(weisen_max9296_max9295_init_setting)/sizeof(uint8_t);
					pipe_96718 = 0x09;
				}
				break;
			case (uint32_t)WEISEN_RAW12_EMB:
				pdata = weisen_max9296_max9295_init_setting_raw12_emb;
				setting_size = sizeof(weisen_max9296_max9295_init_setting_raw12_emb)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case (uint32_t)NURO:
				pdata = nuro_max9296_max9295_init_setting;
				setting_size = sizeof(nuro_max9296_max9295_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x11;
				break;
			case (uint32_t)CONTI:
				pdata = conti_max9296_max9295_init_setting;
				setting_size = sizeof(conti_max9296_max9295_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case (uint32_t)WEISEN_DUAL:
				pdata = weisen_max9296_max9295_dual_init_setting;
				setting_size = sizeof(weisen_max9296_max9295_dual_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x25;
				break;
			case (uint32_t)SENSING_DUAL:
				pdata = sensing_max9296_max9295_dual_init_setting;
				setting_size = sizeof(sensing_max9296_max9295_dual_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x25;
				break;
			case (uint32_t)SENSING_WS0233:
				pdata = sensing_max9296_max9295_max96717_init_setting;
				setting_size = sizeof(sensing_max9296_max9295_max96717_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x25;
				break;
			case (uint32_t)WEISEN_WS0233:
				pdata = weisen_max9296_max9295_max96717_init_setting;
				setting_size = sizeof(weisen_max9296_max9295_max96717_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x25;
				break;
			case (uint32_t)GALAXY_WITH_GAX3C:
			case (uint32_t)GALAXY_SEPA_GAX3C:
			case (uint32_t)GALAXY_WITH_GA0233:
			case (uint32_t)GALAXY_SEPA_GA0233:
				vin_err("config_index err: galaxy not support %s\n", deserial_if->deserial_name);
				return -RET_ERROR;
			default:
				vin_err("config_index is err\n");
				return -RET_ERROR;
 		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			!strcmp(deserial_if->deserial_name, "max96722")) {
		uint8_t poc_a, poc_first, des_port = 0, des_link_en = 0, des_pipe0_sel = 0;
		switch (sensor_info->config_index & 0xff) {
			case (uint32_t)INCEPTIO:
			case (uint32_t)WEISEN:
			case (uint32_t)GALAXY:
			case (uint32_t)SENSING:
				if ((sensor_info->config_index & 0xff) == (uint32_t)WEISEN ||
					(sensor_info->config_index & 0xff) == (uint32_t)GALAXY) {
					pdata = weisen_max96712_max9295_init_setting;
					setting_size = sizeof(weisen_max96712_max9295_init_setting)/sizeof(uint8_t);
				} else {
					pdata = sensing_max96712_max9295_init_setting;
					setting_size = sizeof(sensing_max96712_max9295_init_setting)/sizeof(uint8_t);
				}
				/* auto detect for one link on 96712 */
				if ((poc_addr != INVALID_POC_ADDR) && (sensor_info->deserial_port == 0)) {
					poc_a = (uint8_t)((poc_addr) ? poc_addr : DEFAULT_POC_ADDR);
					poc_first = (uint8_t)poc_linked_first(bus, poc_a);
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
					vin_dbg("hdr 3exp port-%c config mode!\n", des_port);
					setting_modify(pdata, setting_size, DEFAULT_DESERIAL_ADDR, 0x6, des_link_en);
					setting_modify(pdata, setting_size, DEFAULT_DESERIAL_ADDR, 0xf0, des_pipe0_sel);
				} else {
					vin_dbg("hdr 3exp %s config mode!\n", deserial_if->deserial_name);
				}
				break;
			case (uint32_t)WEISEN_DUAL:
				pdata = weisen_max96712_max9295_dual_init_setting;
				setting_size = sizeof(weisen_max96712_max9295_dual_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)SENSING_DUAL:
				pdata = sensing_max96712_max9295_dual_init_setting;
				setting_size = sizeof(sensing_max96712_max9295_dual_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)SENSING_WS0233:
				pdata = sensing_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(sensing_max96712_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)WEISEN_WS0233:
				pdata = weisen_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(weisen_max96712_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)GALAXY_WITH_GAX3C:
				pdata = galaxy_with_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_with_max96712_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)GALAXY_SEPA_GAX3C:
				pdata = galaxy_sepa_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_sepa_max96712_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)GALAXY_WITH_GA0233:
				pdata = galaxy_with_max96712_max9295_max96717_init2_setting;
				setting_size = sizeof(galaxy_with_max96712_max9295_max96717_init2_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)GALAXY_SEPA_GA0233:
				pdata = galaxy_sepa_max96712_max9295_max96717_init2_setting;
				setting_size = sizeof(galaxy_sepa_max96712_max9295_max96717_init2_setting)/sizeof(uint8_t);
				break;
			case (uint32_t)SENSING_RAW12_EMB:
			case (uint32_t)SENSING_RAW16_EMB:
			case (uint32_t)WEISEN_RAW12_EMB:
				vin_err("%s not support emb\n", deserial_if->deserial_name);
				return -RET_ERROR;
			default:
				vin_err("config_index is err\n");
				return -RET_ERROR;
		}
	} else {
		vin_err("des %s not support err\n", deserial_if->deserial_name);
		return -RET_ERROR;
	}
#ifdef POC_RETRY_POLICY
	if (poc_addr != INVALID_POC_ADDR) {
		ret = poc_power_reset(sensor_info);
		if (ret < 0) {
			vin_err("poc_power_reset fail\n");
			return ret;
		}
	} else {
		for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (uint8_t)(DEFAULT_SERIAL_ADDR + 2); i2c_slave++) {
			vin_dbg("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
			hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, 0x0010, 0xf1);
		}
	}
#endif

	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}
	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		pdata = max9296_add_max96718_init_setting;
		setting_size = sizeof(max9296_add_max96718_init_setting)/sizeof(uint8_t);
		if (pipe_96718 != 0u) {
			pdata[4] = pipe_96718;
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max9296_add_max96718_init_setting error\n");
			return ret;
		}
	}
	/* i2c addr remap for galaxy */
	if ((sensor_info->config_index & 0xff) == (uint32_t)GALAXY ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GAX3C ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GA0233 ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233) {
		if (((sensor_info->sensor_addr & 0xFC) != DEFAULT_GALAXY_ADDR) &&
			((sensor_info->sensor_addr & 0x3) != 0x0)) {
			pdata = galaxy_maxser_sensor_i2cmap_setting;
			setting_size = sizeof(galaxy_maxser_sensor_i2cmap_setting)/sizeof(uint8_t);
			pdata[4] = (uint8_t)(sensor_info->sensor_addr << 1);
			pdata[9] = (uint8_t)((sensor_info->sensor_addr & 0xFC) << 1);
			vin_dbg("map ar0820 0x%02x as 0x%02x for galaxy\n",
					(sensor_info->sensor_addr & 0xFC), sensor_info->sensor_addr);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write galaxy_maxser_sensor_i2cmap_setting error\n");
				return ret;
			}
		}
	}
	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->config_index & 0xff) != (uint32_t)WEISEN_DUAL &&
			(sensor_info->config_index & 0xff) != (uint32_t)SENSING_DUAL &&
			(sensor_info->config_index & 0xff) != (uint32_t)SENSING_WS0233 &&
			(sensor_info->config_index & 0xff) != (uint32_t)WEISEN_WS0233) {
			if (sensor_info->extra_mode & DPHY_COPY) {
				pdata = max9296_phy_portall_init_setting;
				setting_size = sizeof(max9296_phy_portall_init_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
									 sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write phy portall error\n");
					return ret;
				}
			} else if (sensor_info->extra_mode & DPHY_PORTB) {
				pdata = max9296_phy_portb_init_setting;
				setting_size = sizeof(max9296_phy_portb_init_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
									 sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write phy portb error\n");
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
				if(ret < 0) {
					vin_err("write max9296_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			!strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->extra_mode & DPHY_PORTB) {
			pdata = max96712_phy_portb_init_setting;
			setting_size = sizeof(max96712_phy_portb_init_setting)/
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_phy_portb register error\n");
				return ret;
			}
			if (sensor_info->extra_mode & DPHY_COPY) {
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
			if (sensor_info->extra_mode & DPHY_COPY) {
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
				regaddr = trigger_reg[2*i + (uint32_t)2*offset] + size;
				vin_dbg("write mfp: w%d@0x%02x 0x%04x=0x%02x\n", deserial_if->bus_num,	/*PRQA S 1891,1860,1861*/
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      (uint8_t)deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i + 2*(uint32_t)offset + 1] & (uint32_t)0xFF));
				if (ret < 0) {
					vin_err("%s write max96712_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	}
	deserial_if->init_state = 1;
	return ret;
}

int32_t ar0820_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (sensor_info->extra_mode & EXT_MASK) {
		/* zu3 */
		uint32_t extra_mode = (sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS;
		uint16_t vts, hts;
		int32_t i, rs;
		REG_INFO *r;
		uint8_t *hv;
		if (sensor_info->sensor_mode == (uint32_t)PWL_M) {
			r = (sensor_info->resolution == 1080) ? ar0820_extra_1080p_pwl_init_s : ar0820_extra_2160p_pwl_init_s;
			rs = (sensor_info->resolution == 1080) ? sizeof(ar0820_extra_1080p_pwl_init_s)/sizeof(REG_INFO) :
				sizeof(ar0820_extra_2160p_pwl_init_s)/sizeof(REG_INFO);
			hv = ar0820_extra_pwl_hts_vts_setting;
		} else {
			r = (sensor_info->resolution == 1080) ? ar0820_extra_1080p_linear_init_s : ar0820_extra_2160p_linear_init_s;
			rs = (sensor_info->resolution == 1080) ? sizeof(ar0820_extra_1080p_linear_init_s)/sizeof(REG_INFO) :
				sizeof(ar0820_extra_2160p_linear_init_s)/sizeof(REG_INFO);
			hv = ar0820_extra_linear_hts_vts_setting;
		}
		if (extra_mode != 1) {
			vts = (uint16_t)(extra_mode >> 12);
			hts = (uint16_t)((extra_mode & 0xfff) * 16);
			vin_dbg("%s %dp %s: hts=%d(0x%04x) vts=%d(0x%04x)\n", sensor_info->sensor_name, sensor_info->resolution,
				(sensor_info->sensor_mode == (uint32_t)PWL_M) ? "pwl" : "linear", hts, hts, vts, vts);
			hv[4] = (uint8_t)(hts >> 8);
			hv[5] = (uint8_t)(hts & (uint8_t)0xff);
			hv[10] = (uint8_t)(vts >> 8);
			hv[11] = (uint8_t)(vts & (uint8_t)0xff);
		} else {
			hts = ((uint16_t)hv[4] << 8) + hv[5];
			vts = ((uint16_t)hv[10] << 8) + hv[11];
			vin_dbg("%s %dp %s: hts=%d(0x%04x) vts=%d(0x%04x)\n", sensor_info->sensor_name, sensor_info->resolution,
				(sensor_info->sensor_mode == (uint32_t)PWL_M) ? "pwl" : "linear", hts, hts, vts, vts);
		}
		for (i = 0; i < rs; i++) {
			pdata = r[i].setting;
			setting_size = r[i].size;
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0)
				vin_err("write register error\n");
		}
		deserial_if->init_state = 1;
	} else {
		/* xj3 / j5 */
		if(sensor_info->sensor_mode == (uint32_t)NORMAL_M) {
			pdata = ar0820_linear_30fps_init_setting;
			setting_size = sizeof(ar0820_linear_30fps_init_setting)/sizeof(uint8_t);
			vin_dbg("linear config mode!\n");
		} else if (sensor_info->sensor_mode == (uint32_t)DOL2_M) {
			pdata = ar0820_dol2_15fps_init_setting;
			setting_size = sizeof(ar0820_dol2_15fps_init_setting)/sizeof(uint8_t);
			vin_dbg("hdr dol2 config mode!\n");
		} else if (sensor_info->sensor_mode == (uint32_t)PWL_M) {
			if (sensor_info->extra_mode & PWL_HDR4_24BIT) {
				pdata = ar0820_hdr_4exp_30fps_init_setting;
				setting_size = sizeof(ar0820_hdr_4exp_30fps_init_setting)/sizeof(uint8_t);
				vin_dbg("hdr 4exp pwl config mode!\n");
			} else {
				pdata = ar0820_hdr_3exp_30fps_init_setting;
				setting_size = sizeof(ar0820_hdr_3exp_30fps_init_setting)/sizeof(uint8_t);
				vin_dbg("hdr 3exp pwl config mode!\n");
			}
		} else {
			vin_err("config mode is err\n");
			return -RET_ERROR;
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// pll setting
	if(sensor_info->extra_mode & XO_25MHZ) {
		if ((deserial_if &&
				(!strcmp(deserial_if->deserial_name, "max96712") ||
				 !strcmp(deserial_if->deserial_name, "max96722") ||
				 ((!strcmp(deserial_if->deserial_name, "max9296") &&
				   ((sensor_info->config_index & 0xff) == (uint32_t)WEISEN_DUAL ||
				    (sensor_info->config_index & 0xff) == (uint32_t)GALAXY ||
				    (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GAX3C ||
				    (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
				    (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GA0233 ||
				    (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233))))) ||
				((sensor_info->config_index & 0xff) == (uint32_t)WEISEN_RAW12_EMB)) {
			pdata = ar0820_pll_multiplier_hvkeep;
			setting_size = sizeof(ar0820_pll_multiplier_hvkeep)/sizeof(uint8_t);
			vin_dbg("25M pll hv keep settint!\n");
		} else {
			pdata = ar0820_pll_multiplier;
			setting_size = sizeof(ar0820_pll_multiplier)/sizeof(uint8_t);
			vin_dbg("25M pll settint!\n");
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// the special setting for weisen ar0820
	if((sensor_info->config_index & 0xff) == (uint32_t)WEISEN ||
		(sensor_info->config_index & 0xff) == (uint32_t)WEISEN_DUAL ||
		(sensor_info->config_index & 0xff) == (uint32_t)WEISEN_RAW12_EMB ||
		(sensor_info->config_index & 0xff) == (uint32_t)WEISEN_WS0233 ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GAX3C ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GA0233 ||
		(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233) {
		/* filp and mirror disable */
		pdata = ar0820_filp_mirror_disable;
		setting_size = sizeof(ar0820_filp_mirror_disable)/sizeof(uint8_t);
		vin_dbg("disable flip and mirror!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}

		/* awb init setting */
		if ((sensor_info->config_index & 0xff) == (uint32_t)GALAXY ||
			(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GAX3C ||
			(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
			(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GA0233 ||
			(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233) {
			if ((sensor_info->extra_mode & PWL_HDR4_24BIT) == 0) {
				vin_dbg("galaxy 3exp pwl setting!\n");
				pdata = ar0820_hdr_3exp_galaxy_pwl_setting;
				setting_size = sizeof(ar0820_hdr_3exp_galaxy_pwl_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write register error\n");
					return ret;
				}
			}
		} else {
			if (sensor_info->extra_mode & EXT_MASK) {
				/* zu3 */
				pdata = ar0820_awb_extra_init_setting;
				setting_size = sizeof(ar0820_awb_extra_init_setting)/sizeof(uint8_t);
			} else {
				/* xj3 / j5 */
				pdata = ar0820_awb_init_setting;
				setting_size = sizeof(ar0820_awb_init_setting)/sizeof(uint8_t);
			}
			vin_dbg("awb init setting!\n");
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write register error\n");
				return ret;
			}
		}
	}

	// the special setting for ar0820 emb
	if((sensor_info->config_index & 0xff) == (uint32_t)SENSING_RAW12_EMB ||
		(sensor_info->config_index & 0xff) == (uint32_t)SENSING_RAW16_EMB ||
		(sensor_info->config_index & 0xff) == (uint32_t)WEISEN_RAW12_EMB) {
		pdata = ar0820_enalbe_embedded;
		setting_size = sizeof(ar0820_enalbe_embedded)/sizeof(uint8_t);
		vin_dbg("emb settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// the special setting for ar0820 raw16.
	if((sensor_info->config_index & 0xff) == (uint32_t)SENSING_RAW16_EMB) {
		pdata = ar0820_datatype_raw16;
		setting_size = sizeof(ar0820_datatype_raw16)/sizeof(uint8_t);
		vin_dbg("raw16 settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// test pattern enable
	if(sensor_info->extra_mode & TEST_PATTERN) {
		pdata = ar0820_test_pattern;
		setting_size = sizeof(ar0820_test_pattern)/sizeof(uint8_t);
		vin_dbg("test pattern init!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}

	// 1080p?
	if(sensor_info->resolution == 1080) {
		if (sensor_info->extra_mode & SCALER_WEIGHT9331) {
			pdata = ar0820_extra_binning10_setting;
			setting_size = sizeof(ar0820_extra_binning10_setting)/sizeof(uint8_t);
		} else {
			pdata = ar0820_extra_binning_setting;
			setting_size = sizeof(ar0820_extra_binning_setting)/sizeof(uint8_t);
		}
		vin_dbg("1080p binning %d settint!\n", pdata[5]);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->extra_mode & SCALER_WEIGHT9331) {
		pdata = ar0820_weight_9331_scaling_setting;
		setting_size = sizeof(ar0820_weight_9331_scaling_setting)/sizeof(uint8_t);
		vin_dbg("1080p 9:3:3:1 scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->extra_mode & SCALER_TRUE_BAYER) {
		pdata = ar0820_true_bayer_scaling_setting;
		setting_size = sizeof(ar0820_true_bayer_scaling_setting)/sizeof(uint8_t);
		vin_dbg("scaler true bayer scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else if (sensor_info->extra_mode & SCALER_WEIGHT2110) {
		pdata = ar0820_weight_2110_scaling_setting;
		setting_size = sizeof(ar0820_weight_9331_scaling_setting)/sizeof(uint8_t);
		vin_dbg("scaler weight 2:1:1:0 scaling settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}
	// fps modify & div.
	if (((sensor_info->fps >= 5) && (sensor_info->fps < 30)) ||
		((sensor_info->fps != 0) && (sensor_info->extra_mode & FPS_DIV))) {
		char init_d[3];
		uint32_t vts_v, vts_s, target_fps = sensor_info->fps;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_VTS, init_d, 2);
		vts_v = (init_d[0] << 8) | init_d[1];
		vts_s = vts_v * 30 / target_fps;
		if (sensor_info->extra_mode & FPS_DIV) {
			target_fps = target_fps / 2;
			vts_s = vts_s * 2;
		}
		vin_info("%dfps settint, vts %d to %d!\n", target_fps, vts_v, vts_s);
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_VTS, vts_s);
		if (ret < 0)
			vin_err("write register error\n");
	}
	return ret;
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
				ret =  (int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
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
		sizeof(turning_data->sensor_name)-1);
	return;
}
int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	uint8_t init_d[3];
	uint32_t x0, m_y0, x1, m_y1, width, height;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_VTS, init_d, 2);
	turning_data->sensor_data.VMAX = init_d[0];
	turning_data->sensor_data.VMAX  = (turning_data->sensor_data.VMAX  << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_HTS, init_d, 2);
	turning_data->sensor_data.HMAX = init_d[0];
	turning_data->sensor_data.HMAX = (turning_data->sensor_data.HMAX << 8) | init_d[1];

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_X_START, init_d, 2);
	x0 = init_d[0];
	x0 = (x0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_Y_START, init_d, 2);
	m_y0 = init_d[0];
	m_y0 = (m_y0 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_X_END, init_d, 2);
	x1 = init_d[0];
	x1 = (x1 << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_Y_END, init_d, 2);
	m_y1 = init_d[0];
	m_y1 = (m_y1 << 8) | init_d[1];
	width = x1 - x0 + 1;
	height = m_y1 - m_y0 + 1;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;

	// set tuning data for different ar0820
	if (sensor_info->extra_mode & EXT_MASK) {
		/* zu3 */
		switch (sensor_info->config_index & 0xff) {
			case (uint32_t)WEISEN:
			case (uint32_t)WEISEN_RAW12_EMB:
			case (uint32_t)WEISEN_DUAL:
			case (uint32_t)WEISEN_WS0233:
			case (uint32_t)GALAXY:
			case (uint32_t)GALAXY_WITH_GAX3C:
			case (uint32_t)GALAXY_SEPA_GAX3C:
			case (uint32_t)GALAXY_WITH_GA0233:
			case (uint32_t)GALAXY_SEPA_GA0233:
				turning_data->sensor_data.gain_max = 128 * 8192;
				turning_data->sensor_data.analog_gain_max = 190 * 8192;
				turning_data->sensor_data.digital_gain_max = 0 * 8192;
				turning_data->sensor_data.exposure_time_min = 1;
				turning_data->sensor_data.exposure_time_max = 4000;
				turning_data->sensor_data.exposure_time_long_max = 4000;
				turning_data->sensor_data.lines_per_second = 1141;  // 156M / 4440
				turning_data->sensor_data.turning_type = 6;   // gain calc
				turning_data->sensor_data.conversion = 1;
				sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
				if (sensor_info->extra_mode & PWL_HDR4_24BIT) {
					sensor_data_bits_fill(&turning_data->sensor_data, 24);
				} else {
					sensor_data_bits_fill(&turning_data->sensor_data, 20);
				}
				break;
			case (uint32_t)SENSING:
			case (uint32_t)SENSING_RAW12_EMB:
			case (uint32_t)SENSING_RAW16_EMB:
			case (uint32_t)SENSING_DUAL:
			case (uint32_t)SENSING_WS0233:
			default:
				turning_data->sensor_data.gain_max = 128 * 8192;
				turning_data->sensor_data.analog_gain_max = 158 * 8192;
				turning_data->sensor_data.digital_gain_max = 0 * 8192;
				turning_data->sensor_data.exposure_time_min = 1;
				turning_data->sensor_data.exposure_time_max = 4000;
				turning_data->sensor_data.exposure_time_long_max = 4000;
				turning_data->sensor_data.lines_per_second = 1141;
				turning_data->sensor_data.turning_type = 6;   // gain calc
				turning_data->sensor_data.conversion = 1;
				sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RCCB);
				sensor_data_bits_fill(&turning_data->sensor_data, 20);
				break;
		}
		return ret;
	}

	/* xj3 / j5 */
	switch (sensor_info->config_index & 0xff) {
		case (uint32_t)INCEPTIO:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 158 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = INCEPTIO_LINES_PER_SECOND;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RCCB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		case (uint32_t)WEISEN:
		case (uint32_t)WEISEN_RAW12_EMB:
		case (uint32_t)WEISEN_DUAL:
		case (uint32_t)WEISEN_WS0233:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = WEISEN_LINES_PER_SECOND;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		case (uint32_t)GALAXY:
		case (uint32_t)GALAXY_WITH_GAX3C:
		case (uint32_t)GALAXY_SEPA_GAX3C:
		case (uint32_t)GALAXY_WITH_GA0233:
		case (uint32_t)GALAXY_SEPA_GA0233:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 1;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			if (sensor_info->extra_mode & PWL_HDR4_24BIT) {
				turning_data->sensor_data.lines_per_second = GAHDR4_LINES_PER_SECOND;  // 156M / 1104 / 16
				sensor_data_bits_fill(&turning_data->sensor_data, 24);
			} else {
				turning_data->sensor_data.lines_per_second = GALAXY_LINES_PER_SECOND;  // 156M / 1480 / 12
				sensor_data_bits_fill(&turning_data->sensor_data, 20);
			}
			break;
		case (uint32_t)NURO:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = 8784;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		case (uint32_t)CONTI:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 190 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = 11764;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RCCB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		case (uint32_t)SENSING:
		case (uint32_t)SENSING_RAW12_EMB:
		case (uint32_t)SENSING_RAW16_EMB:
		case (uint32_t)SENSING_DUAL:
		case (uint32_t)SENSING_WS0233:
			turning_data->sensor_data.gain_max = 128 * 8192;
			turning_data->sensor_data.analog_gain_max = 158 * 8192;
			turning_data->sensor_data.digital_gain_max = 0 * 8192;
			turning_data->sensor_data.exposure_time_min = 20;
			turning_data->sensor_data.exposure_time_max = 4000;
			turning_data->sensor_data.exposure_time_long_max = 4000;
			turning_data->sensor_data.lines_per_second = 8784;  // 156M / 4440
			turning_data->sensor_data.turning_type = 6;   // gain calc
			turning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&turning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&turning_data->sensor_data, 20);
			break;
		default:
			vin_err("don't support config_index %d\n", sensor_info->config_index);
			return -1;
	}
	return ret;
}
static int32_t sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK, i, cnt;
	uint8_t *stream_src;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	cnt = sizeof(ar0820_stream_on_setting) / 6;
	stream_src = ar0820_stream_on_setting;
	if (sizeof(turning_data->stream_ctrl.stream_on) >= (uint64_t)(cnt * 8)) {/* PRQA S 2995, 2991 */
		for (i = 0; i < cnt; i++) {/* PRQA S 2877 */
			stream_on[i * 2] = ((uint32_t)stream_src[i * 6 + 2] << 8) | (stream_src[i * 6 + 3]);
			stream_on[i * 2 + 1] = ((uint32_t)stream_src[i * 6 + 4] << 8) | (stream_src[i * 6 + 5]);
		}
	} else {
		vin_err("Number of registers on stream over 10\n");/* PRQA S 2880 */
		return -RET_ERROR;
	}
	cnt = sizeof(ar0820_stream_off_setting) / 6;
	stream_src = ar0820_stream_off_setting;
	if (sizeof(turning_data->stream_ctrl.stream_off) >= (uint64_t)(cnt * 8)) {/* PRQA S 2995, 2991 */
		for (i = 0; i < cnt; i++) {/* PRQA S 2877 */
			stream_off[i * 2] = (((uint32_t)stream_src[i * 6 + 2] << 8) | (stream_src[i * 6 + 3]));
			stream_off[i * 2 + 1] = (((uint32_t)stream_src[i * 6 + 4] << 8) | (stream_src[i * 6 + 5]));
		}
	} else {
		vin_err("Number of registers on stream over 10\n");/* PRQA S 2880 */
		return -RET_ERROR;
	}
	return ret;
}

int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;
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

	turning_data.normal.param_hold = AR0820_PARAM_HOLD;
	turning_data.normal.param_hold_length = 2;
	turning_data.normal.s_line = AR0820_LINE;
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
	turning_data.normal.again_control_num = 3;
	turning_data.normal.again_control[0] = AR0820_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.again_control[1] = AR0820_FINE_GAIN;
	turning_data.normal.again_control_length[1] = 2;
	turning_data.normal.again_control[2] = AR0820_DGAIN;
	turning_data.normal.again_control_length[2] = 2;
	turning_data.normal.dgain_control_num = 0;
	turning_data.normal.dgain_control[0] = 0;
	turning_data.normal.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	// turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	// turning_data.sensor_awb.grgain_length[0] = 2;
	// turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	// turning_data.sensor_awb.grgain_length[1] = 2;
	// turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	// turning_data.sensor_awb.grgain_length[2] = 2;
	// turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	// turning_data.sensor_awb.gbgain_length[0] = 2;
	// turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	// turning_data.sensor_awb.gbgain_length[1] = 2;
	// turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	// turning_data.sensor_awb.gbgain_length[2] = 2;
	turning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->config_index & 0xff) {
		case (uint32_t)INCEPTIO:
		case (uint32_t)SENSING:
		case (uint32_t)SENSING_RAW12_EMB:
		case (uint32_t)SENSING_RAW16_EMB:
		case (uint32_t)SENSING_DUAL:
		case (uint32_t)SENSING_WS0233:
		case (uint32_t)CONTI:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case (uint32_t)WEISEN:
		case (uint32_t)WEISEN_RAW12_EMB:
		case (uint32_t)WEISEN_DUAL:
		case (uint32_t)WEISEN_WS0233:
		case (uint32_t)GALAXY_WITH_GAX3C:
		case (uint32_t)GALAXY_SEPA_GAX3C:
		case (uint32_t)GALAXY_WITH_GA0233:
		case (uint32_t)GALAXY_SEPA_GA0233:
		case (uint32_t)GALAXY:
		case (uint32_t)NURO:
			ar0820_gain.pdata = rggb_ar0820_gain;
			ar0820_gain.size = sizeof(rggb_ar0820_gain);

			ar0820_dgain.pdata = rggb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

			ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			break;
		default:
			vin_err("don't support config_index %d\n", sensor_info->config_index);
			return -1;
	}

	// set lut table
	turning_data.normal.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.normal.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.normal.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.normal.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[512 + open_cnt], 2);
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

	return ret;
}

static int32_t sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt;
	char str[24] = {0};
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}
	turning_data.dol2.param_hold = AR0820_PARAM_HOLD;
	turning_data.dol2.param_hold_length = 2;

	turning_data.dol2.s_line = AR0820_LINE_S;
	turning_data.dol2.s_line_length = 2;

	turning_data.dol2.m_line = AR0820_LINE;
	turning_data.dol2.m_line_length = 2;

#ifdef TUNING_LUT
	turning_data.dol2.line_p[0].ratio = 1 << 8;
	turning_data.dol2.line_p[0].offset = 0;
	turning_data.dol2.line_p[0].max = 19;
	turning_data.dol2.line_p[1].ratio = 1 << 8;
	turning_data.dol2.line_p[1].offset = 0;
	turning_data.dol2.line_p[1].max = 4000;

	turning_data.dol2.again_control_num = 3;
	turning_data.dol2.again_control[0] = AR0820_GAIN;
	turning_data.dol2.again_control_length[0] = 2;
	turning_data.dol2.again_control[1] = AR0820_FINE_GAIN;
	turning_data.dol2.again_control_length[1] = 2;
	turning_data.dol2.again_control[2] = AR0820_DGAIN;
	turning_data.dol2.again_control_length[2] = 2;
	turning_data.dol2.dgain_control_num = 0;
	turning_data.dol2.dgain_control[0] = 0;
	turning_data.dol2.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	turning_data.sensor_awb.grgain_length[0] = 2;
	turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	turning_data.sensor_awb.grgain_length[1] = 2;
	turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	turning_data.sensor_awb.grgain_length[2] = 2;
	turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	turning_data.sensor_awb.gbgain_length[0] = 2;
	turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	turning_data.sensor_awb.gbgain_length[1] = 2;
	turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	turning_data.sensor_awb.gbgain_length[2] = 2;
	turning_data.sensor_awb.rb_prec = 7;

	turning_data.dol2.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (turning_data.dol2.again_lut != NULL) {
		memset(turning_data.dol2.again_lut, 0xff, 256*2*sizeof(uint32_t));
		memcpy(turning_data.dol2.again_lut,
			rggb_ar0820_gain, sizeof(rggb_ar0820_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[open_cnt], 2);
		}
		memcpy(turning_data.dol2.again_lut + 256,
			rggb_ar0820_fine_gain, sizeof(rggb_ar0820_fine_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_fine_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.dol2.again_lut + 512,
			rggb_ar0820_dgain, sizeof(rggb_ar0820_dgain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_dgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.dol2.again_lut[512 + open_cnt], 2);
		}
	}
#endif
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);	/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}
	if (turning_data.dol2.again_lut)
		free(turning_data.dol2.again_lut);	/* PRQA S 5118 */
	if (turning_data.dol2.dgain_lut)
		free(turning_data.dol2.dgain_lut);	/* PRQA S 5118 */


	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt = 0;
	char str[24] = {0};
	sensor_turning_data_t turning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if(sensor_info->sen_devfd <= 0) {
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0) {
			vin_err("port%d: %s open fail\n", sensor_info->port, str);
			return -RET_ERROR;
		}
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}
	turning_data.pwl.param_hold = AR0820_PARAM_HOLD;
	turning_data.pwl.param_hold_length = 2;
	turning_data.pwl.line = AR0820_LINE;
	turning_data.pwl.line_length = 2;

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	turning_data.pwl.line_p.ratio = 1 << 8;
	turning_data.pwl.line_p.offset = 0;
	turning_data.pwl.line_p.max = 4000;

	turning_data.pwl.again_control_num = 3;
	turning_data.pwl.again_control[0] = AR0820_GAIN;
	turning_data.pwl.again_control_length[0] = 2;
	turning_data.pwl.again_control[1] = AR0820_FINE_GAIN;
	turning_data.pwl.again_control_length[1] = 2;
	turning_data.pwl.again_control[2] = AR0820_DGAIN;
	turning_data.pwl.again_control_length[2] = 2;
	turning_data.pwl.dgain_control_num = 0;
	turning_data.pwl.dgain_control[0] = 0;
	turning_data.pwl.dgain_control_length[0] = 0;

	turning_data.sensor_awb.bgain_addr[0] = 0x3058;
	turning_data.sensor_awb.bgain_length[0] = 2;
	turning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	turning_data.sensor_awb.bgain_length[1] = 2;
	turning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	turning_data.sensor_awb.bgain_length[2] = 2;
	turning_data.sensor_awb.rgain_addr[0] = 0x305a;
	turning_data.sensor_awb.rgain_length[0] = 2;
	turning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	turning_data.sensor_awb.rgain_length[1] = 2;
	turning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	turning_data.sensor_awb.rgain_length[2] = 2;
	turning_data.sensor_awb.grgain_addr[0] = 0x3056;
	turning_data.sensor_awb.grgain_length[0] = 2;
	turning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	turning_data.sensor_awb.grgain_length[1] = 2;
	turning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	turning_data.sensor_awb.grgain_length[2] = 2;
	turning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	turning_data.sensor_awb.gbgain_length[0] = 2;
	turning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	turning_data.sensor_awb.gbgain_length[1] = 2;
	turning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	turning_data.sensor_awb.gbgain_length[2] = 2;
	if (sensor_info->extra_mode & PWL_HDR4_24BIT) {
		turning_data.sensor_awb.bgain_addr[3] = 0x35b2;
		turning_data.sensor_awb.bgain_length[3] = 2;
		turning_data.sensor_awb.rgain_addr[3] = 0x35b4;
		turning_data.sensor_awb.rgain_length[3] = 2;
		turning_data.sensor_awb.grgain_addr[3] = 0x35b0;
		turning_data.sensor_awb.grgain_length[3] = 2;
		turning_data.sensor_awb.gbgain_addr[3] = 0x35b6;
		turning_data.sensor_awb.gbgain_length[3] = 2;
	}
	turning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->config_index & 0xff) {
		case (uint32_t)INCEPTIO:
		case (uint32_t)SENSING:
		case (uint32_t)SENSING_RAW12_EMB:
		case (uint32_t)SENSING_RAW16_EMB:
		case (uint32_t)SENSING_DUAL:
		case (uint32_t)SENSING_WS0233:
		case (uint32_t)CONTI:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case (uint32_t)WEISEN:
		case (uint32_t)WEISEN_RAW12_EMB:
		case (uint32_t)WEISEN_DUAL:
		case (uint32_t)WEISEN_WS0233:
		case (uint32_t)NURO:
			ar0820_gain.pdata = rggb_ar0820_gain;
			ar0820_gain.size = sizeof(rggb_ar0820_gain);

			ar0820_dgain.pdata = rggb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

			ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			break;
		case (uint32_t)GALAXY_WITH_GAX3C:
		case (uint32_t)GALAXY_SEPA_GAX3C:
		case (uint32_t)GALAXY_WITH_GA0233:
		case (uint32_t)GALAXY_SEPA_GA0233:
		case (uint32_t)GALAXY:
			if (sensor_info->extra_mode & PWL_HDR4_24BIT) {
				ar0820_gain.pdata = rggb_ar0820_hdr4_gain;
				ar0820_gain.size = sizeof(rggb_ar0820_hdr4_gain);

				ar0820_dgain.pdata = rggb_ar0820_dgain;
				ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

				ar0820_fine_gain.pdata = rggb_ar0820_hdr4_fine_gain;
				ar0820_fine_gain.size = sizeof(rggb_ar0820_hdr4_fine_gain);
			} else {
				ar0820_gain.pdata = rggb_ar0820_gain;
				ar0820_gain.size = sizeof(rggb_ar0820_gain);

				ar0820_dgain.pdata = rggb_ar0820_dgain;
				ar0820_dgain.size = sizeof(rggb_ar0820_dgain);

				ar0820_fine_gain.pdata = rggb_ar0820_fine_gain;
				ar0820_fine_gain.size = sizeof(rggb_ar0820_fine_gain);
			}
			break;
		default:
			vin_err("don't support config_index %d\n", sensor_info->config_index);
			return -1;
	}

	// set lut table
	turning_data.pwl.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (turning_data.pwl.again_lut != NULL) {
		memset(turning_data.pwl.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(turning_data.pwl.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
		}

		memcpy(turning_data.pwl.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[256 + open_cnt], 2);
		}
		memcpy(turning_data.pwl.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);/* PRQA S 4513 */
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, ret, ret);
		return -RET_ERROR;
	}

	if (turning_data.pwl.again_lut) {
		free(turning_data.pwl.again_lut);	/* PRQA S 5118 */
		turning_data.pwl.again_lut = NULL;
	}
	return ret;
}

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0, i;
	int32_t tmp = 0;
	int32_t entry_num = sensor_info->entry_num;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if ((sensor_info->extra_mode & EXT_MASK) == 0) {
		/* xj3 / j5 */
		if ((deserial_if != NULL) && ((sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
			(sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233)) {
			entry_num = deserial_if->physical_entry;
			vin_dbg("sepa config use physical_entry %d\n", entry_num);
		}
		req = hb_vin_mipi_pre_request((uint32_t)entry_num, 0, 0);
		if (req == 0) {
			ret = deserial_serial_setting(sensor_info);
			hb_vin_mipi_pre_result((uint32_t)entry_num, 0, (uint32_t)ret);		/* PRQA S 2897 */
			if(ret < 0) {
				vin_err("deserial_serial_setting X3_config fail!\n");
				return ret;
			}
			vin_dbg("deserial_serial_setting X3_config OK!\n");
		}
		/* serial sync config */
		if ((sensor_info->config_index & TRIG_STANDARD) ||
			(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
			setting_size = sizeof(max9295_trigger_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
					setting_size, max9295_trigger_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
			}
		}
	}
	/* max9295 need enable LDO */
	if (((sensor_info->config_index & 0xff) == SENSING) ||
	    ((sensor_info->config_index & 0xff) == SENSING_DUAL) ||
	    ((sensor_info->config_index & 0xff) == SENSING_RAW12_EMB) ||
	    ((sensor_info->config_index & 0xff) == SENSING_RAW16_EMB) ||
	    ((sensor_info->config_index & 0xff) == SENSING_WS0233)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}
	ret = ar0820_init(sensor_info);
	if(ret < 0) {
		vin_err("AR0820_X3_config fail!\n");
		return ret;
	}
	vin_dbg("AR0820_X3_config OK!\n");

	if (sensor_info->extra_mode & EXT_MASK) {
		/* zu3 */
		if (sensor_info->sensor_mode == (uint32_t)PWL_M)
			ret = sensor_pwl_data_init(sensor_info);
		else
			ret = sensor_linear_data_init(sensor_info);
	} else {
		/* xj3 / j5 */
		if (sensor_info->sensor_mode == (uint32_t)DOL2_M)
			ret = sensor_dol2_data_init(sensor_info);
		else
			ret = sensor_pwl_data_init(sensor_info);
	}

	if(ret < 0) {
		vin_err("sensor_%s_data_init %s fail\n", sensor_info->sensor_name,
			(sensor_info->sensor_mode != (uint32_t)PWL_M) ? "linear" : "pwl");
		return ret;
	}
	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	pthread_t t1;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	int32_t trigger_gpio;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (sensor_info->extra_mode & EXT_MASK) {
		/* zu3 */
		int32_t i, rs;
		REG_INFO *r;
		r = ar0820_extra_start_s;
		rs = sizeof(ar0820_extra_start_s)/sizeof(REG_INFO);
		for (i = 0; i < rs; i++) {
			pdata = r[i].setting;
			setting_size = r[i].size;
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0)
				vin_err("write register error\n");
		}
		return ret;
	}

	/* xj3 / j5 */
	if ((/*(deserial_if != NULL) &&*/
		(!strcmp(deserial_if->deserial_name, "max96712") ||
		 !strcmp(deserial_if->deserial_name, "max96722"))) &&
		((sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
		 (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233)) {
		pdata = galaxy_sepa_max96712_csia_reset;
		setting_size = sizeof(galaxy_sepa_max96712_csia_reset)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}
	if (((sensor_info->config_index & 0xff) == (uint32_t)GALAXY ||
		 (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GAX3C ||
		 (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GAX3C ||
		 (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_WITH_GA0233 ||
		 (sensor_info->config_index & 0xff) == (uint32_t)GALAXY_SEPA_GA0233) &&
		 (sensor_info->sensor_addr & 0xFC) == (uint32_t)DEFAULT_GALAXY_ADDR) {
		trigger_gpio = 2;
	} else {
		trigger_gpio = 3;
	}
	if (sensor_info->config_index & TRIG_STANDARD) {
		vin_dbg("standard trigger gpio%d\n", trigger_gpio);
		setting_size = sizeof(ar0820_trigger_standard_setting)/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_standard_setting[i*2],
				ar0820_trigger_standard_setting[i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      (uint8_t)sensor_info->sensor_addr, ar0820_trigger_standard_setting[i*2],
				  ar0820_trigger_standard_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : standard trigger %s fail\n", __LINE__,
				       sensor_info->sensor_name);
				return ret;
			}
		}
		setting_size = sizeof(ar0820_trigger_gpio_setting[trigger_gpio])/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      (uint8_t)sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				  ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : standard trigger %s gpio%d fail\n", __LINE__,
				       sensor_info->sensor_name, trigger_gpio);
				return ret;
			}
		}
	} else if (sensor_info->config_index & TRIG_SHUTTER_SYNC) {
		vin_dbg("shutter sync gpio%d\n", trigger_gpio);
		setting_size = sizeof(ar0820_trigger_shutter_sync_setting)/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_shutter_sync_setting[i*2],
				ar0820_trigger_shutter_sync_setting[i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      (uint8_t)sensor_info->sensor_addr, ar0820_trigger_shutter_sync_setting[i*2],
				  ar0820_trigger_shutter_sync_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : shutter sync trigger %s fail\n",
				        __LINE__, sensor_info->sensor_name);
				return ret;
			}
		}
		setting_size = sizeof(ar0820_trigger_gpio_setting[trigger_gpio])/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			vin_dbg("write trig: w%d@0x%02x 0x%04x=0x%04x\n", sensor_info->bus_num,
				sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			      (uint8_t)sensor_info->sensor_addr, ar0820_trigger_gpio_setting[trigger_gpio][i*2],
				  ar0820_trigger_gpio_setting[trigger_gpio][i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : shutter sync trigger %s gpio%d fail\n",
				       sensor_info->sensor_name, trigger_gpio);
				return ret;
			}
		}
	} else {
		pdata = ar0820_stream_on_setting;
		setting_size = sizeof(ar0820_stream_on_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}
	return ret;
}
int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (sensor_info->extra_mode & EXT_MASK) {
		/* zu3 */
		int32_t i, rs;
		REG_INFO *r;
		r = ar0820_extra_stop_s;
		rs = sizeof(ar0820_extra_stop_s)/sizeof(REG_INFO);
		for (i = 0; i < rs; i++) {
			pdata = r[i].setting;
			setting_size = r[i].size;
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
			if (ret < 0)
				vin_err("write register error\n");
		}
		return ret;
	}

	/* xj3 */
	if ((sensor_info->config_index & TRIG_SHUTTER_SYNC) ||
		(sensor_info->config_index & TRIG_STANDARD)) {
		pdata = ar0820_sync_stream_off_setting;
		setting_size = sizeof(ar0820_sync_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	} else {
		pdata = ar0820_stream_off_setting;
		setting_size = sizeof(ar0820_stream_off_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}
	return ret;
}
int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

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
	uint64_t extclk;
	uint32_t x0, m_y0, x1, m_y1;
	int vt_pix_clk_div, vt_sys_clk_div, pre_pll_clk_div, pll_multiplier;

	if (!sp || !si) {
		vin_err("input sp|si is null!\n");
		return -RET_ERROR;
	}

	uint32_t i2c_num = si->bus_num;
	uint8_t i2c_addr = (uint8_t)si->sensor_addr;
	sp->frame_length = (uint32_t)hb_vin_i2c_read_reg16_data16((uint32_t)i2c_num, i2c_addr,
			AR0820_VTS);
	sp->line_length = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			AR0820_HTS);

	x0 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_X_START);
	m_y0 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_Y_START);
	x1 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_X_END);
	m_y1 = (uint32_t)hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr, AR0820_Y_END);

	sp->width = x1 - x0 + 1;
	sp->height = m_y1 - m_y0 + 1;

	vt_pix_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_PIX_CLK_DIV);
	vt_sys_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			VT_SYS_CLK_DIV) & 0x1F;
	pre_pll_clk_div = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PRE_PLL_CLK_DIV);
	pll_multiplier = hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
			PLL_MULTIPLIER);

	if(si->extra_mode & XO_25MHZ) {
		extclk = 25000000;   // 25M
		sp->lines_per_second = WEISEN_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_WISSEN, sizeof(sp->version));	/* PRQA S 2845 */
	} else {
		extclk = 27000000;   // 27M
		sp->lines_per_second = INCEPTIO_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));	/* PRQA S 2845 */
	}

	switch (si->config_index & 0xff) {
		case (uint32_t)GALAXY:
		case (uint32_t)GALAXY_WITH_GAX3C:
		case (uint32_t)GALAXY_SEPA_GAX3C:
		case (uint32_t)GALAXY_WITH_GA0233:
		case (uint32_t)GALAXY_SEPA_GA0233:
			if (si->extra_mode & PWL_HDR4_24BIT) {
				sp->lines_per_second = GAHDR4_LINES_PER_SECOND;  // 156M / 1104 / 16
			} else {
				sp->lines_per_second = GALAXY_LINES_PER_SECOND;  // 156M / 1480 / 12
			}
			break;
		default:
			break;
	}

	sp->pclk = (uint32_t)((extclk * (uint64_t)pll_multiplier) / (uint64_t)(pre_pll_clk_div *
			vt_sys_clk_div * vt_pix_clk_div));

	sp->exp_num = ((uint32_t)(hb_vin_i2c_read_reg16_data16(i2c_num, i2c_addr,
				REG_EXP_NUM) & 0xC) >> 2) + 1;

	sp->fps = ((float)(2*sp->pclk)) / (sp->line_length *
			sp->exp_num * sp->frame_length);
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
		vin_err("ar0820 param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ar0820, CAM_MODULE_FLAG_A16D16);
sensor_module_t ar0820 = {
	.module = SENSOR_MNAME(ar0820),
#else
sensor_module_t ar0820 = {
	.module = "ar0820",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.get_sns_params = get_sns_info,
};

