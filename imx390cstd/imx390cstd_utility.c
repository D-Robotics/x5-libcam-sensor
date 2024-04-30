/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics.
* All rights reserved.
* ***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
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
#include "inc/imx390cstd_setting.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x48)

#define WEISEN_LINES_PER_SECOND             (11764)
#define INCEPTIO_LINES_PER_SECOND           (8784)
#define GALAXY_LINES_PER_SECOND             (8784)
#define GAHDR4_LINES_PER_SECOND             (8832)
#define STOP_DELAY_TIME_US	(1800)
#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"
#define STR_LEN		128

#define CONFIG_INDEX_ALL ( \
		AE_DISABLE | \
		AWB_DISABLE | \
		TEST_PATTERN | \
		FLIP | \
		MIRROR | \
		TRIG_STANDARD | \
		TRIG_SHUTTER_SYNC | \
		TRIG_EXTERNAL \
		)

emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SENSING_M24F120D4_S0R0T7] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_M24F120D4_S0R0T7, "0.0.1", "NULL", "NULL", &emode_data[SENSING_M24F120D4_S0R0T7]),
	SENSOR_EMEND(),
};

static int32_t sensor_config_index_trig_external_mode(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_TRIG_EXTERNAL] = sensor_config_index_trig_external_mode,
};

static int32_t sensor_config_index_trig_external_mode(sensor_info_t *sensor_info)
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
		vin_err("%s %s %s ser_trig_mfp failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ser_trig_mfp;
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
		vin_err("%s %s %s mfp_cfg failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("%s %s %s max_serial_mfp_config failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}

	return  ret;
}

static int32_t sensor_config_index_trig_none_mode(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
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
		vin_err("%s %s %s ser_trig_mfp failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ser_trig_mfp;
	}
	vin_info("port%d %s T%d run in none trigger mode\n",
		sensor_info->port, sensor_info->sensor_name, ser_trig_mfp);
	/* wait more for power on & init done */
	usleep(100 * 1000);
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_OUT_HIGH, 0U);
	if (ret < 0) {
		vin_err("%s %s %s max_serial_mfp_config failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}

	return  ret;
}

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}
	switch(sensor_info->sensor_mode) {
		case (uint32_t)NORMAL_M:  //  normal
			vin_info("NORMAL_M mode %d\n", sensor_info->sensor_mode);
			break;
		default:
		    vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}
	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);
	if (ret < 0) {
		vin_err("%s %s %s sensor_config_do failed\n", \
			deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}

	return ret;
}
int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	uint32_t gpio;
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
						     sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret = (int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
						1-sensor_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("%s %s %s failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);
			}
		}
	}

	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t entry_num = sensor_info->entry_num;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%s %s %s sensor_poweron failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
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
		vin_err("%s %s %s max_serial_init failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}
	vin_dbg("imx390c serializer init done\n");
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%s %s %s sensor_mode_config_init failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
		return ret;
	}
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02, 0x03);
	if (ret < 0) {
		vin_err("%s %s %s write i2c%d-0x%x 0x02 0x00 failed\n", \
			deserial_if->deserial_name, sensor_info->sensor_name, __func__, sensor_info->bus_num, sensor_info->serial_addr);
		return ret;
	}
	usleep(400 * 1000);
	/* support run in none trigger mode */
	if (!SENSOR_CONFIG_ISEN(sensor_info, B_TRIG_EXTERNAL)) {
		ret = sensor_config_index_trig_none_mode(sensor_info);
		if (ret < 0) {
			vin_err("sensor trig none mode fail!\n");
			return ret;
		}
	}

	return ret;
}
int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	uint32_t gpio;
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if (sensor_info->sen_devfd != 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl((uint32_t)sensor_info->gpio_pin[gpio],
									sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("%s %s %s failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
					return -1;
				}
			}
		}
	}

	return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02, 0x43);
		if (ret < 0) {
		vin_err("%s %s %s write i2c%d-0x%x 0x02 0x43 failed\n", \
			deserial_if->deserial_name, sensor_info->sensor_name, __func__, sensor_info->bus_num, sensor_info->serial_addr);
		return ret;
	}

	return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02, 0x03);
	if (ret < 0) {
		vin_err("%s %s %s write i2c%d-0x%x 0x02 0x00 failed\n", \
			deserial_if->deserial_name, sensor_info->sensor_name, __func__, sensor_info->bus_num, sensor_info->serial_addr);
		return ret;
	}

    return ret;
}

int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	ret = sensor_deinit(sensor_info);
	if (ret < 0) {
		vin_err("%s %s %s failed\n", deserial_if->deserial_name, sensor_info->sensor_name, __func__);
	}

	return ret;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(imx390cstd, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx390cstd = {
	.module = SENSOR_MNAME(imx390cstd),
#else
sensor_module_t imx390cstd = {
	.module = "imx390cstd",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
