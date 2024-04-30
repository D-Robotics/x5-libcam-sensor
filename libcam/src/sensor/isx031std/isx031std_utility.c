/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[isx031std]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "inc/isx031std_setting.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "inc/sensor_effect_common.h"
#include "./hb_i2c.h"

#define SENSOR_REG_WIDTH    REG16_VAL16
#define SERDES_REG_WIDTH    REG16_VAL8
#define POC_REG_WIDTH       REG8_VAL8

#ifndef I2C_SMBUS_BLOCK_MAX
#define I2C_SMBUS_BLOCK_MAX 32
#endif

#define EFL_X_ADDR_031     (0x00)
#define EFL_Y_ADDR_031     (0x04)
#define COD_X_ADDR_031     (0x08)
#define COD_Y_ADDR_031     (0x12)
#define K1_ADDR_031        (0x1C)
#define K2_ADDR_031        (0x20)
#define P1_ADDR_031        (0xD0)
#define P2_ADDR_031        (0xD8)
#define K3_ADDR_031        (0x2C)
#define K4_ADDR_031        (0x30)
#define K5_ADDR_031        (0xF0)
#define K6_ADDR_031        (0xF8)

#define INTRIN_DATA_SIZE    58
#define STR_LEN		128

#define CONFIG_INDEX_ALL ( \
		TEST_PATTERN | \
		TRIG_SHUTTER_SYNC | \
		TRIG_EXTERNAL \
		)

static emode_data_t emode_data[MODE_TYPE_MAX] = {
	[HK_M24F217D4_S2R0T8E5] = {
		.serial_addr = 0x42,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M24F190D4_S0R0T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
    [SENSING_M24F190D4_S2R0T8E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
    [WHETRON_M24F190D4_S2R0T8] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M24F190D4_S2R0T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(HK_M24F217D4_S2R0T8E5, "0.0.1", "NULL", "NULL", &emode_data[HK_M24F217D4_S2R0T8E5]),
	SENSOR_EMADD(SENSING_M24F190D4_S0R0T7E5, "0.0.1", "NULL", "NULL", &emode_data[SENSING_M24F190D4_S0R0T7E5]),
    SENSOR_EMADD(SENSING_M24F190D4_S2R0T8E5, "0.0.1", "NULL", "NULL", &emode_data[SENSING_M24F190D4_S2R0T8E5]),
    SENSOR_EMADD(WHETRON_M24F190D4_S2R0T8, "0.0.1", "NULL", "NULL", &emode_data[WHETRON_M24F190D4_S2R0T8]),
	SENSOR_EMADD(SENSING_M24F190D4_S2R0T7E5, "0.0.1", "NULL", "NULL", &emode_data[SENSING_M24F190D4_S2R0T7E5]),
	SENSOR_EMEND(),
};

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_external_mode(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_TEST_PATTERN] = sensor_config_index_test_pattern,
	[B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
	[B_TRIG_EXTERNAL] = sensor_config_index_trig_external_mode,
};

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info)
{
	int32_t i, ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	deserial_module_t *deserial_ops_s = NULL;

	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}
	deserial_ops_s = (deserial_module_t *)(deserial_if->deserial_ops);
	if (deserial_ops_s == NULL) {
	    vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(deserial_ops_s->ops.max);
#endif
	if (maxops == NULL) {
		vin_err("maxops is NULL\n");
		return -1;
	}
	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_id = 1;
	else
		gpio_id = sensor_info->deserial_port;

	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ser_trig_mfp;
	}
	ret = maxops->mfp_cfg(deserial_if, GPIO_TX_GMSL, gpio_id, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	vin_info("set isx031 test pattern\n");
	setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	        SERDES_REG_WIDTH, setting_size, isx031_stream_off_setting);
	if (ret < 0) {
		vin_err("%s set stream off failed\n", sensor_info->sensor_name);
		return ret;
    }
	usleep(40 * 1000);
	setting_size = sizeof(isx031_pattern_mode_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size, isx031_pattern_mode_setting);
	if (ret < 0) {
		vin_err("senor %s write isx031_pattern_mode_setting\n",
				sensor_info->sensor_name);
		return ret;
	}
	setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	        SERDES_REG_WIDTH, setting_size, isx031_stream_on_setting);
	if (ret < 0) {
		vin_err("%s set stream on failed\n", sensor_info->sensor_name);
		return ret;
    }
	usleep(100 * 1000);
    return ret;
}

static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info)
{
	int32_t i, ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	int32_t trigger_gpio;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	deserial_module_t *deserial_ops_s = NULL;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
	deserial_ops_s = (deserial_module_t *)(deserial_if->deserial_ops);
	if (deserial_ops_s == NULL) {
	    vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(deserial_ops_s->ops.max);
#endif
	if (maxops == NULL) {
		vin_err("maxops is NULL\n");
		return -1;
	}
	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_id = 1;
	else
		gpio_id = sensor_info->deserial_port;

	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ser_trig_mfp;
	}
	ret = maxops->mfp_cfg(deserial_if, GPIO_TX_GMSL, gpio_id, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	if (sensor_info->extra_mode == SENSING_M24F190D4_S0R0T7E5 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T8E5 ||
		sensor_info->extra_mode == WHETRON_M24F190D4_S2R0T8 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T7E5) {
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size, isx031_stream_off_setting);
		if (ret < 0) {
			vin_err("%s set trig stream off failed\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(10 * 1000);
		setting_size = sizeof(isx031_trigger_shutter_mode_setting) /sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size, isx031_trigger_shutter_mode_setting);
		if (ret < 0) {
			vin_err("senor %s write trigger shutter mode setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_stream_on_setting);
		if (ret < 0) {
			vin_err("%s set stream on failed\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(100 * 1000);
	}
    return ret;
}

static int32_t sensor_config_index_trig_external_mode(sensor_info_t *sensor_info)
{
	int32_t i, ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	int32_t trigger_gpio;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	deserial_module_t *deserial_ops_s = NULL;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
	deserial_ops_s = (deserial_module_t *)(deserial_if->deserial_ops);
	if (deserial_ops_s == NULL) {
	    vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(deserial_ops_s->ops.max);
#endif
	if (maxops == NULL) {
		vin_err("maxops is NULL\n");
		return -1;
	}
	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_id = 1;
	else
		gpio_id = sensor_info->deserial_port;

	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ser_trig_mfp;
	}
	ret = maxops->mfp_cfg(deserial_if, GPIO_TX_GMSL, gpio_id, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	if (sensor_info->extra_mode == SENSING_M24F190D4_S0R0T7E5 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T8E5 ||
		sensor_info->extra_mode == WHETRON_M24F190D4_S2R0T8 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T7E5) {
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size, isx031_stream_off_setting);
		if (ret < 0) {
			vin_err("%s set trig stream off failed\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(10 * 1000);
		setting_size = sizeof(isx031_trigger_external_mode_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			SERDES_REG_WIDTH, setting_size, isx031_trigger_external_mode_setting);
		if (ret < 0) {
			vin_err("senor %s write trigger shutter mode setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_stream_on_setting);
		if (ret < 0) {
			vin_err("%s set stream on failed\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(100 * 1000);
	}
	return ret;
}

static int32_t sensor_config_index_trig_none_mode(sensor_info_t *sensor_info)
{
	int32_t i, ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	maxdes_ops_t *maxops;
	int32_t trigger_gpio;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	deserial_module_t *deserial_ops_s = NULL;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
	deserial_ops_s = (deserial_module_t *)(deserial_if->deserial_ops);
	if (deserial_ops_s == NULL) {
	    vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(deserial_ops_s->ops.max);
#endif
	if (maxops == NULL) {
		vin_err("maxops is NULL\n");
		return -1;
	}

	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
		return ser_trig_mfp;
	}
	vin_info("port%d %s T%d run in none trigger mode\n",
		sensor_info->port, sensor_info->sensor_name, ser_trig_mfp);
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_OUT_HIGH, 0U);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	if (sensor_info->extra_mode == SENSING_M24F190D4_S0R0T7E5 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T8E5 ||
		sensor_info->extra_mode == WHETRON_M24F190D4_S2R0T8 ||
		sensor_info->extra_mode == SENSING_M24F190D4_S2R0T7E5) {
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SERDES_REG_WIDTH, setting_size, isx031_stream_on_setting);
		if (ret < 0) {
			vin_err("%s set stream on failed\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(100 * 1000);
	}
    return ret;
}

typedef union _eepromtrans
{
	float dEeepromDouble;
	unsigned char lEeepromLong[4];
} eepromtrans_u;

static int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}

	// sensor fps set
	if ((sensor_info->fps != 30) &&
	((sensor_info->extra_mode == SENSING_M24F190D4_S0R0T7E5) ||
	(sensor_info->extra_mode == SENSING_M24F190D4_S2R0T8E5) ||
	(sensor_info->extra_mode == WHETRON_M24F190D4_S2R0T8) ||
	(sensor_info->extra_mode == SENSING_M24F190D4_S2R0T7E5))) {
		usleep(100 * 1000);
		uint16_t vmax_offset = (ISX031_DEF_VMAX * 30 / sensor_info->fps) - ISX031_DEF_VMAX;
		vmax_offset = ((vmax_offset >> 8) & 0xff) | ((vmax_offset << 8) & 0xff00);  // LSB
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, \
					sensor_info->sensor_addr, ISX031_VMAX_OFFSET, vmax_offset);
		if (ret < 0) {
			vin_err("sensor set vmax_offset err!\r\n");
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, \
					sensor_info->sensor_addr, ISX031_VMAX, ISX031_DEF_VMAX);
		if (ret < 0) {
			vin_err("sensor set vmax err!\r\n");
			return ret;
		}
	}
	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);
	if (ret < 0) {
		vin_err("sensor config_index do fail!!!\n");
		return ret;
	}
	if (!SENSOR_CONFIG_ISEN(sensor_info, (TRIG_SHUTTER_SYNC | TRIG_EXTERNAL))) {
		ret = sensor_config_index_trig_none_mode(sensor_info);
		if (ret < 0) {
			vin_err("sensor trig none mode fail!\n");
			return ret;
		}
	}

	return ret;
}

static int32_t sensor_isx031_serdes_stream_on(sensor_info_t *sensor_info)
{
	int8_t ret = RET_OK;
	uint32_t *pdata = NULL, bus;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t setting_size = 0, sensor_addr, deserial_addr;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	sensor_addr = sensor_info->sensor_addr;
	deserial_addr = deserial_if->deserial_addr;
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
	    pdata = max96712_stream_on_setting;
	    setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 2;
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		pdata = max9296_stream_on_setting;
		setting_size = sizeof(max9296_stream_on_setting) / sizeof(uint32_t) / 2;
	}
	ret = vin_write_array(bus, deserial_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("deserial %s stream on fail ret = %d\n", deserial_if->deserial_name, ret);
		return ret;
	}
    return ret;
}

static int32_t sensor_stream_off(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size, i;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		setting_size = sizeof(max96712_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max96712_stream_off_setting[2 * i],
				(uint8_t)(max96712_stream_off_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream off failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size = sizeof(max9296_stream_off_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max9296_stream_off_setting[2 * i],
				(uint8_t)(max9296_stream_off_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream off failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else {
		setting_size = sizeof(isx031_stream_off_setting) / sizeof(uint32_t) / 2;
		vin_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_off_setting[i * 2],
				isx031_stream_off_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s stream off failed\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	return ret;
}

static int32_t sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[24] = {0};
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

    // serial init setting, i2cmap
	ret = max_serial_init(sensor_info);
	if (ret < 0) {
		vin_err("max serial init fail!\n");
		return ret;
	}
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
#ifdef CAM_DIAG
	/* sensor errb serial -> desrial mfp map */
	ret = max_serial_errb_mfp_map(sensor_info);
	if (ret < 0) {
		if (ret == -FLAG_NOT_FIND) {
			vin_warn("port:%d sensor errb not set mfp\n", sensor_info->port);
			ret = 0;
		} else {
			vin_err("port:%d sensor errb map fail\n", sensor_info->port);
			return ret;
		}
	}
	int32_t setting_size = sizeof(fault_notification_mode_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			REG16_VAL8, setting_size, fault_notification_mode_setting);
	if (ret < 0) {
		vin_err("sen_errb %d %s fault_notification_mode_setting write error\n",
			sensor_info->port, sensor_info->sensor_name);
		return ret;
	}
#endif
	return ret;
}

static int32_t sensor_start(sensor_info_t *sensor_info)
{
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	int32_t ret = RET_OK, req;
	int32_t entry_num = sensor_info->entry_num;

	if (deserial_if) {
	  req = hb_vin_mipi_pre_request(entry_num, 1, 0);
	  if (req == 0) {
		  ret = sensor_isx031_serdes_stream_on(sensor_info);
		  if (ret < 0) {
			  ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
			  vin_err("%d : %s sensor_isx031_serdes_stream_on fail\n",
					 __LINE__, sensor_info->sensor_name);
		  }
		  hb_vin_mipi_pre_result(entry_num, 1, ret);
	  }
    }

	return ret;
}

static int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	uint32_t bus = sensor_info->bus_num;

	pdata = isx031_stream_off_setting;
	setting_size = sizeof(isx031_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("write %s register stream off error\n", sensor_info->sensor_name);
	}

	return ret;
}

static int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	return ret;
}

static int32_t sensor_stream_on(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size, i;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 2;
		for (int i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max96712_stream_on_setting[2 * i],
				(uint8_t)(max96712_stream_on_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s strema on failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else if (!strcmp(deserial_if->deserial_name, "max9296")) {
		setting_size = sizeof(max9296_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, max9296_stream_on_setting[2 * i],
				(uint8_t)(max9296_stream_on_setting[2 * i + 1] & 0xFF));
			if (ret < 0) {
				vin_err("%s stream on failed\n", deserial_if->deserial_name);
				return ret;
			}
		}
	} else {
		setting_size = sizeof(isx031_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, isx031_stream_on_setting[i * 2],
				isx031_stream_on_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s : stream on failed\n", sensor_info->sensor_name);
			}
		}
	}
	return ret;
}

static float float_trans_from_char(uint8_t *raw_data, uint8_t base)
{
	eepromtrans_u temp;

	memset(temp.lEeepromLong, 0, sizeof(float));
	memcpy(temp.lEeepromLong, raw_data + base, sizeof(float));

	return temp.dEeepromDouble;
}

static int float_trans_to_char(uint8_t *trans_data, float raw_data, uint8_t base)
{
	eepromtrans_u temp;

	temp.dEeepromDouble = raw_data;
	memcpy(trans_data + base, temp.lEeepromLong, sizeof(float));

	return 0;
}

static int flash_size_set(sensor_info_t *sensor_info)
{
	int ret = 0;
	uint8_t reg_addr[4] = {};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t buf_size = 0;

	// Register Access request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%s : Register Access request failed\n",
			sensor_info->sensor_name);
	}

	// FLASH_SIZE
	reg_addr[0] = 0x8a;
	reg_addr[1] = 0x58;
	reg_size = 2;

	/*
	4 Mbits: 0x080000
	8 Mbits: 0x100000
	16 Mbits: 0x200000
	32 Mbits: 0x400000
	*/
	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x10;
	buf_size = 3;

	// Serial NOR Flash size set (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set to 0x%x failed\n",
			sensor_info->sensor_name, buf[2] << 16 | buf[1] << 8 | buf[0]);
	}
	return ret;
}

static int flash_read(sensor_info_t *sensor_info, uint8_t addr,
	uint8_t* data, uint32_t bytes)
{
	int ret = 0;
	uint8_t reg_addr[4] = {0};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t buf_size = 0;
	uint8_t offset = 0;

	ret = flash_size_set(sensor_info);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash access unlock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF4);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access unlock request failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash all read request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF7);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access request failed\n",
			sensor_info->sensor_name);
	}

	// Serial NOR Flash Read Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x01;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash read request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Read request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1500 * 1000);

	// Buffer read request (Host -> Sensor)
	while (bytes > 0) {
		memset(buf, 0, I2C_SMBUS_BLOCK_MAX);
		reg_size = 2;
		memset(reg_addr, 0, reg_size);
		reg_addr[0] = 0x00;
		reg_addr[1] = addr + offset;

		if (bytes > I2C_SMBUS_BLOCK_MAX - reg_size) {
			buf_size = I2C_SMBUS_BLOCK_MAX - reg_size;
			ret = hb_i2c_read(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : read reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, reg_addr[1], buf_size);
			}
		} else {
			buf_size = bytes;
			ret = hb_i2c_read(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : read reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, reg_addr[1], buf_size);
			}
		}
		memcpy(data + offset, buf, buf_size);
		bytes -= buf_size;
		offset += buf_size;
	}

	// Serial NOR Flash access lock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF5);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access lock request failed\n",
			sensor_info->sensor_name);
	}
	return ret;
}

static int32_t flash_write(sensor_info_t *sensor_info, uint32_t addr,
	uint8_t *data, uint32_t bytes)
{
	int32_t ret = 0;
	uint8_t reg_addr[4] = {};
	uint8_t reg_size = 0;
	uint8_t buf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t buf_size = 0;
	uint8_t offset = 0;

	ret = flash_size_set(sensor_info);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash size set failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash access unlock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF4);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access unlock request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash all read request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF7);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash Erase Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x03;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash sector erase request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash erase request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000 * 1000);

	// Buffer read request (Host -> Sensor)
	while (bytes > 0) {
		memset(buf, 0, I2C_SMBUS_BLOCK_MAX);
		reg_size = 2;
		memset(reg_addr, 0, reg_size);
		reg_addr[0] = 0x00;
		reg_addr[1] = addr + offset;

		if (bytes > I2C_SMBUS_BLOCK_MAX - reg_size) {
			buf_size = I2C_SMBUS_BLOCK_MAX - reg_size;
			memcpy(buf, data + offset, buf_size);
			ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : write reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, addr, buf_size);
			}
		} else {
			buf_size = bytes;
			memcpy(buf, data + offset, buf_size);
			ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
				reg_addr, reg_size, buf, buf_size);
			if (ret < 0) {
				vin_err("%s : write reg_addr 0x%x for %d bytes failed\n",
					sensor_info->sensor_name, addr, buf_size);
			}
		}
		offset += buf_size;
		bytes -= buf_size;
	}
	usleep(1000);

	// Serial NOR Flash Write Subcommand
	reg_addr[0] = 0x80;
	reg_addr[1] = 0x00;
	reg_addr[2] = 0x02;
	reg_size = 3;

	// Address settings of the Serial NOR Flash Device
	buf[0] = 0x00;
	buf[1] = 0x08;
	buf[2] = 0x00;
	buf[3] = 0x00;
	// Execution of a subcommand
	buf[4] = 0x5a;
	buf_size = 5;

	// Serial NOR Flash sector write request (Host -> Sensor)
	ret = hb_i2c_write(sensor_info->bus_num, sensor_info->sensor_addr,
		reg_addr, reg_size, buf, buf_size);
	if (ret < 0) {
		vin_err("%s : sSerial NOR FLash write request failed\n",
			sensor_info->sensor_name);
	}
	usleep(100000);

	// Serial NOR Flash all write (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xFF);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash All Write failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);

	// Serial NOR Flash access lock request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, 0xFFFF, 0xF5);
	if (ret < 0) {
		vin_err("%s : Serial NOR FLash Access lock request failed\n",
			sensor_info->sensor_name);
	}
	usleep(1000);
	return ret;
}

static int get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	return 0;
}

#if 0
// for debug
void data_trans(uint8_t *raw_data, uint8_t size)
{
	uint64_t data;

	float intrinsic_val[9][8] = {
		{433.965759, 433.792175, 961.870239, 718.525574, 0.184173, -0.040078, 0.012576, -0.002737},
		{435.231445, 434.846069, 954.734192, 718.173096, 0.179404, -0.031997, 0.008313, -0.001971},
		{441.790314, 441.508453, 959.902954, 718.690857, 0.224429, -0.064115, 0.010673, -0.001307},
		{436.090973, 435.730499, 970.060242, 718.738525, 0.182133, -0.038199, 0.011377, -0.002514},
		{434.681976, 434.515808, 971.333008, 713.064148, 0.181375, -0.036394, 0.010672, -0.002420},
		{440.628845, 440.290710, 960.465759, 723.735596, 0.22079, -0.058774, 0.006814, -0.000351},
		{442.504578, 442.167877, 950.125061, 725.225403, 0.225708, -0.065291, 0.011578, -0.001479},
		{439.687469, 439.457977, 952.696716, 719.186035, 0.222076, -0.060151, 0.008003, -0.000678},
		{442.774628, 442.333954, 962.581482, 723.569214, 0.223152, -0.063500, 0.010194, -0.001171},
	};
	int base_addr[8] = {EFL_X_ADDR_031, EFL_Y_ADDR_031, COD_X_ADDR_031, COD_Y_ADDR_031,
		K1_ADDR_031, K2_ADDR_031, K3_ADDR_031, K4_ADDR_031};

	for (int i = 0; i < 1; ++i) {
		memset(raw_data, 0, size);
		for (int j = 0; j < 8; ++j) {
			float_trans_to_char(raw_data,
				intrinsic_val[i][j], base_addr[j]);
			vin_info("raw_data[%d]=0x%x, 0x%x, 0x%x, 0x%x\n",
				base_addr[j], raw_data[base_addr[j]], raw_data[base_addr[j]+1],
				raw_data[base_addr[j]+2], raw_data[base_addr[j]+3]);
		}
		// for (int k = 0; k < sizeof(raw_data); ++k) {
		// 	vin_info("0x%x\t", raw_data[k]);
		// }
		vin_info("\nend of the %dth group data\n\n", i);
	}
}
#endif

static int get_intrinsic_params(sensor_info_t *si,
		sensor_intrinsic_parameter_t *sip)
{
	uint64_t data;
	int ret = RET_OK;
	uint8_t intrin_raw_data[INTRIN_DATA_SIZE] = {0};

	if (!sip || !si) {
		vin_err("input sip|si is null!\n");
		return -RET_ERROR;
	}

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));

#if 0
	// for debug
	memset(intrin_raw_data, 0, INTRIN_DATA_SIZE);
	data_trans(intrin_raw_data, INTRIN_DATA_SIZE);
	ret = flash_write(si, EFL_X_ADDR_031, intrin_raw_data, INTRIN_DATA_SIZE);
	if (ret < 0) {
		vin_err("%s : flash_write failed\n",	si->sensor_name);
	}
#endif

	memset(intrin_raw_data, 0, sizeof(intrin_raw_data));

	ret = flash_read(si, EFL_X_ADDR_031, intrin_raw_data,
		sizeof(intrin_raw_data));
	if (ret < 0) {
		vin_err("%s : flash_read failed\n", si->sensor_name);
	}

	sip->focal_u = float_trans_from_char(intrin_raw_data, EFL_X_ADDR_031);
	sip->focal_v = float_trans_from_char(intrin_raw_data, EFL_Y_ADDR_031);
	sip->center_u = float_trans_from_char(intrin_raw_data, COD_X_ADDR_031);
	sip->center_v = float_trans_from_char(intrin_raw_data, COD_Y_ADDR_031);
	sip->k1 = float_trans_from_char(intrin_raw_data, K1_ADDR_031);
	sip->k2 = float_trans_from_char(intrin_raw_data, K2_ADDR_031);
	sip->k3 = float_trans_from_char(intrin_raw_data, K3_ADDR_031);
	sip->k4 = float_trans_from_char(intrin_raw_data, K4_ADDR_031);

	vin_info("focal_u:%0.12f focal_v:%0.12f center_u:%0.12f center_v:%0.12f\n",
			sip->focal_u, sip->focal_v, sip->center_u, sip->center_v);
	vin_info("k1:%0.12f k2:%0.12f k3:%0.12f K4:%0.12f\n",
			sip->k1, sip->k2, sip->k3, sip->k4);

// Register Access request (Host -> Sensor)
	ret = hb_vin_i2c_write_reg16_data8(si->bus_num,
		si->sensor_addr, 0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%s : Register Access request failed\n",
			si->sensor_name);
	}

	return RET_OK;
}

static int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
{
	int ret = RET_OK;

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
		vin_err("%s: get_sns_info param error: type %d !!\n",
			si->sensor_name, type);
		ret = -RET_ERROR;
	}
	return ret;
}

#ifdef CAM_DIAG
static int32_t camera_sony_diag_temperature(diag_node_info_t *node)
{
	int32_t ret = 0;
	int32_t fault = 0;
	uint32_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t dev_addr = node->diag_info.t.reg.reg_dev;
	uint16_t reg_addr = node->diag_info.t.reg.reg_addr;
	uint32_t port = DIAG_SNR_PORT(node->diag_id);
	char value[4] = {0};
	float avg_temp = 0.0;

	ret = hb_vin_i2c_read_block_reg16(bus, dev_addr, reg_addr, value, 4);
	if (ret < 0) {
		vin_err("sony senor port:%d read temperature reg error\n", port);
		return ret;
	}

	avg_temp = (((value[1] << 8 | value[0]) + (value[3] << 8 | value[2])) / 16.0 - 100) / 2.0;
	vin_dbg("Temp monitor port[%d] temp = %f \n", port, avg_temp);
	if (avg_temp > node->diag_info.t.reg.reg_max ||
		avg_temp < node->diag_info.t.reg.reg_min) {
		fault = 1;
		vin_warn("sony sensor port: 0x%x temp: %f out of range(%d,%d)\n",
			port, avg_temp, node->diag_info.t.reg.reg_min, node->diag_info.t.reg.reg_max);
	} else {
		fault = 0;
	}

	return fault;
}

static int32_t sony_sensor_diag_fault_inject(diag_node_info_t *node, int32_t inject)
{
	int32_t ret = 0;
	uint8_t val = 0;
	uint16_t reg_addr = 0;
	uint32_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t dev_addr = node->diag_info.t.reg.reg_dev;
	uint32_t port = DIAG_SNR_PORT(node->diag_id);
	uint16_t diag_type = DIAG_SUB_ID(node->diag_id);

	vin_info("bus:%d dev_addr:0x%02x port:%d diag_type:0x%02x fault inject:%d\n",
			bus, dev_addr, port, diag_type, inject);

	switch (diag_type) {
	case CAMERA_SONY_ERRB_ERROR:
	case CAMERA_SONY_ROW_COLUMN_ID_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG3 : SONY_FAULT_CLEAR_REG3;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(2);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("row column id fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_PLL_CLOCK_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG0 : SONY_FAULT_CLEAR_REG0;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(0);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("pll clock fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_RAM_CRC_1BIT_DATA_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG0 : SONY_FAULT_CLEAR_REG0;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(7);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("ram crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_RAM_CRC_2BIT_DATA_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG1 : SONY_FAULT_CLEAR_REG1;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(0);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("ram crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_RAM_CRC_1BIT_ADDRE_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG1 : SONY_FAULT_CLEAR_REG1;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(1);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("ram crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_ROM_CRC_1BIT_DATA_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG2 : SONY_FAULT_CLEAR_REG2;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(1);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("rom crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_ROM_CRC_2BIT_DATA_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG2 : SONY_FAULT_CLEAR_REG2;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(2);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("rom crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_ROM_CRC_1BIT_ADDRE_ERROR:
		reg_addr = inject ? SONY_FAULT_INJECT_REG2 : SONY_FAULT_CLEAR_REG2;
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, reg_addr);
		val = val | BIT(3);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, reg_addr, val);
		vin_info("rom crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, reg_addr, val);
		break;
	case CAMERA_SONY_AVDD_ERROR:
	case CAMERA_SONY_DOVDD_ERROR:
	case CAMERA_SONY_DVDD_ERROR:
	case CAMERA_SONY_TEMP_ERROR:
		node->diag_info.t.reg.reg_max = inject ? 0 : node->diag_info.t.reg.reg_test;
		vin_info("i2c polling fault inject:%d, reg_max:0x%04x\n",
				inject, node->diag_info.t.reg.reg_max);
		break;
	default:
		vin_err("fault inject diag type is not mismatch\n");
		return -1;
	}
	ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, SONY_ECM_UPDATE_REG, 0x01);
	if (ret < 0) {
		vin_err("bus:%d dev_addr:0x%02x fault inject: %d fail\n",
				bus, dev_addr, inject);
		return ret;
	}

	return 0;
}

static int32_t sensor_diag_nodes_init(sensor_info_t *sensor_info)
{
	int32_t ret = 0;
	uint32_t port_id = 0;
	diag_node_info_t *diag_node = NULL;
	diag_node_info_t *sub_node = NULL;
	diag_node = cam_diag_get_nodes(5);
	sub_node = cam_diag_get_nodes(8);

	if (diag_node == NULL || sub_node == NULL) {
		vin_err("cam_diag_get_nodes failed\n");
		return -RET_ERROR;
	}
	vin_info("diag_nodes info port: %d, name: %s, bus: %d, addr: 0x%x, errb: %d, gpio_type: %d\n",
			 sensor_info->dev_port, sensor_info->sensor_name, sensor_info->bus_num,
			 sensor_info->sensor_addr, sensor_info->sensor_errb, CAM_GPIO_TYPE(sensor_info->sensor_errb));

	port_id = sensor_info->dev_port + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ERRB_CHECK, CAMERA_SONY_ERRB_ERROR);
	diag_node->diag_type = CAM_GPIO_TYPE(sensor_info->sensor_errb);
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.gpio.gpio_type = CAM_GPIO_TYPE(sensor_info->sensor_errb);
	diag_node->diag_info.t.gpio.gpio_index = sensor_info->sensor_errb;
	diag_node->diag_info.t.gpio.gpio_active = CAM_DIAG_GPIO_LOW;
	diag_node->diag_info.t.gpio.gpio_count = 2;
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROW_COLUMN_ID, CAMERA_SONY_ROW_COLUMN_ID_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_3_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x04;
	sub_node->diag_info.t.reg.reg_active = 0x04;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_PLL_CLOCK, CAMERA_SONY_PLL_CLOCK_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_0_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x01;
	sub_node->diag_info.t.reg.reg_active = 0x01;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_RAM_CRC, CAMERA_SONY_RAM_CRC_1BIT_DATA_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_0_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x80;
	sub_node->diag_info.t.reg.reg_active = 0x80;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_RAM_CRC, CAMERA_SONY_RAM_CRC_2BIT_DATA_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_1_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x01;
	sub_node->diag_info.t.reg.reg_active = 0x01;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_RAM_CRC, CAMERA_SONY_RAM_CRC_1BIT_ADDRE_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_1_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x02;
	sub_node->diag_info.t.reg.reg_active = 0x02;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROM_CRC, CAMERA_SONY_ROM_CRC_1BIT_DATA_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_2_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x02;
	sub_node->diag_info.t.reg.reg_active = 0x02;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROM_CRC, CAMERA_SONY_ROM_CRC_2BIT_DATA_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_2_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x04;
	sub_node->diag_info.t.reg.reg_active = 0x04;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROM_CRC, CAMERA_SONY_ROM_CRC_1BIT_ADDRE_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = SONY_FAULT_2_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x08;
	sub_node->diag_info.t.reg.reg_active = 0x08;
	sub_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_SONY_AVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE_LITTLE_ENDIAN(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = SONY_AVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = SONY_AVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_max = SONY_AVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_min = SONY_AVDD_MIN_VOL;
	diag_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_SONY_DOVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE_LITTLE_ENDIAN(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = SONY_DOVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = SONY_DOVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_max = SONY_DOVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_min = SONY_DOVDD_MIN_VOL;
	diag_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_SONY_DVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE_LITTLE_ENDIAN(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = SONY_DVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = SONY_DVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_max = SONY_DVDD_MAX_VOL;
	diag_node->diag_info.t.reg.reg_min = SONY_DVDD_MIN_VOL;
	diag_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_TEMP, CAMERA_SONY_TEMP_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = SONY_TEMP_SEN0_REG;
	diag_node->diag_info.t.reg.reg_test = SONY_MAX_TEMP_VALUE;
	diag_node->diag_info.t.reg.reg_max = SONY_MAX_TEMP_VALUE;
	diag_node->diag_info.t.reg.reg_min = SONY_MIN_TEMP_VALUE;
	diag_node->fault_judging = camera_sony_diag_temperature;
	diag_node->test_fault_inject = sony_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	return 0;
}
#endif  // #ifdef CAM_DIAG

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(isx031std, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D8);
sensor_module_t isx031std = {
	.module = SENSOR_MNAME(isx031std),
#else
sensor_module_t isx031std = {
	.module = "isx031std",
    .emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.stream_off = sensor_stream_off,
	.stream_on = sensor_stream_on,
	.get_sns_params = get_sns_info,
#ifdef CAM_DIAG
	.diag_nodes_init = sensor_diag_nodes_init,
#endif
};
