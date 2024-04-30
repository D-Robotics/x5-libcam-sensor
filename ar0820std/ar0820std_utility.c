/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ar0820std]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ar0820std_setting.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
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

#define WEISEN_LINES_PER_SECOND             (11764)
#define INCEPTIO_LINES_PER_SECOND           (8784)
#define GALAXY_LINES_PER_SECOND             (8784)
#define GAHDR4_LINES_PER_SECOND             (8832)
#define STOP_DELAY_TIME_US	(1800)
#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"
#define STR_LEN		128

#define CONFIG_INDEX_ALL ( \
		EMBEDDED_MODE | \
		TEST_PATTERN | \
		FLIP | \
		MIRROR | \
		TRIG_STANDARD | \
		TRIG_SHUTTER_SYNC \
		)

static emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SUNNY_M25F120D12G3_S1R8T2] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M27F120D12G3_S0R0T7] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_M25F120D12G3_S0R8T7E0] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x18,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SUNNY_M25F120D12G3_S1R8T2, "0.0.1", "lib_CA82GB_pwl12_WS_Fov120.so", "0.22.10.20", &emode_data[SUNNY_M25F120D12G3_S1R8T2]),
	SENSOR_EMADD(SENSING_M27F120D12G3_S0R0T7, "0.0.1", "lib_ar0820RGGB_pwl12_Sens_Fov30.so", "0.22.9.13", &emode_data[SENSING_M27F120D12G3_S0R0T7]),
	SENSOR_EMADD(SUNNY_M25F120D12G3_S0R8T7E0, "0.0.1", "lib_CA82GB_pwl12_WS_Fov120.so", "0.22.10.20", &emode_data[SUNNY_M25F120D12G3_S0R8T7E0]),
	SENSOR_EMEND(),
};

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_embed_setting(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_EMBEDDED_MODE] = sensor_config_index_embed_setting,
	[B_TEST_PATTERN] = sensor_config_index_test_pattern,
	[B_FLIP] = sensor_config_index_filp_setting,
	[B_MIRROR] = sensor_config_index_mirror_setting,
	[B_TRIG_STANDARD] = sensor_config_index_trig_mode,
	[B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
};

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	pdata = ar0820_test_pattern;
	setting_size = sizeof(ar0820_test_pattern)/sizeof(uint32_t)/2;
	vin_dbg("test pattern init!\n");
	ret = vin_write_array(deserial_if->bus_num, sensor_info->sensor_addr, REG16_VAL16,
							setting_size, pdata);
	if (ret < 0)
		vin_err("write register error\n");
	return ret;
}

static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t flip;

	flip = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, AR0820_MIRROR_FLIP);
	flip |= BIT(15);
	vin_info("ar0820_mirror_flip 0x%02x\n", flip);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			AR0820_MIRROR_FLIP, flip);
	if (ret < 0)
		vin_err("senor %s write flip setting error\n",
				sensor_info->sensor_name);
	return ret;
}

static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t mirror;

	mirror = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, AR0820_MIRROR_FLIP);
	mirror |= BIT(14);
	vin_info("ar0820_mirror_flip 0x%02x\n", mirror);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			AR0820_MIRROR_FLIP, mirror);
	if (ret < 0)
		vin_err("senor %s write mirror setting error\n",
				sensor_info->sensor_name);
	return ret;
}

static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info)
{
    int32_t ret = RET_OK;
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
	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
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
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}

	trigger_gpio = vin_sensor_emode_parse(sensor_info, 'G');
	if (trigger_gpio < 0) {
		vin_err("sensor trigger gpio parse fail!!!\n");
		ret = trigger_gpio;
		return ret;
	}

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
	return ret;
}

static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info)
{
    int32_t ret = RET_OK;
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
	ser_trig_mfp = vin_sensor_emode_parse(sensor_info, 'T');
	if (ser_trig_mfp < 0) {
		vin_err("sensor_mode_parse trig pin fail\n");
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
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}

	trigger_gpio = vin_sensor_emode_parse(sensor_info, 'G');
	if (trigger_gpio < 0) {
		vin_err("sensor trigger gpio parse fail!!!\n");
		ret = trigger_gpio;
		return ret;
	}

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
    return ret;
}

static int32_t sensor_config_index_embed_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus = sensor_info->bus_num;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	int32_t serial_type = -1;

	serial_type = vin_sensor_emode_parse(sensor_info, 'S');
	if (serial_type < 0) {
		vin_err("serial_type emode parse fail!!!\n");
		ret = serial_type;
		return ret;
	}
	if (serial_type != MAX9295A) {
		vin_err("Only MAX9295A support emb!!!\n");
		return -1;
	}
	vin_info("ar0820 embeddata enable\n");
	pdata = ar0820_enalbe_embedded;
	setting_size = sizeof(ar0820_enalbe_embedded)/sizeof(uint32_t)/2;
	vin_info("emb settint!\n");
	ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
	if (ret < 0)
		vin_err("write register error\n");
	vin_info("ar0820 embeddata enable done\n");
	return ret;
}
static struct timespec diff_timespec(const struct timespec *time1,
		const struct timespec *time0)
{
	struct timespec diff = {
		.tv_sec = time1->tv_sec - time0->tv_sec,
		.tv_nsec = time1->tv_nsec - time0->tv_nsec};

	if (diff.tv_nsec < 0) {
		diff.tv_nsec += 1000000000;
		diff.tv_sec--;
	}

	return diff;
}

#if 0
/*
 * loop_udelay: udelay not sleep
 * @x: delay time, uint is us, must be less than 1000000
 * */
void loop_udelay(const uint64_t x)
{
	struct timespec t1, t2;
	struct timespec diff;
	int64_t nsec = x * 1000;

	clock_gettime(CLOCK_MONOTONIC, &t1);

	do {
		clock_gettime(CLOCK_MONOTONIC, &t2);
		diff = diff_timespec(&t2, &t1);
	} while (diff.tv_nsec < nsec);
}
#endif

static int32_t ar0820_sensor_debug(sensor_info_t *sensor_info)
{
	int32_t scaler_type = 0;
	uint32_t *pdata = NULL;
	int32_t setting_size = 0, ret = RET_OK;
	int32_t bus, sensor_addr;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	bus = deserial_if->bus_num;
	sensor_addr = sensor_info->sensor_addr;
	ret = sensor_param_parse(sensor_info, "sensor_debug/datatype", ISINT, &scaler_type);
	if (ret < 0) {
		vin_dbg("datetype parse fail ret = %d\n", ret);
		scaler_type = 0;
	}

	vin_dbg("sensor debug mode = %d\n", scaler_type);
	if (sensor_info->resolution == 1080) {
		if (scaler_type == SCALER_WEIGHT9331) {
			pdata = ar0820_extra_binning10_setting;
			setting_size = sizeof(ar0820_extra_binning10_setting)/sizeof(uint32_t)/2;
		} else {
			pdata = ar0820_extra_binning_setting;
			setting_size = sizeof(ar0820_extra_binning_setting)/sizeof(uint32_t)/2;
		}
		vin_info("1080p binning settint!\n");
	} else if (scaler_type == SCALER_WEIGHT9331) {
		pdata = ar0820_weight_9331_scaling_setting;
		setting_size = sizeof(ar0820_weight_9331_scaling_setting)/sizeof(uint32_t)/2;
		vin_info("1080p 9:3:3:1 scaling settint\n");
	} else if (scaler_type == SCALER_TRUE_BAYER) {
		pdata = ar0820_true_bayer_scaling_setting;
		setting_size = sizeof(ar0820_true_bayer_scaling_setting)/sizeof(uint32_t)/2;
		vin_info("scaler true bayer scaling settint\n");
	} else if (scaler_type == SCALER_WEIGHT2110) {
		pdata = ar0820_weight_2110_scaling_setting;
		setting_size = sizeof(ar0820_weight_2110_scaling_setting)/sizeof(uint32_t)/2;
		vin_info("scaler weight 2:1:1:0 scaling settint\n");
	} else {
		vin_dbg("no select sensor debug mode\n");
	}
	ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
	if (ret < 0) {
		vin_err("write scaler binning register error\n");
		return ret;
	}

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
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, AR0820_HTS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts(0x%04x) %d to %d\n",
			ratio, AR0820_HTS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, AR0820_HTS, reg_vs);
		if (ret < 0) {
			vin_err("write AR0820_HTS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_vts_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, AR0820_VTS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("vts ratio %.2f setting: vts(0x%04x) %d to %d\n",
			ratio, AR0820_VTS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, AR0820_VTS, reg_vs);
		if (ret < 0) {
			vin_err("write AR0820_VTS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_pll_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		if (ratio < 0.6) {
			ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, VT_PIX_CLK_DIV, reg_vb, 2);
			reg_v = (reg_vb[0] << 8) | reg_vb[1];
			reg_vs = (uint32_t)(reg_v * (0.6 / ratio) + 0.5);
			if ((reg_v == reg_vs) && (reg_vs < 15))
				reg_vs = reg_v + 1;
			else if (reg_vs == 0)
				reg_vs = 1;
			else if (reg_vs > 15)
				reg_vs = 15;
			vin_info("pll ratio %.2f setting: pix_div(0x%04x) %d to %d\n",
					ratio, VT_PIX_CLK_DIV, reg_v, reg_vs);
			ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, VT_PIX_CLK_DIV, reg_vs);
			if (ret < 0) {
				vin_err("write VT_PIX_CLK_DIV register error\n");
				return ret;
			}
			ratio = ratio * reg_vs / reg_v;
		}
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

static int32_t ar0820_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = deserial_if->poc_addr;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr, mclk;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	/* xj3 / j5 */
	if(sensor_info->sensor_mode == (uint32_t)NORMAL_M) {
		pdata = ar0820_linear_30fps_init_setting;
		setting_size = sizeof(ar0820_linear_30fps_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("linear config mode!\n");
	} else if (sensor_info->sensor_mode == (uint32_t)DOL2_M) {
		pdata = ar0820_dol2_15fps_init_setting;
		setting_size = sizeof(ar0820_dol2_15fps_init_setting)/sizeof(uint32_t)/2;
		vin_dbg("hdr dol2 config mode!\n");
	} else if (sensor_info->sensor_mode == (uint32_t)PWL_M) {
		if (sensor_info->config_index & PWL_24BIT) {
			pdata = ar0820_hdr_4exp_30fps_init_setting;
			setting_size = sizeof(ar0820_hdr_4exp_30fps_init_setting)/sizeof(uint32_t)/2;
			vin_dbg("hdr 4exp pwl config mode!\n");
		} else {
			pdata = ar0820_hdr_3exp_30fps_init_setting;
			setting_size = sizeof(ar0820_hdr_3exp_30fps_init_setting)/sizeof(uint32_t)/2;
			vin_dbg("hdr 3exp pwl config mode!\n");
		}
	} else {
		vin_err("config mode is err\n");
		return -RET_ERROR;
	}
    ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}

	// pll setting
	if (vin_sensor_emode_parse(sensor_info, 'M') == 25) {
		if ((deserial_if &&
				(!strcmp(deserial_if->deserial_name, "max96712") ||
				 !strcmp(deserial_if->deserial_name, "max96722") ||
				 ((!strcmp(deserial_if->deserial_name, "max9296") &&
				   ((sensor_info->extra_mode & 0xff) == SUNNY_M25F120D12G3_S1R8T2))))) ||
				((sensor_info->extra_mode & 0xff) == (uint32_t)SUNNY_M25F120D12G3_S0R8T7E0)) {
			pdata = ar0820_pll_multiplier_hvkeep;
			setting_size = sizeof(ar0820_pll_multiplier_hvkeep)/sizeof(uint32_t)/2;
			vin_dbg("25M pll hv keep settint!\n");
		} else {
			pdata = ar0820_pll_multiplier;
			setting_size = sizeof(ar0820_pll_multiplier)/sizeof(uint32_t)/2;
			vin_dbg("25M pll settint!\n");
		}
		ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	if((sensor_info->extra_mode & 0xff) == (uint32_t)SUNNY_M25F120D12G3_S1R8T2 ||
		(sensor_info->extra_mode & 0xff) == (uint32_t)SUNNY_M25F120D12G3_S0R8T7E0) {
		/* filp and mirror disable */
		pdata = ar0820_filp_mirror_disable;
		setting_size = sizeof(ar0820_filp_mirror_disable)/sizeof(uint32_t)/2;
		vin_dbg("disable flip and mirror!\n");
		ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}

		// galaxy awb init setting
		if ((sensor_info->extra_mode & 0xff) == SUNNY_M25F120D12G3_S1R8T2) {
			/* awb init setting */
			if ((sensor_info->config_index & PWL_24BIT) == 0) {
				vin_dbg("galaxy 3exp pwl setting!\n");
				pdata = ar0820_hdr_3exp_galaxy_pwl_setting;
				setting_size = sizeof(ar0820_hdr_3exp_galaxy_pwl_setting)/sizeof(uint32_t)/2;
			}
		} else {
			pdata = ar0820_awb_init_setting;
			setting_size = sizeof(ar0820_awb_init_setting)/sizeof(uint32_t)/2;
			vin_dbg("awb init setting!\n");
		}
		ret = vin_write_array(deserial_if->bus_num, sensor_info->sensor_addr, REG16_VAL16,
								setting_size, pdata);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	ret = ar0820_sensor_debug(sensor_info);
	if (ret < 0) {
		vin_err("%s sensor debug error\n", sensor_info->sensor_name);
		return ret;
	}
		// fps modify & div.
	if (((sensor_info->fps >= 5) && (sensor_info->fps < 30))) {
		char init_d[3];
		uint32_t vts_v, vts_s, target_fps = sensor_info->fps;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_VTS, init_d, 2);
		vts_v = (init_d[0] << 8) | init_d[1];
		vts_s = vts_v * 30 / target_fps;
		vin_info("%dfps settint, vts %d to %d!\n", target_fps, vts_v, vts_s);
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				AR0820_VTS, vts_s);
		if (ret < 0)
			vin_err("write register error\n");
	}

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

static int32_t sensor_poweron(sensor_info_t *sensor_info)
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

static void sensor_common_data_init(sensor_info_t *sensor_info,
		sensor_turning_data_t *tuning_data)
{
	tuning_data->bus_num = sensor_info->bus_num;
	tuning_data->bus_type = sensor_info->bus_type;
	tuning_data->port = sensor_info->port;
	tuning_data->reg_width = sensor_info->reg_width;
	tuning_data->mode = sensor_info->sensor_mode;
	tuning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(tuning_data->sensor_name, sensor_info->sensor_name,
		sizeof(tuning_data->sensor_name) - 1);
	return;
}

static int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *tuning_data)
{
	int32_t ret = RET_OK;
	uint8_t init_d[3];
	uint32_t x0, m_y0, x1, m_y1, width, height;

	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_VTS, init_d, 2);
	tuning_data->sensor_data.VMAX = init_d[0];
	tuning_data->sensor_data.VMAX  = (tuning_data->sensor_data.VMAX  << 8) | init_d[1];
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, (uint8_t)sensor_info->sensor_addr,
				AR0820_HTS, init_d, 2);
	tuning_data->sensor_data.HMAX = init_d[0];
	tuning_data->sensor_data.HMAX = (tuning_data->sensor_data.HMAX << 8) | init_d[1];

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
	tuning_data->sensor_data.active_width = width;
	tuning_data->sensor_data.active_height = height;

	/* xj3 / j5 */
	switch (sensor_info->extra_mode & 0xff) {
		case (uint32_t)SUNNY_M25F120D12G3_S1R8T2:
		case (uint32_t)SUNNY_M25F120D12G3_S0R8T7E0:
			tuning_data->sensor_data.gain_max = 128 * 8192;
			tuning_data->sensor_data.analog_gain_max = 190 * 8192;
			tuning_data->sensor_data.digital_gain_max = 0 * 8192;
			tuning_data->sensor_data.exposure_time_min = 1;
			tuning_data->sensor_data.exposure_time_max = 4000;
			tuning_data->sensor_data.exposure_time_long_max = 4000;
			tuning_data->sensor_data.turning_type = 6;   // gain calc
			tuning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&tuning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			if (sensor_info->config_index & PWL_24BIT) {
				tuning_data->sensor_data.lines_per_second = GAHDR4_LINES_PER_SECOND;  // 156M / 1104 / 16
				sensor_data_bits_fill(&tuning_data->sensor_data, 24);
			} else {
				tuning_data->sensor_data.lines_per_second = GALAXY_LINES_PER_SECOND;  // 156M / 1480 / 12
				sensor_data_bits_fill(&tuning_data->sensor_data, 20);
			}
			break;
		case (uint32_t)SENSING_M27F120D12G3_S0R0T7:
			tuning_data->sensor_data.gain_max = 128 * 8192;
			tuning_data->sensor_data.analog_gain_max = 158 * 8192;
			tuning_data->sensor_data.digital_gain_max = 0 * 8192;
			tuning_data->sensor_data.exposure_time_min = 20;
			tuning_data->sensor_data.exposure_time_max = 4000;
			tuning_data->sensor_data.exposure_time_long_max = 4000;
			tuning_data->sensor_data.lines_per_second = 8784;  // 156M / 4440
			tuning_data->sensor_data.turning_type = 6;   // gain calc
			tuning_data->sensor_data.conversion = 1;
			sensor_data_bayer_fill(&tuning_data->sensor_data, 12, (uint32_t)BAYER_START_GR, (uint32_t)BAYER_PATTERN_RGGB);
			sensor_data_bits_fill(&tuning_data->sensor_data, 20);
			break;
		default:
			vin_err("don't support config_index %d\n", sensor_info->config_index);
			return -1;
	}
	return ret;
}

static int32_t sensor_stream_control_set(sensor_turning_data_t *tuning_data)
{
	int32_t ret = RET_OK, i, cnt;
	uint32_t *stream_src;
	uint32_t *stream_on = tuning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = tuning_data->stream_ctrl.stream_off;

	tuning_data->stream_ctrl.data_length = 2;
	cnt = sizeof(ar0820_stream_on_setting) / 8;
	stream_src = ar0820_stream_on_setting;
	if (sizeof(tuning_data->stream_ctrl.stream_on) >= (uint64_t)(cnt * 8)) {/* PRQA S 2995, 2991 */
		for (i = 0; i < cnt; i++) {/* PRQA S 2877 */
			stream_on[i * 2] = ((uint32_t)stream_src[2 * i]);
			stream_on[i * 2 + 1] = ((uint32_t)stream_src[2 * i + 1]);
		}
	} else {
		vin_err("Number of registers on stream over 10\n");/* PRQA S 2880 */
		return -RET_ERROR;
	}
	cnt = sizeof(ar0820_stream_off_setting) / 8;
	stream_src = ar0820_stream_off_setting;
	if (sizeof(tuning_data->stream_ctrl.stream_off) >= (uint64_t)(cnt * 8)) {/* PRQA S 2995, 2991 */
		for (i = 0; i < cnt; i++) {/* PRQA S 2877 */
			stream_off[i * 2] = (((uint32_t)stream_src[i * 2]));
			stream_off[i * 2 + 1] = (((uint32_t)stream_src[i * 2 + 1]));
		}
	} else {
		vin_err("Number of registers on stream over 10\n");/* PRQA S 2880 */
		return -RET_ERROR;
	}
	return ret;
}

static int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t tuning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;
	uint32_t open_cnt;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&tuning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &tuning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &tuning_data);
	}

	tuning_data.normal.param_hold = AR0820_PARAM_HOLD;
	tuning_data.normal.param_hold_length = 2;
	tuning_data.normal.s_line = AR0820_LINE;
	tuning_data.normal.s_line_length = 2;

	ret = sensor_stream_control_set(&tuning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	tuning_data.normal.line_p.ratio = 1 << 8;
	tuning_data.normal.line_p.offset = 0;
	tuning_data.normal.line_p.max = 4000;
	tuning_data.normal.again_control_num = 3;
	tuning_data.normal.again_control[0] = AR0820_GAIN;
	tuning_data.normal.again_control_length[0] = 2;
	tuning_data.normal.again_control[1] = AR0820_FINE_GAIN;
	tuning_data.normal.again_control_length[1] = 2;
	tuning_data.normal.again_control[2] = AR0820_DGAIN;
	tuning_data.normal.again_control_length[2] = 2;
	tuning_data.normal.dgain_control_num = 0;
	tuning_data.normal.dgain_control[0] = 0;
	tuning_data.normal.dgain_control_length[0] = 0;

	tuning_data.sensor_awb.bgain_addr[0] = 0x3058;
	tuning_data.sensor_awb.bgain_length[0] = 2;
	tuning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	tuning_data.sensor_awb.bgain_length[1] = 2;
	tuning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	tuning_data.sensor_awb.bgain_length[2] = 2;
	tuning_data.sensor_awb.rgain_addr[0] = 0x305a;
	tuning_data.sensor_awb.rgain_length[0] = 2;
	tuning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	tuning_data.sensor_awb.rgain_length[1] = 2;
	tuning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	tuning_data.sensor_awb.rgain_length[2] = 2;
	tuning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->extra_mode & 0xff) {
		case (uint32_t)SENSING_M27F120D12G3_S0R0T7:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case (uint32_t)SUNNY_M25F120D12G3_S1R8T2:
		case (uint32_t)SUNNY_M25F120D12G3_S0R8T7E0:
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
	tuning_data.normal.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (tuning_data.normal.again_lut != NULL) {
		memset(tuning_data.normal.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(tuning_data.normal.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.normal.again_lut[open_cnt], 2);
		}

		memcpy(tuning_data.normal.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.normal.again_lut[256 + open_cnt], 2);
		}
		memcpy(tuning_data.normal.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.normal.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = vin_sensor_tuning_init(sensor_info, &tuning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", sensor_info->port, ret);
		return -RET_ERROR;
	}
	if (tuning_data.normal.again_lut)
		free(tuning_data.normal.again_lut);	/* PRQA S 5118 */

	return ret;
}

static int32_t sensor_dol2_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt;
	char str[24] = {0};
	sensor_turning_data_t tuning_data;

	memset(&tuning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &tuning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &tuning_data);
	}
	tuning_data.dol2.param_hold = AR0820_PARAM_HOLD;
	tuning_data.dol2.param_hold_length = 2;

	tuning_data.dol2.s_line = AR0820_LINE_S;
	tuning_data.dol2.s_line_length = 2;

	tuning_data.dol2.m_line = AR0820_LINE;
	tuning_data.dol2.m_line_length = 2;

#ifdef TUNING_LUT
	tuning_data.dol2.line_p[0].ratio = 1 << 8;
	tuning_data.dol2.line_p[0].offset = 0;
	tuning_data.dol2.line_p[0].max = 19;
	tuning_data.dol2.line_p[1].ratio = 1 << 8;
	tuning_data.dol2.line_p[1].offset = 0;
	tuning_data.dol2.line_p[1].max = 4000;

	tuning_data.dol2.again_control_num = 3;
	tuning_data.dol2.again_control[0] = AR0820_GAIN;
	tuning_data.dol2.again_control_length[0] = 2;
	tuning_data.dol2.again_control[1] = AR0820_FINE_GAIN;
	tuning_data.dol2.again_control_length[1] = 2;
	tuning_data.dol2.again_control[2] = AR0820_DGAIN;
	tuning_data.dol2.again_control_length[2] = 2;
	tuning_data.dol2.dgain_control_num = 0;
	tuning_data.dol2.dgain_control[0] = 0;
	tuning_data.dol2.dgain_control_length[0] = 0;

	tuning_data.sensor_awb.bgain_addr[0] = 0x3058;
	tuning_data.sensor_awb.bgain_length[0] = 2;
	tuning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	tuning_data.sensor_awb.bgain_length[1] = 2;
	tuning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	tuning_data.sensor_awb.bgain_length[2] = 2;
	tuning_data.sensor_awb.rgain_addr[0] = 0x305a;
	tuning_data.sensor_awb.rgain_length[0] = 2;
	tuning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	tuning_data.sensor_awb.rgain_length[1] = 2;
	tuning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	tuning_data.sensor_awb.rgain_length[2] = 2;
	tuning_data.sensor_awb.grgain_addr[0] = 0x3056;
	tuning_data.sensor_awb.grgain_length[0] = 2;
	tuning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	tuning_data.sensor_awb.grgain_length[1] = 2;
	tuning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	tuning_data.sensor_awb.grgain_length[2] = 2;
	tuning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	tuning_data.sensor_awb.gbgain_length[0] = 2;
	tuning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	tuning_data.sensor_awb.gbgain_length[1] = 2;
	tuning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	tuning_data.sensor_awb.gbgain_length[2] = 2;
	tuning_data.sensor_awb.rb_prec = 7;

	tuning_data.dol2.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (tuning_data.dol2.again_lut != NULL) {
		memset(tuning_data.dol2.again_lut, 0xff, 256*2*sizeof(uint32_t));
		memcpy(tuning_data.dol2.again_lut,
			rggb_ar0820_gain, sizeof(rggb_ar0820_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.dol2.again_lut[open_cnt], 2);
		}
		memcpy(tuning_data.dol2.again_lut + 256,
			rggb_ar0820_fine_gain, sizeof(rggb_ar0820_fine_gain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_fine_gain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.dol2.again_lut[256 + open_cnt], 2);
		}
		memcpy(tuning_data.dol2.again_lut + 512,
			rggb_ar0820_dgain, sizeof(rggb_ar0820_dgain));
		for (open_cnt =0; open_cnt <
			sizeof(rggb_ar0820_dgain)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.dol2.again_lut[512 + open_cnt], 2);
		}
	}
#endif
	ret = vin_sensor_tuning_init(sensor_info, &tuning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}
	if (tuning_data.dol2.again_lut)
		free(tuning_data.dol2.again_lut);	/* PRQA S 5118 */
	if (tuning_data.dol2.dgain_lut)
		free(tuning_data.dol2.dgain_lut);	/* PRQA S 5118 */


	return ret;
}

static int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt = 0;
	char str[24] = {0};
	sensor_turning_data_t tuning_data;
	reg_setting_data_t ar0820_gain;
	reg_setting_data_t ar0820_dgain;
	reg_setting_data_t ar0820_dcgain;
	reg_setting_data_t ar0820_fine_gain;

	memset(&tuning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &tuning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &tuning_data);
	}
	tuning_data.pwl.param_hold = AR0820_PARAM_HOLD;
	tuning_data.pwl.param_hold_length = 2;
	tuning_data.pwl.line = AR0820_LINE;
	tuning_data.pwl.line_length = 2;

	ret = sensor_stream_control_set(&tuning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#ifdef TUNING_LUT
	tuning_data.pwl.line_p.ratio = 1 << 8;
	tuning_data.pwl.line_p.offset = 0;
	tuning_data.pwl.line_p.max = 4000;

	tuning_data.pwl.again_control_num = 3;
	tuning_data.pwl.again_control[0] = AR0820_GAIN;
	tuning_data.pwl.again_control_length[0] = 2;
	tuning_data.pwl.again_control[1] = AR0820_FINE_GAIN;
	tuning_data.pwl.again_control_length[1] = 2;
	tuning_data.pwl.again_control[2] = AR0820_DGAIN;
	tuning_data.pwl.again_control_length[2] = 2;
	tuning_data.pwl.dgain_control_num = 0;
	tuning_data.pwl.dgain_control[0] = 0;
	tuning_data.pwl.dgain_control_length[0] = 0;

	tuning_data.sensor_awb.bgain_addr[0] = 0x3058;
	tuning_data.sensor_awb.bgain_length[0] = 2;
	tuning_data.sensor_awb.bgain_addr[1] = 0x35a2;
	tuning_data.sensor_awb.bgain_length[1] = 2;
	tuning_data.sensor_awb.bgain_addr[2] = 0x35aa;
	tuning_data.sensor_awb.bgain_length[2] = 2;
	tuning_data.sensor_awb.rgain_addr[0] = 0x305a;
	tuning_data.sensor_awb.rgain_length[0] = 2;
	tuning_data.sensor_awb.rgain_addr[1] = 0x35a4;
	tuning_data.sensor_awb.rgain_length[1] = 2;
	tuning_data.sensor_awb.rgain_addr[2] = 0x35ac;
	tuning_data.sensor_awb.rgain_length[2] = 2;
	tuning_data.sensor_awb.grgain_addr[0] = 0x3056;
	tuning_data.sensor_awb.grgain_length[0] = 2;
	tuning_data.sensor_awb.grgain_addr[1] = 0x35a0;
	tuning_data.sensor_awb.grgain_length[1] = 2;
	tuning_data.sensor_awb.grgain_addr[2] = 0x35a8;
	tuning_data.sensor_awb.grgain_length[2] = 2;
	tuning_data.sensor_awb.gbgain_addr[0] = 0x305c;
	tuning_data.sensor_awb.gbgain_length[0] = 2;
	tuning_data.sensor_awb.gbgain_addr[1] = 0x35a6;
	tuning_data.sensor_awb.gbgain_length[1] = 2;
	tuning_data.sensor_awb.gbgain_addr[2] = 0x35ae;
	tuning_data.sensor_awb.gbgain_length[2] = 2;
	if (sensor_info->config_index & PWL_24BIT) {
		tuning_data.sensor_awb.bgain_addr[3] = 0x35b2;
		tuning_data.sensor_awb.bgain_length[3] = 2;
		tuning_data.sensor_awb.rgain_addr[3] = 0x35b4;
		tuning_data.sensor_awb.rgain_length[3] = 2;
		tuning_data.sensor_awb.grgain_addr[3] = 0x35b0;
		tuning_data.sensor_awb.grgain_length[3] = 2;
		tuning_data.sensor_awb.gbgain_addr[3] = 0x35b6;
		tuning_data.sensor_awb.gbgain_length[3] = 2;
	}
	tuning_data.sensor_awb.rb_prec = 7;

	// get lut table for different ar0820
	switch (sensor_info->extra_mode & 0xff) {
		case (uint32_t)SENSING_M27F120D12G3_S0R0T7:
			ar0820_gain.pdata = rccb_ar0820_gain;
			ar0820_gain.size = sizeof(rccb_ar0820_gain);

			ar0820_dgain.pdata = rccb_ar0820_dgain;
			ar0820_dgain.size = sizeof(rccb_ar0820_dgain);

			ar0820_fine_gain.pdata = rccb_ar0820_fine_gain;
			ar0820_fine_gain.size = sizeof(rccb_ar0820_fine_gain);
			break;

		case (uint32_t)SUNNY_M25F120D12G3_S1R8T2:
		case (uint32_t)SUNNY_M25F120D12G3_S0R8T7E0:
			if (sensor_info->config_index & PWL_24BIT) {
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
	tuning_data.pwl.again_lut = malloc(256*3*sizeof(uint32_t));	/* PRQA S 5118 */
	if (tuning_data.pwl.again_lut != NULL) {
		memset(tuning_data.pwl.again_lut, 0xff, 256*3*sizeof(uint32_t));

		memcpy(tuning_data.pwl.again_lut, ar0820_gain.pdata,
			ar0820_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.pwl.again_lut[open_cnt], 2);
		}

		memcpy(tuning_data.pwl.again_lut + 256, ar0820_fine_gain.pdata,
			ar0820_fine_gain.size);
		for (open_cnt =0; open_cnt <
			ar0820_fine_gain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.pwl.again_lut[256 + open_cnt], 2);
		}
		memcpy(tuning_data.pwl.again_lut + 512, ar0820_dgain.pdata,
			ar0820_dgain.size);
		for (open_cnt =0; open_cnt <
			ar0820_dgain.size/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&tuning_data.pwl.again_lut[512 + open_cnt], 2);
		}
	}
#endif

	ret = vin_sensor_tuning_init(sensor_info, &tuning_data);
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, ret, ret);
		return -RET_ERROR;
	}

	if (tuning_data.pwl.again_lut) {
		free(tuning_data.pwl.again_lut);	/* PRQA S 5118 */
		tuning_data.pwl.again_lut = NULL;
	}
	return ret;
}

static int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0, i;
	int32_t tmp = 0;
	int32_t entry_num = sensor_info->entry_num;
	deserial_info_t *deserial_if = sensor_info->deserial_info;

	ret = ar0820_init(sensor_info);
	if(ret < 0) {
		vin_err("AR0820_X3_config fail!\n");
		return ret;
	}
	vin_dbg("AR0820_X3_config OK!\n");

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			ret = sensor_linear_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_linear_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case DOL2_M:
			ret = sensor_dol2_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_dol2_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case PWL_M:
			ret = sensor_pwl_data_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_pwl_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
		    vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}

	if(ret < 0) {
		vin_err("sensor_%s_data_init %s fail\n", sensor_info->sensor_name,
			(sensor_info->sensor_mode != (uint32_t)PWL_M) ? "linear" : "pwl");
		return ret;
	}

	return ret;
}

#ifdef CAM_DIAG
static int32_t camera_ar0820_diag_voltage(diag_node_info_t *node)
{
	uint32_t vol = 0;
	int32_t fault = 0;
	uint16_t value_adc, value_slope, value_offset;
	uint8_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t addr = node->diag_info.t.reg.reg_dev;
	uint16_t offset_addr = OFFSET;
	uint16_t adc_addr = (node->diag_info.t.reg.reg_addr >> 16) & 0xFFFF;
	uint16_t slope_addr = node->diag_info.t.reg.reg_addr & 0xFFFF;

	value_adc = hb_vin_i2c_read_reg16_data16(bus, addr, adc_addr);
	value_slope = hb_vin_i2c_read_reg16_data16(bus, addr, slope_addr);
	value_offset = hb_vin_i2c_read_reg16_data16(bus, addr, offset_addr);

	if ((adc_addr == VAA_ADC) || (adc_addr == VAAPIX_ADC)) {
		vol = (((1023 - value_adc - value_offset) * VAA_coefficient) / value_slope);
	} else if ((adc_addr == VDD_ADC) || (adc_addr == VDDPIN_ADC)) {
		vol = (((1023 - value_adc - value_offset) * VDD_coefficient) / value_slope);
	} else {
		vol = (((1023 - value_adc - value_offset) * VDDIO_coefficient) / value_slope);
	}
	vin_dbg("diag_id 0x%x vol=%d max %d min %d\n", node->diag_id, vol,
		node->diag_info.t.reg.reg_max, node->diag_info.t.reg.reg_min);
	if (vol > node->diag_info.t.reg.reg_max) {
		vin_dbg("diag_id 0x%x vol high:%d reg_max %d\n",
			node->diag_id, vol, node->diag_info.t.reg.reg_max);
		fault = 1;
		return fault;
	} else if (vol < node->diag_info.t.reg.reg_min) {
		vin_dbg("diag_id 0x%x vaa low:%d reg_min %d\n", node->diag_id, vol,
			node->diag_info.t.reg.reg_min);
		fault = 1;
		return fault;
	}
	return fault;
}

static int32_t ar0820_syscheck_fault_inject(diag_node_info_t *node, int32_t inject)
{
	int32_t ret = 0;
	sensor_info_t *sensor_info = (sensor_info_t *)node->cb_data;

	vin_info("%s bus %d, addr 0x%x, reg 0x%x\n", __func__,
		sensor_info->bus_num, sensor_info->sensor_addr, CAM_INJECT_ADDR);
	if (inject) {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr, CAM_INJECT_ADDR, 0x6);
	} else {
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, CAM_INJECT_ADDR, 0);
	}
	if(ret != 0) {
		vin_err(" write error CAM_INJECT_ADDR\n");
		return ret;
	}

	return 0;
}

static int32_t test_ar0820_voltage_fault_inject(diag_node_info_t *node, int32_t inject)
{
	int32_t ret = 0;
	sensor_info_t *sensor_info = (sensor_info_t *)node->cb_data;

	if (inject) {
		node->diag_info.t.reg.reg_max = node->diag_info.t.reg.reg_min;
	} else {
		node->diag_info.t.reg.reg_max = node->diag_info.t.reg.reg_test;
	}
	vin_info("%s done\n", __func__);
	return 0;
}

static int32_t diag_voltage_node_init(sensor_info_t *sensor_info)
{
	uint32_t port;
	diag_node_info_t *reg_node = NULL;

	reg_node = cam_diag_get_nodes(5);
	if (reg_node == NULL) {
		vin_err("cam_diag_get_nodes failed\n");
		return -1;
	}

	port = sensor_info->dev_port + 1;
	vin_info("diag_id 0x%x, info port: %d, name: %s, bus: %d, addr: 0x%x\n",
		CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VAA_ERROR),
		sensor_info->dev_port, sensor_info->sensor_name, sensor_info->bus_num,
		sensor_info->sensor_addr);
	// voltage reg VAA adccode
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VAA_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = (VAA_ADC << 16) | VAA_SLOPE;
	reg_node->diag_info.t.reg.reg_max = AR0820_VAA_H;
	reg_node->diag_info.t.reg.reg_min = AR0820_VAA_L;
	reg_node->diag_info.t.reg.reg_test = AR0820_VAA_H;
	reg_node->fault_judging = camera_ar0820_diag_voltage;
	reg_node->test_fault_inject = test_ar0820_voltage_fault_inject;
	cam_diag_node_register(reg_node);

	// voltage reg VAAPIX
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VAAPIX_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = (VAAPIX_ADC << 16) | VAAPIX_SLOPE;
	reg_node->diag_info.t.reg.reg_max = AR0820_VAAPIX_H;
	reg_node->diag_info.t.reg.reg_test = AR0820_VAAPIX_H;
	reg_node->diag_info.t.reg.reg_min = AR0820_VAAPIX_L;
	reg_node->fault_judging = camera_ar0820_diag_voltage;
	reg_node->test_fault_inject = test_ar0820_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VDD
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VDD_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = (VDD_ADC << 16) | VDD_SLOPE;
	reg_node->diag_info.t.reg.reg_max = AR0820_VDD_H;
	reg_node->diag_info.t.reg.reg_test = AR0820_VDD_H;
	reg_node->diag_info.t.reg.reg_min = AR0820_VDD_L;
	reg_node->fault_judging = camera_ar0820_diag_voltage;
	reg_node->test_fault_inject = test_ar0820_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VDDPIN
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VDDPIN_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = (VDDPIN_ADC << 16) | VDDPIN_SLOPE;
	reg_node->diag_info.t.reg.reg_max = AR0820_VDDPIN_H;
	reg_node->diag_info.t.reg.reg_test = AR0820_VDDPIN_H;
	reg_node->diag_info.t.reg.reg_min = AR0820_VDDPIN_L;
	reg_node->fault_judging = camera_ar0820_diag_voltage;
	reg_node->test_fault_inject = test_ar0820_voltage_fault_inject;
	cam_diag_node_register(reg_node);
	// voltage reg VDDIO
	reg_node = reg_node + 1;
	reg_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_VOLTAGE, AR0820_VDDIO_ERROR);
	reg_node->diag_type = CAM_DIAG_REG;
	reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	reg_node->diag_status = 0;
	reg_node->port_mask = vin_port_mask_of_snr(sensor_info);
	reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	reg_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	reg_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	reg_node->diag_info.t.reg.reg_addr = (VDDIO_ADC << 16) | VDDIO_SLOPE;
	reg_node->diag_info.t.reg.reg_max = AR0820_VDDIO_H;
	reg_node->diag_info.t.reg.reg_test = AR0820_VDDIO_H;
	reg_node->diag_info.t.reg.reg_min = AR0820_VDDIO_L;
	reg_node->fault_judging = camera_ar0820_diag_voltage;
	reg_node->test_fault_inject = test_ar0820_voltage_fault_inject;
	cam_diag_node_register(reg_node);

	return 0;
}

static int32_t diag_errb_node_init(sensor_info_t *sensor_info)
{
	uint32_t port;
	diag_node_info_t *gpio_node = NULL;

	port = sensor_info->dev_port + 1;
	gpio_node = cam_diag_get_nodes(1);
	if (gpio_node == NULL) {
		vin_err("cam_diag_get_nodes failed\n");
		return -1;
	}
	vin_info("diag_id 0x%x, info port: %d, name: %s, errb: %d, gpio_type: %d\n",
		CAM_SNR_DIAG_ID(port, SNR_ERRB_CHECK, AR0820_ERRB_ERROR),
		sensor_info->dev_port, sensor_info->sensor_name,
		sensor_info->sensor_errb, CAM_GPIO_TYPE(sensor_info->sensor_errb));

	gpio_node->diag_id = CAM_SNR_DIAG_ID(port, SNR_ERRB_CHECK, AR0820_ERRB_ERROR);
	gpio_node->diag_type = CAM_GPIO_TYPE(sensor_info->sensor_errb);
	gpio_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	gpio_node->diag_status = 0;
	gpio_node->port_mask = vin_port_mask_of_snr(sensor_info);
	gpio_node->diag_info.t.gpio.gpio_type = CAM_GPIO_TYPE(sensor_info->sensor_errb);
	gpio_node->diag_info.t.gpio.gpio_index = sensor_info->sensor_errb;
	gpio_node->diag_info.t.gpio.gpio_count = 3;
	gpio_node->diag_info.t.gpio.gpio_value = 0;
	gpio_node->diag_info.t.gpio.gpio_nc = 0;
	gpio_node->diag_info.t.gpio.gpio_active = CAM_DIAG_GPIO_HIGH;
	gpio_node->diag_info.t.gpio.gpio_value_last = 0;
	gpio_node->test_fault_inject = ar0820_syscheck_fault_inject;
	gpio_node->cb_data = sensor_info;
	cam_diag_node_register(gpio_node);

	return 0;
}

static int32_t sensor_diag_nodes_init(sensor_info_t *sensor_info)
{
	int32_t ret;
	uint32_t port;

	ret = diag_voltage_node_init(sensor_info);
	if (ret != 0) {
		vin_err("voltage_node init error\n");
		return ret;
	}

	ret = diag_errb_node_init(sensor_info);
	if (ret != 0) {
		vin_err("errb_node init error\n");
		return ret;
	}
	vin_info("init done 0820 node\n");
	return 0;
}
#endif  //  #ifdef CAM_DIAG

static int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	pthread_t t1;

	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	// serial init setting,i2cmap,
	ret = max_serial_init(sensor_info);
	if (ret < 0) {
		vin_err("serial init fail!\n");
		return ret;
	}

	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
#ifdef CAM_DIAG
	ret = max_serial_errb_mfp_map(sensor_info);
	if (ret < 0) {
		if (ret == -FLAG_NOT_FIND) {
			vin_warn("port:%d sensor errb not set mfp\n", sensor_info->port);
			ret = 0;
		} else {
			vin_err("port:%d sensor errb map fail ret: %d\n", sensor_info->port, ret);
			return ret;
		}
	}
	vin_dbg("sensor_info bus %d addr %d\n", sensor_info->bus_num, sensor_info->sensor_addr);
	setting_size = sizeof(ar0820_vlotage_setting) /
		sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
		REG16_VAL16, setting_size, ar0820_vlotage_setting);
	if (ret < 0) {
		vin_err("senor %s write embedded data mode 0 setting error\n",
			sensor_info->sensor_name);
		return ret;
	}
#endif
	return ret;
}

static int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;

	if ((sensor_info->config_index == TRIG_SHUTTER_SYNC) ||
		(sensor_info->config_index == TRIG_STANDARD)) {
		return ret;
	}

	pdata = ar0820_stream_on_setting;
	setting_size = sizeof(ar0820_stream_on_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}

	return ret;
}

static int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	int32_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = deserial_if->poc_addr;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	/* xj3 */
	if ((sensor_info->config_index & TRIG_SHUTTER_SYNC) ||
		(sensor_info->config_index & TRIG_STANDARD)) {
		pdata = ar0820_sync_stream_off_setting;
		setting_size = sizeof(ar0820_sync_stream_off_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
		if (ret < 0)
			vin_err("write register error\n");
	} else {
		/* loop_udelay(STOP_DELAY_TIME_US); */
		pdata = ar0820_stream_off_setting;
		setting_size = sizeof(ar0820_stream_off_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(bus, sensor_addr, REG16_VAL16, setting_size, pdata);
		if (ret < 0)
			vin_err("write register error\n");
	}
	return ret;
}

static int32_t sensor_deinit(sensor_info_t *sensor_info)
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
	return ret;
}

static int32_t sensor_poweroff(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	ret = sensor_deinit(sensor_info);
	if (ret < 0) {
		vin_err("%s %s deinit failed\n", sensor_info->sensor_name, __func__);
	}

	return ret;
}

static int get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
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

	if(vin_sensor_emode_parse(si, 'M') == 25) {
		extclk = 25000000;   // 25M
		sp->lines_per_second = WEISEN_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_WISSEN, sizeof(sp->version));	/* PRQA S 2845 */
	} else {
		extclk = 27000000;   // 27M
		sp->lines_per_second = INCEPTIO_LINES_PER_SECOND;
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));	/* PRQA S 2845 */
	}

	switch (si->extra_mode & 0xff) {
		case (uint32_t)SUNNY_M25F120D12G3_S1R8T2:
			if (si->config_index & PWL_24BIT) {
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
static int32_t hb_e2prom_read_data(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
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

static int32_t hb_e2prom_read_double(int32_t i2c_num, int32_t base_addr, double *data)
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

static int32_t hb_e2prom_read_img_info(int32_t i2c_num, int32_t base_addr, uint64_t *data)
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

static int32_t hb_e2prom_read_array(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
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

static int32_t get_intrinsic_params(sensor_info_t *si,
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

static int32_t get_sns_info(sensor_info_t *si, cam_parameter_t *csp, uint8_t type)
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
SENSOR_MODULE_ECF(ar0820std, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D16);
sensor_module_t ar0820std = {
	.module = SENSOR_MNAME(ar0820std),
#else
sensor_module_t ar0820std = {
	.module = "ar0820std",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.get_sns_params = get_sns_info,
#ifdef CAM_DIAG
	.diag_nodes_init = sensor_diag_nodes_init,
#endif
};
