/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ovx5b]:" fmt

#define SENSOR_ADDRESS 		(0x36)

#define X5B_AGAIN_ADDR_L 	(0x3508)
#define X5B_AGAIN_ADDR_H 	(0x3509)
#define X5B_LINE_ADDR_L 	(0x3501)
#define X5B_LINE_ADDR_H 	(0x3502)

#define OVX5B_30FPS_VMAX 	(2128)
#define OVX5B_30FPS_HMAX 	(376)
#define ACTIVE_WIDTH 		(2592)
#define ACTIVE_HEIGHT 		(1944)
#define FILENAME_RAW "/userdata/x5b/ovx5b_config_raw.json"
#define FILENAME_YUV "/userdata/x5b/ovx5b_config_yuv.json"

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

#include "inc/ovx5b_setting.h"
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
// #include "cJSON.h"
#include "hb_camera_data_config.h"

#define EN_DRIVER_CONTROL 0U
#define ISP_ADDRESS 0x24

typedef int32_t (*SENSOR_CONFIG_FUNC)(sensor_info_t *sensor_info);
static uint32_t ae_enable[CAM_MAX_NUM];
static uint32_t awb_enable[CAM_MAX_NUM];

#define CONFIG_INDEX_ALL ( \
	AE_DISABLE | \
	AWB_DISABLE | \
	TEST_PATTERN | \
	FLIP | \
	MIRROR | \
	TRIG_SHUTTER_SYNC \
	)

emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SENSING_M24F140D10_S1T7E3] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M24F140D8_S1T7E3] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_M24F140D10_S1T7E3, "0.0.1", "null", "null", &emode_data[SENSING_M24F140D10_S1T7E3]),
	SENSOR_EMADD(SENSING_M24F140D8_S1T7E3, "0.0.1", "null", "null", &emode_data[SENSING_M24F140D10_S1T7E3]),
	SENSOR_EMEND(),
};

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_AE_DISABLE] = sensor_config_index_ae_disable,
	[B_AWB_DISABLE] = sensor_config_index_awb_disable,
	// [B_TEST_PATTERN] = sensor_config_index_test_pattern,
	// [B_FLIP] = sensor_config_index_filp_setting,
	// [B_MIRROR] = sensor_config_index_mirror_setting,
	// [B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
};

static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info)
{
	awb_enable[sensor_info->dev_port] = ~HAL_AWB_CCT_CONTROL;
	vin_info("awb is disabled\n");
	return 0;
}

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info)
{
	ae_enable[sensor_info->dev_port] = ~HAL_AE_LINE_GAIN_CONTROL;
	vin_info("ae is disabled\n");
	return 0;
}

static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info)
{
	int32_t i, ret = RET_OK;

	return ret;
}

static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info)
{
	/* mirror enable */
	int32_t mirror;
	int32_t ret = -1;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	return ret;
}

static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info)
{
	/* flip enable */
	int32_t flip;
	int32_t ret = -1;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

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
	maxdes_ops_t *maxops = NULL;
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
		sizeof(turning_data->sensor_name));
	return;
}

/**
 * @brief sensor_param_init : set sensor VTS, HTS, X/Y_START/END reg
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
	float fps;

    if (30 == sensor_info->fps) {
        turning_data->sensor_data.VMAX = OVX5B_30FPS_VMAX;
        turning_data->sensor_data.HMAX = OVX5B_30FPS_HMAX;
    } else {
        vin_err("Invalid fps: %d\n", sensor_info->fps);
        return -RET_ERROR;
    }
    turning_data->sensor_data.active_width = ACTIVE_WIDTH;
    turning_data->sensor_data.active_height = ACTIVE_HEIGHT;

    if (PWL_M == sensor_info->sensor_mode) {
        turning_data->sensor_data.exposure_time_min = 1;
        turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX - 30U;
        turning_data->sensor_data.exposure_time_long_max =  turning_data->sensor_data.VMAX - 30U;
        turning_data->sensor_data.lines_per_second = 63830;
    } else {
        vin_err("Invalid sensor mode: %d\n", sensor_info->sensor_mode);
        return -RET_ERROR;
    }
	turning_data->sensor_data.gain_max = 64 * 8192;
	turning_data->sensor_data.analog_gain_max = 64*8192;
	turning_data->sensor_data.digital_gain_max = 64*8192;

	/* calculate lines_per_second
	hts = 376, sclk = 24mhz
	row_time = hts/sclk = 15.666667us
	lines_per_second = 1/row_time = 63829.78 */

	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;
	turning_data->sensor_data.fps = sensor_info->fps;
	vin_dbg("HMAX = %d, VMAX = %d, width = %d, height = %d, lines_per_second = %d, xclk = %d, fps = %f\n",
		   turning_data->sensor_data.HMAX, turning_data->sensor_data.VMAX,
		   turning_data->sensor_data.active_width, turning_data->sensor_data.active_height,
		   turning_data->sensor_data.lines_per_second, sensor_info->sensor_clk, sensor_info->fps);

	sensor_data_bayer_fill(&turning_data->sensor_data, 10, BAYER_START_R, BAYER_PATTERN_RGGB);
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
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ovx5b_stream_on_setting)) {
		memcpy(stream_on, ovx5b_stream_on_setting, sizeof(ovx5b_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ovx5b_stream_off_setting)) {
		memcpy(stream_off, ovx5b_stream_off_setting, sizeof(ovx5b_stream_off_setting));
	} else {
		vin_err("Number of registers off stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

/**
 * @brief sensor_pwl_data_init : sensor linear mode
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
static int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    uint32_t open_cnt = 0;
    sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_dbg("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
    sensor_common_data_init(sensor_info, &turning_data);

    if (sensor_info->bus_type == I2C_BUS) {
		ret = sensor_param_init(sensor_info, &turning_data);
		if (ret < 0) {
			vin_err("sensor_param_init fail %d\n", ret);
			return -RET_ERROR;
		}
	}

#if EN_DRIVER_CONTROL
    turning_data.pwl.line = IMX728_SP1;
    turning_data.pwl.line_length = 4;
#endif

    ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

#if EN_DRIVER_CONTROL
    turning_data.pwl.line_p.ratio = 1 << 8;
    turning_data.pwl.line_p.offset = 0;
    turning_data.pwl.line_p.max = turning_data.sensor_data.VMAX - 8U;

    turning_data.pwl.again_control_num = 1;
    turning_data.pwl.again_control[0] = FME_SENSAGAIN;
    turning_data.pwl.again_control_length[0] = 2;
    turning_data.pwl.dgain_control_num = 1;
    turning_data.pwl.dgain_control[0] = FME_SENSDGAIN;
    turning_data.pwl.dgain_control_length[0] = 2;

    turning_data.pwl.again_lut = malloc(256 * sizeof(uint32_t));
    turning_data.pwl.dgain_lut = malloc(256 * sizeof(uint32_t));

    if (turning_data.pwl.again_lut != NULL) {
        memset(turning_data.pwl.again_lut, 0xff, 256 * sizeof(uint32_t));
        memcpy(turning_data.pwl.again_lut, imx728_again_lut,
               sizeof(imx728_again_lut));
        for (open_cnt = 0; open_cnt < sizeof(imx728_again_lut) / sizeof(uint32_t);
             open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.again_lut[open_cnt], 2);
		}
    } else {
        vin_err("Invalid malloc\n");
        return -RET_ERROR;
    }

    if (turning_data.pwl.dgain_lut != NULL) {
        memset(turning_data.pwl.dgain_lut, 0xff, 256 * sizeof(uint32_t));
        memcpy(turning_data.pwl.dgain_lut, imx728_dgain_lut,
               sizeof(imx728_dgain_lut));
        for (open_cnt = 0; open_cnt < sizeof(imx728_dgain_lut) / sizeof(uint32_t);
             open_cnt++) {
			VIN_DOFFSET(&turning_data.pwl.dgain_lut[open_cnt], 2);
		}
    } else {
        vin_err("Invalid malloc\n");
        return -RET_ERROR;
    }

    /* Setting the awb gain param. */
    turning_data.sensor_awb.bgain_addr[0] = FULLMWBGAIN_B;
    turning_data.sensor_awb.bgain_length[0] = 2;
    turning_data.sensor_awb.rgain_addr[0] = FULLMWBGAIN_R;
    turning_data.sensor_awb.rgain_length[0] = 2;
    turning_data.sensor_awb.grgain_addr[0] = FULLMWBGAIN_GR;
    turning_data.sensor_awb.grgain_length[0] = 2;
    turning_data.sensor_awb.gbgain_addr[0] = FULLMWBGAIN_GB;
    turning_data.sensor_awb.gbgain_length[0] = 2;
#endif
    /* isp gain result directly write to 16 bit sensor. */
    turning_data.sensor_awb.rb_prec = 8;

    ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
    if (ret < 0) {
        vin_err("sensor_%d ioctl turning param fail %d\n",
				sensor_info->port, ret);
#if EN_DRIVER_CONTROL
        goto err_exit;
#else
        return -RET_ERROR;
#endif
    }

#if EN_DRIVER_CONTROL
err_exit:
    if (turning_data.pwl.again_lut) {
        free(turning_data.pwl.again_lut);
        turning_data.pwl.again_lut = NULL;
    }
    if (turning_data.pwl.dgain_lut) {
        free(turning_data.pwl.dgain_lut);
        turning_data.pwl.dgain_lut = NULL;
    }
#endif
    return ret;
}


int32_t serdes_init(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
    uint32_t value = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	// reset 9296
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, deserial_if->deserial_addr, 0x0010, 0xF1);
	if(ret){
		vin_info("reset 9296 failed\n");
	}
	usleep(100*1000);

	// turn down poc
	ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num, 0x28, 0x01, 0x10);
	if(ret){
		vin_info("poc turn down failed\n");
	}
	usleep(100*1000);

	// turn on poc
	ret = hb_vin_i2c_write_reg8_data8(sensor_info->bus_num, 0x28, 0x01, 0x1F);
	if(ret){
		vin_info("poc turn on failed\n");
	}
	usleep(100*1000);

	setting_size = sizeof(ovx5b_stream_off_setting) / sizeof(uint32_t)/2;
	vin_info("ovx5b_stream_off_setting setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, ovx5b_stream_off_setting);
	if (ret < 0) {
		vin_err("write ovx5b_stream_off_setting error\n");
		return ret;
	}

	if(sensor_info->extra_mode == RAW_TYPE){
		setting_size = sizeof(max9295_raw_init_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2, setting_size, max9295_raw_init_setting);
		if(ret < 0) {
			vin_err("max9295_raw_init_setting %s fail\n", sensor_info->sensor_name);
			return -1;
		}		
	} else if (sensor_info->extra_mode == YUV422_TYPE) {
		setting_size = sizeof(max9295_yuv_init_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2, setting_size, max9295_yuv_init_setting);
		if(ret < 0) {
			vin_err("max9295_yuv_init_setting %s fail\n", sensor_info->sensor_name);
			return -1;
		}			
	}

	setting_size = sizeof(max9296_init_setting) / sizeof(uint32_t)/2;
	vin_info("9296 raw init setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max9296_init_setting);
	if (ret < 0) {
		vin_err("write max9296_init_setting error\n");
		return ret;
	}

    usleep(100*1000);
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, deserial_if->deserial_addr, 0x0003, 0x40);
    if(ret){
        vin_info("write 2c3 failed\n");
    }
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, deserial_if->deserial_addr, 0x02C0, 0xa7);
    if(ret){
        vin_info("write 2c3 failed\n");
    }
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, deserial_if->deserial_addr, 0x02C1, 0x07);
    if(ret){
        vin_info("write 2c4 failed\n");
    }
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, deserial_if->deserial_addr, 0x02BF, 0xeb);
    if(ret){
        vin_info("write 2c2 failed\n");
    }
	usleep(100*1000);
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02D3, 0xF4);
    if(ret){
        vin_info("write 2D6 failed\n");
    }
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02D4, 0x67);
    if(ret){
        vin_info("write 2D7 failed\n");
    }
    ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->serial_addr, 0x02D5, 0x07);
    if(ret){
        vin_info("write 2D8 failed\n");
    }
	usleep(100*1000);       

	return ret;
}

int32_t sensor_poweron(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

	if(sensor_info->power_mode) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
						     sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay *1000);
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
						      1-sensor_info->gpio_level[gpio]);
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

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;

	vin_info("port:%d config sensor_mode: %d\n", sensor_info->port, sensor_info->sensor_mode);

	/*sensor_init*/
	if(sensor_info->extra_mode == RAW_TYPE){
		// ret = parse_sensor_setting(sensor_info, FILENAME_RAW);
		// if(ret < 0) {
		// 	vin_err("parse_sensor_setting failed\n");
		// 	return ret;
		// }
		setting_size = sizeof(ovx5b_raw_init_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, ISP_ADDRESS, 2, setting_size, ovx5b_raw_init_setting);
		if(ret < 0) {
			vin_err("ovx5b_raw_init_setting %s fail\n", sensor_info->sensor_name);
			return -1;
		}
		setting_size = sizeof(ovx5b_strobe_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							setting_size, ovx5b_strobe_setting);
		if (ret < 0) {
			vin_err("write ovx5b_INIT_setting error\n");
			return ret;
		}
		// from ov, bypass x4000isp config
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3000, (uint16_t)0x00);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3001, (uint16_t)0x00);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3002, (uint16_t)0x00);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3003, (uint16_t)0x18);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3004, (uint16_t)0x00);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3005, (uint16_t)0x02);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3006, (uint16_t)0x8C);
		hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, ISP_ADDRESS, (uint16_t)0x3007, (uint16_t)0xFF);				
	} else if (sensor_info->extra_mode == YUV422_TYPE) {
		// ret = parse_sensor_setting(sensor_info, FILENAME_YUV);
		// if(ret < 0) {
		// 	vin_err("parse_sensor_setting failed\n");
		// 	return ret;
		// }
		setting_size = sizeof(ovx5b_yuv_init_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, ISP_ADDRESS, 2, setting_size, ovx5b_yuv_init_setting);
		if(ret < 0) {
			vin_err("ovx5b_yuv_init_setting %s fail\n", sensor_info->sensor_name);
			return -1;
		}
		setting_size = sizeof(ovx5b_strobe_setting) / sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
							setting_size, ovx5b_strobe_setting);
		if (ret < 0) {
			vin_err("write ovx5b_INIT_setting error\n");
			return ret;
		}			
	}

	usleep(100*1000);

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			/* ret = sensor_linear_data_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_linear_data_init %s fail\n", sensor_info->sensor_name);
				return ret;
			} */
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

	return ret;
}

// static int32_t parse_sensor_setting(sensor_info_t *info, const char *filename)
// {
// 	char *filebuf;
// 	FILE *fp;
// 	int32_t ret;
// 	struct stat statbuf;
// 	cJSON *node;
// 	cJSON *arrayitem, *addr, *val;
// 	int32_t array_size, array_index;
// 	uint32_t read_size;
// 	uint32_t init_setting[2] = {0};
// 	if(filename == NULL) {
// 		vin_err("cam config file is null !!\n"); /* PRQA S ALL */ /* print api */
// 		return -1;
// 	}
// 	vin_dbg("filename = %s \n", filename); /* PRQA S ALL */ /* print api */
// 	ret = stat(filename, &statbuf);
// 	if((ret < 0) || (0 == statbuf.st_size)) {
// 		vin_err("cam config file size is zero !!\n"); /* PRQA S ALL */ /* print api */
// 		return -1;
// 	}
// 	fp = fopen(filename,"r"); /* PRQA S ALL */
// 	if(fp == NULL) {
// 		vin_err("open %s fail!!\n", filename); /* PRQA S ALL */ /* print api */
// 		return -1;
// 	}
// 	filebuf = (char *)malloc((uint32_t)(statbuf.st_size + 1)); /*PRQA S ALL */
// 	if(NULL == filebuf) {
// 		vin_err("malloc buff fail !!\n"); /* PRQA S ALL */ /* print api */
// 		(void)fclose(fp);
// 		return -1;
// 	}
// 	(void)memset((void *)filebuf, 0, (size_t)(statbuf.st_size + 1));
// 	read_size = (uint32_t)fread((void *)filebuf, (size_t)statbuf.st_size, (size_t)1, fp);
// 	if (read_size < 1u) {
// 		vin_err("read json fail\n"); /* PRQA S ALL */ /* print api */
// 		free((void *)filebuf);	/* PRQA S 5118*/
// 		filebuf = NULL;
// 	}
// 	root=cJSON_Parse((const char *)filebuf); /* PRQA S ALL */
// 	if(NULL == root) {
// 		vin_err("parse json fail\n"); /* PRQA S ALL */ /* print api */
// 		free((void *)filebuf);	/* PRQA S 5118*/
// 		filebuf = NULL;
// 	}
// 	node = cJSON_GetObjectItem(root, "config");
// 	if(NULL != node) {
// 		array_size = cJSON_GetArraySize(node);
// 		for(array_index = 0; array_index < array_size; array_index++) {
// 			arrayitem = cJSON_GetArrayItem(node, array_index);
// 			if(NULL != arrayitem) {
// 				addr = cJSON_GetObjectItem(arrayitem, "addr");
// 				if(NULL != addr) {
// 					init_setting[0] = (uint32_t)vin_htoi(addr->valuestring);
// 				}
// 				val = cJSON_GetObjectItem(arrayitem, "val");
// 				if(NULL != val) {
// 					init_setting[1] = (uint32_t)vin_htoi(val->valuestring);
// 				}				
// 			}
// 			ret = hb_vin_i2c_write_reg16_data8(info->bus_num, info->sensor_addr, init_setting[0], init_setting[1]);
// 			if(ret < 0){
// 				cJSON_Delete(root);
// 				return -1;
// 			}
// 		}
// 	}	

// 	return ret;
// }

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size=0;

	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}
	ae_enable[sensor_info->dev_port] = HAL_AE_LINE_GAIN_CONTROL;
	awb_enable[sensor_info->dev_port] = HAL_AWB_CCT_CONTROL;

	/* 1.power on*/
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
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

	/* 2. sensor mode config */
	// ret = max_serial_init(sensor_info);
	// if (ret < 0) {
	// 	vin_err("max serial init error\n");
	// 	return ret;
	// }										
	/*======serdes init======*/
	ret = serdes_init(sensor_info);
	if (ret < 0) {
		vin_err("serdes_init fail\n");
		return ret;
	}

	/* 3. sensor mode config */
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	/* 4. config_index_init */
	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);
														
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
  	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	setting_size = sizeof(ovx5b_stream_on_setting) / sizeof(uint32_t)/2;
	vin_info("ovx5b_stream_on_setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, ovx5b_stream_on_setting);
	if(ret < 0) {
		vin_err("start %s fail\n", sensor_info->sensor_name);
    	return -1;
	}
    usleep(100*1000); 					   
	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;					
	usleep(100*1000);  
 	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	setting_size = sizeof(ovx5b_stream_off_setting) / sizeof(uint32_t)/2;
	vin_info("ovx5b_stream_off_setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, ovx5b_stream_off_setting);
	if(ret < 0) {
		vin_err("stop %s fail\n", sensor_info->sensor_name);
    	return -1;
	}
	return ret;
}

int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t gpio, ret = RET_OK;

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

/**
 * @brief sensor_aexp_line_gain_control : calculate line & gain via isp param
 *
 * @param [in] info : sensor info
 * @param [in] mode : mode
 * @param [in] line : isp line
 * @param [in] line_num : isp line_num
 * @param [in] again : isp again
 * @param [in] dgain : isp dgain
 * @param [in] gain_num : isp gain_num
 *
 * @return ret
 */
static int32_t sensor_aexp_line_gain_control(hal_control_info_t *info, uint32_t mode,
		uint32_t *line, uint32_t line_num, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
	int i, ret = RET_OK;
	uint16_t curse_index;
	int bus = info->bus_num;
	uint8_t sensor_addr = info->sensor_addr;
	uint16_t gval;
	uint8_t gval_l,gval_h;
	uint16_t lval;
	uint8_t lval_l,lval_h;	

	vin_dbg("port:%d, aexp mode %d, line:%d -> 0x%x, again:%d -> 0x%x, dgain:%d -> 0x%x\n",
			info->port, mode, line[0], line[0], again[0], rgbir_ovx5b_again[again[0]], dgain[0], rgbir_ovx5b_dgain[dgain[0]]);

    if (PWL_M == info->sensor_mode) {
        curse_index = sizeof(rgbir_ovx5b_again) / sizeof(uint32_t) - 1;
        if (again[0] > curse_index)
            again[0] = curse_index;
        else
            curse_index = again[0];

        gval = (uint16_t)rgbir_ovx5b_again[curse_index];
		vin_info(" again = %d \n", gval);
		gval_l = gval & 0xff;
		gval_h = (gval >> 8) & 0xff;
		vin_info(" ===========index: %d======= AGAIN val high = 0x%x======val low = 0x%x\n", curse_index,info->bus_num, gval_h, gval_l);
		ret = hb_vin_i2c_write_reg16_data8(info->bus_num, info->sensor_addr, X5B_AGAIN_ADDR_H, gval_l);
		if(ret < 0) {
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data8(info->bus_num, info->sensor_addr, X5B_AGAIN_ADDR_L, gval_h);
		if(ret < 0) {
			return ret;
		}

		// line
		lval = line[0];
		lval_l = lval & 0xff;
		lval_h = (lval >> 8) & 0xff;
		vin_info(" line val high = 0x%x======val low = 0x%x\n", lval_h, lval_l);
		ret = hb_vin_i2c_write_reg16_data8(info->bus_num, info->sensor_addr, X5B_LINE_ADDR_H, lval_l);
		if(ret < 0) {
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data8(info->bus_num, info->sensor_addr, X5B_LINE_ADDR_L, lval_h);
		if(ret < 0) {
			return ret;
		}
	} else {
		vin_err("Invid sensor mode: %d\n", info->sensor_mode);
		return -RET_ERROR;
	}

	return ret;
}

/**
 * @brief sensor_awb_cct_control : awb control
 *
 * @param [in] info : sensor info
 * @param [in] mode : mode
 * @param [in] rgain : isp rgain
 * @param [in] bgain : isp bgain
 * @param [in] grgain : isp grgain
 * @param [in] gbgain : isp gbgain
 * @param [in] color_temper : isp color temperature
 *
 * @return ret
 */
static int32_t sensor_awb_cct_control(hal_control_info_t *info, uint32_t mode, uint32_t rgain,
			uint32_t bgain, uint32_t grgain, uint32_t gbgain, uint32_t color_temper)
{
	int ret = RET_OK;
	uint16_t val = 0;
	uint16_t get_val = 0;
	int bus = info->bus_num;
	uint8_t sensor_addr = info->sensor_addr;
	vin_dbg("port:%d, 1 awb rgain: 0x%x, bgain: 0x%x, grgain: %d, gbgain: %d, color_temp: %d\n",
			info->port, rgain, bgain, grgain, gbgain, color_temper);

	return 0;
}

/**
 * @brief sensor_userspace_control : ae, awb enable or disable
 *
 * @param [in] port : port
 * @param [in] enable : set different value to enable different func
 *
 * @return ret
 */
static int32_t sensor_userspace_control(uint32_t port, uint32_t *enable)
{
#if (!EN_DRIVER_CONTROL)
	/* enable awb_cct_control and aexp_line_gain_control */
	*enable = (HAL_AWB_CCT_CONTROL & awb_enable[port]) +
			  (HAL_AE_LINE_GAIN_CONTROL & ae_enable[port]);
	vin_info("dev_port %d enter userspace_control enable = %d\n", port, *enable);
#endif
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(ovx5bstd, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D8);
sensor_module_t ovx5bstd = {
	.module = SENSOR_MNAME(ovx5bstd),
#else
sensor_module_t ovx5bstd = {
	.module = "ovx5bstd",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_line_gain_control = sensor_aexp_line_gain_control,
	.awb_cct_control = sensor_awb_cct_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.userspace_control = sensor_userspace_control,
};



