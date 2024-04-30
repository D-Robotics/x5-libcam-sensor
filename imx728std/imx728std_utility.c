/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[imx728std]:" fmt

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
#include <netinet/in.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/imx728std_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

#define SENSOR_REG_WIDTH REG16_VAL16
#define SERDES_REG_WIDTH REG16_VAL8
#define POC_REG_WIDTH REG8_VAL8

#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)

#define EN_DRIVER_CONTROL 0U

#define CAM_I2C_RETRY_MAX 10

#define EEPROM_I2C_ADDR_ALIAS_ID        (0x51)

typedef int32_t (*SENSOR_CONFIG_FUNC)(sensor_info_t *sensor_info);

#define CONFIG_INDEX_ALL ( \
	AE_DISABLE | \
	AWB_DISABLE | \
	TEST_PATTERN | \
	FLIP | \
	MIRROR | \
	TRIG_STANDARD | \
	TRIG_SHUTTER_SYNC \
)

emode_data_t emode_data[MODE_TYPE_MAX] = {
	[SENSING_GM24F120D12_S1R0T4E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x1a,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_GM24F120D12_S1R0T4E5, "0.0.1", "null", "null", &emode_data[SENSING_GM24F120D12_S1R0T4E5]),
	SENSOR_EMEND()
};

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_fps_div(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_external_mode(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_AE_DISABLE] = sensor_config_index_ae_disable,
	[B_AWB_DISABLE] = sensor_config_index_awb_disable,
	// [B_TEST_PATTERN] = sensor_config_index_test_pattern,
	// [B_FLIP] = sensor_config_index_filp_setting,
	// [B_MIRROR] = sensor_config_index_mirror_setting,
	// [B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
	// [B_TRIG_EXTERNAL] = sensor_config_index_trig_external_mode,
};

static uint32_t ae_enable[CAM_MAX_NUM];
static uint32_t awb_enable[CAM_MAX_NUM];
sensor_turning_data_t tuning_data[CAM_MAX_NUM];


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
	// int32_t setting_size = 0;

	// vin_info("port:%d imx728 set test pattern\n",sensor_info->port);
	// setting_size = sizeof(imx728_stream_off_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	//         SERDES_REG_WIDTH, setting_size, imx728_stream_off_setting);
	// if (ret < 0) {
	// 	vin_err("%s set stream off failed\n", sensor_info->sensor_name);
	// 	return ret;
    // }
	// usleep(40 * 1000);
	// setting_size = sizeof(imx728_pattern_mode_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 		SERDES_REG_WIDTH, setting_size, imx728_pattern_mode_setting);
	// if (ret < 0) {
	// 	vin_err("senor %s write imx728_pattern_mode_setting\n",
	// 			sensor_info->sensor_name);
	// 	return ret;
	// }
	// setting_size = sizeof(imx728_stream_on_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	//         SERDES_REG_WIDTH, setting_size, imx728_stream_on_setting);
	// if (ret < 0) {
	// 	vin_err("%s set stream on failed\n", sensor_info->sensor_name);
	// 	return ret;
    // }
	// usleep(100 * 1000);

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

	if (deserial_if == NULL) {
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
	// setting_size = sizeof(imx728_stream_off_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 	SERDES_REG_WIDTH, setting_size, imx728_stream_off_setting);
	// if (ret < 0) {
	// 	vin_err("%s set trig stream off failed\n", sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(10 * 1000);
	// setting_size = sizeof(imx728_trigger_shutter_mode_setting) /sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 	SERDES_REG_WIDTH, setting_size, imx728_trigger_shutter_mode_setting);
	// if (ret < 0) {
	// 	vin_err("senor %s write trigger shutter mode setting error\n",
	// 		sensor_info->sensor_name);
	// 	return ret;
	// }
	// setting_size = sizeof(imx728_stream_on_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 		SERDES_REG_WIDTH, setting_size, imx728_stream_on_setting);
	// if (ret < 0) {
	// 	vin_err("%s set stream on failed\n", sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(100 * 1000);

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
	maxdes_ops_t *maxops = NULL;
	int32_t trigger_gpio;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;

	if (deserial_if == NULL) {
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

	// setting_size = sizeof(imx728_stream_off_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 	SERDES_REG_WIDTH, setting_size, imx728_stream_off_setting);
	// if (ret < 0) {
	// 	vin_err("%s set trig stream off failed\n", sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(10 * 1000);
	// setting_size = sizeof(imx728_trigger_external_mode_setting)/sizeof(uint32_t)/2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 	SERDES_REG_WIDTH, setting_size, imx728_trigger_external_mode_setting);
	// if (ret < 0) {
	// 	vin_err("senor %s write trigger shutter mode setting error\n",
	// 		sensor_info->sensor_name);
	// 	return ret;
	// }
	// setting_size = sizeof(imx728_stream_on_setting) / sizeof(uint32_t) / 2;
	// ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
	// 		SERDES_REG_WIDTH, setting_size, imx728_stream_on_setting);
	// if (ret < 0) {
	// 	vin_err("%s set stream on failed\n", sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(100 * 1000);

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

static void sensor_common_data_init(sensor_info_t *sensor_info,
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
 * @brief sensor_param_init : set sensor VTS, HTS, X/Y_START/END reg
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 * @param [in] turning_data : store sensor reg
 *
 * @return ret
 */
static int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
    int ret = RET_OK;

    if (FPS_30 == sensor_info->fps) {
        turning_data->sensor_data.VMAX = IMX728_30FPS_VMAX;
        turning_data->sensor_data.HMAX = IMX728_30FPS_HTS;
    } else {
        vin_err("Invalid fps: %d\n", sensor_info->fps);
        return -RET_ERROR;
    }
    turning_data->sensor_data.active_width = ACTIVE_WIDTH;
    turning_data->sensor_data.active_height = ACTIVE_HEIGHT;

    if (PWL_M == sensor_info->sensor_mode) {
        turning_data->sensor_data.analog_gain_max =
            (sizeof(imx728_again_lut) / sizeof(uint32_t) - 1) * 8192;
        turning_data->sensor_data.exposure_time_min = 1;
        turning_data->sensor_data.exposure_time_max = turning_data->sensor_data.VMAX - 8U;
        turning_data->sensor_data.exposure_time_long_max = 0;
        turning_data->sensor_data.lines_per_second = sensor_info->fps * IMX728_30FPS_VMAX;
    } else {
        vin_err("Invalid sensor mode: %d\n", sensor_info->sensor_mode);
        return -RET_ERROR;
    }

	turning_data->sensor_data.digital_gain_max =
			(sizeof(imx728_dgain_lut) / sizeof(uint32_t) - 1) * 8192;
    turning_data->sensor_data.turning_type = 6;       // use tunning lut
    turning_data->sensor_data.fps = sensor_info->fps;  // fps
    turning_data->sensor_data.conversion = 1;

	sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_R, BAYER_PATTERN_RGGB);
	if (sensor_info->config_index & PWL_24BIT) {
		sensor_data_bits_fill(&turning_data->sensor_data, 24);
		vin_info("sensor data bits fill pwl 24bit\n");
	} else {
		sensor_data_bits_fill(&turning_data->sensor_data, 20);
		vin_info("sensor data bits fill pwl 20bit\n");
	}

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
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(imx728_stream_on_setting)) {
		memcpy(stream_on, imx728_stream_on_setting, sizeof(imx728_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(imx728_stream_off_setting)) {
		memcpy(stream_off, imx728_stream_off_setting, sizeof(imx728_stream_off_setting));
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

	memcpy(&tuning_data[sensor_info->dev_port], &turning_data,
		sizeof(sensor_turning_data_t));
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

	vin_info("port:%d config sensor_mode: %d\n", sensor_info->port, sensor_info->sensor_mode);

	/*======sensor_init*/
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0x9730, 0xa5);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x10);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	setting_size = sizeof(rccg_init_parameter1) / sizeof(uint32_t)/2;
	vin_info("rccg_init_parameter1 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_parameter1);
	if (ret < 0) {
		vin_err("write rccg_init_parameter1 error\n");
		return ret;
	}
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	setting_size = sizeof(rccg_init_parameter2) / sizeof(uint32_t)/2;
	vin_info("rccg_init_parameter2 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_parameter2);
	if (ret < 0) {
		vin_err("write rccg_init_parameter2 error\n");
		return ret;
	}	
	usleep(3*1000);

	setting_size = sizeof(rccg_change_mode_setting) / sizeof(uint32_t)/2;
	vin_info("rccg_change_mode_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_change_mode_setting);
	if (ret < 0) {
		vin_err("write rccg_change_mode_setting error\n");
		return ret;
	}
	usleep(1*1000);

	setting_size = sizeof(rccg_crop_setting) / sizeof(uint32_t)/2;
	vin_info("rccg_crop_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_crop_setting);
	if (ret < 0) {
		vin_err("write rccg_crop_setting error\n");
		return ret;
	}
	usleep(1*1000);

	setting_size = sizeof(rccg_hdr_modr_auto_setting) / sizeof(uint32_t)/2;
	vin_info("rccg_hdr_modr_auto_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_hdr_modr_auto_setting);
	if (ret < 0) {
		vin_err("write rccg_hdr_modr_auto_setting error\n");
		return ret;
	}
	usleep(1*1000);

	setting_size = sizeof(rccg_2a_setting) / sizeof(uint32_t)/2;
	vin_info("rccg_2a_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_2a_setting);
	if (ret < 0) {
		vin_err("write rccg_2a_setting error\n");
		return ret;
	}
	usleep(2*1000);

	setting_size = sizeof(rccg_max_nr_setting) / sizeof(uint32_t)/2;
	vin_info("rccg_max_nr_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_max_nr_setting);
	if (ret < 0) {
		vin_err("write rccg_max_nr_setting error\n");
		return ret;
	}
	usleep(2*1000);

	setting_size = sizeof(rccg_init_stream1) / sizeof(uint32_t)/2;
	vin_info("rccg_max_nr_setting = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_max_nr_setting);
	if (ret < 0) {
		vin_err("write rccg_max_nr_setting error\n");
		return ret;
	}
	usleep(1*1000);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x00);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}	
	usleep(1*1000);
	setting_size = sizeof(rccg_init_stream2) / sizeof(uint32_t)/2;
	vin_info("rccg_init_stream2 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_stream2);
	if (ret < 0) {
		vin_err("write rccg_init_stream2 error\n");
		return ret;
	}
	usleep(1*1000);
	setting_size = sizeof(rccg_init_stream3) / sizeof(uint32_t)/2;
	vin_info("rccg_init_stream3 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_stream3);
	if (ret < 0) {
		vin_err("write rccg_init_stream3 error\n");
		return ret;
	}
	usleep(35*1000);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x05);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}			
	usleep(1*1000);
	setting_size = sizeof(rccg_init_stream4) / sizeof(uint32_t)/2;
	vin_info("rccg_init_stream4 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_stream4);
	if (ret < 0) {
		vin_err("write rccg_init_stream4 error\n");
		return ret;
	}
	usleep(2*1000);
	setting_size = sizeof(rccg_init_stream5) / sizeof(uint32_t)/2;
	vin_info("rccg_init_stream5 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_stream5);
	if (ret < 0) {
		vin_err("write rccg_init_stream5 error\n");
		return ret;
	}
	usleep(2*1000);	
	/*init 2*/
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0x9730, 0xa5);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x10);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	setting_size = sizeof(rccg_init_parameter2) / sizeof(uint32_t)/2;
	vin_info("rccg_init_parameter2 = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
						setting_size, rccg_init_parameter2);
	if (ret < 0) {
		vin_err("write rccg_init_parameter2 error\n");
		return ret;
	}
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
			0xFFFF, 0x05);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	usleep(1*1000);

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

int32_t serdes_init(sensor_info_t *sensor_info)
{
	int ret = 0;
	int setting_size = 0;
	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	setting_size = sizeof(max9295_init_setting) / sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,setting_size, max9295_init_setting);
	if (ret < 0) {
		vin_err("write max9295_init_setting error\n");
		return ret;
	}
	usleep(100*1000);

	setting_size = sizeof(max9296_init_setting) / sizeof(uint32_t)/2;
	vin_info("96712 setting_size = %d\n", setting_size);
	ret = vin_write_array(sensor_info->bus_num, deserial_if->deserial_addr, 2,
						setting_size, max9296_init_setting);
	if (ret < 0) {
		vin_err("write max9296_init_setting error\n");
		return ret;
	}
	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;

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
	usleep(100*1000);

	/* 3. sensor mode config */
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	/* 4. config_index_init */
	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);

#if 0  // def CAM_DIAG
	/* sensor errb serial -> desrial mfp map */
	ret = max_serial_errb_mfp_map(sensor_info);
	if (ret < 0) {
		if (ret == -FLAG_NOT_FIND) {
			vin_warn("port:%d sensor errb not set mfp\n", sensor_info->port);
			ret = 0;
		} else {
			vin_err("port:%d sensor errb map fail\n", sensor_info->port);
		}
	}
#endif
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t bus = sensor_info->bus_num;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	vin_info("%d : ===================enter start===============\n", __LINE__);
	
	ret = hb_vin_i2c_write_reg16_data8(bus, deserial_if->deserial_addr, 0x0313, 0x00);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	usleep(1*1000);
	ret = hb_vin_i2c_write_reg16_data8(bus, deserial_if->deserial_addr, 0x0313, 0x02);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	vin_info("port: %d begin stream on\n", sensor_info->port);

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	uint8_t value;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = sensor_info->bus_num;

	setting_size = sizeof(imx728_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("%s sensor_stop setting_size %d\n",
			sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr,
							imx728_stream_off_setting[i*2],
							imx728_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		}
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
	uint16_t gain_val = 0;
	uint32_t line_val = 0;
	uint16_t line_v[2] = {0};

	vin_dbg("port:%d, aexp mode %d, line:%d -> 0x%x, again:%d -> 0x%x, dgain:%d -> 0x%x\n",
			info->port, mode, line[0], ntohl(line[0]),
			again[0], imx728_again_lut[again[0]], dgain[0], imx728_dgain_lut[dgain[0]]);

	if (PWL_M == info->sensor_mode) {
		// again
        curse_index = sizeof(imx728_again_lut) / sizeof(uint32_t) - 1;
        if (again[0] > curse_index)
            again[0] = curse_index;
        else
            curse_index = again[0];

		gain_val = ntohs((uint16_t)imx728_again_lut[curse_index]);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FME_SENSAGAIN, gain_val);
		if (ret < 0) {
			vin_err("port:%d write AGAIN value: 0x%x fail !!!\n", info->port, gain_val);
			return -RET_ERROR;
		}

		// dgain
		curse_index = sizeof(imx728_dgain_lut) / sizeof(uint32_t) - 1;
		if (dgain[0] > curse_index)
			dgain[0] = curse_index;
		else
			curse_index = dgain[0];

		gain_val = ntohs((uint16_t)imx728_dgain_lut[curse_index]);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FME_SENSDGAIN, gain_val);
		if (ret < 0) {
			vin_err("port:%d write DGAIN value: 0x%x fail !!!\n", info->port, gain_val);
			return -RET_ERROR;
		}

		// line
		line_val = ntohl(line[0]);
#if 1
		line_v[0] = (line_val >> 16) & 0xffff;
		line_v[1] = line_val & 0xffff;
		for(int i = 0; i < 2; i++) {
			ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, IMX728_SP1 + 2*i, line_v[i]);
			if (ret < 0) {
				vin_err("port:%d write SP1 value: 0x%x fail !!!\n", info->port, line_v[i]);
				return -RET_ERROR;
			}
			ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, IMX728_SP2 + 2*i, line_v[i]);
			if (ret < 0) {
				vin_err("port:%d write SP2 value: 0x%x fail !!!\n", info->port, line_v[i]);
				return -RET_ERROR;
			}
		}
#else
		if ((ret = hb_vin_i2c_write_block(bus, sensor_addr, IMX728_SP1, line_val, 1)) < 0)
			return -RET_ERROR;
		if ((ret = hb_vin_i2c_write_block(bus, sensor_addr, IMX728_SP2, line_val, 1)) < 0)
			return -RET_ERROR;
#endif
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
	// rgain
	val = ntohs((uint16_t)rgain);
	ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FULLMWBGAIN_R, val);
	if (ret < 0) {
		vin_err("port:%d write RGAIN value: 0x%x fail !!!\n", info->port, val);
		return -RET_ERROR;
	}

	// bgain
	val = ntohs((uint16_t)bgain);
	ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FULLMWBGAIN_B, val);
	if (ret < 0) {
		vin_err("port:%d write BGAIN value: 0x%x fail !!!\n", info->port, val);
		return -RET_ERROR;
	}

	// grgain
	val = ntohs((uint16_t)grgain);
	ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FULLMWBGAIN_GR, val);
	if (ret < 0) {
		vin_err("port:%d write GRGAIN value: 0x%x fail !!!\n", info->port, val);
		return -RET_ERROR;
	}

	// gbgain
	val = ntohs((uint16_t)gbgain);
	ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, FULLMWBGAIN_GB, val);
	if (ret < 0) {
		vin_err("port:%d write GBGAIN value: 0x%x fail !!!\n", info->port, val);
		return -RET_ERROR;
	}

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
SENSOR_MODULE_ECF(imx728std, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx728std = {
	.module = SENSOR_MNAME(imx728std),
#else
sensor_module_t imx728std = {
	.module = "imx728std",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.aexp_line_gain_control = sensor_aexp_line_gain_control,
	.awb_cct_control = sensor_awb_cct_control,
	.userspace_control = sensor_userspace_control,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
};
 
