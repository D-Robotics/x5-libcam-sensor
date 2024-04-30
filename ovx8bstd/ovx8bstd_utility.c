/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ovx8bstd]:" fmt

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
#include <sys/shm.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <signal.h>
#include "inc/hb_vin.h"
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "inc/ovx8bstd_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ov_common_setting.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define DEFAULT_SENSOR_ADDR		(0x36)
#define DEFAULT_SERIAL_ADDR_A	(0x62)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR		(0x48)

#define SENSING_LINES_PER_SECOND             (11764)

#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"
#define FRAME_SKIP_COUNT    1
#define BUF_LEN             128
#define SHIFT_ROUNDING      0.5
#define ALIGN_4(num)	(((uint32_t)(num) + 0x03U) & (0xFFFFFFFcU))

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
	[SENSING_M24F120D12_S0R0T8E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_M24F120D12_S1R8T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 1,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_M24F30D12_S1R8T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 1,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[LCE_M24F121D12_S1R0T8E5] = {
		.serial_addr = 0x42,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x57,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[LCE_M24F30D12_S1R0T8E5] = {
		.serial_addr = 0x42,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x57,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_M24F120D12_S0R0T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 1,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[OFILM_M24F120D12_S1R8T7E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x10,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[TXD_M24F120D12_S1R8T4E6] = {
	        .serial_addr = 0x40,            // serial i2c addr
	        .sensor_addr = 0x36,            // sensor i2c addr
	        .eeprom_addr = 0x50,            // eeprom i2c addr
	        .serial_rclk_out = 0,           // 0: serial rclk disabl, 1: serial_rclk enable
	        .rclk_mfp = 4,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_M24F120D12_S0R0T8E5, "0.0.1", "lib_x8bRGGB_pwl12_PH_Fov120.so", "0.22.10.14", &emode_data[SENSING_M24F120D12_S0R0T8E5]),
	SENSOR_EMADD(SUNNY_M24F120D12_S1R8T7E5, "0.0.1", "lib_CW-OX8GB-A120+065-L.so", "0.23.1.16", &emode_data[SUNNY_M24F120D12_S1R8T7E5]),
	SENSOR_EMADD(SUNNY_M24F30D12_S1R8T7E5, "0.0.1", "lib_CW-OX8GB-A030+017-L.so", "0.23.1.16", &emode_data[SUNNY_M24F30D12_S1R8T7E5]),
	SENSOR_EMADD(LCE_M24F121D12_S1R0T8E5, "0.0.1", "lib_CL-OX8GB-L121+067-L.so", "0.23.03.27", &emode_data[LCE_M24F121D12_S1R0T8E5]),
	SENSOR_EMADD(LCE_M24F30D12_S1R0T8E5, "0.0.1", "lib_CL-OX8GB-L030+017-L.so", "0.23.03.24", &emode_data[LCE_M24F30D12_S1R0T8E5]),
	SENSOR_EMADD(SENSING_M24F120D12_S0R0T7E5, "0.0.1", "lib_x8bRGGB_pwl12_PH_Fov120.so", "0.22.10.14", &emode_data[SENSING_M24F120D12_S0R0T7E5]),
	SENSOR_EMADD(OFILM_M24F120D12_S1R8T7E5, "0.0.1", "lib_CO-OX8GB-A121+055-L.so", "0.23.05.11", &emode_data[OFILM_M24F120D12_S1R8T7E5]),
	SENSOR_EMADD(TXD_M24F120D12_S1R8T4E6, "0.0.1", "lib_CO-OX8GB-A121+055-L.so", "0.23.05.11",
				&emode_data[TXD_M24F120D12_S1R8T4E6]),
	SENSOR_EMEND(),
};

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_fps_div(sensor_info_t *sensor_info);
static int32_t sensor_config_index_dual_roi(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_AE_DISABLE] = sensor_config_index_ae_disable,
	[B_AWB_DISABLE] = sensor_config_index_awb_disable,
	[B_TEST_PATTERN] = sensor_config_index_test_pattern,
	[B_FLIP] = sensor_config_index_filp_setting,
	[B_MIRROR] = sensor_config_index_mirror_setting,
	[B_TRIG_STANDARD] = sensor_config_index_trig_mode,
	[B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_shutter_mode,
	[B_DUAL_ROI] = sensor_config_index_dual_roi,
};

static sensor_info_ex_t sensor_info_exs[CAM_MAX_NUM];

uint32_t ae_vs_line_disable = 0;
uint32_t ae_enable = HAL_AE_LINE_GAIN_CONTROL;
uint32_t awb_enable = HAL_AWB_CCT_CONTROL;   // HAL_AWB_CCT_CONTROL;
uint32_t ae_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_ae_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t awb_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_awb_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t dev_port2port[CAM_MAX_NUM];
uint32_t diag_mask[CAM_MAX_NUM];
uint32_t pre_awb_disable[CAM_MAX_NUM];
uint32_t extra_mode[CAM_MAX_NUM];
int32_t config_index[CAM_MAX_NUM];
uint64_t checksum_flag[CAM_MAX_NUM] = {0};
static uint32_t again_tmp_buf[CAM_MAX_NUM];
static uint32_t dgain_tmp_buf[CAM_MAX_NUM];
static uint32_t line_tmp_buf[CAM_MAX_NUM];
static uint32_t rgain_tmp_buf[CAM_MAX_NUM];
static uint32_t bgain_tmp_buf[CAM_MAX_NUM];
static uint32_t grgain_tmp_buf[CAM_MAX_NUM];
static uint32_t gbgain_tmp_buf[CAM_MAX_NUM];

sensor_turning_data_t turning_data;
sensor_pll_data_t sensor_pll_data;
static uint16_t dcg_add_vs_line_max[CAM_MAX_NUM];
uint16_t skip_frame_count[CAM_MAX_NUM] = {0};
int g_sensor_sts_fd[CAM_MAX_NUM];
static sensor_status_info_t* g_sensor_sts[CAM_MAX_NUM];

/* eeprom_color_ratio[*][0]: d65_lcg_color_ratio_rg **
** eeprom_color_ratio[*][1]: d65_lcg_color_ratio_bg **
** eeprom_color_ratio[*][2]: d65_spd_color_ratio_rg **
** eeprom_color_ratio[*][3]: d65_spd_color_ratio_bg **
** eeprom_color_ratio[*][4]: cwf_lcg_color_ratio_rg **
** eeprom_color_ratio[*][5]: cwf_lcg_color_ratio_bg **
** eeprom_color_ratio[*][6]: cwf_spd_color_ratio_rg **
** eeprom_color_ratio[*][7]: cwf_spd_color_ratio_bg **
** eeprom_color_ratio[*][8]: a_lcg_color_ratio_rg **
** eeprom_color_ratio[*][9]: a_lcg_color_ratio_bg **
** eeprom_color_ratio[*][10]: a_spd_color_ratio_rg **
** eeprom_color_ratio[*][11]: a_spd_color_ratio_bg **/
float golden_eeprom_color_ratio[CAM_MAX_NUM][COLOR_RATIO_NUM];
float eeprom_color_ratio[CAM_MAX_NUM][COLOR_RATIO_NUM];
float hcg_lcg_cg_ratio[CAM_MAX_NUM];	/* hcg/lcg conversion ratio */
float sensitivity_ratio[CAM_MAX_NUM];	/* lpd/spd sensitivity ratio */

int group_hold_start(hal_control_info_t *info);
int group_hold_end(hal_control_info_t *info);
int write_ae_reg(hal_control_info_t *info);
int write_awb_reg(hal_control_info_t *info);
int f_sensor_init_global_data(sensor_info_t *sensor_info);
int f_sensor_deinit_global_data(sensor_info_t *sensor_info);
static int32_t get_sensor_ratio_from_otp(hal_control_info_t *info);


static int32_t cam_setting_to_crc(int32_t reg_width, int32_t setting_size, uint32_t *cam_setting, uint8_t *crc_array)
{
    int32_t i, len;
    if (reg_width == REG16_VAL16) {
        for (i = 0; i < setting_size; i++) {
            crc_array[4*i] = cam_setting[2*i] >> 8;
            crc_array[4*i+1] = cam_setting[2*i] & 0xff;
            crc_array[4*i+2] = cam_setting[2*i+1] >> 8;
            crc_array[4*i+3] = cam_setting[2*i+1] & 0xff;
        }
        len = setting_size >> 2;
    } else if (reg_width == REG16_VAL8) {
        for (i = 0; i < setting_size; i++) {
            crc_array[3*i] = cam_setting[2*i] >> 8;
            crc_array[3*i+1] = cam_setting[2*i] & 0xff;
            crc_array[3*i+2] = cam_setting[2*i+1];
        }
        len = setting_size * 3;
    } else {
        for (i = 0; i < setting_size; i++) {
            crc_array[2*i] = cam_setting[2*i];
            crc_array[2*i+1] = cam_setting[2*i+1];
        }
        len = setting_size * 2;
    }
    return len;
}

static uint16_t cam_crc16(uint16_t crc, uint8_t const *buffer, size_t len)
{
    int32_t i;
    for (; len > 0; len--) {
        crc = crc ^ (*buffer++ << 8);
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC16_POLY;
            else
                crc <<= 1;
        }
        crc &= 0xffff;
    }
    return(crc);
}

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info)
{
	ae_enable = ~HAL_AE_LINE_GAIN_CONTROL;
	return RET_OK;
}

static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info)
{
	awb_enable = ~HAL_AWB_CCT_CONTROL;
	return RET_OK;
}

/* Enable ovx8b test pattern */
static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;

	setting_size = sizeof(ovx8b_test_pattern)/sizeof(uint32_t)/2;
	vin_info("test pattern init!\n");
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
			      setting_size, ovx8b_test_pattern);
	if (ret < 0)
		vin_err("config test pattern fail!!!\n");
	return ret;
}

/* Eable ovx8b flip */
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t flip;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	flip = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
					   sensor_addr, OV_MIRROR_FLIP);
	flip |= BIT(2);
	vin_info("ov_mirror_flip 0x%02x\n", flip);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
					   OV_MIRROR_FLIP, flip);
	if (ret < 0)
		vin_err("senor %s write flip pattern setting error\n",
			sensor_info->sensor_name);
	return ret;
}

/* Eable ovx8b mirror */
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t mirror;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	mirror = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
					     sensor_addr, OV_MIRROR_FLIP);
	mirror &= ~BIT(5);
	vin_info("ov_mirror_flip 0x%02x\n", mirror);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
					   OV_MIRROR_FLIP, mirror);
	if (ret < 0)
		vin_err("senor %s write mirror pattern setting error\n",
			sensor_info->sensor_name);
	return ret;
}

static int32_t sensor_adjust_exposure_point(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t vts_v, init_row_cnt, sync_row_cnt;

	vts_v = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_VTS);
	if (vts_v < 0) {
		vin_err("port_%d read vts error\n", sensor_info->port);
		return vts_v;
	}
	init_row_cnt = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_TC_R_INIT_MAIN);
	if (init_row_cnt < 0) {
		vin_err("port_%d read init_row_cnt error\n", sensor_info->port);
		return init_row_cnt;
	}

	sync_row_cnt = vts_v - init_row_cnt;
	vin_info("port:%d %s adjust exp point write 0x3882 val: 0x%x\n",
		sensor_info->port, sensor_info->sensor_name, sync_row_cnt);
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_SYNC_ROW_CNT_ADJ, sync_row_cnt);
	if (ret < 0)
		vin_err("port_%d write sync_row_cnt error\n", sensor_info->port);
	return ret;
}

/* Enable ovx8b frame sync mode */
static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
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
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	setting_size = sizeof(ovx8b_trigger_arbitrary_mode_setting)/sizeof(uint16_t)/2;
	for(int32_t i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
						   ovx8b_trigger_arbitrary_mode_setting[i*2],
						   ovx8b_trigger_arbitrary_mode_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : standard trigger %s fail\n", __LINE__,
				sensor_info->sensor_name);
			return ret;
		}
	}
	ret = sensor_adjust_exposure_point(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_adjust_exposure_point %s err\n",
			sensor_info->port, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

/* Enable ovx8b line shutter sync mode */
static int32_t sensor_config_index_trig_shutter_mode(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t trig_pin_num = 0;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	maxdes_ops_t *maxops;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
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
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}
	setting_size = sizeof(ovx8b_trigger_arbitrary_mode_setting)/sizeof(uint16_t)/2;
	for(int32_t i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
						   ovx8b_trigger_arbitrary_mode_setting[i*2],
						   ovx8b_trigger_arbitrary_mode_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : standard trigger %s fail\n", __LINE__,
				sensor_info->sensor_name);
			return ret;
		}
	}
	vin_info("%s disable ae vs_line\n", sensor_info->sensor_name);
	ae_vs_line_disable = 1;

	ret = sensor_adjust_exposure_point(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor_adjust_exposure_point %s err\n",
			sensor_info->port, sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

static int32_t sensor_config_index_dual_roi(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	uint32_t bus;

	vin_info("port:%02d dual roi init\n", sensor_info->port);
	setting_size = sizeof(ovx8b_hdr_4exp_30fps_dual_roi_init_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH,
				setting_size, ovx8b_hdr_4exp_30fps_dual_roi_init_setting);
	if (ret < 0) {
		vin_err("senor %s write dual roi setting error\n", sensor_info->sensor_name);
		return ret;
	}
	return 1;        // Return 1 is dual roi config done, to differentiate from dual_roi not set
}

int32_t hb_e2prom_read_data(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
			    uint8_t eeprom_i2c_addr, uint64_t *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = vin_i2c_read_retry(i2c_num, eeprom_i2c_addr, REG16_VAL8,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val &= 0xff;
		ret <<= 8;
		ret |= val;
	}
	*data = ret;
	return RET_OK;
}

int32_t check_eeprom_vendor_id(sensor_info_t *si)
{
	int32_t i2c_num, setting_size;
	int64_t vendor_id = 0;
	int32_t ret = RET_OK;
	uint8_t e2prom_i2c_addr = DEFAULT_EEPROM_I2C_ADDR;

	if (!si) {
		vin_err("input si is null\n");
		return -RET_ERROR;
	}
	if (si->eeprom_addr) {
		e2prom_i2c_addr = si->eeprom_addr;
	} else {
		return -NOT_CONFIG_FLAG;
	}
	i2c_num = si->bus_num;
	vin_info("%s eeprom i2c addr:0x%x, i2c bus:%d\n", si->sensor_name, e2prom_i2c_addr, i2c_num);

	if (si->extra_mode == SUNNY_M24F120D12_S1R8T7E5 ||
		si->extra_mode == SUNNY_M24F30D12_S1R8T7E5) {
		ret = hb_e2prom_read_data(i2c_num, 2, VENDOR_ID_ADDR, e2prom_i2c_addr, &vendor_id);
		if (ret < 0 || vendor_id != WITHOUT_CRYSTAL) {
			setting_size = sizeof(serial_rclk_output_disable) / sizeof(uint32_t) / 2;
			vin_info("serializer need not provide clk, close rclk output\n");
			ret = vin_write_array(i2c_num, si->serial_addr, REG16_VAL8,
								setting_size, serial_rclk_output_disable);
			if (ret < 0) {
				vin_err("write serial rclk output disable error\n");
				return ret;
			}
		}
	}

	return ret;
}

/**
 * @brief check_eeprom_wb_ratio : check wb color ratio in eeprom
 *
 * @param [in] info : sensor info
 *
 * @return ret
 */
int32_t check_eeprom_wb_ratio(sensor_info_t *si) {
	int32_t i2c_num;
	uint64_t data, data1, tmp, checksum = 0;
	int32_t ret = RET_OK;
	uint16_t indi_reg_base, golden_reg_base, checksum_reg;
	deserial_info_t *deserial_if;
	uint8_t eeprom_addr_alias_id;

	if (!si) {
		vin_err("input si is null!\n");
		return -RET_ERROR;
	}
	if (si->eeprom_addr == 0) {
		checksum_flag[si->port] = 0;
		return ret;
	}
	i2c_num = si->bus_num;
	deserial_if = si->deserial_info;
	if ((ret = hb_e2prom_read_data(i2c_num, 2, GOLDEN_D65_LCG_COLOR_RATIO_RG, si->eeprom_addr, &data)) < 0)
			return ret;
	if ((ret = hb_e2prom_read_data(i2c_num, 2, GOLDEN_D65_LCG_COLOR_RATIO_RG_V2, si->eeprom_addr, &data1)) < 0)
			return ret;
	if ((data != 0xffff) && (data != 0x0) && ((data1 == 0x0) || (data1 == 0xffff))) {
		vin_dbg("using sunny eeprom wb calib addr\n");
		golden_reg_base = GOLDEN_D65_LCG_COLOR_RATIO_RG;
		indi_reg_base = D65_LCG_COLOR_RATIO_RG;
		checksum_reg = COLOR_RATIO_CHECKSUM;
	} else if (((data == 0xffff) || (data == 0x0)) && (data1 != 0x0) && (data1 != 0xffff)) {
		vin_dbg("using lce eeprom wb calib addr\n");
		golden_reg_base = GOLDEN_D65_LCG_COLOR_RATIO_RG_V2;
		indi_reg_base = D65_LCG_COLOR_RATIO_RG_V2;
		checksum_reg = COLOR_RATIO_CHECKSUM_V2;
	} else {
		vin_dbg("no eeprom wb calib, using pre_awb\n");
		checksum_flag[si->port] = 0;
		return ret;
	}

	for (int i = 0; i < COLOR_RATIO_NUM; i++) {
		if ((ret = hb_e2prom_read_data(i2c_num, 2, indi_reg_base + 2*i, si->eeprom_addr, &data)) < 0)
			return ret;
		tmp = data & 0xff;
		data = (data >> 8) | (tmp << 8);
		checksum+=data;
		eeprom_color_ratio[si->dev_port][i] = (float)data / 10000.0;
		vin_dbg("0x%x(0x%lx)\n", indi_reg_base + 2*i, data);
		if ((ret = hb_e2prom_read_data(i2c_num, 2, golden_reg_base + 2*i, si->eeprom_addr, &data)) < 0)
			return ret;
		tmp = data & 0xff;
		data = (data >> 8) | (tmp << 8);
		checksum+=data;
		golden_eeprom_color_ratio[si->dev_port][i] = (float)data / 10000.0;
		vin_dbg("0x%x(0x%lx)\n", golden_reg_base + 2*i, data);
	}
	checksum = checksum % 65535 + 1;
	if ((ret = hb_e2prom_read_data(i2c_num, 2, checksum_reg, si->eeprom_addr, &data)) < 0)
		return ret;
	if (checksum != data) {
		/* checksum fail then using pre_awb */
		checksum_flag[si->port] = 0;
		vin_dbg("checksum error reg = %ld, cal = %ld, use pre_awb\n", data, checksum);
	} else {
		checksum_flag[si->port] = 1;
	}
	return ret;
}

/* Sensor awb debug */
static int32_t sensor_awb_info_set(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch (sensor_info->extra_mode) {
		case LCE_M24F121D12_S1R0T8E5:
			extra_mode[sensor_info->port] = LCE_M24F121D12_S1R0T8E5;
			vin_info("The pre awb ratio is OX8GB-L121+067\n");
			break;
		case LCE_M24F30D12_S1R0T8E5:
			extra_mode[sensor_info->port] = LCE_M24F30D12_S1R0T8E5;
			vin_info("The pre awb ratio is OX8GB-L030+017\n");
			break;
		case SUNNY_M24F120D12_S1R8T7E5:
			extra_mode[sensor_info->port] = SUNNY_M24F120D12_S1R8T7E5;
			vin_info("The pre awb ratio is OX8GB-A120+065\n");
			break;
		case SUNNY_M24F30D12_S1R8T7E5:
			extra_mode[sensor_info->port] = SUNNY_M24F30D12_S1R8T7E5;
			vin_info("The pre awb ratio is OX8GB-A130+017\n");
			break;
		case SENSING_M24F120D12_S0R0T8E5:
			extra_mode[sensor_info->port] = SENSING_M24F120D12_S0R0T8E5;
			vin_info("The pre awb ratio is default\n");
			break;
		case SENSING_M24F120D12_S0R0T7E5:
			extra_mode[sensor_info->port] = SENSING_M24F120D12_S0R0T7E5;
			vin_info("The pre awb ratio is default\n");
			break;
		case OFILM_M24F120D12_S1R8T7E5:
			extra_mode[sensor_info->port] = OFILM_M24F120D12_S1R8T7E5;
			vin_info("The pre awb ratio is default\n");
			break;
		case (uint32_t)TXD_M24F120D12_S1R8T4E6:
		        extra_mode[sensor_info->port] = (int32_t)TXD_M24F120D12_S1R8T4E6;
		        vin_info("The pre awb ratio is default\n");
		        break;
		default:
			vin_err("Don't support extra_mode %d\n", sensor_info->extra_mode);
			return -1;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/pre_awb_disable", ISINT,
			&pre_awb_disable[sensor_info->port]);
	if (ret < 0) {
		pre_awb_disable[sensor_info->port] = 0;
	} else if (pre_awb_disable[sensor_info->port]) {
		vin_info("The pre awb is disabled");
	}
	return 0;
}

/* Sensor diag_mask */
static void sensor_config_debug_mask(sensor_info_t *sensor_info)
{
	int8_t sensor_group_hold_disable = 0, sccb_i2c_en = 0;
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];

	/* Group hold config parse*/
	sensor_param_parse(sensor_info, "sensor_debug/group_hold_disable", ISINT,
			&sensor_group_hold_disable);
	sensor_info_ex->diag_mask.sensor_group_hold_off = sensor_group_hold_disable;

	/* Sccb crc check config parse */
	sensor_param_parse(sensor_info, "sensor_debug/sccb_crc", ISINT,
			&sccb_i2c_en);
	sensor_info_ex->diag_mask.sensor_i2c_crc = sccb_i2c_en;

	if (sensor_info_ex->diag_mask.sensor_group_hold_off)
		vin_warn("port [%d] sensor_group_hold is disable\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_i2c_crc)
		vin_warn("port [%d] sensor_i2c_crc is enable\n", sensor_info->port);
}

static int32_t sensor_config_special_timing(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, setting_size = 0;
	int8_t timging_sel;
	uint32_t *pdata = NULL;
	int32_t bus, sensor_addr;
	sensor_addr = sensor_info->sensor_addr;
	bus = sensor_info->bus_num;
	double ratio = 1.0;
	int32_t set = 0;
	uint8_t reg_vb[2];
	uint32_t reg_v, reg_vs;

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_hts_vts", ISINT,
			&timging_sel);
	if (ret == 0) {
		switch (timging_sel) {
			case OVX8B_TIMING_DEFAULT:
				vin_dbg("default settint!\n");
				return RET_OK;
			case OVX8B_TIMING_ONE:
				pdata = ovx8b_hts_vts_spec_setting;
				setting_size = sizeof(ovx8b_hts_vts_spec_setting)/sizeof(uint32_t)/2;
				vin_info("timing_sel%d beging settint!\n", timging_sel);
				break;
			default:
				vin_err("not support timing%d\n", timging_sel);
				return -RET_ERROR;
		}

		ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
		if (ret < 0) {
			vin_err("write special timing error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_hts_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_HTS_DCG, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts_dcg(0x%04x) %d to %d\n",
			ratio, OV_HTS_DCG, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, OV_HTS_DCG, reg_vs);
		if (ret < 0) {
			vin_err("write OV_HTS_DCG register error\n");
			return ret;
		}
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_HTS_S, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts_s(0x%04x) %d to %d\n",
			ratio, OV_HTS_S, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, OV_HTS_S, reg_vs);
		if (ret < 0) {
			vin_err("write OV_HTS_S register error\n");
			return ret;
		}
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_HTS_VS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts_vs(0x%04x) %d to %d\n",
			ratio, OV_HTS_VS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr, OV_HTS_VS, reg_vs);
		if (ret < 0) {
			vin_err("write OV_HTS_VS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_vts_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_VTS, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("vts ratio %.2f setting: vts(0x%04x) %d to %d\n",
			ratio, OV_VTS, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr,
			OV_VTS, reg_vs);
		if (ret < 0) {
			vin_err("write OV_VTS register error\n");
			return ret;
		}
		set++;
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/timing_pll_ratio", ISDOUBLE,
			&ratio);
	if ((ret == 0) && (ratio > 0.01)) {
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_PLL2_MULT, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("pll ratio %.2f setting: pll2_mult(0x%04x) %d to %d\n",
			ratio, OV_PLL2_MULT, reg_v, reg_vs);
		ret = hb_vin_i2c_write_reg16_data16(bus, sensor_addr,
			OV_PLL2_MULT, reg_vs);
		if (ret < 0) {
			vin_err("write OV_PLL2_MULT register error\n");
			return ret;
		}
		set++;
	}

	if (set == 0)
		vin_dbg("no special_timing prase\n");

	return RET_OK;
}

static int32_t ovx8b_config_crop_feature(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, setting_size = 0;
	uint32_t width, height, start;
	int32_t crop_offset_x = 0, crop_offset_y = 0;
	uint32_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t bus = deserial_if->bus_num;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	width = sensor_info->width;
	height = sensor_info->height;

	if (width == 0u || height == 0u)
		return ret;

	width = ALIGN_4(width);
	height = ALIGN_4(height);
	ret = sensor_param_parse(sensor_info, "sensor_debug/crop_offset_x", ISINT,
			&crop_offset_x);
	if (ret < 0) {
		crop_offset_x = -1;
		vin_info("no crop_offset_x prase\n");
	}

	ret = sensor_param_parse(sensor_info, "sensor_debug/crop_offset_y", ISINT,
			&crop_offset_y);
	if (ret < 0) {
		crop_offset_y = -1;
		vin_info("no crop_offset_y prase\n");
	}

	if (crop_offset_x < 0 && crop_offset_y < 0) {
		// no offset, center crop
		if (width > 3840u) {
			vin_err("under total:%d width:%d, not support x_offset:%d \n",
					3856, width, crop_offset_x);
			return -1;
		}
		if (height > 2160u) {
			vin_err("under total:%d height:%d, not support y_offset:%d \n",
					2176, height, crop_offset_y);
			return -1;
		}

		start = (3856u / 2u) - (width / 2u);
		ovx8b_crop_setting[11] = width >> 8u;  // width_h;
		ovx8b_crop_setting[13] = width & 0xffu;  // width_l;
		ovx8b_crop_setting[3] = start >> 8u;
		ovx8b_crop_setting[5] = start & 0xffu;

		start = (2176u / 2u) - (height / 2u);
		ovx8b_crop_setting[15] = height >> 8u;  // height_h;
		ovx8b_crop_setting[17] = height & 0xffu;  // height_l;
		ovx8b_crop_setting[7] = start >> 8u;
		ovx8b_crop_setting[9] = start & 0xffu;
	} else if (crop_offset_x > 0 && crop_offset_y > 0) {
		if (ALIGN_4(crop_offset_x) + width > 3856u) {
			vin_err("under total:%d width:%d, not support x_offset:%d \n",
					3856, width, ALIGN_4(crop_offset_x));
			return -1;
		}
		if (ALIGN_4(crop_offset_y) + height > 2176u - 4u) {
			vin_err("under total:%d height:%d, not support y_offset:%d \n",
					2176, height, ALIGN_4(crop_offset_y));
			return -1;
		}
		ovx8b_crop_setting[11] = width >> 8u;  //  width_h;
		ovx8b_crop_setting[13] = width & 0xffu;  //  width_l;
		ovx8b_crop_setting[3] = ALIGN_4(crop_offset_x) >> 8u;  // offset_h
		ovx8b_crop_setting[5] = ALIGN_4(crop_offset_x) & 0xffu;  //  offset_l

		ovx8b_crop_setting[15] = height >> 8u;  // height_h;
		ovx8b_crop_setting[17] = height & 0xffu;  // height_l;
		ovx8b_crop_setting[7] = ALIGN_4(crop_offset_y) >> 8u;  // offset_h
		ovx8b_crop_setting[9] = ALIGN_4(crop_offset_y) & 0xffu;  //  offset_l
	} else {
		vin_err("not support this crop offset config\n");
		return -1;
	}
	vin_dbg("config crop width_h: 0x%x, width_l: 0x%x, start_h: 0x%x, start_l: 0x%x\n",
			ovx8b_crop_setting[11], ovx8b_crop_setting[13],
			ovx8b_crop_setting[3], ovx8b_crop_setting[5]);
	vin_dbg("config crop height_h: 0x%x, height_l: 0x%x, start_h: 0x%x, start_l: 0x%x\n",
			ovx8b_crop_setting[15], ovx8b_crop_setting[17],
			ovx8b_crop_setting[7], ovx8b_crop_setting[9]);

	pdata = ovx8b_crop_setting;
	setting_size = sizeof(ovx8b_crop_setting)/sizeof(uint32_t)/2UL;
	ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("config rectangle error\n");
		return ret;
	}
	vin_info("config crop success\n");
	return ret;
}

static int32_t sensor_config_index_fps_div(sensor_info_t *sensor_info)
{
	int32_t ret = -1;
	// fps div.
	char init_d[3];
	uint32_t vts_v;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_addr,
					  OV_VTS, init_d, 2);
	vts_v = (init_d[0] << 8) | init_d[1];
	vin_dbg("%dfps setting, vts %d to %d!\n",
		sensor_info->fps / 2, vts_v, vts_v * 2);
	vts_v *= 2;
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_addr,
					    OV_VTS, vts_v);
	if (ret < 0)
		vin_err("write register error\n");
	return ret;
}

int32_t ovx8b_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t serial_addr = sensor_info->serial_addr;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	uint32_t bus, deserial_addr;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;
	ret = vin_sensor_emode_parse(sensor_info, 'R');
	if (ret == -FLAG_NOT_FIND) {
		vin_info("port:%02d reset pin not find, now set software rst\n", sensor_info->port);
		setting_size = sizeof(ovx8b_init_setting_rst)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
					SENSOR_REG_WIDTH,
					setting_size, ovx8b_init_setting_rst);
		if (ret < 0) {
			vin_err("senor %s write rst setting error\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(10*1000);
	}
	if (sensor_info->sensor_mode == PWL_M) {
		ret = sensor_config_do(sensor_info, DUAL_ROI, sensor_config_index_funcs);
		if (ret < 0) {
			vin_err("config index dual roi init fail!!!\n");
			return ret;
		}
		if (ret == 0) {
			setting_size = sizeof(ovx8b_hdr_4exp_30fps_init_setting)/sizeof(uint32_t)/2;
			vin_info("hdr 4exp config mode!\n");
			ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size,
					      ovx8b_hdr_4exp_30fps_init_setting);
			if (ret < 0) {
				vin_err("vin_write_array error\n");
				return ret;
			}
		}
	} else {
		vin_err("sensor mode %d is err\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}

	if(sensor_info->sensor_mode == PWL_M) {
		if (sensor_info->config_index & PWL_24BIT) {
			setting_size = sizeof(sunny_ovx8b_pwl_setting_24bit)/sizeof(uint32_t)/2;
			vin_info("pwl 24bit init!\n");
			ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size,
					      sunny_ovx8b_pwl_setting_24bit);
			if (ret < 0) {
				vin_err("write ovx8b pwl register error\n");
				return ret;
			}
		} else {
			setting_size = sizeof(sunny_ovx8b_pwl_setting_20bit)/sizeof(uint32_t)/2;
			vin_info("pwl 20bit init!\n");
			ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size,
					      sunny_ovx8b_pwl_setting_20bit);
			if (ret < 0) {
				vin_err("write ovx8b pwl register error\n");
				return ret;
			}
		}
	}

	if (sensor_info->fps == 10) {
		setting_size = sizeof(ovx8b_init_setting_2160p_10fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx8b_init_setting_2160p_10fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 12) {
		// 12.5fps
		setting_size = sizeof(ovx8b_init_setting_2160p_12_5fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx8b_init_setting_2160p_12_5fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 15) {
		setting_size = sizeof(ovx8b_init_setting_2160p_15fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx8b_init_setting_2160p_15fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
    } else if (sensor_info->fps == 20) {
		setting_size = sizeof(ovx8b_init_setting_2160p_20fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx8b_init_setting_2160p_20fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
    } else if (sensor_info->fps == 25) {
		setting_size = sizeof(ovx8b_init_setting_2160p_25fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx8b_init_setting_2160p_25fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
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
int32_t sensor_param_init(sensor_info_t *sensor_info,
			sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	uint16_t x0, y0, x1, y1, width, height;
	uint16_t vts, hts_dcg, hts_s, hts_vs;
	uint16_t pll2_prediv0, pll2_prediv, pll2_mult, pll2_divsyspre, pll2_divsys;
	uint16_t pll2_vco, pll2_sclk, pll2_divsys_index;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = sensor_info->bus_num;
	float row_time, fps;
	float pll2_divsys_array[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};

	vts = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num, sensor_addr, OV_VTS);
	dcg_add_vs_line_max[sensor_info->port] = vts - 13;
	turning_data->sensor_data.VMAX  = vts;
	hts_dcg = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_HTS_DCG);
	hts_s = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_HTS_S);
	hts_vs = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_HTS_VS);
	width = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_X_OUTPUT);
	height = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_Y_OUTPUT);

	turning_data->sensor_data.HMAX = hts_dcg + hts_s + hts_vs;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;
	turning_data->sensor_data.gain_max = 1024 * 8192;
	turning_data->sensor_data.analog_gain_max = 1024*8192;
	turning_data->sensor_data.digital_gain_max = 1024*8192;
	turning_data->sensor_data.exposure_time_min = DCG_SPD_LINE_MIN;
	turning_data->sensor_data.exposure_time_max = vts - 13;
	turning_data->sensor_data.exposure_time_long_max = vts - 13;

	pll2_prediv0 = (hb_vin_i2c_read_reg16_data8(bus, sensor_addr,
						    OV_PLL2_PREDIV0) >> 7) + 1;
	pll2_prediv = hb_vin_i2c_read_reg16_data8(bus, sensor_addr,
						  OV_PLL2_PREDIV) & 0x7;
	pll2_mult = hb_vin_i2c_read_reg16_data16(bus, sensor_addr,
						 OV_PLL2_MULT) & 0x3FF;
	pll2_divsyspre = (hb_vin_i2c_read_reg16_data8(bus, sensor_addr,
						      OV_PLL2_DIVSYSPRE) & 0xF) + 1;
	pll2_divsys_index = hb_vin_i2c_read_reg16_data8(bus, sensor_addr, OV_PLL2_DIVSYS) & 0xF;
	if (pll2_divsys_index >= (sizeof(pll2_divsys_array) / sizeof(pll2_divsys_array[0])))
		pll2_divsys_index = 0;
	pll2_divsys = pll2_divsys_array[pll2_divsys_index];
	/* calculate lines_per_second
	hts = 2132, sclk = 162mhz -->81 for double row time
	row_time = hts/sclk = 26.321us
	lines_per_second = 1/row_time = 37992 */
	if (sensor_info->sensor_clk <= 0) {
		ret = vin_sensor_emode_parse(sensor_info, 'M');
		if (ret < 0) {
			vin_err("sensor_clk parse fail\n");
			return -1;
		}
		sensor_info->sensor_clk = ret;
	}
	pll2_vco = sensor_info->sensor_clk * pll2_mult / (pll2_prediv0 * pll2_prediv);
	pll2_sclk = pll2_vco / (pll2_divsyspre * pll2_divsys);
	row_time = (float)(turning_data->sensor_data.HMAX) * 2.0 / (float)pll2_sclk;
	turning_data->sensor_data.lines_per_second = 1000000 / row_time;
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;
	fps = (float)pll2_sclk * 1000000 / (2 * turning_data->sensor_data.HMAX *
		   turning_data->sensor_data.VMAX);
	sensor_pll_data.fps = fps;
	sensor_pll_data.sclk = pll2_sclk;
	vin_info("HMAX = %d, VMAX = %d, width = %d, height = %d, lines_per_second = %d, xclk = %d, fps = %f\n",
		   turning_data->sensor_data.HMAX, turning_data->sensor_data.VMAX,
		   turning_data->sensor_data.active_width, turning_data->sensor_data.active_height,
		   turning_data->sensor_data.lines_per_second, sensor_info->sensor_clk, fps);

	sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_B, BAYER_PATTERN_RGGB);
	if (sensor_info->config_index & PWL_24BIT) {
		sensor_data_bits_fill(&turning_data->sensor_data, 24);
		vin_info("sensor data bits fill pwl 24bit\n");
	} else {
		sensor_data_bits_fill(&turning_data->sensor_data, 20);
		vin_info("sensor data bits fill pwl 20bit\n");
	}

	return ret;
}
static int32_t sensor_stream_control_set(sensor_turning_data_t *turning_data)
{
	int32_t ret = RET_OK;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 2;
	if (sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ov_stream_on_setting)) {
		memcpy(stream_on, ov_stream_on_setting, sizeof(ov_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ov_stream_off_setting)) {
		memcpy(stream_off, ov_stream_off_setting, sizeof(ov_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

int32_t sensor_linear_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint32_t open_cnt = 0;
	char str[24] = {0};
	sensor_turning_data_t turning_data;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
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

	ret = sensor_stream_control_set(&turning_data);
	if (ret < 0) {
		vin_err("sensor_stream_control_set fail %d\n", ret);
		return -RET_ERROR;
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("[%s: %d]: sensor_%d ioctl fail %d\n", __func__, __LINE__, ret, ret);
		return -RET_ERROR;
	}

	return ret;
}

int32_t sensor_mode_config_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0, i;
	int32_t tmp = 0;
	int32_t vendor_id = 0;

	ret = max_serial_init(sensor_info);
	if (ret < 0) {
		vin_err("max serial init fail!\n");
		return ret;
	}
	vin_info("serial_init OK\n");

	// check eeprom wb color ratio
	if ((extra_mode[sensor_info->port]) == SUNNY_M24F120D12_S1R8T7E5 ||
		(extra_mode[sensor_info->port]) == SUNNY_M24F30D12_S1R8T7E5 ||
		(extra_mode[sensor_info->port]) == LCE_M24F121D12_S1R0T8E5 ||
		(extra_mode[sensor_info->port]) == LCE_M24F30D12_S1R0T8E5 ||
		(extra_mode[sensor_info->port]) == OFILM_M24F120D12_S1R8T7E5) {
		ret = check_eeprom_wb_ratio(sensor_info);
		if (ret < 0) {
			vin_err("sensor %s get_eeprom_wb_ratio fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
	ret = check_eeprom_vendor_id(sensor_info);
	if(ret < 0) {
		if (ret == -NOT_CONFIG_FLAG) {
			vin_info("%s not config eeprom map addr\n", sensor_info->sensor_name);
		} else {
			vin_err("%s check eeprom addr fail\n", sensor_info->sensor_name);
			return ret;
		}
	}
	ret = ovx8b_init(sensor_info);
	if(ret < 0) {
		vin_err("OVX8B_X3_config fail!\n");
		return ret;
	}
	vin_info("OVX8B_X3_config OK!\n");

	if (sensor_info->sensor_mode == PWL_M) {
		ret = sensor_pwl_data_init(sensor_info);
	} else {
		ret = sensor_linear_data_init(sensor_info);
	}

	if(ret < 0) {
		vin_err("sensor_%s_data_init %s fail\n", sensor_info->sensor_name,
			(sensor_info->sensor_mode != PWL_M) ? "linear" : "pwl");
		return ret;
	}

	ret = ovx8b_config_crop_feature(sensor_info);
	if (ret < 0) {
		vin_err("%s crop feature error\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	pthread_t t1;

	ret = sensor_awb_info_set(sensor_info);
	if (ret < 0) {
		vin_err("sensor extra_mode config is invalid\n");
		return ret;
	}
	dev_port2port[sensor_info->dev_port] = sensor_info->port;
	config_index[sensor_info->port] = sensor_info->config_index;
	sensor_config_debug_mask(sensor_info);

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

	ret = f_sensor_init_global_data(sensor_info);
	if (ret < 0) {
		vin_err("%d : init_global_data %s fail\n", __LINE__, sensor_info->sensor_name);
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
		}
	}
#endif
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	uint32_t bus = sensor_info->bus_num;

	memcpy(ae_reg_array[sensor_info->dev_port], ae_reg_array_base,
		sizeof(ae_reg_array_base));
	memset(bak_ae_reg_array[sensor_info->port], 0, sizeof(ae_reg_array_base));
	memcpy(awb_reg_array[sensor_info->dev_port], awb_reg_array_base,
		sizeof(awb_reg_array_base));
	memset(bak_awb_reg_array[sensor_info->port], 0, sizeof(awb_reg_array_base));
	again_tmp_buf[sensor_info->port] = 0;
	dgain_tmp_buf[sensor_info->port] = 0;
	line_tmp_buf[sensor_info->port] = 0;
	rgain_tmp_buf[sensor_info->port] = 0;
	bgain_tmp_buf[sensor_info->port] = 0;
	grgain_tmp_buf[sensor_info->port] = 0;
	gbgain_tmp_buf[sensor_info->port] = 0;
	pdata = ovx8b_stream_on_setting;
	setting_size = sizeof(ovx8b_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("sensor_addr 0x%x begin stream on\n", sensor_addr);
	ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0)
		vin_err("write register error\n");

	return ret;
}
int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t sensor_addr = sensor_info->sensor_addr;
	uint32_t bus = sensor_info->bus_num;

	/* xj3 */
	pdata = ovx8b_stream_off_setting;
	setting_size = sizeof(ovx8b_stream_off_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0)
		vin_err("write register error\n");

	return ret;
}
int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t gpio;

	f_sensor_deinit_global_data(sensor_info);
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

int32_t get_sensor_info(sensor_info_t *si, sensor_parameter_t *sp)
{
	int32_t ret = RET_OK;
	if (!sp || !si) {
		vin_err("input intrinsic or sensor info is null!\n");
		return -RET_ERROR;
	}
	int32_t i2c_num = si->bus_num;
	int32_t i2c_addr = si->sensor_addr;

	sp->frame_length = turning_data.sensor_data.VMAX;
	sp->line_length = turning_data.sensor_data.HMAX;

	sp->width = turning_data.sensor_data.active_width;
	sp->height = turning_data.sensor_data.active_height;

	if (si->extra_mode == SENSING_M24F120D12_S0R0T8E5) {
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));
	}

	sp->pclk = sensor_pll_data.sclk;
	sp->fps = sensor_pll_data.fps;
	sp->exp_num = HDR4;
	sp->lines_per_second = turning_data.sensor_data.lines_per_second;

	return ret;
}

int32_t hb_e2prom_read_array(int32_t i2c_num, int32_t byte_num, int32_t base_addr, uint8_t eeprom_i2c_addr,
		uint8_t *data)
{
	int32_t i, val, ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = vin_i2c_read_retry(i2c_num, eeprom_i2c_addr, REG16_VAL8,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		data[i] = val;
	}
	return RET_OK;
}

int32_t hb_e2prom_read_double(int32_t i2c_num, int32_t base_addr, uint8_t eeprom_i2c_addr, double *data)
{
	int32_t i, val, temp;
	uint64_t ret = 0;

	if (base_addr == FOV_ADDR_2) {
		temp = 3;
	} else {
		temp = 7;
	}
	for (i = temp; i >= 0; i --) {
		val = vin_i2c_read_retry(i2c_num, eeprom_i2c_addr, REG16_VAL8,
				base_addr + i);
		if (val < 0) {
			vin_err("i2c read fail i2c%d addr:0x%x ret:%d.\n", i2c_num,
					base_addr + i, val);
			return -RET_ERROR;
		}
		val &= 0xff;
		ret <<= 8;
		ret |= val;
	}
	if (base_addr == FOV_ADDR_2) {
		*data = *((float*)&ret);
	} else {
		*data = *((double*)&ret);
	}
	return RET_OK;
}

int32_t hb_e2prom_read_img_info(int32_t i2c_num, int32_t base_addr, uint8_t eeprom_i2c_addr,
                    uint64_t *data)
{
	int32_t val, ret = 0;

	val = vin_i2c_read_retry(i2c_num, eeprom_i2c_addr, REG16_VAL8,
			base_addr);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr, val);
		return -RET_ERROR;
	}
	val &= 0xff;
	ret = val;

	val = vin_i2c_read_retry(i2c_num, eeprom_i2c_addr, REG16_VAL8,
			base_addr + 1);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr + 1, val);
		return -RET_ERROR;
	}
	ret *= 100;
	ret += val;

	*data = ret;

	return RET_OK;
}

int32_t get_intrinsic_params(sensor_info_t *si,
	sensor_intrinsic_parameter_t *sip)
{
	int32_t i2c_num;
	uint64_t data;
	uint8_t serial_num[40] = {0};
	int32_t ret = RET_OK;
	uint8_t e2prom_i2c_addr = DEFAULT_EEPROM_I2C_ADDR;

	if (!sip || !si) {
		vin_err("input intrinsic or sensor info is null!\n");
		return -RET_ERROR;
	}
	i2c_num = si->bus_num;
	if (si->eeprom_addr) {
		e2prom_i2c_addr = si->eeprom_addr;
	} else {
		vin_info("%s not config eeprom map addr", si->sensor_name);
		return -1;
	}
	vin_info("%s e2prom_i2c_addr = 0x%x i2c:%d\n", si->sensor_name, e2prom_i2c_addr, i2c_num);

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));
	if ((si->extra_mode == LCE_M24F121D12_S1R0T8E5) ||
		(si->extra_mode == LCE_M24F30D12_S1R0T8E5)) {
		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_WIDTH_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->image_width = (uint16_t)data;

		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_HEIGHT_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->image_height = (uint16_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MAJOR_VERSION_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->major_version = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MINOR_VERSION_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->minor_version = (uint8_t)data;

		if ((ret = hb_e2prom_read_array(i2c_num, 32, MODULE_SERIAL_ADDR_2, e2prom_i2c_addr, sip->serial_num)) < 0)
			return ret;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, VENDOR_ID_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->vendor_id = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR_2, e2prom_i2c_addr, &data)) < 0)
			return ret;
		if (data == 2) {
			sip->cam_type = 0;
		} else {
			sip->cam_type = (uint8_t)data;
		}

		if ((ret = hb_e2prom_read_double(i2c_num, COD_X_ADDR_2, e2prom_i2c_addr, &sip->center_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_ADDR_2, e2prom_i2c_addr, &sip->center_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_ADDR_2, e2prom_i2c_addr, &sip->focal_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_ADDR_2, e2prom_i2c_addr, &sip->focal_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, FOV_ADDR_2, e2prom_i2c_addr, &sip->hfov)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K1_ADDR_2, e2prom_i2c_addr, &sip->k1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K2_ADDR_2, e2prom_i2c_addr, &sip->k2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P1_ADDR_2, e2prom_i2c_addr, &sip->p1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P2_ADDR_2, e2prom_i2c_addr, &sip->p2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K3_ADDR_2, e2prom_i2c_addr, &sip->k3)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K4_ADDR_2, e2prom_i2c_addr, &sip->k4)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K5_ADDR_2, e2prom_i2c_addr, &sip->k5)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K6_ADDR_2, e2prom_i2c_addr, &sip->k6)) < 0)
			return ret;
	} else {
		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_WIDTH_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->image_width = (uint16_t)data;

		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_HEIGHT_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->image_height = (uint16_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MAJOR_VERSION_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->major_version = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MINOR_VERSION_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->minor_version = (uint8_t)data;
		if ((ret = hb_e2prom_read_data(i2c_num, 2, VENDOR_ID_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->vendor_id = (uint16_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 4, MODULE_SERIAL_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->module_serial = (uint32_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->cam_type = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORTION_FLAG_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->distortion_flag = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 4, CRC32_1_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->crc32_1 = (uint32_t)data;

		if ((ret = hb_e2prom_read_double(i2c_num, COD_X_ADDR, e2prom_i2c_addr, &sip->center_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_ADDR, e2prom_i2c_addr, &sip->center_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_ADDR, e2prom_i2c_addr, &sip->focal_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_ADDR, e2prom_i2c_addr, &sip->focal_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, FOV_ADDR, e2prom_i2c_addr, &sip->hfov)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K1_ADDR, e2prom_i2c_addr, &sip->k1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K2_ADDR, e2prom_i2c_addr, &sip->k2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P1_ADDR, e2prom_i2c_addr, &sip->p1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P2_ADDR, e2prom_i2c_addr, &sip->p2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K3_ADDR, e2prom_i2c_addr, &sip->k3)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K4_ADDR, e2prom_i2c_addr, &sip->k4)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K5_ADDR, e2prom_i2c_addr, &sip->k5)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K6_ADDR, e2prom_i2c_addr, &sip->k6)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_data(i2c_num, 4, SD_CRC32_GROUP1_ADDR, e2prom_i2c_addr, &data)) < 0)
			return ret;
		sip->crc_group1 = (uint8_t)data;
	}

	vin_info("img_h:%d img_w:%d type:0x%x vendor:0x%x module_serial:0x%x\n",
			sip->image_height, sip->image_width, sip->cam_type, sip->vendor_id, sip->module_serial);
	vin_info("focal_u:%0.12lf focal_v:%0.12lf center_u:%0.12lf center_v:%0.12lf\n",
			sip->focal_u, sip->focal_v, sip->center_u, sip->center_v);
	vin_info("fov:%0.12lf k1:%0.12lf k2:%0.12lf p1:%0.12lf p2:%0.12lf\n",
			sip->hfov, sip->k1, sip->k2, sip->p1, sip->p2);
	vin_info("k3:%0.12lf k4:%0.12lf k5:%0.12lf k6:%0.12lf, crc_group1:%d\n",
			sip->k3, sip->k4, sip->k5, sip->k6, sip->crc_group1);

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
		vin_err("ovx8b param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

int group_hold_start(hal_control_info_t *info) {
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t sensor_addr = info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = info->bus_num;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int crc_last_check[CAM_MAX_NUM];
	int port = dev_port2port[info->port];

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] clear sccb crc failed\n", __func__, port);
			return ret;
		}
	}
	/* group hold start */
	setting_size = sizeof(group_hold_start_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(bus, info->sensor_addr,
		SENSOR_REG_WIDTH, setting_size, group_hold_start_setting);
	if (ret < 0) {
		vin_err("%s port [%d] group hold start failed!\n", __func__, port);
		return ret;
	}
	/* read SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			group_hold_start_setting, crc_array);
		if (len < 0) {
			vin_err("%s port [%d] cam_setting_to_crc failed\n", __func__, port);
		}
		ret = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] read sccb crc failed\n", __func__, port);
			return ret;
		}
		crc16_ret = cam_crc16(~0, crc_array, len);
		if (ret != crc16_ret) {
			vin_err("%s port [%d] crc_reg = 0x%x, crc_cal = 0x%x\n",
				__func__, port, ret, crc16_ret);
		}
		ret = (ret == crc16_ret)? 0 : (-1);
		// only call diag when state changes
		if (crc_last_check[port]!= ret) {
		        // camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[port]= ret;
		}
	}
	return 0;
}

int group_hold_end(hal_control_info_t *info)
{
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int crc_last_check[CAM_MAX_NUM];
	int port = dev_port2port[info->port];
	uint8_t sensor_addr = info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = info->bus_num;

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] clear sccb crc failed\n", __func__, port);
			return ret;
		}
	}
	/* group hold start */
	setting_size = sizeof(group_hold_end_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(bus, info->sensor_addr, SENSOR_REG_WIDTH,
			      setting_size, group_hold_end_setting);
	if (ret < 0) {
		vin_err("%s port [%d] group hold start failed!\n", __func__, port);
		return ret;
	}
	/* read SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			group_hold_end_setting, crc_array);
		if (len < 0) {
			vin_err("%s port [%d] cam_setting_to_crc failed\n", __func__, port);
		}
		ret = hb_vin_i2c_read_reg16_data16(bus, sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] read sccb crc failed\n", __func__, port);
			return ret;
		}
		crc16_ret = cam_crc16(~0, crc_array, len);
		if (ret != crc16_ret) {
			vin_err("%s port [%d] crc_reg = 0x%x, crc_cal = 0x%x\n",
				__func__, port, ret, crc16_ret);
		}
		ret = (ret == crc16_ret)? 0 : (-1);
		// only call diag when state changes
		if (crc_last_check[port]!= ret) {
			// camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[port]= ret;
		}
	}
	return 0;
}

/*
 * ov_write_awb_ae_block_with_valid
 * @valid_reg_array: the flag to distinguish the reg that need write, 1 means to write
 * @s_size: the number of reg, it must be less than BUF_LEN 128
 * */
static int32_t ov_write_awb_ae_block_with_valid(hal_control_info_t *info,
						uint32_t *valid_reg_array,
						uint32_t *psetting, int32_t s_size)
{
	int32_t reg_addr[BUF_LEN], dst_reg;
	int32_t begin_idx, blk_size;
	int32_t i, ret = 0, j = 0;
	int8_t data[BUF_LEN] = {0};

	begin_idx = 0;
	for (i = 0; i < s_size; i++) {
		if (valid_reg_array[i] == 0) {
			continue;
		}
		reg_addr[j] = psetting[i * 2] & 0xFFFF;
		data[j] = psetting[i * 2 + 1] & 0xFF;

		/* not continuous reg iic write */
		if ((j > 0) && (reg_addr[j] - reg_addr[j-1] != 1)) {
			blk_size = j - begin_idx;
			dst_reg = reg_addr[begin_idx];
			// vin_dbg("reg %x %x \n", reg_addr[j-1], reg_addr[j]);
			vin_dbg("i %d j %d bus %d i2c_addr %x reg_addr 0x%02x blk_size %d\n",
				i, j, info->bus_num, info->sensor_addr, dst_reg, blk_size);
			ret = vin_i2c_write_block(info->bus_num, REG_WIDTH_16bit, info->sensor_addr,
						  dst_reg, &data[begin_idx], blk_size);
			if (ret < 0)
				return ret;
			begin_idx += blk_size;
		}
		j++;
	}

	blk_size = j - begin_idx;
	if (blk_size > 0) {
		dst_reg = reg_addr[begin_idx];
		// vin_dbg("reg %x %x \n", reg_addr[j-1], reg_addr[j]);
		vin_dbg("i %d j %d bus %d i2c_addr %x reg_addr 0x%02x blk_size %d\n",
			i, j, info->bus_num, info->sensor_addr, dst_reg, blk_size);
		ret = vin_i2c_write_block(info->bus_num, REG_WIDTH_16bit,
					  info->sensor_addr, dst_reg,
					  &data[begin_idx], blk_size);
	}

	return ret;
}

int write_ae_reg(hal_control_info_t *info)
{
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	int port = dev_port2port[info->port];
	int32_t i;
	uint32_t valid_reg_array[BUF_LEN] = {0};

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num,
						   (uint8_t)info->sensor_addr,
						   (uint16_t)OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] clear sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
	}
	/* write ae reg */
	setting_size = sizeof(ae_reg_array_base) / sizeof(ae_reg_array_base[0]) / 2;
	for (i = 0; i < setting_size; i++) {
		valid_reg_array[i] =
			!!(bak_ae_reg_array[info->port][2 * i + 1] - \
			ae_reg_array[info->port][2 * i + 1]);
		vin_dbg("i %d %d %d delta %d", i,
			bak_ae_reg_array[info->port][2 * i + 1],
			ae_reg_array[info->port][2 * i + 1], valid_reg_array[i]);
	}
	ret = ov_write_awb_ae_block_with_valid(info, valid_reg_array,
					       ae_reg_array[info->port], setting_size);
	if (ret < 0) {
		vin_err("%s info->port [%d] ae reg array write failed!\n",
			__func__, info->port);
		return ret;
	}
	memcpy(bak_ae_reg_array[info->port], ae_reg_array[info->port],
	       sizeof(ae_reg_array[info->port]));

	/* read SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			ae_reg_array[info->port], crc_array);
		if (len < 0) {
			vin_err("%s info->port [%d] cam_setting_to_crc failed\n",
				__func__, info->port);
		}
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num,
						   (uint8_t)info->sensor_addr, (uint16_t)OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] read sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
		crc16_ret = cam_crc16(~0, crc_array, len);
		if (ret != crc16_ret) {
			vin_err("%s info->port [%d] crc_reg = 0x%x, crc_cal = 0x%x\n",
				__func__, info->port, ret, crc16_ret);
		}
		ret = (ret == crc16_ret)? 0 : (-1);
	}
	return 0;
}

int write_awb_reg(hal_control_info_t *info)
{
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int crc_last_check[CAM_MAX_NUM];
	int port = dev_port2port[info->port];
	uint32_t valid_reg_array[BUF_LEN] = {0};
	int32_t i;

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num, info->sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] clear sccb crc failed\n", __func__, port);
			return ret;
		}
	}
	/* write awb reg */
	vin_dbg("ovx8b write awb reg begain\n");
	setting_size = sizeof(awb_reg_array_base) / sizeof(uint32_t) / 2;
	for (i = 0; i < setting_size; i++) {
		valid_reg_array[i] =
			!!(awb_reg_array[info->port][i * 2 + 1] -
			bak_awb_reg_array[info->port][i * 2 + 1]);
		vin_dbg("i %d %d %d delta %d", i,
			awb_reg_array[info->port][i * 2 + 1],
			bak_awb_reg_array[info->port][i * 2 + 1],
			valid_reg_array[i]);
	}

	ret = ov_write_awb_ae_block_with_valid(info, valid_reg_array,
					       awb_reg_array[info->port], setting_size);
	if (ret < 0) {
		vin_err("%s info->port [%d] awb reg array write failed!\n",
			__func__, info->port);
		return ret;
	}
	memcpy(bak_awb_reg_array[info->port], awb_reg_array[info->port],
	       sizeof(awb_reg_array[info->port]));
	/* read SCCB CRC */

	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			awb_reg_array[port], crc_array);
		if (len < 0) {
			vin_err("%s port [%d] cam_setting_to_crc failed\n", __func__, port);
		}
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num, info->sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] read sccb crc failed\n", __func__, port);
			return ret;
		}
		crc16_ret = cam_crc16(~0, crc_array, len);
		if (ret != crc16_ret) {
			vin_err("%s port [%d] crc_reg = 0x%x, crc_cal = 0x%x\n",
				__func__, port, ret, crc16_ret);
		}
		ret = (ret == crc16_ret)? 0 : (-1);
		// only call diag when state changes
		if (crc_last_check[port]!= ret) {
		        // camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[port]= ret;
		}
	}
	return 0;
}

/**
 * @brief ratio_interp : calculate interpolated wb color ratio
 *
 * @param [in] ratio1 : wb color ratio1
 * @param [in] ratio2 : wb color ratio2
 * @param [in] ct1 : color temperture1
 * @param [in] ct2 : color temperture2
 * @param [in] ct_cur : current color temperature
 *
 * @return interpolated wb color ratio
 */
float ratio_interp(float ratio1, float ratio2, int32_t ct1, int32_t ct2, int32_t ct_cur) {
	float temp1, temp2, factor, ratio_cur;
	if (ratio2 > ratio1) {
		temp1 = (float)(ct2 - ct1) / (ratio2 - ratio1);
		temp2 = ct_cur - ct1;
		ratio_cur = ratio1 + temp2 / temp1;
	} else {
		temp1 = (float)(ct2 - ct1) / (ratio1 - ratio2);
		temp2 = ct_cur - ct1;
		ratio_cur = ratio1 - temp2 / temp1;
	}
	return ratio_cur;
}

/**
 * @brief set_awb_reg : calculate color temper and calculate awb
 *
 * @param [in] info : sensor info
 * @param [in] rgain : isp rgain
 * @param [in] bgain : isp bgain
 * @param [in] grgain : isp grgain
 * @param [in] gbgain : isp gbgain
 * @param [in] color_temper : isp color temperature
 *
 * @return ret
 */
int32_t set_awb_reg(hal_control_info_t *info, uint32_t rgain, uint32_t bgain,
		uint32_t grgain, uint32_t gbgain, uint32_t color_temper)
{
	int32_t setting_size = 0, i, ret = 0;
	float temp1, temp2, factor;
	float awb_cur_light[2] = {0.0};
	float awb_alight[2] = {0.0};
	float awb_cwf[2] = {0.0};
	float awb_d65[2] = {0.0};
	uint16_t awb_spd_b_gain, awb_spd_r_gain, awb_lpd_r_gain, awb_lpd_b_gain;
	uint16_t awb_spd_gr_gain, awb_lpd_gr_gain;
	float alight_bgain_ratio, alight_rgain_ratio;
	float cwf_bgain_ratio, cwf_rgain_ratio;
	float d65_bgain_ratio, d65_rgain_ratio;
	int32_t alight_temper, cwf_temper, d65_temper;
	float rgain_ratio_lpd, bgain_ratio_lpd, rgain_ratio_spd, bgain_ratio_spd;
	float indi_rgain_ratio_lpd, indi_bgain_ratio_lpd, indi_rgain_ratio_spd, indi_bgain_ratio_spd;

	switch (extra_mode[info->port]) {
	case SUNNY_M24F120D12_S1R8T7E5:
		alight_bgain_ratio = SN120_ALIGHT_BGAIN_RATIO;
		alight_rgain_ratio = SN120_ALIGHT_RGAIN_RATIO;
		cwf_bgain_ratio = SN120_CWF_BGAIN_RATIO;
		cwf_rgain_ratio = SN120_CWF_RGAIN_RATIO;
		d65_bgain_ratio = SN120_D65_BGAIN_RATIO;
		d65_rgain_ratio = SN120_D65_RGAIN_RATIO;
		alight_temper = SN_ALIGHT_TEMPER;
		cwf_temper = SN_CWF_TEMPER;
		d65_temper = SN_D65_TEMPER;
		break;
	case SUNNY_M24F30D12_S1R8T7E5:
		alight_bgain_ratio = SN30_ALIGHT_BGAIN_RATIO;
		alight_rgain_ratio = SN30_ALIGHT_RGAIN_RATIO;
		cwf_bgain_ratio = SN30_CWF_BGAIN_RATIO;
		cwf_rgain_ratio = SN30_CWF_RGAIN_RATIO;
		d65_bgain_ratio = SN30_D65_BGAIN_RATIO;
		d65_rgain_ratio = SN30_D65_RGAIN_RATIO;
		alight_temper = SN_ALIGHT_TEMPER;
		cwf_temper = SN_CWF_TEMPER;
		d65_temper = SN_D65_TEMPER;
		break;
	case LCE_M24F121D12_S1R0T8E5:
		alight_bgain_ratio = LCE120_ALIGHT_BGAIN_RATIO;
		alight_rgain_ratio = LCE120_ALIGHT_RGAIN_RATIO;
		cwf_bgain_ratio = LCE120_CWF_BGAIN_RATIO;
		cwf_rgain_ratio = LCE120_CWF_RGAIN_RATIO;
		d65_bgain_ratio = LCE120_D65_BGAIN_RATIO;
		d65_rgain_ratio = LCE120_D65_RGAIN_RATIO;
		alight_temper = LCE_ALIGHT_TEMPER;
		cwf_temper = LCE_CWF_TEMPER;
		d65_temper = LCE_D65_TEMPER;
		break;
	case LCE_M24F30D12_S1R0T8E5:
		alight_bgain_ratio = LCE30_ALIGHT_BGAIN_RATIO;
		alight_rgain_ratio = LCE30_ALIGHT_RGAIN_RATIO;
		cwf_bgain_ratio = LCE30_CWF_BGAIN_RATIO;
		cwf_rgain_ratio = LCE30_CWF_RGAIN_RATIO;
		d65_bgain_ratio = LCE30_D65_BGAIN_RATIO;
		d65_rgain_ratio = LCE30_D65_RGAIN_RATIO;
		alight_temper = LCE_ALIGHT_TEMPER;
		cwf_temper = LCE_CWF_TEMPER;
		d65_temper = LCE_D65_TEMPER;
		break;
	case SENSING_M24F120D12_S0R0T8E5:
	case SENSING_M24F120D12_S0R0T7E5:
	case (uint32_t)TXD_M24F120D12_S1R8T4E6:
	case OFILM_M24F120D12_S1R8T7E5:
		alight_bgain_ratio = ALIGHT_BGAIN_RATIO;
		alight_rgain_ratio = ALIGHT_RGAIN_RATIO;
		cwf_bgain_ratio = CWF_BGAIN_RATIO;
		cwf_rgain_ratio = CWF_RGAIN_RATIO;
		d65_bgain_ratio = D65_BGAIN_RATIO;
		d65_rgain_ratio = D65_RGAIN_RATIO;
		alight_temper = ALIGHT_TEMPER;
		cwf_temper = CWF_TEMPER;
		d65_temper = D65_TEMPER;
		break;
	default:
		vin_err("extra_mode bit[18:15] = %d is invalid\n",
				extra_mode[info->port]);
		return -1;
	}
	if (pre_awb_disable[info->port]) {
		alight_bgain_ratio = 1;
		alight_rgain_ratio = 1;
		cwf_bgain_ratio = 1;
		cwf_rgain_ratio = 1;
		d65_bgain_ratio = 1;
		d65_rgain_ratio = 1;
	}
	/* lpd spd b/g gain & r /g gain ratio under standard light */
	awb_alight[0] = 1.0 / alight_bgain_ratio;
	awb_alight[1] = 1.0 / alight_rgain_ratio;
	awb_cwf[0] = 1.0 / cwf_bgain_ratio;
	awb_cwf[1] = 1.0 / cwf_rgain_ratio;
	awb_d65[0] = 1.0 / d65_bgain_ratio;
	awb_d65[1] = 1.0 / d65_rgain_ratio;

	awb_lpd_r_gain = rgain << 2;
	awb_lpd_b_gain = bgain << 2;
	/* lpd spd ratio for grgain equal to 1, grgain = gbgain */
	awb_lpd_gr_gain = grgain << 2;
	awb_spd_gr_gain = awb_lpd_gr_gain;

	/* set wb color ratio */
	if (checksum_flag[info->port] &&
		(((extra_mode[info->port]) == SUNNY_M24F120D12_S1R8T7E5) ||
		((extra_mode[info->port]) == SUNNY_M24F120D12_S1R8T7E5) ||
		((extra_mode[info->port]) == LCE_M24F121D12_S1R0T8E5) ||
		((extra_mode[info->port]) == LCE_M24F30D12_S1R0T8E5) ||
		((extra_mode[info->port]) == OFILM_M24F120D12_S1R8T7E5))) {
		if (color_temper <= alight_temper) {
			rgain_ratio_lpd = golden_eeprom_color_ratio[info->port][8];
			bgain_ratio_lpd = golden_eeprom_color_ratio[info->port][9];
			rgain_ratio_spd = golden_eeprom_color_ratio[info->port][10];
			bgain_ratio_spd = golden_eeprom_color_ratio[info->port][11];
			vin_dbg("ct(%d) under alight\n", color_temper);
		} else if (color_temper <= cwf_temper) {
			rgain_ratio_lpd = ratio_interp(golden_eeprom_color_ratio[info->port][8],
				golden_eeprom_color_ratio[info->port][4], alight_temper, cwf_temper, color_temper);
			bgain_ratio_lpd = ratio_interp(golden_eeprom_color_ratio[info->port][9],
				golden_eeprom_color_ratio[info->port][5], alight_temper, cwf_temper, color_temper);
			rgain_ratio_spd = ratio_interp(golden_eeprom_color_ratio[info->port][10],
				golden_eeprom_color_ratio[info->port][6], alight_temper, cwf_temper, color_temper);
			bgain_ratio_spd = ratio_interp(golden_eeprom_color_ratio[info->port][11],
				golden_eeprom_color_ratio[info->port][7], alight_temper, cwf_temper, color_temper);
			vin_dbg("ct(%d) between alight & cwf\n", color_temper);
		} else if (color_temper <= d65_temper) {
			rgain_ratio_lpd = ratio_interp(golden_eeprom_color_ratio[info->port][4],
				golden_eeprom_color_ratio[info->port][0], cwf_temper, d65_temper, color_temper);
			bgain_ratio_lpd = ratio_interp(golden_eeprom_color_ratio[info->port][5],
				golden_eeprom_color_ratio[info->port][1], cwf_temper, d65_temper, color_temper);
			rgain_ratio_spd = ratio_interp(golden_eeprom_color_ratio[info->port][6],
				golden_eeprom_color_ratio[info->port][2], cwf_temper, d65_temper, color_temper);
			bgain_ratio_spd = ratio_interp(golden_eeprom_color_ratio[info->port][7],
				golden_eeprom_color_ratio[info->port][3], cwf_temper, d65_temper, color_temper);
			vin_dbg("ct(%d) between cwf & d65\n", color_temper);
		} else {
			rgain_ratio_lpd = golden_eeprom_color_ratio[info->port][0];
			bgain_ratio_lpd = golden_eeprom_color_ratio[info->port][1];
			rgain_ratio_spd = golden_eeprom_color_ratio[info->port][2];
			bgain_ratio_spd = golden_eeprom_color_ratio[info->port][3];
			vin_dbg("ct(%d) higher d65\n", color_temper);
		}
		indi_rgain_ratio_lpd = eeprom_color_ratio[info->port][4] -
			golden_eeprom_color_ratio[info->port][4] + rgain_ratio_lpd;
		indi_bgain_ratio_lpd = eeprom_color_ratio[info->port][5] -
			golden_eeprom_color_ratio[info->port][5] + bgain_ratio_lpd;
		indi_rgain_ratio_spd = eeprom_color_ratio[info->port][6] -
			golden_eeprom_color_ratio[info->port][6] + rgain_ratio_spd;
		indi_bgain_ratio_spd = eeprom_color_ratio[info->port][7] -
			golden_eeprom_color_ratio[info->port][7] + bgain_ratio_spd;
		vin_dbg("rgain_ratio_lpd(%f)bgain_ratio_lpd(%f)rgain_ratio_spd(%f)bgain_ratio_spd(%f)\n",
			rgain_ratio_lpd, bgain_ratio_lpd, rgain_ratio_spd, bgain_ratio_spd);
		vin_dbg("indi rgain_ratio_lpd(%f)bgain_ratio_lpd(%f)rgain_ratio_spd(%f)bgain_ratio_spd(%f)\n",
			indi_rgain_ratio_lpd, indi_bgain_ratio_lpd, indi_rgain_ratio_spd, indi_bgain_ratio_spd);

		float individual_pd_ratio_rg = indi_rgain_ratio_lpd / indi_rgain_ratio_spd;
		float individual_pd_ratio_bg = indi_bgain_ratio_lpd / indi_bgain_ratio_spd;
		awb_spd_b_gain = awb_lpd_b_gain * individual_pd_ratio_bg;
		awb_spd_r_gain = awb_lpd_r_gain * individual_pd_ratio_rg;
		vin_dbg("awb_lpd_b_gain(0x%x)awb_lpd_r_gain(0x%x)awb_spd_b_gain(0x%x)awb_spd_r_gain(0x%x)\n",
			awb_lpd_b_gain, awb_lpd_r_gain, awb_spd_b_gain, awb_spd_r_gain);
	} else {
		/* interpolating lpd/spd rgain & bgain ratio */
		if (color_temper <= alight_temper) {
			awb_spd_b_gain = awb_lpd_b_gain * awb_alight[0];
			awb_spd_r_gain = awb_lpd_r_gain * awb_alight[1];
			vin_dbg("light temper under alight\n");
		} else if (color_temper <= cwf_temper) {
			for (i = 0; i < 2; i++) {
				if (awb_cwf[i] - awb_alight[i] >= 0) {
					temp1 = (float)(cwf_temper - alight_temper) /
							(awb_cwf[i] - awb_alight[i]);
					temp2 = color_temper - alight_temper;
					awb_cur_light[i] = awb_alight[i] + temp2 / temp1;
				} else {
					temp1 = (float)(cwf_temper - alight_temper) /
							(awb_alight[i] - awb_cwf[i]);
					temp2 = color_temper - alight_temper;
					awb_cur_light[i] = awb_alight[i] - temp2 / temp1;
				}
			}
			awb_spd_b_gain = (awb_lpd_b_gain * awb_cur_light[0])
						+ SHIFT_ROUNDING;
			awb_spd_r_gain = (awb_lpd_r_gain * awb_cur_light[1])
						+ SHIFT_ROUNDING;
			vin_dbg("light temper between alight & cwf\n");
		} else if (color_temper <= d65_temper) {
			for (i = 0; i < 2; i++) {
				if (awb_d65[i] - awb_cwf[i] >= 0) {
					temp1 = (float)(d65_temper - cwf_temper) /
							(awb_d65[i] - awb_cwf[i]);
					temp2 = color_temper - cwf_temper;
					awb_cur_light[i] = awb_cwf[i] + temp2 / temp1;
				} else {
					temp1 = (float)(d65_temper - cwf_temper) /
							(awb_cwf[i] - awb_d65[i]);
					temp2 = color_temper - cwf_temper;
					awb_cur_light[i] = awb_cwf[i] - temp2 / temp1;
				}
			}
			awb_spd_r_gain = (awb_lpd_r_gain * awb_cur_light[1])
						+ SHIFT_ROUNDING;
			awb_spd_b_gain = (awb_lpd_b_gain * awb_cur_light[0])
						+ SHIFT_ROUNDING;
			vin_dbg("light temper between cwf & d65\n");
		} else {
			awb_spd_b_gain = (awb_lpd_b_gain * awb_d65[0])
					+ SHIFT_ROUNDING;
			awb_spd_r_gain = (awb_lpd_r_gain * awb_d65[1])
					+ SHIFT_ROUNDING;
			vin_dbg("light temper higher d65\n");
		}
	}

	/* nomalization when rgain/bgain < 1x */
	if (awb_lpd_r_gain < 1024) {
		factor = (1024.0 / (float)awb_lpd_r_gain);
		awb_lpd_b_gain = awb_lpd_b_gain * factor;
		awb_lpd_gr_gain = awb_lpd_gr_gain * factor;
		awb_lpd_r_gain = 1024;
	}
	if (awb_lpd_b_gain < 1024) {
		factor = (1024.0 / (float)awb_lpd_b_gain);
		awb_lpd_r_gain = awb_lpd_r_gain * factor;
		awb_lpd_gr_gain = awb_lpd_gr_gain * factor;
		awb_lpd_b_gain = 1024;
	}
	if (awb_spd_r_gain < 1024) {
		factor = (1024.0 / (float)awb_spd_r_gain);
		awb_spd_b_gain = awb_spd_b_gain * factor;
		awb_spd_gr_gain = awb_spd_gr_gain * factor;
		awb_spd_r_gain = 1024;
	}
	if (awb_spd_b_gain < 1024) {
		factor = (1024.0 / (float)awb_spd_b_gain);
		awb_spd_r_gain = awb_spd_r_gain * factor;
		awb_spd_gr_gain = awb_spd_gr_gain * factor;
		awb_spd_b_gain = 1024;
	}

	setting_size = sizeof(awb_reg_array_base) / sizeof(uint32_t);
	for (i = 0; i < setting_size; i++) {
		if (awb_reg_array[info->port][i] == OV_AWB_SPD_B) {
			awb_reg_array[info->port][++i] = awb_spd_b_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_spd_b_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_spd_gr_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_spd_gr_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_spd_gr_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_spd_gr_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_spd_r_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_spd_r_gain & 0xff;
		} else {
			awb_reg_array[info->port][++i] = awb_lpd_b_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_lpd_b_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_lpd_gr_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_lpd_gr_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_lpd_gr_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_lpd_gr_gain & 0xff;
			awb_reg_array[info->port][i+=2] = awb_lpd_r_gain >> 8;
			awb_reg_array[info->port][i+=2] = awb_lpd_r_gain & 0xff;
		}
	}

	return 0;
}

/**
 * @brief sensor_awb_cct_control : awb control
 *
 * @param [in] info : sensor info
 * @param [in] mode : mode
 * @param [in] rgain : isp rgain
 * @param [in] bgain : isp bgain
 * @param [in] grgain : unused
 * @param [in] gbgain : unused
 * @param [in] color_temper : isp color temperature
 *
 * @return ret
 */
static int32_t sensor_awb_cct_control(hal_control_info_t *info, uint32_t mode, uint32_t rgain,
		uint32_t bgain, uint32_t grgain, uint32_t gbgain, uint32_t color_temper)
{
	int ret = RET_OK;
	int32_t port = dev_port2port[info->port];
	vin_dbg("rgain = %d, bgain = %d, grgain = %d, gbgain = %d\n",
			rgain, bgain, grgain, gbgain);
	vin_dbg(" color_temper = %d!\n", color_temper);
	if (skip_frame_count[info->port] < FRAME_SKIP_COUNT + 1) {
		skip_frame_count[info->port]++;
		return 0;
	}

	set_awb_reg(info, rgain, bgain, grgain, gbgain, color_temper);
	ret = write_awb_reg(info);
	if (ret < 0) {
		vin_err("port [%d] write_awb_reg failed\n", info->port);
		return ret;
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
	/* enable awb_cct_control and aexp_line_gain_control */
	*enable = (HAL_AWB_CCT_CONTROL & awb_enable) +
              (HAL_AE_LINE_GAIN_CONTROL & ae_enable);
	vin_info("enter userspace_control enalbe = %d\n", *enable);
	return 0;
}

/**
 * @brief set_line_reg : calculate and set line reg value
 *
 * @param [in] info : sensor info
 * @param [in] hcg_line : hcg_line value
 * @param [in] spd_line : spd_line value
 * @param [in] vs_line : vs_line value
 *
 * @return ret
 */
int32_t set_line_reg(hal_control_info_t *info, uint16_t hcg_line, uint16_t spd_line,
		uint16_t vs_line, int32_t *ae_index)
{
	int32_t ret = 0;
	ae_reg_array[info->port][++(*ae_index)] = hcg_line >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = hcg_line & 0xff;
	ae_reg_array[info->port][(*ae_index)+=2] = spd_line >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = spd_line & 0xff;
	ae_reg_array[info->port][(*ae_index)+=2] = vs_line >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = vs_line & 0xff;
	if (*ae_index > sizeof(ae_reg_array) / sizeof(uint32_t) - 1) {
		vin_err("%s ae_index out of range\n", __func__);
		ret = -1;
	}
	return ret;
}

/**
 * @brief set_again_reg : calculate reg again value
 *
 * @param [in] info : sensor info
 * @param [in] hcg_again : hcg_again value
 * @param [in] lcg_again : lcg_again value
 * @param [in] spd_again : spd_again value
 * @param [in] vs_again : vs_again value
 *
 * @return ret
 */
int32_t set_again_reg(hal_control_info_t *info, float hcg_again, float lcg_again,
		float spd_again, float vs_again, int32_t *ae_index)
{
	int32_t ret = 0;
	uint16_t hcg_real_again, lcg_real_again, spd_real_again, vs_real_again;
	hcg_real_again = (uint16_t)(hcg_again * 16);
	hcg_real_again = (hcg_real_again << 4) | ((hcg_real_again & 0xf) << 4);
	lcg_real_again = (uint16_t)(lcg_again * 16);
	lcg_real_again = (lcg_real_again << 4) | ((lcg_real_again & 0xf) << 4);
	spd_real_again = (uint16_t)(spd_again * 16);
	spd_real_again = (spd_real_again << 4) | ((spd_real_again & 0xf) << 4);
	vs_real_again = (uint16_t)(vs_again * 16);
	vs_real_again = (vs_real_again << 4) | ((vs_real_again & 0xf) << 4);

	ae_reg_array[info->port][(*ae_index)+=2] = hcg_real_again >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = hcg_real_again & 0xff;
	ae_reg_array[info->port][(*ae_index)+=2] = spd_real_again >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = spd_real_again & 0xff;
	ae_reg_array[info->port][(*ae_index)+=2] = lcg_real_again >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = lcg_real_again & 0xff;
	ae_reg_array[info->port][(*ae_index)+=2] = vs_real_again >> 8;
	ae_reg_array[info->port][(*ae_index)+=2] = vs_real_again & 0xff;

	if (*ae_index > sizeof(ae_reg_array) / sizeof(uint32_t) - 1) {
		vin_err("%s ae_index out of range\n", __func__);
		ret = -1;
	}
	return ret;
}

/**
 * @brief set_dgain_reg : calculate and set reg dgain value
 *
 * @param [in] info : sensor info
 * @param [in] hcg_dgain : hcg_dgain value
 * @param [in] lcg_dgain : lcg_dgain value
 * @param [in] spd_dgain : spd_dgain value
 * @param [in] vs_dgain : vs_dgain value
 *
 * @return ret
 */
int32_t set_dgain_reg(hal_control_info_t *info, float hcg_dgain, float lcg_dgain,
		float spd_dgain, float vs_dgain, int32_t *ae_index)
{
	int32_t ret = 0;
	uint16_t hcg_real_dgain, lcg_real_dgain, spd_real_dgain, vs_real_dgain;
	hcg_real_dgain = (uint16_t)(hcg_dgain * 1024);
	lcg_real_dgain = (uint16_t)(lcg_dgain * 1024);
	spd_real_dgain = (uint16_t)(spd_dgain * 1024);
	vs_real_dgain = (uint16_t)(vs_dgain * 1024);

	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(hcg_real_dgain >> 10);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(hcg_real_dgain >> 2);
	ae_reg_array[info->port][(*ae_index)+=2] =
		(uint8_t)((hcg_real_dgain & 0x0003) << 6);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(spd_real_dgain >> 10);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(spd_real_dgain >> 2);
	ae_reg_array[info->port][(*ae_index)+=2] =
		(uint8_t)((spd_real_dgain & 0x0003) << 6);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(lcg_real_dgain >> 10);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(lcg_real_dgain >> 2);
	ae_reg_array[info->port][(*ae_index)+=2] =
		(uint8_t)((lcg_real_dgain & 0x0003) << 6);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(vs_real_dgain >> 10);
	ae_reg_array[info->port][(*ae_index)+=2] = (uint8_t)(vs_real_dgain >> 2);
	ae_reg_array[info->port][(*ae_index)+=2] =
		(uint8_t)((vs_real_dgain & 0x0003) << 6);

	if (*ae_index > sizeof(ae_reg_array) / sizeof(uint32_t) - 1) {
		vin_err("%s ae_index out of range\n", __func__);
		ret = -1;
	}

	return ret;
}

static int32_t get_sensor_ratio_from_otp(hal_control_info_t *info)
{
		int32_t hcg_sensitivity1, hcg_sensitivity2;
	int32_t lcg_sensitivity1, lcg_sensitivity2;
	int32_t spd_sensitivity1, spd_sensitivity2;
	int32_t ovx8b_version_0, ovx8b_version_1;
	int32_t ret = 0;
	// get ovx8b_version
	ovx8b_version_0 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_OTP_VERSION_REG_1);
	if (ovx8b_version_0 < 0) {
		vin_err("read ovx8b_version 0x%x fail!!!\n", OVX8B_OTP_VERSION_REG_1);
		return -1;
	}
	ovx8b_version_1 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_OTP_VERSION_REG_2);
	if (ovx8b_version_1 < 0) {
		vin_err("read ovx8b_version 0x%x fail!!!\n", OVX8B_OTP_VERSION_REG_2);
		return -1;
	}
	vin_info("version 0x%x=0x%x, 0x%x=0x%x\n",
		OVX8B_OTP_VERSION_REG_1, ovx8b_version_0, OVX8B_OTP_VERSION_REG_2, ovx8b_version_1);

	int32_t otp_hcg = hb_vin_i2c_read_reg16_data16(info->bus_num,
			info->sensor_addr, OVX8B_OTP_HCG_SENS_REG);
	if (otp_hcg < 0) {
		vin_err("read 0x%x failed\n", OVX8B_OTP_HCG_SENS_REG);
		return -1;
	}
	int32_t otp_lcg = hb_vin_i2c_read_reg16_data16(info->bus_num,
			info->sensor_addr, OVX8B_OTP_LCG_SENS_REG);
	if (otp_lcg < 0) {
		vin_err("read 0x%x failed\n", OVX8B_OTP_LCG_SENS_REG);
		return -1;
	}
	int32_t otp_spd = hb_vin_i2c_read_reg16_data16(info->bus_num,
			info->sensor_addr, OVX8B_OTP_SPD_SENS_REG);
	if (otp_spd < 0) {
		vin_err("read 0x%x failed\n", OVX8B_OTP_SPD_SENS_REG);
		return -1;
	}
	int32_t otp_vs = hb_vin_i2c_read_reg16_data16(info->bus_num,
			info->sensor_addr, OVX8B_OTP_VS_SENS_REG);
	if (otp_vs < 0) {
		vin_err("read 0x%x failed\n", OVX8B_OTP_VS_SENS_REG);
		return -1;
	}
	vin_info("otp 0x%x=0x%x, 0x%x=0x%x, 0x%x=0x%x, 0x%x=0x%x\n",
		OVX8B_OTP_HCG_SENS_REG, otp_hcg, OVX8B_OTP_LCG_SENS_REG, otp_lcg,
		OVX8B_OTP_SPD_SENS_REG, otp_spd, OVX8B_OTP_VS_SENS_REG, otp_vs);
	if (ovx8b_version_0 == OVX8B_OTP_VERSION_THRESHOLD_1) {
		if (ovx8b_version_1 < OVX8B_OTP_VERSION_THRESHOLD_2) {
			// a. if 0x704b == 0xa5 && 0x700e < 0x06
			uint32_t temp = otp_spd * otp_hcg / otp_lcg;
			ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
				OVX8B_SPD_SENSITIVITY_REG, temp);
			if (ret < 0) {
				vin_err("write 0x%x=0x%x failed\n", OVX8B_SPD_SENSITIVITY_REG, temp);
				return ret;
			}
		}
	} else if (ovx8b_version_1 >= OVX8B_OTP_VERSION_THRESHOLD_2) {
		// b. if 0x704b != 0xa5 && 0x700e >= 0x06
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_HCG_SENSITIVITY_REG, otp_hcg);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_HCG_SENSITIVITY_REG, otp_hcg);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_LCG_SENSITIVITY_REG, otp_lcg);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_LCG_SENSITIVITY_REG, otp_lcg);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_SPD_SENSITIVITY_REG, otp_spd);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_SPD_SENSITIVITY_REG, otp_spd);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_VS_SENSITIVITY_REG, otp_vs);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_VS_SENSITIVITY_REG, otp_vs);
			return ret;
		}
	} else {
		// b. if 0x704b != 0xa5 && 0x700e < 0x06
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_HCG_SENSITIVITY_REG, otp_hcg);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_HCG_SENSITIVITY_REG, otp_hcg);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_LCG_SENSITIVITY_REG, otp_lcg);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_LCG_SENSITIVITY_REG, otp_lcg);
			return ret;
		}
		uint32_t temp = otp_spd * otp_hcg / otp_lcg;
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_SPD_SENSITIVITY_REG, temp);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_SPD_SENSITIVITY_REG, temp);
			return ret;
		}
		ret = hb_vin_i2c_write_reg16_data16(info->bus_num, info->sensor_addr,
			OVX8B_VS_SENSITIVITY_REG, otp_vs);
		if (ret < 0) {
			vin_err("write 0x%x=0x%x failed\n", OVX8B_VS_SENSITIVITY_REG, otp_vs);
			return ret;
		}
	}
	hcg_sensitivity1 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_HCG_SENSITIVITY_REG);
	if (hcg_sensitivity1 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}
	hcg_sensitivity2 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_HCG_SENSITIVITY_REG + 1);
	if (hcg_sensitivity2 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}
	lcg_sensitivity1 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_LCG_SENSITIVITY_REG);
	if (lcg_sensitivity1 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}
	lcg_sensitivity2 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_LCG_SENSITIVITY_REG + 1);
	if (lcg_sensitivity2 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}
	spd_sensitivity1 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_SPD_SENSITIVITY_REG);
	if (spd_sensitivity1 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}
	spd_sensitivity2 = hb_vin_i2c_read_reg16_data8(info->bus_num,
			info->sensor_addr, OVX8B_SPD_SENSITIVITY_REG + 1);
	if (spd_sensitivity2 < 0) {
		vin_err("read cg_gain from otp fail!!!\n");
		return -1;
	}

	vin_dbg("hcg_sensitivity1_reg 0x%x, hcg_sensitivity1 = 0x%x\n",
			OVX8B_HCG_SENSITIVITY_REG, hcg_sensitivity1);
	vin_dbg("hcg_sensitivity2_reg 0x%x, hcg_sensitivity2 = 0x%x\n",
			OVX8B_HCG_SENSITIVITY_REG + 1, hcg_sensitivity2);
	vin_dbg("lcg_sensitivity1_reg 0x%x, lcg_sensitivity1 = 0x%x\n",
			OVX8B_LCG_SENSITIVITY_REG, lcg_sensitivity1);
	vin_dbg("lcg_sensitivity2_reg 0x%x, lcg_sensitivity2 = 0x%x\n",
			OVX8B_LCG_SENSITIVITY_REG + 1, lcg_sensitivity2);
	vin_dbg("spd_sensitivity1_reg 0x%x, spd_sensitivity1 = 0x%x\n",
			OVX8B_SPD_SENSITIVITY_REG, spd_sensitivity1);
	vin_dbg("spd_sensitivity2_reg 0x%x, spd_sensitivity2 = 0x%x\n",
			OVX8B_SPD_SENSITIVITY_REG + 1, spd_sensitivity2);

	hcg_lcg_cg_ratio[info->port] =
		(float)((hcg_sensitivity1 * OVX8B_RATIO_SHIFT) +
				hcg_sensitivity2) /
		(float)((lcg_sensitivity1 * OVX8B_RATIO_SHIFT) +
				lcg_sensitivity2);
	sensitivity_ratio[info->port] =
		(float)((lcg_sensitivity1 * OVX8B_RATIO_SHIFT) +
				lcg_sensitivity2) /
		(float)((spd_sensitivity1 * OVX8B_RATIO_SHIFT) +
				spd_sensitivity2);
	if ((hcg_lcg_cg_ratio[info->port] < 1.0) ||
		(sensitivity_ratio[info->port] < 1.0)) {
		vin_warn("the sensitivity_ratio/sensitivity_ratio"
				"should should be greater than 1,"
				"please check sensor status\n");
		hcg_lcg_cg_ratio[info->port] = OVX8B_HCG_LCG_CG_RATIO_DEFT;
		sensitivity_ratio[info->port] = OVX8B_SENSITIVITY_RATIO_DEFT;
	}
	vin_dbg("hcg_lcg_cg_ratio = %f, sensitivity_ratio = %f \n",
			hcg_lcg_cg_ratio[info->port],
			sensitivity_ratio[info->port]);


	return 0;
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
	int32_t ret = 0;
	uint32_t line_tmp = 0;
	int32_t port = dev_port2port[info->port];
	line_gain_control_t hcg, lcg, spd, vs;
	int32_t ae_index = 0;

	vin_dbg(" gain mode %d, --line %d, again %d, dgain %d \n",
			mode, line[0], again[0], dgain[0]);

	if (skip_frame_count[info->port] < FRAME_SKIP_COUNT) {
		skip_frame_count[info->port]++;
		return ret;
	}

	if((again[0] == again_tmp_buf[info->port]) &&
			(dgain[0] == dgain_tmp_buf[info->port]) &&
			(line[0] == line_tmp_buf[info->port]))
		return 0;

	again_tmp_buf[info->port] = again[0];
	dgain_tmp_buf[info->port] = dgain[0];
	line_tmp_buf[info->port] = line[0];

	/* get the ae ratio only once, then ratio > 1.0 is confirmed */
	if ((hcg_lcg_cg_ratio[info->port] < 1.0) &&
		sensitivity_ratio[info->port] < 1.0) {
		ret = get_sensor_ratio_from_otp(info);
		if (ret < 0) {
			vin_err("get sensor ovx8b ratio from otp fail\n");
			return ret;
		}
	}
	/* calculate exposure value = line * 2^(gain/32) */
	if (config_index[info->port] & PWL_24BIT) {
		lcg.gain = pow(2, ((float)(again[0] + dgain[0]) / 32));
		lcg.exp_value = line[0] * lcg.gain;
		hcg.exp_value = lcg.exp_value * OVX8B_HCG_LCG_CHANNEL_RATIO /
						hcg_lcg_cg_ratio[info->port];
		spd.exp_value = lcg.exp_value * sensitivity_ratio[info->port] /
						OVX8B_LCG_SPD_CHANNEL_RATIO;
		vs.exp_value = lcg.exp_value / OVX8B_LCG_VS_CHANNEL_RATIO;

		/* calculate hcg line & gain */
		hcg.line = line[0];
		COMPARE_AND_ASSIGN(hcg.line, DCG_SPD_LINE_MIN,
		dcg_add_vs_line_max[info->port] - VS_LINE_MIN);

		/* calculate lcg gain */
		lcg.again = lcg.gain / LCG_VS_DGAIN_MIN;
		COMPARE_AND_ASSIGN(lcg.again, LCG_VS_AGAIN_MIN, AGAIN_MAX);
		lcg.dgain = lcg.gain / lcg.again;
		COMPARE_AND_ASSIGN(lcg.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);

		/* calculate hcg gain */
		hcg.gain = hcg.exp_value / (float)hcg.line;
		hcg.again = hcg.gain;
		COMPARE_AND_ASSIGN(hcg.again, HCG_SPD_AGAIN_MIN, AGAIN_MAX);
		hcg.dgain = hcg.gain / hcg.again;
		COMPARE_AND_ASSIGN(hcg.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);
	} else {
		hcg.gain = pow(2, ((float)(again[0] + dgain[0])/32));
		hcg.exp_value = line[0] * hcg.gain;
		lcg.exp_value = hcg.exp_value * hcg_lcg_cg_ratio[info->port] /
						OVX8B_HCG_LCG_CHANNEL_RATIO;
		spd.exp_value = lcg.exp_value * sensitivity_ratio[info->port] /
						OVX8B_LCG_SPD_CHANNEL_RATIO;
		vs.exp_value = lcg.exp_value / OVX8B_LCG_VS_CHANNEL_RATIO;

		/* calculate hcg line & gain */
		hcg.line = line[0];
		COMPARE_AND_ASSIGN(hcg.line, DCG_SPD_LINE_MIN,
		dcg_add_vs_line_max[info->port] - VS_LINE_MIN);

		/* calculate hcg gain */
		hcg.again = hcg.gain;
		COMPARE_AND_ASSIGN(hcg.again, HCG_SPD_AGAIN_MIN, AGAIN_MAX);
		hcg.dgain = hcg.gain / hcg.again;
		COMPARE_AND_ASSIGN(hcg.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);

		/* calculate lcg gain */
		lcg.gain = lcg.exp_value / (float)hcg.line;
		lcg.again = lcg.gain / LCG_VS_DGAIN_MIN;
		COMPARE_AND_ASSIGN(lcg.again, LCG_VS_AGAIN_MIN, AGAIN_MAX);
		lcg.dgain = lcg.gain / lcg.again;
		COMPARE_AND_ASSIGN(lcg.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);
	}

	/* calculate spd line & gain */
	line_tmp = spd.exp_value / (HCG_SPD_AGAIN_MIN * DGAIN_MIN);
	if (line_tmp >= hcg.line)
		spd.line = hcg.line;
	else
		spd.line = (uint16_t)line_tmp;
	if (spd.line <= DCG_SPD_LINE_MIN)
		spd.line = DCG_SPD_LINE_MIN;
	spd.gain = spd.exp_value / (float)spd.line;
	spd.again = spd.gain / DGAIN_MIN;
	COMPARE_AND_ASSIGN(spd.again, HCG_SPD_AGAIN_MIN, AGAIN_MAX);
	spd.dgain = spd.gain / spd.again;
	COMPARE_AND_ASSIGN(spd.dgain, DGAIN_MIN, DGAIN_MAX);

	/* calculate vs line & gain*/
	/* exp_value control */
	vs.line = vs.exp_value / (LCG_VS_AGAIN_MIN * LCG_VS_DGAIN_MIN) - 0.5;

	COMPARE_AND_ASSIGN(vs.line, VS_LINE_MIN, VS_LINE_MAX);
	// Max(DCG_exp + VS_exp, SPD_exp) <= VTS - 13
	if (vs.line >= dcg_add_vs_line_max[info->port] - hcg.line)
		vs.line = dcg_add_vs_line_max[info->port] - hcg.line;
	/* line gain control */
	if (vs.line == VS_LINE_MIN)
		vs.gain = vs.exp_value;
	else
		vs.gain = vs.exp_value / (float)vs.line;
	vs.again = vs.gain / LCG_VS_DGAIN_MIN;
	COMPARE_AND_ASSIGN(vs.again, LCG_VS_AGAIN_MIN, AGAIN_MAX);

	vs.dgain = vs.gain / vs.again;
	COMPARE_AND_ASSIGN(vs.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);

	ret = set_line_reg(info, hcg.line, spd.line, vs.line, &ae_index);
	if (ret < 0) {
		vin_err("port [%d] set_line_reg fail\n", info->port);
		return ret;
	}
	ret = set_again_reg(info, hcg.again, lcg.again, spd.again, vs.again, &ae_index);
	if (ret < 0) {
		vin_err("port [%d] set_again_reg fail\n", info->port);
		return ret;
	}
	ret = set_dgain_reg(info, hcg.dgain, lcg.dgain, spd.dgain, vs.dgain, &ae_index);
	if (ret < 0) {
		vin_err("port [%d] set_dgain_reg fail\n", info->port);
		return ret;
	}

	ret = write_ae_reg(info);
	if (ret < 0) {
		vin_err("port [%d] write_awb_reg failed\n", info->port);
		return ret;
	}

	return ret;
}

static int32_t sensor_start_control(hal_control_info_t *info)
{
	int32_t ret = RET_OK;
	int32_t port = dev_port2port[info->port];

	if (((diag_mask_u)diag_mask[port]).sensor_group_hold_off == 0) {
		ret = group_hold_start(info);
		if (ret < 0) {
			vin_err("port [%d] group_hold_start failed\n", port);
			return ret;
		}
	}
	return ret;
}

static int32_t sensor_end_control(hal_control_info_t *info)
{
	int32_t ret = RET_OK;
	int32_t port = dev_port2port[info->port];

	if (((diag_mask_u)diag_mask[port]).sensor_group_hold_off == 0) {
		ret = group_hold_end(info);
		if (ret < 0) {
			vin_err("port [%d] group_hold_end failed\n", port);
			return ret;
		}
	}
	return ret;
}

int f_sensor_init_global_data(sensor_info_t *sensor_info)
{
	int32_t ret = 0;
	char file_buff[256] = {0};
	snprintf(file_buff, sizeof(file_buff), "/sensor_status_%d", sensor_info->port);
	if (g_sensor_sts_fd[sensor_info->port] > 0)
		return 0;
	g_sensor_sts_fd[sensor_info->port] = shm_open(file_buff,
		O_RDWR | O_CREAT | O_EXCL, S_IRUSR | S_IWUSR);
	if (g_sensor_sts_fd[sensor_info->port] == -1) {
		usleep(50);
		g_sensor_sts_fd[sensor_info->port] =
			shm_open(file_buff, O_RDWR, S_IRUSR | S_IWUSR);
		g_sensor_sts[sensor_info->port] = mmap(0, sizeof(sensor_status_info_t),
			PROT_READ | PROT_WRITE, MAP_SHARED, g_sensor_sts_fd[sensor_info->port], 0);
	} else {
		/* set the size */
		ret = ftruncate(g_sensor_sts_fd[sensor_info->port], sizeof(sensor_status_info_t));
		g_sensor_sts[sensor_info->port] = mmap(0, sizeof(sensor_status_info_t),
			PROT_READ | PROT_WRITE, MAP_SHARED, g_sensor_sts_fd[sensor_info->port], 0);
		memset(g_sensor_sts[sensor_info->port], 0,
			sizeof(g_sensor_sts[sensor_info->port]));
	}
	return ret;
}

int f_sensor_deinit_global_data(sensor_info_t *sensor_info)
{
	int ret;
	char file_buff[256] = {0};
	snprintf(file_buff, sizeof(file_buff), "/sensor_status_%d", sensor_info->port);
	ret = munmap(g_sensor_sts[sensor_info->port], sizeof(sensor_status_info_t));
	g_sensor_sts[sensor_info->port] = NULL;
	if (ret) {
		vin_err("sensor_status_t munmap error\n");
		return -1;
	}
	ret = shm_unlink(file_buff);
	if (ret) {
		vin_err("sensor_status_t unlink error\n");
		return -2;
	}
	if (g_sensor_sts_fd[sensor_info->port] > 0) {
		close(g_sensor_sts_fd[sensor_info->port]);
		g_sensor_sts_fd[sensor_info->port] = -1;
	}
	return 0;
}

#ifdef CAM_DIAG
static int32_t camera_ov_diag_temperature(diag_node_info_t *node)
{
	int32_t ret = 0;
	int32_t fault = 0;
	uint32_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t dev_addr = node->diag_info.t.reg.reg_dev;
	uint16_t reg_addr = node->diag_info.t.reg.reg_addr;
	uint32_t port = DIAG_SNR_PORT(node->diag_id);
	int32_t value = 0;
	float temp = 0.0;

	ret = hb_vin_i2c_read_reg16_data16(bus, dev_addr, reg_addr);
	if (ret < 0) {
		vin_err("ov senor read temperature reg error\n");
		return ret;
	}
	value = (ret <= 0xC000) ? ret : (-1) * (ret - 0xC000);
	value = (value * OV_TEMP_RATIO) >> 8;
	temp = value / 1000.0;
	vin_dbg("Temp monitor port:%d temp = %f \n", port, temp);
	if (temp > node->diag_info.t.reg.reg_max ||
		temp < node->diag_info.t.reg.reg_min) {
		fault = 1;
		vin_warn("ov sensor port: %d temp: %f out of range(%d,%d)\n",
			port, temp, node->diag_info.t.reg.reg_min, node->diag_info.t.reg.reg_max);
	} else {
		fault = 0;
	}

	return fault;
}

static int32_t ov_sensor_diag_fault_inject(diag_node_info_t *node, int32_t inject)
{
	int32_t ret = 0;
	uint8_t val = 0;
	uint32_t bus = node->diag_info.t.reg.reg_bus;
	uint8_t dev_addr = node->diag_info.t.reg.reg_dev;
	uint16_t diag_type = DIAG_SUB_ID(node->diag_id);

	vin_info("bus:%d dev_addr:0x%02x diag_id:0x%x diag_type:0x%02x fault inject:%d\n",
			bus, dev_addr, node->diag_id, diag_type, inject);

	switch (diag_type) {
	case CAMERA_OV_ERRB_ERROR:
	case CAMERA_OV_COLUMN_ID_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, COLUMN_ROW_FAULT_REG);
		val = inject ? val & (~BIT(0)) : val | BIT(0);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, COLUMN_ROW_FAULT_REG, val);
		vin_info("column id fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, COLUMN_ROW_FAULT_REG, val);
		break;
	case CAMERA_OV_ROW_ID_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, COLUMN_ROW_FAULT_REG);
		val = inject ? val | BIT(1) : val & (~BIT(1));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, COLUMN_ROW_FAULT_REG, val);
		vin_info("row id fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, COLUMN_ROW_FAULT_REG, val);
		break;
	case CAMERA_OV_ONLINE_PIXEL_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, ONLINE_PIXEL_ALLOW_INJECT);
		val &= (~BIT(3));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, ONLINE_PIXEL_ALLOW_INJECT, val);
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, ONLINE_PIXEL_FAULT_REG);
		val = inject ? val | BIT(2) : val & (~BIT(2));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, ONLINE_PIXEL_FAULT_REG, val);
		vin_info("online pixel fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, ONLINE_PIXEL_FAULT_REG, val);
		break;
	case CAMERA_OV_PLL_CLOCK_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, PLL_CLOCK_ALLOW_INJECT);
		val &= (~BIT(1));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, PLL_CLOCK_ALLOW_INJECT, val);
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, PLL_CLOCK_FAULT_REG);
		val = inject ? val & (~BIT(6)) : val | BIT(6);
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, PLL_CLOCK_FAULT_REG, val);
		vin_info("pll clock fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, PLL_CLOCK_FAULT_REG, val);
		break;
	case CAMERA_OV_RAM_CRC_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, RAM_CRC_ALLOW_INJECT);
		val &= (~BIT(3));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, RAM_CRC_ALLOW_INJECT, val);
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, RAM_CRC_FAULT_REG);
		val = inject ? val | BIT(0) : val & (~BIT(0));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, RAM_CRC_FAULT_REG, val);
		vin_info("ram crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, RAM_CRC_FAULT_REG, val);
		break;
	case CAMERA_OV_ROM_CRC_ERROR:
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, ROM_CRC_ALLOW_INJECT);
		val &= (~BIT(2));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, ROM_CRC_ALLOW_INJECT, val);
		val = hb_vin_i2c_read_reg16_data8(bus, dev_addr, ROM_CRC_FAULT_REG);
		val = inject ? val | BIT(0) : val & (~BIT(0));
		ret = hb_vin_i2c_write_reg16_data8(bus, dev_addr, ROM_CRC_FAULT_REG, val);
		vin_info("rom crc fault inject:%d, reg_addr:0x%04x, val:0x%02x\n",
				inject, ROM_CRC_FAULT_REG, val);
		break;
	case CAMERA_OV_AVDD_ERROR:
	case CAMERA_OV_DOVDD_ERROR:
	case CAMERA_OV_DVDD_ERROR:
	case CAMERA_OV_TEMP_ERROR:
		node->diag_info.t.reg.reg_max = inject ? 0 : node->diag_info.t.reg.reg_test;
		vin_info("vol or temp fault inject:%d, reg_max:0x%04x\n",
				inject, node->diag_info.t.reg.reg_max);
		break;
	default:
		vin_err("fault inject diag type is not mismatch\n");
		return -1;
	}
	if (ret < 0) {
		vin_err("bus:%d dev_addr:0x%02x fault inject: %d fail\n",
				bus, dev_addr, inject);
		return ret;
	}

	return 0;
}

int32_t sensor_diag_nodes_init(sensor_info_t *sensor_info)
{
	int32_t ret = 0;
	uint32_t port_id = 0;
	diag_node_info_t *diag_node = NULL;
	diag_node_info_t *sub_node = NULL;
	diag_node = cam_diag_get_nodes(5);
	sub_node = cam_diag_get_nodes(6);

	if (diag_node == NULL || sub_node == NULL) {
		vin_err("cam_diag_get_nodes failed\n");
		return -RET_ERROR;
	}
	vin_info("diag_nodes info port: %d, name: %s, bus: %d, addr: 0x%x, errb: %d, gpio_type: %d\n",
			sensor_info->dev_port, sensor_info->sensor_name, sensor_info->bus_num,
			sensor_info->sensor_addr, sensor_info->sensor_errb, CAM_GPIO_TYPE(sensor_info->sensor_errb));

	port_id = sensor_info->dev_port + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ERRB_CHECK, CAMERA_OV_ERRB_ERROR);
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
	diag_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROW_COLUMN_ID, CAMERA_OV_COLUMN_ID_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_1_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x80;
	sub_node->diag_info.t.reg.reg_active = 0x80;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROW_COLUMN_ID, CAMERA_OV_ROW_ID_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_2_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x01;
	sub_node->diag_info.t.reg.reg_active = 0x01;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_PLL_CLOCK, CAMERA_OV_PLL_CLOCK_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_0_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x10;
	sub_node->diag_info.t.reg.reg_active = 0x10;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_RAM_CRC, CAMERA_OV_RAM_CRC_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_2_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x02;
	sub_node->diag_info.t.reg.reg_active = 0x02;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ROM_CRC, CAMERA_OV_ROM_CRC_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_0_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x20;
	sub_node->diag_info.t.reg.reg_active = 0x20;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	sub_node = sub_node + 1;
	sub_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_ONLINE_PIXEL, CAMERA_OV_ONLINE_PIXEL_ERROR);
	sub_node->diag_type = CAM_DIAG_REG;
	sub_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN | DIAG_MON_DELAY;
	sub_node->diag_status = 0;
	sub_node->port_mask = vin_port_mask_of_snr(sensor_info);
	sub_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
	sub_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	sub_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	sub_node->diag_info.t.reg.reg_addr = OV_FAULT_0_REG;
	sub_node->diag_info.t.reg.reg_mask = 0x02;
	sub_node->diag_info.t.reg.reg_active = 0x02;
	sub_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_add_subnode(diag_node, sub_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_OV_AVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = OV_AVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = AVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_max = AVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_min = AVDD_MIN_VOL_VALUE;
	diag_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_OV_DOVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = OV_DOVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = DOVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_max = DOVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_min = DOVDD_MIN_VOL_VALUE;
	diag_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_VOLTAGE, CAMERA_OV_DVDD_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = OV_DVDD_VOLTAGE_REG;
	diag_node->diag_info.t.reg.reg_test = DVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_max = DVDD_MAX_VOL_VALUE;
	diag_node->diag_info.t.reg.reg_min = DVDD_MIN_VOL_VALUE;
	diag_node->test_fault_inject  = ov_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	diag_node = diag_node + 1;
	diag_node->diag_id = CAM_SNR_DIAG_ID(port_id, SNR_TEMP, CAMERA_OV_TEMP_ERROR);
	diag_node->diag_type = CAM_DIAG_REG;
	diag_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
	diag_node->diag_status = 0;
	diag_node->port_mask = vin_port_mask_of_snr(sensor_info);
	diag_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(VALUE_TYPE, REG16_VAL16);
	diag_node->diag_info.t.reg.reg_bus = sensor_info->bus_num;
	diag_node->diag_info.t.reg.reg_dev = sensor_info->sensor_addr;
	diag_node->diag_info.t.reg.reg_addr = OV_AVER_TEMP_REG;
	diag_node->diag_info.t.reg.reg_test = OV_MAX_TEMP_VALUE;
	diag_node->diag_info.t.reg.reg_max = OV_MAX_TEMP_VALUE;
	diag_node->diag_info.t.reg.reg_min = OV_MIN_TEMP_VALUE;
	diag_node->fault_judging = camera_ov_diag_temperature;
	diag_node->test_fault_inject = ov_sensor_diag_fault_inject;
	cam_diag_node_register(diag_node);

	return 0;
}
#endif

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(ovx8bstd, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D16);
sensor_module_t ovx8bstd = {
	.module = SENSOR_MNAME(ovx8bstd),
#else
sensor_module_t ovx8bstd = {
	.module = "ovx8bstd",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.get_sns_params = get_sns_info,
	.start_control = sensor_start_control,
	.end_control = sensor_end_control,
	.awb_cct_control = sensor_awb_cct_control,
	.aexp_line_gain_control = sensor_aexp_line_gain_control,
	.userspace_control = sensor_userspace_control,
	// .get_sns_status = sensor_get_status,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
#ifdef CAM_DIAG
	.diag_nodes_init = sensor_diag_nodes_init,
#endif
};
