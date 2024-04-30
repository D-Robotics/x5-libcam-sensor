/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ovx3cstd]:" fmt

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
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <pthread.h>

#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ovx3cstd_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ov_common_setting.h"
// #include "inc/ds960_setting.h"
// #include "inc/ds954_setting.h"
// #include "inc/ds953_setting.h"

#include "../../inc/cam_common.h"
#include "inc/sensorstd_common.h"
#include "../serial/max_serial.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define VERSION_SENSING     "1.0.0"
#define VERSION_OF_OVX3C    "2.1.0"
#define ALIGN_4(num)	(((uint32_t)(num) + 0x03U) & (0xFFFFFFFcU))

#define FPS_HTS
#define BUF_LEN  128
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)

#define EEPROM_I2C_ADDR_ALIAS_ID        (0x51)
#define FRAME_SKIP_COUNT    1u

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
	[SENSING_GM27F216D12_S0R0T8E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[GALAXY_GM24F100D12_S2R5T8E3] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_GM24F100D12_S2R5T8E3] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SUNNY_GM24F60D12_S2R5T8E3] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[LCE_GM24F103D12_S2T0E6] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x57,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[SENSING_GM24F106D12_S0R0T8E5] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[OFILM_GM24F106D12_S0R3T0E6] = {
		.serial_addr = 0x42,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 1,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[OFILM_GM24F106D12_S2R3T0E6] = {
		.serial_addr = 0x42,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x50,		// eeprom i2c addr
		.serial_rclk_out = 1,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 4,          // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
	[LCE_GM24F60D12_S2T0E6] = {
		.serial_addr = 0x40,		// serial i2c addr
		.sensor_addr = 0x36,		// sensor i2c addr
		.eeprom_addr = 0x57,		// eeprom i2c addr
		.serial_rclk_out = 0,		// 0: serial rclk disabl, 1: serial_rclk enable
		.rclk_mfp = 0,                // if serial_rclk_out = 1, the rclk output on rclk_mfp
	},
};

static const sensor_emode_type_t sensor_emode[MODE_TYPE_NUM] = {
	SENSOR_EMADD(SENSING_GM27F216D12_S0R0T8E5, "0.0.1", "lib_x3cRGGB_Pwl12_SY_Fov106.so", "0.22.12.15", &emode_data[SENSING_GM27F216D12_S0R0T8E5]),
	SENSOR_EMADD(GALAXY_GM24F100D12_S2R5T8E3, "0.0.1", "lib_COX3CGB_pwl12_OF_Fov99.so", "0.22.12.15", &emode_data[GALAXY_GM24F100D12_S2R5T8E3]),
	SENSOR_EMADD(SUNNY_GM24F100D12_S2R5T8E3, "0.0.1", "lib_COX3CGB_pwl12_OF_Fov99.so", "0.22.12.15", &emode_data[SUNNY_GM24F100D12_S2R5T8E3]),
	SENSOR_EMADD(SUNNY_GM24F60D12_S2R5T8E3, "0.0.1", "lib_CW-OX3GB-O060+038-L.so", "0.22.8.23", &emode_data[SUNNY_GM24F60D12_S2R5T8E3]),
	SENSOR_EMADD(LCE_GM24F103D12_S2T0E6, "0.0.1", "lib_CL-OX3GB-L103+067-L.so", "0.23.3.17",
				&emode_data[LCE_GM24F103D12_S2T0E6]),
	SENSOR_EMADD(SENSING_GM24F106D12_S0R0T8E5, "0.0.1", "lib_x3cRGGB_Pwl12_SY_Fov106.so", "0.22.12.15", &emode_data[SENSING_GM24F106D12_S0R0T8E5]),
	SENSOR_EMADD(OFILM_GM24F106D12_S0R3T0E6, "0.0.1", "lib_COX3CGB_pwl12_OF_Fov99.so", "0.22.12.15",
				&emode_data[OFILM_GM24F106D12_S0R3T0E6]),
	SENSOR_EMADD(OFILM_GM24F106D12_S2R3T0E6, "0.0.1", "lib_CH-OX3GB-O100+065-L.so", "0.23.7.24",
				&emode_data[OFILM_GM24F106D12_S2R3T0E6]),
	SENSOR_EMADD(LCE_GM24F60D12_S2T0E6, "0.0.1", "lib_CL-OX3GB-L060+039-L.so", "0.23.3.17",
				&emode_data[LCE_GM24F60D12_S2T0E6]),
	SENSOR_EMEND()
};

static int32_t sensor_config_index_ae_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_awb_disable(sensor_info_t *sensor_info);
static int32_t sensor_config_index_test_pattern(sensor_info_t *sensor_info);
static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info);
static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info);
static int32_t sensor_config_index_fps_div(sensor_info_t *sensor_info);

static SENSOR_CONFIG_FUNC sensor_config_index_funcs[B_CONFIG_INDEX_MAX] = {
	[B_AE_DISABLE] = sensor_config_index_ae_disable,
	[B_AWB_DISABLE] = sensor_config_index_awb_disable,
	[B_TEST_PATTERN] = sensor_config_index_test_pattern,
	[B_FLIP] = sensor_config_index_filp_setting,
	[B_MIRROR] = sensor_config_index_mirror_setting,
	[B_TRIG_STANDARD] = sensor_config_index_trig_mode,
	[B_TRIG_SHUTTER_SYNC] = sensor_config_index_trig_mode
};

typedef struct fcnt_tv_s {
	struct timeval tv;
	uint32_t fcnt;
} fcnt_tv_t;

#define CAMERA_INIT_ID          0xC001
#define SENSOR_TEMPER_ID        0xC002
#define SENSOR_TP_ID            0xC003
#define SENSOR_SYS_ID           0xC004
#define SERDES_LOCK_ID          0xC005
#define SENSOR_FCNT_ID          0xC006
#define SENSOR_I2C_CRC_ID       0xC007
#define SENSOR_POR_ID           0xC008
#define JUNC_TEMPER_RATIO       1000
#define JUNC_TEMPER_MAX         (125 * JUNC_TEMPER_RATIO)
#define JUNC_TEMPER_MIN         (-400 * JUNC_TEMPER_RATIO)
#define CRC16_POLY              0x8005
#define CRC8_POLY               0xF4
#define MONITOR_PERIOD_US       1000000

enum EXP_NUM {
	HDR3 = 3,
	HDR4
};

typedef struct fcnt_check_s {
	fcnt_tv_t fcnt_tv;
	int32_t running;
} fcnt_check_t;

typedef union sensor_status {
	uint32_t value;
	struct {
		/* sensor status */
		/* 0: normal */
		/* stream_off = 1: stream off
		** fps_check = 1: fps check fail
		temp_check = 1: higher then JUNC_TEMPER_MAX
		temp_check = 2: lower then JUNC_TEMPER_MIN
		lock_check = 1: unlocked
		scaler_check = 1: scaler check fail */
		uint32_t stream_off:1;
		uint32_t fps_check:1;
		uint32_t temp_check:2;
		uint32_t lock_check:1;
		uint32_t scaler_check:1;
		uint32_t reserved1:2;
		uint32_t reserved2:8;
	};
} sensor_status_u;

typedef union diag_mask {
	uint32_t value;
	struct {
		uint32_t sensor_group_hold_off:1;
		uint32_t sensor_temperature:1;
		uint32_t sensor_test_pattern:1;
		uint32_t sensor_system_check:1;
		uint32_t serdes_lock:1;
		uint32_t sensor_fcnt_test:1;
		uint32_t sensor_i2c_crc:1;
		uint32_t sensor_poc_check:1;
		uint32_t reserved:24;
	};
} diag_mask_u;

typedef struct sensor_info_ex_s {
	int32_t temperature;
	fcnt_check_t fcnt_check;
	sensor_status_u sensor_status;
	diag_mask_u diag_mask;
}sensor_info_ex_t;
static sensor_info_ex_t sensor_info_exs[CAM_MAX_NUM];

/* from j3 auto hb_cam_interface.h */
typedef struct sensor_status_info_s {
    int32_t temperature;
    uint32_t sensor_status;
} sensor_status_info_t;

uint16_t skip_frame_count[CAM_MAX_NUM] = {0};

uint32_t ae_vs_line_disable = 0;
static uint32_t ae_enable[CAM_MAX_NUM];
static uint32_t awb_enable[CAM_MAX_NUM];
static pthread_t sensor_monitor_tids[CAM_MAX_NUM];
uint32_t ae_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t awb_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_ae_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t bak_awb_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t dev_port2port[CAM_MAX_NUM];
uint32_t name_2a_thread_once[CAM_MAX_NUM];
uint32_t diag_mask[CAM_MAX_NUM];
static int32_t g_sensor_sts_fd[CAM_MAX_NUM];
uint32_t pre_awb_disable[CAM_MAX_NUM];
int32_t extra_mode[CAM_MAX_NUM];
sensor_turning_data_t tuning_data[CAM_MAX_NUM];
sensor_pll_data_t sensor_pll_data;
static sensor_status_info_t* g_sensor_sts[CAM_MAX_NUM];
static uint16_t dcg_add_vs_line_max[CAM_MAX_NUM];
static uint32_t again_tmp_buf[CAM_MAX_NUM];
static uint32_t dgain_tmp_buf[CAM_MAX_NUM];
static uint32_t line_tmp_buf[CAM_MAX_NUM];
static uint32_t rgain_tmp_buf[CAM_MAX_NUM];
static uint32_t bgain_tmp_buf[CAM_MAX_NUM];
static uint32_t grgain_tmp_buf[CAM_MAX_NUM];
static uint32_t gbgain_tmp_buf[CAM_MAX_NUM];

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
	int32_t ret = -1;
	uint32_t setting_size;

	vin_dbg("ov_test_pattern 0x%04x\n", ov_test_pattern[1]);
	setting_size = sizeof(ov_test_pattern)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ov_test_pattern);
	if (ret < 0) {
		vin_err("write ov_test_pattern error\n");
	}
	return ret;
}

static int32_t sensor_config_index_mirror_setting(sensor_info_t *sensor_info)
{
	/* mirror enable */
	int32_t mirror;
	int32_t ret = -1;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	mirror = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
					     sensor_addr, OV_MIRROR_FLIP);
	mirror &= ~BIT(5);
	vin_dbg("ov_mirror_flip 0x%02x\n", mirror);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
					   OV_MIRROR_FLIP, mirror);
	if (ret < 0) {
		vin_err("senor %s write mirror pattern setting error\n",
			sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

static int32_t sensor_config_index_filp_setting(sensor_info_t *sensor_info)
{
	/* flip enable */
	int32_t flip;
	int32_t ret = -1;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	flip = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
					   sensor_addr, OV_MIRROR_FLIP);
	flip |= BIT(2);
	vin_dbg("ov_mirror_flip 0x%02x\n", flip);
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_addr,
					   OV_MIRROR_FLIP, flip);
	if (ret < 0) {
		vin_err("senor %s write flip pattern setting error\n",
			sensor_info->sensor_name);
		return ret;
	}
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

static int32_t sensor_config_index_trig_mode(sensor_info_t *sensor_info)
{
	/* set x3c trigger mode */
	int32_t ret = -1;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t setting_size = 0;
	int32_t ser_trig_mfp;
	uint8_t gpio_id;
	uint32_t trig_pin_num = 0;
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
		vin_err("%s mfp trig config fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_trig_mfp, GPIO_RX_GMSL, gpio_id);
	if (ret < 0) {
		vin_err("max serial mfp config fail\n");
		return ret;
	}
	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		setting_size = sizeof(ovx3c_trigger_arbitrary_mode_setting)/
			sizeof(uint32_t)/2;
		vin_dbg("%s trigger setting %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ovx3c_trigger_arbitrary_mode_setting);
		if (ret < 0) {
			vin_err("senor %s write ARBITRARY_SYNC mode setting error\n",
					sensor_info->sensor_name);
			return ret;
		}
		if (sensor_info->config_index & TRIG_SHUTTER_SYNC) {
			vin_dbg("%s disable ae vs_line\n", sensor_info->sensor_name);
			ae_vs_line_disable = 1;
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

/* from j3 auot hb_cam_utility.c */
static int32_t camera_read_retry(uint32_t bus, uint32_t i2c_addr, int32_t reg_width, uint32_t reg_addr)
{
    x2_camera_i2c_t i2c_cfg;
    int32_t ret = RET_OK, k;

    i2c_cfg.i2c_addr = i2c_addr & SHIFT_8BIT;
    i2c_cfg.reg_size = reg_width;
    i2c_cfg.reg = reg_addr;

    k = CAM_I2C_RETRY_MAX;
    do {
        if (i2c_cfg.reg_size == REG16_VAL16) {
            ret = hb_vin_i2c_read_reg16_data16(bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
        } else if (i2c_cfg.reg_size == REG16_VAL8) {
            ret = hb_vin_i2c_read_reg16_data8(bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
        } else {
            ret = hb_vin_i2c_read_reg8_data8(bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
        }

        if (ret >= RET_OK) {
            break;
        } else {
            vin_warn("camera read reg 0x%2x fail for the %d time\n",
                i2c_cfg.reg, CAM_I2C_RETRY_MAX + 1 - k);
        }
        usleep(20 * 1000);
    } while (k--); /* bad case: read 10 time in case of fail */

    if (ret < 0) {
        vin_err("camera read 0x%2x fail \n", i2c_cfg.reg);
    }
    return ret;
}

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

/* from j3 auto hb_camera.c */
static int32_t camera_diag(uint16_t mod_id, int32_t ret, int32_t event_id)
{
	return 0;
}

int32_t get_fcnt(sensor_info_t *sensor_info)
{
	int32_t val = 0, fcnt = 0;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	val = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					   sensor_addr, OV_VFIFO_FCNT1);
	if (val < 0) {
		vin_err("senor %s read frame counter low bytes error\n",
			sensor_info->sensor_name);
		return val;
	}
	fcnt = val;
	val = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					   sensor_addr, OV_VFIFO_FCNT3);
	if (val < 0) {
		vin_err("senor %s read frame counter high bytes error\n",
			sensor_info->sensor_name);
		return val;
	}
	fcnt |= val << 16;
	return fcnt;
}

int32_t get_sensor_frame_count(sensor_info_t *sensor_info) {
	int32_t fcnt, fcnt_init;
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	fcnt_init = get_fcnt(sensor_info);
	if (fcnt_init < 0) {
		vin_err("%d : get fcnt failed\n", __LINE__);
		return -1;
	}
	for (int32_t i = 0; i < FCNT_RETRY_MAX; i++) {
		usleep(500);
		fcnt = get_fcnt(sensor_info);
		if (fcnt < 0) {
			vin_err("%d : get fcnt failed\n", __LINE__);
			return -1;
		}
		if ((fcnt_init != fcnt) && ((fcnt_init + 1) != fcnt)) {
			vin_err("port [%d] fcnt last read = %ld, now read = %ld, i = %d\n",
				sensor_info->port, fcnt_init, fcnt, i);
			fcnt_init = fcnt;
			continue;
		} else {
			sensor_info_ex->fcnt_check.fcnt_tv.fcnt = fcnt;
			gettimeofday(&sensor_info_ex->fcnt_check.fcnt_tv.tv, NULL);
			vin_dbg("port [%d], fcnt = %d, tv = %ld\n", sensor_info->port,
				sensor_info_ex->fcnt_check.fcnt_tv.fcnt, sensor_info_ex->fcnt_check.fcnt_tv.tv.tv_sec
				* 1000000 + sensor_info_ex->fcnt_check.fcnt_tv.tv.tv_usec);
			return 0;
		}
	}
	vin_err("fcnt reg read err\n");
	return -1;
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
	turning_data->port = sensor_info->dev_port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	// turning_data->tuning_type = TYPE_USER;
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
	uint16_t x0, y0, x1, y1, width, height;
	uint16_t vts, hts_dcg, hts_s, hts_vs;
	uint16_t pll2_prediv0, pll2_prediv, pll2_mult, pll2_divsyspre, pll2_divsys;
	uint16_t pll2_vco, pll2_sclk, pll2_divsys_index, pll2_prediv_index;
	float row_time, fps;
	float pll2_divsys_array[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
	float pll2_prediv_array[] = {1, 1.5, 2, 2.5, 3, 4, 6, 8};
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	vts = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					   sensor_addr, OV_VTS);
	dcg_add_vs_line_max[sensor_info->port] = vts - 12;
	turning_data->sensor_data.VMAX  = vts;
	hts_dcg = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					       sensor_addr, OV_HTS_DCG);

	hts_s = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					     sensor_addr, OV_HTS_S);

	hts_vs = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					      sensor_addr, OV_HTS_VS);

	width = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					     sensor_addr, OV_X_OUTPUT);

	height = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
					      sensor_addr, OV_Y_OUTPUT);

	turning_data->sensor_data.HMAX = hts_dcg + hts_s + hts_vs;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;
	turning_data->sensor_data.gain_max = 1024 * 8192;
	turning_data->sensor_data.analog_gain_max = 1024*8192;
	turning_data->sensor_data.digital_gain_max = 1024*8192;
	if (sensor_info->extra_mode == SUNNY_GM24F100D12_S2R5T8E3 ||
		sensor_info->extra_mode == (uint32_t)LCE_GM24F103D12_S2T0E6 ||
		sensor_info->extra_mode == (uint32_t)LCE_GM24F60D12_S2T0E6) {
		turning_data->sensor_data.exposure_time_min = DCG_LINE_MIN_GALAXY;
	} else if (sensor_info->extra_mode == OFILM_GM24F106D12_S2R3T0E6) {
		turning_data->sensor_data.exposure_time_min = DCG_LINE_MIN_NIO;
	} else {
		turning_data->sensor_data.exposure_time_min = DCG_LINE_MIN;
	}
	turning_data->sensor_data.exposure_time_max = vts - 13;
	turning_data->sensor_data.exposure_time_long_max = vts - 13;

	pll2_prediv0 = (hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
						    sensor_addr, OV_PLL2_PREDIV0) >> 7) + 1;
	pll2_prediv_index = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
							sensor_addr, OV_PLL2_PREDIV) & 0x7;
	pll2_mult = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
						 sensor_addr, OV_PLL2_MULT) & 0x3FF;
	pll2_divsyspre = (hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
						      sensor_addr, OV_PLL2_DIVSYSPRE) & 0xF) + 1;
	pll2_divsys_index = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
							sensor_addr, OV_PLL2_DIVSYS) & 0xF;
	if (pll2_divsys_index >= (sizeof(pll2_divsys_array) / sizeof(pll2_divsys_array[0])))
		pll2_divsys_index = 0;
	pll2_divsys = pll2_divsys_array[pll2_divsys_index];
	if (pll2_prediv_index >= (sizeof(pll2_prediv_array) / sizeof(pll2_prediv_array[0])))
		pll2_prediv_index = 0;
	pll2_prediv = pll2_prediv_array[pll2_prediv_index];
	/* calculate lines_per_second
	hts = 2132, sclk = 162mhz -->81 for double row time
	row_time = hts/sclk = 26.321us
	lines_per_second = 1/row_time = 37992 */
	if (sensor_info->sensor_clk <= 0) {
		ret = vin_sensor_emode_parse(sensor_info, 'M');
		if (ret < 0) {
			vin_err("sensor embode sensor_clk parse fail!!!\n");
			return -1;
		}
		sensor_info->sensor_clk = ret;
	}

	pll2_vco = sensor_info->sensor_clk * pll2_mult / (pll2_prediv0 * pll2_prediv);
	pll2_sclk = pll2_vco / (pll2_divsyspre * pll2_divsys);
	row_time = (float)(turning_data->sensor_data.HMAX)  / (float)pll2_sclk;
	turning_data->sensor_data.lines_per_second = 1000000 / row_time;
	turning_data->sensor_data.turning_type = 6;   // gain calc
	turning_data->sensor_data.conversion = 1;
	fps = (float)pll2_sclk * 1000000 / (turning_data->sensor_data.HMAX *turning_data->sensor_data.VMAX);
	sensor_pll_data.fps = fps;
	sensor_pll_data.sclk = pll2_sclk;
	vin_dbg("HMAX = %d, VMAX = %d, width = %d, height = %d, lines_per_second = %d, xclk = %d, fps = %f\n",
		turning_data->sensor_data.HMAX, turning_data->sensor_data.VMAX,
		turning_data->sensor_data.active_width, turning_data->sensor_data.active_height,
		turning_data->sensor_data.lines_per_second, sensor_info->sensor_clk, fps);

	sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_B, BAYER_PATTERN_RGGB);
	if (sensor_info->config_index & PWL_24BIT)
		sensor_data_bits_fill(&turning_data->sensor_data, 24);
	else
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
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ov_stream_on_setting)) {
		memcpy(stream_on, ov_stream_on_setting, sizeof(ov_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data->stream_ctrl.stream_on) >= sizeof(ov_stream_off_setting)) {
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
	sensor_turning_data_t turning_data;
	uint32_t open_cnt;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
	}
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));
	sensor_common_data_init(sensor_info, &turning_data);
	if(sensor_info->bus_type == I2C_BUS) {
		sensor_param_init(sensor_info, &turning_data);
	}

	turning_data.normal.param_hold = OV_PARAM_HOLD;
	turning_data.normal.param_hold_length = 2;
	turning_data.normal.s_line = OV_DCG_LINE;
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
	turning_data.normal.again_control[0] = OV_HCG_AGAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = OV_HCG_DGAIN;
	turning_data.normal.dgain_control_length[0] = 2;
	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, ovx3c_again_lut,
			sizeof(ovx3c_again_lut));
		for (open_cnt =0; open_cnt < sizeof(ovx3c_again_lut)/
			sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}
	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut,
			ovx3c_dgain_lut, sizeof(ovx3c_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(ovx3c_dgain_lut)/sizeof(uint32_t); open_cnt++) {
			VIN_DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}
#endif

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", sensor_info->port, ret);
		return -RET_ERROR;
	}
	if (turning_data.normal.again_lut)
		free(turning_data.normal.again_lut);
	if (turning_data.normal.dgain_lut)
		free(turning_data.normal.dgain_lut);

	return ret;
}

int32_t sensor_pwl_data_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	sensor_turning_data_t turning_data;
	uint32_t  open_cnt = 0;

	if (sensor_info->dev_port < 0) {
		vin_info("%s ignore dev_port,return ok\n", __func__);
		return RET_OK;
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
	memcpy(&tuning_data[sensor_info->dev_port], &turning_data,
		sizeof(sensor_turning_data_t));
	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("SENSOR_TURNING_PARAM ioctl fail %d\n", ret);
		return -RET_ERROR;
	}
	return ret;
}

static int32_t sensor_x3c_res_fix(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, start, out;
	int32_t setting_size;

	if (sensor_info->width == 1280 && sensor_info->height == 720) {
		vin_info("%s res %dx%d\n", sensor_info->sensor_name,
			 sensor_info->width, sensor_info->height);
		setting_size = sizeof(ovx3c_res_1280x720_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_res_1280x720_init_setting);
		if (ret < 0) {
			vin_err("write ovx3c_res_1280x720_init_setting error\n");
		}
		return ret;
	}

	if (sensor_info->width == RES_WIDTH_1920) {
		start = 968 - (sensor_info->width / 2);
		ovx3c_width_1920_init_setting[1] = start >> 8;
		ovx3c_width_1920_init_setting[3] = start & 0xff;
		ovx3c_width_1920_init_setting[5] = sensor_info->width >> 8;
		ovx3c_width_1920_init_setting[7] = sensor_info->width & 0xff;
		start = (ovx3c_width_1920_init_setting[1] << 8) + ovx3c_width_1920_init_setting[3];
		out = (ovx3c_width_1920_init_setting[5] << 8) + ovx3c_width_1920_init_setting[7];
		vin_dbg("%s width %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,
			out, start, start + out - 1);
		setting_size = sizeof(ovx3c_width_1920_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_width_1920_init_setting);
		if (ret < 0) {
			vin_err("write ovx3c_width_1920_init_setting error\n");
		}
	}

	if (sensor_info->height == RES_HEIGHT_1080) {
		start = 648 - (sensor_info->height / 2);
		ovx3c_height_1080_init_setting[1] = start >> 8;
		ovx3c_height_1080_init_setting[3] = start & 0xff;
		ovx3c_height_1080_init_setting[5] = sensor_info->height >> 8;
		ovx3c_height_1080_init_setting[7] = sensor_info->height & 0xff;
		start = (ovx3c_height_1080_init_setting[1] << 8) + ovx3c_height_1080_init_setting[3];
		out = (ovx3c_height_1080_init_setting[5] << 8) + ovx3c_height_1080_init_setting[7];
		vin_dbg("%s height %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,
				out, start, start + out - 1);
		setting_size = sizeof(ovx3c_height_1080_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_height_1080_init_setting);
		if (ret < 0) {
			vin_err("write ovx3c_height_1080_init_setting error\n");
		}
	}

	return ret;
}

/**
 * @brief sensor_x3c_linear_init : sensor linear mode
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_x3c_linear_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size, tmp = 0;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	setting_size = sizeof(ovx3c_x3_30fps_linear_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("x3 setting_size %d\n", setting_size);
	for(i = 0; i < setting_size; i++) {
#ifdef FPS_HTS
		if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_HTS
			&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
			ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 50160 / sensor_info->fps;
#else
		if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_VTS
			&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
			ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 40380 / sensor_info->fps;
#endif

		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_addr,
						    ovx3c_x3_30fps_linear_init_setting[i*2],
						    ovx3c_x3_30fps_linear_init_setting[i*2 + 1]);
		if (ret < 0) {
			tmp++;
			if (tmp < 10) {
				i--;
				usleep(10*1000);
				continue;
			}
			vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
				sensor_info->sensor_name,
				sensor_info->bus_num, sensor_addr,
				i, ovx3c_x3_30fps_linear_init_setting[i*2],
				ovx3c_x3_30fps_linear_init_setting[i*2 + 1]);
			return ret;
		}
		if((i == 1) || (i == 1070))
			usleep(200*1000);
		tmp = 0;
	}

	ret = sensor_x3c_res_fix(sensor_info);
	if (ret < 0)
		return ret;

	ret = sensor_linear_data_init(sensor_info);
	if (ret < 0)
		return ret;

	vin_dbg("sensor_x3c_linear_init OK!\n");

	return ret;
}

/**
 * @brief sensor_x3c_pwl_init : write sy/ws x3c pwl setting
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_x3c_pwl_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, i;
	int32_t setting_size, tmp = 0;

	ret = vin_sensor_emode_parse(sensor_info, 'R');
	if (ret == -FLAG_NOT_FIND) {
		vin_info("port:%02d reset pin not find, now set software rst\n", sensor_info->port);
		setting_size = sizeof(ovx3c_init_setting_rst)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
			      SENSOR_REG_WIDTH,
			      setting_size, ovx3c_init_setting_rst);
		if (ret < 0) {
			vin_err("senor %s write rst setting error\n", sensor_info->sensor_name);
			return ret;
		}
		usleep(10*1000);
	}
	if (sensor_info->sensor_clk <= 0) {
		ret = vin_sensor_emode_parse(sensor_info, 'M');
		if (ret < 0) {
			vin_err("sensor emode %s sensor_clk parse fail\n", SENSOR_EMODE_NAME(sensor_info));
			return -1;
		}
		sensor_info->sensor_clk = ret;
	}
	// write init setting
	if (sensor_info->sensor_clk == 27) {
		setting_size = sizeof(ovx3c_init_setting_27M_hdr4)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_27M_hdr4);
		if (ret < 0) {
			vin_err("senor %s write 27M hdr4 setting error\n",
					sensor_info->sensor_name);
			return ret;
		}
	} else if (sensor_info->sensor_clk == 24) {
		setting_size = sizeof(ovx3c_init_setting_24M_hdr4)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_24M_hdr4);
		if (ret < 0) {
			vin_err("senor %s write 24M hdr4 setting error\n",
					sensor_info->sensor_name);
			return ret;
		}
	} else {
		vin_err("sensor clk %dM is not support!!!\n", sensor_info->sensor_clk);
		return -1;
	}
	if (sensor_info->config_index & PWL_24BIT) {
		vin_dbg("senor %s pwl 24bit\n", sensor_info->sensor_name);
		setting_size = sizeof(sy_ovx3c_pwl_setting_24bit) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, sy_ovx3c_pwl_setting_24bit);
	} else {
		setting_size = sizeof(of_ovx3c_pwl_setting) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, of_ovx3c_pwl_setting);
	}
	if (ret < 0) {
		vin_err("senor %s pwl setting error\n",
			sensor_info->sensor_name);
		return ret;
	}

	if (sensor_info->fps == 30) {
		setting_size = sizeof(ovx3c_init_setting_1280p_30fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_30fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 25) {
		setting_size = sizeof(ovx3c_init_setting_1280p_25fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_25fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 20) {
		setting_size = sizeof(ovx3c_init_setting_1280p_20fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_20fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 15) {
		setting_size = sizeof(ovx3c_init_setting_1280p_15fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_15fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 10) {
		setting_size = sizeof(ovx3c_init_setting_1280p_10fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_10fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 12) {
		// 12.5fps
		setting_size = sizeof(ovx3c_init_setting_1280p_12_5fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				      SENSOR_REG_WIDTH,
				      setting_size, ovx3c_init_setting_1280p_12_5fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution,
				sensor_info->fps);
			return ret;
		}
	} else {
		vin_err("senor %s write resolution=%d--fps=%d setting not supported\n",
			sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
		return -RET_ERROR;
	}

	ret = sensor_x3c_res_fix(sensor_info);
	if (ret < 0)
		return ret;

	vin_info("sensor_x3c_pwl_init OK!\n");
	ret = sensor_pwl_data_init(sensor_info);
	if(ret < 0) {
		vin_err("sensor_pwl_data_init %s fail\n", sensor_info->sensor_name);
		return ret;
	}
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
	vin_dbg("%dfps settint, vts %d to %d!\n",
		sensor_info->fps / 2, vts_v, vts_v * 2);
	vts_v *= 2;
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_addr,
					    OV_VTS, vts_v);
	if (ret < 0)
		vin_err("write register error\n");
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
		ret = hb_vin_i2c_read_block_reg16(bus, sensor_addr, OV_HTS_DCG, reg_vb, 2);
		reg_v = (reg_vb[0] << 8) | reg_vb[1];
		reg_vs = (uint32_t)(reg_v * ratio);
		vin_info("hts ratio %.2f setting: hts(0x%04x) %d to %d\n",
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

static int32_t ovx3c_config_crop_feature(sensor_info_t *sensor_info)
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
		if (width > 1920u) {
			vin_err("under total:%d width:%d, not support x_offset:%d \n",
					1936, width, crop_offset_x);
			return -1;
		}
		if (height > 1280u) {
			vin_err("under total:%d height:%d, not support y_offset:%d \n",
					1296, height, crop_offset_y);
			return -1;
		}

		start = (1936u / 2u) - (width / 2u);
		ovx3c_crop_setting[11] = width >> 8u;  // width_h;
		ovx3c_crop_setting[13] = width & 0xffu;  // width_l;
		ovx3c_crop_setting[3] = start >> 8u;
		ovx3c_crop_setting[5] = start & 0xffu;

		start = 4; // (1296u / 2u) - (height / 2u);
		ovx3c_crop_setting[15] = height >> 8u;  // height_h;
		ovx3c_crop_setting[17] = height & 0xffu;  // height_l;
		ovx3c_crop_setting[7] = start >> 8u;
		ovx3c_crop_setting[9] = start & 0xffu;
	} else if (crop_offset_x > 0 && crop_offset_y > 0) {
		if (ALIGN_4(crop_offset_x) + width > 1936u) {
			vin_err("under total:%d width:%d, not support x_offset:%d \n",
					1936, width, ALIGN_4(crop_offset_x));
			return -1;
		}
		if (ALIGN_4(crop_offset_y) + height > 1296u - 4u) {
			vin_err("under total:%d height:%d, not support y_offset:%d \n",
					1296, height, ALIGN_4(crop_offset_y));
			return -1;
		}
		ovx3c_crop_setting[11] = width >> 8u;  //  width_h;
		ovx3c_crop_setting[13] = width & 0xffu;  //  width_l;
		ovx3c_crop_setting[3] = ALIGN_4(crop_offset_x) >> 8u;  // offset_h
		ovx3c_crop_setting[5] = ALIGN_4(crop_offset_x) & 0xffu;  //  offset_l

		ovx3c_crop_setting[15] = height >> 8u;  // height_h;
		ovx3c_crop_setting[17] = height & 0xffu;  // height_l;
		ovx3c_crop_setting[7] = ALIGN_4(crop_offset_y) >> 8u;  // offset_h
		ovx3c_crop_setting[9] = ALIGN_4(crop_offset_y) & 0xffu;  //  offset_l
	} else {
		vin_err("not support this crop offset config\n");
		return -1;
	}
	vin_dbg("config crop width_h: 0x%x, width_l: 0x%x, start_h: 0x%x, start_l: 0x%x\n",
			ovx3c_crop_setting[11], ovx3c_crop_setting[13],
			ovx3c_crop_setting[3], ovx3c_crop_setting[5]);
	vin_dbg("config crop height_h: 0x%x, height_l: 0x%x, start_h: 0x%x, start_l: 0x%x\n",
			ovx3c_crop_setting[15], ovx3c_crop_setting[17],
			ovx3c_crop_setting[7], ovx3c_crop_setting[9]);

	pdata = ovx3c_crop_setting;
	setting_size = sizeof(ovx3c_crop_setting)/sizeof(uint32_t)/2UL;
	ret = vin_write_array(bus, sensor_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("config rectangle error\n");
		return ret;
	}
	vin_info("config crop success\n");
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
	uint32_t bus_num, i2c_addr;

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:  //  normal
			ret = sensor_x3c_linear_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_x3c_linear_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		case PWL_M:
			ret = sensor_x3c_pwl_init(sensor_info);
			if(ret < 0) {
				vin_err("sensor_x3c_pwl_init %s fail\n", sensor_info->sensor_name);
				return ret;
			}
			break;
		default:
		    vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			break;
	}

	/* embedded data enable */
	if (sensor_info->config_index & EMBEDDED_MODE) {
		if (sensor_info->config_index & EMBEDDED_DATA) {
			setting_size = sizeof(emb_data_front_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_front_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode 0 setting error\n",
					sensor_info->sensor_name);
				return ret;
			}
		} else {
			setting_size = sizeof(emb_data_front_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_front_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode %d setting error\n",
					sensor_info->sensor_name, EMBEDDED_MODE);
				return ret;
			}
			ret = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr, OV_Y_OUTPUT);
			if (ret < 0) {
				vin_err("senor %s read output size error\n", sensor_info->sensor_name);
				return ret;
			}
			ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
				sensor_info->sensor_addr, OV_Y_OUTPUT, ret - 2);
			if (ret < 0) {
				vin_err("senor %s write output size error\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}

	ret = ovx3c_config_crop_feature(sensor_info);
	if (ret < 0) {
		vin_err("%s crop feature error\n", sensor_info->sensor_name);
		return ret;
	}

	ret = sensor_config_special_timing(sensor_info);
	if (ret < 0) {
		vin_err("%s sensor special timing error\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info)
{
		int32_t ret = RET_OK;

		switch(sensor_info->sensor_mode) {
			case NORMAL_M:  //  normal
				ret = sensor_linear_data_init(sensor_info);
				if (ret < 0) {
					vin_err("sensor_linear_data_init fail\n");
					return ret;
				}
				break;
			case PWL_M:
				ret = sensor_pwl_data_init(sensor_info);
				if (ret < 0) {
					vin_err("sensor_pwl_data_init fail\n");
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
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	if (fps < 1 || sensor_info->fps > 30) {
		vin_err("%s %s %dfps not support\n", __func__, sensor_info->sensor_name, fps);
		return -RET_ERROR;
	}
	vin_info("%s %s %dfps\n", __func__, sensor_info->sensor_name, fps);
#ifdef FPS_HTS
	switch (sensor_info->sensor_mode) {
		case NORMAL_M:
			xts = 50160 / fps;
			break;
		case PWL_M:
			xts = 60000 / fps;
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			return -RET_ERROR;
	}
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num,
					    sensor_addr, OV_HTS, xts);
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
			OVX3C_VTS, xts);
#endif
	if(ret < 0) {
		vin_err("camera: write 0x%x block fail\n", sensor_addr);
		return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
	}
	sensor_info->fps = fps;
	sensor_update_fps_notify_driver(sensor_info);
	vin_info("dynamic_switch to %dfps success\n", fps);
	return RET_OK;
}

int32_t f_sensor_init_global_data(sensor_info_t *sensor_info);

static int32_t sensor_awb_info_set(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch (sensor_info->extra_mode) {
		case SUNNY_GM24F60D12_S2R5T8E3:
			extra_mode[sensor_info->port] = SUNNY_GM24F60D12_S2R5T8E3;
			vin_info("The pre awb ratio is OX3GB-O060+038\n");
			break;
		case SUNNY_GM24F100D12_S2R5T8E3:
			extra_mode[sensor_info->port] = SUNNY_GM24F100D12_S2R5T8E3;
			vin_info("The pre awb ratio is default\n");
			break;
		case SENSING_GM27F216D12_S0R0T8E5:
			extra_mode[sensor_info->port] = SENSING_GM27F216D12_S0R0T8E5;
			vin_info("The pre awb ratio is default\n");
			break;
		case LCE_GM24F103D12_S2T0E6:
			extra_mode[sensor_info->port] = LCE_GM24F103D12_S2T0E6;
			vin_info("The pre awb ratio is default\n");
			break;
		case SENSING_GM24F106D12_S0R0T8E5:
			extra_mode[sensor_info->port] = SENSING_GM24F106D12_S0R0T8E5;
			vin_info("The pre awb ratio is default\n");
			break;
		case OFILM_GM24F106D12_S0R3T0E6:
			extra_mode[sensor_info->port] = OFILM_GM24F106D12_S0R3T0E6;
			vin_info("The pre awb ratio is default\n");
			break;
		case GALAXY_GM24F100D12_S2R5T8E3:
			extra_mode[sensor_info->port] = GALAXY_GM24F100D12_S2R5T8E3;
			vin_info("The pre awb ratio is default\n");
			break;
		case OFILM_GM24F106D12_S2R3T0E6:
			extra_mode[sensor_info->port] = OFILM_GM24F106D12_S2R3T0E6;
			vin_info("The pre awb ratio is default\n");
			break;
		case (uint32_t)LCE_GM24F60D12_S2T0E6:
			extra_mode[sensor_info->port] = (int32_t)LCE_GM24F60D12_S2T0E6;
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
	int32_t ret = RET_OK;
	int8_t sensor_group_hold_disable = 0, sccb_i2c_en = 0;
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];

	/* Group hold config parse*/
	ret = sensor_param_parse(sensor_info, "sensor_debug/group_hold_disable", ISINT,
			&sensor_group_hold_disable);
	sensor_info_ex->diag_mask.sensor_group_hold_off = sensor_group_hold_disable;

	/* Sccb crc check config parse */
	ret = sensor_param_parse(sensor_info, "sensor_debug/sccb_crc", ISINT,
			&sccb_i2c_en);
	sensor_info_ex->diag_mask.sensor_i2c_crc = sccb_i2c_en;

	if (sensor_info_ex->diag_mask.sensor_group_hold_off)
		vin_warn("port [%d] sensor_group_hold is disable\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_i2c_crc)
		vin_warn("port [%d] sensor_i2c_crc is enable\n", sensor_info->port);
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	int32_t entry_num = sensor_info->entry_num;

	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}
	ret = sensor_awb_info_set(sensor_info);
	if (ret < 0) {
		vin_err("sensor extra_mode config is invalid\n");
		return ret;
	}
	sensor_config_debug_mask(sensor_info);
	ae_enable[sensor_info->dev_port] = HAL_AE_LINE_GAIN_CONTROL;
	awb_enable[sensor_info->dev_port] = HAL_AWB_CCT_CONTROL;

	name_2a_thread_once[sensor_info->port] = 1;
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

	ret = max_serial_init(sensor_info);
	if (ret < 0) {
		vin_err("max serial init error\n");
		return ret;
	}

	vin_info("x3c serializer init done\n");
	/* 4. sensor mode config */
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	/* 5. config_index_init */
	ret = sensor_config_do(sensor_info, CONFIG_INDEX_ALL, sensor_config_index_funcs);
	f_sensor_init_global_data(sensor_info);

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

int32_t sensor_ovx3c_serdes_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = ((deserial_module_t *)(deserial_if->deserial_ops))->stream_on(
		deserial_if, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("deserdial %s stream on fail\n", deserial_if->deserial_name);
	}
	return ret;
}

int32_t sensor_ovx3c_serdes_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	ret = ((deserial_module_t *)(deserial_if->deserial_ops))->stream_off(
		deserial_if, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("deserdial %s stream off fail\n", deserial_if->deserial_name);
	}
	return ret;
}

void *sensor_status_monitor(void *arg);
int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t setting_size = 0, i, req;
	uint8_t *pdata = NULL;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = sensor_info->bus_num;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	int32_t ret = RET_OK, tmp = 0;
	int32_t entry_num = sensor_info->entry_num;

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
	setting_size = sizeof(ov_stream_on_setting)/sizeof(uint32_t)/2;
	vin_info("%s sensor_start setting_size %d\n",
			sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		(void)hb_vin_i2c_lock(bus);
		ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr,
							ov_stream_on_setting[i*2],
							ov_stream_on_setting[i*2 + 1]);
		(void)hb_vin_i2c_unlock(bus);
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

	if (deserial_if) {
		req = hb_vin_mipi_pre_request(entry_num, 1, 0);
		if (req == 0) {
			ret = sensor_ovx3c_serdes_stream_on(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
				vin_err("%d : %s sensor_ovx3c_serdes_stream_on fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
			hb_vin_mipi_pre_result(entry_num, 1, ret);
		}
	}

	ret = get_sensor_frame_count(sensor_info);
	if (ret < 0) {
		vin_err("senor %s port [%d] get fcnt error\n", sensor_info->sensor_name,
			sensor_info->port);
	}
	if (CAM_START != sensor_info->start_state) {
		ret = pthread_create(&sensor_monitor_tids[sensor_info->port],
			NULL, sensor_status_monitor, sensor_info);
		if (ret) {
			vin_err("sensor_fcnt_test pthread_create fail\n");
			return -HB_CAM_START_FAIL;
		}
	}
	sensor_info_ex->fcnt_check.running = 1;

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	int32_t setting_size = 0, i;
	uint8_t value;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	uint32_t bus = sensor_info->bus_num;

	sensor_info_ex->fcnt_check.running = 0;
	setting_size = sizeof(ov_stream_off_setting)/sizeof(uint32_t)/2;
	vin_info("%s sensor_stop setting_size %d\n",
			sensor_info->sensor_name, setting_size);
	for(i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(bus, sensor_addr,
							ov_stream_off_setting[i*2],
							ov_stream_off_setting[i*2 + 1]);
		if (ret < 0) {
			vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
		}
	}

	pthread_join(sensor_monitor_tids[sensor_info->port], NULL);
	return ret;
}

int32_t f_sensor_deinit_global_data(sensor_info_t *sensor_info);
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

	f_sensor_deinit_global_data(sensor_info);
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
	int32_t i2c_num;
	int32_t i2c_addr;

	if (!sp || !si) {
		vin_err("input sp|si is null!\n");
		return -RET_ERROR;
	}
	i2c_num = si->bus_num;
	i2c_addr = si->sensor_addr;

	sp->frame_length = tuning_data[si->dev_port].sensor_data.VMAX;
	sp->line_length = tuning_data[si->dev_port].sensor_data.HMAX;

	sp->width = tuning_data[si->dev_port].sensor_data.active_width;
	sp->height = tuning_data[si->dev_port].sensor_data.active_height;

	if (si->extra_mode == SENSING_GM27F216D12_S0R0T8E5) {
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));
	} else if (si->extra_mode == SUNNY_GM24F100D12_S2R5T8E3) {
		strncpy(sp->version, VERSION_OF_OVX3C, sizeof(sp->version));
	}

	sp->pclk = sensor_pll_data.sclk;
	sp->fps = sensor_pll_data.fps;
	sp->exp_num = HDR4;
	sp->lines_per_second = tuning_data[si->dev_port].sensor_data.lines_per_second;

	return ret;
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
	int32_t port = dev_port2port[info->port];

	if((rgain == rgain_tmp_buf[info->port]) &&
			(bgain == bgain_tmp_buf[info->port]) &&
			(grgain == grgain_tmp_buf[info->port]) &&
			(gbgain == gbgain_tmp_buf[info->port]))
		return 0;
	rgain_tmp_buf[info->port] = rgain;
	bgain_tmp_buf[info->port] = bgain;
	grgain_tmp_buf[info->port] = grgain;
	gbgain_tmp_buf[info->port] = gbgain;

	/* lpd spd b/g gain & r/g gain ratio under standard light */
	if (extra_mode[port] == SUNNY_GM24F60D12_S2R5T8E3) {
		awb_alight[0] =
			SN60_ALIGHT_SPD_BGAIN / SN60_ALIGHT_LPD_BGAIN *
			SN60_ALIGHT_LPD_GGAIN / SN60_ALIGHT_SPD_GGAIN;
		awb_alight[1] =
			SN60_ALIGHT_SPD_RGAIN / SN60_ALIGHT_LPD_RGAIN *
			SN60_ALIGHT_LPD_GGAIN / SN60_ALIGHT_SPD_GGAIN;
		awb_cwf[0] = SN60_CWF_SPD_BGAIN / SN60_CWF_LPD_BGAIN;
		awb_cwf[1] = SN60_CWF_SPD_RGAIN / SN60_CWF_LPD_RGAIN;
		awb_d65[0] = SN60_D65_SPD_BGAIN / SN60_D65_LPD_BGAIN;
		awb_d65[1] = SN60_D65_SPD_RGAIN / SN60_D65_LPD_RGAIN;
	} else if (extra_mode[port] == (int32_t)LCE_GM24F103D12_S2T0E6 ||
		extra_mode[port] == (int32_t)OFILM_GM24F106D12_S2R3T0E6) {
		awb_alight[0] =
			LCE100_ALIGHT_SPD_BGAIN / LCE100_ALIGHT_LPD_BGAIN *
			LCE100_ALIGHT_LPD_GGAIN / LCE100_ALIGHT_SPD_GGAIN;
		awb_alight[1] =
			LCE100_ALIGHT_SPD_RGAIN / LCE100_ALIGHT_LPD_RGAIN *
			LCE100_ALIGHT_LPD_GGAIN / LCE100_ALIGHT_SPD_GGAIN;
		awb_cwf[0] = LCE100_CWF_SPD_BGAIN / LCE100_CWF_LPD_BGAIN;
		awb_cwf[1] = LCE100_CWF_SPD_RGAIN / LCE100_CWF_LPD_RGAIN;
		awb_d65[0] = LCE100_D65_SPD_BGAIN / LCE100_D65_LPD_BGAIN;
		awb_d65[1] = LCE100_D65_SPD_RGAIN / LCE100_D65_LPD_RGAIN;
	} else if (extra_mode[port] == (int32_t)LCE_GM24F60D12_S2T0E6) {
		awb_alight[0] =
			LCE60_ALIGHT_SPD_BGAIN / LCE60_ALIGHT_LPD_BGAIN *
			LCE60_ALIGHT_LPD_GGAIN / LCE60_ALIGHT_SPD_GGAIN;
		awb_alight[1] =
			LCE60_ALIGHT_SPD_RGAIN / LCE60_ALIGHT_LPD_RGAIN *
			LCE60_ALIGHT_LPD_GGAIN / LCE60_ALIGHT_SPD_GGAIN;
		awb_cwf[0] = LCE60_CWF_SPD_BGAIN / LCE60_CWF_LPD_BGAIN;
		awb_cwf[1] = LCE60_CWF_SPD_RGAIN / LCE60_CWF_LPD_RGAIN;
		awb_d65[0] = LCE60_D65_SPD_BGAIN / LCE60_D65_LPD_BGAIN;
		awb_d65[1] = LCE60_D65_SPD_RGAIN / LCE60_D65_LPD_RGAIN;
	} else {
		awb_alight[0] =
			ALIGHT_SPD_BGAIN / ALIGHT_LPD_BGAIN *
			ALIGHT_LPD_GGAIN / ALIGHT_SPD_GGAIN;
		awb_alight[1] =
			ALIGHT_SPD_RGAIN / ALIGHT_LPD_RGAIN *
			ALIGHT_LPD_GGAIN / ALIGHT_SPD_GGAIN;
		awb_cwf[0] = CWF_SPD_BGAIN / CWF_LPD_BGAIN;
		awb_cwf[1] = CWF_SPD_RGAIN / CWF_LPD_RGAIN;
		awb_d65[0] = D65_SPD_BGAIN / D65_LPD_BGAIN;
		awb_d65[1] = D65_SPD_RGAIN / D65_LPD_RGAIN;
	}

	if (pre_awb_disable[info->port]) {
		awb_alight[0] = 1;
		awb_alight[1] = 1;
		awb_cwf[0] = 1;
		awb_cwf[1] = 1;
		awb_d65[0] = 1;
		awb_d65[1] = 1;
	}
	awb_lpd_r_gain = rgain << 2;
	awb_lpd_b_gain = bgain << 2;
	/* lpd spd ratio for grgain equal to 1, grgain = gbgain */
	awb_lpd_gr_gain = grgain << 2;
	awb_spd_gr_gain = awb_lpd_gr_gain;

	/* interpolating lpd/spd rgain & bgain ratio */
	if (color_temper <= ALIGHT_TEMPER) {
		awb_spd_b_gain = awb_lpd_b_gain * awb_alight[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_alight[1];
		vin_dbg("light temper under alight\n");
	} else if (color_temper <= CWF_TEMPER) {
		for (i = 0; i < 2; i++) {
			if (awb_cwf[i] - awb_alight[i] >= 0) {
				temp1 = (float)(CWF_TEMPER - ALIGHT_TEMPER) /
						(awb_cwf[i] - awb_alight[i]);
				temp2 = color_temper - ALIGHT_TEMPER;
				awb_cur_light[i] = awb_alight[i] + temp2 / temp1;
			} else {
				temp1 = (float)(CWF_TEMPER - ALIGHT_TEMPER) /
						(awb_alight[i] - awb_cwf[i]);
				temp2 = color_temper - ALIGHT_TEMPER;
				awb_cur_light[i] = awb_alight[i] - temp2 / temp1;
			}
		}
		awb_spd_b_gain = awb_lpd_b_gain * awb_cur_light[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_cur_light[1];
		vin_dbg("light temper between alight & cwf\n");
	} else if (color_temper <= D65_TEMPER) {
		for (i = 0; i < 2; i++) {
			if (awb_d65[i] - awb_cwf[i] >= 0) {
				temp1 = (float)(D65_TEMPER - CWF_TEMPER) /
						(awb_d65[i] - awb_cwf[i]);
				temp2 = color_temper - CWF_TEMPER;
				awb_cur_light[i] = awb_cwf[i] + temp2 / temp1;
			} else {
				temp1 = (float)(D65_TEMPER - CWF_TEMPER) /
						(awb_cwf[i] - awb_d65[i]);
				temp2 = color_temper - CWF_TEMPER;
				awb_cur_light[i] = awb_cwf[i] - temp2 / temp1;
			}
		}
		awb_spd_r_gain = awb_lpd_r_gain * awb_cur_light[1];
		awb_spd_b_gain = awb_lpd_b_gain * awb_cur_light[0];
		vin_dbg("light temper between cwf & d65\n");
	} else {
		awb_spd_b_gain = awb_lpd_b_gain * awb_d65[0];
		awb_spd_r_gain = awb_lpd_r_gain * awb_d65[1];
		vin_dbg("light temper higher d65\n");
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

int32_t group_hold_start(hal_control_info_t *info);
int32_t group_hold_end(hal_control_info_t *info);
int32_t write_awb_reg(hal_control_info_t *info);
int32_t write_ae_reg(hal_control_info_t *info);

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
	int32_t ret = 0;
	int32_t port = dev_port2port[info->port];
	vin_dbg("dev_port %d rgain = %d, bgain = %d, grgain = %d, gbgain = %d\n",
			info->port, rgain, bgain, grgain, gbgain);
	vin_dbg(" color_temper = %d!\n", color_temper);

    // rgain = 256;
    // bgain = 256;
	if (skip_frame_count[info->port] < FRAME_SKIP_COUNT + 1) {
		skip_frame_count[info->port]++;
		return ret;
	}

	ret = set_awb_reg(info, rgain, bgain, grgain, gbgain, color_temper);
	if (ret < 0) {
		vin_err("port [%d] set_awb_reg failed\n", info->port);
		return ret;
	}
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
	*enable = (HAL_AWB_CCT_CONTROL & awb_enable[port]) +
              (HAL_AE_LINE_GAIN_CONTROL & ae_enable[port]);
	vin_info("dev_port %d enter userspace_control enable = %d\n", port, *enable);

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
	int32_t ret = 0, setting_size;
	int32_t ae_index = 0;
	line_gain_control_t hcg, lcg, spd, vs;
	int32_t hcg_lcg_ratio = OVX3C_HCG_LCG_RATIO;
	int32_t	lcg_spd_ratio = OVX3C_LCG_SPD_RATIO;
	int32_t lcg_vs_ratio = OVX3C_LCG_VS_RATIO;
	int32_t port = dev_port2port[info->port];
	int32_t dcg_line_min_cfg = tuning_data[info->port].sensor_data.exposure_time_min;
	char tname[16] = {0};
	uint32_t line_tmp = 0;
	if (name_2a_thread_once[port]) {
		snprintf(tname, sizeof(tname), "ovx3c_ae_%d", port);
		prctl(PR_SET_NAME, tname);
		name_2a_thread_once[port] = 0;
	}

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

	vin_dbg("dev port %d gain mode %d, --line %d, again %d, dgain %d \n",
		info->port, mode, line[0], again[0], dgain[0]);

	if (extra_mode[port] == (int32_t)SUNNY_GM24F60D12_S2R5T8E3 ||
		extra_mode[port] == (int32_t)LCE_GM24F60D12_S2T0E6) {
		lcg_spd_ratio = OVX3C_LCG_SPD_RATIO_SN60;
		lcg_vs_ratio = OVX3C_LCG_VS_RATIO_SN60;
	}
	if (extra_mode[port] == (int32_t)LCE_GM24F103D12_S2T0E6 ||
		extra_mode[port] == (int32_t)OFILM_GM24F106D12_S2R3T0E6) {
		lcg_spd_ratio = OVX3C_LCG_SPD_RATIO_LCE100;
		lcg_vs_ratio = OVX3C_LCG_VS_RATIO_LCE100;
	}
	/* calculate exposure value = line * 2^(gain/32) */
	hcg.gain = pow(2, ((float)(again[0] + dgain[0])/32));
	hcg.exp_value = line[0] * hcg.gain;
	lcg.exp_value = hcg.exp_value * OVX3C_HCG_LCG_CG_RATIO /
					hcg_lcg_ratio;
	spd.exp_value = lcg.exp_value * OVX3C_SENS_RATIO / lcg_spd_ratio;
	vs.exp_value = lcg.exp_value / lcg_vs_ratio;

	/* calculate hcg line & gain */
	hcg.line = line[0];
	COMPARE_AND_ASSIGN(hcg.line, dcg_line_min_cfg,
	dcg_add_vs_line_max[info->port] - VS_LINE_MIN);

	hcg.again = hcg.gain;
	COMPARE_AND_ASSIGN(hcg.again, HCG_AGAIN_MIN, AGAIN_MAX);

	hcg.dgain = hcg.gain / hcg.again;
	COMPARE_AND_ASSIGN(hcg.dgain, DGAIN_MIN, DGAIN_MAX);

	/* calculate lcg gain */
	lcg.gain = lcg.exp_value / (float)hcg.line;
	lcg.again = lcg.gain / LCG_VS_DGAIN_MIN;
	COMPARE_AND_ASSIGN(lcg.again, LCG_VS_AGAIN_MIN, AGAIN_MAX);

	lcg.dgain = lcg.gain / lcg.again;
	COMPARE_AND_ASSIGN(lcg.dgain, LCG_VS_DGAIN_MIN, DGAIN_MAX);

	/* calculate spd line & gain */
	line_tmp = spd.exp_value / (SPD_AGAIN_MIN * DGAIN_MIN);
	COMPARE_AND_ASSIGN(line_tmp, SPD_LINE_MIN, dcg_add_vs_line_max[info->port]);
	spd.line = line_tmp;
	spd.gain = spd.exp_value / (float)spd.line;
	spd.again = spd.gain / DGAIN_MIN;
	COMPARE_AND_ASSIGN(spd.again, SPD_AGAIN_MIN, AGAIN_MAX);

	spd.dgain = spd.gain / spd.again;
	COMPARE_AND_ASSIGN(spd.dgain, DGAIN_MIN, DGAIN_MAX);

	/* calculate vs line & gain*/
	/* exp_value control */
	vs.line = vs.exp_value / (LCG_VS_AGAIN_MIN * LCG_VS_DGAIN_MIN) - 0.5;

	COMPARE_AND_ASSIGN(vs.line, VS_LINE_MIN, VS_LINE_MAX);
	// Max(DCG_exp + VS_exp, SPD_exp) < VTS - 12
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

	return 0;
}

int32_t sensor_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size, i;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	setting_size = sizeof(ov_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("%s sensor_stop setting_size %d\n",
	sensor_info->sensor_name, setting_size);
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
						   sensor_addr, ov_stream_off_setting[i * 2],
						   ov_stream_off_setting[i * 2 + 1]);
		if (ret < 0) {
			vin_err("%s stream off failed\n", sensor_info->sensor_name);
			return ret;
		}
	}
	return ret;
}

int32_t sensor_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size, i;
	uint8_t sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;

	setting_size = sizeof(ov_stream_on_setting) / sizeof(uint32_t) / 2;
	for (i = 0; i < setting_size; i++) {
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
						   sensor_addr, ov_stream_on_setting[i * 2],
						   ov_stream_on_setting[i * 2 + 1]);
		if (ret < 0) {
			vin_err("%s : stream on failed\n", sensor_info->sensor_name);
		}
	}
	ret = get_sensor_frame_count(sensor_info);
	if (ret < 0) {
		vin_err("senor %s port [%d] get fcnt error\n", sensor_info->sensor_name,
			sensor_info->port);
	}

	return ret;
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
	if (si->extra_mode == (uint32_t)LCE_GM24F103D12_S2T0E6 ||
		si->extra_mode == (uint32_t)LCE_GM24F60D12_S2T0E6) {
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
		sip->cam_type = (uint8_t)data;
		if (data == 2) {
			sip->cam_type = 0;
		} else if (data == 5) {
			sip->cam_type = 7;
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
		vin_err("ovx3c param error type:%d i2c-num:%d eeprom-addr:0x%x!!\n",
				type, si->bus_num, si->eeprom_addr);
		ret = -RET_ERROR;
	}
	return ret;
}

int32_t sensor_get_status(sensor_info_t *sensor_info)
{
	int32_t val, ret = 0;
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];

        // sensor lock
	if (sensor_info_ex->diag_mask.serdes_lock) {
		deserial_info_t *deserial_if = sensor_info->deserial_info;
#if 0
		if (deserial_if != NULL) {
			deserial_module_t *deserial_module = deserial_if->deserial_ops;
			deserial_status_t *deserial_status = &deserial_if->deserial_status;
			uint32_t last_lock = sensor_info_ex->sensor_status.lock_check;
			ret = deserial_module->get_status(deserial_if, deserial_status);
			if (ret < 0) {
				vin_err("port [%d] get %s status failed!\n", sensor_info->port,
					deserial_if->deserial_name);
				return ret;
			}
			sensor_info_ex->sensor_status.lock_check =
				!deserial_status->links[sensor_info->deserial_port].lock;
			if (last_lock != sensor_info_ex->sensor_status.lock_check) {
				camera_diag(SERDES_LOCK_ID, sensor_info_ex->sensor_status.lock_check,
					sensor_info->port + 1);
			}
			if (sensor_info_ex->sensor_status.lock_check == 1) {
				vin_err("port [%d] is unlocked\n", sensor_info->port);
				return 0;
			}
		}
#endif
	}


	// stream state check
	val = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_STREAMING);

	if (val < 0) {
		vin_err("senor %s port [%d] read stream state error\n",
			sensor_info->sensor_name, sensor_info->port);
		return val;
	}

	if (0 == val) {
		sensor_info_ex->sensor_status.stream_off = 1;
		vin_err("sensor %s port [%d] is in stream off mode\n",
			sensor_info->sensor_name, sensor_info->port);
		return 0;
	} else {
		sensor_info_ex->sensor_status.stream_off = 0;
	}


	// temperature check
	if (sensor_info_ex->diag_mask.sensor_temperature) {
		int32_t temper;
		ret = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_AVER_TEMPER);
		if (ret < 0) {
			vin_err("senor %s port [%d] read temper error\n",
				sensor_info->sensor_name, sensor_info->port);
			return ret;
		} else {
			int32_t last_temp = sensor_info_ex->sensor_status.temp_check;
			temper = (ret <= 0xC000) ? ret : (-1) * (ret - 0xC000);
			temper = (temper * JUNC_TEMPER_RATIO) >> 8;
			sensor_info_ex->temperature = temper;
			vin_dbg("port [%d] temper = %d\n", sensor_info->port, temper);

			if (temper > JUNC_TEMPER_MAX) {
				sensor_info_ex->sensor_status.temp_check = 1;
				vin_warn("port [%d] temper = %d higher than %d!\n",
					sensor_info->port, temper, JUNC_TEMPER_MAX);
			} else if (temper < JUNC_TEMPER_MIN) {
				sensor_info_ex->sensor_status.temp_check = 2;
				vin_warn("port [%d] temper = %d lower than %d!\n",
					sensor_info->port, temper, JUNC_TEMPER_MIN);
			} else {
				sensor_info_ex->sensor_status.temp_check = 0;
			}
			if (last_temp != sensor_info_ex->sensor_status.temp_check) {
				camera_diag(SENSOR_TEMPER_ID, sensor_info_ex->sensor_status.temp_check,
					sensor_info->port + 1);
			}
		}
	}


	// fps check
	if (sensor_info_ex->diag_mask.sensor_fcnt_test) {
		uint64_t time_diff = 0;
		int32_t fcnt_check_state;
		float fps = 0.0;
		float fps_setting = sensor_info->fps;
		sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
		fcnt_tv_t fcnt_last;
		fcnt_last.tv = sensor_info_ex->fcnt_check.fcnt_tv.tv;
		fcnt_last.fcnt = sensor_info_ex->fcnt_check.fcnt_tv.fcnt;
		fcnt_check_state = sensor_info_ex->sensor_status.fps_check;
		ret = get_sensor_frame_count(sensor_info);
		if (ret < 0) {
			vin_err("senor %s port [%d] get fcnt error\n", sensor_info->sensor_name,
				sensor_info->port);
			return ret;
		}
		time_diff =
			(sensor_info_ex->fcnt_check.fcnt_tv.tv.tv_sec - fcnt_last.tv.tv_sec) * 1000000 +
			(sensor_info_ex->fcnt_check.fcnt_tv.tv.tv_usec - fcnt_last.tv.tv_usec);

		if (time_diff > MONITOR_PERIOD_US) {
			fps = (sensor_info_ex->fcnt_check.fcnt_tv.fcnt - fcnt_last.fcnt) *
				1000000.0 / time_diff;
			vin_warn("port [%d] read fps is %f\n", sensor_info->port, fps);
			if (fps < (fps_setting - FCNT_ERR_RANGE) ||
				fps > (fps_setting + FCNT_ERR_RANGE)) {
				sensor_info_ex->sensor_status.fps_check = 1;
				vin_err("port [%d] fps check error, the setting fps is %f, "
					"while the read fps is %f\n", sensor_info->port, fps_setting, fps);
				vin_err("port [%d] fcnt_last:%d, fcnt:%d, time_diff:%ld\n",
					sensor_info->port, fcnt_last.fcnt,
					sensor_info_ex->fcnt_check.fcnt_tv.fcnt, time_diff);
			} else {
				sensor_info_ex->sensor_status.fps_check = 0;
			}
			if (fcnt_check_state != sensor_info_ex->sensor_status.fps_check) {
				camera_diag(SENSOR_FCNT_ID, sensor_info_ex->sensor_status.fps_check,
					sensor_info->port + 1);
			}
		} else {
			sensor_info_ex->fcnt_check.fcnt_tv = fcnt_last;
		}
	}

	return 0;
}

void *sensor_status_monitor(void *arg) {
	int32_t ret;
	sensor_info_t *sensor_info = arg;
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	char tname[16] = {0};
	snprintf(tname, sizeof(tname), "sen_stat_m%d", sensor_info->port);
	prctl(PR_SET_NAME, tname);
	while (sensor_info_ex->fcnt_check.running) {
		usleep(MONITOR_PERIOD_US);
		ret = sensor_get_status(sensor_info);
		if (ret < 0) {
			vin_err("port [%d] sensor_get_status err\n", sensor_info->port);
		}
		if ((g_sensor_sts_fd[sensor_info->port] > 0) &&
			(g_sensor_sts[sensor_info->port] != NULL)) {
			g_sensor_sts[sensor_info->port]->temperature =
				sensor_info_ex->temperature;
			g_sensor_sts[sensor_info->port]->sensor_status =
				sensor_info_ex->sensor_status.value;
		}
	}
	vin_info("sensor_status_monitor port %d exit done\n", sensor_info->port);
	pthread_exit(NULL);
}

int32_t group_hold_start(hal_control_info_t *info) {
	int32_t ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int32_t crc_last_check[CAM_MAX_NUM];
	int32_t port = dev_port2port[info->port];
	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] clear sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
	}
	/* group hold start */
	setting_size = sizeof(group_hold_start_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(info->bus_num, info->sensor_addr,
		SENSOR_REG_WIDTH, setting_size, group_hold_start_setting);
	if (ret < 0) {
		vin_err("%s info->port [%d] group hold start failed!\n",
			__func__, info->port);
		return ret;
	}
	/* read SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			group_hold_start_setting, crc_array);
		if (len < 0) {
			vin_err("%s info->port [%d] cam_setting_to_crc failed\n",
				__func__, info->port);
		}
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
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
		// only call diag when state changes
		if (crc_last_check[info->port]!= ret) {
			camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[info->port]= ret;
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
			ret = vin_i2c_write_block(info->bus_num, REG_WIDTH_16bit,
						  info->sensor_addr, dst_reg,
						  &data[begin_idx], blk_size);
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

int32_t write_ae_reg(hal_control_info_t *info) {
	int32_t ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int32_t crc_last_check[CAM_MAX_NUM];
	int32_t port = dev_port2port[info->port];
	uint32_t valid_reg_array[BUF_LEN] = {0};
	int32_t i;

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] clear sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
	}
	/* write ae reg */
	setting_size = sizeof(ae_reg_array_base) / sizeof(uint32_t) / 2;
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
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
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
		// only call diag when state changes
		if (crc_last_check[info->port]!= ret) {
			camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[info->port]= ret;
		}
	}
	return 0;
}

int32_t write_awb_reg(hal_control_info_t *info) {
	int32_t ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int32_t crc_last_check[CAM_MAX_NUM];
	int32_t port = dev_port2port[info->port];
	uint32_t valid_reg_array[BUF_LEN] = {0};
	int32_t i;

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] clear sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
	}
	/* write awb reg */
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
			awb_reg_array[info->port], crc_array);
		if (len < 0) {
			vin_err("%s info->port [%d] cam_setting_to_crc failed\n",
				__func__, info->port);
		}
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
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
		// only call diag when state changes
		if (crc_last_check[info->port]!= ret) {
			camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[info->port]= ret;
		}
	}
	return 0;
}

int32_t group_hold_end(hal_control_info_t *info) {
	int32_t ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int32_t crc_last_check[CAM_MAX_NUM];
	int32_t port = dev_port2port[info->port];
	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s info->port [%d] clear sccb crc failed\n",
				__func__, info->port);
			return ret;
		}
	}
	/* group hold end */
	setting_size = sizeof(group_hold_end_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(info->bus_num, info->sensor_addr,
		SENSOR_REG_WIDTH, setting_size, group_hold_end_setting);
	if (ret < 0) {
		vin_err("%s info->port [%d] group hold end failed!\n",
			__func__, info->port);
		return ret;
	}
	/* read SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		len = cam_setting_to_crc(SENSOR_REG_WIDTH, setting_size,
			group_hold_end_setting, crc_array);
		if (len < 0) {
			vin_err("%s info->port [%d] cam_setting_to_crc failed\n",
				__func__, info->port);
		}
		ret = camera_read_retry(info->bus_num, info->sensor_addr,
			REG16_VAL16, OV_SCCB_CRC);
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
		// only call diag when state changes
		if (crc_last_check[info->port]!= ret) {
			camera_diag(SENSOR_I2C_CRC_ID, ret, port + 1);
			crc_last_check[info->port]= ret;
		}
	}
	return 0;
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

int32_t f_sensor_init_global_data(sensor_info_t *sensor_info)
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

int32_t f_sensor_deinit_global_data(sensor_info_t *sensor_info)
{
	int32_t ret;
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

	if(diag_node == NULL || sub_node == NULL) {
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
#endif  // #ifdef CAM_DIAG

#if 0
int32_t sensor_parse_embed_data(sensor_info_t *sensor_info, char* raw_data,
	struct embed_data_info_s* embed_data)
{
	if (sensor_info == NULL || raw_data == NULL || embed_data == NULL) {
		vin_err("%s input para is null\n", __func__);
		return -1;
	}
	embed_data->frame_count = (raw_data[10]) | (raw_data[7] << 8) |
		(raw_data[4] << 16);
	embed_data->line[0] = (raw_data[16]) | (raw_data[13] << 8);
	embed_data->line[2] = (raw_data[22]) | (raw_data[19] << 8);
	embed_data->line[3] = (raw_data[28]) | (raw_data[25] << 8);
	embed_data->exposure =
		(embed_data->line[0] + embed_data->line[3]) * 1000000 /
		tuning_data[sensor_info->dev_port].sensor_data.lines_per_second;
	embed_data->temperature = (raw_data[34]) | (raw_data[31] << 8);
	embed_data->temperature =
		(embed_data->temperature <= 0xC000) ?
		embed_data->temperature : (-1) * (embed_data->temperature - 0xC000);
	embed_data->temperature = embed_data->temperature * 100 / 256;
	vin_info("%s sensor_%d frame_count = %d, dcg_line = %d, spd_line = %d,"
		" vs_line = %d, exposure = %d us, temper = %f\n",
		__func__, sensor_info->port,
		embed_data->frame_count, embed_data->line[0],
		embed_data->line[2], embed_data->line[3], embed_data->exposure,
		embed_data->temperature / 100.0);
}
#endif

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_ECF(ovx3cstd, sensor_emode, sensor_config_index_funcs, CAM_MODULE_FLAG_A16D16);
sensor_module_t ovx3cstd = {
	.module = SENSOR_MNAME(ovx3cstd),
#else
sensor_module_t ovx3cstd = {
	.module = "ovx3cstd",
	.emode = sensor_emode,
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.get_sns_params = get_sns_info,
	.start_control = sensor_start_control,
	.end_control = sensor_end_control,
	.awb_cct_control = sensor_awb_cct_control,
	.aexp_line_gain_control = sensor_aexp_line_gain_control,
	.userspace_control = sensor_userspace_control,
	.stream_off = sensor_stream_off,
	.stream_on = sensor_stream_on,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
#ifdef CAM_DIAG
	.diag_nodes_init = sensor_diag_nodes_init,
#endif
};
