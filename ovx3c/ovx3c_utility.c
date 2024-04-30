/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ovx3c]:" fmt

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
#include "inc/ovx3c_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ov_common_setting.h"
// #include "inc/ds960_setting.h"
// #include "inc/ds954_setting.h"
// #include "inc/ds953_setting.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define VERSION_SENSING     "1.0.0"
#define VERSION_OF_OVX3C    "2.1.0"

#define FPS_HTS
#define BUF_LEN  128
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_SENSOR_ADDR		(0x10)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR	(0x29)
#define DEFAULT_9296_ADDR	    (0x48)
#define DEFAULT_MAX96712_ADDR		(0x29)
#define DEFAULT_MAX9296_ADDR		(0x48)
#define INVALID_CAM_ADDR		(0xFF)

#define EEPROM_I2C_ADDR_ALIAS_ID        (0x51)
#define FRAME_SKIP_COUNT    1u
#define DEFAULT_SENSOR_I2C_ADDR   0x6C
#define LCE_EEPROM_ADDR           0XAE

enum EXP_NUM {
	HDR3 = 3,
	HDR4
};

enum MODE_TYPE {
	NC_DEFAULT_4LANE,
	NC_DEFAULT_2LANE,
	NC_DEFAULT_960_DUAL,
	SENSING_27M,
	SENSING_27M_DUAL,
	SENSING_27M_QUAD,
	PHENIX_24M,
	PHENIX_24M_QUAD,
	PHENIX_24M_DUAL,
	PHENIX_24M_TRIP,
	SENSING_27M_TRIP,
	GAX3C,
	GAX3C_DUAL,
	GAX3C_TRIP,
	GAX3C_QUAD,
	SENSING_SY0820,
	SENSING_WS0820,
	GAX3C_WS0820,
	GAX3C_WITH_WS0820,
	GAX3C_SEPA_WS0820,
	GAX3C_WITH_GA0820,
	GAX3C_SEPA_GA0820,
	SENSING_X8B_WITH_X3C,
	DMS_WITH_SUNNY_X3C_TRIP,
	LCEX3C,
	LCE_X8B_WITH_X3C,
	LCEX3C_QUAD,
	SN_X3C_WITH_X8B,
	DMS_WITH_SENSING_X3C_TRIP,
	SUNNY_X3C_TRIP,
	SENSING_WITH_OVX8B,
	LCE_X3C_WITH_X8B,
	SN_X3C_WITHOUT_X8B,
	SUNNY_X3C_WITH_X8B_DPLL,
	OF_X3C_WITHOUT_X8B,
	OF_X3C_QUAD,
	SN_X3C_WITH_X8B_DUAL,
	OF_OVX3C_24M = GAX3C,
};

enum AWB_TYPE {
	OVX3C = 0,
	OF_OVX3C,
	SN60_OVX3C,
	LCE100_OVX3C,
	LCE60_OVX3C,
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
#ifdef POC_RETRY_POLICY
static int32_t set_gpio(deserial_info_t *deserial_info)
{
	int32_t gpio, ret = -HB_CAM_SENSOR_POWERON_FAIL;

	for(gpio = 0; gpio < deserial_info->gpio_num; gpio++) {
		vin_dbg("Set gpio level is %d for gpio%d\n", deserial_info->gpio_level[gpio], deserial_info->gpio_pin[gpio]);
		if(deserial_info->gpio_pin[gpio] != -1) {
			ret = vin_power_ctrl(deserial_info->gpio_pin[gpio],
					deserial_info->gpio_level[gpio]);
			usleep(100 *1000);
			ret |= vin_power_ctrl(deserial_info->gpio_pin[gpio],
					1-deserial_info->gpio_level[gpio]);
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
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, poc_addr, 0x01, 0x00);
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
		ret = hb_vin_i2c_write_reg8_data8(deserial_if->bus_num, poc_addr, 0x01, 0x1f);
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

/* from j3 auot hb_cam_utility.c */
static int32_t camera_read_retry(int32_t bus, uint32_t i2c_addr, int32_t reg_width, uint32_t reg_addr)
{
    x2_camera_i2c_t i2c_cfg;
    int32_t ret = RET_OK, k;

    i2c_cfg.i2c_addr = i2c_addr;
    i2c_cfg.reg_size = reg_width;
    i2c_cfg.reg = reg_addr;

    k = CAM_I2C_RETRY_MAX;
    do {
        if (i2c_cfg.reg_size == REG16_VAL16) {
            ret = hb_vin_i2c_read_reg16_data16(
                bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
        } else if (i2c_cfg.reg_size == REG16_VAL8) {
            ret = hb_vin_i2c_read_reg16_data8(
                bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
        } else {
            ret = hb_vin_i2c_read_reg8_data8(
                bus, i2c_cfg.i2c_addr, i2c_cfg.reg);
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

uint32_t ae_vs_line_disable = 0;
static uint32_t ae_enable[CAM_MAX_NUM];
static uint32_t awb_enable[CAM_MAX_NUM];
static pthread_t sensor_monitor_tids[CAM_MAX_NUM];
uint32_t ae_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_ae_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t awb_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_awb_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t dev_port2port[CAM_MAX_NUM];
uint32_t name_2a_thread_once[CAM_MAX_NUM];
uint32_t diag_mask[CAM_MAX_NUM];
static int32_t g_sensor_sts_fd[CAM_MAX_NUM];
int32_t extra_mode[CAM_MAX_NUM];
sensor_turning_data_t tuning_data[CAM_MAX_NUM];
sensor_pll_data_t sensor_pll_data;
static sensor_status_info_t* g_sensor_sts[CAM_MAX_NUM];
uint16_t dcg_add_vs_line_max[CAM_MAX_NUM];

static uint32_t again_tmp_buf[CAM_MAX_NUM];
static uint32_t dgain_tmp_buf[CAM_MAX_NUM];
static uint32_t line_tmp_buf[CAM_MAX_NUM];
static uint32_t rgain_tmp_buf[CAM_MAX_NUM];
static uint32_t bgain_tmp_buf[CAM_MAX_NUM];
static uint32_t grgain_tmp_buf[CAM_MAX_NUM];
static uint32_t gbgain_tmp_buf[CAM_MAX_NUM];

typedef struct json_info_s {
	int32_t extra_mode;
	int32_t config_index;
} json_info;

typedef struct ovx3c_info_s
{
	char deserial_name[128];
	json_info sensor_info[CAM_MAX_NUM];
	uint32_t port;
	int32_t deserial_port;
} ovx3c_info;

static ovx3c_info info_for_serdes_link;

int32_t poc_linked_first(int32_t bus, int32_t poc_addr)
{
	int32_t ret = 0, i, val;
	for (i = 0; i < 4; i++) {
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, 0x6 + i);
		usleep(2000);
		/* read again to get current state */
		val = hb_vin_i2c_read_reg8_data8(bus, poc_addr, 0x6 + i);
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
					r = (pdata[i + 2] << 8) | (pdata[i + 3]);
					break;
				case 3:
					r = pdata[i + 2];
					break;
				// case 0:
				default:
					r = 0xffff;
					break;
			}
			if (r == reg) {
				switch (len) {
					case 5:
						pdata[i + 4] = v >> 8;
						pdata[i + 5] = v & 0xff;
						break;
					case 4:
						pdata[i + 4] = v & 0xff;
						break;
					case 3:
						pdata[i + 3] = v & 0xff;
						break;
					// case 0:
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
	uint32_t port = info_for_serdes_link.port;
	int32_t extra_mode_m = info_for_serdes_link.sensor_info[port].extra_mode;
	int32_t config_mode_m = info_for_serdes_link.sensor_info[port].config_index;
	uint16_t link = 0, failed_link = 0;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 5) {
			i2c_slave = pdata[i + 1] >> 1;
			if (sensor_addr != 0 && i2c_slave == DEFAULT_SENSOR_ADDR)
				i2c_slave = sensor_addr;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = (pdata[i + 4] << 8) | pdata[i + 5];
			ret = hb_vin_i2c_write_reg16_data16(bus, i2c_slave, reg_addr, value);
			if (ret < 0) {
				vin_err("write ovx3c %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			// 	usleep(5*1000);
			i = i + len + 1;
			vin_dbg("write ovx3c %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && (i2c_slave == DEFAULT_DESERIAL_ADDR ||
				i2c_slave == DEFAULT_9296_ADDR)) {
				i2c_slave = deserial_addr;
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

			if (!strcmp(info_for_serdes_link.deserial_name, "max96712")) {
				if ((extra_mode_m & EXT_MODE) == GAX3C_QUAD) {
					if ((config_mode_m >> DES_RX_6G) & 0x01) {
						if (reg_addr == 0x10 || reg_addr == 0x11) {
							value = 0x22;
						}
					}
				}
			}
			k = RETRY_TIME_MAX;
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			while (ret < 0 && k--) {
				vin_warn("write serdes %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			}
			if (ret < 0) {
				vin_err("write serdes %d@0x%02x: 0x%04x=0x%02x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				failed_link |= link;
				if (i2c_slave == DEFAULT_SERIAL_ADDR && reg_addr == 0x00) {
					failed_dev = value >> 1;
				} else {
					failed_dev = i2c_slave;
				}
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
				if (poc_addr != 0 && i2c_slave == DEFAULT_POC_ADDR)
					i2c_slave = poc_addr;
				ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
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
			usleep(delay * 1000);
			i = i + 2;
		}
	}
	return ret;
}

int write_register_j5(int bus, uint8_t *pdata, int setting_size)
{
	int ret = RET_OK;
	uint8_t i2c_slave, value;
	uint16_t reg_addr, delay;
	int i, len, k = 10;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, reg_addr, value);
			vin_info("write reg:0x%x  value:%x\n", reg_addr, value);
			while (ret < 0 && k--) {
				vin_info("init serdes reg:0x%x	value:%x  k:%d\n", reg_addr, value, k);
				(void)usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8((uint32_t)bus, i2c_slave, reg_addr, value);
			}
			if (ret < 0) {
				vin_err("init serdes bus %x i2c_slave = %x reg:0x%x value:%x error\n",
					   bus, i2c_slave, reg_addr, value);
				return ret;
			}
			i = i + len + 1;
			vin_info("init serdes bus %x i2c_slave = %x reg:0x%x value:%x\n",
					   bus, i2c_slave, reg_addr, value);
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
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
	int32_t i, len, k = CAM_I2C_RETRY_MAX;

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
			vin_info("write serdes %d@0x%02x: 0x%04x=0x%02x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 0) {
			delay = pdata[i + 1];
			usleep(delay * 1000);
			i = i + 2;
		}
	}
	return ret;
}
#endif

static int32_t max96712_link_enable(uint32_t bus, uint8_t addr, uint8_t link, int32_t flag)
{
	int32_t ret;
	uint16_t reg;
	uint8_t link_cfg;

	reg = 0x6;
	ret = hb_vin_i2c_read_reg16_data8(bus, addr, reg);
	if (ret < 0) {
		vin_err("%s %d access reg 0x%x error\n", __FUNCTION__, __LINE__, reg);
		return -1;
	}

	link_cfg = (uint8_t)ret;
	if (flag == 0) {
		link_cfg &= (~(1U << link));
	} else {
		link_cfg |= (1U << link);
	}

	if (link_cfg == (uint8_t)ret) {
		vin_dbg("%s %d ignore\n", __FUNCTION__, __LINE__);
		return 0;
	}

	ret = hb_vin_i2c_write_reg16_data8(bus, addr, reg, link_cfg);
	if (ret < 0) {
		vin_err("%s %d access reg 0x%x error\n", __FUNCTION__, __LINE__, reg);
		return -1;
	}
	vin_info("%s %d reg 0x%x ret 0x%x\n", __FUNCTION__, __LINE__, reg, ret);

	return 1;
}

static int32_t max96712_link_is_lock(uint32_t bus, uint8_t addr, uint8_t link)
{
	uint16_t link_lock_reg[4] = {0x1a, 0xa, 0xb, 0xc};
	int32_t ret;

	ret = hb_vin_i2c_read_reg16_data8(bus, addr, link_lock_reg[link]);
	if (ret < 0) {
		vin_err("read link %d lock status failed\n", link);
		return -1;
	}

	if (((uint32_t)ret & (1U << 3)) != 0U) {
		return 1;
	} else {
		return 0;
	}
}

static int32_t des_link_enable(deserial_info_t *deserial_if, uint8_t link_port, int32_t flag)
{
	int32_t ret = -1;
	uint32_t bus;
	uint8_t slave_addr;

	slave_addr = (uint8_t)deserial_if->deserial_addr;
	bus = deserial_if->bus_num;

	if (strcmp(deserial_if->deserial_name, "max96712") == 0) {
		ret = max96712_link_enable(bus, slave_addr, link_port, flag);
	} else {
		vin_err("not supported des-%s\n", deserial_if->deserial_name);
		return -1;
	}

	if (ret < 0) {
		vin_err("des-%s link %d %s failed\n", deserial_if->deserial_name,
			link_port, (flag != 0) ? "enable" : "disable");
	} else if (ret == 0) {
		vin_info("des-%s link %d already %s\n", deserial_if->deserial_name,
			link_port, (flag != 0) ? "enable" : "disable");
	} else {
		vin_info("des-%s link %d set %s\n", deserial_if->deserial_name,
			link_port, (flag != 0) ? "enable" : "disable");
	}

	return ret;
}

static int32_t des_check_link_lock(deserial_info_t *deserial_if, uint8_t link_port,
				int32_t timeout_ms, int32_t interval)
{
	int32_t ret = -1;
	uint32_t bus;
	int32_t locked = -1;
	uint8_t slave_addr;
	// MAX96712

	slave_addr = (uint8_t)deserial_if->deserial_addr;
	bus = deserial_if->bus_num;

	do {
		if (strcmp(deserial_if->deserial_name, "max96712") == 0) {
			ret = max96712_link_is_lock(bus, slave_addr, link_port);
		} else {
			vin_err("not supported des-%s\n", deserial_if->deserial_name);
			return -1;
		}

		if (ret > 0) {
			locked = 1;
			break;
		}

		timeout_ms -= interval;
		if (timeout_ms >= 0) {
			(void)usleep((useconds_t)(interval * 1000));
		}
	} while (timeout_ms >= 0);

	return locked;
}

static void serial_reset(sensor_info_t *sensor_info)
{
	uint32_t bus;
	uint8_t addr, def_addr = 0x40;

	bus = sensor_info->bus_num;
	addr = (uint8_t)sensor_info->serial_addr;

	if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_WITHOUT_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_QUAD) {
		def_addr = 0x42;
	}
	// reset
	(void)hb_vin_i2c_write_reg16_data8(bus, def_addr, 0x0010, 0xf1);
	(void)hb_vin_i2c_write_reg16_data8(bus, addr, 0x0010, 0xf1);
	(void)usleep(50*1000);
	// remap i2c addr
	(void)hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
						def_addr, 0x0, DEFAULT_SER_ADDR);
	vin_info("serial_reset bus %d addr 0x%x done\n", bus, addr);

	return;
}

static int32_t des_select_reverse_chn(deserial_info_t *deserial_if, uint8_t link_port)
{
	int32_t ret;
	uint16_t reg = REG_LINK_SET_96712;
	uint8_t val = LINK_ALL_96712;

	if (strcmp(deserial_if->deserial_name, "max96712") == 0) {
		reg = REG_LINK_SET_96712;
		if (link_port < DES_PORT_NUM_MAX) {
			val = (uint8_t)LINK_NONE_96712 & (~(1U << (2U * link_port)));
		} else if (link_port == LINK_ALL) {
			val = LINK_ALL_96712;
		}
	} else {
		return 0;
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, (uint8_t)deserial_if->deserial_addr,
		reg, val);
	if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
			deserial_if->deserial_name);
		return -1;
	}
	return 0;
}

static int32_t link_switch_cold(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = -1, timeout_ms, action = 0;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);   // NOLINT(readability/casting)

	if (link_port == LINK_ALL) {
		(void)des_select_reverse_chn(des, LINK_ALL);
		return 0;
	}

	ret = des_select_reverse_chn(des, link_port);
	if (ret < 0) {
		vin_err("link %d switch reverse failed\n", link_port);
		return -1;
	}

	action = ret = des_link_enable(des, link_port, 1);
	if (ret < 0) {
		(void)des_select_reverse_chn(des, LINK_ALL);
		vin_err("link %d enable failed\n", link_port);
		return -1;
	}

	timeout_ms = 100;
	ret = des_check_link_lock(des, link_port, timeout_ms, 20);
	if (ret < 0) {
		(void)des_select_reverse_chn(des, LINK_ALL);
		vin_err("link %d lock check failed\n", link_port);
		if (action != 0) {
			(void)des_link_enable(des, link_port, 0);
		}
		return -1;
	}

	serial_reset(sensor_info);

	return ret;
}

static int32_t link_switch_hot(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint16_t reg1 = 0;
	uint16_t reg2 = 0;
	uint8_t  val = 0;
	int      val_read1 = 0;
	int      val_read2 = 0;
	uint32_t desport_num = 0;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	desport_num = deserial_if->reserved[0];

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_LINK_SET_9296;
		if (desport_num > 1) {
			if (link_port < DES_PORT_NUM_MAX - 2) {
				val = LINK_NONE_9296 | (1 << link_port);
			} else if (link_port == LINK_ALL) {
				val = LINK_ALL_9296;
			} else {
				vin_err("%s link_port 0x%x not supported for des-%s!\n",
					__func__, link_port, deserial_if->deserial_name);
				return -1;
			}
		} else {
			val = LINK_ALL_9296_ANY;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
			val = LINK_ALL_9296;
			reg = REG_LINK_SET_9296;
			reg1 = REG_LINKA_SET_96718;
			reg2 = REG_LINKB_SET_96718;
			val_read1 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg1);
			if (val_read1 < 0) {
				vin_err("%s read reg 0x%x failed!\n", __func__, reg1);
				return -1;
			}
			val_read2 = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg2);
			if (val_read2 < 0) {
				vin_err("%s read reg 0x%x failed!\n", __func__, reg2);
				return -1;
			}
			if (link_port == 0) {
				val_read1 &= 0xEF;                // open linkA
				val_read2 |= 0x04;                // close linkB
			} else if (link_port == 1) {
				val_read1 |= 0x10;                // close linkA
				val_read2 &= 0xFB;                // open linkB
			} else if (link_port == LINK_ALL) {
				val_read1 &= 0xEF;                // open linkA
				val_read2 &= 0xFB;                // open linkB
			}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		reg = REG_LINK_SET_96712;
		if (desport_num > 1) {
			if (link_port < DES_PORT_NUM_MAX) {
				val = LINK_NONE_96712 & (~(1 << (2 * link_port)));
				// val = LINK_NONE_96712 | (1 << link_port);
			} else if (link_port == LINK_ALL) {
				val = LINK_ALL_96712;
			} else {
				vin_err("%s link_port 0x%x not supported for des-%s!\n", __func__, link_port,
					deserial_if->deserial_name);
				return -1;
			}
		} else {
			val = LINK_ALL_96712;
		}
	} else {
		vin_info("%s not supported des-%s, drop\n", __func__, deserial_if->deserial_name);
		return 0;
	}
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
		reg, val);
	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg1, val_read1);
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg2, val_read2);
	}
	if (ret < 0) {
		vin_err("%s switch to port 0x%x for des-%s failed!\n", __func__, link_port,
			deserial_if->deserial_name);
		return -1;
	}
	vin_info("%s switch to port 0x%x successfully for des-%s!\n",
		__func__, link_port, deserial_if->deserial_name);
	usleep(20 * 1000);
	return 0;
}

static int32_t link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = -1;

	if (sensor_info->start_state == CAM_START) {
		ret = link_switch_hot(sensor_info, link_port);
	} else {
		ret = link_switch_cold(sensor_info, link_port);
	}

	if (ret < 0) {
		vin_err("port %d link_switch %s failed\n", link_port,
			(sensor_info->start_state == CAM_START) ? "hot" : "cold");
	}

	return ret;
}

int32_t get_fcnt(sensor_info_t *sensor_info)
{
	int32_t val = 0, fcnt = 0;
	val = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_VFIFO_FCNT1);
	if (val < 0) {
		vin_err("senor %s read frame counter low bytes error\n",
			sensor_info->sensor_name);
		return val;
	}
	fcnt = val;
	val = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
		sensor_info->sensor_addr, OV_VFIFO_FCNT3);
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
			vin_err("port [%d] fcnt last read = %d, now read = %d, i = %d\n",
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

/**
 * @brief sensor_ovx3c_des_init : write deserial init setting,
 *                                 including 965, 9296, 96712, etc.
 *
 * @param [in] sensor_info : sensor_info parsed from cam json
 *
 * @return ret
 */
int32_t sensor_ovx3c_des_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)sensor_info->deserial_info;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;
	uint32_t *sp_setting = NULL;
	int32_t sp_setting_size = 0;
	int board_type = 0x00;
	board_type = vin_get_board_id();

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	if (deserial_if->init_state == 1)
		return ret;

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
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
			ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
									 setting_size, poc_init_setting);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
			usleep(10 * 1000);
			ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
								 setting_size, poc_init_setting + 2);
			if (ret < 0) {
				vin_err("write poc 0x%02x init setting error\n", poc_addr);
				return ret;
			}
#endif

		} else if (!sensor_info->power_mode) {
			uint8_t *rst_des_pdata = NULL;
			int32_t rst_des_setting_size = 0;
			board_type = vin_get_board_id();
			vin_info("board id: 0x%x\n", board_type);
			if (board_type == BOARD_ID_MATRIXDUO_A_V2 ||
				board_type == BOARD_ID_MATRIXDUO_B_V2 ||
				board_type == BOARD_ID_MATRIXDSOLO_V2 ||
				board_type == BOARD_ID_MATRIXDUO_A_V3 ||
				board_type == BOARD_ID_MATRIXDUO_B_V3 ||
				board_type == BOARD_ID_MATRIXSOLO_V3) {
				if ((sensor_info->extra_mode & 0xff) == SN_X3C_WITH_X8B ||
					(sensor_info->extra_mode & 0xff) == LCE_X8B_WITH_X3C ||
					(sensor_info->extra_mode & 0xff) == SUNNY_X3C_WITH_X8B_DPLL ||
					(sensor_info->extra_mode & 0xff) == SN_X3C_WITH_X8B_DUAL) {
					vin_info("now write max96718_reset_serial_with_3G_init\n");
					rst_des_pdata = max96718_reset_serial_with_3G_init;
					rst_des_setting_size = sizeof(max96718_reset_serial_with_3G_init) / sizeof(uint8_t);
				} else if ((sensor_info->extra_mode & 0xff) == LCE_X3C_WITH_X8B) {
					vin_info("now write max96718_reset_serial_with_3G_linka_init\n");
					rst_des_pdata = max96718_reset_serial_with_3G_linka_init;
					rst_des_setting_size = sizeof(max96718_reset_serial_with_3G_linka_init) / sizeof(uint8_t);
				}
				if (rst_des_pdata != NULL) {
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, rst_des_pdata, rst_des_setting_size);
					if (ret < 0) {
						vin_err("write max96718 reset serial write register init error\n");
					}
				}
			}

			/* reset all serials replace to poc off */
			int32_t i2c_slave;
			for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
				vin_dbg("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
				hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, i2c_slave, 0x0010, 0xf1);
			}
		}

		if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) {
			setting_size = 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
					setting_size, max9296_init_setting);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			usleep(10 * 1000);
			setting_size = sizeof(max9296_init_setting) / sizeof(uint32_t) / 2 - 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
					setting_size, max9296_init_setting + 2);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			pipe_96718 = 0x09;
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
				   (sensor_info->extra_mode & EXT_MODE) == LCEX3C) {
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				sp_setting = max96718_init_setting_ws;
				sp_setting_size = sizeof(max96718_init_setting_ws) / sizeof(uint32_t) / 2 - 1;
			} else {
				sp_setting = max9296_init_setting_ws;
				sp_setting_size = sizeof(max9296_init_setting_ws) / sizeof(uint32_t) / 2 - 1;
			}
			setting_size = 1;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
					setting_size, sp_setting);
			if (ret < 0) {
				vin_err("write max9296_init_setting error\n");
				return ret;
			}
			usleep(10 * 1000);
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
					sp_setting_size, sp_setting + 2);
			if (ret < 0) {
				vin_err("write max9296_init_setting_ws error\n");
				return ret;
			}
			usleep(100 * 1000);
			// borad_type = vin_get_board_id();
			// vin_info("borad id: 0x%x\n", borad_type);
			if (board_type == BOARD_ID_MATRIXDUO_A || board_type == BOARD_ID_MATRIXDUO_A_V2 ||
				board_type == BOARD_ID_MATRIXDUO_B || board_type == BOARD_ID_MATRIXDUO_B_V2 ||
				board_type == BOARD_ID_MATRIXDUO_A_V3 || board_type == BOARD_ID_MATRIXDUO_B_V3) {
				pipe_96718 = 0x36;
			} else {
				pipe_96718 = 0x0A;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
				(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL) {
			pdata = max9296_max9295_dual_init_setting;
			setting_size = sizeof(max9296_max9295_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max9295 dual error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL) {
			pdata = max9296_max96717_dual_init_setting;
			setting_size = sizeof(max9296_max96717_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max96717 dual error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820) {
			pdata = sensing_max9296_max9295_dual_init_setting;
			setting_size = sizeof(sensing_max9296_max9295_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max9295 sy0820 error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820) {
			pdata = weisen_max9296_max9295_dual_init_setting;
			setting_size = sizeof(weisen_max9296_max9295_dual_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max9295 ws0820 error\n");
				return ret;
			}
			pipe_96718 = 0x25;
		} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_X8B_WITH_X3C ||
					(sensor_info->extra_mode & EXT_MODE) == LCE_X8B_WITH_X3C) {
			pipe_96718 = 0x31;
		} else if ((sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
					(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
					(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
			if (!strcmp(deserial_if->deserial_name, "max9296")) {
				pdata = sunny_max9296_max96717_max96717_init_setting;
				setting_size = sizeof(sunny_max9296_max96717_max96717_init_setting)/sizeof(uint8_t);
			} else {
				pdata = sunny_max96718_max96717_max96717_init_setting;
				setting_size = sizeof(sunny_max96718_max96717_max96717_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x31;
			}
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max96717 dual error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == LCE_X3C_WITH_X8B) {
			pdata = lce_max96718_max96717_max96717_init_setting;
			setting_size = sizeof(lce_max96718_max96717_max96717_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max9296_max9295 lce x3c with x8b error\n");
				return ret;
			}
			pipe_96718 = 0x31;
		} else if ((sensor_info->extra_mode & EXT_MODE) == OF_X3C_WITHOUT_X8B) {
			pdata = ofilm_max96718_max96717_init_setting;
			setting_size = sizeof(ofilm_max96718_max96717_init_setting)/sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718_max96717 ofilm x3c error\n");
				return ret;
			}
			pipe_96718 = 0x31;
		} else if ((sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B) {
			pdata = sunny_max96718_init_setting;
			setting_size = sizeof(sunny_max96718_init_setting) / sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96718 snx3c error\n");
				return ret;
			}
			pipe_96718 = 0x31;
		} else {
			vin_err("%s not support extra_mode %d\n", deserial_if->deserial_name,
					sensor_info->extra_mode);
			return -1;
		}
		if (!strcmp(deserial_if->deserial_name, "max96718")) {
			if (pipe_96718 != 0) {
				max9296_add_max96718_init_setting[1] = pipe_96718;
			}
			setting_size = sizeof(max9296_add_max96718_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
					setting_size, max9296_add_max96718_init_setting);
			if (ret < 0) {
				vin_err("write max9296_add_max96718_init_setting error\n");
				return ret;
			}
		}
		if (((sensor_info->extra_mode & EXT_MODE) == SENSING_27M) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) ||
			((sensor_info->extra_mode & EXT_MODE) == LCEX3C) ||
			((sensor_info->extra_mode & EXT_MODE) == GAX3C)) {
			if (sensor_info->config_index & DPHY_COPY) {
				setting_size = sizeof(max9296_phy_portall_init_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
						setting_size, max9296_phy_portall_init_setting);
				if (ret < 0) {
					vin_err("write max9296_phy_portall_init_setting error\n");
					return ret;
				}
			} else if (sensor_info->config_index & DPHY_PORTB) {
				setting_size = sizeof(max9296_phy_portb_init_setting) / sizeof(uint32_t) / 2;
				ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr, SERDES_REG_WIDTH,
						setting_size, max9296_phy_portb_init_setting);
				if (ret < 0) {
					vin_err("write max9296_phy_portall_init_setting error\n");
					return ret;
				}
			}
		}
		if (!strcmp(deserial_if->deserial_name, "max96718") &&
					((sensor_info->extra_mode & 0xff) == LCE_X3C_WITH_X8B ||
					(sensor_info->extra_mode & 0xff) == SN_X3C_WITH_X8B ||
					(sensor_info->extra_mode & 0xff) == SN_X3C_WITHOUT_X8B ||
					(sensor_info->extra_mode & 0xff) == OF_X3C_WITHOUT_X8B ||
					(sensor_info->extra_mode & 0xff) == SUNNY_X3C_WITH_X8B_DPLL ||
					(sensor_info->extra_mode & 0xff) == SN_X3C_WITH_X8B_DUAL)) {  // max96718 DUAL
			if (sensor_info->config_index & PORTA_OUT) {
				pdata = max96718_porta_out_setting;
				setting_size = sizeof(max96718_porta_out_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_porta_out_setting error\n");
					return ret;
				}
			} else if (sensor_info->config_index & PORTB_OUT) {
				pdata = max96718_portb_out_setting;
				setting_size = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_portb_out_setting error\n");
					return ret;
				}
			} else {
				switch (board_type) {
				case BOARD_ID_MATRIXDUO_A:
				case BOARD_ID_MATRIXDUO_A_V2:
				case BOARD_ID_MATRIXDUO_A_V3: {
					pdata = max96718_portb_out_setting;
					setting_size = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
							sensor_addr, pdata, setting_size);
					if (ret < 0) {
						vin_err("write max96718_portb_out_setting error\n");
						return ret;
					}
				}
				break;
				case BOARD_ID_MATRIXDUO_B:
				case BOARD_ID_MATRIXDUO_B_V2:
				case BOARD_ID_MATRIXDUO_B_V3:
				case BOARD_ID_MATRIXDSOLO_V2:
				case BOARD_ID_MATRIXSOLO_V3: {
					pdata = max96718_porta_out_setting;
					setting_size = sizeof(max96718_porta_out_setting)/sizeof(uint8_t);
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
							sensor_addr, pdata, setting_size);
					if (ret < 0) {
						vin_err("write max96718_porta_out_setting error\n");
						return ret;
					}
				}
				break;
				default:
					break;
				}
			}
			if ((sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
				(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
				(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
				pdata = max96717_mfp5_3_errb_mapping_max96718_mfp0_6_setting;
				setting_size = sizeof(max96717_mfp5_3_errb_mapping_max96718_mfp0_6_setting) /
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write bus: %d errb mfp mapping register error\n", bus);
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == LCE_X3C_WITH_X8B) {
				pdata = max96717_mfp6_5_errb_mapping_max96718_mfp0_6_setting;
				setting_size = sizeof(max96717_mfp6_5_errb_mapping_max96718_mfp0_6_setting) /
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write bus: %d errb mfp mapping register error\n", bus);
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
			uint32_t setting_size = 0;
			uint16_t regaddr = 0, offset = 0;
			uint16_t *trigger_reg;

			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				if ((sensor_info->extra_mode & 0xff) == LCE_X3C_WITH_X8B ||
					(sensor_info->extra_mode & 0xff) == SN_X3C_WITH_X8B_DUAL) {
					trigger_reg = lce_max96718_max96717_trigger_mfp;
					setting_size = sizeof(lce_max96718_max96717_trigger_mfp) / sizeof(uint16_t) / 2;
					offset = 0;
				} else {
					trigger_reg = max96718_trigger_mfp;
					setting_size = sizeof(max96718_trigger_mfp) / sizeof(uint16_t) / 2;
					offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
				}
			} else {
				if (deserial_if->mfp_index == 0xffff) {
					trigger_reg = max9296_trigger_mfp5;
					setting_size = sizeof(max9296_trigger_mfp5) / sizeof(uint16_t) / 2;
					offset = 0;
				} else {
					trigger_reg = max9296_trigger_mfp;
					setting_size = sizeof(max9296_trigger_mfp) / sizeof(uint16_t) / 2;
					offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
				}
			}

			for (int32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i] + offset;
				vin_info("trig set deserial mfp%d: i2c%d@0x%02x 0x%04x=0x%02x\n",
					deserial_if->mfp_index, deserial_if->bus_num,
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i+1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s max9296_trig_setting error\n", deserial_if->deserial_name);
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
			case PHENIX_24M:
			case SENSING_27M:
			case LCEX3C:
			case GAX3C:
				ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						deserial_if->deserial_addr, 0x096D, 0x00);
				/* no break */
			case PHENIX_24M_DUAL:
			case SENSING_27M_DUAL:
			case GAX3C_DUAL:
				ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						deserial_if->deserial_addr, 0x09AD, 0x00);
				/* no break */
			case PHENIX_24M_TRIP:
			case SENSING_27M_TRIP:
			case GAX3C_TRIP:
			case SUNNY_X3C_TRIP:
				ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
						deserial_if->deserial_addr, 0x09ED, 0x00);
				/* no break */
				break;
			default:
				break;
			}
			if (sensor_info->width) {
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x0167, sensor_info->width >> 8);
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x0168, sensor_info->width & 0xfF);
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x0169, (4200 - sensor_info->width) >> 8);
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x016A, (4200 - sensor_info->width) & 0xfF);
			}
			if (sensor_info->height) {
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x016B, sensor_info->height >> 8);
					ret |= hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
							deserial_if->deserial_addr, 0x016C, sensor_info->height & 0xfF);
			}
			if (ret < 0) {
				vin_err("write %s_testpattern res register error\n", deserial_if->deserial_name);
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
				ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
										 setting_size, poc_init_setting);
				if (ret < 0) {
					vin_err("write poc 0x%02x init setting error\n", poc_addr);
					return ret;
				}
				usleep(10 * 1000);
				ret = vin_write_array(deserial_if->bus_num, poc_addr, POC_REG_WIDTH,
									 setting_size, poc_init_setting + 2);
				if (ret < 0) {
					vin_err("write poc 0x%02x init setting error\n", poc_addr);
					return ret;
				}
#endif

			} else if (!sensor_info->power_mode) {
				/* reset all serials replace to poc off */
				int32_t i2c_slave;
				if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
					(sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == GAX3C ||
					(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
					(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
					(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP) {
					board_type = vin_get_board_id();
					vin_info("borad id: 0x%x\n", board_type);
					if (board_type == BOARD_ID_MATRIXDUO_A_V2 ||
						board_type == BOARD_ID_MATRIXDUO_B_V2 ||
						board_type == BOARD_ID_MATRIXDSOLO_V2 ||
						board_type == BOARD_ID_MATRIXDUO_A_V3 ||
						board_type == BOARD_ID_MATRIXDUO_B_V3 ||
						board_type == BOARD_ID_MATRIXSOLO_V3) {
						vin_info("now write max96712_reset_serial_with_3G_init\n");
						pdata = max96712_reset_serial_with_3G_init;
						setting_size = sizeof(max96712_reset_serial_with_3G_init) / sizeof(uint8_t);
						ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
							sensor_addr, pdata, setting_size);
						if (ret < 0) {
							vin_err("write max96712 reset serial write register init error\n");
						}
					}
				}
				for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 4); i2c_slave++) {
					vin_dbg("reset serial 0x%02x: 0x0010=0xf1\n", i2c_slave);
					hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, i2c_slave, 0x0010, 0xf1);
				}
			}

			if (((sensor_info->extra_mode & EXT_MODE) == SENSING_27M) ||
				((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) ||
				((sensor_info->extra_mode & EXT_MODE) == LCEX3C) ||
				((sensor_info->extra_mode & EXT_MODE) == GAX3C)) {
				uint8_t poc_first = 0, des_port = 0, des_link_en = 0, des_pipe0_sel = 0;
				if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
					(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) {
					pdata = max96712_max9295_init_setting;
					setting_size = sizeof(max96712_max9295_init_setting)/sizeof(uint8_t);
				} else {
					pdata = max96712_max96717_init_setting;
					setting_size = sizeof(max96712_max96717_init_setting)/sizeof(uint8_t);
				}
				/* auto detect for one link on 96712 */
				if ((poc_addr != INVALID_POC_ADDR) && (sensor_info->deserial_port == 0)) {
					poc_addr = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
					poc_first = poc_linked_first(sensor_info->bus_num, poc_addr);
					if (poc_first >= 3) // fix map for dvb.
						poc_first = (7 - poc_first);
				} else {
					poc_first = sensor_info->deserial_port + 1;
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
					setting_modify(pdata, setting_size, deserial_if->deserial_addr, 0x6, des_link_en);
					setting_modify(pdata, setting_size, deserial_if->deserial_addr, 0xf0, des_pipe0_sel);
				} else {
					vin_dbg("%s config mode!\n", sensor_info->sensor_name);
				}
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write %s 1 port register error\n", sensor_info->sensor_name);
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_QUAD) {
				pdata = max96712_max9295_quad_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_quad_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 quad register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
					(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL) {
				pdata = max96712_max9295_dual_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_dual_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 dual register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP ||
					(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_TRIP) {
				pdata = max96712_max9295_trip_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_trip_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 trip register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD) {
				if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD) {
					pdata = max96712_galaxy_map_setting;
					setting_size = sizeof(max96712_galaxy_map_setting)/sizeof(uint8_t);
				} else if ((sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD) {
					pdata = max96712_lce_map_setting;
					setting_size = sizeof(max96712_lce_map_setting)/sizeof(uint8_t);
				} else if ((sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD) {
					pdata = max96712_ofilm_map_setting;
					setting_size = sizeof(max96712_ofilm_map_setting)/sizeof(uint8_t);
				}
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712 map setting register error\n");
					return ret;
				}
				pdata = max96712_max96717_quad_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_quad_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 quad setting register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL) {
				pdata = max96712_max96717_dual_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_dual_init_setting_4lane)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 dual register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP) {
				pdata = max96712_max96717_trip_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_trip_init_setting_4lane)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 trip register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820) {
				pdata = sensing_max96712_max9295_dual_init_setting;
				setting_size = sizeof(sensing_max96712_max9295_dual_init_setting)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 sy0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820) {
				pdata = weisen_max96712_max9295_dual_init_setting;
				setting_size = sizeof(weisen_max96712_max9295_dual_init_setting)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295 ws0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820) {
				pdata = weisen_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(weisen_max96712_max9295_max96717_init_setting)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 ws0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
					   (sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820) {
				pdata = galaxy_with_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_with_max96712_max9295_max96717_init_setting)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 with ga0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
					   (sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820) {
				pdata = galaxy_sepa_max96712_max9295_max96717_init_setting;
				setting_size = sizeof(galaxy_sepa_max96712_max9295_max96717_init_setting)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max9295_max96717 sepa ga0820 register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SENSING_WITH_OVX8B) {
				pdata = sensing_with_max96712_max9295_max9295_init_setting;
				setting_size = sizeof(sensing_with_max96712_max9295_max9295_init_setting)/
                    sizeof(uint8_t);
			} else if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP) {
			  	pdata = max96712_max96717_ovx3c_max9295_dms_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_ovx3c_max9295_dms_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717_max9295 trip ovx3c and one dms register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP) {
			  	pdata = max96712_max9295_ovx3c_max9295_dms_init_setting_4lane;
				setting_size = sizeof(max96712_max9295_ovx3c_max9295_dms_init_setting_4lane)/
					sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712 trip sensing ovx3c and one dms register error\n");
					return ret;
				}
			} else if ((sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP) {
				pdata = max96712_max96717_trip_sunny_init_setting_4lane;
				setting_size = sizeof(max96712_max96717_trip_sunny_init_setting_4lane)/
				               sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
					sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96712_max96717 trip register error\n");
					return ret;
				}
			} else {
				vin_err("extra_mode %d not supported\n", sensor_info->extra_mode);
				return -1;
			}
			/* i2c addr remap for galaxy */
			if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820) {
				pdata = galaxy_maxser_sensor_i2cmap_setting;
				setting_size = sizeof(galaxy_maxser_sensor_i2cmap_setting)/
					sizeof(uint8_t);
				pdata[1] = (sensor_info->serial_addr - 1) << 1;
				pdata[6] = (sensor_info->serial_addr - 1) << 1;
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
			uint32_t setting_size = sizeof(max96712_trigger_setting_mfp) /
			                        sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
			uint16_t offset = 0, size = 0, regaddr = 0;
			uint16_t *trigger_reg;

			if (deserial_if->mfp_index == 0xffff) {
				trigger_reg = max96712_trigger_setting_mfp14;
				setting_size = sizeof(max96712_trigger_setting_mfp14) / sizeof(uint16_t) / 2;
				offset = 0;
				size = 0;
			} else if ((sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD) {
				trigger_reg = lce_max96712_trigger_setting_mfp;
				setting_size = sizeof(lce_max96712_trigger_setting_mfp) / sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
				offset = deserial_if->mfp_index % MAX96712_MFP_LOOP * setting_size;
				size = deserial_if->mfp_index / MAX96712_MFP_LOOP * MAX96712_MFP_OFFSET;
			} else {
				trigger_reg = max96712_trigger_setting_mfp;
				setting_size = sizeof(max96712_trigger_setting_mfp) / sizeof(uint16_t) / 2 / MAX96712_MFP_LOOP;
				offset = deserial_if->mfp_index % MAX96712_MFP_LOOP * setting_size;
				size = deserial_if->mfp_index / MAX96712_MFP_LOOP * MAX96712_MFP_OFFSET;
			}

			for (int32_t i = 0; i < setting_size; ++i) {
				regaddr = trigger_reg[2*i + 2*offset] + size;
				vin_info("trig set deserial mfp%d: i2c%d@0x%02x 0x%04x=0x%02x\n",
					deserial_if->mfp_index, deserial_if->bus_num,
					deserial_if->deserial_addr, regaddr,
					(uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				      deserial_if->deserial_addr, regaddr,
					  (uint8_t)(trigger_reg[2*i + 2*offset + 1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s max96712_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
		if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD) {
			pdata = max96717_mfp3_errb_mapping_max96712_setting;
			setting_size = sizeof(max96717_mfp3_errb_mapping_max96712_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_max96717 errb mfp3 mapping register error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD) {
			pdata = max96717_mfp6_errb_mapping_max96712_setting;
			setting_size = sizeof(max96717_mfp6_errb_mapping_max96712_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write max96712_max96717 errb mfp6 mapping register error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP) {
			pdata = trip_max96717_mfp3_errb_mapping_max96712_setting;
			setting_size = sizeof(trip_max96717_mfp3_errb_mapping_max96712_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write trip_max96717_max96712 errb mfp3 mapping register error\n");
				return ret;
			}
		}
	} else {
		vin_err("deserial %s not support error\n", deserial_if->deserial_name);
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}
	if ((sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL) {
		pdata = sunny_max96718_1p249Gbps_setting;
		setting_size = sizeof(sunny_max96718_1p249Gbps_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write sunny_max96718_1p249Gbps_setting error\n");
			return ret;
		}
	}
	deserial_if->init_state = 1;
	vin_info("deserial %s init done\n", deserial_if->deserial_name);
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
	turning_data->port = sensor_info->dev_port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	// turning_data->tuning_type = TYPE_USER;
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
	uint16_t x0, y0, x1, y1, width, height;
	uint16_t vts, hts_dcg, hts_s, hts_vs;
	uint16_t pll2_prediv0, pll2_prediv, pll2_mult, pll2_divsyspre, pll2_divsys;
	uint16_t pll2_vco, pll2_sclk, pll2_divsys_index, pll2_prediv_index;
	float row_time, fps;
	float pll2_divsys_array[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};
	float pll2_prediv_array[] = {1, 1.5, 2, 2.5, 3, 4, 6, 8};

	vts = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_VTS);
	dcg_add_vs_line_max[sensor_info->port] = vts - 12;
	turning_data->sensor_data.VMAX  = vts;
	hts_dcg = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_HTS_DCG);

	hts_s = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_HTS_S);

	hts_vs = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_HTS_VS);

	width = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_X_OUTPUT);

	height = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_Y_OUTPUT);

	turning_data->sensor_data.HMAX = hts_dcg + hts_s + hts_vs;
	turning_data->sensor_data.active_width = width;
	turning_data->sensor_data.active_height = height;
	turning_data->sensor_data.gain_max = 1024 * 8192;
	turning_data->sensor_data.analog_gain_max = 1024*8192;
	turning_data->sensor_data.digital_gain_max = 1024*8192;
	if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == LCEX3C ||
		(sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP) {
		turning_data->sensor_data.exposure_time_min = DCG_LINE_MIN_GALAXY;
	} else {
		turning_data->sensor_data.exposure_time_min = DCG_LINE_MIN;
	}
	turning_data->sensor_data.exposure_time_max = vts - 13;
	turning_data->sensor_data.exposure_time_long_max = vts - 13;

	pll2_prediv0 = (hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_PREDIV0) >> 7) + 1;
	pll2_prediv_index = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_PREDIV) & 0x7;
	pll2_mult = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_MULT) & 0x3FF;
	pll2_divsyspre = (hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_DIVSYSPRE) & 0xF) + 1;
	pll2_divsys_index = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_DIVSYS) & 0xF;
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
		if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_WITH_OVX8B) {
			sensor_info->sensor_clk = 27;
		} else {
			sensor_info->sensor_clk = DEFAULT_SENSOR_XCLK;
		}
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
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ovx3c_res_1280x720_init_setting);
		if (ret < 0) {
			vin_err("write ovx3c_res_1280x720_init_setting error\n");
		}
		return ret;
	}

	if (sensor_info->config_index & RES_WIDTH_1920 || sensor_info->width != 0) {
		if (sensor_info->width !=0) {
			start = 968 - (sensor_info->width / 2);
			ovx3c_width_1920_init_setting[1] = start >> 8;
			ovx3c_width_1920_init_setting[3] = start & 0xff;
			ovx3c_width_1920_init_setting[5] = sensor_info->width >> 8;
			ovx3c_width_1920_init_setting[7] = sensor_info->width & 0xff;
		}
		start = (ovx3c_width_1920_init_setting[1] << 8) + ovx3c_width_1920_init_setting[3];
		out = (ovx3c_width_1920_init_setting[5] << 8) + ovx3c_width_1920_init_setting[7];
		vin_dbg("%s width %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,
				out, start, start + out - 1);
		setting_size = sizeof(ovx3c_width_1920_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ovx3c_width_1920_init_setting);
		if (ret < 0) {
			vin_err("write ovx3c_width_1920_init_setting error\n");
		}
	}

	if (sensor_info->config_index & RES_HEIGHT_1080 || sensor_info->height != 0) {
		if (sensor_info->height !=0) {
			start = 648 - (sensor_info->height / 2);
			ovx3c_height_1080_init_setting[1] = start >> 8;
			ovx3c_height_1080_init_setting[3] = start & 0xff;
			ovx3c_height_1080_init_setting[5] = sensor_info->height >> 8;
			ovx3c_height_1080_init_setting[7] = sensor_info->height & 0xff;
		}
		start = (ovx3c_height_1080_init_setting[1] << 8) + ovx3c_height_1080_init_setting[3];
		out = (ovx3c_height_1080_init_setting[5] << 8) + ovx3c_height_1080_init_setting[7];
		vin_dbg("%s height %d [0x%04x,0x%04x]\n", sensor_info->sensor_name,
				out, start, start + out - 1);
		setting_size = sizeof(ovx3c_height_1080_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
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

	setting_size = sizeof(ovx3c_x3_30fps_linear_init_setting)/sizeof(uint32_t)/2;
	vin_dbg("x3 setting_size %d\n", setting_size);
	for(i = 0; i < setting_size; i++) {
		if ((sensor_info->extra_mode & EXT_MASK) == 0) { /* xj3 / j5 */
#ifdef FPS_HTS
			if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_HTS
				&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
				ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 50160 / sensor_info->fps;
#else
			if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_VTS
				&& sensor_info->fps >= 1 && sensor_info->fps <= 30)
				ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 40380 / sensor_info->fps;
#endif
		} else if (((sensor_info->extra_mode & EXT_MASK) >> EXT_OFFS) == 1) { /* zu3 2fps */
			if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_HTS)
				ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 5293;
			else if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_VTS)
				ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = 6375;
		} else if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_VTS) {
			tmp = ((uint32_t)sensor_info->extra_mode) >> 16;
			vin_dbg("%s vts=%d(0x%04x)\n", sensor_info->sensor_name, tmp, tmp);
			ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = tmp;
		} else if (ovx3c_x3_30fps_linear_init_setting[i*2] == OV_HTS) {
			tmp = ((uint32_t)sensor_info->extra_mode) & 0xffff;
			vin_dbg("%s hts=%d(0x%04x)\n", sensor_info->sensor_name, tmp, tmp);
			ovx3c_x3_30fps_linear_init_setting[i*2 + 1] = tmp;
		}
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				ovx3c_x3_30fps_linear_init_setting[i*2], ovx3c_x3_30fps_linear_init_setting[i*2 + 1]);
		if (ret < 0) {
			tmp++;
			if (tmp < 10) {
				i--;
				usleep(10*1000);
				continue;
			}
			vin_err("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__, sensor_info->sensor_name,
				sensor_info->bus_num, sensor_info->sensor_addr,
				i, ovx3c_x3_30fps_linear_init_setting[i*2], ovx3c_x3_30fps_linear_init_setting[i*2 + 1]);
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

	setting_size = sizeof(ovx3c_init_setting_rst)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_rst);
	if (ret < 0) {
		vin_err("senor %s write rst setting error\n", sensor_info->sensor_name);
		return ret;
	}
	usleep(10*1000);
	// write init setting
	setting_size = sizeof(ovx3c_init_setting_hdr4)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_hdr4);
	if (ret < 0) {
		vin_err("senor %s write hdr4 setting error\n", sensor_info->sensor_name);
		return ret;
	}
	if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_WITH_OVX8B) {
		setting_size = sizeof(ovx3c_init_setting_27M)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ovx3c_init_setting_27M);
		if (ret < 0) {
			vin_err("senor %s write 27M pll setting error\n",
					sensor_info->sensor_name);
			return ret;
		}

		if (sensor_info->config_index & PWL_24BIT) {
			vin_dbg("senor %s pwl 24bit\n", sensor_info->sensor_name);
			setting_size = sizeof(sy_ovx3c_pwl_setting_24bit) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
					setting_size, sy_ovx3c_pwl_setting_24bit);
		} else {
			setting_size = sizeof(sy_ovx3c_pwl_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
					setting_size, sy_ovx3c_pwl_setting);
		}
		if (ret < 0) {
			vin_err("senor %s init sy pwl error\n",
				sensor_info->sensor_name);
			return ret;
		}
	} else if ((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_X8B_WITH_X3C ||
			(sensor_info->extra_mode & EXT_MODE) == LCEX3C ||
			(sensor_info->extra_mode & EXT_MODE) == LCE_X8B_WITH_X3C ||
			(sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == LCE_X3C_WITH_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == OF_X3C_WITHOUT_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
		setting_size = sizeof(ovx3c_init_setting_24M)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ovx3c_init_setting_24M);
		if (ret < 0) {
			vin_err("senor %s write 24M pll setting error\n",
					sensor_info->sensor_name);
			return ret;
		}
		if (sensor_info->config_index & PWL_24BIT) {
			vin_dbg("senor %s pwl 24bit\n", sensor_info->sensor_name);
			setting_size = sizeof(sy_ovx3c_pwl_setting_24bit) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
					setting_size, sy_ovx3c_pwl_setting_24bit);
		} else {
			setting_size = sizeof(of_ovx3c_pwl_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
					setting_size, of_ovx3c_pwl_setting);
		}
		if (ret < 0) {
			vin_err("senor %s init of pwl error\n",
				sensor_info->sensor_name);
			return ret;
		}
	}

	extra_mode[sensor_info->port] = (sensor_info->config_index >>
			AWB_CTRL_MODE_OFFS) & AWB_CTRL_MODE_MASK;
	if ((extra_mode[sensor_info->port] & 0xf) == SN60_OVX3C)
		vin_dbg("The pre awb ratio is OX3GB-O060+038\n");
	else if ((extra_mode[sensor_info->port] & 0xf) == OF_OVX3C)
		vin_dbg("The pre awb ratio is OX3GB-A100");
	else
		vin_dbg("The pre awb ratio is default\n");
	if (extra_mode[sensor_info->port] & AWB_CTRL_RATIO_DISABLE)
		vin_dbg("The pre awb is disabled");

	if (sensor_info->fps == 30) {
		setting_size = sizeof(ovx3c_init_setting_1280p_30fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_30fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 25) {
		setting_size = sizeof(ovx3c_init_setting_1280p_25fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_25fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 20) {
		setting_size = sizeof(ovx3c_init_setting_1280p_20fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_20fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 15) {
		setting_size = sizeof(ovx3c_init_setting_1280p_15fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_15fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else if (sensor_info->fps == 10) {
		setting_size = sizeof(ovx3c_init_setting_1280p_10fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_10fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
	} else {
		setting_size = sizeof(ovx3c_init_setting_1280p_30fps)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx3c_init_setting_1280p_30fps);
		if (ret < 0) {
			vin_err("senor %s write resolution=%d--fps=%d setting error\n",
				sensor_info->sensor_name, sensor_info->resolution, sensor_info->fps);
			return ret;
		}
		uint16_t vts_tmp = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
								sensor_info->sensor_addr, OV_VTS);
		vts_tmp = vts_tmp * 30 / sensor_info->fps;

		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr, OV_VTS, vts_tmp);
		if (ret < 0) {
			vin_err("port:%d x3c write %dfps vts error \n", sensor_info->port, sensor_info->fps);
			return ret;
		}
	}

	// fps div.
	if(sensor_info->config_index & FPS_DIV) {
		char init_d[3];
		uint32_t vts_v;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				OV_VTS, init_d, 2);
		vts_v = (init_d[0] << 8) | init_d[1];
		vin_dbg("%dfps settint, vts %d to %d!\n", sensor_info->fps / 2, vts_v, vts_v * 2);
		vts_v *= 2;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				OV_VTS, vts_v);
		if (ret < 0)
			vin_err("write register error\n");
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

	if (sensor_info->config_index & TEST_PATTERN) {
		vin_dbg("ov_test_pattern 0x%04x\n", ov_test_pattern[1]);
		setting_size = sizeof(ov_test_pattern)/sizeof(uint32_t)/2;
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
				setting_size, ov_test_pattern);
		if (ret < 0) {
			vin_err("write ov_test_pattern error\n");
		}
	}

	/* mirror enable */
	if (sensor_info->config_index & MIRROR) {
		int32_t mirror;
		mirror = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_MIRROR_FLIP);
		mirror &= ~BIT(5);
		vin_dbg("ov_mirror_flip 0x%02x\n", mirror);
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
		OV_MIRROR_FLIP, mirror);
		if (ret < 0) {
			vin_err("senor %s write mirror pattern setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
	}

	/* flip enable */
	if (sensor_info->config_index & FLIP) {
		int32_t flip;
		flip = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_MIRROR_FLIP);
		flip |= BIT(2);
		vin_dbg("ov_mirror_flip 0x%02x\n", flip);
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
		OV_MIRROR_FLIP, flip);
		if (ret < 0) {
			vin_err("senor %s write flip pattern setting error\n",
				sensor_info->sensor_name);
			return ret;
		}
	}

#if 0
	/* embedded data enable */
	if (sensor_info->config_index & EMBEDDED_MODE) {
		if (sensor_info->config_index & EMBEDDED_MODE_0) {
			setting_size = sizeof(emb_data_front_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_front_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode 0 setting error\n",
					sensor_info->sensor_name);
				return ret;
			}
		} else if (sensor_info->config_index & EMBEDDED_MODE_1) {
			setting_size = sizeof(emb_data_front_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_front_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode %d setting error\n",
					sensor_info->sensor_name, EMBEDDED_MODE_1);
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
		} else if (sensor_info->config_index & EMBEDDED_MODE_2) {
			setting_size = sizeof(emb_data_back_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_back_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode %d setting error\n",
					sensor_info->sensor_name, EMBEDDED_MODE_2);
				return ret;
			}
		} else if (sensor_info->config_index & EMBEDDED_MODE_3) {
			setting_size = sizeof(emb_data_back_2rows_setting) /
				sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_back_2rows_setting);
			if (ret < 0) {
				vin_err("senor %s write embedded data mode %d setting error\n",
					sensor_info->sensor_name, EMBEDDED_MODE_3);
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
#endif

	/* set x3c trigger mode */
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
		char val[3];
		uint32_t vts_v, init_row_cnt, sync_row_cnt;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr, OV_VTS, val, 2);
		if (ret < 0) {
			vin_err("port_%d read vts error\n", sensor_info->port);
			return ret;
		}
		vts_v = (val[0] << 8) | val[1];
		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num,
				sensor_info->sensor_addr, OV_TC_R_INIT_MAIN, val, 2);
		if (ret < 0) {
			vin_err("port_%d read init_row_cnt error\n", sensor_info->port);
			return ret;
		}
		init_row_cnt = (val[0] << 8) | val[1];
		sync_row_cnt = vts_v - init_row_cnt;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				OV_SYNC_ROW_CNT_ADJ, sync_row_cnt);
		if (ret < 0) {
			vin_err("port_%d write sync_row_cnt error\n", sensor_info->port);
			return ret;
		}
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
	ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
			OV_HTS, xts);
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
		vin_err("camera: write 0x%x block fail\n", sensor_info->sensor_addr);
		return -HB_CAM_I2C_WRITE_BLOCK_FAIL;
	}
	sensor_info->fps = fps;
	sensor_update_fps_notify_driver(sensor_info);
	vin_info("dynamic_switch to %dfps success\n", fps);
	return RET_OK;
}

int sensor_ovx3c_serializer_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint8_t *pdata = NULL;
	int setting_size = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	setting_size = sizeof(serializer_pipez_setting) / sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num, serializer_pipez_setting,
		setting_size);
	if (ret < 0) {
		vin_err("serializer_pipez_setting failed for port%d\n",
			sensor_info->port);
		ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			sensor_info->serial_addr, REG_ALIAS_ID_SER, DEFAULT_SER_ADDR);
		if (ret < 0) {
			vin_err("set alias id to default failed for port%d\n",
				sensor_info->port);
		}
		return -1;
	}

	if ((!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) &&
		(sensor_info->deserial_port == 0)) {
			vin_info("set patch for max9296's second port\n");
			pdata = max9296_dual_setting_patch;
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				pdata[4] = 0x11;
			}
			setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
			ret = write_register_j5(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("max9296_dual_setting_patch failed\n");
				return ret;
			}
	}

	if ((sensor_info->extra_mode & 0xff) == LCEX3C ||
		(sensor_info->extra_mode & 0xff) == LCE_X8B_WITH_X3C ||
		(sensor_info->extra_mode & 0xff) == LCEX3C_QUAD ||
		((sensor_info->extra_mode & 0xff) == LCE_X3C_WITH_X8B)) {
		alias_id_setting[sensor_info->deserial_port][14] = LCE_EEPROM_ADDR;
	}
	alias_id_setting[sensor_info->deserial_port][24] = DEFAULT_SENSOR_I2C_ADDR;
	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num,
		alias_id_setting[sensor_info->deserial_port], setting_size);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}

	usleep(5000);

	/* max9295 need enable LDO */
	if (((sensor_info->extra_mode & 0xff) == SENSING_27M) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_DUAL) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_QUAD) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_TRIP) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_SY0820) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_WITH_OVX8B) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_WS0820) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_X8B_WITH_X3C)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0)
			vin_err("serial enalbe ldo fail!!!\n");
	}
	return ret;
}

int32_t f_sensor_init_global_data(sensor_info_t *sensor_info);
int32_t hotplug_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	int32_t entry_num = sensor_info->entry_num;

	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}

	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	memcpy(bak_awb_reg_array[sensor_info->port], awb_reg_array_base,
	       sizeof(awb_reg_array_base));
	memcpy(awb_reg_array[sensor_info->port], awb_reg_array_base,
	       sizeof(awb_reg_array_base));
	memcpy(bak_ae_reg_array[sensor_info->port], ae_reg_array_base,
	       sizeof(ae_reg_array_base));
	memcpy(ae_reg_array[sensor_info->port], ae_reg_array_base,
	       sizeof(ae_reg_array_base));
	again_tmp_buf[sensor_info->port] = 0;
	dgain_tmp_buf[sensor_info->port] = 0;
	line_tmp_buf[sensor_info->port] = 0;
	rgain_tmp_buf[sensor_info->port] = 0;
	bgain_tmp_buf[sensor_info->port] = 0;
	grgain_tmp_buf[sensor_info->port] = 0;
	gbgain_tmp_buf[sensor_info->port] = 0;

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
	vin_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
			 sensor_info->dev_port, sensor_info->sen_devfd);

	(void)hb_vin_i2c_lock(deserial_if->bus_num);
	if (deserial_if && (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718"))) {
		vin_info("switch to port %d...\n", sensor_info->deserial_port);
		ret = link_switch(sensor_info, sensor_info->deserial_port);
		if (ret < 0) {
			vin_err("link switch to des port_%d failed\n", sensor_info->deserial_port);
			(void)hb_vin_i2c_unlock(deserial_if->bus_num);
			return ret;
		}

		// usleep(100*1000);
		ret = sensor_ovx3c_serializer_init(sensor_info);
		if (ret < 0) {
			(void)link_switch(sensor_info, LINK_ALL);
			vin_err("sensor_ovx3c_serializer_init fail\n");
			(void)hb_vin_i2c_unlock(deserial_if->bus_num);
			return ret;
		}
		if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_WITHOUT_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_QUAD) {
			setting_size = (int32_t)(sizeof(max96717_setting_rclk) / sizeof(uint32_t) / 2U);
			vin_info("sensor without crystal, serializer provide clk\n");
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
							setting_size, max96717_setting_rclk);
			if (ret < 0) {
				vin_err("write max96717_setting_rclk error\n");
			}
		}
	}

	ret = link_switch(sensor_info, LINK_ALL);
	if (ret < 0) {
		vin_err("switch to link all failed for port%d\n",
			sensor_info->port);
	}
	(void)hb_vin_i2c_unlock(deserial_if->bus_num);

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) {
			vin_info("x3c 9295 start init \n");
			setting_size = sizeof(max9295_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_init_setting);
			if (ret < 0) {
				vin_err("write max9295_init_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
				   (sensor_info->extra_mode & EXT_MODE) == LCEX3C) {
			vin_info("x3c 96717 start init \n");
			// sleep 100ms for 9296 to lock 96717
			usleep(100*1000);
			setting_size = sizeof(max96717_init_setting_ws) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max96717_init_setting_ws);
			if (ret < 0) {
				vin_err("write max96717_init_setting_ws error\n");
				return ret;
			}
		}
	}

	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if (((sensor_info->extra_mode & EXT_MODE) == SENSING_27M) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_QUAD) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_TRIP) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820)) {
			setting_size = sizeof(max9295_trigger_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_trigger_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_X8B_WITH_X3C ||
			(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
			setting_size = sizeof(max9295_max96717_trigger_mfp8) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp8\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_max96717_trigger_mfp8);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp8 error\n");
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == LCEX3C ||
					(sensor_info->extra_mode & EXT_MODE) == LCE_X8B_WITH_X3C) {
			setting_size = sizeof(max9295_max96717_trigger_mfp0_setting) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp0_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_max96717_trigger_mfp0_setting);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp0_setting error\n");
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP) {
			setting_size = sizeof(max9295_trigger_setting_mfp7) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_trigger_mfp7_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_trigger_setting_mfp7);
			if (ret < 0) {
				vin_err("write max9295_trigger_mfp7_setting error\n");
			}
		}  else if ((sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_WITHOUT_X8B ||
				   (sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_QUAD) {
			setting_size = (int32_t)(sizeof(max9295_max96717_trigger_mfp0_setting) / sizeof(uint32_t) / 2U);
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp0_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_max96717_trigger_mfp0_setting);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp0_setting error\n");
			}
		}
	}

	vin_info("x3c serializer init done\n");
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	f_sensor_init_global_data(sensor_info);
	return ret;
}

int32_t special_serial_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	int32_t bus;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
		REG_LINK_SET_9296, LINK_ALL_9296);
	if (ret < 0) {
		vin_err("link all failed\n");
		return ret;
	}
	usleep(100*1000);

	ret = hb_vin_i2c_lock(bus);
	if (ret < 0) {
		vin_err("i2c%d lock failed\n", bus);
		return -1;
	}

	vin_info("ovx3c serial addr map\n");
	pdata = linkb_seri2cmap;
	setting_size = sizeof(linkb_seri2cmap)/sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num, linkb_seri2cmap,
		setting_size);
	if (ret < 0) {
		vin_err("linkb_seri2cmap failed for port%d\n",
			sensor_info->port);
		hb_vin_i2c_unlock(bus);
		return ret;
	}

	hb_vin_i2c_unlock(bus);

	setting_size = sizeof(serializer_linkb_pipez_setting) / sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num, serializer_linkb_pipez_setting,
		setting_size);
	if (ret < 0) {
		vin_err("serializer_linkb_pipez_setting failed for port%d\n",
			sensor_info->port);
		return ret;
	}

	setting_size = sizeof(max96717_mfp3_errb_mapping_max96718_mfp6_setting) /
		sizeof(uint8_t);
	ret = write_register_j5(bus, max96717_mfp3_errb_mapping_max96718_mfp6_setting, setting_size);
	if (ret < 0) {
		vin_err("write bus: %d errb mfp mapping register error\n", bus);
		return ret;
	}

	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t entry_num = sensor_info->entry_num;
	if (sensor_info->dev_port < 0) {
		vin_err("%s dev_port must be valid\n", __func__);
		return -1;
	}

	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	dev_port2port[sensor_info->dev_port] = sensor_info->port;
	diag_mask[sensor_info->port] = sensor_info->stream_control;  /* used as diag_mask */
	sensor_info_ex->diag_mask.value = diag_mask[sensor_info->port];
	info_for_serdes_link.port = sensor_info->port;
	strncpy(info_for_serdes_link.deserial_name, deserial_if->deserial_name, 8);
	info_for_serdes_link.deserial_port = sensor_info->deserial_port;
	info_for_serdes_link.sensor_info[sensor_info->port].extra_mode = sensor_info->extra_mode;
	info_for_serdes_link.sensor_info[sensor_info->port].config_index = sensor_info->config_index;
	if (sensor_info_ex->diag_mask.sensor_fcnt_test)
		vin_warn("port [%d] sensor_fcnt_test en\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_group_hold_off)
		vin_warn("port [%d] sensor_group_hold_off en\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_i2c_crc)
		vin_warn("port [%d] sensor_i2c_crc en\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_poc_check)
		vin_warn("port [%d] sensor_poc_check en\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.sensor_temperature)
		vin_warn("port [%d] sensor_temperature en\n", sensor_info->port);
	if (sensor_info_ex->diag_mask.serdes_lock)
		vin_warn("port [%d] serdes_lock en\n", sensor_info->port);
	memcpy(ae_reg_array[sensor_info->dev_port], ae_reg_array_base,
		sizeof(ae_reg_array_base));
	memcpy(awb_reg_array[sensor_info->dev_port], awb_reg_array_base,
		sizeof(awb_reg_array_base));
	name_2a_thread_once[sensor_info->port] = 1;
	ae_enable[sensor_info->dev_port] = (sensor_info->config_index & AE_DISABLE) ?
		(~HAL_AE_LINE_GAIN_CONTROL) : HAL_AE_LINE_GAIN_CONTROL;
	awb_enable[sensor_info->dev_port] = (sensor_info->config_index & AWB_DISABLE) ?
		(~HAL_AWB_CCT_CONTROL) : HAL_AWB_CCT_CONTROL;
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

	if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M ||
		(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820 ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820 ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_WITH_OVX8B ||
		(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
		(sensor_info->extra_mode & EXT_MODE) == SENSING_X8B_WITH_X3C ||
		(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == LCEX3C ||
		(sensor_info->extra_mode & EXT_MODE) == LCE_X8B_WITH_X3C ||
		(sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
		(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD ||
		(sensor_info->config_index & TEST_PATTERN_SERDES) ||
		(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP ||
		(sensor_info->extra_mode & EXT_MODE) == LCE_X3C_WITH_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == OF_X3C_WITHOUT_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
		if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820) {
			entry_num = deserial_if->physical_entry;
			vin_info("sepa config use physical_entry %d\n", entry_num);
		}

		if ((sensor_info->init_state == CAM_STATE_INVALID) &&
			(sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_WITHOUT_X8B) {
			deserial_if->init_state = 0;
			vin_info("%s %d retry sensor_ovx3c_des_init\n", __func__, entry_num);
			ret = sensor_ovx3c_des_init(sensor_info);
			if (ret < 0) {
				vin_err("sensor_ovx3c_des_init fail\n");
				return ret;
			}
		} else {
			req = hb_vin_mipi_pre_request(entry_num, 0, 0);
			vin_dbg("hb_vin_mipi_pre_request req %d\n", req);
			if (req == 0) {
				vin_info("x3c serdes start init \n");
				ret = sensor_ovx3c_des_init(sensor_info);
				(void)hb_vin_mipi_pre_result(entry_num, 0, (uint32_t)ret);
				if (ret < 0) {
					vin_err("sensor_ovx3c_des_init fail\n");
					return ret;
				}
			}
		}
	}

	if ((sensor_info->init_state == CAM_STATE_INVALID) &&
		(sensor_info->extra_mode & EXT_MODE) == (uint32_t)OF_X3C_QUAD) {
		return hotplug_init(sensor_info);
	}

	if ((sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B) {
		ret = special_serial_setting(sensor_info);
		if (ret < 0) {
			vin_err("special_serial_setting error\n");
			return ret;
		}
	}
	// set serializer rclk
	if ((sensor_info->extra_mode & EXT_MODE) == OF_X3C_WITHOUT_X8B ||
		(sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD) {
		setting_size = sizeof(max96717_setting_rclk) / sizeof(uint32_t) / 2;
		vin_info("sensor without crystal, serializer provide clk\n");
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
						setting_size, max96717_setting_rclk);
		if (ret < 0) {
			vin_err("write max96717_setting_rclk error\n");
		}
	}

	if (sensor_info->config_index & TEST_PATTERN_SERDES)
		return ret;

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->extra_mode & EXT_MODE) == SENSING_27M ||
			(sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) {
			vin_info("x3c 9295 start init \n");
			setting_size = sizeof(max9295_init_setting) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_init_setting);
			if (ret < 0) {
				vin_err("write max9295_init_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
				   (sensor_info->extra_mode & EXT_MODE) == LCEX3C) {
			vin_info("x3c 96717 start init \n");
			// sleep 100ms for 9296 to lock 96717
			usleep(100*1000);
			setting_size = sizeof(max96717_init_setting_ws) / sizeof(uint32_t) / 2;
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max96717_init_setting_ws);
			if (ret < 0) {
				vin_err("write max96717_init_setting_ws error\n");
				return ret;
			}
		}
	}

	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if (((sensor_info->extra_mode & EXT_MODE) == SENSING_27M) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_DUAL) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_QUAD) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_27M_TRIP) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_DUAL) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_QUAD) ||
			((sensor_info->extra_mode & EXT_MODE) == PHENIX_24M_TRIP) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_SY0820) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820) ||
			((sensor_info->extra_mode & EXT_MODE) == SENSING_WS0820)) {
			setting_size = sizeof(max9295_trigger_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_trigger_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
				return ret;
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == GAX3C ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_QUAD ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_DUAL ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_WITH_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SENSING_X8B_WITH_X3C ||
			(sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820 ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_WITH_X8B_DPLL ||
			(sensor_info->extra_mode & EXT_MODE) == SUNNY_X3C_TRIP ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITHOUT_X8B ||
			(sensor_info->extra_mode & EXT_MODE) == SN_X3C_WITH_X8B_DUAL) {
			setting_size = sizeof(max9295_max96717_trigger_mfp8) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp8\n",
					sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_max96717_trigger_mfp8);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp8 error\n");
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == LCEX3C_QUAD ||
					(sensor_info->extra_mode & EXT_MODE) == LCEX3C ||
					(sensor_info->extra_mode & EXT_MODE) == LCE_X8B_WITH_X3C ||
					(sensor_info->extra_mode & EXT_MODE) == LCE_X3C_WITH_X8B) {
			setting_size = sizeof(lce_max9295_max96717_trigger_mfp0_setting) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp0_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, lce_max9295_max96717_trigger_mfp0_setting);
			if (ret < 0) {
				vin_err("write lce_max9295_max96717_trigger_mfp0_setting error\n");
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == DMS_WITH_SENSING_X3C_TRIP) {
			setting_size = sizeof(max9295_trigger_setting_mfp7) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_trigger_mfp7_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_trigger_setting_mfp7);
			if (ret < 0) {
				vin_err("write max9295_trigger_mfp7_setting error\n");
			}
		} else if ((sensor_info->extra_mode & EXT_MODE) == OF_X3C_WITHOUT_X8B ||
				   (sensor_info->extra_mode & EXT_MODE) == OF_X3C_QUAD) {
			setting_size = sizeof(max9295_max96717_trigger_mfp0_setting) / sizeof(uint32_t) / 2;
			vin_info("write serial: %d@0x%2x max9295_max96717_trigger_mfp0_setting\n",
					 sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, SERDES_REG_WIDTH,
					setting_size, max9295_max96717_trigger_mfp0_setting);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp0_setting error\n");
			}
		}
	}

	/* max9295 need enable LDO */
	if (((sensor_info->extra_mode & 0xff) == SENSING_27M) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_DUAL) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_QUAD) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_27M_TRIP) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_SY0820) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_WITH_OVX8B) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_WS0820) ||
	    ((sensor_info->extra_mode & 0xff) == SENSING_X8B_WITH_X3C)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}

	vin_info("x3c serializer init done\n");
	ret = sensor_mode_config_init(sensor_info);
	if (ret < 0) {
		vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	}
	f_sensor_init_global_data(sensor_info);
	return ret;
}

int32_t sensor_ovx3c_serdes_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		uint32_t setting_size = sizeof(max9296_start_setting) / sizeof(uint16_t) / 2;
		for (int32_t i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
				max9296_start_setting[2*i], (uint8_t)(max9296_start_setting[2*i+1] & 0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_info("sensor_start write %s successfully\n", deserial_if->deserial_name);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			vin_info("96712 testpattern start\n");
			uint32_t setting_size = sizeof(max96712_tp_start_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_tp_start_setting[2*i], (uint8_t)(max96712_tp_start_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		} else {
			uint32_t setting_size = sizeof(max96712_start_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_start_setting[2*i], (uint8_t)(max96712_start_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
			vin_info("sensor_start write %s successfully\n", deserial_if->deserial_name);
		}
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}
unlock:
	return ret;
}

int32_t sensor_ovx3c_serdes_stream_off(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t value;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -HB_CAM_SERDES_CONFIG_FAIL;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		uint32_t setting_size = sizeof(max9296_stop_setting) / sizeof(uint16_t) / 2;
		for (int32_t i = 0; i < setting_size; ++i) {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
				max9296_stop_setting[2*i], (uint8_t)(max9296_stop_setting[2*i+1] & 0xFF));
			if (ret < 0) {
				vin_err("write %s failed\n", deserial_if->deserial_name);
				goto unlock;
			}
		}
		vin_info("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		if (sensor_info->config_index & TEST_PATTERN_SERDES) {
			uint32_t setting_size = sizeof(max96712_tp_stop_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_tp_stop_setting[2*i], (uint8_t)(max96712_tp_stop_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		} else {
			uint32_t setting_size = sizeof(max96712_stop_setting) / sizeof(uint16_t) / 2;
			for (int32_t i = 0; i < setting_size; ++i) {
				ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
						max96712_stop_setting[2*i], (uint8_t)(max96712_stop_setting[2*i+1] & 0xFF));
				if (ret < 0) {
					vin_err("write %s failed\n", deserial_if->deserial_name);
					goto unlock;
				}
			}
		}
		vin_info("sensor_stop write %s successfully\n", deserial_if->deserial_name);
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}

unlock:
	return ret;
}

void *sensor_status_monitor(void *arg);
int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t setting_size = 0, i, req;
	uint8_t *pdata = NULL;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	int32_t ret = RET_OK, tmp = 0;
	int32_t entry_num = sensor_info->entry_num;

	if (((deserial_if != NULL) &&
		(!strcmp(deserial_if->deserial_name, "max96712") ||
		 !strcmp(deserial_if->deserial_name, "max96722"))) &&
		((sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
		 (sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820)) {
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
		!(sensor_info->config_index & TEST_PATTERN_SERDES)) {
		setting_size = sizeof(ov_stream_on_setting)/sizeof(uint32_t)/2;
		vin_info("%s sensor_start setting_size %d\n",
				sensor_info->sensor_name, setting_size);

		for(i = 0; i < setting_size; i++) {
			(void)hb_vin_i2c_lock(sensor_info->bus_num);
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
					sensor_info->sensor_addr, ov_stream_on_setting[i*2],
					ov_stream_on_setting[i*2 + 1]);
			(void)hb_vin_i2c_unlock(sensor_info->bus_num);
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

	if (deserial_if) {
		if (sensor_info->config_index & DES_STREAMOFF) {
			ret = sensor_ovx3c_serdes_stream_on(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
				vin_err("%d : %s sensor_ovx3c_serdes_stream_on fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
		} else {
			if ((sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_WS0820 ||
				(sensor_info->extra_mode & EXT_MODE) == GAX3C_SEPA_GA0820) {
				entry_num = deserial_if->physical_entry;
				vin_info("sepa config use physical_entry %d\n", entry_num);
			}
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
	}

	if (deserial_if &&
		(sensor_info->config_index & TEST_PATTERN_SERDES)) {
		return ret;
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
	sensor_info_ex->fcnt_check.running = 0;

	return ret;
}

int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, ret1 = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	int32_t setting_size = 0, i;
	uint8_t value;

	sensor_info_ex->fcnt_check.running = 0;
	if (!deserial_if ||
		!(sensor_info->config_index & TEST_PATTERN_SERDES)) {
		setting_size = sizeof(ov_stream_off_setting)/sizeof(uint32_t)/2;
		vin_info("%s sensor_stop setting_size %d\n",
				sensor_info->sensor_name, setting_size);
		for(i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
					sensor_info->sensor_addr, ov_stream_off_setting[i*2],
					ov_stream_off_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
			}
		}
	}
	if (deserial_if) {
		if (sensor_info->config_index & DES_STREAMOFF) {
			ret1 = sensor_ovx3c_serdes_stream_off(sensor_info);
			if (ret1 < 0) {
				ret1 = -HB_CAM_SERDES_STREAM_OFF_FAIL;
				vin_err("%d : %s sensor_ovx3c_serdes_stream_off fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
		}
	}
	pthread_join(sensor_monitor_tids[sensor_info->port], NULL);
	return ret + ret1;
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

	if ((si->extra_mode & EXT_MODE) == SENSING_27M ||
		(si->extra_mode & EXT_MODE) == PHENIX_24M) {
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));
	} else if ((si->extra_mode & EXT_MODE) == GAX3C) {
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
	if (extra_mode[port] & AWB_CTRL_RATIO_DISABLE) {
		awb_alight[0] = 1;
		awb_alight[1] = 1;
		awb_cwf[0] = 1;
		awb_cwf[1] = 1;
		awb_d65[0] = 1;
		awb_d65[1] = 1;
	} else if (extra_mode[port] == OF_OVX3C) {
		awb_alight[0] =
			OF_ALIGHT_SPD_BGAIN / OF_ALIGHT_LCG_BGAIN *
			OF_ALIGHT_LCG_GGAIN / OF_ALIGHT_SPD_GGAIN;
		awb_alight[1] =
			OF_ALIGHT_SPD_RGAIN / OF_ALIGHT_LCG_RGAIN *
			OF_ALIGHT_LCG_GGAIN / OF_ALIGHT_SPD_GGAIN;
		awb_cwf[0] = OF_CWF_SPD_BGAIN / OF_CWF_LCG_BGAIN;
		awb_cwf[1] = OF_CWF_SPD_RGAIN / OF_CWF_LCG_RGAIN;
		awb_d65[0] = OF_D65_SPD_BGAIN / OF_D65_LCG_BGAIN;
		awb_d65[1] = OF_D65_SPD_RGAIN / OF_D65_LCG_RGAIN;
	} else if (extra_mode[port] == SN60_OVX3C) {
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
	} else if (extra_mode[port] == LCE100_OVX3C) {
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
	} else if (extra_mode[port] == LCE60_OVX3C) {
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

	if ((extra_mode[port] & AE_CTRL_MODE_MASK) == OF_OVX3C_24M) {
		hcg_lcg_ratio = OVX3C_HCG_LCG_RATIO_OF;
		lcg_spd_ratio = OVX3C_LCG_SPD_RATIO_OF;
	}
	if (((extra_mode[port] & AE_CTRL_MODE_MASK) == SN60_OVX3C) ||
		((extra_mode[port] & AE_CTRL_MODE_MASK) == LCE60_OVX3C)) {
		lcg_spd_ratio = OVX3C_LCG_SPD_RATIO_SN60;
		lcg_vs_ratio = OVX3C_LCG_VS_RATIO_SN60;
	}
	if ((extra_mode[port] & AE_CTRL_MODE_MASK) == LCE100_OVX3C) {
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
		setting_size = sizeof(ov_stream_off_setting) / sizeof(uint32_t) / 2;
		vin_info("%s sensor_stop setting_size %d\n",
		sensor_info->sensor_name, setting_size);
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, ov_stream_off_setting[i * 2],
				ov_stream_off_setting[i * 2 + 1]);
			if (ret < 0) {
				vin_err("%s stream off failed\n", sensor_info->sensor_name);
				return ret;
			}
		}
	}
	return ret;
}

int32_t sensor_stream_on(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size, i;
	deserial_info_t *deserial_if = NULL;

	deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max96712")) {
		setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 2;
		for (int32_t i = 0; i < setting_size; ++i) {
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
		setting_size = sizeof(ov_stream_on_setting) / sizeof(uint32_t) / 2;
		for (i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
				sensor_info->sensor_addr, ov_stream_on_setting[i * 2],
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
	}
	return ret;
}

static int32_t e2prom_i2c_addr;
int32_t hb_e2prom_read_data(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint64_t *data)
{
	int32_t i, val;
	uint64_t ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
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

int32_t hb_e2prom_read_double(int32_t i2c_num, int32_t base_addr, double *data)
{
	int32_t i, val, temp;
	uint64_t ret = 0;

	if (base_addr == FOV_ADDR_2) {
		temp = 3;
	} else {
		temp = 7;
	}
	for (i = temp; i >= 0; i --) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
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

int32_t hb_e2prom_read_img_info(int32_t i2c_num, int32_t base_addr, uint64_t *data)
{
	int32_t val, ret = 0;

	val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
			base_addr);
	if (val < 0) {
		vin_err("e2prom read img info fail(i2c_num %d addr 0x%x ret %d)\n",
				i2c_num, base_addr, val);
		return -RET_ERROR;
	}
	val &= 0xff;
	ret = val;

	val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
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

int32_t hb_e2prom_read_array(int32_t i2c_num, int32_t byte_num, int32_t base_addr,
		uint8_t *data)
{
	int32_t i, val, ret = 0;
	for (i = 0; i < byte_num; i ++) {
		val = hb_vin_i2c_read_reg16_data8(i2c_num, e2prom_i2c_addr,
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

/**
* PARAMS_2 : get lce params
* GALAXY_PARAMS : get galaxy params
* defualt : get standard params
*/
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
	eeprom_addr_alias_id = EEPROM_I2C_ADDR_ALIAS_ID + si->deserial_port;

	if (si->eeprom_addr == 0) {
		e2prom_i2c_addr = eeprom_addr_alias_id;
	} else {
		e2prom_i2c_addr = si->eeprom_addr;
	}
	if (e2prom_i2c_addr != eeprom_addr_alias_id)
		vin_warn("The eeprom_addr is not default (0x%x)\n", e2prom_i2c_addr);

	memset(sip, 0, sizeof(sensor_intrinsic_parameter_t));
	if (si->extra_mode & PARAMS_2) {
		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_WIDTH_ADDR_2, &data)) < 0)
			return ret;
		sip->image_width = (uint16_t)data;

		if ((ret = hb_e2prom_read_img_info(i2c_num, IMG_HEIGHT_ADDR_2, &data)) < 0)
			return ret;
		sip->image_height = (uint16_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MAJOR_VERSION_ADDR_2, &data)) < 0)
			return ret;
		sip->major_version = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, MINOR_VERSION_ADDR_2, &data)) < 0)
			return ret;
		sip->minor_version = (uint8_t)data;

		if ((ret = hb_e2prom_read_array(i2c_num, 32, MODULE_SERIAL_ADDR_2, sip->serial_num)) < 0)
			return ret;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, VENDOR_ID_ADDR_2, &data)) < 0)
			return ret;
		sip->vendor_id = (uint8_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR_2, &data)) < 0)
			return ret;
		sip->cam_type = (uint8_t)data;
		if (data == 2) {
			sip->cam_type = 0;
		} else if (data == 5) {
			sip->cam_type = 7;
		} else {
			sip->cam_type = (uint8_t)data;
		}

		if ((ret = hb_e2prom_read_double(i2c_num, COD_X_ADDR_2, &sip->center_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, COD_Y_ADDR_2, &sip->center_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_X_ADDR_2, &sip->focal_u)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, EFL_Y_ADDR_2, &sip->focal_v)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, FOV_ADDR_2, &sip->hfov)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K1_ADDR_2, &sip->k1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K2_ADDR_2, &sip->k2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P1_ADDR_2, &sip->p1)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, P2_ADDR_2, &sip->p2)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K3_ADDR_2, &sip->k3)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K4_ADDR_2, &sip->k4)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K5_ADDR_2, &sip->k5)) < 0)
			return ret;
		if ((ret = hb_e2prom_read_double(i2c_num, K6_ADDR_2, &sip->k6)) < 0)
			return ret;
	} else {
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

		if ((ret = hb_e2prom_read_data(i2c_num, 4, MODULE_SERIAL_ADDR, &data)) < 0)
			return ret;
		sip->module_serial = (uint32_t)data;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, CAM_TYPE_ADDR, &data)) < 0)
			return ret;

		if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORTION_FLAG_ADDR, &data)) < 0)
			return ret;
		sip->distortion_flag = (uint8_t)data;

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
		if ((ret = hb_e2prom_read_data(i2c_num, 4, SD_CRC32_GROUP1_ADDR, &data)) < 0)
			return ret;
		sip->crc_group1 = (uint8_t)data;

		if (si->extra_mode & GALAXY_PARAMS) {
			if ((ret = hb_e2prom_read_data(i2c_num, 2, MODULE_ID_ADDR, &data)) < 0)
				return ret;
			sip->module_id = (uint16_t)data;

			if ((ret = hb_e2prom_read_data(i2c_num, 2, YEAR_ADDR, &data)) < 0)
				return ret;
			sip->year = (uint16_t)data;

			if ((ret = hb_e2prom_read_data(i2c_num, 1, MONTH_ADDR, &data)) < 0)
				return ret;
			sip->month = (uint8_t)data;

			if ((ret = hb_e2prom_read_data(i2c_num, 1, DAY_ADDR, &data)) < 0)
				return ret;
			sip->day = (uint8_t)data;

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

			if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_PARAMS_ADDR, &data)) < 0)
				return ret;
			sip->distort_params = (uint8_t)data;

			if ((ret = hb_e2prom_read_data(i2c_num, 1, DISTORT_MODEL_TYPE_ADDR, &data)) < 0)
				return ret;
			sip->distort_model_type = (uint8_t)data;

			if ((ret = hb_e2prom_read_double(i2c_num, PP_X_ADDR, &sip->pp_x)) < 0)
				return ret;
			if ((ret = hb_e2prom_read_double(i2c_num, PP_Y_ADDR, &sip->pp_y)) < 0)
				return ret;
			if ((ret = hb_e2prom_read_double(i2c_num, CAM_SKEW_ADDR, &sip->cam_skew)) < 0)
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

			if ((ret = hb_e2prom_read_data(i2c_num, 4, GALAXY_CRC32_GROUP1_ADDR, &data)) < 0)
				return ret;
			sip->crc_group1 = (uint8_t)data;
		}
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
	{
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
	}

	// stream state check
	{
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
	}

	// temperature check
	{
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
	}

	// fps check
	{
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
			// j = 0;
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
	int32_t i;
	static int32_t reg_state[CAM_MAX_NUM] = {0};
	uint32_t valid_reg_array[BUF_LEN] = {0};

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
	setting_size = sizeof(ae_reg_array_base) / sizeof(ae_reg_array_base[0]) / 2;
	if (reg_state[info->port] == 0) {
		memset(valid_reg_array, -1, sizeof(valid_reg_array));
	} else {
		for (i = 0; i < setting_size; i++) {
			valid_reg_array[i] =
				!!(bak_ae_reg_array[info->port][2 * i + 1] - \
				ae_reg_array[info->port][2 * i + 1]);
			vin_dbg("i %d %d %d delta %d", i,
				bak_ae_reg_array[info->port][2 * i + 1],
				ae_reg_array[info->port][2 * i + 1], valid_reg_array[i]);
		}
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
	reg_state[info->port] = 1;

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
	static int32_t reg_state[CAM_MAX_NUM] = {0};
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
	setting_size = sizeof(awb_reg_array_base) / sizeof(awb_reg_array_base[0]) / 2;

	if (reg_state[info->port] == 0) {
		memset(valid_reg_array, -1, sizeof(valid_reg_array));
	} else {
		for (i = 0; i < setting_size; i++) {
			valid_reg_array[i] =
				!!(awb_reg_array[info->port][i * 2 + 1] -
				bak_awb_reg_array[info->port][i * 2 + 1]);
			vin_dbg("i %d %d %d delta %d", i,
				awb_reg_array[info->port][i * 2 + 1],
				bak_awb_reg_array[info->port][i * 2 + 1],
				valid_reg_array[i]);
		}
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
	reg_state[info->port] = 1;
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
			sizeof(*g_sensor_sts[sensor_info->port]));
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
SENSOR_MODULE_F(ovx3c, CAM_MODULE_FLAG_A16D16);
sensor_module_t ovx3c = {
	.module = SENSOR_MNAME(ovx3c),
#else
sensor_module_t ovx3c = {
	.module = "ovx3c",
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
	.hotplug_init = hotplug_init,
};

