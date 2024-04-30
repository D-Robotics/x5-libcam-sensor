/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ovx8b]:" fmt

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
#include <byteswap.h>
#include "inc/hb_vin.h"
#include "./hb_cam_utility.h"
#include "./hb_i2c.h"
#include "inc/ovx8b_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/ov_common_setting.h"
#include "hb_camera_data_config.h"

#define TUNING_LUT
#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)
#define DEFAULT_SENSOR_ADDR		(0x36)
#define DEFAULT_SERIAL_ADDR_A	(0x62)
#define DEFAULT_SERIAL_ADDR		(0x40)
#define DEFAULT_DESERIAL_ADDR		(0x48)
#define DEFAULT_MAX96712_ADDR		(0x29)
#define DEFAULT_MAX9296_ADDR		(0x48)
#define INVALID_CAM_ADDR		(0xFF)

#define SENSING_LINES_PER_SECOND             (11764)

#define VERSION_SENSING "0.0.1"
#define VERSION_WISSEN  "0.0.2"
#define FRAME_SKIP_COUNT    1
#define BUF_LEN             128
#define SHIFT_ROUNDING      0.5
#define MIN(a, b) ((a) < (b)? (a) : (b))
#define LCE_SERIAL_ADDR     0X84
#define LCE_SENSOR_I2C_ADDR 0x6C
#define LCE_EEPROM_ADDR     0XAE
#define REG_ALIAS_ID_SER    0x0000
#define DEFAULT_SER_ADDR    0x80
#define SER_RATE_REG        0X0001
#define SER_3G_VAL          0X04

enum EXP_NUM {
	HDR3 = 3,
	HDR4
};

enum MODE_TYPE {
	SENSING_24M,
	WISSEN_27M,
	SUNNY_24M,
};

enum AWB_TYPE {
	OVX8B_AWB_DEFT,
	OVX8B_AWB_SN120,
	OVX8B_AWB_SN30,
	OVX8B_AWB_LCE120,
	OVX8B_AWB_LCE30,
};

/* from j3 auto hb_cam_interface.h */
typedef struct sensor_status_info_s {
    int32_t temperature;
    uint32_t sensor_status;
} sensor_status_info_t;

typedef struct frame_count {
	struct timeval tv;
	uint32_t frame_counter;
} frame_count_t;

frame_count_t frame_count_last;
frame_count_t frame_count_cur;

typedef struct fcnt_tv_s {
	struct timeval tv;
	uint32_t fcnt;
} fcnt_tv_t;

typedef struct fcnt_check_s {
	fcnt_tv_t fcnt_tv;
	int32_t running;
} fcnt_check_t;

typedef union sensor_status {
	uint32_t value;
	struct {
		/* sensor status */
		/* 0: normal
		** stream_off = 1: stream off
		** fps_check = 1: fps check fail
		** temp_check = 1: higher then JUNC_TEMPER_MAX
		** temp_check = 2: lower then JUNC_TEMPER_MIN
		** lock_check = 1: unlocked
		** scaler_check = 1: scaler check fail */
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

typedef struct json_info_s {
	int32_t extra_mode;
	int32_t config_index;
} json_info;

typedef struct ovx8b_info_s
{
	char deserial_name[128];
	json_info sensor_info[CAM_MAX_NUM];
	uint32_t port;
	int32_t deserial_port;
} ovx8b_info;

static ovx8b_info info_for_serdes_link;

static sensor_info_ex_t sensor_info_exs[CAM_MAX_NUM];

uint32_t ae_vs_line_disable = 0;
uint32_t ae_enable = HAL_AE_LINE_GAIN_CONTROL;
uint32_t awb_enable = 0;   // HAL_AWB_CCT_CONTROL;
uint32_t ae_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_ae_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t awb_reg_array[CAM_MAX_NUM][BUF_LEN];
uint32_t bak_awb_reg_array[CAM_MAX_NUM][BUF_LEN] = {0};
uint32_t dev_port2port[CAM_MAX_NUM];
uint32_t diag_mask[CAM_MAX_NUM];
uint64_t checksum_flag[CAM_MAX_NUM] = {0};

sensor_turning_data_t turning_data;
sensor_pll_data_t sensor_pll_data;
uint16_t dcg_add_vs_line_max[CAM_MAX_NUM];
uint16_t skip_frame_count[CAM_MAX_NUM] = {0};
int g_sensor_sts_fd[CAM_MAX_NUM];
static sensor_status_info_t* g_sensor_sts[CAM_MAX_NUM];
void *sensor_status_monitor(void *arg);
int group_hold_start(hal_control_info_t *info);
int group_hold_end(hal_control_info_t *info);
int write_ae_reg(hal_control_info_t *info);
int write_awb_reg(hal_control_info_t *info);
int f_sensor_init_global_data(sensor_info_t *sensor_info);
int f_sensor_deinit_global_data(sensor_info_t *sensor_info);
static int32_t get_sensor_ratio_from_otp(hal_control_info_t *info);

int32_t extra_mode[CAM_MAX_NUM];
int32_t config_index[CAM_MAX_NUM];
float hcg_lcg_cg_ratio[CAM_MAX_NUM];	/* hcg/lcg conversion ratio */
float sensitivity_ratio[CAM_MAX_NUM];	/* lpd/spd sensitivity ratio */

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

static uint32_t again_tmp_buf[CAM_MAX_NUM];
static uint32_t dgain_tmp_buf[CAM_MAX_NUM];
static uint32_t line_tmp_buf[CAM_MAX_NUM];
static uint32_t rgain_tmp_buf[CAM_MAX_NUM];
static uint32_t bgain_tmp_buf[CAM_MAX_NUM];
static uint32_t grgain_tmp_buf[CAM_MAX_NUM];
static uint32_t gbgain_tmp_buf[CAM_MAX_NUM];

int32_t special_serial_setting(sensor_info_t *sensor_info);

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
			value = pdata[i + 5];
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			k = 10;
			while (ret < 0 && k--) {
				vin_warn("write sensor %d@0x%02x: 0x%04x=0x%02x ret %d retry %d\n", bus, i2c_slave, reg_addr, value, ret, k);
				usleep(20 * 1000);
				ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			}
			if (ret < 0) {
				vin_err("write x8b %d@0x%02x: 0x%04x=0x%04x error %d\n", bus, i2c_slave, reg_addr, value, ret);
				return ret;
			}
			i = i + len + 1;
			vin_dbg("write x8b %d@0x%02x: 0x%04x=0x%04x\n", bus, i2c_slave, reg_addr, value);
		} else if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			if (deserial_addr != 0 && i2c_slave == DEFAULT_DESERIAL_ADDR) {
				i2c_slave = deserial_addr;
			} else if (serial_addr != 0 && i2c_slave == DEFAULT_SERIAL_ADDR_A) {
				i2c_slave = serial_addr;
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

			if (!strcmp(info_for_serdes_link.deserial_name, "max96718")) {
				if ((config_mode_m & 0xff) == SENSING_X8B_WITH_X3C ||
						(config_mode_m & 0xff) == SUNNY_X8B_WITH_X3C ||
						(config_mode_m & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
						(config_mode_m & 0xff) == SN_X8B_WITH_X3C_DUAL) {
					if (extra_mode_m & DES_RX_6G) {
						if (reg_addr == 0x04) {
							value = 0x02;
						}
					}
				}
			}
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			k = 10;
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
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = pdata[i + 2];
			value = pdata[i + 3];
			if (poc_addr != INVALID_POC_ADDR) {
				if (poc_addr != 0 && i2c_slave == DEFAULT_POC_ADDR)
					i2c_slave = poc_addr;
				ret = hb_vin_i2c_write_reg8_data8(bus, i2c_slave, reg_addr, value);
				if (ret < 0) {
					vin_err("write poc %d@0x%02x: 0x%02x=0x%02x error\n", bus, i2c_slave, reg_addr, value);
					return ret;
				}
				// usleep(100*1000);
				vin_dbg("write poc %d@0x%02x: 0x%02x=0x%02x\n", bus, i2c_slave, reg_addr, value);
			} else {
				if (reg_addr == 0x01 && value == 0x00) {
					/* reset all serials replace to poc off */
					for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 2); i2c_slave++) {
						vin_dbg("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
						hb_vin_i2c_write_reg16_data8(bus, i2c_slave, 0x0010, 0xf1);
					}
				}
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

int write_sensor_reg_block(int bus, int reg_width, int device_addr, uint32_t reg_addr, char *buffer, uint32_t size) {
	int k, ret = 0;
	uint32_t reg_addr_tmp, size_tmp;
	char *buffer_tmp;
	for (int i = 0; i <= size/I2C_BLOCK_MAX; i++) {
		reg_addr_tmp = reg_addr + i*I2C_BLOCK_MAX;
		buffer_tmp = buffer + i*I2C_BLOCK_MAX;
		size_tmp = MIN(I2C_BLOCK_MAX, size - i*I2C_BLOCK_MAX);
		ret = vin_i2c_write_block(bus, reg_width, device_addr, reg_addr_tmp, buffer_tmp, size_tmp);
		k = 10;
		while (ret < 0 && k--) {
			vin_warn("write sensor %d@0x%02x: 0x%04x count %d ret %d retry %d\n",
				bus, device_addr, reg_addr_tmp, size_tmp, ret, k);
			usleep(20 * 1000);
			ret = vin_i2c_write_block(bus, reg_width, device_addr, reg_addr_tmp, buffer_tmp, size_tmp);
		}
		if (ret < 0) {
			vin_err("write sensor %d@0x%02x: 0x%04x size %d fail\n",
				bus, device_addr, reg_addr, size);
			return ret;
		}
	}
	return 0;
}

int write_register_j5(int bus, uint8_t *pdata, int setting_size)
{
	int ret = RET_OK;
	uint8_t i2c_slave;
	uint16_t reg_addr, value, delay;
	int i, len, k = 10;

	for (i = 0; i < setting_size;) {
		len = pdata[i];
		if (len == 4) {
			i2c_slave = pdata[i + 1] >> 1;
			reg_addr = (pdata[i + 2] << 8) | pdata[i + 3];
			value = pdata[i + 4];
			ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
			vin_info("write reg:0x%x  value:%x\n", reg_addr, value);
			while (ret < 0 && k--) {
				vin_info("init serdes reg:0x%x	value:%x  k:%d\n", reg_addr, value, k);
				if (k % 10 == 9) {
					usleep(20 * 1000);
					ret = hb_vin_i2c_write_reg16_data8(bus, i2c_slave, reg_addr, value);
					vin_err("write reg:0x%x  value:%x\n", reg_addr, value);
				}
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

uint8_t one_shot_reset(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	uint16_t reg = 0;
	int val = 0;
	uint16_t rega = 0;
	uint16_t vala = 0;
	uint16_t regb = 0;
	uint16_t valb = 0;

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = MAX9296_ONE_SHOT_RESET_REG;
		val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= 0x20;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		rega = MAX96718_ONE_SHOT_RESETA_REG;
		regb = MAX96718_ONE_SHOT_RESETB_REG;
		vala = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, rega);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, rega);
			return -1;
		}
		valb = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, regb);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, regb);
			return -1;
		}
		vala |= 0x20;
		valb |= 0x20;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			rega, vala);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, rega);
			return -1;
		}
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			regb, valb);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, regb);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		!strcmp(deserial_if->deserial_name, "max96722")) {
		reg = MAX96712_ONE_SHOT_RESET_REG;
		val = hb_vin_i2c_read_reg16_data8(deserial_if->bus_num,
			deserial_if->deserial_addr, reg);
		if (val < 0) {
			vin_err("%s read reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
		val |= 0x0F;
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
			reg, val);
		if (val < 0) {
			vin_err("%s write reg 0x%x failed!\n", __func__, reg);
			return -1;
		}
	}

	return 0;
}

uint8_t link_switch(sensor_info_t *sensor_info, uint8_t link_port)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
	uint16_t reg = 0;
	uint16_t reg1 = 0;
	uint16_t reg2 = 0;
	uint8_t  val = 0;
	int 	 val_read1 = 0;
	int 	 val_read2 = 0;
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
			val_read1 &= 0xEF;				  // open linkA
			val_read2 |= 0x04;				  // close linkB
		} else if (link_port == 1) {
			val_read1 |= 0x10;				  // close linkA
			val_read2 &= 0xFB;				  // open linkB
		} else if (link_port == LINK_ALL) {
			val_read1 &= 0xEF;				  // open linkA
			val_read2 &= 0xFB;				  // open linkB
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

uint8_t rx_rate_switch(sensor_info_t *sensor_info, uint8_t data_rate)
{
	int ret = RET_OK;
	uint16_t reg = 0, value = 0;
	uint8_t  val = 0;
	int      val_read = 0;
	char tx_rate_s[10];
	static int rx_rate_init_once = 0;
	uint32_t desport_num = 0;

	deserial_info_t *deserial_if = NULL;
	deserial_if = (deserial_info_t *)sensor_info->deserial_info;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	desport_num = deserial_if->reserved[0];

	if (data_rate == 1) {
		val = TXRATE_3G;
		memcpy(tx_rate_s, "3G", 3);
	} else if (data_rate == 0) {
		val = TXRATE_6G;
		memcpy(tx_rate_s, "6G", 3);
	} else {
		vin_err("%s data_rate %d not supported for des-%s!\n",
			__func__, data_rate, deserial_if->deserial_name);
		return -1;
	}

	if (!strcmp(deserial_if->deserial_name, "max9296")) {
		reg = REG_TXRATE_9296;
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
			reg = REG_TXRATE_96712;
	} else if (!strcmp(deserial_if->deserial_name, "max96718")) {
		if (sensor_info->deserial_port == 0)
			reg = REG_TXRATE_96718_A;
		else
			reg = REG_TXRATE_96718_B;
	}
	val_read = hb_vin_i2c_read_reg16_data16(deserial_if->bus_num,
		deserial_if->deserial_addr, reg);
	val_read = bswap_16(val_read);
	if (val_read < 0) {
		vin_err("%s read reg 0x%x failed!\n", __func__, reg);
		return -1;
	}
	if (!strcmp(deserial_if->deserial_name, "max9296") ||
			!strcmp(deserial_if->deserial_name, "max96718")) {
		if (desport_num > 1) {
			val |= (val_read & 0xFC);
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, reg, val);
		} else {
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96718_A, val);
			if (ret < 0) {
				vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
				return -1;
			}
			ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96718_B, val);
		}
		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712")) {
		value = val << (sensor_info->deserial_port * 4);
		value |= (val_read & ~(0x3 << (sensor_info->deserial_port * 4)));
		value = bswap_16(value);
		if (desport_num > 1) {
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
				deserial_if->deserial_addr, reg, value);
		} else {
			value &= 0xCCCC;
			value |= (val | (val << 4) | (val << 8) | (val << 12));        // all link swich same rate
			ret = hb_vin_i2c_write_reg16_data16(deserial_if->bus_num,
				deserial_if->deserial_addr, REG_TXRATE_96712, value);
		}

		if (ret < 0) {
			vin_err("%s switch to %s failed!\n", __func__, tx_rate_s);
			return -1;
		}
	}

	vin_info("%s switch to %s successfully for des-%s!\n", __func__, tx_rate_s,
		deserial_if->deserial_name);
	ret = one_shot_reset(deserial_if);
	if (ret < 0) {
		vin_err("one_shot_reset failed!\n");
		return -1;
	}
	usleep(160 * 1000);

	return 0;
}

#if 0
int32_t get_fcnt_val(sensor_info_t *sensor_info)
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

int32_t get_sensor_frame_count(sensor_info_t *sensor_info, frame_count_t *frame) {
	int32_t fcnt, fcnt_init;
	fcnt_init = get_fcnt_val(sensor_info);
	if (fcnt_init < 0) {
		vin_err("%d : get fcnt failed\n", __LINE__);
		return -1;
	}
	for (int32_t i = 0; i < 4; i++) {
		usleep(500);
		fcnt = get_fcnt_val(sensor_info);
		if (fcnt < 0) {
			vin_err("%d : get fcnt failed\n", __LINE__);
			return -1;
		}
		if ((fcnt_init != fcnt) && ((fcnt_init + 1) != fcnt)) {
			vin_err("fcnt last read = %ld, now read = %ld, i = %d\n", fcnt_init, fcnt, i);
			fcnt_init = fcnt;
			continue;
		} else {
			frame->frame_counter = fcnt;
			gettimeofday(&frame->tv, NULL);
			vin_dbg("frame_counter = %d, tv = %ld\n", frame->frame_counter,
				frame->tv.tv_sec * 1000000 + frame->tv.tv_usec);
			return 0;
		}
	}
	vin_err("fcnt reg read err\n");
	return -1;
}
#endif

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

int32_t check_eeprom_vendor_id(sensor_info_t *si)
{
	int32_t i2c_num;
	uint64_t data = 0;
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if;
	uint8_t eeprom_addr_alias_id;

	if (!si) {
		vin_err("input si is null!\n");
		return -RET_ERROR;
	}
	i2c_num = si->bus_num;
	deserial_if = si->deserial_info;
	if ((si->extra_mode & GALAXY_PARAMS) ||
		(si->extra_mode & PARAMS_2) ||
		(si->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		(si->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		(si->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		(si->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		(si->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
		eeprom_addr_alias_id = EEPROM_I2C_ADDR_ALIAS_ID + si->deserial_port;
	} else {
		eeprom_addr_alias_id = DEFAULT_EEPROM_I2C_ADDR + si->deserial_port;
	}
	if (si->eeprom_addr == 0) {
		e2prom_i2c_addr = eeprom_addr_alias_id;
	} else {
		e2prom_i2c_addr = si->eeprom_addr;
	}
	if (e2prom_i2c_addr != eeprom_addr_alias_id)
		vin_warn("The eeprom_addr is not default (0x%x)\n", e2prom_i2c_addr);

	if ((ret = hb_e2prom_read_data(i2c_num, 2, VENDOR_ID_ADDR, &data)) < 0)
		return ret;
	return data;
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
	i2c_num = si->bus_num;
	deserial_if = si->deserial_info;
	if ((si->extra_mode & GALAXY_PARAMS) ||
		(si->extra_mode & PARAMS_2) ||
		(si->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		(si->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		(si->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		(si->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		(si->config_index & 0xff) == LCE_X8B_WITH_X3C ||
		(si->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
		eeprom_addr_alias_id = EEPROM_I2C_ADDR_ALIAS_ID + si->deserial_port;
	} else {
		eeprom_addr_alias_id = DEFAULT_EEPROM_I2C_ADDR + si->deserial_port;
	}
	if (si->eeprom_addr == 0) {
		e2prom_i2c_addr = eeprom_addr_alias_id;
	} else {
		e2prom_i2c_addr = si->eeprom_addr;
	}
	if (e2prom_i2c_addr != eeprom_addr_alias_id)
		vin_warn("The eeprom_addr is not default (0x%x)\n", e2prom_i2c_addr);
	if ((ret = hb_e2prom_read_data(i2c_num, 2, GOLDEN_D65_LCG_COLOR_RATIO_RG, &data)) < 0)
			return ret;
	if ((ret = hb_e2prom_read_data(i2c_num, 2, GOLDEN_D65_LCG_COLOR_RATIO_RG_V2, &data1)) < 0)
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
		if ((ret = hb_e2prom_read_data(i2c_num, 2, indi_reg_base + 2*i, &data)) < 0)
			return ret;
		tmp = data & 0xff;
		data = (data >> 8) | (tmp << 8);
		checksum+=data;
		eeprom_color_ratio[si->dev_port][i] = (float)data / 10000.0;
		vin_dbg("0x%x(0x%lx)\n", indi_reg_base + 2*i, data);
		if ((ret = hb_e2prom_read_data(i2c_num, 2, golden_reg_base + 2*i, &data)) < 0)
			return ret;
		tmp = data & 0xff;
		data = (data >> 8) | (tmp << 8);
		checksum+=data;
		golden_eeprom_color_ratio[si->dev_port][i] = (float)data / 10000.0;
		vin_dbg("0x%x(0x%lx)\n", golden_reg_base + 2*i, data);
	}
	checksum = checksum % 65535 + 1;
	if ((ret = hb_e2prom_read_data(i2c_num, 2, checksum_reg, &data)) < 0)
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

int32_t deserial_serial_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint8_t *pdata = NULL;
	uint8_t i2c_slave;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	uint8_t pipe_96718 = 0;
	uint32_t borad_type = 0x00;

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
			case SENSING_NARROW:
			case SENSING:
				pdata = sensing_max9296_max9295_init_setting;
				setting_size = sizeof(sensing_max9296_max9295_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case SENSING_DUAL:
				pdata = sensing_max9296_max9295_dual_init_setting;
				setting_size = sizeof(sensing_max9296_max9295_dual_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x25;
				break;
			case SENSING_WS0233:
				pdata = sensing_max9296_max9295_max96717_init_setting;
				setting_size = sizeof(sensing_max9296_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case SUNNY_NC120:
			case SUNNY_NC30:
				pdata = sunny_max9296_max96717_init_setting;
				setting_size = sizeof(sunny_max9296_max96717_init_setting)/sizeof(uint8_t);
				break;
			case LCE_120:
			case LCE_30:
				pdata = lce_max9296_max96717_init_setting;
				setting_size = sizeof(lce_max9296_max96717_init_setting)/sizeof(uint8_t);
				break;
			case SENSING_X8B_WITH_X3C:
				pdata = sensing_max96718_max9295_max96717_init_setting;
				setting_size = sizeof(sensing_max96718_max9295_max96717_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x31;
				break;
			case SUNNY:
				pdata = sunny_max96718_max96717_init_setting;
				setting_size = sizeof(sunny_max96718_max96717_init_setting)/sizeof(uint8_t);
				pipe_96718 = 0x09;
				break;
			case SENSING_X8B_RCLK:
				pdata = sensing_max96718_max9295_init_setting;
				setting_size = sizeof(sensing_max96718_max9295_init_setting) / sizeof(uint8_t);
				break;
			case SUNNY_X8B_WITH_X3C:
			case SUNNY_X8B_WITH_X3C_DPLL:
			case SN_X8B_WITH_X3C_DUAL:
				if (!strcmp(deserial_if->deserial_name, "max9296")) {
					pdata = sunny_max9296_max96717_max96717_init_setting;
					setting_size = sizeof(sunny_max9296_max96717_max96717_init_setting)/sizeof(uint8_t);
				} else {
					pdata = sunny_max96718_max96717_max96717_init_setting;
					setting_size = sizeof(sunny_max96718_max96717_max96717_init_setting)/sizeof(uint8_t);
					pipe_96718 = 0x31;
				}
				break;
			case LCE_X8B_WITH_X3C:
				pdata = lce_max96718_max96717_max96717_init_setting;
				setting_size = sizeof(lce_max96718_max96717_max96717_init_setting) / sizeof(uint8_t);
				pipe_96718 = 0x31;
				break;
			case SUNNY_NC120_WITHOUT_X3C:
			case SUNNY_NC30_WITHOUT_X3C:
				pdata = sunny_max96718_init_setting;
				setting_size = sizeof(sunny_max96718_init_setting) / sizeof(uint8_t);
				pipe_96718 = 0x31;
				break;
			case LCE_X3C_WITH_X8B:
				pdata = lce_max96718_max96717_max96717_init_setting_x3c_x8b;
				setting_size = sizeof(lce_max96718_max96717_max96717_init_setting_x3c_x8b) / sizeof(uint8_t);
				pipe_96718 = 0x31;
				break;
			default:
				vin_err("config index %d is err\n", sensor_info->config_index);
				return -RET_ERROR;
		}
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
			   !strcmp(deserial_if->deserial_name, "max96722")) {
		switch (sensor_info->config_index & 0xff) {
			uint8_t poc_a, poc_first, des_port, des_link_en, des_pipe0_sel;
			case SENSING_NARROW:
			case SENSING:
				pdata = max96712_max9295_init_setting;
				setting_size = sizeof(max96712_max9295_init_setting)/sizeof(uint8_t);
				/* auto detect for one link on 96712 */
				if ((poc_addr != INVALID_POC_ADDR) && (sensor_info->deserial_port == 0)) {
					poc_a = (poc_addr) ? poc_addr : DEFAULT_POC_ADDR;
					poc_first = poc_linked_first(bus, poc_a);
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
					vin_dbg("hdr 3exp port-%c config mode!\n", des_port);
					setting_modify(pdata, setting_size, DEFAULT_DESERIAL_ADDR, 0x6, des_link_en);
					setting_modify(pdata, setting_size, DEFAULT_DESERIAL_ADDR, 0xf0, des_pipe0_sel);
				} else {
					vin_dbg("hdr 3exp %s config mode!\n", deserial_if->deserial_name);
				}
				break;
			case SENSING_DUAL:
				pdata = max96712_max9295_dual_init_setting;
				setting_size = sizeof(max96712_max9295_dual_init_setting)/sizeof(uint8_t);
				break;
			case SENSING_WS0233:
				pdata = max96712_max9295_max96717_init_setting;
				setting_size = sizeof(max96712_max9295_max96717_init_setting)/sizeof(uint8_t);
				break;
			case SENSING_WITH_OVX3C:
				pdata = sensing_with_max96712_max9295_max9295_init_setting;
				setting_size = sizeof(sensing_with_max96712_max9295_max9295_init_setting)/sizeof(uint8_t);
				break;
			default:
				vin_err("config index %d is err\n", sensor_info->config_index);
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
			uint8_t *rst_des_pdata = NULL;
			int32_t rst_des_setting_size = 0;
			borad_type = vin_get_board_id();
			vin_info("borad id: 0x%x\n", borad_type);
			if (borad_type == BOARD_ID_MATRIXDUO_A_V2 ||
				borad_type == BOARD_ID_MATRIXDUO_B_V2 ||
				borad_type == BOARD_ID_MATRIXDSOLO_V2 ||
				borad_type == BOARD_ID_MATRIXDUO_A_V3 ||
				borad_type == BOARD_ID_MATRIXDUO_B_V3 ||
				borad_type == BOARD_ID_MATRIXSOLO_V3) {
				if ((sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
					(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
					(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
					(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
					vin_info("now write max96718_reset_serial_with_3G_init\n");
					rst_des_pdata = max96718_reset_serial_with_3G_init;
					rst_des_setting_size = sizeof(max96718_reset_serial_with_3G_init) / sizeof(uint8_t);
				} else if ((sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B) {
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
		for (i2c_slave = DEFAULT_SERIAL_ADDR; i2c_slave <= (DEFAULT_SERIAL_ADDR + 2); i2c_slave++) {
			vin_dbg("reset serial %d@0x%02x: 0x0010=0xf1\n", bus, i2c_slave);
			hb_vin_i2c_write_reg16_data8(bus, i2c_slave, 0x0010, 0xf1);
		}
	}
#endif

	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}

	// ctrl serial mfp pulldown-up x8b rst pin for reset sensor
	if (poc_addr == INVALID_POC_ADDR) {
		if ((sensor_info->config_index & 0xff) == SUNNY ||
			(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
			(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
			(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
				setting_size = sizeof(ctrl_serial_mfp8_reset_ovx8b) / sizeof(uint32_t) / 2;
				vin_dbg("ctrl serial: %d@0x%2x mfp8 reset x8b\n", bus, serial_addr);
				ret = vin_write_array(bus, serial_addr, 2,
								setting_size, ctrl_serial_mfp8_reset_ovx8b);
				if (ret < 0) {
					vin_err("write serial mfp8 reset sensor error\n");
				}
		} else if ((sensor_info->config_index & 0xff) == LCE_120 ||
				(sensor_info->config_index & 0xff) == LCE_30 ||
				(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
				(sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B) {
				setting_size = sizeof(ctrl_serial_mfp0_reset_ovx8b) / sizeof(uint32_t) / 2;
				vin_dbg("ctrl serial: %d@0x%2x mfp0 reset x8b\n", bus, serial_addr);
				ret = vin_write_array(bus, serial_addr, 2,
								setting_size, ctrl_serial_mfp0_reset_ovx8b);
				if (ret < 0) {
					vin_err("write serial mfp0 reset sensor error\n");
				}
		}
	}

	if (!strcmp(deserial_if->deserial_name, "max96718")) {
		pdata = max9296_add_max96718_init_setting;
		setting_size = sizeof(max9296_add_max96718_init_setting)/sizeof(uint8_t);
		if (pipe_96718 != 0) {
			pdata[4] = pipe_96718;
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write max9296_add_max96718_init_setting error\n");
			return ret;
		}

		if ((sensor_info->config_index & 0xff) == SENSING_X8B_WITH_X3C ||
			(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
			(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
			(sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
			(sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
			(sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B ||
			(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
			(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {  // max96718 DUAL
			if (sensor_info->extra_mode & PORTA_OUT) {
				pdata = max96718_porta_out_setting;
				setting_size = sizeof(max96718_porta_out_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_porta_out_setting error\n");
					return ret;
				}
			} else if (sensor_info->extra_mode & PORTB_OUT) {
				pdata = max96718_portb_out_setting;
				setting_size = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
				ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
						sensor_addr, pdata, setting_size);
				if (ret < 0) {
					vin_err("write max96718_portb_out_setting error\n");
					return ret;
				}
			} else {
				borad_type = vin_get_board_id();
				if (borad_type == BOARD_ID_MATRIXDUO_A ||
						borad_type == BOARD_ID_MATRIXDUO_A_V2 ||
						borad_type == BOARD_ID_MATRIXDUO_A_V3) {
					pdata = max96718_portb_out_setting;
					setting_size = sizeof(max96718_portb_out_setting)/sizeof(uint8_t);
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
							sensor_addr, pdata, setting_size);
					if (ret < 0) {
						vin_err("write max96718_portb_out_setting error\n");
						return ret;
					}
				} else if (borad_type == BOARD_ID_MATRIXDUO_B ||
								borad_type == BOARD_ID_MATRIXDUO_B_V2 ||
								borad_type == BOARD_ID_MATRIXDUO_B_V3 ||
								borad_type == BOARD_ID_MATRIXSOLO_V3) {
					pdata = max96718_porta_out_setting;
					setting_size = sizeof(max96718_porta_out_setting)/sizeof(uint8_t);
					ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
							sensor_addr, pdata, setting_size);
					if (ret < 0) {
						vin_err("write max96718_porta_out_setting error\n");
						return ret;
					}
				}
			}
		}
		if ((sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
			(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
			(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
			pdata = max96717_mfp5_3_errb_mapping_max96718_mfp0_6_setting;
			setting_size = sizeof(max96717_mfp5_3_errb_mapping_max96718_mfp0_6_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write bus: %d errb mfp mapping register error\n", bus);
				return ret;
			}
		} else if ((sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C) {
			pdata = max96717_mfp5_6_errb_mapping_max96718_mfp0_6_setting;
			setting_size = sizeof(max96717_mfp5_6_errb_mapping_max96718_mfp0_6_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write bus: %d errb mfp mapping register error\n", bus);
				return ret;
			}
		} else if ((sensor_info->config_index & 0xff) == LCE_120 ||
				   (sensor_info->config_index & 0xff) == LCE_30) {
			pdata = max96717_mfp5_errb_mapping_max96718_mfp0_setting;
			setting_size = sizeof(max96717_mfp5_errb_mapping_max96718_mfp0_setting) /
				sizeof(uint8_t);
			ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
			if (ret < 0) {
				vin_err("write bus: %d errb mfp mapping register error\n", bus);
				return ret;
			}
		}
	}
	if (!strcmp(deserial_if->deserial_name, "max9296")||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		if ((sensor_info->config_index & 0xff) != SENSING_DUAL &&
			(sensor_info->config_index & 0xff) != SENSING_WS0233) {
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
			uint32_t setting_size = 0;
			uint16_t regaddr = 0, offset = 0;
			uint16_t *trigger_reg;

			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				offset = deserial_if->mfp_index * MAX9296_MFP_OFFSET;
				if ((sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C) {
					trigger_reg = lce_max96718_trigger_mfp;
					setting_size = sizeof(lce_max96718_trigger_mfp) / sizeof(uint16_t) / 2;
				} else if ((sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B ||
					(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
					trigger_reg = lce_max96718_max96717_trigger_mfp;
					setting_size = sizeof(lce_max96718_max96717_trigger_mfp) / sizeof(uint16_t) / 2;
					 offset = 0;
				} else {
					trigger_reg = max96718_trigger_mfp;
					setting_size = sizeof(max96718_trigger_mfp) / sizeof(uint16_t) / 2;
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
				if(ret < 0) {
					vin_err("write %s desrial_trig_setting error\n", deserial_if->deserial_name);
				}
			}
		}
	}
	if ((sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL) {
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
	return ret;
}

int32_t ovx8b_init(sensor_info_t *sensor_info)
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

	setting_size = sizeof(ovx8b_init_setting_rst)/sizeof(uint32_t)/2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, SENSOR_REG_WIDTH,
			setting_size, ovx8b_init_setting_rst);
	if (ret < 0) {
		vin_err("senor %s write rst setting error\n", sensor_info->sensor_name);
		return ret;
	}
	usleep(10*1000);

	if(sensor_info->sensor_mode == NORMAL_M) {
		pdata = ovx8b_linear_30fps_init_setting;
		setting_size = sizeof(ovx8b_linear_30fps_init_setting)/sizeof(uint8_t);
		vin_dbg("linear config mode!\n");
	} else if (sensor_info->sensor_mode == DOL2_M) {
		pdata = ovx8b_hdr_2exp_30fps_init_setting;
		setting_size = sizeof(ovx8b_hdr_2exp_30fps_init_setting)/sizeof(uint8_t);
		vin_dbg("hdr 2exp config mode!\n");
	} else if (sensor_info->sensor_mode == PWL_M) {
		pdata = ovx8b_hdr_4exp_30fps_init_setting;
		setting_size = sizeof(ovx8b_hdr_4exp_30fps_init_setting)/sizeof(uint8_t);
		vin_dbg("hdr 4exp config mode!\n");
	} else {
		vin_err("sensor mode %d is err\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}
	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
	if (ret < 0) {
		vin_err("write register error\n");
		return ret;
	}

	// spec sensor setting
	if ((sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL) {
		pdata = ovx8b_hts_vts_spec_setting;
		setting_size = sizeof(ovx8b_hts_vts_spec_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write register error\n");
			return ret;
		}
	}

	// set pre lens
	if ((extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_SN120) {
		pdata = sn120_golden_pre_lens_setting;
		setting_size = sizeof(sn120_golden_pre_lens_setting)/sizeof(uint8_t);
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
		sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("senor %s write sn120 pre_lens error\n", sensor_info->sensor_name);
			return ret;
		}
		ret = write_sensor_reg_block(bus, REG_WIDTH_16bit, sensor_addr, OVX8B_LENS_CORRECT_A,
			sn120_golden_pre_lens_alight, sizeof(sn120_golden_pre_lens_alight));
		if (ret < 0) {
			vin_err("senor %s write sn120_golden_pre_lens_alight error\n",
				sensor_info->sensor_name);
			return ret;
		}
		ret = write_sensor_reg_block(bus, REG_WIDTH_16bit, sensor_addr, OVX8B_LENS_CORRECT_CWF,
			sn120_golden_pre_lens_cwf, sizeof(sn120_golden_pre_lens_cwf));
		if (ret < 0) {
			vin_err("senor %s write sn120_golden_pre_lens_cwf error\n",
				sensor_info->sensor_name);
			return ret;
		}
		ret = write_sensor_reg_block(bus, REG_WIDTH_16bit, sensor_addr, OVX8B_LENS_CORRECT_D65,
			sn120_golden_pre_lens_d65, sizeof(sn120_golden_pre_lens_d65));
		if (ret < 0) {
			vin_err("senor %s write sn120_golden_pre_lens_d65 error\n",
				sensor_info->sensor_name);
			return ret;
		}
	}
	// ov requirement must be enable front 2rows emb data
	// but we set sensor default not output front 2rows emb data
	setting_size = sizeof(emb_data_front_2rows_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr,
				SENSOR_REG_WIDTH, setting_size, emb_data_front_2rows_setting);
	if (ret < 0) {
		vin_err("senor %s write front 2rows emb data setting error\n",
			sensor_info->sensor_name);
		return ret;
	}

	if(sensor_info->sensor_mode == PWL_M) {
		if (sensor_info->config_index & PWL_24BIT) {
			pdata = sunny_ovx8b_pwl_setting_24bit;
			setting_size = sizeof(sunny_ovx8b_pwl_setting_24bit)/sizeof(uint8_t);
			vin_dbg("pwl 24bit init!\n");
		} else {
			pdata = sunny_ovx8b_pwl_setting_20bit;
			setting_size = sizeof(sunny_ovx8b_pwl_setting_20bit)/sizeof(uint8_t);
			vin_dbg("pwl 20bit init!\n");
		}
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write ovx8b pwl register error\n");
			return ret;
		}
	}
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

	// test pattern enable
	if(sensor_info->extra_mode & TEST_PATTERN) {
		pdata = ovx8b_test_pattern;
		setting_size = sizeof(ovx8b_test_pattern)/sizeof(uint8_t);
		vin_dbg("test pattern init!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
	}

	// 1080p?
	if(sensor_info->resolution == 1080) {
#if 0
		pdata = ovx8b_extra_binning_setting;
		setting_size = sizeof(ovx8b_extra_binning_setting)/sizeof(uint8_t);
		vin_dbg("1080p binning settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
#endif
	}

	// fps div.
	if (sensor_info->fps == 15) {
		pdata = ovx8b_fps_div;
		setting_size = sizeof(ovx8b_fps_div)/sizeof(uint8_t);
		vin_info("15 fps settint!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0)
			vin_err("write register error\n");
		#if 0
		char init_d[3];
		uint32_t vts_v;

		ret = hb_vin_i2c_read_block_reg16(sensor_info->bus_num, sensor_info->sensor_addr,
				OVX8B_VTS, init_d, 2);
		vts_v = (init_d[0] << 8) | init_d[1];
		vin_dbg("%dfps settint, vts %d to %d!\n", sensor_info->fps / 2, vts_v, vts_v * 2);
		vts_v *= 2;
		ret = hb_vin_i2c_write_reg16_data16(sensor_info->bus_num, sensor_info->sensor_addr,
				OVX8B_VTS, vts_v);
		if (ret < 0)
			vin_err("write register error\n");
		#endif
	} else if (sensor_info->fps == 10) {
		pdata = ovx8b_init_setting_2160p_10fps;
		setting_size = sizeof(ovx8b_init_setting_2160p_10fps)/sizeof(uint8_t);
		vin_info("ovx8b init 10 fps setting!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write ovx8b 10 fps register error\n");
			return -RET_ERROR;
        }
    } else if (sensor_info->fps == 20) {
		pdata = ovx8b_init_setting_2160p_20fps;
		setting_size = sizeof(ovx8b_init_setting_2160p_20fps)/sizeof(uint8_t);
		vin_info("ovx8b init 20 fps setting!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write ovx8b 20 fps register error\n");
			return -RET_ERROR;
        }
    } else if (sensor_info->fps == 25) {
		pdata = ovx8b_init_setting_2160p_25fps;
		setting_size = sizeof(ovx8b_init_setting_2160p_25fps)/sizeof(uint8_t);
		vin_info("ovx8b init 25 fps setting!\n");
		ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
		if (ret < 0) {
			vin_err("write ovx8b 25 fps register error\n");
			return -RET_ERROR;
        }
    }

	if(sensor_info->extra_mode & FPS_DIV) {
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

	if (sensor_info->config_index & TRIG_STANDARD ||
		sensor_info->config_index & TRIG_SHUTTER_SYNC) {
		setting_size = sizeof(ovx8b_trigger_arbitrary_mode_setting)/sizeof(uint16_t)/2;
		for(int32_t i = 0; i < setting_size; i++) {
			ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num,
			      sensor_info->sensor_addr, ovx8b_trigger_arbitrary_mode_setting[i*2],
				  ovx8b_trigger_arbitrary_mode_setting[i*2 + 1]);
			if (ret < 0) {
				vin_err("%d : standard trigger %s fail\n", __LINE__,
				       sensor_info->sensor_name);
				return ret;
			}
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
		if (ret < 0)
			vin_err("port_%d write sync_row_cnt error\n", sensor_info->port);
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
		sizeof(turning_data->sensor_name)-1);
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
	float row_time, fps;
	float pll2_divsys_array[] = {1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5};

	vts = hb_vin_i2c_read_reg16_data16(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_VTS);
	dcg_add_vs_line_max[sensor_info->port] = vts - 13;
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
	turning_data->sensor_data.exposure_time_min = DCG_SPD_LINE_MIN;
	turning_data->sensor_data.exposure_time_max = vts - 13;
	turning_data->sensor_data.exposure_time_long_max = vts - 13;

	pll2_prediv0 = (hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_PLL2_PREDIV0) >> 7) + 1;
	pll2_prediv = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
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
	/* calculate lines_per_second
	hts = 2132, sclk = 162mhz -->81 for double row time
	row_time = hts/sclk = 26.321us
	lines_per_second = 1/row_time = 37992 */
	if (sensor_info->sensor_clk <= 0)
		sensor_info->sensor_clk = DEFAULT_SENSOR_XCLK;
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
	vin_dbg("HMAX = %d, VMAX = %d, width = %d, height = %d, lines_per_second = %d, xclk = %d, fps = %f\n",
		   turning_data->sensor_data.HMAX, turning_data->sensor_data.VMAX,
		   turning_data->sensor_data.active_width, turning_data->sensor_data.active_height,
		   turning_data->sensor_data.lines_per_second, sensor_info->sensor_clk, fps);

	sensor_data_bayer_fill(&turning_data->sensor_data, 12, BAYER_START_B, BAYER_PATTERN_RGGB);
	if (sensor_info->config_index & PWL_24BIT) {
		sensor_data_bits_fill(&turning_data->sensor_data, 24);
		vin_dbg("sensor data bits fill pwl 24bit\n");
	} else {
		sensor_data_bits_fill(&turning_data->sensor_data, 20);
		vin_dbg("sensor data bits fill pwl 20bit\n");
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

	/* Previously failed sensor */
	if ((sensor_info->init_state == CAM_STATE_INVALID) &&
		(((sensor_info->config_index & 0xFFU) == (uint32_t)SENSING_X8B_RCLK) ||
		((sensor_info->config_index & 0xFFU) == (uint32_t)SUNNY))) {
		((deserial_info_t *)(sensor_info->deserial_info))->init_state = 0;  // NOLINT(readability/casting)
		ret = deserial_serial_setting(sensor_info);
		if(ret < 0) {
			vin_err("deserial_serial_setting failed\n");
			return ret;
		}
	} else {
		req = hb_vin_mipi_pre_request(sensor_info->entry_num, 0, 0);
		vin_dbg("hb_vin_mipi_pre_request req %d\n", req);
		if (req == 0) {
			ret = deserial_serial_setting(sensor_info);
			(void)hb_vin_mipi_pre_result(sensor_info->entry_num, 0, (uint32_t)ret);
			if(ret < 0) {
				vin_err("deserial_serial_setting X3_config fail!\n");
				return ret;
			}
			vin_dbg("deserial_serial_setting X3_config OK!\n");
		}
	}

	if ((sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C) {
		ret = special_serial_setting(sensor_info);
		if (ret < 0) {
			vin_err("special_serial_setting error\n");
			return ret;
		}
	}
	// check eeprom vendor id & set serializer rclk
	if ((sensor_info->config_index & 0xff) == SUNNY_NC120 ||
		(sensor_info->config_index & 0xff) == SUNNY_NC30 ||
		(sensor_info->config_index & 0xff) == SUNNY ||
		(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
		vendor_id = check_eeprom_vendor_id(sensor_info);
		vin_dbg("read eeprom vendor id = %x\n", vendor_id);
		if (vendor_id == WITHOUT_CRYSTAL) {
			setting_size = sizeof(max96717_setting_rclk) / sizeof(uint32_t) / 2;
			vin_dbg("sensor without crystal, serializer provide clk\n");
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								setting_size, max96717_setting_rclk);
			if (ret < 0) {
				vin_err("write max96717_setting_rclk error\n");
			}
		}
	} else if ((sensor_info->config_index & 0xff) == SENSING_X8B_RCLK) {
		setting_size = sizeof(max9295_setting_rclk) / sizeof(uint32_t) / 2;
		vin_info("sensor without crystal, serializer max9295 provide clk\n");
		ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
						setting_size, max9295_setting_rclk);
		if (ret < 0) {
			vin_err("write max9295_setting_rclk error\n");
		}
	}
	// check eeprom wb color ratio
	if ((extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_SN120 ||
		(extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_SN30 ||
		(extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_LCE120 ||
		(extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_LCE30) {
		ret = check_eeprom_wb_ratio(sensor_info);
		if (ret < 0) {
			vin_err("sensor %s get_eeprom_wb_ratio fail\n", sensor_info->sensor_name);
			return ret;
		}
	}

	/* serial sync config */
	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if ((sensor_info->config_index & 0xff) == SENSING_NARROW ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC120 ||
		 (sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		 (sensor_info->config_index & 0xff) == SENSING_X8B_WITH_X3C ||
		 (sensor_info->config_index & 0xff) == SENSING_X8B_RCLK ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC30 ||
		 (sensor_info->config_index & 0xff) == SUNNY) {
			setting_size = sizeof(max9295_trigger_mfp7_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig7\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_trigger_mfp7_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
			}
		} else if ((sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
			setting_size = sizeof(max9295_max96717_trigger_mfp8_setting_sync) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max96717 trig8\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_max96717_trigger_mfp8_setting_sync);
			if (ret < 0) {
				vin_err("write max9295_max96717_trigger_mfp8_setting_sync error\n");
			}
		} else {
			setting_size = sizeof(max9295_trigger_mfp8_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig8\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_trigger_mfp8_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
			}
		}
	}
	/* max9295 need enable LDO */
	if (((sensor_info->config_index & 0xff) == SENSING) ||
	    ((sensor_info->config_index & 0xff) == SENSING_24M) ||
	    ((sensor_info->config_index & 0xff) == SENSING_DUAL) ||
	    ((sensor_info->config_index & 0xff) == SENSING_WITH_OVX3C) ||
	    ((sensor_info->config_index & 0xff) == SENSING_WS0233) ||
	    ((sensor_info->config_index & 0xff) == SENSING_X8B_RCLK) ||
	    ((sensor_info->config_index & 0xff) == SENSING_NARROW) ||
	    ((sensor_info->config_index & 0xff) == SENSING_X8B_WITH_X3C)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}

	ret = ovx8b_init(sensor_info);
	if(ret < 0) {
		vin_err("OVX8B_X3_config fail!\n");
		return ret;
	}
	vin_dbg("OVX8B_X3_config OK!\n");

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
	return ret;
}

int sensor_ovx8b_serializer_init(sensor_info_t *sensor_info)
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
	if ((sensor_info->config_index & 0xff) == LCE_120 ||
		(sensor_info->config_index & 0xff) == LCE_30 ||
		(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
		(sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B) {
		for (int i = 0; i < setting_size; i += 5) {
			serializer_pipez_setting[i + 1] = LCE_SERIAL_ADDR;
		}
	}
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
		return ret;
	}

	if ((!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) &&
		(sensor_info->deserial_port == 0)) {
			vin_info("set patch for max9296's second port\n");
			setting_size = sizeof(max9296_dual_setting_patch) / sizeof(uint8_t);
			if ((sensor_info->config_index & 0xff) == LCE_120 ||
				(sensor_info->config_index & 0xff) == LCE_30 ||
				(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
				(sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B) {
				for (int i = 0; i < setting_size - 2; i += 5) {
					max9296_dual_setting_patch[i + 1] = LCE_SERIAL_ADDR;
				}
			}
			pdata = max9296_dual_setting_patch;
			if (!strcmp(deserial_if->deserial_name, "max96718")) {
				pdata[4] = 0x11;
			}
			ret = write_register_j5(deserial_if->bus_num, pdata, setting_size);
			if (ret < 0) {
				vin_err("max9296_dual_setting_patch failed\n");
				return ret;
			}
	}
	if ((sensor_info->config_index & 0xff) == LCE_120 ||
		(sensor_info->config_index & 0xff) == LCE_30 ||
		(sensor_info->config_index & 0xff) == LCE_X8B_WITH_X3C ||
		(sensor_info->config_index & 0xff) == LCE_X3C_WITH_X8B) {
		alias_id_setting[sensor_info->deserial_port][1] = LCE_SERIAL_ADDR;
		alias_id_setting[sensor_info->deserial_port][14] = LCE_EEPROM_ADDR;
		alias_id_setting[sensor_info->deserial_port][24] = LCE_SENSOR_I2C_ADDR;
	}
	setting_size = sizeof(alias_id_setting[0]) / sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num,
		alias_id_setting[sensor_info->deserial_port], setting_size);
	if (ret < 0) {
		vin_err("alias_id_setting failed\n");
		return ret;
	}

	usleep(5000);
	vin_info("sensor %s serializer init done\n", sensor_info->sensor_name);
	return ret;
}

int32_t hotplug_init(sensor_info_t *sensor_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	int32_t entry_num = sensor_info->entry_num;
	uint16_t reg = 0;
	pthread_t t1;
	int32_t vendor_id = 0;
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

	ret = link_switch(sensor_info, sensor_info->deserial_port);
	if (ret < 0) {
		vin_err("link switch to port_%d failed\n", sensor_info->deserial_port);
		return ret;
	}
	if (sensor_info->fps == 15 &&
		!strcmp(deserial_if->deserial_name, "max9296")) {
		ret = rx_rate_switch(sensor_info, 0);
		if (ret < 0) {
			vin_err("ovx8b set 6g rx rate fail");
			return ret;
		}
		// usleep(100*1000);
		if ((sensor_info->config_index & 0xff) == LCE_120 ||
			(sensor_info->config_index & 0xff) == LCE_30) {
			reg = LCE_SERIAL_ADDR >> 1;
		} else {
			reg = DEFAULT_SER_ADDR >> 1;
		}
		ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, reg,
			SER_RATE_REG, SER_3G_VAL);
		if (ret < 0) {
			vin_err("write serializer rate reg failed\n");
			return -1;
		}
		ret = rx_rate_switch(sensor_info, 1);
		if (ret < 0) {
			vin_err("ovx8b set 3g rx rate fail");
			return ret;
		}
	}
	// usleep(100*1000);
	ret = sensor_ovx8b_serializer_init(sensor_info);
	if (ret < 0) {
		vin_err("sensor_ovx8b_serializer_init fail\n");
		return ret;
	}

	if ((sensor_info->config_index & 0xff) != LCE_120) {
		ret = link_switch(sensor_info, LINK_ALL);
		if (ret < 0) {
			vin_err("switch to link all failed for port%d\n",
				sensor_info->port);
			return ret;
		}
	}

	if ((sensor_info->config_index & 0xff) == SUNNY_NC120 ||
		(sensor_info->config_index & 0xff) == SUNNY_NC30 ||
		(sensor_info->config_index & 0xff) == SUNNY ||
		(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		(sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		(sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
		vendor_id = check_eeprom_vendor_id(sensor_info);
		vin_dbg("read eeprom vendor id = %x\n", vendor_id);
		if (vendor_id == WITHOUT_CRYSTAL) {
			setting_size = sizeof(max96717_setting_rclk) / sizeof(uint32_t) / 2;
			vin_dbg("sensor without crystal, serializer provide clk\n");
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								setting_size, max96717_setting_rclk);
			if (ret < 0) {
				vin_err("write max96717_setting_rclk error\n");
			}
		}
	}

	/* serial sync config */
	if ((sensor_info->config_index & TRIG_STANDARD) ||
		(sensor_info->config_index & TRIG_SHUTTER_SYNC)) {
		if ((sensor_info->config_index & 0xff) == SENSING_NARROW ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC120 ||
		 (sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC120_WITHOUT_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC30_WITHOUT_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_X8B_WITH_X3C_DPLL ||
		 (sensor_info->config_index & 0xff) == SENSING_X8B_WITH_X3C ||
		 (sensor_info->config_index & 0xff) == SUNNY_NC30 ||
		 (sensor_info->config_index & 0xff) == SUNNY) {
			setting_size = sizeof(max9295_trigger_mfp7_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig7\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_trigger_mfp7_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
			}
		} else if ((sensor_info->config_index & 0xff) == SN_X8B_WITH_X3C_DUAL) {
			setting_size = sizeof(max9295_max96717_trigger_mfp8_setting_sync) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max96717 trig8\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_max96717_trigger_mfp8_setting_sync);
			if (ret < 0) {
				vin_err("write max96717_trigger_mfp8_setting_sync error\n");
			}
		} else {
			setting_size = sizeof(max9295_trigger_mfp8_setting) / sizeof(uint32_t) / 2;
			vin_dbg("write serial: %d@0x%2x max9295 trig8\n",
				sensor_info->bus_num, sensor_info->serial_addr);
			ret = vin_write_array(sensor_info->bus_num, sensor_info->serial_addr, 2,
								  setting_size, max9295_trigger_mfp8_setting);
			if (ret < 0) {
				vin_err("write max9295_trig_setting error\n");
			}
		}
	}

	/* max9295 need enable LDO */
	if (((sensor_info->config_index & 0xff) == SENSING) ||
	    ((sensor_info->config_index & 0xff) == SENSING_24M) ||
	    ((sensor_info->config_index & 0xff) == SENSING_DUAL) ||
	    ((sensor_info->config_index & 0xff) == SENSING_WITH_OVX3C) ||
	    ((sensor_info->config_index & 0xff) == SENSING_WS0233) ||
	    ((sensor_info->config_index & 0xff) == SENSING_X8B_RCLK) ||
	    ((sensor_info->config_index & 0xff) == SENSING_NARROW) ||
	    ((sensor_info->config_index & 0xff) == SENSING_X8B_WITH_X3C)) {
		setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(sensor_info->bus_num, sensor_info->serial_addr,
					       REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
		if (ret < 0) {
			vin_err("serial enalbe ldo fail!!!\n");
			return ret;
		}
	}

	ret = ovx8b_init(sensor_info);
	if(ret < 0) {
		vin_err("OVX8B_X3_config fail!\n");
		return ret;
	}
	vin_dbg("OVX8B_X3_config OK!\n");

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

	return ret;
}

int32_t special_serial_setting(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	int32_t bus, i;
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

	vin_info("ovx8b serial addr map\n");
	pdata = linka_seri2cmap;
	setting_size = sizeof(linka_seri2cmap)/sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num, linka_seri2cmap,
		setting_size);
	if (ret < 0) {
		vin_err("linka_seri2cmap failed for port%d\n",
			sensor_info->port);
		hb_vin_i2c_unlock(bus);
		return ret;
	}

	hb_vin_i2c_unlock(bus);

	setting_size = sizeof(serializer_linka_pipez_setting) / sizeof(uint8_t);
	ret = write_register_j5(deserial_if->bus_num, serializer_linka_pipez_setting,
		setting_size);
	if (ret < 0) {
		vin_err("serializer_linka_pipez_setting failed for port%d\n",
			sensor_info->port);
		return ret;
	}

    setting_size = sizeof(sensor1_max96717_mfp5_errb_mapping_max96718_mfp0_setting) /
		sizeof(uint8_t);
	ret = write_register_j5(bus, sensor1_max96717_mfp5_errb_mapping_max96718_mfp0_setting, setting_size);
	if (ret < 0) {
		vin_err("write bus: %d errb mfp mapping register error\n", bus);
		return ret;
	}

	return ret;
}

int32_t sensor_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, i;
	pthread_t t1;

	config_index[sensor_info->port] = sensor_info->config_index;
	extra_mode[sensor_info->port] = (sensor_info->extra_mode >>
			AWB_CTRL_MODE_OFFS) & AWB_CTRL_MODE_MASK;
	if ((extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_SN120)
		vin_dbg("The pre awb ratio is OX8GB-A120+065\n");
	else if ((extra_mode[sensor_info->port] & 0xf) == OVX8B_AWB_SN30)
		vin_dbg("The pre awb ratio is OX8GB-A030+017\n");
	else
		vin_dbg("The pre awb ratio is default\n");
	if (extra_mode[sensor_info->port] & AWB_CTRL_RATIO_DISABLE)
		vin_dbg("The pre awb is disabled");
	sensor_info_ex_t *sensor_info_ex = &sensor_info_exs[sensor_info->dev_port];
	dev_port2port[sensor_info->dev_port] = sensor_info->port;
	diag_mask[sensor_info->port] = sensor_info->stream_control;  /* used as diag_mask */
	sensor_info_ex->diag_mask.value = diag_mask[sensor_info->port];
	deserial_info_t *deserial_if = (deserial_info_t *)(sensor_info->deserial_info);
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
	ae_enable = (sensor_info->config_index & AE_DISABLE) ?
		(~HAL_AE_LINE_GAIN_CONTROL) : HAL_AE_LINE_GAIN_CONTROL;
	awb_enable = (sensor_info->config_index & AWB_DISABLE) ?
		(~HAL_AWB_CCT_CONTROL) : HAL_AWB_CCT_CONTROL;

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

	return ret;
}

int32_t sensor_ovx8b_serdes_stream_on(sensor_info_t *sensor_info)
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
	} else {
		vin_err("serdes %s not support error\n", deserial_if->deserial_name);
		goto unlock;
	}
unlock:
	return ret;
}

int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0, req;
	uint8_t *pdata = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t serial_addr = sensor_info->serial_addr;
	int32_t poc_addr = sensor_info->serial_addr1;
	int32_t sensor_addr = sensor_info->sensor_addr;
	int32_t bus, deserial_addr;
	int32_t entry_num = sensor_info->entry_num;

	if (deserial_if == NULL) {
		vin_err("no deserial here\n");
		return -1;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	pdata = ovx8b_stream_on_setting;
	setting_size = sizeof(ovx8b_stream_on_setting)/sizeof(uint8_t);
	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
			sensor_addr, pdata, setting_size);
	if (ret < 0)
		vin_err("write register error\n");

	if (deserial_if) {
		req = hb_vin_mipi_pre_request(entry_num, 1, 0);
		if (req == 0) {
			ret = sensor_ovx8b_serdes_stream_on(sensor_info);
			if (ret < 0) {
				ret = -HB_CAM_SERDES_STREAM_ON_FAIL;
				vin_err("%d : %s sensor_ovx8b_serdes_stream_on fail\n",
					   __LINE__, sensor_info->sensor_name);
			}
			hb_vin_mipi_pre_result(entry_num, 1, ret);
		}
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

	/* xj3 */
	pdata = ovx8b_stream_off_setting;
	setting_size = sizeof(ovx8b_stream_off_setting)/sizeof(uint8_t);
	ret = write_register(bus, deserial_addr, poc_addr, serial_addr,
				sensor_addr, pdata, setting_size);
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
		vin_err("input sp|si is null!\n");
		return -RET_ERROR;
	}
	int32_t i2c_num = si->bus_num;
	int32_t i2c_addr = si->sensor_addr;

	sp->frame_length = turning_data.sensor_data.VMAX;
	sp->line_length = turning_data.sensor_data.HMAX;

	sp->width = turning_data.sensor_data.active_width;
	sp->height = turning_data.sensor_data.active_height;

	if ((si->config_index & 0xff) == SENSING ||
		(si->config_index & 0xff) == SENSING_NARROW ||
		(si->config_index & 0xff) == SENSING_DUAL ||
		(si->config_index & 0xff) == SENSING_WS0233) {
		strncpy(sp->version, VERSION_SENSING, sizeof(sp->version));
	}

	sp->pclk = sensor_pll_data.sclk;
	sp->fps = sensor_pll_data.fps;
	sp->exp_num = HDR4;
	sp->lines_per_second = turning_data.sensor_data.lines_per_second;

	return ret;
}

int32_t group_hold_start(hal_control_info_t *info) {
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int crc_last_check[CAM_MAX_NUM];
	int port = dev_port2port[info->port];

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num, info->sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] clear sccb crc failed\n", __func__, port);
			return ret;
		}
	}
	/* group hold start */
	setting_size = sizeof(group_hold_start_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(info->bus_num, info->sensor_addr,
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

int32_t group_hold_end(hal_control_info_t *info)
{
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	static int crc_last_check[CAM_MAX_NUM];
	int port = dev_port2port[info->port];

	/* clear SCCB CRC */
	if (((diag_mask_u)diag_mask[port]).sensor_i2c_crc) {
		ret = hb_vin_i2c_read_reg16_data16(info->bus_num, info->sensor_addr, OV_SCCB_CRC);
		if (ret < 0) {
			vin_err("%s port [%d] clear sccb crc failed\n", __func__, port);
			return ret;
		}
	}
	/* group hold start */
	setting_size = sizeof(group_hold_end_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(info->bus_num, info->sensor_addr,
		SENSOR_REG_WIDTH, setting_size, group_hold_end_setting);
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
			info->sensor_addr, dst_reg, &data[begin_idx], blk_size);
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

int write_ae_reg(hal_control_info_t *info)
{
	int ret = 0, len, setting_size, crc16_ret;
	uint8_t crc_array[MAX_NUM_LENGTH];
	int port = dev_port2port[info->port];
	int32_t i;
	static int32_t reg_state[CAM_MAX_NUM] = {0};
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
	static int32_t reg_state[CAM_MAX_NUM] = {0};

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
			sizeof(sensor_status_info_t));
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

	if((rgain == rgain_tmp_buf[info->port]) &&
			(bgain == bgain_tmp_buf[info->port]) &&
			(grgain == grgain_tmp_buf[info->port]) &&
			(gbgain == gbgain_tmp_buf[info->port]))
		return 0;

	rgain_tmp_buf[info->port] = rgain;
	bgain_tmp_buf[info->port] = bgain;
	grgain_tmp_buf[info->port] = grgain;
	gbgain_tmp_buf[info->port] = gbgain;

	switch ((extra_mode[info->port] & 0xf)) {
	case OVX8B_AWB_SN120:
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
	case OVX8B_AWB_SN30:
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
	case OVX8B_AWB_LCE120:
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
	case OVX8B_AWB_LCE30:
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
	case OVX8B_AWB_DEFT:
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
	if (extra_mode[info->port] & AWB_CTRL_RATIO_DISABLE) {
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
		(((extra_mode[info->port] & 0xf) == OVX8B_AWB_SN120) ||
		((extra_mode[info->port] & 0xf) == OVX8B_AWB_SN30) ||
		((extra_mode[info->port] & 0xf) == OVX8B_AWB_LCE120) ||
		((extra_mode[info->port] & 0xf) == OVX8B_AWB_LCE30))) {
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

	return ret;
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
	*enable = (HAL_AWB_CCT_CONTROL & awb_enable) +
              (HAL_AE_LINE_GAIN_CONTROL & ae_enable);
	vin_dbg("enter userspace_control enalbe = %d\n", *enable);
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
	if ((si->extra_mode & GALAXY_PARAMS) ||
		(si->extra_mode & PARAMS_2)) {
		eeprom_addr_alias_id = EEPROM_I2C_ADDR_ALIAS_ID + si->deserial_port;
	} else {
		eeprom_addr_alias_id = DEFAULT_EEPROM_I2C_ADDR + si->deserial_port;
	}
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
		if (data == 2) {
			sip->cam_type = 0;
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
		sip->cam_type = (uint8_t)data;

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
	vin_dbg("fov:%0.12lf k1:%0.12lf k2:%0.12lf p1:%0.12lf p2:%0.12lf\n",
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

#if 0
int32_t sensor_get_status(sensor_info_t *sensor_info)
{
	int32_t val = 0;
	int32_t ret = 0;
	sensor_status_u status = {0};

	// stream state check
	{
		val = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num,
			sensor_info->sensor_addr, OV_STREAMING);

		if (val < 0) {
			vin_err("senor %s read stream state error\n",
				sensor_info->sensor_name);
			return val;
		}

		if (0 == val) {
			status.stream_off = 1;
			vin_err("sensor %s enters stream off mode\n",
				sensor_info->sensor_name)
		}
	}

	// fps check
	{
		uint64_t time_diff = 0;
		float fps = 0.0;
		float fps_setting = sensor_info->fps;
		int32_t val = 0;
		ret = get_sensor_frame_count(sensor_info, &frame_count_cur);
		time_diff =
			(frame_count_cur.tv.tv_sec - frame_count_last.tv.tv_sec) * 1000000
			+ (frame_count_cur.tv.tv_usec - frame_count_last.tv.tv_usec);

		if (time_diff > (1000000)) {
			fps = (frame_count_cur.frame_counter -
				frame_count_last.frame_counter) * 1000000.0 / time_diff;
			vin_dbg("senor %s read fps is %f\n", sensor_info->sensor_name, fps);
			if (fps < (fps_setting - 1) || fps > (fps_setting + 1)) {
				status.fps_check = 1;
				vin_err("sensor %s fps check error, the setting fps is %f, "
					"while the read fps is %f\n", sensor_info->sensor_name,
					fps_setting, fps);
				vin_err("frame_counter_last:%d, frame_counter_cur:%d, time_diff:%ld\n",
					frame_count_last.frame_counter, frame_count_cur.frame_counter,
					time_diff);
				val = get_fcnt_val(sensor_info);
				vin_err("secondly read fcnt val = 0x%x\n", val);
			}
			frame_count_last.frame_counter = frame_count_cur.frame_counter;
			frame_count_last.tv.tv_sec = frame_count_cur.tv.tv_sec;
			frame_count_last.tv.tv_usec = frame_count_cur.tv.tv_usec;
		}
	}

	return status.value;
}
#endif

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ovx8b, CAM_MODULE_FLAG_A16D16);
sensor_module_t ovx8b = {
	.module = SENSOR_MNAME(ovx8b),
#else
sensor_module_t ovx8b = {
	.module = "ovx8b",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.get_sns_params = get_sns_info,
	.awb_cct_control = sensor_awb_cct_control,
	.aexp_line_gain_control = sensor_aexp_line_gain_control,
	.start_control = sensor_start_control,
	.end_control = sensor_end_control,
	.userspace_control = sensor_userspace_control,
	// .get_sns_status = sensor_get_status,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.hotplug_init = hotplug_init,
};

