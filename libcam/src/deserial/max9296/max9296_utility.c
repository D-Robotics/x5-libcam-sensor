/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[max9296]:[%s][%d]" fmt, __func__, __LINE__

#include <string.h>
#include "../../inc/hb_vin.h"
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "./max9296_setting.h"
#include "./max_common.h"

#define INIT_STATE 1
#define DEINIT_STATE 2
#define DES_RESET_DELAY 2000
#define LINK_MAX	2
#define VC_MAX		2
#define MAX_BIT_NUM		8
#define LINK_ALL_FLAG	0x0f
#define LINK_MASK_ALL	0x3
#define INVALID_DESERIAL_ADDR 0xff
#define SETTING_SIZE	128

typedef struct pipe_info_s {
	uint32_t pipe_num;
	uint32_t pipe_config_index;
	uint32_t pipe_datatype[DES_LINK_NUM_MAX];
	uint32_t pipe_to_csi[DES_LINK_NUM_MAX];
	int32_t pipe_vc_arry[LINK_MAX][VC_MAX];
	uint32_t pdata[LINK_MAX][SETTING_SIZE];
	uint32_t setting_size[LINK_MAX];
}pipe_info_t;

#ifdef CAM_DIAG
typedef struct node_info_s {
	uint8_t subid;
	uint8_t subevent_id;
	uint8_t subtype;
	uint16_t mask;
	int32_t reg_addr;
	int32_t (*fault_judging)(struct diag_node_info_s *node);
	int32_t (*fault_clear)(struct diag_node_info_s * node);
	int32_t (*test_fault_inject)(struct diag_node_info_s *node, int32_t inject);
	void *cb_data;
} node_info_t;
#endif

// PRQA S 3206++

static inline int32_t max9296_addr_check(uint32_t deserial_addr)
{
	return ((deserial_addr == INVALID_DESERIAL_ADDR) ? -1 : 0);
}

int32_t max9296_reset(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
			RESET_REG, RESET_VAL);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x\n", bus, slave_addr,
				RESET_REG, RESET_VAL);
		return ret;
	}
	usleep(100*1000);
	return ret;
}

static int32_t max9296_get_deserial_link_info(deserial_info_t *deserial_if,
					       pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK;
	int32_t i;
	uint32_t tmp = 0;
	uint32_t csi_index = 0;

	for (i = 0; i < LINK_MAX; i++) {
		if (deserial_if->sensor_info[i] != NULL) {
			if (((sensor_info_t *)(deserial_if->sensor_info[i]))->sensor_ops != NULL) {
				tmp |= (1 << SENSOR_INFO(i)->deserial_port);
				ret = vin_sensor_emode_parse(SENSOR_INFO(i), 'D');
				if (ret < 0) {
					vin_err("sensor %s datatype parse fail\n", SENSOR_INFO(i)->sensor_name);
					return ret;
				}
				pipe_info->pipe_datatype[i] = ret;
				csi_index = deserial_if->deserial_csi[SENSOR_INFO(i)->deserial_port];
				if (csi_index > CSI_NUM) {
					vin_err("max9296 deserial_csi value only support 0 or 1\n");
					return -1;
				} else {
					pipe_info->pipe_to_csi[i] = csi_index;
				}
			}
			pipe_info->pipe_num++;
		} else if (deserial_if->port_desp[i] != NULL) {
			tmp |= (1 << i);
			ret = vin_string_parse(deserial_if->port_desp[i], 'D');
			if (ret < 0) {
				vin_err("sensor %s datatype parse fail\n", deserial_if->port_desp[i]);
				return ret;
			}
			pipe_info->pipe_datatype[i] = ret;
			csi_index = deserial_if->deserial_csi[i];
			if (csi_index > CSI_NUM) {
				vin_err("max9296 deserial_csi value only support 0 or 1\n");
				return -1;
			} else {
				pipe_info->pipe_to_csi[i] = csi_index;
			}
			pipe_info->pipe_num++;
		}
	}
	pipe_info->pipe_config_index = tmp;

	return RET_OK;
}

static void max9296_get_pipe_vc(deserial_info_t *deserial_if, pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK;
	int32_t i;
	int32_t vc = 0;
	int32_t link_vc_arry[LINK_MAX][VC_MAX];

	for (i = 0; i < LINK_MAX; i ++) {
		if (deserial_if->sensor_info[i] != NULL) {
			link_vc_arry[i][0] = vc++;
			if (SENSOR_INFO(i)->config_index & BIT(B_DUAL_ROI))
				link_vc_arry[i][1] = vc++;
			else
				link_vc_arry[i][1] = -1;
			continue;
		}
		if (deserial_if->port_desp[i] != NULL) {
			link_vc_arry[i][0] = vc++;
			ret = vin_string_parse(deserial_if->port_desp[i], PDESP_F_SEN_CONFIG_INDEX);
			if ((ret >= 0) && (ret & BIT(B_DUAL_ROI)))
				link_vc_arry[i][1] = vc++;
			else
				link_vc_arry[i][1] = -1;
			continue;
		}
		link_vc_arry[i][0] = -1;
		link_vc_arry[i][1] = -1;
	}
	memcpy(pipe_info->pipe_vc_arry, link_vc_arry, sizeof(link_vc_arry));
	return;
}

static void max9296_change_pipe_csi_map(uint32_t *pdata, uint32_t csi_index)
{
	if (csi_index == 1)
		pdata[3] = PIPE_MAP_TO_CSI1;
	else
		pdata[3] = PIPE_MAP_TO_CSI0;
}

static int32_t max9296_change_pipe_setting(uint32_t *pdata, pipe_info_t *pipe_info,
					   uint32_t setting_size, int32_t pipe_index,
					   uint32_t vc_idx, uint32_t vc_num)
{
	deserial_change_pipe_vc(pdata, vc_idx, vc_num);
	max9296_change_pipe_csi_map(pdata, pipe_info->pipe_to_csi[pipe_index]);
	if (SETTING_SIZE - pipe_info->setting_size[pipe_index] < setting_size / sizeof(uint32_t)) {
		vin_err("The setting size is more than %d\n", SETTING_SIZE);
		return -1;
	}
	memcpy(pipe_info->pdata[pipe_index] + pipe_info->setting_size[pipe_index],
	       pdata, setting_size);
	pipe_info->setting_size[pipe_index] += setting_size / sizeof(uint32_t);
	return 0;
}

static int32_t max9296_get_pipe_setting(pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK;
	uint32_t tmp_size, i;

	for (i = 0; i < LINK_MAX; i++) {
		if (((uint32_t)1 << i) & pipe_info->pipe_config_index) {
			if ((pipe_info->pipe_datatype[i]) == EMODE_RAW12) {
				pipe_info->setting_size[i] = 0;
				if(pipe_info->pipe_vc_arry[i][0] >= 0) {        // override vc0 to dst vc
					tmp_size = sizeof(max9296_raw_pipe_setting[i]);
					ret = max9296_change_pipe_setting(max9296_raw_pipe_setting[i], pipe_info, tmp_size,
									i, PIPE_VC0_SETTING_IDX, pipe_info->pipe_vc_arry[i][0]);
					if (ret < 0) {
						vin_err("max9296 change pipe setting fail\n");
						return ret;
					}
				}
				if(pipe_info->pipe_vc_arry[i][1] >= 0) {        // override vc1 to dst vc
					tmp_size = sizeof(max9296_raw_pipe_vc1_setting[i]);
					ret = max9296_change_pipe_setting(max9296_raw_pipe_vc1_setting[i], pipe_info, tmp_size,
									i, PIPE_VC1_SETTING_IDX, pipe_info->pipe_vc_arry[i][1]);
					if (ret < 0) {
						vin_err("max9296 change pipe setting fail\n");
						return ret;
					}
				}
			} else if ((pipe_info->pipe_datatype[i]) == EMODE_YUV422) {
				pipe_info->setting_size[i] = 0;
				if(pipe_info->pipe_vc_arry[i][0] >= 0) {        // override vc0 to dst vc
					tmp_size = sizeof(max9296_yuv_pipe_setting[i]);
					ret = max9296_change_pipe_setting(max9296_yuv_pipe_setting[i], pipe_info, tmp_size,
									i, PIPE_VC0_SETTING_IDX, pipe_info->pipe_vc_arry[i][0]);
					if (ret < 0) {
						vin_err("max9296 change pipe setting fail\n");
						return ret;
					}
				}
			} else {
				vin_err("Don't support datatype %d\n", pipe_info->pipe_datatype[i]);
				return -1;
			}
		}
		vin_dbg("setting_size[%d] = %d\n", i, pipe_info->setting_size[i]);
	}
	return 0;
}

static int32_t max9296_gmsl_speed_init(deserial_info_t *deserial_if, uint32_t link_num)
{
	int32_t ret = RET_OK, i;
	int8_t gmsl_tmp[LINK_MAX] = {-1, -1}, gmsl_speed, gmsl_val;
	uint8_t val = 0;
	int16_t gmsl_reg = GMSL_REG;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	ret = deserial_get_gmsl_speed(deserial_if, gmsl_tmp, LINK_MAX);
	if (ret < 0) {
		vin_err("max9296 get gmsl speed fail!!!\n");
		return ret;
	}

	if (link_num == 1) {
		gmsl_speed = gmsl_tmp[0] < 0 ? gmsl_tmp[1] : gmsl_tmp[0];
	} else if (link_num ==2) {
		if (gmsl_tmp[0] != gmsl_tmp[1]) {
			vin_err("max9296 don't support different gmsl speed of dual links\n");
			return -1;
		}
		gmsl_speed = gmsl_tmp[0];
	} else {
		vin_err("max9296 only 2 links support, %d is more than 2\n", link_num);
		return -1;
	}
	if (gmsl_speed == GMSL_MODE3) {
		gmsl_val = GMSL_3GBPS;
	} else if (gmsl_speed == GMSL_MODE6) {
		gmsl_val = GMSL_6GBPS;
	} else {
		vin_err("max9296 don't support gmsl speed %dgbps\n", gmsl_speed);
		return -1;
	}
	ret = hb_vin_i2c_read_reg16_data8(bus, slave_addr, gmsl_reg);
	if (ret < 0) {
		vin_err("read max9296 gmsl speed reg 0x%x fail!!!\n", gmsl_reg);
		return -1;
	}
	val = ret & SHIFT_8BIT;
	if ((val & GMSL_MASK) == gmsl_val) {  // The current speed is the same as the speed to be set
		vin_info("The rate has already been configured\n");
		return RET_OK;
	}
	val &= (~GMSL_MASK);
	val |= gmsl_val;
	vin_info("gmsl speed = %d, gmsl_reg = 0x%x\n", val, gmsl_reg);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, gmsl_reg, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
			slave_addr, gmsl_reg, val);
		return ret;
	}
	return ret;
}
static int32_t max9296_link_lock_check(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	int32_t timeout;

	timeout = 0;
	while (timeout <= LINK_LOCK_TIMEOUT) {
		ret = hb_vin_i2c_read_reg16_data8(bus, slave_addr, LINK_LOCK_REG);
		if (ret < 0) {
			vin_err("%s read link lock status fail!!!\n", deserial_if->deserial_name);
			return ret;
		}
		if (ret & LINK_LOCK_MASK) {
			break;
		}
		if (timeout + 20 > LINK_LOCK_TIMEOUT) {
			vin_err("%s link lock timeout %dms!!!, link lock reg val is 0x%x.\n",
				deserial_if->deserial_name, timeout, ret);
			return -1;
		}
		usleep(20*1000);
		timeout += 20;
	}
	vin_info("%s link is locked, lock time is %dms\n", deserial_if->deserial_name, timeout);
	return RET_OK;
}

int32_t max9296_link_enable(deserial_info_t *deserial_if, uint8_t link_mask)
{
	int32_t ret = RET_OK, link_cfg = -1;
	uint8_t val = 0;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	pipe_info_t pipe_info;

	if (max9296_addr_check(deserial_if->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	pipe_info.pipe_num = 0;
	ret = max9296_get_deserial_link_info(deserial_if, &pipe_info);
	if (ret < 0)
		return ret;
	if (pipe_info.pipe_num == 2) {
		link_mask &= 0x3;
		link_cfg = hb_vin_i2c_read_reg16_data8(bus, slave_addr, LINK_EN_REG);
		val |= link_mask | ONESHOTRESET_MASK;
		vin_dbg("link_cfg 0x%x, link_mask 0x%x, val 0x%x\n", link_cfg, link_mask, val);
		ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, LINK_EN_REG, val);
		if (ret < 0) {
			vin_err("write max9296_link_en_reg fail\n");
			return ret;
		}
		usleep(100*1000);
	}

	ret = max9296_link_lock_check(deserial_if);
	if (ret < 0) {
		if (pipe_info.pipe_num == 2) {
			vin_warn("link lock check failed, restore reg 0x%x value 0x%x\n",
					LINK_EN_REG, link_cfg);
			if (link_mask != LINK_MASK_ALL) {
				(void)hb_vin_i2c_write_reg16_data8(bus, slave_addr, LINK_EN_REG, link_cfg);
				return -1;
			}
			/* LINK_MASK_ALL will retry link check */
			link_cfg |= ONESHOTRESET_MASK;
			ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, LINK_EN_REG, link_cfg);
			if (ret < 0) {
				vin_err("write max9296_link_en_reg fail\n");
				return ret;
			}
			usleep(100*1000);
			ret = max9296_link_lock_check(deserial_if);
		}
	}
	return ret;
}

int32_t max9296_trig_mode_config(deserial_info_t *deserial_if, uint8_t deserial_link, uint8_t gpio_id)
{
	int32_t ret = RET_OK;
	uint8_t val;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	uint32_t trig_pin_num = 0;
	uint8_t gpio_index;
	uint16_t reg_addr;

	for (int32_t i = 0; i < DES_LINK_NUM_MAX; i++)
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;

	if (trig_pin_num == 1)
		gpio_index = TRIG_PIN(deserial_if, 0);
	else
		gpio_index = TRIG_PIN(deserial_if, deserial_link);

	if (gpio_index > GPIO_INDEX_MAX) {
		vin_err("max9296 trig_pin is %d, exceeded the maximum value %d!\n",
			gpio_index, GPIO_INDEX_MAX);
		return -1;
	}
	val = GPIO_TO_TRIG_VAL;
	reg_addr = REG_ADDR_GPIO(gpio_index);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			slave_addr,
			GPIO_BASE_REG + (gpio_index * 3),
			val);
		return ret;
	}
	vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr, val);
	val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr + 1);
	val &= (~GPIO_ID_MASK);
	val |= (gpio_id + GPIO_ID_OFFSET);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr + 1, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			slave_addr, reg_addr + 1, val);
		return ret;
	}
	vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr + 1, val);
	if (gpio_index == 1) {
		reg_addr = 0x05;
		val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
		val &= (~LOCK_EN);
		ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
		if (ret < 0) {
			vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
				bus, slave_addr, reg_addr, val);
			return ret;
		}
		vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
				slave_addr, deserial_link, gpio_index, reg_addr, val);
	}
	if (gpio_index == 5 || gpio_index == 6) {
		reg_addr = GMSL_MASK;
		val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
		val &= (~UART_1_EN);
		ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
		if (ret < 0) {
			vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
				bus, slave_addr, reg_addr, val);
			return ret;
		}
		vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
				slave_addr, deserial_link, gpio_index, reg_addr, val);
	}
	return ret;
}

int32_t max9296_diag_mode_config(deserial_info_t *deserial_if, uint8_t deserial_link)
{
	int32_t ret = RET_OK, i;
	uint8_t val;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	int8_t gpio_index = 0;
	uint16_t reg_addr;

	gpio_index = CAMERR_PIN(deserial_if, deserial_link);
	if (gpio_index < 0) {
		vin_err("max9296 link%d sensor err pin not set\n", deserial_link);
		return -1;
	}
	if (gpio_index > GPIO_INDEX_MAX) {
		vin_err("max9296 camerr_pin[%d] is %d, exceeded the maximum value %d!\n",
			deserial_link, gpio_index, GPIO_INDEX_MAX);
		return -1;
	}
	val = GPIO_BASE_VAL | GPIO_TO_DIAG_VAL;
	reg_addr = MAX9296_REG_ADDR_GPIO(gpio_index);
	vin_info("dev_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr, val);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			slave_addr,
			GPIO_BASE_REG + (gpio_index * 3),
			val);
		return ret;
	}
	reg_addr = MAX9296_ERRB_GPIO_RX_ADDR(gpio_index);
	val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
	val &= (~GPIO_ID_MASK);
	val |= deserial_link;
	vin_info("dev_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr, val);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			slave_addr, reg_addr, val);
		return ret;
	}
	// modify max9296 mfp5 uart mode to gpio mode
	if (gpio_index == 5) {
		reg_addr = GMSL_MASK;
		val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
		val &= (~UART_1_EN);
		ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
		if (ret < 0) {
			vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
				bus, slave_addr, reg_addr, val);
			return ret;
		}
		vin_info("Deserial RX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
				slave_addr, deserial_link, gpio_index, reg_addr, val);
	}

	return ret;
}

int32_t max9296_mfp_cfg(deserial_info_t *deserial_if,
			uint8_t gpio_mode, uint8_t gpio_id, uint8_t deserial_link)
{
	int32_t ret = RET_OK;

	if (max9296_addr_check(deserial_if->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	if (gpio_mode & GPIO_TX_GMSL) {
		ret = max9296_trig_mode_config(deserial_if, deserial_link, gpio_id);
	} else if (gpio_mode & GPIO_RX_GMSL) {
		ret = max9296_diag_mode_config(deserial_if, deserial_link);
	} else {
		vin_err("max9296 don't support mfp mode 0x%x\n", gpio_mode);
		return -1;
	}
	return ret;
}

int32_t max9296_phy_speed_cfg(deserial_info_t *deserial_if, uint32_t link_num)
{
	int32_t ret = RET_OK, i;
	int8_t val;
	uint16_t reg;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	uint32_t phy_speed;
	uint32_t phy_copy;
	uint32_t setting_size;

	if (link_num == 1) {
		phy_speed = ((deserial_if->lane_speed) ? deserial_if->lane_speed : 1200) / 100;
	} else if (link_num == 2) {
		phy_speed = ((deserial_if->lane_speed) ? deserial_if->lane_speed : 2000) / 100;
	} else {
		vin_err("max9296 only 2 links support, %d is more than 2\n", link_num);
		return -1;
	}
	phy_copy = (deserial_if->lane_mode & (1 << 6)) ? 1 : 0;

	for (i = 0; i < CSI_NUM; i++) {
		reg = MIPICSIA_SPEED_REG + (i * 3);
		val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg);
		if (val < 0) {
			vin_err("read %d@0x%x reg 0x%x fail!\n", bus, slave_addr, reg);
			return -1;
		}
		val &= (~MAXIM_PHYSPEED_MASK);
		val |= phy_speed;
		ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg, val);
		if (ret < 0) {
			vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
					slave_addr, reg, val);
			return -1;
		}
	}
	if (phy_copy == 1) {
		setting_size = sizeof(max9296_phy_copy_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(bus, slave_addr, REG16_VAL8,
				      setting_size, max9296_phy_copy_init_setting);
		if (ret < 0)
			vin_info("max9296 phy copy setting fail\n");
	}
	return ret;
}

int32_t data_intf_init(deserial_info_t *deserial_if, pipe_info_t *pipe_info, uint8_t *intf_init)
{
	int32_t ret = RET_OK, i, intf_type, desport;
	uint32_t reg_addr, setting_size = 0;
	uint32_t *pdata = NULL;

	if(*intf_init == 1)
		return RET_OK;
	vin_dbg("pipe num = %d, pipe_config_index = %d, intf_init = %d",
			pipe_info->pipe_num, pipe_info->pipe_config_index, *intf_init);
	for (i = 0; i < LINK_MAX; i++) {
		if (pipe_info->pipe_config_index & BIT(i)) {
			desport = i;
		} else {
			continue;
		}
		if (SENSOR_INFO(desport) != NULL) {
			intf_type = vin_sensor_emode_parse(SENSOR_INFO(desport), 'I');
			if (intf_type < 0) {
				vin_dbg("sensor data type intf_type = %d is not dvp!!!\n", intf_type);
				continue;
			}
			if (intf_type == DVP) {
				if (pipe_info->pipe_num == 1) {
					pdata = max9296_datatype_bpp_mux_init_setting[0];
				} else {
					pdata = max9296_datatype_bpp_mux_init_setting[desport];
				}
				setting_size = sizeof(max9296_datatype_bpp_mux_init_setting[0]) / sizeof(uint32_t) / 3;
				ret = vin_i2c_bit_array_write8(deserial_if->bus_num, deserial_if->deserial_addr,
									REG_WIDTH_16bit, setting_size, pdata);
				if (ret < 0) {
					vin_err("i2c bit array write data fail!!!\n");
					return ret;
				}
	    	}
		}
		*intf_init = 1;
	}

	return ret;
}

static int32_t max9296_pipe_config(deserial_info_t *deserial_if, pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK;
	uint8_t link_num = 0;
	uint8_t val, intf_init = 0;
	int32_t pipe_config_index, setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus = deserial_if->bus_num, i;
	uint8_t slave_addr = (uint8_t)deserial_if->deserial_addr;

	max9296_get_pipe_vc(deserial_if, pipe_info);
	ret = max9296_get_pipe_setting(pipe_info);
	if (ret < 0) {
		vin_err("%s get pipe setting fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	for (i = 0; i < LINK_MAX; i++) {
		if (((uint32_t)1 << i) & pipe_info->pipe_config_index) {
			setting_size = pipe_info->setting_size[i]/2;
			pdata = pipe_info->pdata[i];
			ret = data_intf_init(deserial_if, pipe_info, &intf_init);
			if (ret < 0) {
				vin_err("parse data intf fail!\n");
			}
			ret = vin_write_array(bus, slave_addr, REG16_VAL8, setting_size, pdata);
			if (ret < 0) {
				vin_err("deserial init fail!\n");
				return ret;
			}
		}
	}
	vin_info("max9296 pipe config done\n");

	return ret;
}

static int32_t deserializer_stream_on(deserial_info_t *max9296_info, uint32_t port)
{
	int32_t ret = RET_OK;
	int32_t setting_size;
	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	setting_size = sizeof(max9296_stream_on_setting) / sizeof(uint32_t) / 3;
	ret = vin_i2c_bit_array_write8(max9296_info->bus_num, max9296_info->deserial_addr,
			      REG_WIDTH_16bit, setting_size, max9296_stream_on_setting);
	if (ret < 0)
		vin_err("max9296 stream on fail!!!\n");
	return ret;
}

static int32_t deserializer_stream_off(deserial_info_t *max9296_info, uint32_t port)
{
	int32_t ret = RET_OK;
	int32_t setting_size;
	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	setting_size = sizeof(max9296_stream_off_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(max9296_info->bus_num, max9296_info->deserial_addr,
			      REG16_VAL8, setting_size, max9296_stream_off_setting);
	if (ret < 0)
		vin_err("max9296 stream off fail!!!\n");
	return ret;
}

int32_t max9296_init(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	uint32_t setting_size;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = (uint8_t)deserial_if->deserial_addr;
	pipe_info_t pipe_info = {0};

	ret = max9296_get_deserial_link_info(deserial_if, &pipe_info);
	if (ret < 0) {
		vin_err("max9296 get link mask fail!!!\n");
		return ret;
	}
	ret = max9296_reset(deserial_if);
	if (ret < 0) {
		vin_err("max9296 reset fail!\n");
		return ret;
	}
	ret = deserializer_stream_off(deserial_if, 0);
	if (ret < 0)
		return ret;
	ret = max9296_gmsl_speed_init(deserial_if, pipe_info.pipe_num);
	if (ret < 0) {
		vin_info("%s gmsl speed init fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max9296_pipe_config(deserial_if, &pipe_info);
	if (ret < 0) {
		vin_err("max9296 pipe config fail!!!\n");
		return ret;
	}
	ret = max9296_phy_speed_cfg(deserial_if, pipe_info.pipe_num);
	if (ret < 0) {
		vin_err("max9296 phy speed config fail!\n");
		return ret;
	}

	return ret;
}

static int32_t deserializer_deinit(deserial_info_t *max9296_info)
{
	int32_t ret = RET_OK;

	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	// to do
	return ret;
}
static int32_t deserializer_init(deserial_info_t *max9296_info)
{
	int32_t req, ret = RET_OK;
	uint32_t setting_size;
	uint32_t bus;
	int32_t i;
	uint8_t slave_addr;;
	sensor_info_t *sensor_info = NULL;
	sensor_module_t *sensor_ops = NULL;
	uint32_t entry_num = 0;

	if (max9296_info == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	for (i = 0; i < DES_LINK_NUM_MAX; i ++) {
		if (max9296_info->sensor_info[i] != NULL) {
			sensor_info = ((sensor_info_t *)(max9296_info->sensor_info[i]));
			sensor_ops = ((sensor_module_t *)(sensor_info->sensor_ops));
			if (sensor_ops == NULL) {
				continue;
			}
#ifdef CAMERA_FRAMEWORK_HBN
			if (SENSOR_EMODE(sensor_info) == NULL)
#else
			if (sensor_ops->emode == NULL)
#endif
				return ret;
			entry_num = sensor_info->entry_num;
		}
	}
	vin_info("max9296 init begin!\n");
	req = hb_vin_mipi_pre_request((uint32_t)entry_num, 0, 0);
	if (req == 0) {
		ret = max9296_init(max9296_info);
		hb_vin_mipi_pre_result((uint32_t)entry_num, 0, (uint32_t)ret);		/* PRQA S 2897 */
		if(ret < 0) {
			vin_err("max9296_init fail!\n");
			return ret;
		}
	}
	return ret;
}

static int32_t deserializer_start_physical(const deserial_info_t *max9296_info)
{
	int32_t ret = RET_OK;

	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	// to do
	return ret;
}

static int32_t deserializer_reset(const deserial_info_t *max9296_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	if(max9296_info->power_mode == 1u) {
		for(gpio = 0; gpio < max9296_info->gpio_num; gpio++) {
			if(max9296_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)max9296_info->gpio_pin[gpio],
							max9296_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail gpio %d\n",
						gpio);
					return -RET_ERROR;
				}
			}
		}
		(void)usleep(DES_RESET_DELAY);
		for(gpio = 0; gpio < max9296_info->gpio_num; gpio++) {
			if(max9296_info->gpio_pin[gpio] >= 0) {
				ret =(int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)max9296_info->gpio_pin[gpio],
							1-max9296_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("vin_power_ctrl fail gpio %d\n",
						gpio);
					return -RET_ERROR;
				}
			}
		}
	}
	return ret;
}

#ifdef CAM_DIAG
static int32_t get_enrty_num_of_sen(deserial_info_t *des)
{
	uint16_t link;
	sensor_info_t *sen;
	for (link = 0; link < LINK_MAX; link++) {
		sen = des->sensor_info[link];
		if (sen)
			break;
	}
	return (sen ? (sen->entry_num) : -1);
}

static void sub_node_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node, node_info_t sub_node)
{
	int32_t entry_num;
	uint32_t i, link, bit_field, link_mask, m_port_mask, m_diag_id;
	diag_node_info_t *sub_reg_node = NULL;
	entry_num = get_enrty_num_of_sen(max9296_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	for (i = 0, bit_field = 0; i < MAX_BIT_NUM; i++) {
		if ((sub_node.mask & ((uint8_t)1u << i)) != 0) {
			link = (sub_node.subevent_id) ? (sub_node.subevent_id + bit_field) : (i+1);
			m_diag_id = CAM_DES_DIAG_ID(entry_num, link, sub_node.subtype, sub_node.subid);
			link_mask = (sub_node.subevent_id == LINK_ALL_FLAG) ? LINK_MASK_ALL : BIT(link-1);
			m_port_mask = vin_port_mask_of_des(max9296_info, link_mask);
			if (m_port_mask == 0) {
				bit_field++;
				vin_info("reg_addr0x%x bit%d, port_mask %d\n", sub_node.reg_addr, i, m_port_mask);
				continue;
			}
			sub_reg_node = cam_diag_get_nodes(1);
			if (sub_reg_node == NULL) {
				bit_field++;
				vin_err("reg_addr0x%x bit%d, get node fail\n", sub_node.reg_addr, i);
				continue;
			}
			sub_reg_node->diag_id = m_diag_id;
			sub_reg_node->port_mask = m_port_mask;
			sub_reg_node->diag_status = 0;
			sub_reg_node->diag_type = CAM_DIAG_REG;
			sub_reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
			sub_reg_node->diag_info.t.reg.reg_bus = max9296_info->bus_num;
			sub_reg_node->diag_info.t.reg.reg_dev = max9296_info->deserial_addr;
			sub_reg_node->diag_info.t.reg.reg_addr = sub_node.reg_addr;
			sub_reg_node->diag_info.t.reg.reg_mask = (uint16_t)1u << i;
			sub_reg_node->fault_clear = sub_node.fault_clear;
			sub_reg_node->cb_data = sub_node.cb_data;
			sub_reg_node->test_fault_inject = sub_node.test_fault_inject;
			if (sub_node.subid == MAX9296_UNLOCK) {
				sub_reg_node->diag_info.t.reg.reg_active = 0;
				sub_reg_node->diag_flag = DIAG_LINK_LOCK | DIAG_MON_START | DIAG_REPORT_EN;
			} else {
				sub_reg_node->diag_info.t.reg.reg_active = (uint16_t)1u << i;  // 1: fault
				sub_reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
			}
			cam_diag_add_subnode(parent_node, sub_reg_node);
			bit_field++;
		}
	}
}

static void node_init(deserial_info_t *max9296_info, node_info_t node)
{
	int32_t entry_num;
	uint32_t i, link, bit_field, link_mask, m_port_mask, m_diag_id;
	diag_node_info_t *reg_node;
	entry_num = get_enrty_num_of_sen(max9296_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	for (i = 0, bit_field = 0; i < MAX_BIT_NUM; i++) {
		if ((node.mask & ((uint8_t)1u << i)) != 0) {
			link = (node.subevent_id) ? (node.subevent_id + bit_field) : (i+1);
			m_diag_id = CAM_DES_DIAG_ID(entry_num, link, node.subtype, node.subid);
			link_mask = (node.subevent_id == LINK_ALL_FLAG) ? LINK_MASK_ALL : BIT(link-1);
			m_port_mask = vin_port_mask_of_des(max9296_info, link_mask);
			if (m_port_mask == 0) {
				bit_field++;
				vin_info("reg_addr0x%x bit%d, port_mask %d\n", node.reg_addr, i, m_port_mask);
				continue;
			}
			reg_node = cam_diag_get_nodes(1);
			if (reg_node == NULL) {
				bit_field++;
				vin_err("reg_addr0x%x bit%d, get node fail\n", node.reg_addr, i);
				continue;
			}
			reg_node->diag_id = m_diag_id;
			reg_node->port_mask = m_port_mask;
			reg_node->diag_status = 0;
			reg_node->diag_type = CAM_DIAG_REG;
			reg_node->diag_info.t.reg.reg_type = DIAG_REG_TYPE(BIT_TYPE, REG16_VAL8);
			reg_node->diag_info.t.reg.reg_bus = max9296_info->bus_num;
			reg_node->diag_info.t.reg.reg_dev = max9296_info->deserial_addr;
			reg_node->diag_info.t.reg.reg_addr = node.reg_addr;
			reg_node->diag_info.t.reg.reg_mask = (uint16_t)1u << i;
			reg_node->fault_clear = node.fault_clear;
			reg_node->cb_data = node.cb_data;
			reg_node->test_fault_inject = node.test_fault_inject;
			if (node.subid == MAX9296_UNLOCK) {
				reg_node->diag_info.t.reg.reg_active = 0;
				reg_node->diag_flag = DIAG_LINK_LOCK | DIAG_MON_START | DIAG_REPORT_EN;
			} else {
				reg_node->diag_info.t.reg.reg_active = (uint16_t)1u << i;  // 1: fault
				reg_node->diag_flag = DIAG_MON_START | DIAG_REPORT_EN;
			}
			cam_diag_node_register(reg_node);
			bit_field++;
		}
	}
}

static void inject_pf(struct diag_node_info_s * node, int32_t inject)
{
	vin_info("diag_id 0x%x, %s\n", node->diag_id, inject ? "inject":"disable");
}
static void ser_errg_ctrl(sensor_info_t *sen, int32_t inject)
{
	uint16_t value1;
	value1 = inject ? 0x10 : 0x08;
	vin_i2c_write_retry(sen->bus_num,
						sen->serial_addr,
						REG16_VAL8,
						SER_ERRG_EN,
						value1);
}

static int32_t decode_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i;
	int32_t bus, dev, reg;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;

	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				reg = DEC_ERR_CNT_A_9296REG;
				break;
			case 1:
				reg = DEC_ERR_CNT_B_9296REG;
				break;
			default:
				reg = 0;
				break;
			}
			if (reg > 0)
				vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
		}
	}
	return 0;
}

int32_t test_decode_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	deserial_info_t *des_info = (deserial_info_t*)node->cb_data;
	uint8_t i;
	sensor_info_t *sen;
	uint16_t value1, value2;

	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				sen = (sensor_info_t*)des_info->sensor_info[0];
				break;
			case 1:
				sen = (sensor_info_t*)des_info->sensor_info[1];
				break;
			default:
				sen = NULL;
				break;
			}
			if(sen) {
				inject_pf(node, inject);
				ser_errg_ctrl(sen, inject);
			}
		}
	}
	return 0;
}

static void decode_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 3;  // bit0~1
	sub_node_info.reg_addr = 0x1b;  // LFLT_INT(bit3), IDLE_ERR(bit2), DEC_ERR(bit0~bit1)
	sub_node_info.subid = MAX9296_DEC_ERR;
	sub_node_info.subtype = DES_DEC_ERR_FLAG;
	sub_node_info.fault_clear = decode_fault_clear;
	sub_node_info.cb_data = (void*)max9296_info;
	sub_node_info.test_fault_inject = test_decode_fault_inject;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

static void line_fault_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3
	sub_node_info.reg_addr = 0x1b;  // LFLT_INT(bit3)
	sub_node_info.subid = MAX9296_LEFT_INT;
	sub_node_info.subtype = DES_LFLT_INT;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

static int32_t max_rtra_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i;
	int32_t bus, dev, reg;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 3: {
					reg = MAX_RT_ERR_CNT_9296REG;
					vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

int32_t test_max_rtra_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i;
	int32_t bus, dev;
	uint16_t value1, value2;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;

	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 3: {
					inject_pf(node, inject);
					if (inject) {
						value1 = 0x10; value2 = 0x08;
					} else {
						value1 = 0x08; value2 = 0x20;
					}
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										MAX9296_MAX_RT_LIMIT,
										0x12);
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										MAX9296_ERRG_EN,
										value1);
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										MAX9296_ERRG_CGF,
										value2);
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

static void arq_max_rtra_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3
	sub_node_info.reg_addr = 0x1d;  // MAX_RT(bit3)
	sub_node_info.subid = MAX9296_MAX_RT;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	sub_node_info.subtype = DES_MAX_RT_FLAG_0;
	sub_node_info.fault_clear = max_rtra_fault_clear;
	sub_node_info.test_fault_inject = test_max_rtra_fault_inject;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

static int32_t vid_pxl_crc_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i, j, num;
	int32_t bus, dev, reg;
	int32_t clear_reg[] = {	VID_PXL_CRC_CNT_X_9296REG,
							VID_PXL_CRC_CNT_Y_9296REG,
							VID_PXL_CRC_CNT_Z_9296REG,
							VID_PXL_CRC_CNT_U_9296REG};
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0: {
					num = sizeof(clear_reg) / sizeof(clear_reg[0]);
					for (j = 0; j < num; j++) {
						vin_i2c_read_retry(bus, dev, REG16_VAL8, clear_reg[j]);
					}
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

int32_t test_vid_pxl_crc_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	deserial_info_t *des_info = (deserial_info_t*)node->cb_data;
	uint8_t i;
	sensor_info_t *sen;
	uint16_t value1, value2;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				sen = (sensor_info_t*)des_info->sensor_info[0];
				break;
			default:
				sen = NULL;
				break;
			}
			if(sen) {
				inject_pf(node, inject);
				ser_errg_ctrl(sen, inject);
			}
		}
	}
	return 0;
}

static void vid_pxl_crc_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 1;  // bit0
	sub_node_info.reg_addr = 0x1f;  // VID_PXL
	sub_node_info.subid = MAX9296_VID_PXL_CRC;
	sub_node_info.subtype = DES_VID_PXL_CRC_ERR_0;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	sub_node_info.fault_clear = vid_pxl_crc_fault_clear;
	sub_node_info.cb_data = (void*)max9296_info;
	sub_node_info.test_fault_inject = test_vid_pxl_crc_fault_inject;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

static int32_t lcrc_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i, j, num;
	int32_t bus, dev, reg;
	int32_t clear_reg[] = {	LCRC_ERR_CNT_0_9296REG,
							LCRC_ERR_CNT_1_9296REG,
							LCRC_ERR_CNT_2_9296REG,
							LCRC_ERR_CNT_3_9296REG};
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 3: {
					num = sizeof(clear_reg) / sizeof(clear_reg[0]);
					for (j = 0; j < num; j++) {
						vin_i2c_read_retry(bus, dev, REG16_VAL8, clear_reg[j]);
					}
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

int32_t test_lcrc_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i;
	sensor_info_t *sen;
	deserial_info_t *des_info = (deserial_info_t*)node->cb_data;
	uint16_t value1;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 3:
				sen = (sensor_info_t*)des_info->sensor_info[0];
				break;
			default:
				sen = NULL;
				break;
			}
			if(sen) {
				inject_pf(node, inject);
				ser_errg_ctrl(sen, inject);
			}
		}
	}
	return 0;
}

static void video_line_crc_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3
	sub_node_info.reg_addr = 0x1f;  // LCRC
	sub_node_info.subid = MAX9296_LCRC_ERR;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	sub_node_info.subtype = DES_LCRC_ERR;
	sub_node_info.fault_clear = lcrc_fault_clear;
	sub_node_info.cb_data = (void*)max9296_info;
	sub_node_info.test_fault_inject = test_lcrc_fault_inject;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

static void link_lock_diag_init(deserial_info_t *max9296_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3
	sub_node_info.reg_addr = 0x13;  // link lock
	sub_node_info.subid = MAX9296_UNLOCK;
	sub_node_info.subtype = DES_UNLOCK;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	if (parent_node == NULL) {
		node_init(max9296_info, sub_node_info);
	} else {
		sub_node_init(max9296_info, parent_node, sub_node_info);
	}
}

int32_t test_line_memory_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i, num;
	int32_t bus, dev;
	uint16_t value1;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	int32_t dpll_reg[] = {	MAX9296_PYH0_DPLL,
							MAX9296_PYH1_DPLL,
							MAX9296_PYH2_DPLL,
							MAX9296_PYH3_DPLL};

	num = sizeof(dpll_reg) / sizeof(dpll_reg[0]);
	inject_pf(node, inject);
	value1 = inject ? 0x21 : 0x34;
	for (i = 0; i < num; i++) {
		vin_i2c_write_retry(bus,
							dev,
							REG16_VAL8,
							dpll_reg[i],
							value1);
	}
	return 0;
}

static void line_memory_diag_init(deserial_info_t *max9296_info)
{
	node_info_t node_info;
	(void)memset((void*)&node_info, 0, sizeof(node_info_t));
	node_info.mask = 6;
	node_info.reg_addr = 0x312;
	node_info.subid = MAX9296_LMO_ERR;
	node_info.subevent_id = 1;
	node_info.subtype = DES_LMO_ERR;
	node_info.test_fault_inject = test_line_memory_fault_inject;
	node_init(max9296_info, node_info);
}

int32_t max9296_diag_enable(deserial_info_t *max9296_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus, deserial_addr;

	if (max9296_info == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	bus = max9296_info->bus_num;
	deserial_addr = max9296_info->deserial_addr;
	pdata = max9296_diag_cfg;
	setting_size = sizeof(max9296_diag_cfg)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, deserial_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0)
		vin_err("deserial init fail!\n");

	return ret;
}

static int32_t deserializer_diag_nodes_init(deserial_info_t *max9296_info)
{
	int32_t entry_num;
	int32_t ret = RET_OK;
	int32_t gpio_index = -1;
	diag_node_info_t *parent_node = NULL;
	diag_node_info_t *parent_node1 = NULL;

	vin_info("diag init\n");
	if (max9296_addr_check(max9296_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	ret = max9296_diag_enable(max9296_info);
	if(ret < 0) {
		vin_err("max9296 diag init fail!\n");
		return ret;
	}
	/****************************errb*************************/
	gpio_index = ERRB_PIN(max9296_info);
	entry_num = get_enrty_num_of_sen(max9296_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	if (gpio_index > 0) {
		parent_node = cam_diag_get_nodes(1);
		if (parent_node) {
			parent_node->diag_id = \
			CAM_DES_DIAG_ID((uint8_t)entry_num, LINK_ALL_FLAG, DES_ERRB, MAX9296_ERRB);
			parent_node->port_mask = vin_port_mask_of_des(max9296_info, LINK_MASK_ALL);
			parent_node->diag_type = CAM_GPIO_TYPE(gpio_index);
			parent_node->diag_flag = DIAG_MON_START;
			parent_node->diag_info.t.gpio.gpio_type = CAM_GPIO_TYPE(gpio_index);
			parent_node->diag_info.t.gpio.gpio_index = (uint32_t)gpio_index;
			parent_node->diag_info.t.gpio.gpio_active = 0;
			parent_node->diag_info.t.gpio.gpio_count = 3;
			cam_diag_node_register(parent_node);
			vin_dbg("errb pin %d, gpio_type %d, gpio_active %d\n",
				parent_node->diag_info.t.gpio.gpio_index,
				parent_node->diag_info.t.gpio.gpio_type,
				parent_node->diag_info.t.gpio.gpio_active);
		}
	}
	decode_diag_init(max9296_info, parent_node);
	line_fault_diag_init(max9296_info, parent_node);
	arq_max_rtra_diag_init(max9296_info, parent_node);
	vid_pxl_crc_diag_init(max9296_info, parent_node);
	video_line_crc_diag_init(max9296_info, parent_node);

	/****************************link lock*************************/
	gpio_index = LOCK_PIN(max9296_info);
	if (gpio_index > 0) {
		parent_node1 = cam_diag_get_nodes(1);
		if (parent_node1) {
			parent_node1->diag_id = \
			CAM_DES_DIAG_ID((uint8_t)entry_num, LINK_ALL_FLAG, DES_UNLOCK, MAX9296_LOCK);
			parent_node1->port_mask = vin_port_mask_of_des(max9296_info, LINK_MASK_ALL);
			parent_node1->diag_type = CAM_GPIO_TYPE(gpio_index);
			parent_node1->diag_flag = DIAG_MON_START;
			parent_node1->diag_info.t.gpio.gpio_type = CAM_GPIO_TYPE(gpio_index);
			parent_node1->diag_info.t.gpio.gpio_index = (uint32_t)gpio_index;
			parent_node1->diag_info.t.gpio.gpio_active = 0;
			parent_node1->diag_info.t.gpio.gpio_count = 3;
			cam_diag_node_register(parent_node1);
			vin_dbg("lock pin %d, gpio_type %d, gpio_active %d\n",
				parent_node1->diag_info.t.gpio.gpio_index,
				parent_node1->diag_info.t.gpio.gpio_type,
				parent_node1->diag_info.t.gpio.gpio_active);
		}
	}
	link_lock_diag_init(max9296_info, parent_node1);

	/****************************reg poll*************************/
	line_memory_diag_init(max9296_info);
	vin_info("diad init end\n");
	return ret;
}
#endif  //  #ifdef CAM_DIAG

#ifdef CAMERA_FRAMEWORK_HBN
DESERIAL_MAX_MODULE(max9296, max9296_link_enable, NULL, max9296_mfp_cfg);
deserial_module_t max9296 = {
	.module = DESERIAL_MNAME(max9296),
#else
deserial_module_t max9296 = {
	.module = "max9296",
	.ops.max = {
		.link_enable = max9296_link_enable,
		.mfp_cfg = max9296_mfp_cfg,
	},
#endif
	.init = deserializer_init,
	.stream_on = deserializer_stream_on,
	.stream_off = deserializer_stream_off,
	.start_physical = deserializer_start_physical,
	.deinit = deserializer_deinit,
	.reset = deserializer_reset,
#ifdef CAM_DIAG
	.diag_nodes_init = deserializer_diag_nodes_init,
#endif
};
// PRQA S --
