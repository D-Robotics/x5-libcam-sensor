/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[max96712]:[%s][%d]" fmt, __func__, __LINE__

#include <string.h>
#include "../../inc/hb_vin.h"
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "./max96712_setting.h"
#include "./max_common.h"

#define INIT_STATE 1
#define DEINIT_STATE 2
#define DES_RESET_DELAY 2000
#define LINK_MAX	4
#define LINK_MASK_A	0x1
#define LINK_MASK_B	0x2
#define LINK_MASK_C	0x4
#define LINK_MASK_D	0x8
#define LINK_MASK_ALL	0xF
#define SENSOR_INFO(i) ((sensor_info_t *)(deserial_if->sensor_info[(i)]))
#define MAX_BIT_NUM	8
#define LINK_ALL_FLAG 0x0f
#define INVALID_DESERIAL_ADDR 0xff

typedef struct pipe_info_s {
	uint32_t pipe_num;
	uint32_t pipe_config_index;
	uint32_t pipe_datatype[DES_LINK_NUM_MAX];
	uint32_t pipe_to_csi[DES_LINK_NUM_MAX];
}pipe_info_t;

uint16_t gmsl_reg_mask[LINK_MAX] = {0x0300, 0x3000, 0x0003, 0x0030};
uint32_t gmsl_reg_shift[LINK_MAX] = {8, 12, 0, 4};

#ifdef	CAM_DIAG
typedef struct node_info_s {
	uint8_t subid;
	uint8_t subevent_id;
	uint8_t subtype;
	uint16_t mask;
	int32_t reg_addr;
	int32_t (*fault_judging)(struct diag_node_info_s *node);
	int32_t (*fault_clear)(struct diag_node_info_s * node);
	int32_t (*cb_inject)(struct diag_node_info_s * node);
	int32_t (*test_fault_inject)(struct diag_node_info_s *node, int32_t inject);
	void *cb_data;
} node_info_t;
#endif

static inline int32_t max96712_addr_check(uint32_t deserial_addr)
{
	return ((deserial_addr == INVALID_DESERIAL_ADDR) ? -1 : 0);
}

int32_t max96712_reset(deserial_info_t *deserial_if)
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
	return 0;
}

uint32_t max96712_get_deserial_link_info(deserial_info_t *deserial_if,
					 pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK;
	int32_t i;
	uint32_t tmp = 0;
	uint32_t csi_index = 0;

	for (i = 0; i < LINK_MAX; i++) {
		if (deserial_if->sensor_info[i] != NULL) {
			tmp |= (1 << SENSOR_INFO(i)->deserial_port);
			ret = vin_sensor_emode_parse(SENSOR_INFO(i), 'D');
			if (ret < 0) {
				vin_err("sensor %s datatype parse fail\n", SENSOR_INFO(i)->sensor_name);
				return ret;
			}
			pipe_info->pipe_datatype[pipe_info->pipe_num] = ret;
			csi_index = deserial_if->deserial_csi[SENSOR_INFO(i)->deserial_port];
			if (csi_index > CSI_NUM) {
				vin_err("max96712 deserial_csi value only support 0 or 1\n");
				return -1;
			} else {
				pipe_info->pipe_to_csi[pipe_info->pipe_num] = csi_index;
			}
			pipe_info->pipe_num++;
		} else if (deserial_if->port_desp[i] != NULL) {
			if (strlen(deserial_if->port_desp[i]) == 0)
				continue;
			tmp |= (1 << i);
			ret = vin_string_parse(deserial_if->port_desp[i], 'D');
			if (ret < 0) {
				vin_err("port_desp[%d] %s datatype parse fail\n", i, deserial_if->port_desp[i]);
				return ret;
			}
			pipe_info->pipe_datatype[pipe_info->pipe_num] = ret;
			csi_index = deserial_if->deserial_csi[i];
			if (csi_index > CSI_NUM) {
				vin_err("max96712 deserial_csi value only support 0 or 1\n");
				return -1;
			} else {
				pipe_info->pipe_to_csi[pipe_info->pipe_num] = csi_index;
			}
			pipe_info->pipe_num++;
		}
	}
	vin_info("max96712 config link num is %d\n", pipe_info->pipe_num);
	pipe_info->pipe_config_index = tmp;

	return RET_OK;
}

int32_t max96712_one_shot_reset(deserial_info_t *deserial_if, uint8_t link_mask)
{
	int32_t ret = RET_OK;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
					   RESET_ONESHOT_REG,
					   link_mask);
	return ret;
}

static int32_t max96712_gmsl_speed_init(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK, i;
	int8_t gmsl_speed[LINK_MAX] = {-1, -1, -1, -1};
	uint16_t val = 0, gmsl_val = 0, gmsl_mask = 0;
	int16_t gmsl_reg = GMSL_REG;
	int32_t gmsl_set_flag = 0;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	ret = deserial_get_gmsl_speed(deserial_if, gmsl_speed, LINK_MAX);
	if (ret < 0) {
		vin_err("max96712 get gmsl speed fail!!!\n");
		return ret;
	}
	for (i = 0; i < LINK_MAX; i++) {
		if (gmsl_speed[i] == -1)
			continue;
		if (gmsl_speed[i] == GMSL_MODE3) {
			gmsl_val |= GMSL_3GBPS << gmsl_reg_shift[i];
			gmsl_mask |= gmsl_reg_mask[i];
		} else if (gmsl_speed[i] == GMSL_MODE6) {
			gmsl_val |= GMSL_6GBPS <<  gmsl_reg_shift[i];
			gmsl_mask |= gmsl_reg_mask[i];
		} else {
			vin_err("max96712 don't support gmsl speed %dgbps\n", gmsl_speed[i]);
			return -1;
		}
	}
	ret = hb_vin_i2c_read_reg16_data16(bus, slave_addr, gmsl_reg);
	if (ret < 0) {
		vin_err("read max96712 gmsl speed reg 0x%x fail!!!\n", gmsl_reg);
		return -1;
	}
	val = ret & SHIFT_16BIT;
	if ((val & gmsl_mask) == gmsl_val) {  // The current speed is the same as the speed to be set
		vin_info("The rate has already been configured\n");
		return RET_OK;
	}
	val &= (~gmsl_mask);
	val |= gmsl_val;
	vin_dbg("val = 0x%x, gmsl_val = 0x%x, gmsl_reg = 0x%x\n", val, gmsl_val, gmsl_reg);
	ret = hb_vin_i2c_write_reg16_data16(bus, slave_addr, gmsl_reg, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
			slave_addr, gmsl_reg, val);
		return ret;
	}
	return ret;
}

static int32_t max96712_link_lock_check(deserial_info_t *deserial_if, uint8_t link_mask)
{
	int32_t ret = RET_OK, i;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	uint16_t link_lock_reg[LINK_MAX] = {LINKA_LOCK_REG, LINKB_LOCK_REG, \
					    LINKC_LOCK_REG, LINKD_LOCK_REG};
	int32_t timeout;

	for (i = 0; i < LINK_MAX; i ++) {
		timeout = 0;
		if (link_mask & (1 << i)) {
			while (timeout <= LINK_LOCK_TIMEOUT) {     // if timeout 100ms, lock fail!
				ret = hb_vin_i2c_read_reg16_data8(bus, slave_addr, link_lock_reg[i]);
				if (ret < 0) {
					vin_err("%s read link %d lock status fail!!!\n",
						deserial_if->deserial_name, i);
					return ret;
				}
				if (ret & LINK_LOCK_MASK) {
					break;
				}
				if (timeout + 20 > LINK_LOCK_TIMEOUT) {
					vin_err("%s link %d lock timeout %dms!!! link lock reg val is 0x%x.\n",
						deserial_if->deserial_name, i, timeout, ret);
					return -1;
				}
				usleep(20*1000);
				timeout += 20;
			}
		}
	}
	vin_info("%s link is locked, link mask 0x%x, lock time is %dms\n",
		 deserial_if->deserial_name, link_mask, timeout);
	return RET_OK;
}

int32_t max96712_link_enable(deserial_info_t *deserial_if, uint8_t link_mask)
{
	int32_t ret = RET_OK, link_cfg;
	uint8_t val;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	if (max96712_addr_check(deserial_if->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	if (link_mask > LINK_ALL) {
		vin_err("the link_mask 0x%x more then max96712 link max 0x%x\n",
			link_mask, LINK_ALL);
		return -1;
	}
	ret = hb_vin_i2c_lock(bus);
	if (ret < 0)
		return -1;
	link_cfg = ret = hb_vin_i2c_read_reg16_data8(bus, slave_addr, LINK_REG);
	if (ret < 0) {
		vin_err("read %d@0x%x reg 0x%x fail!\n", bus,
			slave_addr, LINK_REG);
		hb_vin_i2c_unlock(bus);
		return -1;
	}
	vin_info("link reg 0x%x is 0x%x\n", LINK_REG, ret);
	val = ret;
	val |= link_mask;
	vin_info("link mask is 0x%x, val = 0x%x\n", link_mask, val);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, LINK_REG, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail\n", bus,
			slave_addr, GMSL_REG, val);
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	ret = max96712_one_shot_reset(deserial_if, link_mask);
	if(ret < 0) {
		vin_info("max96712 one shot reset fail\n");
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	usleep(100 * 1000);
	ret = max96712_link_lock_check(deserial_if, link_mask);
	if (ret < 0) {
		vin_warn("link lock check failed, restore reg 0x%x value 0x%x\n",
			LINK_REG, link_cfg);
		(void)hb_vin_i2c_write_reg16_data8(bus, slave_addr, LINK_REG, link_cfg);
	}
	hb_vin_i2c_unlock(bus);
	return ret;
}

int32_t max96712_link_map(deserial_info_t *deserial_if, uint8_t *pipe_arr)
{
	int32_t ret = RET_OK;
	uint16_t tmp = 0x2222;
	uint8_t val;
	uint8_t i = 0;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	if (pipe_arr == NULL) {
		vin_err("vc_arr is NULL\n");
		return -1;
	}

	for (i = 0; i < LINK_NUM; i++) {
		tmp |= (pipe_arr[i] << (((i+1) * 4) - 2));
	}
	val = tmp & 0xFF;
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
					   LINK_MAP_REG1, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail\n",
				bus, slave_addr, LINK_MAP_REG1, val);
	}
	val = tmp >> 8;
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr,
					   LINK_MAP_REG2, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail\n",
			    bus, slave_addr, LINK_MAP_REG2, val);
	}
	return ret;
}
int32_t max96712_remote_control(deserial_info_t *deserial_if,
				uint8_t link_mask)
{
	int32_t ret = RET_OK;
	uint8_t val = 0xAA;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;

	if (max96712_addr_check(deserial_if->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	val |= (~link_mask);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, REMTCH_REG, val);
	if (ret < 0)
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!\n", bus, slave_addr,
			REMTCH_REG, val);
	return ret;
}

static int32_t max96712_trig_mode_config(deserial_info_t *deserial_if,
					 uint32_t deserial_link, uint8_t gpio_id)
{
	int32_t ret = RET_OK;
	uint32_t trig_pin_num = 0;
	int32_t trig_pin;
	uint16_t reg_addr;
	int32_t i;
	uint32_t bus = deserial_if->bus_num;
	uint8_t deserial_addr = (uint8_t)(deserial_if->deserial_addr);
	int32_t val;

	for (i = 0; i < DES_LINK_NUM_MAX; i++) {
		if (TRIG_PIN(deserial_if, i) > GPIO_INDEX_MAX) {
			vin_err("max96712 trig_pin[%d] is %d, exceeded the maximum value %d!\n",
				 i, TRIG_PIN(deserial_if, i), GPIO_INDEX_MAX);
			return -1;
		}
		if (TRIG_PIN(deserial_if, i) >= 0)
			trig_pin_num++;
	}

	if (trig_pin_num == 1)
		trig_pin = TRIG_PIN(deserial_if, 0);
	else
		trig_pin = TRIG_PIN(deserial_if, deserial_link);

	reg_addr = REG_ADDR_GPIO(trig_pin);
	ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, reg_addr , GPIO_TO_TRIG);
	vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
		deserial_addr, deserial_link, trig_pin, reg_addr, GPIO_TO_TRIG);
	reg_addr = max96712_mfp_index[trig_pin][deserial_link];
	if (deserial_link == LINK_A) {
		val = GPIO_TRIG_VAL0 | (gpio_id + GPIO_ID_OFFSET);
	} else {
		val = GPIO_TRIG_VAL | (gpio_id + GPIO_ID_OFFSET);
	}
	ret = hb_vin_i2c_write_reg16_data8(bus, deserial_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
			bus, deserial_addr, reg_addr, val);
		return ret;
	}
	vin_info("Deserial TX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
		deserial_addr, deserial_link, trig_pin, reg_addr, val);

	return ret;
}

static int32_t max96712_diag_mode_config(deserial_info_t *deserial_if, uint8_t deserial_link)
{
	int32_t ret = RET_OK;
	uint8_t val = 0;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	int8_t gpio_index = 0;
	uint16_t reg_addr;

	reg_addr = MAX96712_REG1;
	val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
	val |= MAX96712_REG1_IIC_EN;
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
			bus, slave_addr, reg_addr, val);
		return ret;
	}

	gpio_index = CAMERR_PIN(deserial_if, deserial_link);
	if (gpio_index < 0) {
		vin_err("max96712 link%d sensor err pin not set\n", deserial_link);
		return -1;
	}
	if (gpio_index > GPIO_INDEX_MAX) {
		vin_err("max96712 camerr_pin[%d] is %d, exceeded the maximum value %d!\n",
			deserial_link, gpio_index, GPIO_INDEX_MAX);
		return -1;
	}
	if (deserial_link == LINK_A)
		val = GPIO_BASE_VAL | GPIO_TO_DIAG_VAL;
	else
		val = GPIO_BASE_VAL;

	reg_addr = REG_ADDR_GPIO(gpio_index);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
				bus, slave_addr, reg_addr, val);
		return ret;
	}
	vin_info("Deserial RX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr, val);

	// config rx id
	reg_addr = max96712_mfp_index[gpio_index][deserial_link] + 1;
	val = hb_vin_i2c_read_reg16_data8(bus, slave_addr, reg_addr);
	val &= (~GPIO_ID_MASK);
	val |= (GPIO_BCD_RX_EN | deserial_link);
	ret = hb_vin_i2c_write_reg16_data8(bus, slave_addr, reg_addr, val);
	vin_info("dev_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
				slave_addr, deserial_link, gpio_index, reg_addr, val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n",
				bus, slave_addr, reg_addr, val);
		return ret;
	}
	vin_info("Deserial RX des_addr:0x%x, des_link:%d mfp:%d, reg_addr:0x%04x, val:0x%02x\n",
			slave_addr, deserial_link, gpio_index, reg_addr, val);
	return ret;
}

int32_t max96712_mfp_cfg(deserial_info_t *deserial_if, uint8_t gpio_mode,
			 uint8_t gpio_id, uint8_t deserial_link)
{
	int32_t ret = RET_OK;

	if (max96712_addr_check(deserial_if->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	if (gpio_mode & GPIO_TX_GMSL) {
		ret = max96712_trig_mode_config(deserial_if, deserial_link, gpio_id);
	} else if (gpio_mode & GPIO_RX_GMSL) {
		ret = max96712_diag_mode_config(deserial_if, deserial_link);
	} else {
		vin_err("max96712 don't support mfp mode 0x%x\n", gpio_mode);
		return -1;
	}
	return ret;
}

int32_t max96712_soft_rst_dpll_config(deserial_info_t *deserial_if, uint32_t num)
{
	int32_t ret = RET_OK;
	uint32_t *pdata = NULL;
	uint32_t setting_size = 0;

	pdata = phy_speed_reset_dpll_config[num];
	setting_size = sizeof(phy_speed_reset_dpll_config[num]) / sizeof(uint32_t) / 2;
	ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr,
						REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("max96712 phy speed reset dpll hold reg set fail!!!\n");
		return ret;
	}

	return ret;
}

int32_t max96712_phy_speed_cfg(deserial_info_t *deserial_if, uint32_t link_num)
{
	int32_t ret = RET_OK, i;
	int8_t val;
	uint16_t reg;
	uint32_t bus = deserial_if->bus_num;
	uint8_t slave_addr = deserial_if->deserial_addr;
	uint8_t phy_speed;
	// uint32_t phy_mode;
	uint32_t phy_copy = 0;
	uint32_t setting_size;

	phy_copy = (deserial_if->lane_mode & (1 << 6)) ? 1 : 0;
	if (link_num == 1) {
		phy_speed = ((deserial_if->lane_speed) ? deserial_if->lane_speed : 1200) / 100;
	} else {
		phy_speed = ((deserial_if->lane_speed) ? deserial_if->lane_speed : 2000) / 100;
	}
	// phy_mode = (deserial_if->lane_mode & (1 << 5)) ? 1 : 0;
    max96712_soft_rst_dpll_config(deserial_if, MAX96712_DPLL_HOLD);
	for (i = 0; i < CSI_NUM; i++) {
		reg = max96712_phy_speed_reg[i];
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
			return ret;
		}
	}
	max96712_soft_rst_dpll_config(deserial_if, MAX96712_DPLL_REL);
	if (phy_copy == 1) {
		setting_size = sizeof(max96712_phy_copy_init_setting)/sizeof(uint32_t)/2;
		ret = vin_write_array(bus, slave_addr, REG16_VAL8,
				      setting_size, max96712_phy_copy_init_setting);
		if (ret < 0)
			vin_info("max96712 phy copy setting fail\n");
	}
	return ret;
}

int32_t data_intf_init(deserial_info_t *deserial_if, pipe_info_t *pipe_info, uint8_t *intf_init)
{
	int32_t ret = RET_OK, i, intf_type, desport;
	uint32_t reg_addr, setting_size = 0, dvp_sensor_num = 0;
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
			} else if (intf_type == DVP) {
				dvp_sensor_num++;
			} else {
				vin_dbg("sensor data type not define!!!\n");
			}
		}
	}
	for (i = 0; i < dvp_sensor_num; i++) {
		pdata = max96712_datatype_bpp_mux_init_setting[i];
		setting_size = sizeof(max96712_datatype_bpp_mux_init_setting[0]) / sizeof(uint32_t) / 3;
		ret = vin_i2c_bit_array_write8(deserial_if->bus_num, deserial_if->deserial_addr,
							REG_WIDTH_16bit, setting_size, pdata);
		if (ret < 0) {
			vin_err("i2c bit array write data fail!!!\n");
			return ret;
		}
	}
	*intf_init = 1;

	return ret;
}

int32_t max96712_pipe_config(deserial_info_t *deserial_if, pipe_info_t *pipe_info)
{
	int32_t ret = RET_OK, i;
	uint8_t val = 0, intf_init = 0;
	int32_t pipe_config_index, setting_size = 0;
	uint32_t *pdata = NULL;

	/* Link-pipe Map */
	pipe_config_index = pipe_info->pipe_config_index;
	ret = max96712_link_map(deserial_if, max96712_pipe_map[pipe_config_index]);
	if (ret < 0) {
		vin_err("max96712 link map fail!!!\n");
		return ret;
	}

	/* Pipe-mipi config */
	for(i = 0; i < pipe_info->pipe_num; i++) {
		if (pipe_info->pipe_datatype[i] == EMODE_RAW12) {
			setting_size = sizeof(max96712_raw_pipe_to_mipi_config[0])/sizeof(uint32_t)/2;
			pdata = max96712_raw_pipe_to_mipi_config[i];
		} else if (pipe_info->pipe_datatype[i] == EMODE_YUV422) {
			setting_size = sizeof(max96712_yuv_pipe_to_mipi_config[i])/sizeof(uint32_t)/2;
			pdata = max96712_yuv_pipe_to_mipi_config[i];
			ret = data_intf_init(deserial_if, pipe_info, &intf_init);
		    if (ret < 0) {
			    vin_err("parse data intf fail!\n");
			}
		} else {
			vin_err("Don't support datatype %d\n", pipe_info->pipe_datatype[i]);
			return ret;
		}
		if (pipe_info->pipe_to_csi[i] == 1) {
			pdata[3] = PIPE_MAP_TO_CSI1;
		} else {
			pdata[3] = PIPE_MAP_TO_CSI0;
		}
		setting_size = sizeof(max96712_raw_pipe_to_mipi_config[0])/sizeof(uint32_t)/2;
		ret = vin_write_array(deserial_if->bus_num, deserial_if->deserial_addr,
				      REG16_VAL8, setting_size, pdata);
		if (ret < 0) {
			vin_err("max96712 pipe to mipi config fail!!!\n");
			return ret;
		}
		val |= (1 << i);
	}
	/* Pipe enable */
	ret = hb_vin_i2c_write_reg16_data8(deserial_if->bus_num, deserial_if->deserial_addr,
					   PIPE_ENABLE_REG, val);
	if (ret < 0)
		vin_err("max96712 pipe config fail!!!\n");

	return ret;
}

static int32_t deserial_init_rec_check(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;

	if (deserial_if->deserial_attr & BIT(1)) {
		ret = -NOT_INIT_FLAG;
		return ret;
	}

	return ret;
}

static int32_t deserializer_stream_on(deserial_info_t *max96712_info, uint32_t port)
{
	int32_t ret = RET_OK;
	int32_t setting_size;

	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	setting_size = sizeof(max96712_stream_on_setting) / sizeof(uint32_t) / 3;
	ret = vin_i2c_bit_array_write8(max96712_info->bus_num, max96712_info->deserial_addr,
			      REG_WIDTH_16bit, setting_size, max96712_stream_on_setting);
	if (ret < 0)
		vin_err("max96712 stream on fail\n");
	return ret;
}

static int32_t deserializer_stream_off(deserial_info_t *max96712_info, uint32_t port)
{
	int32_t ret = RET_OK;
	int32_t setting_size;

	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	setting_size = sizeof(max96712_stream_off_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(max96712_info->bus_num, max96712_info->deserial_addr,
			      REG16_VAL8, setting_size, max96712_stream_off_setting);
	if (ret < 0)
		vin_err("max96712 stream off fail\n");
	return ret;
}

int32_t  max96712_init(deserial_info_t *deserial_if)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus, deserial_addr;
	pipe_info_t pipe_info = {0};

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	vin_info("max96712 init begin!\n");
	ret = max96712_get_deserial_link_info(deserial_if, &pipe_info);
	if (ret < 0) {
		vin_err("max96712 get deserial link info fail!!!\n");
		return ret;
	}
	bus = deserial_if->bus_num;
	deserial_addr = deserial_if->deserial_addr;

	ret = deserial_init_rec_check(deserial_if);
	if(ret < 0) {
		if (ret == -NOT_INIT_FLAG) {
			vin_info("des %s not need to init\n", deserial_if->deserial_name);
			return RET_OK;
		}
	}

	ret = max96712_reset(deserial_if);
	if (ret < 0) {
		vin_err("max96712 reset fail!\n");
		return ret;
	}
	ret = max96712_gmsl_speed_init(deserial_if);
	if (ret < 0) {
		vin_info("%s gmsl speed init fail!!!\n", deserial_if->deserial_name);
		return ret;
	}
	pdata = max96712_base_setting;
	setting_size = sizeof(max96712_base_setting)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, deserial_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("deserial init fail!\n");
		return ret;
	}
	ret = max96712_phy_speed_cfg(deserial_if, pipe_info.pipe_num);
	if (ret < 0) {
		vin_err("max96712 phy speed config fail!\n");
		return ret;
	}
	ret = max96712_pipe_config(deserial_if, &pipe_info);
	if (ret < 0) {
		vin_err("max96712 pipe config fail!!!\n");
		return ret;
	}

	return ret;
}

static int32_t deserializer_deinit(deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;

	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	// to do
	return ret;
}
static int32_t deserializer_init(deserial_info_t *max96712_info)
{
	int32_t req, ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus, deserial_addr;
	int32_t i;
	sensor_info_t *sensor_info = NULL;
	sensor_module_t *sensor_ops = NULL;
	uint32_t entry_num = 0;

	if (max96712_info == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	for (i = 0; i < DES_LINK_NUM_MAX; i ++) {
		if (max96712_info->sensor_info[i] != NULL) {
			sensor_info = ((sensor_info_t *)(max96712_info->sensor_info[i]));
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
	vin_info("max96712 init begin!\n");
	req = hb_vin_mipi_pre_request((uint32_t)entry_num, 0, 0);
	if (req == 0) {
		ret = max96712_init(max96712_info);
		hb_vin_mipi_pre_result((uint32_t)entry_num, 0, (uint32_t)ret);		/* PRQA S 2897 */
		if(ret < 0) {
			vin_err("max96712_init fail!\n");
			return ret;
		}
	}
	return ret;
}

static int32_t deserializer_start_physical(const deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;

	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	// to do
	return ret;
}

static int32_t deserializer_reset(const deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	uint32_t gpio;

	if(max96712_info->power_mode == 1u) {
		for(gpio = 0; gpio < max96712_info->gpio_num; gpio++) {
			if(max96712_info->gpio_pin[gpio] >=0) {
				ret = vin_power_ctrl((uint32_t)max96712_info->gpio_pin[gpio],
						      max96712_info->gpio_level[gpio]);
			}
		}
		(void)usleep(DES_RESET_DELAY);
		for(gpio = 0; gpio < max96712_info->gpio_num; gpio++) {
			if(max96712_info->gpio_pin[gpio] >= 0) {
				ret =(int32_t)((uint32_t)ret | (uint32_t)vin_power_ctrl((uint32_t)max96712_info->gpio_pin[gpio],
							1-max96712_info->gpio_level[gpio]));
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
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

static void sub_node_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node,
		node_info_t sub_node)
{
	int32_t entry_num;
	uint32_t i, link, bit_field, link_mask, m_port_mask, m_diag_id;
	diag_node_info_t *sub_reg_node = NULL;
	entry_num = get_enrty_num_of_sen(max96712_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	for (i = 0, bit_field = 0; i < MAX_BIT_NUM; i++) {
		if ((sub_node.mask & ((uint8_t)1u << i)) != 0) {
			link = (sub_node.subevent_id) ? (sub_node.subevent_id + bit_field) : (i+1);
			m_diag_id = CAM_DES_DIAG_ID(entry_num, link, sub_node.subtype, sub_node.subid);
			link_mask = (sub_node.subevent_id == LINK_ALL_FLAG) ? LINK_ALL : BIT(link-1);
			m_port_mask = vin_port_mask_of_des(max96712_info, link_mask);
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
			sub_reg_node->diag_info.t.reg.reg_bus = max96712_info->bus_num;
			sub_reg_node->diag_info.t.reg.reg_dev = max96712_info->deserial_addr;
			sub_reg_node->diag_info.t.reg.reg_addr = sub_node.reg_addr;
			sub_reg_node->diag_info.t.reg.reg_mask = (uint16_t)1u << i;  // bit0~3, linka
			sub_reg_node->fault_clear = sub_node.fault_clear;
			sub_reg_node->cb_data = sub_node.cb_data;
			sub_reg_node->test_fault_inject = sub_node.test_fault_inject;
			if (sub_node.subid == MAX96712_UNLOCK) {
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

static void node_init(deserial_info_t *max96712_info, node_info_t node)
{
	int32_t entry_num;
	uint32_t i, link, bit_field, link_mask, m_port_mask, m_diag_id;
	diag_node_info_t *reg_node;
	entry_num = get_enrty_num_of_sen(max96712_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	for (i = 0, bit_field = 0; i < MAX_BIT_NUM; i++) {
		if ((node.mask & ((uint8_t)1u << i)) != 0) {
			link = (node.subevent_id) ? (node.subevent_id + bit_field) : (i+1);
			m_diag_id = CAM_DES_DIAG_ID(entry_num, link, node.subtype, node.subid);
			link_mask = (node.subevent_id == LINK_ALL_FLAG) ? LINK_ALL : BIT(link-1);
			m_port_mask = vin_port_mask_of_des(max96712_info, link_mask);
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
			reg_node->diag_info.t.reg.reg_bus = max96712_info->bus_num;
			reg_node->diag_info.t.reg.reg_dev = max96712_info->deserial_addr;
			reg_node->diag_info.t.reg.reg_addr = node.reg_addr;
			reg_node->diag_info.t.reg.reg_mask = (uint16_t)1u << i;
			reg_node->fault_clear = node.fault_clear;
			reg_node->cb_data = node.cb_data;
			reg_node->test_fault_inject = node.test_fault_inject;
			if (node.subid == MAX96712_UNLOCK) {
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
				reg = DEC_ERR_CNT_A_96712REG;
				break;
			case 1:
				reg = DEC_ERR_CNT_B_96712REG;
				break;
			case 2:
				reg = DEC_ERR_CNT_C_96712REG;
				break;
			case 3:
				reg = DEC_ERR_CNT_D_96712REG;
				break;
			default:
				reg = -1;
				break;
			}
			if(reg > 0) {
				vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
			}
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
			case 2:
				sen = (sensor_info_t*)des_info->sensor_info[2];
				break;
			case 3:
				sen = (sensor_info_t*)des_info->sensor_info[3];
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

static void decode_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 15;  // bit0~3, linka
	sub_node_info.reg_addr = 0x26;  // dec
	sub_node_info.subid = MAX96712_DEC_ERR;
	sub_node_info.subtype = DES_DEC_ERR_FLAG;
	sub_node_info.fault_clear = decode_fault_clear;
	sub_node_info.cb_data = (void*)max96712_info;
	sub_node_info.test_fault_inject = test_decode_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static int32_t idle_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i;
	int32_t bus, dev, reg;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				reg = IDLE_ERR_CNT_A_96712REG;
				break;
			case 1:
				reg = IDLE_ERR_CNT_B_96712REG;
				break;
			case 2:
				reg = IDLE_ERR_CNT_C_96712REG;
				break;
			case 3:
				reg = IDLE_ERR_CNT_D_96712REG;
				break;
			default:
				reg = -1;
				break;
			}
			if(reg > 0) {
				vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
			}
		}
	}
	return 0;
}

int32_t test_idle_fault_inject(struct diag_node_info_s * node, int32_t inject)
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
			case 2:
				sen = (sensor_info_t*)des_info->sensor_info[2];
				break;
			case 3:
				sen = (sensor_info_t*)des_info->sensor_info[3];
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

static void idle_word_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 15;  // bit0~3, linka
	sub_node_info.reg_addr = 0x2c;  // idle
	sub_node_info.subid = MAX96712_IDLE_ERR;
	sub_node_info.subtype = DES_IDLE_ERR_FLAG;
	sub_node_info.fault_clear = idle_fault_clear;
	sub_node_info.cb_data = (void*)max96712_info;
	sub_node_info.test_fault_inject = test_idle_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static int32_t lcrc_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i, j, num;
	int32_t bus, dev;
	int32_t clear_reg[] = {	LCRC_ERR_CNT_0_96712REG,
							LCRC_ERR_CNT_1_96712REG,
							LCRC_ERR_CNT_2_96712REG,
							LCRC_ERR_CNT_3_96712REG,
							LCRC_ERR_CNT_4_96712REG,
							LCRC_ERR_CNT_5_96712REG,
							LCRC_ERR_CNT_6_96712REG,
							LCRC_ERR_CNT_7_96712REG };
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

static void video_line_crc_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3
	sub_node_info.reg_addr = 0x2a;  // lcrc
	sub_node_info.subid = MAX96712_LCRC_ERR;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	sub_node_info.subtype = DES_LCRC_ERR;
	sub_node_info.fault_clear = lcrc_fault_clear;
	sub_node_info.cb_data = (void*)max96712_info;
	sub_node_info.test_fault_inject = test_lcrc_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
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
			case 0:
				reg = MAX_RT_ERR_CNT_A_96712REG;
				break;
			case 1:
				reg = MAX_RT_ERR_CNT_B_96712REG;
				break;
			case 2:
				reg = MAX_RT_ERR_CNT_C_96712REG;
				break;
			case 3:
				reg = MAX_RT_ERR_CNT_D_96712REG;
				break;
			default:
				reg = -1;
				break;
			}
			if(reg > 0) {
				vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
			}
		}
	}
	return 0;
}

int32_t test_max_rtra_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i;
	int32_t bus, dev, reg1, reg2, reg3, value1, value2, value3;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;

	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				reg1 = MAX96712_GMSLA_ERRG_EN;
				reg2 = MAX96712_GMSLA_ERRG_CGF;
				reg3 = MAX96712_MAX_RT_LIMIT0;
				break;
			case 1:
				reg1 = MAX96712_GMSLB_ERRG_EN;
				reg2 = MAX96712_GMSLB_ERRG_CGF;
				reg3 = MAX96712_MAX_RT_LIMIT1;
				break;
			case 2:
				reg1 = MAX96712_GMSLC_ERRG_EN;
				reg2 = MAX96712_GMSLC_ERRG_CGF;
				reg3 = MAX96712_MAX_RT_LIMIT2;
			break;
			case 3:
				reg1 = MAX96712_GMSLD_ERRG_EN;
				reg2 = MAX96712_GMSLD_ERRG_CGF;
				reg3 = MAX96712_MAX_RT_LIMIT3;
				break;
			default:
				reg1 = -1; reg2 = -1; reg3 = -1;
				break;
			}
			if (reg1 >= 0 && reg2 >= 0 && reg3 >=0) {
					if (inject) {
						vin_info("diag_id 0x%x, inject\n", node->diag_id);
						value1 = 0x10; value2 = 0x08; value3 = 0x12;
					} else {
						vin_info("diag_id 0x%x, disable\n", node->diag_id);
						value1 = 0x08; value2 = 0x20; value3 = 0x72;
					}
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										(uint16_t)reg1,
										(uint16_t)value1);
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										(uint16_t)reg2,
										(uint16_t)value2);
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										(uint16_t)reg3,
										(uint16_t)value3);
			}
		}
	}
	return 0;
}

static void arq_max_rtra_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 15;  // bit0~3
	sub_node_info.reg_addr = 0x2e;  // MAX_RT
	sub_node_info.subid = MAX96712_MAX_RT;
	sub_node_info.subtype = DES_MAX_RT_FLAG_0;
	sub_node_info.fault_clear = max_rtra_fault_clear;
	sub_node_info.test_fault_inject = test_max_rtra_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static int32_t vid_pxl_crc_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i;
	int32_t bus, dev, reg;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 0:
				reg = VID_PXL_CRC_CNT_AZ_96712REG;
				break;
			case 1:
				reg = VID_PXL_CRC_CNT_BZ_96712REG;
				break;
			case 2:
				reg = VID_PXL_CRC_CNT_CZ_96712REG;
				break;
			case 3:
				reg = VID_PXL_CRC_CNT_DZ_96712REG;
				break;
			default:
				reg = -1;
				break;
			}
			if (reg > 0)
				vin_i2c_read_retry(bus, dev, REG16_VAL8, reg);
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
			case 1:
				sen = (sensor_info_t*)des_info->sensor_info[1];
				break;
			case 2:
				sen = (sensor_info_t*)des_info->sensor_info[2];
				break;
			case 3:
				sen = (sensor_info_t*)des_info->sensor_info[3];
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

static void vid_pxl_crc_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 15;  // bit0~3
	sub_node_info.reg_addr = 0x45;  // VID_PXL
	sub_node_info.subid = MAX96712_VID_PXL_CRC;
	sub_node_info.subtype = DES_VID_PXL_CRC_ERR_0;
	sub_node_info.fault_clear = vid_pxl_crc_fault_clear;
	sub_node_info.cb_data = (void*)max96712_info;
	sub_node_info.test_fault_inject = test_vid_pxl_crc_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static int32_t mem_ecc_fault_clear(struct diag_node_info_s * node)
{
	uint8_t i;
	int32_t bus, dev, reg;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 7: {
					reg = MEM_ECC_ERR2_CNT_96712REG;
					vin_i2c_write_retry(bus, dev, REG16_VAL8, reg, 0x02);
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

int32_t test_mem_ecc_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i;
	int32_t bus, dev;
	uint16_t value1, value2;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;

	for (i = 0; i < MAX_BIT_NUM; i++) {
		if ((node->diag_info.t.reg.reg_mask & ((uint8_t)1 << i)) != 0) {
			switch (i) {
			case 7: {
					inject_pf(node, inject);
					value1 = inject ? 0x02 : 0x00;
					vin_i2c_write_retry(bus,
										dev,
										REG16_VAL8,
										MEM_ECC_FAULT_INJECT_96712REG,
										value1);
				}
				break;
			default:
				break;
			}
		}
	}
	return 0;
}

static void mem_ecc_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 128;  // bit7
	sub_node_info.reg_addr = 0x45;  // VID_PXL
	sub_node_info.subid = MAX96712_MEM_ECC;
	sub_node_info.subtype = DES_MEM_ECC_ERR2;
	sub_node_info.subevent_id = LINK_ALL_FLAG;
	sub_node_info.fault_clear = mem_ecc_fault_clear;
	sub_node_info.test_fault_inject = test_mem_ecc_fault_inject;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static void linka_lock_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3, linka
	sub_node_info.reg_addr = 0x1a;  // link lock
	sub_node_info.subid = MAX96712_UNLOCK;
	sub_node_info.subtype = DES_UNLOCK;
	sub_node_info.subevent_id = 1;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static void linkb_lock_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3, linkb
	sub_node_info.reg_addr = 0x0a;  // link lock
	sub_node_info.subid = MAX96712_UNLOCK;
	sub_node_info.subtype = DES_UNLOCK;
	sub_node_info.subevent_id = 2;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static void linkc_lock_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3, linkc
	sub_node_info.reg_addr = 0x0b;  // link lock
	sub_node_info.subid = MAX96712_UNLOCK;
	sub_node_info.subtype = DES_UNLOCK;
	sub_node_info.subevent_id = 3;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

static void linkd_lock_diag_init(deserial_info_t *max96712_info, diag_node_info_t *parent_node)
{
	node_info_t sub_node_info;
	(void)memset((void *)&sub_node_info, 0, sizeof(sub_node_info));
	sub_node_info.mask = 8;  // bit3, linkd
	sub_node_info.reg_addr = 0x0c;  // link lock
	sub_node_info.subid = MAX96712_UNLOCK;
	sub_node_info.subtype = DES_UNLOCK;
	sub_node_info.subevent_id = 4;
	if (parent_node == NULL) {
		node_init(max96712_info, sub_node_info);
	} else {
		sub_node_init(max96712_info, parent_node, sub_node_info);
	}
}

int32_t test_line_memory_fault_inject(struct diag_node_info_s * node, int32_t inject)
{
	uint8_t i, num;
	int32_t bus, dev;
	uint16_t value1;
	bus = node->diag_info.t.reg.reg_bus;
	dev = node->diag_info.t.reg.reg_dev;
	int32_t dpll_reg[] = {	MAX96712_PYH0_DPLL,
							MAX96712_PYH1_DPLL,
							MAX96712_PYH2_DPLL,
							MAX96712_PYH3_DPLL};

	num = sizeof(dpll_reg) / sizeof(dpll_reg[0]);
	if (inject) {
		vin_info("diag_id 0x%x, inject\n", node->diag_id);
		value1 = 0x21;
	} else {
		vin_info("diag_id 0x%x, disable\n", node->diag_id);
		value1 = 0x34;
	}
	for (i = 0; i < num; i++) {
		vin_i2c_write_retry(bus,
							dev,
							REG16_VAL8,
							dpll_reg[i],
							value1);
	}
	return 0;
}

static void line_memory_diag_init(deserial_info_t *max96712_info)
{
	node_info_t node_info;
	(void)memset((void*)&node_info, 0, sizeof(node_info_t));
	node_info.mask = 15;
	node_info.reg_addr = 0x40a;
	node_info.subid = MAX96712_LMO_ERR;
	node_info.subtype = DES_LMO_ERR;
	node_info.test_fault_inject = test_line_memory_fault_inject;
	node_init(max96712_info, node_info);
}


int32_t max96712_diag_enable(deserial_info_t *max96712_info)
{
	int32_t ret = RET_OK;
	int32_t setting_size = 0;
	uint32_t *pdata = NULL;
	uint32_t bus, deserial_addr;

	if (max96712_info == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}

	bus = max96712_info->bus_num;
	deserial_addr = max96712_info->deserial_addr;
	pdata = max96712_diag_cfg;
	setting_size = sizeof(max96712_diag_cfg)/sizeof(uint32_t)/2;
	ret = vin_write_array(bus, deserial_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0)
		vin_err("deserial init fail!\n");

	return ret;
}


static int32_t deserializer_diag_nodes_init(deserial_info_t *max96712_info)
{
	int32_t entry_num;
	int32_t ret = RET_OK;
	int32_t gpio_index = -1;
	diag_node_info_t *parent_node = NULL;
	diag_node_info_t *parent_node1 = NULL;

	vin_info("diag init\n");
	if (max96712_addr_check(max96712_info->deserial_addr) < 0) {
		vin_dbg("deserial addr invalid\n");
		return 0;
	}
	ret = max96712_diag_enable(max96712_info);
	if(ret < 0) {
		vin_err("max96712 diag init fail!\n");
		return ret;
	}
	/****************************errb*************************/
	gpio_index = ERRB_PIN(max96712_info);
	entry_num = get_enrty_num_of_sen(max96712_info);
	entry_num = (entry_num < 0) ? 0xf : entry_num;
	if (gpio_index > 0) {
		parent_node = cam_diag_get_nodes(1);
		if (parent_node) {
			parent_node->diag_id = \
			CAM_DES_DIAG_ID((uint8_t)entry_num, LINK_ALL_FLAG, DES_ERRB, MAX96712_ERRB);
			parent_node->port_mask = vin_port_mask_of_des(max96712_info, LINK_MASK_ALL);
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

	decode_diag_init(max96712_info, parent_node);
	idle_word_diag_init(max96712_info, parent_node);
	video_line_crc_diag_init(max96712_info, parent_node);
	arq_max_rtra_diag_init(max96712_info, parent_node);
	vid_pxl_crc_diag_init(max96712_info, parent_node);
	mem_ecc_diag_init(max96712_info, parent_node);
	/****************************link lock*************************/
	gpio_index = LOCK_PIN(max96712_info);
	if (gpio_index > 0) {
		parent_node1 = cam_diag_get_nodes(1);
		if (parent_node1) {
			parent_node1->diag_id = \
			CAM_DES_DIAG_ID((uint8_t)entry_num, LINK_ALL_FLAG, DES_UNLOCK, MAX96712_LOCK);
			parent_node1->port_mask = vin_port_mask_of_des(max96712_info, LINK_MASK_ALL);
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
	linka_lock_diag_init(max96712_info, parent_node1);
	linkb_lock_diag_init(max96712_info, parent_node1);
	linkc_lock_diag_init(max96712_info, parent_node1);
	linkd_lock_diag_init(max96712_info, parent_node1);

	/****************************reg poll*************************/
	line_memory_diag_init(max96712_info);
	vin_info("diad init end\n");
	return ret;
}
#endif

#ifdef CAMERA_FRAMEWORK_HBN
DESERIAL_MAX_MODULE(max96712, max96712_link_enable, NULL, max96712_mfp_cfg);
deserial_module_t max96712 = {
	.module = DESERIAL_MNAME(max96712),
#else
deserial_module_t max96712 = {
	.module = "max96712",
	.ops.max = {
		.link_enable = max96712_link_enable,
		.remote_control = max96712_remote_control,
		.mfp_cfg = max96712_mfp_cfg,
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

