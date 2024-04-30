/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[max_serial]:%s " fmt, __func__

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "../hb_cam_utility.h"
#include "../hb_cam_gpio.h"
#include "../hb_i2c.h"
#include "../hb_spi.h"
#include "../../inc/cam_common.h"

#include "./max_serial.h"

#define INVALID_POC_ADDR		(0xFF)
#define DEFAULT_POC_ADDR		(0x28)

#define MAX9296_RSTREG		(0x10)
#define MAX9296_AUTOLINK	BIT(4)
#define DUAL_PATCH_CHECK_REG	(0x83)
#define DUAL_PATCH_CHECK_VAL	BIT(2)

static uint32_t max9295_ldo_enable[] = {
	0x10, 0x04, 0x04,
	0x12, 0x10, 0x10,
};
uint32_t max9295_pipeline_raw12_emb_setting[] = {
	0x0312u, 0x04u,   // Double EMB8 on pipe Z
	0x031Eu, 0x2Cu,   // Min BPP = 12 on pipe Z
	0x0111u, 0x50u,   // Max BPP = 16
	0x0110u, 0x60u,   // Disble auto BPP

	//0x0112u, 0x0Eu,   // Limit heartbeat
	0x0057u, 0x00u,
};

uint32_t max9295_pipeline_raw12_setting[] = {
	0x0002u, 0x43u,    // Enable pipe Z
	0x0308u, 0x64u,    // Enable CSI B and select pipe Z
	0x0311u, 0x40u,    // Start Video Pipe Z form mipi csi B
	0x0318u, 0x6cu,    // Enalbe datatype routing, and set datatype raw
	0x0057u, 0x00u,
};

uint32_t max96717_pipeline_raw12_setting[] = {
	0x0002u, 0x43u,    // Enable pipe Z
	0x0308u, 0x64u,    // Enable CSI B and select pipe Z
	0x0311u, 0x40u,    // Start Video Pipe Z form mipi csi B
	0x0318u, 0x6cu,    // Enalbe datatype routing, and set datatype raw
	0x0383u, 0x00u,    // Default Pixel mode
};

uint32_t max9295_pipeline_yuv_setting[] = {
	0x0002u, 0x43u,    // Enable pipe Z
	0x0308u, 0x64u,    // Enable CSI B and select pipe Z
	0x0311u, 0x40u,    // Start Video Pipe Z form mipi csi B

	0x0318u, 0x5eu,    // Enalbe datatype routing, and set datatype raw
	0x0057u, 0x00u,
};
uint32_t max96717_pipeline_yuv_setting[] = {
	0x0002u, 0x43u,    // Enable pipe Z
	0x0308u, 0x64u,    // Enable CSI B and select pipe Z
	0x0311u, 0x40u,    // Start Video Pipe Z form mipi csi B

	0x0318u, 0x5eu,    // Enalbe datatype routing, and set datatype raw
};

uint32_t max9296_dual_patch[] = {
	0x006B, 0x16,
	0x0073, 0x17,
	0x007B, 0x36,
	0x0083, 0x36,
	0x0093, 0x36,
	0x009B, 0x36,
	0x00A3, 0x36,
	0x00AB, 0x36,
	0x008B, 0x36,
};

uint32_t dvp_serializer_pipex_setting[] = {
	/* DVP configuration for serial pipe X */
	0x01b0, 0x04,
	0x01b1, 0x05,
	0x01b2, 0x06,
	0x01b3, 0x07,
	0x01b4, 0x08,
	0x01b5, 0x09,
	0x01b6, 0x0a,
	0x01b7, 0x0b,
	0x0007, 0xF7, 	// Parellel enable
	0x0002, 0x13, 	// Turn on pipe x
	0x0053, 0x02,   /* pipe x -> stream 2 0x02 */
	0x0100, 0x60,	/* Line-CRC enabled, HS,VS,DE encoding on */
	0x0101, 0x4A,	/* Color bits per pixel */
	0x0311, 0x10,	/* csi-b -> x */
	0x0308, 0x61,	/* csi-b */
};

uint32_t serial_rclk_output_enable[] = {
	0x0003, 0x03,	 // Reference PLL output
	0x0006, 0xb0,	 // RCLK output enabale
};

int32_t poc_reset(uint32_t bus, uint8_t slave_addr, uint8_t reset_mask)
{
	int32_t ret = RET_OK;
	int32_t val;

	/* POC Power Down */
	vin_info("poc_reset begain\n");
	ret = hb_vin_i2c_lock(bus);
	if (ret < 0)
		return -1;
	val = hb_vin_i2c_read_reg8_data8(bus, slave_addr, POC_REG_ADDR);
	if (val < 0) {
		vin_err("read %d@0x%x reg 0x%x fail!!!\n", bus, slave_addr, POC_REG_ADDR);
		hb_vin_i2c_unlock(bus);
		return -1;
	}
	val &= (~reset_mask);
	ret = hb_vin_i2c_write_reg8_data8(bus, slave_addr, POC_REG_ADDR, (uint8_t)val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
				slave_addr, POC_REG_ADDR, val);
		hb_vin_i2c_unlock(bus);
		return -1;
	}
	hb_vin_i2c_unlock(bus);
	usleep(100 *1000);
	vin_info("poc power down\n");
	/* POC Powern Up*/
	ret = hb_vin_i2c_lock(bus);
	if (ret < 0)
		return -1;
	val = hb_vin_i2c_read_reg8_data8(bus, slave_addr, (uint8_t)POC_REG_ADDR);
	if (val < 0) {
		vin_err("read %d@0x%x reg 0x%x fail!!!\n", bus, slave_addr, POC_REG_ADDR);
		hb_vin_i2c_unlock(bus);
		return -1;
	}
	val |= reset_mask;
	ret = hb_vin_i2c_write_reg8_data8(bus, slave_addr, POC_REG_ADDR, (uint8_t)val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x fail!!!\n", bus,
				slave_addr, POC_REG_ADDR, val);
		hb_vin_i2c_unlock(bus);
		return -1;
	}
	hb_vin_i2c_unlock(bus);
	usleep(100 *1000);
	return ret;
}

int32_t dvp_sensor_serial_init(sensor_info_t *sensor_info)
{
	uint32_t setting_size = 0, *pdata = NULL;
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	int32_t ser_model;

	ser_model = vin_sensor_emode_parse(sensor_info, 'S');
	if (ser_model == MAX9295A) {
		pdata = dvp_serializer_pipex_setting;
		setting_size = sizeof(dvp_serializer_pipex_setting) / sizeof(uint32_t) / 2;
	}
	ret = vin_write_array(deserial_if->bus_num, sensor_info->serial_addr,
							REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("write serial with dvp sensor register error\n");
		return ret;
	}

	return ret;
}
int32_t max9295_pipeline_init(uint32_t bus, uint8_t slave_addr,
			      uint8_t datatype, uint32_t emb_flag)
{
	int32_t ret = RET_OK;
	uint32_t *pdata = NULL;
	int32_t setting_size = 0;

	setting_size = sizeof(max9295_ldo_enable) / sizeof(uint32_t) / 3;
	ret = vin_i2c_bit_array_write8(bus, slave_addr, REG_WIDTH_16bit, setting_size, max9295_ldo_enable);
	if (ret < 0) {
		vin_err("max9295 enable ldo fail!!!\n");
		return ret;
	}
	switch (datatype) {
		case EMODE_RAW12:
			if (emb_flag == 1) {
				pdata = max9295_pipeline_raw12_emb_setting;
				setting_size = sizeof(max9295_pipeline_raw12_emb_setting)/sizeof(uint32_t)/2;
			} else {
				pdata = max9295_pipeline_raw12_setting;
				setting_size = sizeof(max9295_pipeline_raw12_setting)/sizeof(uint32_t)/2;
			}
			break;
		case EMODE_YUV422:
			pdata = max9295_pipeline_yuv_setting;
			setting_size = sizeof(max9295_pipeline_yuv_setting)/sizeof(uint32_t)/2;
			break;
		default:
			vin_err("Don't support datatype 0x%x", datatype);
			return -1;
	}
	ret = vin_write_array(bus, slave_addr, REG16_VAL8,
						  setting_size, pdata);
	if (ret < 0)
		vin_err("serial pipeline init fail!\n");
	return ret;
}
int32_t max96717_pipeline_init(uint32_t bus, uint8_t slave_addr, uint8_t datatype)
{
	int32_t ret = RET_OK;
	uint32_t *pdata = NULL;
	int32_t setting_size = 0;

	switch (datatype) {
		case EMODE_RAW12:
			pdata = max96717_pipeline_raw12_setting;
			setting_size = sizeof(max96717_pipeline_raw12_setting)/sizeof(uint32_t)/2;
			break;
		case EMODE_YUV422:
			pdata = max96717_pipeline_yuv_setting;
			setting_size = sizeof(max96717_pipeline_yuv_setting)/sizeof(uint32_t)/2;
			break;
		default:
			vin_err("Don't support datatype 0x%x", datatype);
			return -1;
	}
	ret = vin_write_array(bus, slave_addr, REG16_VAL8,
						  setting_size, pdata);
	if (ret < 0)
		vin_err("serial pipeline init fail!\n");
	return ret;
}

int32_t serial_pipeline_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t slave_addr = sensor_info->serial_addr & SHIFT_8BIT;
	uint32_t bus = sensor_info->bus_num;
	int32_t datatype, ser_model;
	uint32_t emb_flag = 0;

	emb_flag = (sensor_info->config_index & BIT(B_EMBEDDED_MODE)) ? 1 : 0;
	datatype = vin_sensor_emode_parse(sensor_info, 'D');
	ser_model = vin_sensor_emode_parse(sensor_info, 'S');
	switch (ser_model) {
		case MAX9295A:
			ret = max9295_pipeline_init(bus, slave_addr, datatype, emb_flag);
			break;
		case MAX96717:
		case MAX96717F:
			ret = max96717_pipeline_init(bus, slave_addr, datatype);
			break;
		default:
			vin_err("Don't support serial 0x%x", ser_model);
			return -1;
	}
	return ret;
}

int32_t serial_i2c_addr_map(uint32_t bus, uint8_t dst_addr, uint8_t src_addr)
{
	int32_t ret = RET_OK;
	uint8_t val = 0;
	if (dst_addr == src_addr)
		return ret;

	val = (uint8_t)(src_addr << 1);
	ret = hb_vin_i2c_write_reg16_data8(bus, dst_addr, SER_I2C_MAP_REG, val);
	if (ret < 0)
		vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			dst_addr, SER_I2C_MAP_REG, val);
	return ret;
}

int32_t sensor_i2c_addr_map(uint32_t bus, uint8_t serial_addr,
		uint8_t dst_addr, uint8_t src_addr)
{
	int32_t ret = RET_OK;
	uint8_t val = 0;

	if (dst_addr == src_addr)
		return ret;
	val = (uint8_t)(src_addr << 1);
	ret = vin_i2c_write_retry(bus, serial_addr, REG16_VAL8, SENSOR_I2C_SRC_REG, val);
	if (ret < 0) {
		vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			src_addr, SENSOR_I2C_SRC_REG, val);
		return ret;
	}
	val = (uint8_t)(dst_addr << 1);
	ret = vin_i2c_write_retry(bus, serial_addr, REG16_VAL8, SENSOR_I2C_DST_REG, val);
	if (ret < 0)
		vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
			serial_addr, SENSOR_I2C_DST_REG, val);
	return ret;
}

int32_t eeprom_i2c_addr_map(uint32_t bus, uint8_t serial_addr,
		uint8_t dst_addr, uint8_t src_addr)
{
	int32_t ret = RET_OK;
	uint8_t val = 0;

	if ((dst_addr == src_addr) || (src_addr == 0))
		return ret;
	val = (uint8_t)(src_addr << 1);
	ret = vin_i2c_write_retry(bus, serial_addr, REG16_VAL8, EEPROM_I2C_SRC_REG, val);
	if (ret < 0) {
		vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
				src_addr, EEPROM_I2C_SRC_REG, val);
		return ret;
	}
	val = (uint8_t)(dst_addr << 1);
	ret = vin_i2c_write_retry(bus, serial_addr, REG16_VAL8, EEPROM_I2C_DST_REG, val);
	if (ret < 0)
		vin_err("bus %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
				serial_addr, EEPROM_I2C_DST_REG, val);
	return ret;
}

int32_t serial_pipe_stream_id_config(uint32_t bus, uint8_t slave_addr,
		uint8_t stream_id)
{
	int32_t ret = RET_OK;
	int32_t val;

	val = vin_i2c_read_retry(bus, slave_addr, REG16_VAL8, SERIAL_PIPEZ_ID_REG);
	if (val < 0) {
		vin_err("read serial TX_STR_SEL reg fail\n");
		return -1;
	}
	val &= (~SER_STRAME_ID_MASK);
	val |= stream_id;
	ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8, SERIAL_PIPEZ_ID_REG, (uint8_t)val);
	if (ret < 0)
		vin_err("set serial stream_id fail\n");
	return ret;
}

static int32_t deserial_link_num(deserial_info_t *deserial_if)
{
	int32_t i, num = 0;

	for (i = 0; i < DES_LINK_NUM_MAX; i++) {
		if ((deserial_if->sensor_info[i] != NULL) ||
		    ((deserial_if->port_desp[i] != NULL) && deserial_if->port_desp[i][0] != '\0'))
			num ++;
	}

	return num;
}

int32_t max_serial_mfp_config(uint32_t bus, uint8_t slave_addr,
		uint8_t gpio_index, uint8_t gpio_mode, uint8_t gpio_id)
{
	int32_t ret = RET_OK;
	int32_t val;

	if (gpio_mode & GPIO_TX_GMSL) {
		val = vin_i2c_read_retry(bus, slave_addr, REG16_VAL8,
					 (uint16_t)(REG_ADDR_GPIO(gpio_index) + 1));
		val &= (~GPIO_ID_MASK);
		val |= gpio_id;
		vin_info("Serial TX bus:%d, des_link:%d, mfp:%d, ser_addr:0x%x, reg_addr:0x%04x, val:0x%02x\n",
			bus, gpio_id, gpio_index, slave_addr, (uint16_t)(REG_ADDR_GPIO(gpio_index) + 1), (uint8_t)val);
		ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8,
					  (uint16_t)(REG_ADDR_GPIO(gpio_index) + 1),
					  (uint8_t)val);
		if (ret < 0) {
			vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
				slave_addr,
				REG_ADDR_GPIO(gpio_index) + 1,
				val);
			return ret;
		}
	}
	if (gpio_mode & GPIO_RX_GMSL) {
		val = vin_i2c_read_retry(bus, slave_addr, REG16_VAL8,
					 (uint16_t)(REG_ADDR_GPIO(gpio_index) + 2));
		val &= (~GPIO_ID_MASK);
		val |= (gpio_id + GPIO_ID_OFFSET);
		vin_info("Serial RX bus:%d, mfp:%d, ser_addr:0x%x, reg_addr:0x%04x, val:0x%02x\n",
			bus, gpio_index, slave_addr, (uint16_t)(REG_ADDR_GPIO(gpio_index) + 2), (uint8_t)val);
		ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8,
					  (uint16_t)(REG_ADDR_GPIO(gpio_index) + 2),
					  (uint8_t)val);
		if (ret < 0) {
			vin_err("write %d@0x%x reg 0x%x val 0x%x fail\n", bus,
				slave_addr,
				(gpio_index * 3) + GPIO_BASE_REG + 2,
				val);
			return ret;
		}
	}

	val = GPIO_BASE_VAL;
	ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8,
				  REG_ADDR_GPIO(gpio_index), (uint8_t)val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
				slave_addr,
				GPIO_BASE_REG + (gpio_index * 3),
				val);
		return ret;
	}
	usleep(5);
	val = GPIO_BASE_VAL | gpio_mode;
	ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8,
				  REG_ADDR_GPIO(gpio_index), (uint8_t)val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x%x val 0x%x write fail\n", bus,
				slave_addr,
				GPIO_BASE_REG + (gpio_index * 3),
				val);
		return ret;
	}

	return ret;
}

int32_t serial_rclk_output(uint32_t bus, uint8_t slave_addr,
		uint8_t rclk_to_mfp, uint8_t mclk)
{
	int32_t ret = RET_OK;
	int32_t val;
	uint32_t setting_size = 0, *pdata = NULL;

	switch (mclk) {
		case 27:
			ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8, SER_PLL_SET_REG, SER_OUT27MHZ);
			break;
		case 24:
			ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8, SER_PLL_SET_REG, SER_OUT24MHZ);
			break;
		default:
			vin_err("Don't support mclk %dMhz\n", mclk);
			return ret;
	}
	if(ret < 0) {
		vin_err("set rclk to mclk = %d fail\n", mclk);
		return ret;
	}
	val = vin_i2c_read_retry(bus, slave_addr, REG16_VAL8, SER_PCLK_SET_REG);
	if(val < 0) {
		vin_err("read %d@0x%x reg 0x3f1 fail\n", bus, slave_addr);
		return ret;
	}
	val &= ~SER_PCLK_GPIO_MASK;
	val |= (rclk_to_mfp << 1);
	val |= SER_PCLKEN;
	ret = vin_i2c_write_retry(bus, slave_addr, REG16_VAL8,
				  SER_PCLK_SET_REG, (uint8_t)val);
	if (ret < 0) {
		vin_err("write %d@0x%x reg 0x3f1 val 0x%x fail\n", bus, slave_addr, val);
		return -RET_ERROR;
	}

	pdata = serial_rclk_output_enable;
	setting_size = sizeof(serial_rclk_output_enable) / sizeof(uint32_t) / 2;
	ret = vin_write_array(bus, slave_addr, REG16_VAL8, setting_size, pdata);
	if (ret < 0) {
		vin_err("serial rclk out config write enable fail!\n");
		return -RET_ERROR;
	}

	return ret;
}

static int32_t dual_patch_set(sensor_info_t *sensor_info, int32_t dual_patch_flag)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint8_t deserial_addr = deserial_if->deserial_addr & SHIFT_8BIT;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	uint32_t bus = deserial_if->bus_num;
	int32_t setting_size;

	ret = hb_vin_i2c_read_reg16_data8(bus, deserial_addr, MAX9296_RSTREG);
	if (ret < 0) {
		vin_err("get max9296 link mode fail\n");
		return -1;
	}
	if (ret & MAX9296_AUTOLINK)
		return 0;
	if (dual_patch_flag == 1) {
		setting_size = sizeof(max9296_dual_patch) / sizeof(uint32_t) / 2;
		ret = vin_write_array(bus, serial_addr, REG16_VAL8, setting_size, max9296_dual_patch);
		if (ret < 0)
			vin_err("max9296 patch config fail!!!\n");
	}
	return ret;
}

static int32_t check_dual_patch_is_set(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	uint8_t another_ser_addr;
	uint32_t bus = sensor_info->bus_num;
	int32_t dual_patch_flag = 0;

	if (sensor_info->deserial_port == 1)
		another_ser_addr = serial_addr - 1;
	else
		another_ser_addr = serial_addr + 1;
	ret = hb_vin_i2c_read_reg16_data8(bus, another_ser_addr, DUAL_PATCH_CHECK_REG);
	if ((ret < 0) || (ret & DUAL_PATCH_CHECK_VAL) == 0)
		dual_patch_flag = 1;

	return dual_patch_flag;
}

static int32_t dual_serial_i2c_addr_map(sensor_info_t *sensor_info, maxdes_ops_t *maxops)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint8_t serial_addr, sensor_addr, eeprom_addr;
	uint8_t poc_addr = deserial_if->poc_addr & SHIFT_8BIT;
	uint8_t serial_addrh, sensor_addrh, eeprom_addrh;
	uint32_t bus = deserial_if->bus_num;
	int32_t dual_patch_flag = 0;
	uint32_t enable_mask, check_id;

	vin_info("Seriai i2c map begain\n");
	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}
	serial_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->serial_addr;
	sensor_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->sensor_addr;
	eeprom_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->eeprom_addr;
	serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	eeprom_addr = sensor_info->eeprom_addr & SHIFT_8BIT;
	/* map serial i2c addr */
	ret = hb_vin_i2c_lock(bus);
	if (ret < 0)
		return -1;
	dual_patch_flag = check_dual_patch_is_set(sensor_info);

	// keep enable if other link is init
	enable_mask = 1 << sensor_info->deserial_port;
	check_id = (enable_mask == 0x1 ? 1 : 0);
	ret = vin_deserial_state_check(deserial_if, check_id);
	if (ret > 0)
		enable_mask |= 0x3;
	else if (ret < 0)
		vin_err("deserial max9296 check state fail!\n");
	// printf("enable_mask: 0x%x --> 0x%x\n", 1 << sensor_info->deserial_port, enable_mask);
	ret = maxops->link_enable(deserial_if, (uint8_t)enable_mask);
	if (ret < 0) {
		vin_err("%s link %d enable fail!\n", deserial_if->deserial_name,
				sensor_info->deserial_port);
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	if (poc_addr == INVALID_POC_ADDR) {
		hb_vin_i2c_write_reg16_data8(bus, serial_addr, SERIAL_RESET_REG, SERIAL_RESET_VALUE);
		usleep(50*1000);
	}
	ret = serial_i2c_addr_map(bus, serial_addrh, serial_addr);
	if (ret < 0) {
		vin_err("serial i2c addr map fail!\n");
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	/* Map sensor addr */
	ret  = sensor_i2c_addr_map(bus, serial_addr, sensor_addrh, sensor_addr);
	if (ret < 0) {
		vin_err("sensor i2c addr map fail!\n");
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	/* Map eeprom addr */
	ret  = eeprom_i2c_addr_map(bus, serial_addr, eeprom_addrh, eeprom_addr);
	if (ret < 0) {
		vin_err("eeprom i2c addr map fail!\n");
		hb_vin_i2c_unlock(bus);
	}
	/* max9296 reverse splitter mode patch config */
	if (strcmp(deserial_if->deserial_name, "max9296") == 0) {
		ret = dual_patch_set(sensor_info, dual_patch_flag);
		if (ret < 0) {
			vin_err("serial dual patch config fail!\n");
			hb_vin_i2c_unlock(bus);
			return ret;
		}
	}
	ret = maxops->link_enable(deserial_if, MAX9296_LINKALL);
	if (ret < 0) {
		vin_err("%s link %d enable fail!\n", deserial_if->deserial_name,
			sensor_info->deserial_port);
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	ret = vin_deserial_state_confirm(deserial_if, sensor_info->deserial_port);
	if (ret < 0)
		vin_err("deserial max9296 confirm state fail\n");
	hb_vin_i2c_unlock(bus);
	return ret;
}


static int32_t quad_serial_i2c_addr_map(sensor_info_t *sensor_info, maxdes_ops_t *maxops)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint8_t serial_addr, sensor_addr, eeprom_addr;
	uint8_t poc_addr = deserial_if->poc_addr & SHIFT_8BIT;
	uint8_t serial_addrh, sensor_addrh, eeprom_addrh;
	uint32_t bus = deserial_if->bus_num;
	uint32_t enable_mask;
	int32_t i;

	vin_info("Seriai i2c map begain\n");
	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}

	serial_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->serial_addr;
	sensor_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->sensor_addr;
	eeprom_addrh = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->eeprom_addr;
	serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	sensor_addr = sensor_info->sensor_addr & SHIFT_8BIT;
	eeprom_addr = sensor_info->eeprom_addr & SHIFT_8BIT;
	/* Link enable */
	ret = maxops->link_enable(deserial_if, (uint8_t)(1 << sensor_info->deserial_port));
	if (ret < 0) {
		vin_err("%s link %d enable fail!\n", deserial_if->deserial_name,
				sensor_info->deserial_port);
		return ret;
	}
	if (poc_addr == INVALID_POC_ADDR) {
		hb_vin_i2c_write_reg16_data8(bus, serial_addr, SERIAL_RESET_REG, SERIAL_RESET_VALUE);
		usleep(50*1000);
	}
	ret = hb_vin_i2c_lock(bus);
	if (ret < 0)
		return -1;
	// keep enable if other link is init
	enable_mask = REMTCH_MASK(sensor_info->deserial_port);
	for (i = 0; i < 4; i++) {
		ret = vin_deserial_state_check(deserial_if, i);
		if (ret > 0)
			enable_mask |= REMTCH_MASK(i);
		else if (ret < 0)
			vin_err("deserial max9296 check state fail!\n");
	}
	// printf("enable_mask : 0x%x --> 0x%x\n", REMTCH_MASK(sensor_info->deserial_port), enable_mask);
	if (maxops->remote_control != NULL) {
		ret = maxops->remote_control(deserial_if, enable_mask);
		if (ret < 0) {
			vin_err("%s remount control channel set fail\n",
					deserial_if->deserial_name);
			hb_vin_i2c_unlock(bus);
			return ret;
		}
	}
	ret = serial_i2c_addr_map(bus, serial_addrh, serial_addr);
	if (ret < 0) {
		vin_err("serial i2c addr map fail!\n");
		hb_vin_i2c_unlock(bus);
		return ret;
	}
	if (maxops->remote_control != NULL) {
		ret = maxops->remote_control(deserial_if, REMTCH_MASK_ALL);
		if (ret < 0) {
			vin_err("%s remount control channel set fail\n",
					deserial_if->deserial_name);
			hb_vin_i2c_unlock(bus);
			return ret;
		}
	}
	hb_vin_i2c_unlock(bus);
	/* Map sensor addr */
	ret  = sensor_i2c_addr_map(bus, serial_addr, sensor_addrh, sensor_addr);
	if (ret < 0) {
		vin_err("sensor i2c addr map fail!\n");
		return ret;
	}
	/* Map eeprom addr */
	ret  = eeprom_i2c_addr_map(bus, serial_addr, eeprom_addrh, eeprom_addr);
	if (ret < 0) {
		vin_err("eeprom i2c addr map fail!\n");
		return ret;
	}
	ret = vin_deserial_state_confirm(deserial_if, sensor_info->deserial_port);
	if (ret < 0)
		vin_err("deserial max9296 confirm state fail\n");
	return ret;
}

static int32_t i2c_addr_map(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint32_t bus = deserial_if->bus_num;
	maxdes_ops_t *maxops;

	vin_info("Seriai i2c map begain\n");
	if (deserial_if == NULL) {
		vin_err("deserial info is null!\n");
		return -1;
	}
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(((deserial_module_t *)(deserial_if->deserial_ops))->ops.max);
#endif

	if (maxops == NULL) {
		vin_info("%s ops is null\n", deserial_if->deserial_name);
		return -1;
	}

	/* map serial i2c addr */
	if (!strcmp(deserial_if->deserial_name, "max9296") ||
	    !strcmp(deserial_if->deserial_name, "max96718")) {
		ret = dual_serial_i2c_addr_map(sensor_info, maxops);
	} else if (!strcmp(deserial_if->deserial_name, "max96712") ||
		   !strcmp(deserial_if->deserial_name, "max96722")) {
		ret = quad_serial_i2c_addr_map(sensor_info, maxops);
	} else {
		vin_err("don't support deserial %s\n", deserial_if->deserial_name);
		return -1;
	}
	if (ret < 0)
		vin_err("serial_addr map fail\n");

	return ret;
}

static int32_t serial_rclk_output_config(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	uint8_t serial_rclk_out = 0;
	uint8_t serial_addr = 0;
	uint32_t rclk_mfp = 0;
	uint32_t mclk = 0;

	serial_rclk_out = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->serial_rclk_out;
	rclk_mfp = ((emode_data_t *)SENSOR_EMODE_DATA(sensor_info))->rclk_mfp;
	serial_addr = sensor_info->serial_addr;

	if (serial_rclk_out) {
		switch (rclk_mfp) {
			case 4:
				ret = vin_i2c_write_retry(sensor_info->bus_num, serial_addr,
						REG16_VAL8, PIO_SLEW_1, MFP4_MASK_PIO);
				break;
			default:
				vin_info("serial rclk out mfp: %d", rclk_mfp);
		}
		if (ret < 0) {
			vin_err("line: %d write %x mfp reg fail\n", __LINE__, serial_addr);
			return -RET_ERROR;
		}
		mclk = vin_sensor_emode_parse(sensor_info, 'M');
		if (mclk < 0) {
			vin_err("sensor embode serial rclk parse fail!!!\n");
			return -RET_ERROR;
		}
		vin_info("port:%d serial mfp%d out %dM rclk config\n", sensor_info->port, rclk_mfp, mclk);
		ret = serial_rclk_output(sensor_info->bus_num, serial_addr, rclk_mfp, mclk);
		if (ret < 0) {
			vin_err("serial rclk out config write mclk setting fail!\n");
			return -RET_ERROR;
		}
	}

	return 0;
}

int32_t max_serial_init(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;
	uint32_t poc_addr, poc_map;
	uint8_t poc_mask;
	uint32_t bus;
	int32_t rst_mfp, intf_type;
	int32_t setting_size;

	if (deserial_if == NULL) {
		vin_err("no deserial here error\n");
		return -1;
	}
	bus = deserial_if->bus_num;;

	vin_info("serial_init begain\n");
	/* POC reset */
	if (deserial_if->poc_addr != INVALID_POC_ADDR) {
		poc_addr = deserial_if->poc_addr ? deserial_if->poc_addr : POC_ADDR_DEFT;
		poc_map = deserial_if->poc_map ? deserial_if->poc_map : POC_MAP_DEFT;
		poc_mask = POC_MASK(poc_map, sensor_info->deserial_port);
		vin_info("poc_map = 0x%x, poc_mask = 0x%x\n", poc_map, poc_mask);
		ret = poc_reset(sensor_info->bus_num, (uint8_t)poc_addr, poc_mask);
		if (ret < 0) {
			vin_err("sensor %s poc reset fail\n",  sensor_info->sensor_name);
			return ret;
		}
		vin_info("poc reset done\n");
	} else {
		vin_info("No action required for poc\n");
	}

	ret = i2c_addr_map(sensor_info);
	if (ret < 0) {
		vin_err("i2c addr map fail!\n");
		return ret;
	}

	/* Serial pipeline config */
	ret = serial_pipeline_init(sensor_info);
	if (ret < 0) {
		vin_err("serial pipeline init fail!\n");
		return ret;
	}
	intf_type = vin_sensor_emode_parse(sensor_info, 'I');
	if (intf_type < 0) {
		if (intf_type == -FLAG_NOT_FIND) {
			vin_dbg("the sensor %s is not the type that find\n", sensor_info->sensor_name);
		}
	} else if (intf_type == 1) {
		vin_dbg("the sensor %s is dvp sensor\n", sensor_info->sensor_name);
		ret = dvp_sensor_serial_init(sensor_info);
		if (ret < 0) {
			vin_err("serial pipeline init fail!\n");
			return ret;
		}
	}

	if (!strcmp(deserial_if->deserial_name, "max9296") ||
		!strcmp(deserial_if->deserial_name, "max96718")) {
		intf_type = (deserial_link_num(deserial_if) <= 1) ? 1 : sensor_info->deserial_port + 1;
		ret = serial_pipe_stream_id_config(bus, serial_addr, (uint8_t)intf_type);
		if (ret < 0) {
			vin_err("serial_pipe_stream_id_config fail!\n");
			return ret;
		}
	}

	ret = serial_rclk_output_config(sensor_info);
	if (ret < 0) {
		vin_err("serial_rclk_output_config fail !\n");
		return ret;
	}

	/* Sensor reset */
	rst_mfp = vin_sensor_emode_parse(sensor_info, 'R');
	if (rst_mfp < 0) {
		if (rst_mfp == -FLAG_NOT_FIND) {
			vin_info("the sensor %s is not reset pin\n", sensor_info->sensor_name);
			return RET_OK;
		}
		vin_err("sensor_mode_parse rst mfp fail!!!\n");
		return -1;
	}
	vin_info("rst_mfp is %d\n", rst_mfp);
	ret = max_serial_mfp_config(bus, serial_addr,
				(uint8_t)rst_mfp, GPIO_OUT_HIGH, 0);
	if (ret < 0) {
		vin_err("serial mfp config error, sensor reset fail!\n");
		return ret;
	}
	usleep(100 *1000);
	return ret;
}

int32_t max_serial_errb_mfp_map(sensor_info_t *sensor_info)
{
	int32_t ret = -1;
	int32_t ser_errb_mfp = 0;
	uint8_t des_link = 0;
	maxdes_ops_t *maxops = NULL;
	deserial_info_t *deserial_if = sensor_info->deserial_info;
	uint8_t serial_addr = sensor_info->serial_addr & SHIFT_8BIT;

	if (deserial_if == NULL) {
		vin_err("deserial_if is NULL\n");
		return -1;
	}
#ifdef CAM_DIAG
	if (sensor_info->diag_mask_disabled & BIT(SNR_ERRB_CHECK))
		return 0;
#endif
#ifdef CAMERA_FRAMEWORK_HBN
	maxops = DESERIAL_MAXOPS(deserial_if);
#else
	maxops = &(((deserial_module_t *)(deserial_if->deserial_ops))->ops.max);
#endif
	ser_errb_mfp = vin_sensor_emode_parse(sensor_info, 'E');
	if (ser_errb_mfp < 0) {
		vin_err("sensor_mode_parse sensor_errb pin fail ret = %d\n", ser_errb_mfp);
		return ser_errb_mfp;
	}
	des_link = sensor_info->deserial_port;
	vin_info("errb map port:%d, name:%s, des_link:%d, serial_errb_mfp:%d\n",
		sensor_info->port, sensor_info->sensor_name, des_link, ser_errb_mfp);
	ret = maxops->mfp_cfg(deserial_if, GPIO_RX_GMSL, ser_errb_mfp, des_link);
	if (ret < 0) {
		vin_err("%s camerr_pin config fail\n", deserial_if->deserial_name);
		return ret;
	}
	ret = max_serial_mfp_config(sensor_info->bus_num, serial_addr,
				ser_errb_mfp, GPIO_TX_GMSL, des_link);
	if (ret < 0) {
		vin_err("serial mfp config fail\n");
		return ret;
	}

	return ret;
}
