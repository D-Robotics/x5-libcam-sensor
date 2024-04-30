/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SERIAL_MAX_SERIAL_H_
#define UTILITY_SERIAL_MAX_SERIAL_H_

#include "./hb_cam_utility.h"

#define POC_ADDR_DEFT			0x28
#define POC_REG_ADDR			0x01
#define POC_RESET_ALL			0x1F
#define POC_MAP_DEFT			0x3210    // Link A~D POC chanel, Bit[0~3]:A, [4~7]:B, [8~11]:C, [12~15]:D
#define POC_MASK(m, i)			((uint8_t)(1 << (((m) >> ((i) * 4)) & 0xf)))    // m:poc_map, i:Link_index

// deserial remote channel mask
#define REMTCH_MASK(i)     ((uint8_t)(1 << ((i) * 2)))  // max96712 reg 0x03: bit 0/2/4/6 -- link A/B/C/D
#define REMTCH_MASK_ALL    ((uint8_t)0x55)

#define MAX9296_LINKALL		0x23

#define MAXIM_GMSL_3G			0x1
#define MAXIM_GMSL_6G			0x2

#define SER_I2C_MAP_REG		((uint16_t)0x00)
#define EEPROM_I2C_SRC_REG	((uint16_t)0x42)
#define EEPROM_I2C_DST_REG	((uint16_t)0x43)
#define SENSOR_I2C_SRC_REG	((uint16_t)0x44)
#define SENSOR_I2C_DST_REG	((uint16_t)0x45)
#define SERIAL_PIPEZ_ID_REG	((uint16_t)0x5B)        // Pipe z stream_id reg
#define SER_STRAME_ID_MASK	0x03        // Bit[1:0] set stream_id

#define MAX9295_PIPEY_ID_REG	((uint16_t)0x57)

#define SER_PLL_SET_REG		((uint16_t)0x3f0)
#define SER_PCLK_SET_REG	((uint16_t)0x3f1)
#define SER_OUT27MHZ		((uint8_t)0x51)
#define SER_OUT24MHZ		((uint8_t)0x59)
#define SER_PCLK_GPIO_MASK	0x3e        // Bit[5:1] gpio num
#define SER_PCLKEN		BIT(0)      // Bit[0] 1: enable PCLK, 0: disable PCLK
#define PIO_SLEW_1			((uint16_t)0x570)
#define MFP4_MASK_PIO		0x0c	 // disable PIO to mfp4

#define GPIO_BASE_REG		((uint16_t)0x2be)
#define REG_ADDR_GPIO(i)	((uint16_t)(GPIO_BASE_REG + ((i) * 3)))
#define GPIO_BASE_VAL		0x80
#define GPIO_OUT_DIS		((uint8_t)BIT(0))
#define GPIO_TX_GMSL		((uint8_t)BIT(1) | GPIO_OUT_DIS)
#define GPIO_RX_GMSL		((uint8_t)BIT(2))
#define GPIO_OUT_HIGH		((uint8_t)BIT(4))
#define GPIO_ID_MASK		0x1f
#define GPIO_ID_OFFSET		4

#define SHIFT_8BIT				(0xff)
#define SERIAL_RESET_REG        (0x0010)
#define SERIAL_RESET_VALUE      (0xf1)

#ifdef CAMERA_FRAMEWORK_HBN
#include "camera_mod_sensor_emdata.h"
#else
typedef struct emode_data_s {
	uint8_t serial_addr;
	uint8_t sensor_addr;
	uint8_t eeprom_addr;
	uint8_t serial_rclk_out;
	uint32_t rclk_mfp;
#ifdef SENSOR_CUSTOM_EMODE_DATA
	SENSOR_CUSTOM_EMODE_DATA;
#endif
} emode_data_t;
#endif

/*
 * poc_reset() - Set poc reg to contrl sensor power Down and Up
 * @bus: i2c bus num
 * @slave_addr: i2c device addr of poc
 * @reset_mask: the bit mask need to set
 */
int32_t poc_reset(uint32_t bus, uint8_t slave_addr, uint8_t reset_mask);

/****************************** SERIAL Config API *******************************/
/*
 * serial_i2c_addr_map() - map serializer to new addr that is src_addr.
 * @dst_addr: Hardware address of the serializer
 * @src_addr: New addr
 */
int32_t serial_i2c_addr_map(uint32_t bus, uint8_t dst_addr, uint8_t src_addr);
/*
 * sensor_i2c_addr_map() - map sensor to new addr that is src_addr.
 * @dst_addr: Hardware address of the sensor
 * @src_addr: New addr
 */
int32_t sensor_i2c_addr_map(uint32_t bus, uint8_t serial_addr,
		uint8_t dst_addr, uint8_t src_addr);

/*
 * eeprom_i2c_addr_map() - map eeprom to new addr thar is src_addr.
 * @dst_addr: Hardware address of the eeprom
 * @src_addr: New addr
 */
int32_t eeprom_i2c_addr_map(uint32_t bus, uint8_t serial_addr,
		uint8_t dst_addr, uint8_t src_addr);

/*
 * max_serial_mfp_config() - set serial gpio
 * @gpio_num: the gpio num to need be configured
 * @gpio_mode: bit[0]-output en, bit[1]-tx to gmsl en, bit[2]-rx from gmsl en, bit[4] output value
 * @gpio_id: if gpio_mode bit[1] or bit[2] is height level, need set gpio_id
 */
int32_t max_serial_mfp_config(uint32_t bus, uint8_t slave_addr,
		uint8_t gpio_num, uint8_t gpio_mode, uint8_t gpio_id);

/*
 * serial_rclk_outpu() - set serial rclk
 * @rclk_to_mfp: the rclk output pin num
 * @mclk: the rclk output frequency
 */
int32_t serial_rclk_output(uint32_t bus, uint8_t slave_addr,
		uint8_t rclk_to_mfp, uint8_t mclk);

/*
 * serial_pipe_stream_id_config() - set serial pipe stream_id
 * @stream_id: stream_id of serial pipe
 */
int32_t serial_pipe_stream_id_config(uint32_t bus, uint8_t slave_addr, uint8_t stream_id);

int32_t max_serial_init(sensor_info_t *sensor_info);
int32_t max_serial_errb_mfp_map(sensor_info_t *sensor_info);

#if 0
uint8_t serial_sensor_i2c_addr_map[] = {
		0x04, 0x80, 0x00, 0x00, 0x82,  // Set Ser new Address
		0x04, 0x82, 0x00, 0x42, 0xA2,  // Set eeprom new Address
		0x04, 0x82, 0x00, 0x43, 0xA0,  // I2C eeprom hardware Address
		0x04, 0x82, 0x00, 0x44, 0x22,  // Set sensor new Address
		0x04, 0x82, 0x00, 0x45, 0x6c,  // I2C sensor hardware Address
};
#endif

#endif    // UTILITY_SERIAL_MAX_SERIAL_H_
