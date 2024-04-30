/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_reg.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_REG_H__
#define __CAMERA_REG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_I2C_RETRY_MAX	10

#define DELAY_FLAG              (0xFFFF)

#define REG_WIDTH_16bit 16
#define REG_WIDTH_8bit 8
// #define I2C_BUS  0
// #define SPI_BUS  1

#define REG16_VAL16 3
#define REG16_VAL8  2
#define REG8_VAL8   1

typedef struct _x2_camera_i2c_t {
	uint32_t i2c_addr;
	uint32_t reg_size;
	uint32_t reg;
	uint32_t data;
} x2_camera_i2c_t;

extern int32_t camera_reg_i2c_read8(int32_t bus, int32_t reg_width, int32_t sensor_addr, uint32_t reg_addr);
extern int32_t camera_reg_i2c_read16(int32_t bus, int32_t reg_width, int32_t sensor_addr, uint32_t reg_addr);
extern int32_t camera_reg_i2c_read_block(int32_t bus, int32_t reg_width, int32_t device_addr,  uint32_t reg_addr, char *buffer, uint32_t size);
extern int32_t camera_reg_i2c_write8(int32_t bus, int32_t reg_width, int32_t sensor_addr, uint32_t reg_addr, uint8_t value);
extern int32_t camera_reg_i2c_write16(int32_t bus, int32_t reg_width, int32_t sensor_addr, uint32_t reg_addr, uint16_t value);
extern int32_t camera_reg_i2c_write_block(int32_t bus, int32_t reg_width, int32_t device_addr,  uint32_t reg_addr, char *buffer, uint32_t size);

extern int32_t camera_reg_i2c_write8_s(uint32_t bus, uint32_t i2c_addr, uint32_t reg_addr,
			uint32_t reg_width, int32_t bit_mask, uint32_t value);
extern int32_t camera_reg_i2c_bit_write8(uint32_t bus, uint32_t i2c_addr, uint32_t reg_addr,
			uint32_t reg_width, int32_t bit_mask, uint32_t value);
extern int32_t camera_reg_i2c_bit_array_write8(uint32_t bus, uint32_t i2c_addr,  uint32_t reg_width,
			int32_t setting_size, uint32_t *cam_setting);
extern int32_t camera_reg_i2c_write_array(uint32_t bus, uint32_t i2c_addr, int32_t reg_width, int32_t setting_size, uint32_t *cam_setting);

extern int32_t camera_reg_i2c_read_retry(uint32_t bus, uint8_t i2c_addr,
			int32_t reg_width, uint16_t reg_addr);
extern int32_t camera_reg_i2c_write_retry(uint32_t bus, uint8_t i2c_addr,
			int32_t reg_width, uint16_t reg_addr, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_REG_H__ */
