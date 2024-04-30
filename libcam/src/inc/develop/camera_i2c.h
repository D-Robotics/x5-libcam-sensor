/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_env.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_I2C_H__
#define __CAMERA_I2C_H__

#include <stdint.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAMERA_I2C_BUS_MAX
 * i2c bus index max supported
 */
#ifdef CAM_CONFIG_I2C_BUS_MAX
#define CAMERA_I2C_BUS_MAX		CAM_CONFIG_I2C_BUS_MAX
#else
#define CAMERA_I2C_BUS_MAX		(10)
#endif

/**
 * @def CAMERA_I2C_BUS_DEV_PATH
 * i2c bus index max supported
 */
#ifdef CAM_CONFIG_I2C_BUS_DEV_PATH
#define CAMERA_I2C_BUS_DEV_PATH		CAM_CONFIG_I2C_BUS_DEV_PATH
#else
#define CAMERA_I2C_BUS_DEV_PATH		"/dev/i2c-%d"
#endif

/**
 * @def CAMERA_I2C_DEV_PATH_LENGTH
 * i2c bus device path buffer length
 */
#define CAMERA_I2C_DEV_PATH_LENGTH	(12)

/* internal apis */
extern int32_t camera_i2c_init(uint32_t bus);
extern int32_t camera_i2c_deinit(uint32_t bus);

extern int32_t camera_i2c_lock(uint32_t bus);
extern int32_t camera_i2c_unlock(uint32_t bus);

extern int32_t camera_i2c_timeout_set(uint32_t bus, uint32_t timeout_ms);

extern int32_t camera_i2c_read_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
extern int32_t camera_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
extern int32_t camera_i2c_read_reg8_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
extern int32_t camera_i2c_read_reg8_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
extern int32_t camera_i2c_write_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t value);
extern int32_t camera_i2c_write_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t value);
extern int32_t camera_i2c_write_reg8_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t value);
extern int32_t camera_i2c_write_reg8_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t value);
extern int32_t camera_i2c_write_block(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint32_t value, uint8_t cnt);

extern int32_t camera_i2c_read_block_reg16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr,
			unsigned char *buf, uint32_t count);
extern int32_t camera_i2c_read_block_reg8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr,
			unsigned char *buf, uint32_t count);
extern int32_t camera_i2c_write_block_reg16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr,
			const char *buf, uint32_t count);
extern int32_t camera_i2c_write_block_reg8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr,
			unsigned char *buf, uint32_t count);

extern int32_t camera_i2c_write(int32_t bus, uint8_t i2c_addr, const uint8_t *reg_addr,
			int32_t reg_addr_len, const uint8_t *buf, uint8_t buf_len);
extern int32_t camera_i2c_read(int32_t bus, uint8_t i2c_addr, const uint8_t *reg_addr,
			int32_t reg_addr_len, uint8_t *buf, uint8_t buf_len);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_I2C_H__ */
