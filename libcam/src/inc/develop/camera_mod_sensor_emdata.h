/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_sensor_emdata.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_SENSOR_EMDATA_H__
#define __CAMERA_MOD_SENSOR_EMDATA_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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


#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_SENSOR_EMDATA_H__ */

