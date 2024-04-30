/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_txser_data.h
 *
 * @NO{S10E02C06}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_TXSER_DATA_H__
#define __CAMERA_MOD_TXSER_DATA_H__

#include <stdint.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TXS_LINK_MAP_CSI_VALID(v, c)	(((((v) >> 2) & 0x3) == (c)) || ((((v) >> 6) & 0x3) == (c)) || \
					 ((((v) >> 10) & 0x3) == (c)) || ((((v) >> 14) & 0x3) == (c)))

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
#define GPIO_NUMBER 6
#endif

/**
 * @struct txser_info_s
 * txser module data info struct
 * @NO{S10E02C05}
 */
typedef struct txser_info_s {
	uint32_t index;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t txser_addr;
	uint32_t bus_timeout;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t gpio_num;
	int32_t gpio_pin[GPIO_NUMBER];
	int32_t gpio_level[GPIO_NUMBER];
#endif
	int32_t gpio_enable;
	int32_t gpio_levels;
	uint32_t lane_mode;
	uint32_t reset_delay;
	uint32_t txser_attr;
	char *txser_name;
	void *txser_ops;
	uint32_t reserved[16];
} txser_info_t;

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_TXSER_DATA_H__ */


