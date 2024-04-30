/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_common.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_COMMON_H__
#define __CAMERA_MOD_COMMON_H__

#include <stdint.h>

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_BUS		(0)
#define SPI_BUS		(1)

#define I2C_ADDR_INVALID (0xFF)

#define CSI_ATTR_PHY_DPHY	(0)
#define CSI_ATTR_PHY_CPHY	(1)
#define CSI_ATTR_VC_MAX		(4)

/**
 * @struct common_module_s
 * common module struct with camera module base
 * @NO{S10E02C07}
 */
typedef struct common_module_s {
	CAMERA_MODULE_HEADER;
	void *module_fucs;
} common_module_t;

/**
 * @struct csi_attr_s
 * mipi csi attr info struct
 * @NO{S10E02C07}
 */
typedef struct csi_attr_s {
	uint16_t phy;
	uint16_t lane;
	uint16_t datatype[CSI_ATTR_VC_MAX];
	uint16_t fps;
	uint16_t mipiclk;
	uint16_t width;
	uint16_t height;
	uint16_t settle;
} csi_attr_t;

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_COMMON_H__ */


