/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_deserial_lib.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAM_DESERIAL_LIB_H__
#define __CAM_DESERIAL_LIB_H__

#include <stdint.h>

#include "hb_camera_data_config.h"

#include "camera_mod_deserial.h"
#include "camera_mod_poc.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_DESERIAL_CHECK_ADDR_EXCEPT		(0xFF)
#define CAM_DESERIAL_CHECK_ADDR_MIN		(0x00)
#define CAM_DESERIAL_CHECK_ADDR_MAX		(0x7F)
#define CAM_DESERIAL_CHECK_RESET_DELAY_MIN	(0)
#define CAM_DESERIAL_CHECK_RESET_DELAY_MAX	(10000)
#define CAM_DESERIAL_CHECK_LANE_SPEED_MIN	(0)
#define CAM_DESERIAL_CHECK_LANE_SPEED_MAX	(2500)

#define CAM_DESERIAL_RESET_DELAY_DEFAULT	(20)
#define CAM_DESERIAL_LINK_MAP_ARRAY_DEFAULT	{ /* link enable mask */ \
	0x0000U, 0x0000U, 0x0000U, 0x0010U,	/* 0000, 0001, 0010, 0011, */ \
	0x0000U, 0x0100U, 0x0100U, 0x0210U,	/* 0100, 0101, 0110, 0111, */ \
	0x0000U, 0x1000U, 0x1000U, 0x2010U,	/* 1000, 1001, 1010, 1011, */ \
	0x1000U, 0x2100U, 0x2100U, 0x3210U,	/* 1100, 1101, 1110, 1111, */ \
}
#define CAM_DESERIAL_CSI_LANE_DEFAULT		(4)
#define CAM_DESERIAL_CSI_SPEED_1V_DEFAULT	(1200)
#define CAM_DESERIAL_CSI_SPEED_nV_DEFAULT	(2000)
#define CAM_DESERIAL_CSI_SPEED_hV_DEFAULT	(2500)
#define CAM_DESERIAL_CSI_DATATYPE_DEFAULT	(0x2C)

/* internal apis */
extern int32_t camera_deserial_config_check(camera_module_lib_t *lib, deserial_config_t *des_config);
extern int32_t camera_deserial_config_has_poc(deserial_handle_st *hdes);
extern int32_t camera_deserial_ops_bind(deserial_handle_st *hdes, deserial_info_t *des_if, poc_info_t *poc_if);
extern int32_t camera_deserial_config_parse(deserial_handle_st *hdes, deserial_info_t *des_if);
extern int32_t camera_deserial_csi_attr_parse(deserial_handle_st *hdes, deserial_info_t *des_if,
				mipi_config_t *mipi_to, mipi_bypass_t *bypass_to);

extern int32_t camera_deserial_init(deserial_info_t *des_if);
extern int32_t camera_deserial_deinit(deserial_info_t *des_if);
extern int32_t camera_deserial_stream_on(deserial_info_t *des_if, int32_t link);
extern int32_t camera_deserial_stream_off(deserial_info_t *des_if, int32_t link);
extern int32_t camera_deserial_get_csi_attr(deserial_info_t *des_if, csi_attr_t *csi_attr);
extern int32_t camera_deserial_get_version(deserial_info_t *des_if, char *name, char *version);
extern int32_t camera_deserial_dump(deserial_info_t *des_if, int32_t link_mask);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_DESERIAL_LIB_H__ */


