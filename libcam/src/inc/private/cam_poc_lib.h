/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_poc_lib.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAM_POC_LIB_H__
#define __CAM_POC_LIB_H__

#include <stdint.h>

#include "camera_mod_poc.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_POC_INSIDE_NAME		"inside"

#define CAM_POC_CHECK_ADDR_EXCEPT	(0xFF)
#define CAM_POC_CHECK_ADDR_MIN		(0x00)
#define CAM_POC_CHECK_ADDR_MAX		(0x7F)
#define CAM_POC_POWER_DELAY_MIN		(0)
#define CAM_POC_POWER_DELAY_MAX		(10000)
#define CAM_POC_MAP_MASK_MIN		(0x0000)
#define CAM_POC_MAP_MASK_MAX		(0x3333)
#define CAM_POC_MAP_MASK_VAL		(0x3333)

#define CAM_POC_MAP_DEFAULT		(0x3210)

/* internal apis */
extern int32_t camera_poc_config_check(camera_module_lib_t *lib, poc_config_t *poc_config);
extern int32_t camera_poc_ops_bind(deserial_handle_st *hdes, poc_info_t *poc_if);
extern int32_t camera_poc_config_parse(deserial_handle_st *hdes, poc_info_t *poc_if);

extern int32_t camera_poc_init(poc_info_t *poc_if);
extern int32_t camera_poc_deinit(poc_info_t *poc_if);
extern int32_t camera_poc_on(poc_info_t *poc_if, int32_t link);
extern int32_t camera_poc_off(poc_info_t *poc_if, int32_t link);
extern int32_t camera_poc_get_version(poc_info_t *poc_if, char *name, char *version);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_POC_LIB_H__ */

