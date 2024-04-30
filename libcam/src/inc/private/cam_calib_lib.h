/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_calib_lib.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAM_CALIB_LIB_H__
#define __CAM_CALIB_LIB_H__

#include <stdint.h>

#include "camera_mod_calib.h"

#include "cam_handle.h"

#ifdef __cplusplus
extern "C" {
#endif

/* internal apis */
extern int32_t camera_calib_config_check(camera_module_lib_t *lib, const char *version);
extern int32_t camera_calib_ops_bind(camera_handle_st *hcam, calib_info_t *cal_if);
extern int32_t camera_calib_config_parse(camera_handle_st *hcam, calib_info_t *cal_if);

extern int32_t camera_calib_init(calib_info_t *cal_if);
extern int32_t camera_calib_deinit(calib_info_t *cal_if);
extern int32_t camera_calib_get_version(calib_info_t *cal_if, char *name, char *version);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_CALIB_LIB_H__ */


