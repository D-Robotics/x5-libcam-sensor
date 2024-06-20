/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_sensor_lib.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAM_SENSOR_LIB_H__
#define __CAM_SENSOR_LIB_H__

#include <stdint.h>

#include "hb_camera_data_config.h"
#include "hb_camera_data_info.h"

#include "camera_mod_deserial.h"
#include "camera_mod_sensor.h"
#include "camera_mod_calib.h"

#include "cam_handle.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_SENSOR_CHECK_ADDR_EXCEPT	(0xFF)
#define CAM_SENSOR_CHECK_ADDR_MIN	(0x00)
#define CAM_SENSOR_CHECK_ADDR_MAX	(0x7F)
#define CAM_SENSOR_CHECK_SMODE_MIN	(1)
#define CAM_SENSOR_CHECK_SMODE_MAX	(6)
#define CAM_SENSOR_CHECK_FPS_MIN	(0)
#define CAM_SENSOR_CHECK_FPS_MAX	(120)
#define CAM_SENSOR_CHECK_WIDTH_MIN	(0)
#define CAM_SENSOR_CHECK_WIDTH_MAX	(8192)
#define CAM_SENSOR_CHECK_HEIGHT_MIN	(0)
#define CAM_SENSOR_CHECK_HEIGHT_MAX	(4096)

#define CAM_SENSOR_CALIB_STRING_INVALID	"NULL"
#define CAM_SENSOR_CALIB_LNAME_DISABLE	"disable"

/* internal apis */
extern int32_t camera_sensor_config_check(camera_module_lib_t *lib, camera_config_t *cam_config);
extern int32_t camera_sensor_get_link_desp(camera_handle_st *hcam, char *desp, int32_t size);
extern const char* camera_sensor_config_calib_lname(camera_handle_st *hcam);
extern const char* camera_sensor_config_calib_version(camera_handle_st *hcam);
extern int32_t camera_sensorl_config_has_calib(camera_handle_st *hcam);
extern int32_t camera_sensor_ops_bind(camera_handle_st *hcam, sensor_info_t *sen_if, calib_info_t *cal_if, deserial_info_t *des_if, int32_t link);
extern int32_t camera_sensor_config_parse(camera_handle_st *hcam, sensor_info_t *sen_if);
extern int32_t camera_sensor_csi_attr_parse(camera_handle_st *hcam, sensor_info_t *sen_if, mipi_config_t *mipi_to, mipi_bypass_t *bypass_to);

extern int32_t camera_sensor_is_inited(sensor_info_t *sen_if);
extern int32_t camera_sensor_is_started(sensor_info_t *sen_if);
extern int32_t camera_sensor_init(sensor_info_t *sen_if);
extern int32_t camera_sensor_deinit(sensor_info_t *sen_if);
extern int32_t camera_sensor_start(sensor_info_t *sen_if);
extern int32_t camera_sensor_stop(sensor_info_t *sen_if);
extern int32_t camera_sensor_reset(sensor_info_t *sen_if, int32_t do_stop);
extern int32_t camera_sensor_dynamic_switch_fps(sensor_info_t *sen_if, int32_t fps);
extern int32_t camera_sensor_read_register(sensor_info_t *sen_if, camera_reg_type_t type, uint32_t reg_addr);
extern int32_t camera_sensor_parse_embed_data(sensor_info_t *sen_if, char* embed_raw, struct embed_data_info_s *embed_info);
extern int32_t camera_sensor_update_ae_info(sensor_info_t *sen_if, camera_ae_info_t *ae_info);
extern int32_t camera_sensor_set_sensor_info_to_driver(sensor_info_t *sen_if);
extern int32_t camera_sensor_get_sensor_info_from_driver(int32_t camera_index, cam_parameter_t *sp);
extern int32_t camera_sensor_get_sensor_info(sensor_info_t *sen_if, camera_param_type_t type, cam_parameter_t *sp);
extern int32_t camera_sensor_stream_on(sensor_info_t *sen_if);
extern int32_t camera_sensor_stream_off(sensor_info_t *sen_if);
extern int32_t camera_sensor_get_csi_attr(sensor_info_t *sen_if, csi_attr_t *csi_attr);
extern int32_t camera_sensor_get_version(sensor_info_t *sen_if, char *name, char *version);
extern int32_t camera_sensor_dump(sensor_info_t *sen_if);
extern int32_t camera_sensor_set_cali_name(camera_handle_st *hcam, char *sensor_name, int32_t camera_index);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_SENSOR_LIB_H__ */


