/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_common.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

/* dummy for legacy header */
#include "hb_camera_data_info.h"
#include "hb_camera_error.h"

#include "camera_mod_sensor.h"
#include "camera_mod_deserial.h"
#include "camera_sensor_dev.h"
#ifndef HOBOT_MCU_CAMSYS
#include "camera_sensor_dev_ioctl.h"
#endif

#include "cam_data_info.h"

#define CAM_MAX_NUM		(CAM_CONFIG_CAMERA_MAX)
#define DES_MAX_NUM		(CAM_CONFIG_DESERIAL_MAX)
#define MAX_NUM_LENGTH		(128)
#define I2C_BLOCK_MAX		(30)

typedef sensor_tuning_data_t	sensor_turning_data_t;
#define SENSOR_TURNING_PARAM    SENSOR_TUNING_PARAM

/* deserail driver adapt */
#if defined HOBOT_MCU_CAMSYS || !defined CAMERA_FRAMEWORK_HBN
#define vin_deserial_state_check(des_if, id)		({int32_t __ret = 0; __ret;})
#define vin_deserial_state_confirm(des_if, id)		({int32_t __ret = 0; __ret;})
#define vin_deserial_state_clear(des_if, id)		({int32_t __ret = 0; __ret;})
#else
#include "camera_deserial_dev.h"
#define vin_deserial_state_check(des_if, id)		camera_deserial_dev_state_check(des_if, id)
#define vin_deserial_state_confirm(des_if, link)	camera_deserial_dev_state_confirm(des_if, link)
#define vin_deserial_state_clear(des_if, link)		camera_deserial_dev_state_clear(des_if, link)
#endif

/* sensor driver adapt */
#ifdef CAMERA_FRAMEWORK_HBN
#define vin_sensor_tuning_init(sen_if, pdata)		camera_sensor_dev_tuning_init(sen_if, pdata)
#endif
