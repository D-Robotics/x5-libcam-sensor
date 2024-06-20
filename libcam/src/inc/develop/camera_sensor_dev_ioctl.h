/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_dev_ioctl.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_SENSOR_DEV_IOCTL_H__
#define __CAMERA_SENSOR_DEV_IOCTL_H__

#include <stdint.h>
#include <sys/ioctl.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def SENSOR_DEV_PATH
 * sensor dev path fromat string
 */
#ifdef CAM_CONFIG_SENSOR_DEV_PATH
#define SENSOR_DEV_PATH		CAM_CONFIG_SENSOR_DEV_PATH
#else
#define SENSOR_DEV_PATH		"/dev/port_%d"
#endif

#define SENSOR_IOC_MAGIC   'x'
#define SENSOR_TUNING_PARAM   _IOW(SENSOR_IOC_MAGIC, 0, sensor_tuning_data_t)
#define SENSOR_OPEN_CNT       _IOR(SENSOR_IOC_MAGIC, 1, int32_t)
#define SENSOR_SET_START_CNT  _IOW(SENSOR_IOC_MAGIC, 2, int32_t)
#define SENSOR_GET_START_CNT  _IOR(SENSOR_IOC_MAGIC, 3, int32_t)
#define SENSOR_USER_LOCK      _IOW(SENSOR_IOC_MAGIC, 4, int32_t)
#define SENSOR_USER_UNLOCK    _IOW(SENSOR_IOC_MAGIC, 5, int32_t)
#define SENSOR_AE_SHARE       _IOW(SENSOR_IOC_MAGIC, 6, int32_t)

#define SENSOR_SET_INIT_CNT   _IOW(SENSOR_IOC_MAGIC, 8, int32_t)
#define SENSOR_GET_INIT_CNT   _IOR(SENSOR_IOC_MAGIC, 9, int32_t)
#define SENSOR_INPUT_PARAM    _IOW(SENSOR_IOC_MAGIC, 10, sensor_input_param_t)
#define SENSOR_SET_INTRINSIC_PARAM	_IOW(SENSOR_IOC_MAGIC, 11, cam_usr_info_t)
#define SENSOR_GET_INTRINSIC_PARAM	_IOR(SENSOR_IOC_MAGIC, 12, cam_usr_info_t)

#define SENSOR_INIT_REQ       _IOW(SENSOR_IOC_MAGIC, 16, int32_t)
#define SENSOR_INIT_RESULT    _IOW(SENSOR_IOC_MAGIC, 17, int32_t)
#define SENSOR_DEINIT_REQ     _IOW(SENSOR_IOC_MAGIC, 18, int32_t)
#define SENSOR_START          _IOW(SENSOR_IOC_MAGIC, 19, int32_t)
#define SENSOR_STOP           _IOW(SENSOR_IOC_MAGIC, 20, int32_t)
#define SENSOR_EVENT_GET      _IOR(SENSOR_IOC_MAGIC, 21, sensor_event_info_t)
#define SENSOR_EVENT_PUT      _IOW(SENSOR_IOC_MAGIC, 22, int32_t)
#define SENSOR_UPDATE_AE_INFO _IOW(SENSOR_IOC_MAGIC, 23, camera_ae_info_t)
#define SENSOR_GET_VERSION    _IOR(SENSOR_IOC_MAGIC, 24, sensor_version_info_t)

/**
 * @def SENSOR_IOC_NAMES
 * sensor ioctl command name string array
 */
#define SENSOR_IOC_NAMES { \
	"SENSOR_TUNING_PARAM", \
	"SENSOR_OPEN_CNT", \
	"SENSOR_SET_START_CNT", \
	"SENSOR_GET_START_CNT", \
	"SENSOR_USER_LOCK", \
	"SENSOR_USER_UNLOCK", \
	"SENSOR_AE_SHARE", \
	"NC7", \
	"SENSOR_SET_INIT_CNT", \
	"SENSOR_GET_INIT_CNT", \
	"SENSOR_INPUT_PARAM", \
	"SENSOR_SET_INTRINSIC_PARAM", \
	"SENSOR_GET_INTRINSIC_PARAM", \
	"NC13", \
	"NC14", \
	"NC15", \
	"SENSOR_INIT_REQ", \
	"SENSOR_INIT_RESULT", \
	"SENSOR_DEINIT_REQ", \
	"SENSOR_START", \
	"SENSOR_STOP", \
	"SENSOR_EVENT_GET", \
	"SENSOR_EVENT_PUT", \
	"SENSOR_UPDATE_AE_INFO", \
	"SENSOR_GET_VERSION", \
}

/**
 * @def SENSOR_CDEV_PATH
 * sensor ctrl dev path string
 */
#ifdef CAM_CONFIG_SENSOR_CDEV_PATH
#define SENSOR_CDEV_PATH	CAM_CONFIG_SENSOR_CDEV_PATH
#else
#define SENSOR_CDEV_PATH	"/dev/sensor_ctrl"
#endif

#define CAMERA_CTRL_IOC_MAGIC	'x'
#define CAMERA_CTRL_IOC_BASE	(40)
#define SENSOR_CTRL_INFO_SYNC	_IOWR(CAMERA_CTRL_IOC_MAGIC, 40, sensor_ctrl_info_t)
#define SENSOR_CTRL_RESULT      _IOW(CAMERA_CTRL_IOC_MAGIC, 41, sensor_ctrl_result_t)
#define SENSOR_CTRL_GET_VERSION	_IOR(CAMERA_CTRL_IOC_MAGIC, 48, sensor_version_info_t)

/**
 * @def SENSOR_CTRL_IOC_NAMES
 * sensor ctrl ioctl command name string array
 */
#define SENSOR_CTRL_IOC_NAMES { \
	"SENSOR_CTRL_INFO_SYNC", \
	"SENSOR_CTRL_RESULT", \
	"NC2", "NC3",  "NC4", "NC5", "NC6", "NC7",\
	"SENSOR_CTRL_GET_VERSION", \
}

/**
 * @def SENSOR_IDEV_PATH
 * sensor iq(calib) dev path string
 */
#ifdef HB_X5_CALI
#define SENSOR_IDEV_PATH        "/dev/isi_sensor"
#define AC_CALIB_IOC_MAGIC      's'	// ISI_SENSOR_IOC_MAGIC
#define AC_CALIB_INIT           _IOW(AC_CALIB_IOC_MAGIC, 13, camera_calib_t)
#else
#ifdef CAM_CONFIG_SENSOR_IDEV_PATH
#define SENSOR_IDEV_PATH	CAM_CONFIG_SENSOR_IDEV_PATH
#else
#define SENSOR_IDEV_PATH	"/dev/sensor_iq_calib"
#endif

#define AC_CALIB_IOC_MAGIC      'd'
#define AC_CALIB_INIT           _IOW(AC_CALIB_IOC_MAGIC, 0, camera_calib_t)
#endif

#define AC_CALIB_RELEASE        _IOW(AC_CALIB_IOC_MAGIC, 1, camera_calib_t)
#define AC_CALIB_TOTAL_SIZE     _IOWR(AC_CALIB_IOC_MAGIC, 4, uint32_t)
#define AC_CALIB_GET_VERSION    _IOR(AC_CALIB_IOC_MAGIC, 5, sensor_version_info_t)
/**
 * @def SENSOR_IQ_IOC_NAMES
 * sensor iq ioctl command name string array
 */
#define SENSOR_IQ_IOC_NAMES { \
	"AC_CALIB_INIT", \
	"AC_CALIB_RELEASE", \
	"NC3", \
	"AC_CALIB_TOTAL_SIZE", \
	"AC_CALIB_GET_VERSION", \
}

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_SENSOR_DEV_IOCTL_H__ */


