/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_deserial_dev.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAMERA_DESERIAL_DEV_IOCTL_H__
#define __CAMERA_DESERIAL_DEV_IOCTL_H__

#include <stdint.h>
#include <sys/ioctl.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAM_CONFIG_DESERIAL_DEV_PATH
 * deserial dev path fromat string
 */
#ifdef CAM_CONFIG_DESERIAL_DEV_PATH
#define DESERIAL_DEV_PATH	CAM_CONFIG_DESERIAL_DEV_PATH
#else
#define DESERIAL_DEV_PATH	"/dev/deserial_%d"
#endif

#define DESERIAL_IOC_MAGIC      's'
#define DESERIAL_DATA_INIT      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 0, deserial_info_data_t)
#define DESERIAL_INIT_REQ       _IOW((uint32_t)DESERIAL_IOC_MAGIC, 1, int32_t)
#define DESERIAL_INIT_RESULT    _IOW((uint32_t)DESERIAL_IOC_MAGIC, 2, int32_t)
#define DESERIAL_DEINIT_REQ     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 3, int32_t)
#define DESERIAL_START_REQ      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 4, int32_t)
#define DESERIAL_START_RESULT   _IOW((uint32_t)DESERIAL_IOC_MAGIC, 5, int32_t)
#define DESERIAL_STOP_REQ       _IOW((uint32_t)DESERIAL_IOC_MAGIC, 6, int32_t)
#define DESERIAL_STREAM_GET     _IOR((uint32_t)DESERIAL_IOC_MAGIC, 7, deserial_op_info_t)
#define DESERIAL_STREAM_PUT     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 8, int32_t)
#define DESERIAL_STREAM_ON      _IOW((uint32_t)DESERIAL_IOC_MAGIC, 9, int32_t)
#define DESERIAL_STREAM_OFF     _IOW((uint32_t)DESERIAL_IOC_MAGIC, 10, int32_t)
#define DESERIAL_GET_VERSION    _IOR((uint32_t)DESERIAL_IOC_MAGIC, 11, deserial_version_info_t)
#define DESERIAL_STATE_CHECK    _IOR((uint32_t)DESERIAL_IOC_MAGIC, 12, uint32_t)
#define DESERIAL_STATE_CONFIRM  _IOW((uint32_t)DESERIAL_IOC_MAGIC, 13, uint32_t)
#define DESERIAL_STATE_CLEAR    _IOW((uint32_t)DESERIAL_IOC_MAGIC, 14, uint32_t)

/**
 * @def DESERIAL_IONAMES
 * deserial ioctl command name string array
 */
#define DESERIAL_IOC_NAMES { \
	"DESERIAL_DATA_INIT", \
	"DESERIAL_INIT_REQ", \
	"DESERIAL_INIT_RESULT", \
	"DESERIAL_DEINIT_REQ", \
	"DESERIAL_START_REQ", \
	"DESERIAL_START_RESULT", \
	"DESERIAL_STOP_REQ", \
	"DESERIAL_STREAM_GET", \
	"DESERIAL_STREAM_PUT", \
	"DESERIAL_STREAM_ON", \
	"DESERIAL_STREAM_OFF", \
	"DESERIAL_GET_VERSION", \
	"DESERIAL_STATE_CHECK", \
	"DESERIAL_STATE_CONFIRM", \
	"DESERIAL_STATE_CLEAR", \
}

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_DESERIAL_DEV_IOCTL_H__ */

