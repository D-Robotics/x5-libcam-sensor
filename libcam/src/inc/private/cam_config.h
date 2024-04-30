/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
* @file cam_config.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_CONFIG_H__
#define __CAM_CONFIG_H__

/**
 * @def CAM_CONFIG_CAMERA_MAX
 * camera config: camera handle max
 */
#define CAM_CONFIG_CAMERA_MAX		(24)
/**
 * @def CAM_CONFIG_DESERIAL_MAX
 * camera config: deserial handle max
 */
#define CAM_CONFIG_DESERIAL_MAX		(6)
/**
 * @def CAM_CONFIG_TXSER_MAX
 * camera config: txser handle max
 */
#define CAM_CONFIG_TXSER_MAX		(2)
/**
 * @def CAM_CONFIG_HANDLE_NUM_MAX
 * camera config: the max number of all handle
 */
#define CAM_CONFIG_HANDLE_NUM_MAX	(256)

/**
 * @def CAM_CONFIG_INFO_LEGACY_COMPATIBLE
 * camera config: enable module info compatiable legacy
 */
#define CAM_CONFIG_INFO_LEGACY_COMPATIBLE

/**
 * @def CAM_CONFIG_DEBUG_EN
 * camera config: enable debug module
 */
#define CAM_CONFIG_DEBUG_EN
/**
 * @def CAM_CONFIG_DEBUG_DUMP_PATH
 * camera config: enable and set the default debug dump path
 */
#define CAM_CONFIG_DEBUG_DUMP_PATH	"/log/camera"

/**
 * @def CAM_CONFIG_ENV_EN
 * camera config: enable env function
 */
#define CAM_CONFIG_ENV_EN

/**
 * @def CAM_CONFIG_LIBVPF_EN
 * camera config: enable libvpf for vin
 */
#define CAM_CONFIG_LIBVPF_EN

/**
 * @def CAM_CONFIG_LIBDIAG_EN
 * camera config: enable libdiag for diag report
 */
// #define CAM_CONFIG_LIBDIAG_EN

/**
 * @def CAM_CONFIG_LOG_USE_ALOG
 * camera config: log use libalog.so
 */
#define CAM_CONFIG_LOG_USE_ALOG
/**
 * @def CAM_CONFIG_LOG_DEBUG_EN
 * camera config: log enable debug info
 */
#define CAM_CONFIG_LOG_DEBUG_EN
/**
 * @def CAM_CONFIG_LOG_MOD_FUNC
 * camera config: log pr_mod auto add __func__
 */
#define CAM_CONFIG_LOG_MOD_FUNC
/**
 * @def CAM_CONFIG_LOG_MOD_LINE
 * camera config: log pr_mod auto add __LINE__
 */
#define CAM_CONFIG_LOG_MOD_LINE
/**
 * @def CAM_CONFIG_LOG_BUFF_SIZE
 * camera config: log buffer size
 */
#define CAM_CONFIG_LOG_BUFF_SIZE (1024)

/**
 * @def CAM_CONFIG_I2C_BUS_MAX
 * camera config: i2c bus max supported
 */
#define CAM_CONFIG_I2C_BUS_MAX	(10)
/**
 * @def CAM_CONFIG_I2C_BUS_DEV_PATH
 * camera config: i2c bus max supported
 */
#define CAM_CONFIG_I2C_BUS_DEV_PATH "/dev/i2c-%d"

/**
 * @def CAM_CONFIG_SYSTEM_CFG_PATH
 * camera config: the system camera config path
 */
#define CAM_CONFIG_SYSTEM_CFG_PATH "/system/etc/cam"

/**
 * @def CAM_CONFIG_BOARD_ID_PATH
 * camera config: board id sysfs path for board_id read
 */
#define CAM_CONFIG_BOARD_ID_PATH "/sys/class/socinfo/board_id"

/**
 * @def CAM_CONFIG_SENSOR_DEV_PATH
 * camera config: sensro device path for operation
 */
#define CAM_CONFIG_SENSOR_DEV_PATH "/dev/port_%d"
/**
 * @def CAM_CONFIG_SENSOR_CDEV_PATH
 * camera config: sensro ctrl device path for 2A
 */
#define CAM_CONFIG_SENSOR_CDEV_PATH "/dev/sensor_ctrl"
/**
 * @def CAM_CONFIG_SENSOR_IDEV_PATH
 * camera config: sensro iq(calib) device path for tuning
 */
#define CAM_CONFIG_SENSOR_IDEV_PATH "/dev/sensor_iq_calib"
/**
 * @def CAM_CONFIG_DESERIAL_DEVPATH
 * camera config: deserial device path for operation
 */
#define CAM_CONFIG_DESERIAL_DEV_PATH "/dev/deserial_%d"

#endif /* __CAM_CONFIG_H__ */
