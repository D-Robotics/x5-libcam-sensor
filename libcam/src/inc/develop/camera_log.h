/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_log.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_LOG_H__
#define __CAMERA_LOG_H__

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAMERA_LOG_PREFIX_LENGTH
 * camera log time prefix buffer length
 */
#define CAMERA_LOG_PREFIX_LENGTH	(32)
/**
 * @def CAMERA_LOG_PREFIX_LENGTH
 * camera log time prefix buffer length
 */
#ifdef CAM_CONFIG_LOG_BUFF_SIZE
#define CAMERA_LOG_BUFFER_LENGTH	(CAM_CONFIG_LOG_BUFF_SIZE)
#else
#define CAMERA_LOG_BUFFER_LENGTH	(1024)
#endif

#ifndef pr_fmt
#ifndef pr_mod
/* no pr_fmt pr_mod */
#define pr_fmt(fmt) fmt
#else /* pr_mod */
/* pr_mod support (without pr_fmt) */
#if defined CAM_CONFIG_LOG_MOD_FUNC && defined CAM_CONFIG_LOG_MOD_LINE
#define pr_fmt(fmt) "[" pr_mod "]:[%s][%d] " fmt, __func__, __LINE__
#elif defined CAM_CONFIG_LOG_MOD_FUNC
#define pr_fmt(fmt) "[" pr_mod "]:[%s] " fmt, __func__
#elif defined CAM_CONFIG_LOG_MOD_LINE
#define pr_fmt(fmt) "[" pr_mod "]:[%d] " fmt, __LINE__
#else
#define pr_fmt(fmt) "[" pr_mod "]: "
#endif
#endif /* pr_mod */
#else
#endif /* pr_fmt */

/**
 * enum camera_loglevel_e
 * log level enum to control log output
 * @NO{S10E02C07}
 */
typedef enum camera_loglevel_e {
	CAM_NONE = 0,
	CAM_ERR,
	CAM_WARN,
	CAM_INFO,
	CAM_DEBUG,
	CAM_LOGLEVEL_MAX
} camera_loglevel_t;

#ifdef HOBOT_MCU_CAMSYS
#include <stdio.h>

#define camera_log_warpper(level, fmt, ...)	printf(fmt, ##__VA_ARGS__)

#else /* HOBOT_MCU_CAMSYS */
#ifdef CAM_CONFIG_LOG_USE_ALOG
#include <logging.h>

#define camera_printLog_dbg(fmt, ...)	android_printLog(3, NULL, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_info(fmt, ...)	android_printLog(4, NULL, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_warn(fmt, ...)	android_printLog(5, NULL, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_err(fmt, ...)	android_printLog(6, NULL, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#else
#include <stdio.h>

#define camera_printLog_dbg(fmt, ...)	fprintf(stdout, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_info(fmt, ...)	fprintf(stdout, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_warn(fmt, ...)	fprintf(stdout, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define camera_printLog_err(fmt, ...)	fprintf(stderr, fmt, ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#endif

extern void camera_log_warpper(camera_loglevel_t level, const char *format, ...);
#endif /* HOBOT_MCU_CAMSYS */

#define cam_err(fmt, ...)		camera_log_warpper(CAM_ERR, pr_fmt(fmt), ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define cam_warn(fmt, ...)		camera_log_warpper(CAM_WARN, pr_fmt(fmt), ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#define cam_info(fmt, ...)		camera_log_warpper(CAM_INFO, pr_fmt(fmt), ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#ifdef CAM_CONFIG_LOG_DEBUG_EN
#define cam_dbg(fmt, ...)		camera_log_warpper(CAM_DEBUG, pr_fmt(fmt), ##__VA_ARGS__)  /*PRQA S 0342,1038*/
#else
#define cam_dbg(fmt, ...)
#endif /* CAM_CONFIG_DEBUG_EN */

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_LOG_H__ */
