/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_data_info.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_DATA_INFO_H__
#define __CAM_DATA_INFO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @typedef bool_t
 * the bool type define
 */
typedef int32_t bool_t;

/**
 * @def TRUE
 * the bool value: TRUE
 */
#ifndef TRUE
#define TRUE		(1)
#endif
/**
 * @def FALSE
 * the bool value: FALSE
 */
#ifndef FALSE
#define FALSE		(0)
#endif

/**
 * @def RET_OK
 * the return value: OK
 * normally used as: return RET_OK;
 */
#ifndef RET_OK
#define RET_OK		(0)
#endif
/**
 * @def RET_ERROR
 * the return value: ERROR
 * normally used as: return -RET_ERROR;
 */
#ifndef RET_ERROR
#define RET_ERROR	(1)
#endif

/**
 * @def BIT
 * get value wit bit offset operation
 */
#ifndef BIT
#define BIT(i)		(1 << (i))
#endif

#ifndef container_of
#define container_of(ptr, type, member) ({ \
		typeof( ((type *)0)->member  ) *__mptr = (ptr);    \
		(type *)( (char *)__mptr - offsetof(type,member) );})
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

/**
 * @def CAM_DESERIAL_CSI_MAX
 * the max csi port of deserial supported
 */
#define CAM_DESERIAL_CSI_MAX		(4)
/**
 * @def CAM_DESERIAL_VC_MAX
 * the max vc of deserial supported
 */
#define CAM_DESERIAL_VC_MAX		(4)

/**
 * @def CAM_TXSER_CSI_MAX
 * the max csi port of tx serial supported
 */
#define CAM_TXSER_CSI_MAX		(2)

/**
 * @def CAM_CONFIG_CHECK_MIN
 * camera config value check if v >= min
 */
#define CAM_CONFIG_CHECK_MIN(v, min)	((v) >= (min))
/**
 * @def CAM_CONFIG_CHECK_MAX
 * camera config value check if v <= max
 */
#define CAM_CONFIG_CHECK_MAX(v, max)	((v) <= (max))
/**
 * @def CAM_CONFIG_CHECK_RANGE
 * camera config value check if min <= v <= max
 */
#define CAM_CONFIG_CHECK_RANGE(v, min, max) \
			(CAM_CONFIG_CHECK_MIN(v, min) && CAM_CONFIG_CHECK_MAX(v, max))

/**
 * @def CAM_CONFIG_CHECK_MASK
 * camera config value check if v & mask
 */
#define CAM_CONFIG_CHECK_MASK(v, mask)	(((v) & (mask)) != 0)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAM_DATA_INFO_H__ */
