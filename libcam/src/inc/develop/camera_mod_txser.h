/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_txser.h
 *
 * @NO{S10E02C06}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_TXSER_H__
#define __CAMERA_MOD_TXSER_H__

#include <stdint.h>

#include "camera_mod_txser_data.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def VERSION
 * define the defult version string for txser module
 */
#ifndef VERSION
#define VERSION		"1.0.0"
#endif

/**
 * @struct txser_module_s
 * txser module struct with camera module base
 * @NO{S10E02C06}
 */
typedef struct txser_module_s {
	CAMERA_MODULE_HEADER;
	int32_t (*init)(txser_info_t *txser_info);
	int32_t (*deinit)(txser_info_t *txser_info);
	int32_t (*diag_nodes_init)(txser_info_t *txser_info);
} txser_module_t;

/**
 * @def TXSER_MODULE
 * txser so module define with flags and max 1 so_data
 * d - so_data[0]: dump regs array point
 * * - so_data[1]: NC
 * * - so_data[2]: NC
 * * - so_data[3]: NC
 * f - flags: reg operation flags
 */
#define TXSER_MODULE(n, f, x, d) \
		CAMERA_MODULE(n, CAM_MODULE_TXSER, txser_module_t, txser_info_t, f, 0U, 0U, VERSION, x, d, NULL, NULL, NULL)
/**
 * @def TXSER_MNAME
 * get the name string of module struct for info link
 */
#define TXSER_MNAME(n)			CAMERA_MODULE_NAME(n)
/**
 * @def TXSER_MODULE_
 * txser so module define without so_data
 */
#define TXSER_MODULE_(n)		TXSER_MODULE(n, 0U, 0, NULL)
/**
 * @def TXSER_MODULE_DF
 * txser so module define with 1 so_data(d) and flags
 */
#define TXSER_MODULE_DF(n, d, f)	TXSER_MODULE(n, f, 1, d)

/**
 * @def TXSER_MODULE_CHECK
 * check txser so module if valid with txser config info
 */
#define TXSER_MODULE_CHECK(m)		CAMERA_MODULE_CHECK(m, CAM_MODULE_TXSER, txser_module_t, txser_info_t)

/**
 * @def TXSER_MODULE_GET_DREGS
 * get so_data of dump regs with int32_t array
 */
#define TXSER_MODULE_GET_DREGS(m)	((int32_t *)((CAMERA_MODULE_GET_DATA(m, 0))))

/**
 * @def TXSER_MODULE_T
 * covert pointer to txser_module_t struct point
 */
#define TXSER_MODULE_T(m)		((txser_module_t *)(m))
/**
 * @def TXSER_MODULE_TS
 * get txser module pointer from txser_info_t* ti and covert it to txser_module_t struct point
 */
#define TXSER_MODULE_TS(ti)		(((ti) != NULL) ? TXSER_MODULE_T((ti)->txser_ops) : NULL)
/**
 * @def TXSER_MODULE_M
 * get camera base camera_module_t struct pointer from txser_info_t* ti
 */
#define TXSER_MODULE_M(ti)		((TXSER_MODULE_TS(ti) != NULL) ? CAMERA_MODULE_T(TXSER_MODULE_TS(ti)->module) : NULL)

/**
 * @def TXSER_FLAGS
 * get txser flags value from txser_info_t* ti
 */
#define TXSER_FLAGS(ti)			(CAMERA_MODULE_GET_FLAGS(TXSER_MODULE_M(ti)))
/**
 * @def TXSER_FV_DLEN
 * get txser flags DLEN value from txser_info_t* ti
 */
#define TXSER_FV_DLEN(ti)		(CAM_MODULE_GET_FLAG_DLEN(TXSER_FLAGS(ti)))
/**
 * @def TXSER_FV_ALEN
 * get txser flags ALEN value from txser_info_t* ti
 */
#define TXSER_FV_ALEN(ti)		(CAM_MODULE_GET_FLAG_ALEN(TXSER_FLAGS(ti)))

/**
 * @def TXSER_DREGS
 * get txser dump regs array pointer from txser_info_t* ti
 */
#define TXSER_DREGS(ti)			(TXSER_MODULE_GET_DREGS(TXSER_MODUELE_M(ti)))

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_TXSER_H__ */


