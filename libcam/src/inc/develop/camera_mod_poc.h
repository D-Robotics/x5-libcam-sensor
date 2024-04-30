/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_deserial.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_POC_H__
#define __CAMERA_MOD_POC_H__

#include <stdint.h>

#include "camera_mod_poc_data.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def VERSION
 * define the defult version string for poc module
 */
#ifndef VERSION
#define VERSION		"1.0.0"
#endif

/**
 * @struct poc_module_s
 * poc module struct with camera module base
 * @NO{S10E02C05}
 */
typedef struct poc_module_s {
	CAMERA_MODULE_HEADER;
	int32_t (*init)(poc_info_t *poc_info);
	int32_t (*power_on)(poc_info_t *poc_info, uint32_t port);
	int32_t (*power_off)(poc_info_t *poc_info, uint32_t port);
	int32_t (*deinit)(poc_info_t *poc_info);
	int32_t (*diag_nodes_init)(poc_info_t *poc_info);
} poc_module_t;

/**
 * @def POC_MODULE
 * poc so module define with flags and max 2 so_data
 * d - so_data[0]: dump regs array point
 * * - so_data[1]: NC
 * * - so_data[2]: NC
 * * - so_data[3]: NC
 * f - flags: reg operation flags
 */
#define POC_MODULE(n, f, x, d) \
		CAMERA_MODULE(n, CAM_MODULE_POC, poc_module_t, poc_info_t, f, 0U, 0U, VERSION, x, d, NULL, NULL, NULL)
/**
 * @def POC_MNAME
 * get the name string of module struct for info link
 */
#define POC_MNAME(n)			CAMERA_MODULE_NAME(n)
/**
 * @def POC_MODULE_
 * poc so module define without so_data
 */
#define POC_MODULE_(n)			DESRIALR_MODULE(n, 0U, 0, NULL)

/**
 * @def POC_MODULE_DF
 * poc so module define with 1 so_data(d) and flags
 */
#define POC_MODULE_DF(n, d, f)		DESRIALR_MODULE(n, f, 1, d)

/**
 * @def POC_MODULE_CHECK
 * check poc so module if valid with poc config info
 */
#define POC_MODULE_CHECK(m)		CAMERA_MODULE_CHECK(m, CAM_MODULE_POC, poc_module_t, poc_info_t)

/**
 * @def POC_MODULE_GET_DREGS
 * get so_data of dump regs with int32_t array
 */
#define POC_MODULE_GET_DREGS(m)		((int32_t *)((CAMERA_MODULE_GET_DATA(m, 0))))

/**
 * @def POC_MODULE_T
 * covert pointer to poc_module_t struct point
 */
#define POC_MODULE_T(m)			((poc_module_t *)(m))
/**
 * @def POC_MODULE_TS
 * get poc module pointer from poc_info_t* s and covert it to poc_module_t struct point
 */
#define POC_MODULE_TS(pi)		(((pi) != NULL) ? POC_MODULE_T((pi)->poc_ops) : NULL)
/**
 * @def POC_MODULE_M
 * get camera base camera_module_t struct pointer from poc_info_t* pi
 */
#define POC_MODULE_M(pi)		((POC_MODULE_TS(pi) != NULL) ? CAMERA_MODULE_T(POC_MODULE_TS(pi)->module) : NULL)

/**
 * @def POC_FLAGS
 * get poc flags value from poc_info_t* pi
 */
#define POC_FLAGS(pi)			(CAMERA_MODULE_GET_FLAGS(POC_MODULE_M(pi)))
/**
 * @def POC_FV_DLEN
 * get poc flags DLEN value from poc_info_t* pi
 */
#define POC_FV_DLEN(pi)			(CAM_MODULE_GET_FLAG_DLEN(POC_FLAGS(pi)))
/**
 * @def POC_FV_ALEN
 * get poc flags ALEN value from poc_info_t* pi
 */
#define POC_FV_ALEN(pi)			(CAM_MODULE_GET_FLAG_ALEN(POC_FLAGS(pi)))

/**
 * @def POC_DREGS
 * get poc dump regs array pointer from poc_info_t* pi
 */
#define POC_DREGS(pi)			(POC_MODULE_GET_DREGS(POC_MODUELE_TS(pi)->module))

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_POC_H__ */


