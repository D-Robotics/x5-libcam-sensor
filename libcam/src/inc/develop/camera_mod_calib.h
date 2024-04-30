/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_calib.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_CALIB_H__
#define __CAMERA_MOD_CALIB_H__

#include <stdint.h>

#include "camera_mod_calib_data.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def VERSION
 * define the defult version string for calib module
 */
#ifndef VERSION
#define VERSION		"1.0.0"
#endif

/**
 * @struct calib_module_s
 * calib module struct with camera module base
 * @NO{S10E02C04}
 */
typedef struct calib_module_s {
	CAMERA_MODULE_HEADER;
	uint32_t (*get_calib_dynamic)(ACameraCalibrations *c);
	uint32_t (*get_calib_static)(ACameraCalibrations *c);
	uint32_t (*get_calib_num)(uint32_t *num);
} calib_module_t;

#define CALIB_MODULE_NAME		"cammod_calibration"

typedef struct calib_module_legacy_s {
	const char cname[CALIBRATION_NAME_SIZE];
	uint32_t (*get_calib_dynamic)(ACameraCalibrations *c);
	uint32_t (*get_calib_static)(ACameraCalibrations *c);
} calib_module_legacy_t;

#define CALIB_MODULE_LEGACY_NAME	"camera_calibration"

/**
 * @def CALIB_MODULE
 * calib so module define without flags and max 1 so_data
 * e - so_data[0]: so_name with version string
 * c - so_data[1]: NC
 * d - so_data[2]: NC
 * * - so_data[3]: NC
 */
#define CALIB_MODULE(kva, kvi, s) \
		CAMERA_MODULE(calibration, CAM_MODULE_CALIB, calib_module_t, int32_t, 0, kva, kvi, VERSION, 1, s, NULL, NULL, NULL)
/**
 * @def CALIB_MNAME
 * get the name string of module struct for info link
 */
#define CALIB_MNAME(n)			CAMERA_MODULE_NAME(n)

/**
 * @def CALIB_MODULE_CHECK
 * check calib so module if valid with calib config info
 */
#define CALIB_MODULE_CHECK(m)         CAMERA_MODULE_CHECK(m, CAM_MODULE_CALIB, calib_module_t, uint32_t)

/**
 * @def CALIB_MODULE_GET_EMODE
 * get so_data of emode with const char * so_name string
 */
#define CALIB_MODULE_GET_SONAME(m)	((const char*)((CAMERA_MODULE_GET_DATA(m, 0))))

/**
 * @def CALIB_MODULE_T
 * covert pointer to calib_module_t struct point
 */
#define CALIB_MODULE_T(m)		((calib_module_t *)(m))
/**
 * @def CALIB_MODULE_TS
 * get calib module pointer from calib_info_t* ci and covert it to calib_module_t struct point
 */
#define CALIB_MODULE_TS(ci)		(((ci) != NULL) ? CALIB_MODULE_T((ci)->calib_ops) : NULL)
/**
 * @def CALIB_MODULE_M
 * get camera base camera_module_t struct pointer from calib_info_t* ci
 */
#define CALIB_MODULE_M(ci)		((CALIB_MODULE_TS(ci) != NULL) ? CAMERA_MODULE_T(CALIB_MODULE_TS(ci)->module) : NULL)

/**
 * @def CALIB_SONAME
 * get calib emode struct pointer from calib_info_t* ci
 */
#define CALIB_SONAME(ci)		(CALIB_MODULE_GET_SONAME(CALIB_MODULE_M(ci)))

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_CALIB_H__ */


