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

#ifndef __CAMERA_MOD_DESERIAL_H__
#define __CAMERA_MOD_DESERIAL_H__

#include <stdint.h>

#include "camera_mod_deserial_data.h"
#include "camera_deserial_dev.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def VERSION
 * define the defult version string for deserial module
 */
#ifndef VERSION
#define VERSION		"1.0.0"
#endif

/**
 * @struct deserial_module_s
 * deserial module struct with camera module base
 * @NO{S10E02C05}
 */
typedef struct deserial_module_s {
	CAMERA_MODULE_HEADER;
	int32_t (*init)(deserial_info_t *deserial_info);
	int32_t (*stream_on)(deserial_info_t *deserial_info, uint32_t port);
	int32_t (*stream_off)(deserial_info_t *deserial_info, uint32_t port);
	int32_t (*deinit)(deserial_info_t *deserial_info);
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	int32_t (*start_physical)(const deserial_info_t *deserial_info);
	int32_t (*reset)(const deserial_info_t *deserial_info);
#endif
	int32_t (*diag_nodes_init)(deserial_info_t *deserial_info);
	int32_t (*get_csi_attr)(deserial_info_t *deserial_info, csi_attr_t *csi_attr);
} deserial_module_t;

/**
 * @def DESERIAL_MODULE
 * deserial so module define with flags and max 2 so_data
 * o - so_data[0]: addtion operate funcs point
 * d - so_data[1]: dump regs array point
 * * - so_data[2]: NC
 * * - so_data[3]: NC
 * f - flags: reg operation flags
 */
#define DESERIAL_MODULE(n, f, x, o, d) \
		CAMERA_MODULE(n, CAM_MODULE_DESERIAL, deserial_module_t, deserial_info_t, f, DESERIAL_VER_MAJOR, DESERIAL_VER_MINOR, VERSION, x, o, d, NULL, NULL)
/**
 * @def DESERIAL_MNAME
 * get the name string of module struct for info link
 */
#define DESERIAL_MNAME(n)		CAMERA_MODULE_NAME(n)
/**
 * @def DESERIAL_MODULE_
 * deserial so module define without so_data
 */
#define DESERIAL_MODULE_(n)		DESERIAL_MODULE(n, 0U, 0, NULL, NULL)
/**
 * @def DESERIAL_MODULE_O
 * deserial so module define with 1 so_data(o)
 */
#define DESERIAL_MODULE_O(n, o)		DESERIAL_MODULE(n, 0U, 1, o, NULL)
/**
 * @def DESERIAL_MODULE_ODF
 * deserial so module define with 2 so_data(o,d) and flags
 */
#define DESERIAL_MODULE_ODF(n, o, d, f)	DESERIAL_MODULE(n, f, 2, o, d)

/**
 * @def DESERIAL_FLAG_NO_OPTHREAD
 * set the flag bit for deserial: not need operation thread run
 */
#define DESERIAL_FLAG_NO_OPTHREAD	(CAM_MODULE_FLAG_EXT(0x1 << 0))

/**
 * @def DESERIAL_OPS_MAX
 * define a deserial ops struct with op funcs for maxim deserial
 */
#define DESERIAL_OPS_MAX(lf, rf, mf) \
static const maxdes_ops_t deserial_ops = { \
	.link_enable = lf, \
	.remote_control = rf, \
	.mfp_cfg = mf, \
}
/**
 * @def DESERIAL_MODULE_MAX
 * deserial so module define with max ops funcs
 */
#define DESERIAL_MAX_MODULE(n, lf, rf, mf) \
	DESERIAL_OPS_MAX(lf, rf, mf); \
	DESERIAL_MODULE_O(n, &deserial_ops)
/**
 * @def DESERIAL_MODULE_MAX_DF
 * deserial so module define with max ops funcs and dump regs, flags
 */
#define DESERIAL_MAX_MODULE_DF(n, lf, rf, mf, d, f) \
	DESERIAL_OPS_MAX(lf, rf, mf); \
	DESERIAL_MODULE_ODF(n, &deserial_ops, d, f)

/**
 * @def DESERIAL_MODULE_CHECK
 * check deserial so module if valid with deserial config info
 */
#define DESERIAL_MODULE_CHECK(m)	CAMERA_MODULE_CHECK(m, CAM_MODULE_DESERIAL, deserial_module_t, deserial_info_t)

/**
 * @def DESERIAL_MODULE_GET_OPS
 * get so_data of emode with deserial_ops_type_t struct
 */
#define DESERIAL_MODULE_GET_OPS(m)	((deserial_ops_t *)((CAMERA_MODULE_GET_DATA(m, 0))))
/**
 * @def DESERIAL_MODULE_GET_DREGS
 * get so_data of dump regs with int32_t array
 */
#define DESERIAL_MODULE_GET_DREGS(m)	((int32_t *)((CAMERA_MODULE_GET_DATA(m, 1))))

/**
 * @def DESERIAL_MODULE_T
 * covert pointer to deserial_module_t struct point
 */
#define DESERIAL_MODULE_T(m)		((deserial_module_t *)(m))
/**
 * @def DESERIAL_MODULE_TS
 * get deserial module pointer from deserial_info_t* s and covert it to deserial_module_t struct point
 */
#define DESERIAL_MODULE_TS(di)		(((di) != NULL) ? DESERIAL_MODULE_T((di)->deserial_ops) : NULL)
/**
 * @def DESERIAL_MODULE_M
 * get camera base camera_module_t struct pointer from deserial_info_t* di
 */
#define DESERIAL_MODULE_M(di)		((DESERIAL_MODULE_TS(di) != NULL) ? CAMERA_MODULE_T(DESERIAL_MODULE_TS(di)->module) : NULL)

/**
 * @def DESERIAL_FLAGS
 * get deserial flags value from deserial_info_t* di
 */
#define DESERIAL_FLAGS(di)		(CAMERA_MODULE_GET_FLAGS(DESERIAL_MODULE_M(di)))
/**
 * @def DESERIAL_FV_DLEN
 * get deserial flags DLEN value from deserial_info_t* di
 */
#define DESERIAL_FV_DLEN(di)		(CAM_MODULE_GET_FLAG_DLEN(DESERIAL_FLAGS(di)))
/**
 * @def DESERIAL_FV_ALEN
 * get deserial flags ALEN value from deserial_info_t* di
 */
#define DESERIAL_FV_ALEN(di)		(CAM_MODULE_GET_FLAG_ALEN(DESERIAL_FLAGS(di)))
/**
 * @def DESERIAL_FIS_NO_OPTHREAD
 * get deserial flags NO_OPTHREAD is set from deserial_info_t* di
 */
#define DESERIAL_FIS_NO_OPTHREAD(di)	(DESERIAL_FLAGS(di) & DESERIAL_FLAG_NO_OPTHREAD)

/**
 * @def DESERIAL_OPS
 * get deserial ops struct pointer from deserial_info_t* di
 */
#define DESERIAL_OPS(di)		(DESERIAL_MODULE_GET_OPS(DESERIAL_MODULE_M(di)))
/**
 * @def DESERIAL_EMODE
 * get deserial maxim ops struct pointer from deserial_info_t* di
 */
#define DESERIAL_MAXOPS(di)		(&(DESERIAL_OPS(di)->max))
/**
 * @def DESERIAL_DREGS
 * get deserial dump regs array pointer from deserial_info_t* di
 */
#define DESERIAL_DREGS(di)		(DESERIAL_MODULE_GET_DREGS(DESERIAL_MODULE_M(di)))



#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_DESERIAL_H__ */


