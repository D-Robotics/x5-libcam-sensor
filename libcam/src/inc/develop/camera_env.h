/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_env.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_ENV_H__
#define __CAMERA_ENV_H__

#include "cam_config.h"
#include "cam_data_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAMENV_LOGLEVEL
 * camera env sting: log level to contrl log output
 *   env value type: number, see: camera_loglevel_e
 */
#define CAMENV_LOGLEVEL			"LOGLEVEL"
/**
 * @def CAMENV_CAM_LOGLEVEL
 * camera env sting: log level to contrl log output for camera only
 *   env value type: number, see: camera_loglevel_e
 */
#define CAMENV_CAM_LOGLEVEL		"CAM_LOGLEVEL"
/**
 * @def CAMENV_I2C_OP_DUMP
 * camera env sting: i2c operation dump info ?
 *   env value type: number
 */
#define CAMENV_I2C_OP_DUMP		"CAM_I2C_OP_DUMP"
/**
 * @def CAMENV_I2C_DUMMY
 * camera env sting: i2c operation as dummy ?
 *   env value type: number
 */
#define CAMENV_I2C_DUMMY		"CAM_I2C_DUMMY"

/**
 * @def CAMENV_LD_LIBRARY_PATH
 * camera env sting: the system library path for info show, json env not support
 *   env value type: string
 */
#define CAMENV_LD_LIBRARY_PATH		"LD_LIBRARY_PATH"

/**
 * @def CAMENV_CONIFIG_SELECT
 * camera env sting: can select the json config root by env, json env not support
 *   env value type: string
 */
#define CAMENV_CONIFIG_SELECT		"CAM_CONFIG_SELECT"
/**
 * @def CAMENV_CONFIG_PATH
 * camera env sting: set the system camera json config path to used
 *   env value type: string
 */
#define CAMENV_CONFIG_PATH		"CAM_CONFIG_PATH"

/**
 * @def CAMENV_CTRL_DISABLE
 * camera env sting: sensor ctrl function disable ?
 *   env value type: number
 */
#define CAMENV_CTRL_DISABLE		"CAM_CTRL_DISABLE"

/**
 * @def CAMENV_DEBUG_LEVEL
 * camera env sting: debug functiion level set
 *   env value type: number
 */
#define CAMENV_DEBUG_LEVEL		"CAM_DEBUG_LEVEL"

/**
 * @def CAMENV_DIAG_DISABLE
 * camera env sting: diag function disable ?
 *   env value type: number
 */
#define CAMENV_DIAG_DISABLE		"CAM_DIAG_DISABLE"
/**
 * @def CAMENV_DIAG_DEBUG
 * camera env sting: diag debug enable flag ?
 *   env value type: number
 */
#define CAMENV_DIAG_DEBUG		"CAM_DIAG_DEBUG"

/**
 * @def CAMENV_CONFIG_NOCHECK
 * camera env sting: check the config struct of create?
 *   env value type: bool
 */
#define CAMENV_CONFIG_NOCHECK		"CAM_CONFIG_NOCHECK"
/**
 * @def CAMENV_MODULE_NOCHECK
 * camera env sting: check the lib so as the standard module?
 *   env value type: bool
 */
#define CAMENV_MODULE_NOCHECK		"CAM_MODULE_NOCHECK"
/**
 * @def CAMENV_CALVER_NOCHECK
 * camera env sting: check the calib lib so version is match?
 *   env value type: bool
 */
#define CAMENV_CALVER_NOCHECK		"CAM_CALVER_NOCHECK"

/**
 * @def CAMENV_DRIVER_NODESERIAL
 * camera env sting: enable if no deserial driver for test?
 *   env value type: bool
 */
#define CAMENV_DRIVER_NODESERIAL	"CAM_DRIVER_NODESERIAL"
/**
 * @def CAMENV_DRIVER_NOSENSOR
 * camera env sting: enable if no sensor driver for test?
 *   env value type: bool
 */
#define CAMENV_DRIVER_NOSENSOR		"CAM_DRIVER_NOSENSOR"
/**
 * @def CAMENV_DRIVER_NOVERSION
 * camera env sting: enable if drivers no version check?
 *   env value type: bool
 */
#define CAMENV_DRIVER_NOVERSION		"CAM_DRIVER_NOVERSION"

/**
 * @def CAMENV_MIPI_TX_WITH_RX
 * camera env sting: use mipi tx with rx but not with txser
 *   env value type: bool
 */
#define CAMENV_MIPI_TX_WITH_RX		"CAM_MIPI_TX_WITH_RX"
/**
 * @def CAMENV_VPF_MIPI_VRATIO
 * camera env sting: can modify the default ration for mipi vblank cal
 *   env value type: double
 */
#define CAMENV_VPF_MIPI_VRATIO		"CAM_VPF_MIPI_VRATIO"

#ifndef CAM_CONFIG_LIBVPF_EN
/**
 * @def CAMENV_VPF_DUMMY_FLOW
 * camera env sting: the dummy base flow id for test without vpf
 *   env value type: number
 */
#define CAMENV_VPF_DUMMY_FLOW		"CAM_VPF_DUMMY_FLOW"
/**
 * @def CAMENV_VPF_DUMMY_RX
 * camera env sting: the dummy base rx for test without vpf
 *   env value type: number
 */
#define CAMENV_VPF_DUMMY_RX		"CAM_VPF_DUMMY_RX"
/**
 * @def CAMENV_VPF_DUMMY_VC
 * camera env sting: the dummy base vc for test without vpf
 *   env value type: number
 */
#define CAMENV_VPF_DUMMY_VC		"CAM_VPF_DUMMY_VC"
/**
 * @def CAMENV_VPF_DUMMY_BUS
 * camera env sting: the dummy base bus for test without vpf
 *   env value type: number
 */
#define CAMENV_VPF_DUMMY_BUS		"CAM_VPF_DUMMY_BUS"

#define CAMENV_VPF_NAMES \
	CAMENV_VPF_MIPI_VRATIO, \
	CAMENV_VPF_DUMMY_FLOW, \
	CAMENV_VPF_DUMMY_RX, \
	CAMENV_VPF_DUMMY_VC, \
	CAMENV_VPF_DUMMY_BUS
#else
#define CAMENV_VPF_NAMES \
	CAMENV_VPF_MIPI_VRATIO
#endif

#ifdef CAM_CONFIG_ENV_EN
#define CAMENV_NAMES { \
	CAMENV_LOGLEVEL, \
	CAMENV_CAM_LOGLEVEL, \
	CAMENV_I2C_OP_DUMP, \
	CAMENV_I2C_DUMMY, \
	CAMENV_CONFIG_PATH, \
	CAMENV_CTRL_DISABLE, \
	CAMENV_DEBUG_LEVEL, \
	CAMENV_DIAG_DISABLE, \
	CAMENV_DIAG_DEBUG, \
	CAMENV_CONFIG_NOCHECK, \
	CAMENV_MODULE_NOCHECK, \
	CAMENV_CALVER_NOCHECK, \
	CAMENV_DRIVER_NODESERIAL, \
	CAMENV_DRIVER_NOSENSOR, \
	CAMENV_DRIVER_NOVERSION, \
	CAMENV_MIPI_TX_WITH_RX, \
	CAMENV_VPF_NAMES, \
}
#else
#define CAMENV_NAMES { }
#endif

/**
 * @struct camera_env_type_e
 * camera env type for auto parse and func call
 * @NO{S10E02C07}
 */
typedef enum camera_env_type_e {
	CAMERA_ENV_STRING,
	CAMERA_ENV_BOOL,
	CAMERA_ENV_LONG,
	CAMERA_ENV_ULONG,
	CAMERA_ENV_DOUBLE,
	CAMERA_ENV_INVALID,
} camera_env_type_t;

/**
 * @union camera_env_val_u
 * camera env value for auto parse and func call
 * @NO{S10E02C07}
 */
typedef union camera_env_val_e {
	const char *vstring;
	bool_t vbool;
	int32_t vlong;
	uint32_t vulong;
	double vdouble;
	char vdata[8];
} camera_env_val_t;

/**
 * @typedef camera_env_func
 * camera env auto parse and call func
 * @NO{S10E02C07}
 */
typedef int32_t (*camera_env_func)(const char *name, camera_env_val_t *value, void *data);

#ifdef CAM_CONFIG_ENV_EN
extern const char* camera_env_get(const char *name, const char *dft);
extern bool_t camera_env_get_bool(const char *name, int32_t dft);
extern int32_t camera_env_get_long(const char *name, int32_t dft);
extern uint32_t camera_env_get_ulong(const char *name, uint32_t dft);
extern double camera_env_get_double(const char *name, double dft);

extern int32_t camera_env_set(const char *name, const char *value);
extern int32_t camera_env_set_bool(const char *name, bool_t value);
extern int32_t camera_env_set_long(const char *name, int32_t value);
extern int32_t camera_env_set_ulong(const char *name, uint32_t value);
extern int32_t camera_env_set_double(const char *name, double value);

extern int32_t camera_env_call(const char *name, camera_env_type_t type,
			camera_env_func env_func, void *data);
#else
#define camera_env_get(name, dft)		(dft)
#define camera_env_get_bool(name, dft)		camera_env_get(name, dft)
#define camera_env_get_long(name, dft)		camera_env_get(name, dft)
#define camera_env_get_ulong(name, dft)		camera_env_get(name, dft)
#define camera_env_get_double(name, dft)	camera_env_get(name, dft)

#define camera_env_set(name, value)		({int32_t ret = -1; ret;})
#define camera_env_set_bool(name, value)	camera_env_set(name, value)
#define camera_env_set_long(name, value)	camera_env_set(name, value)
#define camera_env_set_ulong(name, value)	camera_env_set(name, value)
#define camera_env_set_double(name, value)	camera_env_set(name, value)

#define camera_env_call(name, type, env_func, data) ({int32_t ret = 0; ret;})
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_ENV_H__ */
