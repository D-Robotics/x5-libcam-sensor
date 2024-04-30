/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_sensor.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_SENSOR_H__
#define __CAMERA_MOD_SENSOR_H__

#include <stdint.h>

#include "hb_camera_data_info.h"

#include "camera_mod_sensor_data.h"
#include "camera_sensor_dev.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def VERSION
 * define the defult version string for sensor module
 */
#ifndef VERSION
#define VERSION		"1.0.0"
#endif

/**
 * @struct sensor_module_s
 * sensor module struct with camera module base
 * @NO{S10E02C04}
 */
typedef struct sensor_module_s {
	CAMERA_MODULE_HEADER;
	int32_t (*init)(sensor_info_t *sensor_info);
	int32_t (*deinit)(sensor_info_t *sensor_info);
	int32_t (*start)(sensor_info_t *sensor_info);
	int32_t (*stop)(sensor_info_t *sensor_info);
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	int32_t (*power_on)(sensor_info_t *sensor_info);
	int32_t (*power_off)(sensor_info_t *sensor_info);
	int32_t (*power_reset)(sensor_info_t *sensor_info);
	int32_t (*extern_isp_poweron)(sensor_info_t *sensor_info);
	int32_t (*extern_isp_poweroff)(sensor_info_t *sensor_info);
	int32_t (*extern_isp_reset)(sensor_info_t *sensor_info);
	int32_t (*spi_read)(sensor_info_t *sensor_info,  uint32_t reg_addr, char *buffer, uint32_t sizee);
	int32_t (*spi_write)(sensor_info_t *sensor_info, uint32_t reg_addr, char *buffer, uint32_t sizee);
#endif
	int32_t (*set_awb)(int32_t i2c_bus, int32_t sensor_addr, float rg_gain, float b_gain);
	int32_t (*set_ex_gain)( int32_t i2c_bus, int32_t sensor_addr, uint32_t exposure_setting,
			uint32_t gain_setting_0, uint16_t gain_setting_1);
	int32_t (*dynamic_switch_fps)(sensor_info_t *sensor_info, uint32_t fps);
	int32_t (*ae_share_init)(uint32_t flag);
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	int32_t (*get_vts)(sensor_info_t *sensor_info, uint32_t *vts);
	int32_t (*get_hts)(sensor_info_t *sensor_info, uint32_t *hts);
	int32_t (*set_vts)(sensor_info_t *sensor_info, uint32_t *vts);
	int32_t (*set_hts)(sensor_info_t *sensor_info, uint32_t *hts);
#endif
	int32_t (*get_sns_params)(sensor_info_t *sensor_info, cam_parameter_t *sp, uint8_t type);
	int32_t (*start_control)(hal_control_info_t *info);
	int32_t (*end_control)(hal_control_info_t *info);
	int32_t (*aexp_gain_control)(hal_control_info_t *info, uint32_t mode,
			uint32_t *again, uint32_t *dgain, uint32_t gain_num);
	int32_t (*aexp_line_control)(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num);
	int32_t (*aexp_line_gain_control)(hal_control_info_t *info, uint32_t mode, uint32_t *line,
			uint32_t line_num, uint32_t *again, uint32_t *dgain, uint32_t gain_num);
	int32_t (*awb_control)(hal_control_info_t *info, uint32_t mode, uint32_t rgain,
			uint32_t bgain, uint32_t grgain, uint32_t gbgain);
	int32_t (*awb_cct_control)(hal_control_info_t *info, uint32_t mode, uint32_t rgain,
			uint32_t bgain, uint32_t grgain, uint32_t gbgain, uint32_t temper);
	int32_t (*af_control)(hal_control_info_t *info, uint32_t mode, uint32_t pos);
	int32_t (*zoom_control)(hal_control_info_t *info, uint32_t mode, uint32_t pos);
	int32_t (*userspace_control)(uint32_t port, uint32_t *enable);
	int32_t (*stream_off)(sensor_info_t *sensor_info);
	int32_t (*stream_on)(sensor_info_t *sensor_info);
	int32_t (*parse_embed_data)(sensor_info_t *sensor_info, char* embed_raw, embed_data_info_t* embed_info);
	int32_t (*hotplug_init)(sensor_info_t *sensor_info);
	int32_t (*diag_nodes_init)(sensor_info_t *sensor_info);
	int32_t (*get_csi_attr)(sensor_info_t *sensor_info, csi_attr_t *csi_attr);
} sensor_module_t;

/**
 * @def SENSOR_MODULE
 * sensor so module define with flags and max 3 so_data
 * e - so_data[0]: emode struct point
 * c - so_data[1]: config_index funcs array point
 * d - so_data[2]: dump regs array point
 * * - so_data[3]: NC
 * f - flags: reg operation flags
 */
#define SENSOR_MODULE(n, f, x, e, c, d) \
		CAMERA_MODULE(n, CAM_MODULE_SENSOR, sensor_module_t, sensor_info_t, f, SENSOR_VER_MAJOR, SENSOR_VER_MINOR, VERSION, x, e, c, d, NULL)
/**
 * @def SENSOR_MNAME
 * get the name string of module struct for info link
 */
#define SENSOR_MNAME(n)			CAMERA_MODULE_NAME(n)
/**
 * @def SENSOR_MODULE_
 * sensor so module define without so_data
 */
#define SENSOR_MODULE_(n)		SENSOR_MODULE(n, 0U, 0, NULL, NULL, NULL)
/**
 * @def SENSOR_MODULE_E
 * sensor so module define with 1 so_data(e)
 */
#define SENSOR_MODULE_E(n, e)		SENSOR_MODULE(n, 0U, 1, e, NULL, NULL)
/**
 * @def SENSOR_MODULE_F
 * sensor so module define with flags
 */
#define SENSOR_MODULE_F(n, f)		SENSOR_MODULE(n, f, 0, NULL, NULL, NULL)
/**
 * @def SENSOR_MODULE_EF
 * sensor so module define with 1 so_data(e) and flags
 */
#define SENSOR_MODULE_EF(n, e, f)	SENSOR_MODULE(n, f, 1, e, NULL, NULL)
/**
 * @def SENSOR_MODULE_EC
 * sensor so module define with 2 so_data(e,c)
 */
#define SENSOR_MODULE_EC(n, e, c)	SENSOR_MODULE(n, 0U, 2, e, c, NULL)
/**
 * @def SENSOR_MODULE_ECF
 * sensor so module define with 2 so_data(e,c) and flags
 */
#define SENSOR_MODULE_ECF(n, e, c, f)	SENSOR_MODULE(n, f, 2, e, c, NULL)
/**
 * @def SENSOR_MODULE_ECDF
 * sensor so module define with 3 so_data(e,c,d) and flags
 */
#define SENSOR_MODULE_ECDF(n, e, c, d, f) SENSOR_MODULE(n, f, 3, e, c, d)

/**
 * @def SENSOR_FLAG_NO_OPTHREAD
 * set the flag bit for sensor: not need operation thread run
 */
#define SENSOR_FLAG_NO_OPTHREAD		(CAM_MODULE_FLAG_EXT(0x1 << 0))

/**
 * @def SENSOR_MODULE_CHECK
 * check sensor so module if valid with sensor config info
 */
#define SENSOR_MODULE_CHECK(m)         CAMERA_MODULE_CHECK(m, CAM_MODULE_SENSOR, sensor_module_t, sensor_info_t)

/**
 * @def SENSOR_MODULE_GET_EMODE
 * get so_data of emode with sensor_emode_type_t struct
 */
#define SENSOR_MODULE_GET_EMODE(m)	((sensor_emode_type_t *)((CAMERA_MODULE_GET_DATA(m, 0))))
/**
 * @def SENSOR_MODULE_GET_CFUNCS
 * get so_data of cfuncs with sensor_config_func array
 */
#define SENSOR_MODULE_GET_CFUNCS(m)	((sensor_config_func *)((CAMERA_MODULE_GET_DATA(m, 1))))
/**
 * @def SENSOR_MODULE_GET_DREGS
 * get so_data of dump regs with int32_t array
 */
#define SENSOR_MODULE_GET_DREGS(m)	((int32_t *)((CAMERA_MODULE_GET_DATA(m, 2))))

/**
 * @def SENSOR_MODULE_T
 * covert pointer to sensor_module_t struct point
 */
#define SENSOR_MODULE_T(m)		((sensor_module_t *)(m))
/**
 * @def SENSOR_MODULE_TS
 * get sensor module pointer from sensor_info_t* si and covert it to sensor_module_t struct point
 */
#define SENSOR_MODULE_TS(si)		(((si) != NULL) ? SENSOR_MODULE_T((si)->sensor_ops) : NULL)
/**
 * @def SENSOR_MODULE_M
 * get camera base camera_module_t struct pointer from sensor_info_t* si
 */
#define SENSOR_MODULE_M(si)		((SENSOR_MODULE_TS(si) != NULL) ? CAMERA_MODULE_T(SENSOR_MODULE_TS(si)->module) : NULL)

/**
 * @def SENSOR_FLAGS
 * get sensor flags value from sensor_info_t* si
 */
#define SENSOR_FLAGS(si)		(CAMERA_MODULE_GET_FLAGS(SENSOR_MODULE_M(si)))
/**
 * @def SENSOR_FV_DLEN
 * get sensor flags DLEN value from sensor_info_t* si
 */
#define SENSOR_FV_DLEN(si)		(CAM_MODULE_GET_FLAG_DLEN(SENSOR_FLAGS(si)))
/**
 * @def SENSOR_FV_ALEN
 * get sensor flags ALEN value from sensor_info_t* si
 */
#define SENSOR_FV_ALEN(si)		(CAM_MODULE_GET_FLAG_ALEN(SENSOR_FLAGS(si)))
/**
 * @def SENSOR_FIS_NO_OPTHREAD
 * get sensor flags NO_OPTHREAD is set from sensor_info_t* si
 */
#define SENSOR_FIS_NO_OPTHREAD(si)	(SENSOR_FLAGS(si) & SENSOR_FLAG_NO_OPTHREAD)

/**
 * @def SENSOR_EMODE
 * get sensor emode struct pointer from sensor_info_t* si
 */
#define SENSOR_EMODE(si)		(SENSOR_MODULE_GET_EMODE(SENSOR_MODULE_M(si)))
/**
 * @def SENSOR_CFUNCS
 * get sensor config funcs array pointer from sensor_info_t* si
 */
#define SENSOR_CFUNCS(si)		(SENSOR_MODULE_GET_CFUNCS(SENSOR_MODULE_M(si)))
/**
 * @def SENSOR_DREGS
 * get sensor dump regs array pointer from sensor_info_t* si
 */
#define SENSOR_DREGS(si)		(SENSOR_MODULE_GET_DREGS(SENSOR_MODULE_M(si)))


#define SENSOR_EMODE_MODE(si)		((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].mode)
#define SENSOR_EMODE_NAME(si)		((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].name)
#define SENSOR_EMODE_VERSION(si)	((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].version)
#define SENSOR_EMODE_CALIB_LNAME(si)	((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].calib_lname)
#define SENSOR_EMODE_CALIB_VERSION(si)	((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].calib_version)
#define SENSOR_EMODE_DATA(si)		((SENSOR_EMODE(si) == NULL) ? NULL : SENSOR_EMODE(si)[(si)->extra_mode].data)

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_SENSOR_H__ */


