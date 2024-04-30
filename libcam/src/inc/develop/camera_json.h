/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_json.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_JSON_H__
#define __CAMERA_JSON_H__

#include "cJSON.h"

#include "hb_camera_data_config.h"

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAMERA_JSON_NODE_NAME_LEN
 * the node name string max length for parse
 */
#define CAMERA_JSON_NODE_NAME_LEN	(16)
/**
 * @def CAMERA_JSON_STRING_FIELD_LEN
 * the field name string max length for parse
 */
#define CAMERA_JSON_STRING_FIELD_LEN	(512)
/**
 * @def CAMERA_JSON_ENV_NAME_LEN
 * the evn name string max length for parse
 */
#define CAMERA_JSON_ENV_NAME_LEN	(64)
/**
 * @def CAMERA_JSON_INT_MAX
 * the max value of int for check no max limit
 */
#define CAMERA_JSON_INT_MAX		((0x1L << 31) - 1)

/**
 * @def CAMERA_JSON_INT_MAX
 * the max value of int for check no max limit
 */
#ifdef CAM_CONFIG_SYSTEM_CFG_PATH
#define CAMERA_JSON_SYSTEM_CFG_PATH	CAM_CONFIG_SYSTEM_CFG_PATH
#else
#define CAMERA_JSON_SYSTEM_CFG_PATH	"/system/etc/cam"
#endif

/**
 * the json node name string define
 */
#define CAMERA_JSON_NAME_CONFIG_DEFAULT	"config_0"
#define CAMERA_JSON_NAME_GLOBAL		"global"
#define CAMERA_JSON_NAME_ENV		"env"
#define CAMERA_JSON_NAMEF_DESERIAL	"deserial_%d"
#define CAMERA_JSON_NAMEF_SENSOR	"port_%d"
#define CAMERA_JSON_NAMEF_ATTACH	"attach_%d"
#define CAMERA_JSON_NAMEF_TXSER		"txser_%d"
#define CAMERA_JSON_NAME_POC		"poc"
#define CAMERA_JSON_NAME_MIPI		"mipi"
#define CAMERA_JSON_NAME_DES_PARAM	"deserial_param"
#define CAMERA_JSON_NAME_TXS_PARAM	"txser_param"
#define CAMERA_JSON_NAME_SEN_PARAM	"sensor_param"

#define CAMERA_JSON_RAW_DISABLE		"disable"
#define CAMERA_JSON_STRING_NULL		"null"
#define CAMERA_JSON_RAW_FLAG_CHAR	'{'

/**
 * @struct camera_json_data_type
 * camera json node data type to parse get
 * @NO{S10E02C07}
 */
typedef enum camera_json_data_type {
	CAMER_JSON_ISINT = 0,
	CAMER_JSON_ISDOUBLE,
	CAMER_JSON_ISSTRING,
} camera_json_data_type_e;

/**
 * @struct camera_json_s
 * camera json node support inside node and independ file
 * @NO{S10E02C07}
 */
typedef struct camera_json_s {
	struct camera_json_s *parent;
	const char *name;
	const char *filename;
	char *new_name;
	char *buffer;
	int32_t buffsize;
	cJSON *root;
} camera_json_t;

/**
 * @struct camera_json_file_s
 * camera config json file parse struct
 * @NO{S10E02C07}
 */
typedef struct camera_json_file_s {
	camera_json_t root;
	camera_json_t config;
	camera_json_t global;
	camera_json_t glo_env;
	camera_json_t deserial[CAM_CONFIG_DESERIAL_MAX];
	camera_json_t des_poc[CAM_CONFIG_DESERIAL_MAX];
	camera_json_t des_mipi[CAM_CONFIG_DESERIAL_MAX];
	camera_json_t des_param[CAM_CONFIG_CAMERA_MAX];
	camera_json_t sensor[CAM_CONFIG_CAMERA_MAX];
	camera_json_t sen_param[CAM_CONFIG_CAMERA_MAX];
	camera_json_t sen_mipi[CAM_CONFIG_CAMERA_MAX];
	camera_json_t attach[CAM_CONFIG_CAMERA_MAX];
	camera_json_t txser[CAM_CONFIG_TXSER_MAX];
	camera_json_t txs_mipi[CAM_CONFIG_TXSER_MAX];
	camera_json_t txs_param[CAM_CONFIG_TXSER_MAX];
} camera_json_file_t;

/**
 * @struct camera_json_attach_s
 * camera config attach struct from json file parse
 * @NO{S10E02C07}
 */
typedef struct camera_json_attach_s {
	int32_t deserial_index;
	int32_t deserial_link;
} camera_json_attach_t;

/**
 * @struct camera_json_bypass_s
 * camera config bypass struct from json file parse
 * @NO{S10E02C07}
 */
typedef struct camera_json_bypass_s {
	int32_t csi_from[CAMERA_TXS_CSIMAX];
} camera_json_bypass_t;

/**
 * @struct camera_json_cfg_s
 * camera config struct from json file parse
 * @NO{S10E02C07}
 */
typedef struct camera_json_cfg_s {
	deserial_config_t *deserial[CAM_CONFIG_DESERIAL_MAX];
	camera_config_t *sensor[CAM_CONFIG_CAMERA_MAX];
	txser_config_t *txser[CAM_CONFIG_TXSER_MAX];
	camera_global_config_t gconfig;
	camera_json_attach_t attach[CAM_CONFIG_CAMERA_MAX];
	camera_json_bypass_t bypass[CAM_CONFIG_TXSER_MAX];
} camera_json_cfg_t;

/* internal apis */
extern int32_t camera_json_file_pre(const char *cfg_file, camera_json_file_t *jfile);
extern int32_t camera_json_file_post(camera_json_file_t *jfile);
extern int32_t camera_json_file_parse(camera_json_file_t *jfile, camera_json_cfg_t *jcfg);
extern void camera_json_file_pfree(camera_json_cfg_t *jcfg);
extern int32_t camera_json_string_parse(const char *string,
		char *field_name, int32_t datatype, void *data);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_JSON_H__ */
