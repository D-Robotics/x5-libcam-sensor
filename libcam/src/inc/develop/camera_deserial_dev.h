/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_deserial_dev.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAMERA_DESERIAL_DEV_H__
#define __CAMERA_DESERIAL_DEV_H__

#include <stdint.h>

#include "camera_reg_common.h"
#include "camera_mod_deserial_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DESERIAL_VER_MAJOR     (1u)
#define DESERIAL_VER_MINOR     (0u)

/**
 * @def DESERIAL_DEV_NAME_LEN
 * deserial dev name string length
 */
#define DESERIAL_DEV_NAME_LEN	(32)
/**
 * @def DESERIAL_LINK_NUM_MAX
 * deserial link number maaxl
 */
#define DESERIAL_LINK_NUM_MAX	(4)
/**
 * @def DESERIAL_NAME_LEN_MA
 * deserial chip name string length
 */
#define DESERIAL_NAME_LEN_MAX	(32)
/**
 * @def DESERIAL_GPIO_NUM_MAX
 * deserial gpio number max
 */
#define DESERIAL_GPIO_NUM_MAX	(32)
/**
 * @def DESERIAL_PORT_DESPLEN
 * deserial port description string length
 */
#define DESERIAL_PORT_DESPLEN	(64)

/**
 * @struct deserial_version_info_s
 * deserial driver version info struct
 * @NO{S10E02C05}
 */
typedef struct deserial_version_info_s {
	uint32_t major;/**< the major version number >*/
	uint32_t minor;/**< the minor version number >*/
} deserial_version_info_t;

/**
 * @struct deserial_info_data_s
 * deserial device info data struct
 * @NO{S10E02C05}
 */
typedef struct deserial_info_data_s {
	uint32_t  index;
	char      deserial_name[DESERIAL_NAME_LEN_MAX];
	uint32_t  deserial_addr;
	char      poc_name[DESERIAL_NAME_LEN_MAX];
	uint32_t  poc_addr;
	uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_addr;
	uint32_t  chip_id;
	int32_t  gpios[DESERIAL_GPIO_NUM_MAX];
	uint32_t  gpio_enable;
	uint32_t  gpio_level;
	uint32_t  poc_map;
	uint32_t  link_map;
	int32_t sensor_index[DESERIAL_LINK_NUM_MAX];
	char link_desp[DESERIAL_LINK_NUM_MAX][DESERIAL_PORT_DESPLEN];
	camera_reg_t init;
	camera_reg_t start;
	camera_reg_t stop;
	camera_reg_t deinit;
} deserial_info_data_t;

/**
 * @struct deseial_op_type_e
 * deserial device user operation state
 * @NO{S10E02C05}
 */
typedef enum deseial_op_type_e {
	DES_OP_TYPE_INVALID = 0,
	DES_OP_TYPE_STREAM,
	DES_OP_TYPE_MAX,
} deseial_op_type_t;

/**
 * @struct deserial_op_info_s
 * deserial device operation info struct
 * @NO{S10E02C05}
 */
typedef struct deserial_op_info_s {
	int32_t type;
	int32_t data;
	int32_t reserved[2];
} deserial_op_info_t;

typedef int32_t (*DESERIAL_INFO_DATA_FILL_FUNC)(deserial_info_t *des_if, deserial_info_data_t *data);

/* internal apis */
extern int32_t camera_deserial_dev_open(deserial_info_t *des_if);
extern int32_t camera_deserial_dev_nodrv(deserial_info_t *des_if);
extern int32_t camera_deserial_dev_close(deserial_info_t *des_if);
extern int32_t camera_deserial_dev_info_init(deserial_info_t *des_if, DESERIAL_INFO_DATA_FILL_FUNC fill);
extern int32_t camera_deserial_dev_init_req(deserial_info_t *des_if, int32_t timeout);
extern int32_t camera_deserial_dev_init_result(deserial_info_t *des_if, int32_t result);
extern int32_t camera_deserial_dev_deinit(deserial_info_t *des_if);
extern int32_t camera_deserial_dev_start_req(deserial_info_t *des_if, int32_t timeout);
extern int32_t camera_deserial_dev_start_result(deserial_info_t *des_if, int32_t result);
extern int32_t camera_deserial_dev_stop(deserial_info_t *des_if);
extern int32_t camera_deserial_dev_pre_req(int32_t deserial_index, int32_t type, int32_t timeout);
extern int32_t camera_deserial_dev_pre_result(int32_t deserial_index, int32_t type, int32_t result);
extern int32_t camera_deserial_dev_stream_get(deserial_info_t *des_if, deserial_op_info_t *op_info);
extern int32_t camera_deserial_dev_stream_put(deserial_info_t *des_if, int32_t result);
extern int32_t camera_deserial_dev_stream_on(deserial_info_t *des_if, int32_t link);
extern int32_t camera_deserial_dev_stream_off(deserial_info_t *des_if, int32_t link);
extern int32_t camera_deserial_dev_get_version(deserial_info_t *des_if, deserial_version_info_t *ver);
extern int32_t camera_deserial_dev_state_check(deserial_info_t *des_if, uint32_t check_id);
extern int32_t camera_deserial_dev_state_confirm(deserial_info_t *des_if, uint32_t confirm_id);
extern int32_t camera_deserial_dev_state_clear(deserial_info_t *des_if, uint32_t clear_id);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_DESERIAL_DEV_H___CAMERA_SENSOR_DEV_H__ */


