/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_debug.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_DEBUG_H__
#define __CAM_DEBUG_H__

#include "cam_config.h"
#include "cam_handle.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CAM_CONFIG_DEBUG_EN

#define CAMERA_DEBUG_LEVEL_DISABLE	(0x0U)
#define CAMERA_DEBUG_LEVEL_ENABLE	(0x1U)
#define CAMERA_DEBUG_LEVEL_AS_INFO	(0x2U)
#define CAMERA_DEBUG_LEVEL_TAG_SHOW	(0x10U)
#define CAMERA_DEBUG_LEVEL_CALL_SHOW	(0x20U)
#define CAMERA_DEBUG_LEVEL_LOOP_SHOW	(0x40U)
#define CAMERA_DEBUG_LEVEL_I2C_SHOW	(0x80U)
#define CAMERA_DEBUG_LEVEL_DUMP_ERR	(0x100U)
#define CAMERA_DEBUG_LEVEL_DUMP_I2C	(0x200U)

#define CAMERA_DEBUG_LEVEL_DEFAULT	(CAMERA_DEBUG_LEVEL_ENABLE)
#define CAMERA_DEBUG_LEVEL_EN(v, f)	(((v) & (f)) != 0U)

#define CAMERA_DEBUG_INFO_TAG		(0x1U)
#define CAMERA_DEBUG_INFO_CALL		(0x2U)
#define CAMERA_DEBUG_INFO_LOOP		(0x4U)
#define CAMERA_DEBUG_INFO_I2C		(0x4U)
#define CAMERA_DEBUG_INFO_ALL		(0xFU)
#define CAMERA_DEBUG_INFO_EN(v, f)	(((v) & (f)) != 0U)

#define CAMERA_DEBUG_FUNC_RUN_MAX	(256)
#define CAMERA_DEBUG_FUNC_REUSE_NUM	(64)
#define CAMERA_DEBUG_FUNC_ONCE_NUM	(CAMERA_DEBUG_FUNC_RUN_MAX - CAMERA_DEBUG_FUNC_REUSE_NUM)
#define CAMERA_DEBUG_FUNC_STEP_MAX	(2)
#define CAMERA_DEBUG_LOOP_MAX		(10)
#define CAMERA_DEBUG_TAG_MAX		(10)
#define CAMERA_DEBUG_I2C_OPS_MAX	(4096)
#define CAMERA_DEBUG_I2C_REUSE_NUM	(512)
#define CAMERA_DEBUG_I2C_ONCE_NUM	(CAMERA_DEBUG_I2C_OPS_MAX - CAMERA_DEBUG_I2C_REUSE_NUM)
#define CAMERA_DEBUG_I2C_ADDR_MAX	(128)
#define CAMERA_DEBUG_I2C_DEV_MAX	(6)
#define CAMERA_DEBUG_DUMP_PATH_MAX	(512)
#define CAMERA_DEBUG_DUMP_PROC_LEN	(32)

/**
 * @struct camera_debug_func_s
 * camera debug func record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_func_s {
	const char* name;
	uint32_t step_over;
	uint32_t line[CAMERA_DEBUG_FUNC_STEP_MAX];
	uint32_t time_us[CAMERA_DEBUG_FUNC_STEP_MAX];
} camera_debug_func_t;

/**
 * @struct camera_debug_time_s
 * camera debug time diff record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_time_s {
	uint32_t min_us;
	uint32_t max_us;
	uint32_t avg_us;
} camera_debug_time_t;

/**
 * @struct camera_debug_tag_s
 * camera debug func tag record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_tag_s {
	const char* name;
	int32_t index_func;
	camera_debug_func_t *func;
} camera_debug_tag_t;

/**
 * @struct camera_debug_i2c_info_s
 * camera debug i2c info struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_i2c_info_s {
	uint8_t bus;
	uint8_t dnum;
	uint8_t addr[CAMERA_DEBUG_I2C_DEV_MAX];
	uint64_t base_us;
} camera_debug_i2c_info_t;

/**
 * @struct camera_debug_i2c_op_s
 * camera debug i2c operation record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_i2c_op_s {
	uint8_t addr_op;
	int8_t ret;
	uint8_t alen;
	uint8_t dlen;
	uint32_t time_us;
	uint32_t reg;
	uint32_t value;
} camera_debug_i2c_op_t;

/**
 * @struct camera_debug_i2c_s
 * camera debug i2c record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_i2c_s {
	uint32_t size_op;
	uint32_t index_op;
	uint32_t count_op;
	uint32_t op_over;
	camera_debug_i2c_info_t info;
	camera_debug_i2c_op_t ops[CAMERA_DEBUG_I2C_OPS_MAX];
} camera_debug_i2c_t;

/**
 * @struct camera_debug_call_s
 * camera debug func call record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_call_s {
	uint32_t index_func;
	uint32_t count_func;
	uint32_t func_over;
	uint32_t count_tag;
	uint32_t tag_over;
	camera_debug_func_t funcs[CAMERA_DEBUG_FUNC_RUN_MAX];
	camera_debug_tag_t tags[CAMERA_DEBUG_TAG_MAX];
} camera_debug_call_t;

/**
 * @struct camera_debug_lcall_s
 * camera debug loop func call struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_lcall_s {
	uint32_t count_call;
	const char *alias;
	camera_debug_func_t func;
	camera_debug_time_t t_in;
	camera_debug_time_t t_use;
} camera_debug_lcall_t;

/**
 * @struct camera_debug_loop_s
 * camera debug loop func call record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_loop_s {
	uint32_t count_loop;
	camera_debug_lcall_t lcall[CAMERA_DEBUG_LOOP_MAX];
} camera_debug_loop_t;

/**
 * @struct camera_debug_record_s
 * camera debug record struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_record_s {
	camera_debug_i2c_t i2c;
	camera_debug_call_t call;
	camera_debug_loop_t loop;
} camera_debug_record_t;

/**
 * @struct camera_debug_node_s
 * camera debug node struct for handle
 * @NO{S10E02C07}
 */
typedef struct camera_debug_node_s {
	char type_flag;
	uint8_t type;
	uint32_t index;
	uint32_t enable;
	uint32_t level;
	uint64_t base_us;
	camera_debug_record_t record;
} camera_debug_node_t;

/**
 * @struct camera_debug_s
 * camera debug runtime struct
 * @NO{S10E02C07}
 */
typedef struct camera_debug_s {
	camera_debug_node_t *cam_dbg[CAM_CONFIG_CAMERA_MAX];
	camera_debug_node_t *des_dbg[CAM_CONFIG_DESERIAL_MAX];
	camera_debug_node_t *txs_dbg[CAM_CONFIG_TXSER_MAX];
	camera_debug_i2c_t *i2c_dbg[CAM_CONFIG_I2C_BUS_MAX][CAMERA_DEBUG_I2C_ADDR_MAX];
} camera_debug_t;

/* inter apis */
extern void camera_debug_handle_init(camera_handle_head_t *handle_head);
extern void camera_debug_handle_deinit(camera_handle_head_t *handle_head);
extern void camera_debug_handle_attach(camera_handle_head_t *handle_head);
extern void camera_debug_handle_detach(camera_handle_head_t *handle_head);
extern void camera_debug_handle_enable(camera_handle_head_t *handle_head, uint32_t enable);
extern void camera_debug_handle_level(camera_handle_head_t *handle_head,  uint32_t level);
extern void camera_debug_handle_call_record(camera_handle_head_t *handle_head, const char *func_name,
	uint32_t line, const char *tag_name, uint32_t step);
extern void camera_debug_call_record(camera_handle_type_t type, uint32_t index,
	const char *func_name, uint32_t line, const char *tag_name, uint32_t step);
extern void camera_debug_loop_record(camera_handle_type_t type, uint32_t index,
	const char *func_name, uint32_t line, uint32_t loop, const char *alias_name, uint32_t step);
extern void camera_debug_i2c_record(uint8_t bus, uint8_t addr, uint8_t op,
	uint8_t alen, uint8_t dlen, uint32_t reg, uint32_t value, int32_t ret);
extern void camera_debug_show_node(camera_handle_type_t type, uint32_t index, uint32_t info);
extern void camera_debug_show(void);
extern void camera_debug_dump_i2c(const char *file);
extern void camera_debug_exec_i2c(const char *file);
extern void camera_debug_dump(const char *file);
extern void camera_debug_dump_err(const char *file);
#else /* CAM_CONFIG_DEBUG_EN */

#define camera_debug_handle_init(handle_head)
#define camera_debug_handle_deinit(handle_head)
#define camera_debug_handle_attach(handle_head)
#define camera_debug_handle_detach(handle_head)
#define camera_debug_handle_enable(handle_head, enable)
#define camera_debug_handle_level(handle_head, level)
#define camera_debug_handle_call_record(handle_head, func_name, line, tag_name, step)
#define camera_debug_call_record(type, index, func_name, line, tag_name, step)
#define camera_debug_loop_record(type, index, func_name, line, loop, alias_name, step)
#define camera_debug_i2c_record(bus, addr, op, alen, dlen, reg, value, ret)
#define camera_debug_show_node(type, index, info)
#define camera_debug_show()
#define camera_debug_dump_i2c(file)
#define camera_debug_exec_i2c(file)
#define camera_debug_dump(file)
#define camera_debug_dump_err(file)
#endif /* CAM_CONFIG_DEBUG_EN */

/* debug macro */
#define camera_debug_init(h)		camera_debug_handle_init(&(h)->head)
#define camera_debug_deinit(h)		camera_debug_handle_deinit(&(h)->head)
#define camera_debug_attach(h)		camera_debug_handle_attach(&(h)->head)
#define camera_debug_detach(h)		camera_debug_handle_detach(&(h)->head)
#define camera_debug_enable(h, enable)	camera_debug_handle_enable(&(h)->head, enable)
#define camera_debug_level(h, level)	camera_debug_handle_level(&(h)->head, level)
#define camera_debug_hcall_record(h, func_name, line, tag_name, step) \
					camera_debug_handle_call_record(&(h)->head, func_name, line, tag_name, step)

#define camera_debug_hcall_rt(h, tag_name, step) \
					camera_debug_hcall_record(h, __func__, __LINE__, tag_name, step)
#define camera_debug_hcall_rti(h, tag_name)	\
					camera_debug_hcall_rt(h, tag_name, 0U)
#define camera_debug_hcall_rto(h, tag_name)	\
					camera_debug_hcall_rt(h, tag_name, 1U)

#define camera_debug_hcall_r(h, step)	camera_debug_hcall_rt(h, NULL, step)
#define camera_debug_hcall_ri(h)	camera_debug_hcall_r(h, 0U)
#define camera_debug_hcall_ro(h)	camera_debug_hcall_r(h, 1U)

#define camera_debug_call_tr(type, index, tag_name, step) \
					camera_debug_call_record(type, index, __func__, __LINE__, tag_name, step)
#define camera_debug_call_camti(index, tag_name) camera_debug_call_tr(CAM_HANDLE_CAMERA, index, tag_name, 0U)
#define camera_debug_call_camto(index, tag_name) camera_debug_call_tr(CAM_HANDLE_CAMERA, index, tag_name, 1U)
#define camera_debug_call_desti(index, tag_name) camera_debug_call_tr(CAM_HANDLE_DESERIAL, index, tag_name, 0U)
#define camera_debug_call_desto(index, tag_name) camera_debug_call_tr(CAM_HANDLE_DESERIAL, index, tag_name, 1U)
#define camera_debug_call_txsti(index, tag_name) camera_debug_call_tr(CAM_HANDLE_TXSER, index, tag_name, 0U)
#define camera_debug_call_txsto(index, tag_name) camera_debug_call_tr(CAM_HANDLE_TXSER, index, tag_name, 1U)

#define camera_debug_call_r(type, index, step) \
					camera_debug_call_record(type, index, __func__, __LINE__, NULL, step)
#define camera_debug_call_cami(index)	camera_debug_call_r(CAM_HANDLE_CAMERA, index, 0U)
#define camera_debug_call_camo(index)	camera_debug_call_r(CAM_HANDLE_CAMERA, index, 1U)
#define camera_debug_call_desi(index)	camera_debug_call_r(CAM_HANDLE_DESERIAL, index, 0U)
#define camera_debug_call_deso(index)	camera_debug_call_r(CAM_HANDLE_DESERIAL, index, 1U)
#define camera_debug_call_txsi(index)	camera_debug_call_r(CAM_HANDLE_TXSER, index, 0U)
#define camera_debug_call_txso(index)	camera_debug_call_r(CAM_HANDLE_TXSER, index, 1U)

#define camera_debug_loop_r(type, index, loop, alias_name, step) \
					camera_debug_loop_record(type, index, __func__, __LINE__, loop, alias_name, step)
#define camera_debug_loop_cami(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_CAMERA, index, loop, alias_name, 0U)
#define camera_debug_loop_camo(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_CAMERA, index, loop, alias_name, 1U)
#define camera_debug_loop_desi(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_DESERIAL, index, loop, alias_name, 0U)
#define camera_debug_loop_deso(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_DESERIAL, index, loop, alias_name, 1U)
#define camera_debug_loop_txsi(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_TXSER, index, loop, alias_name, 0U)
#define camera_debug_loop_txso(index, loop, alias_name) camera_debug_loop_r(CAM_HANDLE_TXSER, index, loop, alias_name, 1U)

#define camera_debug_i2c_r(bus, addr, op, alen, dlen, reg, value, ret) \
					camera_debug_i2c_record((uint8_t)(bus), (uint8_t)(addr), (uint8_t)(op), \
						(uint8_t)(alen), (uint8_t)(dlen), (uint32_t)(reg), (uint32_t)(value), \
						(int32_t)((ret < 0) ? ret : 0))
#define camera_debug_i2c_rw(bus, addr, alen, dlen, reg, value, ret) \
					camera_debug_i2c_r(bus, addr, 0U, alen, dlen, reg, value, ret)
#define camera_debug_i2c_rr(bus, addr, alen, dlen, reg, value, ret) \
					camera_debug_i2c_r(bus, addr, 1U, alen, dlen, reg, value, ret)

#define camera_debug_show_cam(index, info) camera_debug_show_node(CAM_HANDLE_CAMERA, index, info)
#define camera_debug_show_des(index, info) camera_debug_show_node(CAM_HANDLE_DESERIAL, index, info)
#define camera_debug_show_txs(index, info) camera_debug_show_node(CAM_HANDLE_TXSER, index, info)
#ifdef __cplusplus
}
#endif

#endif /* __CAM_DEBUG_H__ */
