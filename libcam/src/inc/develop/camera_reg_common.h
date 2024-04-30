/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_reg_common.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_REG_COMMON_H__
#define __CAMERA_REG_COMMON_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @union camera_reg_cmd_e
 * camera subsys device register operation cmd enum
 * @NO{S10E02C07}
 */
enum camera_reg_cmd_e {
	CAM_CMD_SYS,
	CAM_CMD_READ,
	CAM_CMD_WRITE,
	CAM_CMD_MASKW,
};

/**
 * @union camera_reg_cmd_sys_e
 * camera subsys device register operation sys cmd enum
 * @NO{S10E02C07}
 */
enum camera_reg_cmd_sys_e {
	CAM_CMD_SYS_NOP,
	CAM_CMD_SYS_SLEEP,
	CAM_CMD_SYS_PRINT,
	CAM_CMD_SYS_END,
};

#define CAM_COFFSET_CMD		(6)
#define CAM_COFFSET_ATYPE	(4)
#define CAM_COFFSET_DSIZE	(0)

#define CC_CMD(c, a, d)	(((c) << CAM_COFFSET_CMD) | ((a) << CAM_COFFSET_ATYPE) | ((d) << CAM_COFFSET_DSIZE))
#define CC_BYTE(v, i)	(((a) >> ((i) * 8)) & 0xff)

#define CC_NOP()		CC_CMD(CAM_CMD_SYS, CAM_CMD_SYS_NOP, 0)
#define CC_SLP(ms)		CC_CMD(CAM_CMD_SYS, CAM_CMD_SYS_SLEEP, 4), \
					CC_BYTE(ms, 3), CC_BYTE(ms, 2), CC_BYTE(ms, 1), CC_BYTE(ms, 0)
#define CC_PRT(e)		CC_CMD(CAM_CMD_SYS, CAM_CMD_SYS_PRINT, 1), \
					CC_BYTE(e, 0)
#define CC_END()		CC_CMD(CAM_CMD_SYS, CAM_CMD_SYS_END, 0)

#define CC_RD8(a)		CC_CMD(CAM_CMD_READ, 0, 1), \
					CC_BYTE(a, 0)
#define CC_R16(a)		CC_CMD(CAM_CMD_READ, 1, 2), \
					CC_BYTE(a, 1), CC_BYTE(a, 0)
#define CC_R32(a)		CC_CMD(CAM_CMD_READ, 2, 4), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0)
#define CC_WR8(a, v)		CC_CMD(CAM_CMD_WRITE, 0, 1+1), \
					CC_BYTE(a, 0), \
					CC_BYTE(v, 0)
#define CC_W16(a, v)		CC_CMD(CAM_CMD_WRITE, 1, 2+1), \
					CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 0)
#define CC_W16D(a, v)		CC_CMD(CAM_CMD_WRITE, 1, 2+2), \
					CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 1), CC_BYTE(v, 0)
#define CC_W32(a, v)		CC_CMD(CAM_CMD_WRITE, 2, 4+1), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 0)
#define CC_W32D(a, v)		CC_CMD(CAM_CMD_WRITE, 2, 4+2), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 1), CC_BYTE(v, 0)
#define CC_W32Q(a, v)		CC_CMD(CAM_CMD_WRITE, 2, 4+4), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 3), CC_BYTE(v, 2), CC_BYTE(v, 1), CC_BYTE(v, 0)
#define CC_MW8(a, v, m)		CC_CMD(CAM_CMD_WRITE, 0, 1+1+1), \
					CC_BYTE(a, 0), \
					CC_BYTE(v, 0), \
					CC_BYTE(m, 0)
#define CC_M16(a, v, m)		CC_CMD(CAM_CMD_WRITE, 1, 2+1+1), \
					CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 0), \
					CC_BYTE(m, 0)
#define CC_M16D(a, v, m)	CC_CMD(CAM_CMD_WRITE, 1, 2+2+2), \
					CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 1), CC_BYTE(v, 0), \
					CC_BYTE(m, 1), CC_BYTE(m, 0)
#define CC_M32(a, v, m)		CC_CMD(CAM_CMD_WRITE, 2, 4+1+1), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 0), \
					CC_BYTE(m, 0)
#define CC_M32D(a, v, m)	CC_CMD(CAM_CMD_WRITE, 2, 4+2+2), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 1), CC_BYTE(v, 0), \
					CC_BYTE(m, 1), CC_BYTE(m, 0)
#define CC_M32Q(a, v, m)	CC_CMD(CAM_CMD_WRITE, 2, 4+4+4), \
					CC_BYTE(a, 3), CC_BYTE(a, 2), CC_BYTE(a, 1), CC_BYTE(a, 0), \
					CC_BYTE(v, 3), CC_BYTE(v, 2), CC_BYTE(v, 1), CC_BYTE(v, 0), \
					CC_BYTE(m, 3), CC_BYTE(m, 2), CC_BYTE(m, 1), CC_BYTE(m, 0)

/**
 * @union camera_oreg_type_e
 * camera subsys device op register data type enum
 * @NO{S10E02C07}
 */
enum camera_oreg_type_e {
	CAM_REG_INVALID,
	CAM_REG_ADDR_U8,
	CAM_REG_ADDR_U16,
	CAM_REG_ADDR_U32,
	CAM_REG_ADDR_NC,
	CAM_REG_AVAL_U8,
	CAM_REG_AVAL_U16,
	CAM_REG_AVAL_U32,
	CAM_REG_AVAL_NC,
	CAM_REG_MAX_LIST,
	CAM_REG_CMD_LIST,
	CAM_REG_TYPE_MAX,
	CAM_REG_VAL_U8 = CAM_REG_ADDR_U8,
	CAM_REG_VAL_U16 = CAM_REG_ADDR_U16,
	CAM_REG_VAL_U32 = CAM_REG_ADDR_U32,
	CAM_REG_VAL_NC = CAM_REG_ADDR_NC,
};

/**
 * @union camera_reg_data_type_e
 * camera subsys device register data type enum
 * @NO{S10E02C07}
 */
enum camera_reg_data_type_e {
	CAM_REG_DATA_CONTENT,
	CAM_REG_DATA_POINTER,
	CAM_REG_DATA_INVALID,
};

/**
 * @union camera_reg_data
 * camera register reg data union
 * @NO{S10E02C07}
 */
union camera_reg_data_t {
	char content[sizeof(void*)];
	void *data;
	char *cmd;
	uint8_t *a8;
	uint16_t *a16;
	uint32_t *a32;
};

/**
 * @struct camera_reg_s
 * camera register info struct
 * @NO{S10E02C07}
 */
typedef struct camera_reg_s {
	int8_t cmd_nop;
	int8_t reg_type;
	int8_t data_type;
	int8_t reserved;
	int32_t data_size;
	union camera_reg_data_t data;
} camera_reg_t;

#define CAM_REG_CC_NOP()	CC_CMD(CAM_CMD_SYS, CAM_CMD_SYS_NOP, 7)
#define CAM_REG_SSIZE(preg)	(((preg)->reg_type == CAM_REG_DATA_CONTENT) ? (8 + (preg)->data_size) : sizeof(camera_reg_t))
#define CAM_REG_PDATA(preg)	(((preg)->data_type == CAM_REG_DATA_CONTENT) ? (preg)->data.content : (preg)->data.data)

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_REG_COMMON_H__ */
