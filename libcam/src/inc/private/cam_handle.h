/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_handle.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_HANDLE_H__
#define __CAM_HANDLE_H__

#include "hb_camera_data_config.h"
#include "hb_camera_data_info.h"

#include "camera_sys.h"

#include "cam_data_info.h"
#include "vpf_data_info.h"

#include "cam_config.h"
#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def CAM_HANDLE_INDEX_INVALID
 * camera handle index invalid number
 * define as: HB&c
 */
#define CAM_HANDLE_INDEX_INVALID	(-1)
/**
 * @def CAM_HANDLE_NUM_MAX
 * the max number camera module create support
 */
#ifdef CAM_CONFIG_HANDLE_NUM_MAX
#define CAM_HANDLE_NUM_MAX		CAM_CONFIG_HANDLE_NUM_MAX
#else
#define CAM_HANDLE_NUM_MAX		(256)
#endif
/**
 * @def CAM_HANDLE_MAGIC_BASE
 * camera handle magic code base to verify
 * define as: C#
 */
#define CAM_HANDLE_MAGIC_BASE		(0x4330)

/**
 * @enum camera_handle_type_e
 * camera handle type enum
 * @NO{S10E02C07}
 */
typedef enum camera_handle_type_e {
    CAM_HANDLE_INVALID = 0,
    CAM_HANDLE_CAMERA,
    CAM_HANDLE_DESERIAL,
    CAM_HANDLE_TXSER,
    CAM_HANDLE_UNSUPPORTED,
} camera_handle_type_t;

#define CAMERA_HANDLE_TYPE_NAMES { \
	"invalid", \
	"camera", \
	"deserial", \
	"txser", \
	"unsupported", \
}

/**
 * @typedef camera_handle_fd_t
 * the common camera handle fd type
 */
typedef int64_t camera_handle_fd_t;

/**
 * @def CAM_HANDLE_MAGIC_TYPE
 * camera handle magic code with type to verify
 * define as: C#(t)
 */
#define CAM_HANDLE_MAGIC_CODE(t)	(CAM_HANDLE_MAGIC_BASE | ((int8_t)(t) & 0xF))
/**
 * @def CAM_HANDLE_ID_TO_FD
 * camera handle fd get by type and id
 */
#define CAM_HANDLE_ID_TO_FD(t, id)	(((id) << 16) | CAM_HANDLE_MAGIC_CODE(t))
/**
 * @def CAM_HANDLE_FD_MAIGC_VALID
 * camera handle fd check magic is valid?
 */
#define CAM_HANDLE_FD_MAIGC_VALID(fd)	(((fd) & 0xFFF0) == CAM_HANDLE_MAGIC_BASE)
/**
 * @def CAM_HANDLE_FD_TYPE_VALID
 * camera handle fd check type is valid?
 */
#define CAM_HANDLE_FD_TYPE_VALID(fd)	((((fd) & 0xF) > CAM_HANDLE_INVALID) && (((fd) & 0xF) < CAM_HANDLE_UNSUPPORTED))
/**
 * @def CAM_HANDLE_FD_ID_VALID
 * camera handle fd check id is valid?
 */
#define CAM_HANDLE_FD_ID_VALID(fd)	((((fd) >> 16) & 0xFFFF) < CAM_HANDLE_NUM_MAX)
/**
 * @def CAM_HANDLE_FD_VALID
 * camera handle id check value is valid?
 */
#define CAM_HANDLE_FD_VALID(fd)		(CAM_HANDLE_FD_MAIGC_VALID(fd) && CAM_HANDLE_FD_TYPE_VALID(fd) && \
					 CAM_HANDLE_FD_ID_VALID(fd))
/**
 * @def CAM_HANDLE_FD_TO_ID
 * camera handle fd transfor to index
 */
#define CAM_HANDLE_FD_TO_ID(fd)		(CAM_HANDLE_FD_VALID(fd) ? ((fd) >> 16) : CAM_HANDLE_INDEX_INVALID)
/**
 * @def CAM_HANDLE_FD_TO_TYPE
 * camera handle fd transfor to type
 */
#define CAM_HANDLE_FD_TO_TYPE(fd)	(CAM_HANDLE_FD_VALID(fd) ? (camera_handle_type_t)((fd) & 0xF) : CAM_HANDLE_INVALID)
/**
 * @def CAM_HANDLE_FD_TYPE_CHCK
 * camera handle fd check the type is match?
 */
#define CAM_HANDLE_FD_TYPE_CHCK(fd, t)	(CAM_HANDLE_FD_TO_TYPE(fd) == (t))

/**
 * @struct camera_handle_head_s
 * camera handle head info struct
 * @NO{S10E02C07}
 */
typedef struct camera_handle_head_s {
	camera_handle_fd_t fd;
	char *param_buffer;
	camera_mutex_t mutex;
	int16_t id;
	int8_t type;
	int8_t reserved[5];
} camera_handle_head_t;

/**
 * @def CAM_HANDLE_HEAD_INIT
 * camera handle head struc init
 */
#define CAM_HANDLE_HEAD_INIT(t, id) { \
	.fd = CAM_HANDLE_ID_TO_FD(t, id), \
	.id = (int16_t)id, \
	.type = (int8_t)(t), \
}

/**
 * @def CAM_HANDLE_HEAD_CHECK
 * camera handles head check with verify info
 */
#define CAM_HANDLE_HEAD_CHECK(h, t)	(((h) != NULL) && CAM_HANDLE_FD_TYPE_CHCK((h)->fd, t))

/**
 * @enum camera_attach_state_e
 * camera handle attach state enum
 * @NO{S10E02C07}
 */
typedef enum camera_attach_state_e {
    CAM_ATTACH_DEFAULT = 0,
    CAM_ATTACH_VIN,
    CAM_ATTACH_DESERIAL,
    CAM_ATTACH_DES_VIN,
    CAM_ATTACH_INVALID,
} camera_attach_state_t;

#define CAMERA_ATTACH_STATE_NAMES { \
	"default", \
	"vin", \
	"deserial", \
	"des_vin", \
	"invalid", \
}


typedef struct camera_handle_ss camera_handle_st;
typedef struct deserial_handle_ss deserial_handle_st;
typedef struct txser_handle_ss txser_handle_st;

/**
 * @struct camera_handle_ss
 * camera handle info struct
 * @NO{S10E02C07}
 */
struct camera_handle_ss {
	camera_handle_head_t    head;		// 统一头部
	int32_t                 camera_index;	// camera索引
	int32_t                 deserial_link;	// deserial link索引
	camera_attach_state_t   attach_state;	// attach标志
	camera_config_t         cam_config;	// camera配置
	mipi_config_t           mipi_config;	// MIPI配置
	mipi_bypass_t		mipi_bypass;	// RX->TX bypass配置;
	mipi_config_t           mipi_to;	// MIPI配置: 运行时
	mipi_bypass_t		bypass_to;	// RX->TX bypass配置: 运行时
	camera_vin_attr_t       vin_attr;	// VIN(VCON)配置
	camera_module_lib_t     sensor_lib;	// sensor module solib
	camera_module_lib_t     calib_lib;	// calib module solib
	deserial_handle_st     *des_handle;	// attach的deserial
	vpf_handle_t            vin_handle;	// attach的vin
	void		      (*event_callback)(cam_event_t* fault_info); // event回调.
	void			*debug;		// debug用结构;
};

/**
 * @struct deserial_handle_ss
 * camera handle info struct
 * @NO{S10E02C07}
 */
struct deserial_handle_ss {
	camera_handle_head_t    head;		// 统一头部
	int32_t                 deserial_index;	// deserial索引
	camera_attach_state_t   attach_state;	// attach标志
	deserial_config_t       des_config;	// deserial配置
	poc_config_t            poc_config;	// poc配置
	mipi_config_t           mipi_config;	// MIPI配置
	mipi_bypass_t		mipi_bypass;	// RX->TX bypass配置
	mipi_config_t           mipi_to;	// MIPI配置: 运行时
	mipi_bypass_t		bypass_to;	// RX->TX bypass配置: 运行时
	camera_vin_attr_t       vin_attr;	// VIN(VCON)配置
	camera_module_lib_t     deserial_lib;	// deserial module solib
	camera_module_lib_t     poc_lib;	// poc module solib
	camera_handle_st       *cam_handle[CAMERA_DES_LINKMAX];	// attach的camera: Link
	vpf_handle_t            vin_handle[CAMERA_DES_LINKMAX]; // attach的vin: Link
	void			*debug;		// debug用结构;
};

/**
 * @struct txser_handle_ss
 * camera handle info struct
 * @NO{S10E02C07}
 */
struct txser_handle_ss {
	camera_handle_head_t    head;		// 统一头部
	int32_t                 txser_index;	// txser索引
	camera_attach_state_t   attach_state;	// attach标志
	txser_config_t          txs_config;	// txser配置
	mipi_config_t           mipi_config;	// MIPI配置
	mipi_bypass_t		mipi_bypass;	// RX->TX bypass配置;
	mipi_config_t           mipi_to;	// MIPI配置: 运行时
	mipi_bypass_t		bypass_to;	// RX->TX bypass配置: 运行时
	camera_vin_attr_t       vin_attr;	// VIN(VCON)配置
	camera_module_lib_t     txser_lib;	// txser module solib
	vpf_handle_t            vin_handle[CAM_TXSER_CSI_MAX]; // attach的vin:CSI
	void			*debug;		// debug用结构;
};

/**
 * @def CAMERA_HANDLE_T
 * covert camera handle struct pointer to fd
 */
#define CAMERA_HANDLE_T(p)		(camera_handle_t)(p->head.fd)
/**
 * @def CAMERA_HANDLE_ST
 * covert camera handle head pointer to struct pointer
 */
#define CAMERA_HANDLE_ST(phead)		(container_of(phead, camera_handle_st, head))
/**
 * @def CAMERA_HANDLE_TCHECK
 * check if fd is a valid camera handle fd
 */
#define CAMERA_HANDLE_TCHECK(fd)	CAM_HANDLE_FD_TYPE_CHCK(fd, CAM_HANDLE_CAMERA)

/**
 * @def DESERIAL_HANDLE_T
 * covert deserial handle struct pointer to fd
 */
#define DESERIAL_HANDLE_T(p)		(deserial_handle_t)(p->head.fd)
/**
 * @def DESERIAL_HANDLE_ST
 * covert deserial handle head pointer to struct pointer
 */
#define DESERIAL_HANDLE_ST(phead)	(container_of(phead, deserial_handle_st, head))
/**
 * @def DESERIAL_HANDLE_TCHECK
 * check if fd is a valid deserial handle fd
 */
#define DESERIAL_HANDLE_TCHECK(fd)	CAM_HANDLE_FD_TYPE_CHCK(fd, CAM_HANDLE_DESERIAL)

/**
 * @def TXSER_HANDLE_T
 * covert txser handle struct pointer to fd
 */
#define TXSER_HANDLE_T(p)		(txser_handle_t)(p->head.fd)
/**
 * @def TXSER_HANDLE_ST
 * covert txsedr handle head pointer to struct pointer
 */
#define TXSER_HANDLE_ST(phead)		(container_of(phead, txser_handle_st, head))
/**
 * @def TXSER_HANDLE_TCHECK
 * check if fd is a valid deserial handle fd
 */
#define TXSER_HANDLE_TCHECK(fd)		CAM_HANDLE_FD_TYPE_CHCK(fd, CAM_HANDLE_TXSER)

/**
 * @def camera_handle_lock
 * mutex lock for handle use the mutex of head
 */
#define camera_handle_lock(p)		camera_mutex_lock(&((p)->head.mutex));
/**
 * @def camera_handle_unlock
 * mutex unlock for handle use the mutex of head
 */
#define camera_handle_unlock(p)		camera_mutex_unlock(&((p)->head.mutex));

extern camera_handle_head_t* camera_handle_malloc(camera_handle_type_t type, char *param);
extern int32_t camera_handle_free(camera_handle_head_t *head);

extern camera_handle_st* camera_handle_st_by_fd(camera_handle_t cam_fd, const char *func);
extern deserial_handle_st* deserial_handle_st_by_fd(deserial_handle_t des_fd, const char *func);
extern txser_handle_st* txser_handle_st_by_fd(txser_handle_t txs_fd, const char *func);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAM_HANDLE_H__ */
