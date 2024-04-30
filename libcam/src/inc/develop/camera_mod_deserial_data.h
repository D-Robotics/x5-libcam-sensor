/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_deserial_data.h
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_DESERIAL_DATA_H__
#define __CAMERA_MOD_DESERIAL_DATA_H__

#include <stdint.h>

#include "camera_mod_common.h"
#include "camera_sys.h"

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DES_CSI_NUM_MAX		(4)
#define DES_LINK_NUM_MAX	(4)
#define DES_GPIO_MAX		(16)

#define TRIG_GPIOARR_INDEX	(0)
#define CAMERR_GPIOARR_INDEX	(4)
#define ERRB_GPIOARR_INDEX	(8)
#define LOCK_GPIOARR_INDEX	(9)

#define DES_LINK_MAP_CSI(v, l)	(((v) >> (((l) * 4) + 2)) & 0x3)
#define DES_LINK_MAP_VC(v, l)	(((v) >> ((l) * 4)) & 0x3)

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
#define GPIO_NUMBER 6
#endif

/**
 * @struct deserial_info_s
 * deserial runtime info struct
 * @NO{S10E02C05}
 */
typedef struct deserial_info_s {
	uint32_t index;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t deserial_addr;
	uint32_t power_mode;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t physical_entry;
	uint32_t mfp_index;
	uint32_t poc_addr;
	uint32_t poc_map;
#endif
	uint32_t lane_mode;
	uint32_t lane_speed;
	uint32_t deserial_csi[DES_CSI_NUM_MAX];
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t gpio_num;
	int32_t gpio_pin[GPIO_NUMBER];
	int32_t gpio_level[GPIO_NUMBER];
#endif
	char *deserial_name;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	char *deserial_config_path;
#endif
	void *deserial_ops;
	void *deserial_fd;
	void  *sensor_info[DES_LINK_NUM_MAX];
	int32_t init_state;
	camera_pthread_t init_thread_id;
	int32_t thread_created;
	uint32_t bus_timeout;
	uint8_t gmsl_speed[DES_LINK_NUM_MAX];
	uint32_t data_type[DES_LINK_NUM_MAX];
	char  *serial_type[DES_LINK_NUM_MAX];
	int32_t deserial_gpio[DES_GPIO_MAX];
	char *port_desp[DES_LINK_NUM_MAX];
	uint32_t deserial_attr;
	/* new add */
	int32_t des_devfd;
	int8_t data_info_inited;
	int8_t reserved8[3];
	int32_t gpio_enable;
	int32_t gpio_levels;
	int32_t reset_delay;
	void *poc_info;
	uint32_t reserved[10];
} deserial_info_t;

#define TRIG_PIN(d, i)		(d->deserial_gpio[TRIG_GPIOARR_INDEX+(i)])
#define CAMERR_PIN(d, i)	(d->deserial_gpio[CAMERR_GPIOARR_INDEX + (i)])
#define ERRB_PIN(d)		(d->deserial_gpio[ERRB_GPIOARR_INDEX])
#define LOCK_PIN(d)		(d->deserial_gpio[LOCK_GPIOARR_INDEX])

/**
 * @struct maxdes_ops_s
 * maxim deserial operation functions struct
 * @NO{S10E02C05}
 */
typedef struct maxdes_ops_s {
    int32_t (*link_enable)(deserial_info_t *deserial_info, uint8_t link_mask);
    int32_t (*remote_control)(deserial_info_t *deserial_info, uint8_t link_mask);
    int32_t (*mfp_cfg)(deserial_info_t *deserial_if, uint8_t mfp_mode, uint8_t gpio_id, uint8_t deserial_link);
} maxdes_ops_t;

/**
 * @union deserial_ops_u
 * deserial operation functions union
 * @NO{S10E02C05}
 */
typedef union deserial_ops_u {
    maxdes_ops_t max;
} deserial_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_DESERIAL_DATA_H__ */


