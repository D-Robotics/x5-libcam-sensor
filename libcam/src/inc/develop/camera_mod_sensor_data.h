/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_sensor_data.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_SENSOR_DATA_H__
#define __CAMERA_MOD_SENSOR_DATA_H__

#include <stdint.h>

#include "camera_mod_common.h"
#include "camera_sys.h"

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @enum CONFIG_INDEX_B
 * sensor config_index bit enum
 * @NO{S10E02C04}
 */
typedef enum CONFIG_INDEX_B {
	B_AE_DISABLE,
	B_AWB_DISABLE,
	B_TEST_PATTERN,
	B_DPHY_PORTB,
	B_DPHY_COPY,
	B_EMBEDDED_MODE,
	B_EMBEDDED_DATA,
	B_TRIG_SOURCE,
	B_TRIG_STANDARD,
	B_TRIG_SHUTTER_SYNC,
	B_TRIG_EXTERNAL,
	B_DUAL_ROI,
	B_MIRROR,
	B_FLIP,
	B_PWL_24BIT,
	B_PDAF,
	B_CONFIG_INDEX_MAX,
} camera_sensor_config_index_t;

/**
 * @def CONFIG_INDEX_BNAME
 * the name string array of sensor config_index bit
 */
#define CONFIG_INDEX_BNAME {\
	"ae_disable", \
	"awb_disable", \
	"test_pattern", \
	"dphy_portb", \
	"dphy_copy", \
	"embedded_mode", \
	"embedded_data", \
	"trig_source", \
	"trig_standard", \
	"trig_shutter_sync", \
	"trig_external", \
	"dual_roi", \
	"mirror", \
	"flip", \
	"pwl_24bit", \
}

/**
 * @def SENSOR_CONFIG_ISEN
 * test if sensor_info_t s->config_index bit is enable(valid)?
 */
#define SENSOR_CONFIG_ISEN(s, c)    ((((s)->config_index) & (c)) != 0)

/**
 * @enum serial_type
 * datatype of serial type enum for S
 * @NO{S10E02C04}
 */
typedef enum serial_type {
	MAX9295A = 0,
	MAX96717 = 1,
	MAX96717F = 2,
	SER_TYPE_BUTT     // add new serial type before SER_TYPE_BUTT
} camera_serial_type_t;

/**
 * @enum emode_datatype
 * datatype of sensor emode enum for D
 * @NO{S10E02C04}
 */
typedef enum emode_datatype {
	EMODE_YUV422 = 4,	// 0x1e
	EMODE_RAW8 = 8,		// 0x2a
	EMODE_RAW10 = 10,	// 0x2b
	EMODE_RAW12 = 12,	// 0x2c
} camera_sensor_emode_datatype_t;

/**
 * @enum inf_type
 * interface type of sensor enum for I
 * @NO{S10E02C04}
 */
typedef enum inf_type {
	MIPI = 0,
	DVP = 1,
} camera_sensor_inf_type_t;

/**
 * @def EMODE_F_SEN_TRIG_GPIO
 * the flag char of emode name string: sensor trig gpio index
 */
#define EMODE_F_SEN_TRIG_GPIO	'G'
/**
 * @def EMODE_F_SEN_MCLK
 * the flag char of emode name string: sensor mclk freq MHz
 */
#define EMODE_F_SEN_MCLK	'M'
/**
 * @def EMODE_F_SEN_FOV
 * the flag char of emode name string: sensor fov
 */
#define EMODE_F_SEN_FOV		'F'
/**
 * @def EMODE_F_SEN_DATATYPE
 * the flag char of emode name string: sensor datatype
 */
#define EMODE_F_SEN_DATATYPE	'D'

/**
 * @def EMODE_F_SER_TYPE
 * the flag char of emode name string: serial type
 */
#define EMODE_F_SER_TYPE	'S'
/**
 * @def EMODE_F_SER_TRIG_PIN
 * the flag char of emode name string: serial trig mfp index
 */
#define EMODE_F_SER_TRIG_PIN	'T'
/**
 * @def EMODE_F_SER_RST_PIN
 * the flag char of emode name string: serial reset mfp index
 */
#define EMODE_F_SER_RST_PIN	'R'
/**
 * @def EMODE_F_SER_CAMERR_PIN
 * the flag char of emode name string: serial camera error mfp index
 */
#define EMODE_F_SER_CAMERR_PIN	'E'
/**
 * @def EMODE_F_SER_LINK_SPEED
 * the flag char of emode name string: serial link speed config
 */
#define EMODE_F_SER_LINK_SPEED	'L'
/**
 * @def PDESP_F_SEN_CONFIG_INDEX
 * the flag char of port_desp string: extra_mode.name@config_index
 */
#define PDESP_F_SEN_CONFIG_INDEX '@'

/**
 * @struct sensor_emode_type_s
 * sensor extra mode struct
 * @NO{S10E02C04}
 */
typedef struct sensor_emode_type_s {
	int32_t mode;
	const char *name;
	const char *version;
	const char *calib_lname;
	const char *calib_version;
	void *data;
} sensor_emode_type_t;

/**
 * @def SENSOR_EMADD
 * add emode struct of camera module to emode struct array
 * m     - emode enum index named with flag string
 * v     - version string of this emode
 * calib - the calib so name string for this emode
 * cv    - the matched version of calib so for this emode
 * d     - the private data for this emode
 */
#define SENSOR_EMADD(m, v, calib, cv, d)	{ (int32_t)(m), #m, v, calib, cv, (void *)(d) }
/**
 * @def SENSOR_EYADD
 * add emode struct of YUV camera module without calib info
 */
#define SENSOR_ESADD(m, v, d)			SENSOR_EMADD(m, v, NULL, NULL, d)
/**
 * @def SENSOR_EMEND
 * end of emode struct array
 */
#define SENSOR_EMEND()				{ 0 }

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
#define GPIO_NUMBER 6

typedef struct spi_data {
	int32_t spi_mode;
	int32_t spi_cs;
	uint32_t spi_speed;
} spi_data_t;
#endif

/* vin init and start status*/
typedef enum cam_state_s {
	CAM_INIT = 1,
	CAM_DEINIT,
	CAM_START,
	CAM_STOP,
	CAM_POWERON,
	CAM_POWEROFF,
	CAM_DNODE_INIT,
	CAM_STATE_INVALID,
	CAM_DLINIT,
} cam_state_e;

/**
 * @def HAL_LINE_CONTROL
 * hal control bit enable mask: line
 */
#define HAL_LINE_CONTROL	(0x00000001)
/**
 * @def HAL_GAIN_CONTROL
 * hal control bit enable mask: gain
 */
#define HAL_GAIN_CONTROL	(0x00000002)
/**
 * @def HAL_AWB_CONTROL
 * hal control bit enable mask: awb
 */
#define HAL_AWB_CONTROL		(0x00000004)
/**
 * @def HAL_AF_CONTROL
 * hal control bit enable mask: af
 */
#define HAL_AF_CONTROL		(0x00000008)
/**
 * @def HAL_ZOOM_CONTROL
 * hal control bit enable mask: zoom
 */
#define HAL_ZOOM_CONTROL	(0x00000010)
/**
 * @def HAL_AWB_CCT_CONTROL
 * hal control bit enable mask: awb_cct
 */
#define HAL_AWB_CCT_CONTROL	(0x00000020)
/**
 * @def HAL_AE_LINE_GAIN_CONTROL
 * hal control bit enable mask: ae_line_gain
 */
#define HAL_AE_LINE_GAIN_CONTROL (0x00000040)

/**
 * @struct hal_control_info_s
 * sensor hal control info struct
 * @NO{S10E02C04}
 */
typedef struct hal_control_info_s {
	uint32_t port;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t sensor_addr;
	uint32_t sensor1_addr;
	uint32_t serial_addr;
	uint32_t serial_addr1;
	uint32_t sensor_mode;
	uint32_t eeprom_addr;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	spi_data_t sensor_spi_info;
#endif
	uint32_t af_bus_num;
	uint32_t af_addr;
	uint32_t af_info[4];
	uint32_t zoom_bus_num;
	uint32_t zoom_addr;
	uint32_t zoom_info[4];
} hal_control_info_t;

/**
 * @struct sensor_info_s
 * sensor runtime info struct
 * @NO{S10E02C04}
 */
typedef struct sensor_info_s {
	uint32_t port;
	uint32_t bus_type;
	uint32_t bus_num;
	uint32_t isp_addr;
	uint32_t sensor_addr;
	uint32_t sensor1_addr;
	uint32_t serial_addr;
	uint32_t serial_addr1;
	uint32_t imu_addr;
	uint32_t sensor_clk;
	uint32_t eeprom_addr;
	uint32_t power_mode;
	uint32_t sensor_mode;
	uint32_t entry_num;
	uint32_t reg_width;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t gpio_num;
	int32_t gpio_pin[GPIO_NUMBER];
	int32_t gpio_level[GPIO_NUMBER];
#endif
	uint32_t fps;
	uint32_t width;
	uint32_t height;
	uint32_t format;
	uint32_t resolution;
	uint32_t extra_mode;
	uint32_t power_delay;
	int32_t deserial_index;
	int32_t deserial_port;
	char *sensor_name;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	char *config_path;
	char *data_path_info;
#endif
	void *sensor_ops;
	void *sensor_fd;
	void *deserial_info;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	uint32_t stream_control;
#endif
	uint32_t config_index;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	spi_data_t spi_info;
#endif
	cam_state_e init_state;
	cam_state_e start_state;
	int32_t sen_devfd;
	int32_t dev_port;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	int32_t init_cnt;
	int32_t start_cnt;
#endif
	uint32_t bus_timeout;
	uint32_t reserved[16];
	void *sensor_param_root;
	uint32_t sensor_errb;
	uint32_t diag_mask_disabled;
	/* new add */
	void *calib_info;
	int32_t ae_share_flag;
	int32_t ts_compensate;
	int32_t sen_cdevfd;
	int32_t iparam_mode;
	camera_pthread_t ctrl_thread_id;
	int32_t ctrl_thread_created;
	camera_pthread_t op_thread_id;
	int32_t op_thread_created;
} sensor_info_t;

/**
 * @typedef sensor_config_func
 * sensor config funct for config_index bits
 */
typedef int32_t (*sensor_config_func)(sensor_info_t *);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_SENSOR_DATA_H__ */


