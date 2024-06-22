/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_dev.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_SENSOR_DEV_H__
#define __CAMERA_SENSOR_DEV_H__

#include <stdint.h>

#include "hb_camera_data_info.h"

#include "camera_mod_sensor_data.h"
#include "camera_mod_calib_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct sensor_version_info_s
 * sensor driver version info struct
 * @NO{S10E02C05}
 */
typedef struct sensor_version_info_s {
	uint32_t major;/**< the major version number >*/
	uint32_t minor;/**< the minor version number >*/
} sensor_version_info_t;

/**
 * sensor driver
 */
#define SENSOR_VER_MAJOR     (1u)
#define SENSOR_VER_MINOR     (0u)

/**
 * @def SENSOR_DEV_NAME_LEN
 * sensor dev name string length
 */
#define SENSOR_DEV_NAME_LEN	(32)
#define CAMERA_SENSOR_NAME	(20)

#define SENSOR_SHARE_AE_FLAG	(0xA0u)
#define SENSOR_AE_SHARE_FLAG(src, user) \
				(((SENSOR_SHARE_AE_FLAG | (src)) << 16) | (user))
#define SENSOR_AE_SHARE_SRC(flag) (((((flag) >> 16) & 0xE0) == SENSOR_SHARE_AE_FLAG) ? \
				(((flag) >> 16) & 0x1F) : -1);
#define SENSOR_AE_SHARE_USER(flag) (((((flag) >> 16) & 0xE0) == SENSOR_SHARE_AE_FLAG) ? \
				((flag) & 0xFFFF) : -1);

/**
 * @enum enum_bayer_start_e
 * sensor device param of bayer start enum
 * @NO{S10E02C05}
 */
typedef enum enum_bayer_start_e
{
	BAYER_START_R    = 0,
	BAYER_START_GR   = 1,
	BAYER_START_GB   = 2,
	BAYER_START_B    = 3,
	BAYER_START_IR   = 4,
	BAYER_START_C    = 5,
	BAYER_START_CY   = 6,

	BAYER_START_BUTT
} bayer_start_e;

/**
 * @struct enum_bayer_rgbir_4x4_start_e
 * sensor bayer pattern start type
 */
typedef enum enum_bayer_rgbir_4x4_statr_e {
	BAYER_RGBIR_4x4_START_GRIRG = 0,//BAYER_RGGB_START_GBRG
	BAYER_RGBIR_4x4_START_RGGIR = 1,//BAYER_RGGB_START_BGGR
	BAYER_RGBIR_4x4_START_GBIRG = 2,//BAYER_RGGB_START_GBRG
	BAYER_RGBIR_4x4_START_BGGIR = 3,//BAYER_RGGB_START_BGGR
	BAYER_RGBIR_4x4_START_IRGGB = 4,//BAYER_RGGB_START_RGGB
	BAYER_RGBIR_4x4_START_GIRBG = 5,//BAYER_RGGB_START_GRBG
	BAYER_RGBIR_4x4_START_IRGGR = 6,//BAYER_RGGB_START_RGGB
	BAYER_RGBIR_4x4_START_GIRRG = 7,//BAYER_RGGB_START_GRBG

	BAYER_RGBIR_4x4_START_BUTT
} bayer_rgbir_4x4_start_e;

/**
 * @enum enum_bayer_pattern_e
 * sensor device param of bayer pattern enum
 * @NO{S10E02C05}
 */
typedef enum enum_bayer_pattern_e
{
	BAYER_PATTERN_RGGB      = 0,
	BAYER_PATTERN_RCCC      = 1,
	BAYER_PATTERN_RCCB      = 2,
	BAYER_PATTERN_RCCG      = 3,
	BAYER_PATTERN_CCCC      = 4,
	BAYER_PATTERN_RGBIR_2X2 = 5,
	BAYER_PATTERN_GRBIR_4X4 = 6,
	BAYER_PATTERN_RYYCy     = 7,
	BAYER_PATTERN_RYYB      = 8,
	BAYER_PATTERN_RGBW      = 9,

	BAYER_PATTERN_BUTT
} bayer_pattern_e;

/**
 * @struct sensor_data
 * sensor device base param struct
 * @NO{S10E02C05}
 */
typedef struct sensor_data {
	uint32_t  turning_type;  //  1:imx290 2: ar0233
	uint32_t  step_gain;
	uint32_t  again_prec;
	uint32_t  dgain_prec;
	uint32_t  conversion;
	uint32_t  VMAX;
	uint32_t  HMAX;
	uint32_t  FSC_DOL2;
	uint32_t  FSC_DOL3;
	uint32_t  RHS1;
	uint32_t  RHS2;
	uint32_t  lane;
	uint32_t  clk;
	uint32_t  fps;
	uint32_t  gain_max;
	uint32_t  lines_per_second;
	uint32_t  analog_gain_max;
	uint32_t  digital_gain_max;
	uint32_t  exposure_time_max;
	uint32_t  exposure_time_min;
	uint32_t  exposure_time_long_max;
	uint32_t  active_width;
	uint32_t  active_height;
#ifndef COMP_XJ3_CAM
	uint32_t  data_width;       // Bits per pixel.
	uint32_t  bayer_start;      // RGGB pattern start (R/Gr/Gb/B).
	uint32_t  bayer_pattern;    // CFA pattern type (RGGB/RCCC/RIrGB/RGIrB).
	uint32_t  exposure_max_bit_width; // pwl mode Bits
	uint32_t  exposure_time_step;
#endif
}sensor_data_t;

#ifndef COMP_XJ3_CAM
static void inline sensor_data_bayer_fill(sensor_data_t *sd,
	uint32_t data_width, uint32_t bayer_start, uint32_t bayer_pattern) {
	sd->data_width = data_width;
	sd->bayer_start = bayer_start;
	sd->bayer_pattern = bayer_pattern;
}
static void inline sensor_data_bits_fill(sensor_data_t *sd, uint32_t exposure_max_bit_width) {
	sd->exposure_max_bit_width = exposure_max_bit_width;
}
#else
static void inline sensor_data_bayer_fill(sensor_data_t *sd,
	uint32_t data_width, uint32_t bayer_start, uint32_t bayer_pattern) {
	return;
}
static void inline sensor_data_bits_fill(sensor_data_t *sd, uint32_t exposure_max_bit_width) {
	return;
}
#endif

/* line use y = ratio * x + offset;
 * input param:
 * ratio(0,1) : 0: -1, 1: 1
 * offset:
 * max:
 */

typedef struct ctrlp_s {
	int32_t ratio;
	uint32_t offset;
	uint32_t max;
	uint32_t min;
} ctrlp_t;

/*
 * distinguish between dgain and again
 * note1: a sensor could only have again or dgain
 * note2: some sensor again/dgain in the same register, could only use again
 *        eg. imx327
 * note3: some sensor the again is stepped, could noly use again.
 * note4: again/dgain could have multiple registers,
 * note5: again [0,255], actual = 2^(again/32)
 * note6: dgain [0,255], actual = 2^(dgain/32)
 * note7: dol2/dol3/dol4 used the same again/dgain
 */

/**
 * @struct dol3_s
 * sensor device tuning ae dol3 param struct
 * @NO{S10E02C05}
 */
typedef struct dol3_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[3];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t l_line;
	uint32_t l_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
} dol3_t;

/**
 * @struct dol2_s
 * sensor device tuning ae dol2 param struct
 * @NO{S10E02C05}
 */
typedef struct dol2_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p[2];
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t m_line;
	uint32_t m_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}dol2_t;

/**
 * @struct normal_s
 * sensor device tuning ae normal param struct
 * @NO{S10E02C05}
 */
typedef struct normal_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	ctrlp_t  line_p;
	uint32_t s_line;
	uint32_t s_line_length;
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}normal_t;

/**
 * @struct pwl_s
 * sensor device tuning ae pwl param struct
 * @NO{S10E02C05}
 */
typedef struct pwl_s {
	uint32_t param_hold;
	uint32_t param_hold_length;
	uint32_t l_s_mode; //0 not use, 1 long; 2 short
	uint32_t line_num;
	ctrlp_t  line_p;
	ctrlp_t  line_p_ext[4];
	uint32_t line;
	uint32_t line_ext[4];
	uint32_t line_length;
	uint32_t line_length_ext[4];
	uint32_t again_control_num;
	uint32_t again_control[4];
	uint32_t again_control_length[4];
	uint32_t dgain_control_num;
	uint32_t dgain_control[4];
	uint32_t dgain_control_length[4];
	uint32_t *again_lut;
	uint32_t *dgain_lut;
}pwl_t;

/**
 * @struct stream_ctrl_s
 * sensor device stream ctrl param struct
 * @NO{S10E02C05}
 */
typedef struct stream_ctrl_s {
	uint32_t stream_on[10];
	uint32_t stream_off[10];
	uint32_t data_length;
}stream_ctrl_t;

/**
 * @struct sensor_awb_ctrl_s
 * sensor device tuning awb ctrl param struct
 * @NO{S10E02C05}
 */
typedef struct sensor_awb_ctrl_s {
	uint32_t rgain_addr[4];
	uint32_t rgain_length[4];
	uint32_t bgain_addr[4];
	uint32_t bgain_length[4];
	uint32_t grgain_addr[4];
	uint32_t grgain_length[4];
	uint32_t gbgain_addr[4];
	uint32_t gbgain_length[4];
	uint32_t rb_prec;
	uint32_t apply_lut_gain;
} sensor_awb_ctrl_t;

/**
 * camera user parameter info state
 */
enum cam_usr_info_state {
	CAM_SP_STATE_UNKNOWN,
	CAM_SP_STATE_SUCCESS,
	CAM_SP_STATE_FAILED,
	CAM_SP_STATE_ERROR,
	CAM_SP_STATE_NOT_SUPPROT,
	CAM_SP_STATEA_MAX,
};

/**
 * camera parameter state name string
 */
#define CAM_SP_STATE_NAMES { \
	"UNKNOWN", \
	"SUCCESS", \
	"FAILED", \
	"ERROR", \
	"NOT_SUPPORT", \
}

/**
 * @struct cam_usr_info_s
 * camera parameter struct info for usr
 */
typedef struct cam_usr_info_s {
	int32_t state;
	cam_parameter_t iparam;
} cam_usr_info_t;

enum led_type {
	INVALID,
	ON_OFF,
	LINEAR,
	LUT,
	LED_TYPE_MAX
};

/**
 * @struct sensor_led_s
 * sensor device tuning led info
 * @NO{S10E02C05}
 */
typedef struct sensor_led_s {
	uint32_t type; //led type : 0 invalid ; 1 on/off; 2 linear; 3 lut
	uint32_t pos_value;
	uint32_t neg_value;
	uint32_t reg_addr; // linear ratio
	uint32_t *led_lut;
} sensor_led_t;

/**
 * @struct sensor_tuning_data
 * sensor device tuning param struct
 * @NO{S10E02C05}
 */
typedef struct sensor_tuning_data {
	uint32_t  port;
	char      sensor_name[CAMERA_SENSOR_NAME];
	uint32_t  sensor_addr;
	uint32_t  bus_num;
	uint32_t  bus_type;
	uint32_t  reg_width;
	uint32_t  chip_id;
	uint32_t  mode;
	uint32_t  cs;
	uint32_t  spi_mode;
	uint32_t  spi_speed;
	normal_t normal;
	dol2_t   dol2;
	dol3_t   dol3;
	pwl_t    pwl;
	sensor_awb_ctrl_t sensor_awb;
	stream_ctrl_t stream_ctrl;
	sensor_data_t sensor_data;
	sensor_led_t led_info;
}sensor_tuning_data_t;

/**
 * @struct sensor_event_type_e
 * sensor device user event type enum
 * @NO{S10E02C05}
 */
typedef enum sensor_event_type_e {
	SEN_EVENT_TYPE_INVALID = 0,
	SEN_EVENT_TYPE_STREAM,
	SEN_EVENT_TYPE_MAX,
} sensor_event_type_t;

/**
 * @struct sensor_event_info_s
 * sensor device event info struct
 * @NO{S10E02C05}
 */
typedef struct sensor_event_info_s {
	int32_t type;
	int32_t data;
	int32_t reserved[2];
} sensor_event_info_t;

/**
 * @struct sensor_input_param_s
 * sensor device input info param struct
 * @NO{S10E02C05}
 */
typedef struct sensor_input_param_s {
	int32_t  ts_compensate;
	int32_t  reserved[3];
} sensor_input_param_t;

/* funcs call for fill */
typedef int32_t (*sensor_data_fill)(sensor_info_t *sen_if, sensor_data_t *sensor_data);
typedef int32_t (*sensor_awb_fill)(sensor_info_t *sen_if, sensor_awb_ctrl_t *sensor_awb);
typedef int32_t (*sensor_ae_fill)(sensor_info_t *sen_if, void *sensor_ae);
typedef int32_t (*sensor_ctrl_fill)(sensor_info_t *sen_if, stream_ctrl_t *stream_ctrl);

/* internal dev apis */
extern int32_t camera_sensor_dev_open(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_nodrv(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_close(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_tuning_init(sensor_info_t *sen_if, sensor_tuning_data_t *pdata);
extern int32_t camera_sensor_dev_tuning_fill_init(sensor_info_t *sen_if, sensor_data_fill snsf, sensor_awb_fill awbf, sensor_ae_fill aef, sensor_ctrl_fill ctlf);
extern int32_t camera_sensor_dev_ae_share(sensor_info_t *sen_if, uint32_t ae_share_flag);
extern int32_t camera_sensor_dev_input_param(sensor_info_t *sen_if, sensor_input_param_t *input_param);
extern int32_t camera_sensor_dev_set_intrinsic_param(sensor_info_t *sen_if,  cam_usr_info_t *sp);
extern int32_t camera_sensor_dev_get_intrinsic_param(sensor_info_t *sen_if,  cam_usr_info_t *sp);
extern int32_t camera_sensor_dev_init_req(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_init_result(sensor_info_t *sen_if, int32_t result);
extern int32_t camera_sensor_dev_deinit(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_start(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_stop(sensor_info_t *sen_if);
extern int32_t camera_sensor_dev_event_get(sensor_info_t *sen_if, sensor_event_info_t *event);
extern int32_t camera_sensor_dev_event_put(sensor_info_t *sen_if, int32_t result);
extern int32_t camera_sensor_dev_update_ae_info(sensor_info_t *sen_if, camera_ae_info_t *ae_info);
extern int32_t camera_sensor_dev_get_version(sensor_info_t *sen_if, sensor_version_info_t *ver);

/**
 * sensor ctrl driver
 */
#define SENSOR_CTRL_VER_MAJOR     (1u)
#define SENSOR_CTRL_VER_MINOR     (0u)

/**
 * @struct sensor_ctrl_info_s
 * sensor ctrl device info param struct
 * @NO{S10E02C05}
 */
typedef struct sensor_ctrl_info_s {
	uint32_t port;
	uint32_t gain_num;
	uint32_t gain_buf[4];
	uint32_t dgain_num;
	uint32_t dgain_buf[4];
	uint32_t en_dgain;
	uint32_t line_num;
	uint32_t line_buf[4];
	uint32_t rgain;
	uint32_t bgain;
	uint32_t grgain;
	uint32_t gbgain;
	uint32_t af_pos;
	uint32_t zoom_pos;
	uint32_t mode;
	uint32_t color_temper;
	uint32_t id;
	uint32_t reserverd[7];
} sensor_ctrl_info_t;

/**
 * @struct sensor_ctrl_result_s
 * camera sensor control result struct
 * @NO{S10E02C05}
 */
typedef struct sensor_ctrl_result_s {
        uint32_t port;
        uint32_t id;
        uint32_t ops;
        int32_t result;
} sensor_ctrl_result_t;

/* internal cdev apis */
extern int32_t camera_sensor_cdev_open(sensor_info_t *sen_if);
extern int32_t camera_sensor_cdev_close(sensor_info_t *sen_if);
extern int32_t camera_sensor_cdev_info_sync(sensor_info_t *sen_if, sensor_ctrl_info_t *info);
extern int32_t camera_sensor_cdev_result(sensor_info_t *sen_if, sensor_ctrl_result_t *res);
extern int32_t camera_sensor_cdev_get_version(sensor_info_t *sen_if, sensor_version_info_t *ver);

/**
 * sensor iq driver
 */
#define SENSOR_IQ_VER_MAJOR     (1u)
#define SENSOR_IQ_VER_MINOR     (0u)

#define CALIB_NUM_LENGTH    CALIBRATION_NAME_SIZE
#define CALIBRATION_MAX_ID  CALIBRATION_MULTI_NUM

/**
 * @struct camera_calib_s
 * sensor iq calib device param struct
 * @NO{S10E02C05}
 */
typedef struct camera_calib_s {
	char name[CALIB_NUM_LENGTH];
	uint32_t port;
	uint32_t total_calib;
	uint32_t calib_total_size[CALIBRATION_MAX_ID];
	uint32_t calib_mem_size[CALIBRATION_MAX_ID];
	void *plut[CALIBRATION_MAX_ID];
} camera_calib_t;

/* internale idev apis */
extern int32_t camera_sensor_idev_open(calib_info_t *cal_if);
extern int32_t camera_sensor_idev_close(calib_info_t *cal_if);
extern int32_t camera_sensor_idev_totalsize(calib_info_t *cal_if);
extern int32_t camera_sensor_idev_init(calib_info_t *cal_if, camera_calib_t *pcalib);
extern int32_t camera_sensor_idev_deinit(calib_info_t *cal_if, camera_calib_t *pcalib);
extern int32_t camera_sensor_idev_get_version(calib_info_t *cal_if, sensor_version_info_t *ver);
extern int32_t camera_sensor_isi_dev_open(camera_module_lib_t *cal_lib);
extern int32_t camera_sensor_isi_dev_data_put(camera_module_lib_t *cal_lib, camera_calib_t *pcalib);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_SENSOR_DEV_H__ */

