/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_common.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_SENSOR_COMMON_H__
#define __CAMERA_SENSOR_COMMON_H__

#include <stdint.h>

#include "camera_mod_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FLAG_NOT_FIND 2

#define SENSOR_PARAM_LEN_MAX	512

enum JSON_TYPE {
	ISINT = 0,
	ISDOUBLE,
	ISSTRING,
};

/* format for v0.1 */
#define EEPROM_I2C_ADDR_ALIAS_ID (0x51)
#define DEFAULT_EEPROM_I2C_ADDR  (0x50)
#define MAJOR_VERSION_ADDR      (0x0000)
#define MINOR_VERSION_ADDR      (0x0001)
#define VENDOR_ID_ADDR          (0x0004)
	#define WITHOUT_CRYSTAL     (0x5A4C)
#define MODULE_ID_ADDR          (0x0006)
#define MODULE_SERIAL_ADDR      (0x0008)
#define YEAR_ADDR               (0x000C)
#define MONTH_ADDR              (0x000E)
#define DAY_ADDR                (0x000F)
#define CAM_TYPE_ADDR           (0x0010)
#define MODULE_FLAG_ADDR        (0x0016)
#define EFL_FLAG_ADDR           (0x0017)
#define COD_FLAG_ADDR           (0x0018)
#define PP_FLAG_ADDR            (0x0019)
#define DISTORTION_FLAG_ADDR    (0x001A)
#define IMG_HEIGHT_ADDR         (0x001C)
#define IMG_WIDTH_ADDR          (0x001E)
#define FOV_ADDR                (0x0020)
#define CRC32_1_ADDR            (0x003C)
#define EFL_X_ADDR              (0x0040)
#define EFL_Y_ADDR              (0x0048)
#define COD_X_ADDR              (0x0050)
#define COD_Y_ADDR              (0x0058)
#define PP_X_ADDR               (0x0080)
#define PP_Y_ADDR               (0x0088)
#define DISTORT_PARAMS_ADDR     (0x0090)
#define DISTORT_MODEL_TYPE_ADDR (0x0091)
#define CAM_SKEW_ADDR           (0x0098)
#define K1_ADDR                 (0x00C0)
#define K2_ADDR                 (0x00C8)
#define P1_ADDR                 (0x00D0)
#define P2_ADDR                 (0x00D8)
#define K3_ADDR                 (0x00E0)
#define K4_ADDR                 (0x00E8)
#define K5_ADDR                 (0x00F0)
#define K6_ADDR                 (0x00F8)
#define K7_ADDR                 (0x0100)
#define K8_ADDR                 (0x0108)
#define K9_ADDR                 (0x0110)
#define K10_ADDR                (0x0118)
#define K11_ADDR                (0x0120)
#define K12_ADDR                (0x0128)
#define K13_ADDR                (0x0130)
#define K14_ADDR                (0x0138)
#define EFL_X_2_ADDR            (0x0148)
#define EFL_Y_2_ADDR            (0x0150)
#define COD_X_2_ADDR            (0x0158)
#define COD_Y_2_ADDR            (0x0160)
#define K1_2_ADDR               (0x0168)
#define K2_2_ADDR               (0x0170)
#define K3_2_ADDR               (0x0178)
#define K4_2_ADDR               (0x0180)
#define SERIAL_NUM_ADDR         (0x0188)
#define GALAXY_CRC32_GROUP1_ADDR  (0x01C0)
#define SD_CRC32_GROUP1_ADDR    (0x0140)
#define CRC32_GROUP1_ADDR       (0x01C0)
#define GALAXY_PARAMS             BIT(5)  // get galaxy params
#define PARAMS_2                  BIT(6)  // get lce params

/*lce params addr*/
#define IMG_HEIGHT_ADDR_2         (0x001A)
#define IMG_WIDTH_ADDR_2          (0x0018)
#define MAJOR_VERSION_ADDR_2      (0x003C)
#define MINOR_VERSION_ADDR_2      (0x003D)
#define VENDOR_ID_ADDR_2          (0x0005)
#define MODULE_SERIAL_ADDR_2      (0x001C)
#define CAM_TYPE_ADDR_2           (0x0009)
#define EFL_X_ADDR_2              (0x008D)
#define EFL_Y_ADDR_2              (0x0095)
#define COD_X_ADDR_2              (0x0065)
#define COD_Y_ADDR_2              (0x006D)
#define FOV_ADDR_2                (0x0010)  // 4
#define K1_ADDR_2                 (0x009D)
#define K2_ADDR_2                 (0x00A5)
#define P1_ADDR_2                 (0x00CD)
#define P2_ADDR_2                 (0x00D5)
#define K3_ADDR_2                 (0x00AD)
#define K4_ADDR_2                 (0x00B5)
#define K5_ADDR_2                 (0x00BD)
#define K6_ADDR_2                 (0x00C5)

/* white balance color ratio addr */
#define GOLDEN_D65_LCG_COLOR_RATIO_RG      (0x439)
#define D65_LCG_COLOR_RATIO_RG             (0x451)
#define COLOR_RATIO_CHECKSUM               (0x469)
/* v2 only for LCE */
#define GOLDEN_D65_LCG_COLOR_RATIO_RG_V2   (0x12B)
#define D65_LCG_COLOR_RATIO_RG_V2          (0x143)
#define COLOR_RATIO_CHECKSUM_V2            (0x15B)

#define COLOR_RATIO_NUM                    (12)


extern int32_t camera_sensor_param_parse(sensor_info_t *sen_if,
		char *field_name, int32_t datatype, void *data);
extern int32_t camera_sensor_emode_string_parse(const char *str, const char flag);
extern int32_t camera_sensor_emode_parse(sensor_info_t *sen_if, const char flag);
extern uint32_t camera_sensor_port_mask(sensor_info_t *sen_if);
extern int32_t camera_sensor_emode_datatype_hex(int32_t type);
extern int32_t camera_sensor_param_get(sensor_info_t *sen_if, cam_parameter_t *csp);
extern int32_t camera_sensor_config_do(sensor_info_t *sen_if, int32_t mask, sensor_config_func *funcs);
extern void camera_sensor_lut_byte_swap(uint32_t *x, uint32_t n);

#define camera_sensor_string_datatype_parse(str) \
		camera_sensor_emode_datatype_hex(camera_sensor_emode_string_parse(str, EMODE_F_SEN_DATATYPE))
#define camera_sensor_emode_datatype_parse(sen_if) \
		camera_sensor_emode_datatype_hex(camera_sensor_emode_parse(sen_if, EMODE_F_SEN_DATATYPE))

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_SENSOR_COMMON_H__ */

