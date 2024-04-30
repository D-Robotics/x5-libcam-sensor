/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_common.c
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#define pr_mod	"sensor_comm"

#include <stdio.h>
#include <string.h>

#include "hb_camera_error.h"

#include "camera_mod_sensor.h"
#include "camera_sensor_common.h"
#include "camera_log.h"

#include "cam_data_info.h"

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor private json config field parse
 *
 * @param[in] sen_if: sensor info struct
 * @param[in] field_name: the config field name string
 * @param[in] datatype: the config datatype to parse
 * @param[out] data: the parsed date return
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
#ifdef HOBOT_MCU_CAMSYS
int32_t camera_sensor_param_parse(sensor_info_t *sen_if,
		char *field_name, int32_t datatype, void *data)
{
	return -1;
}

#else /* HOBOT_MCU_CAMSYS */
#include "camera_json.h"
int32_t camera_sensor_param_parse(sensor_info_t *sen_if,
		char *field_name, int32_t datatype, void *data)
{
	int32_t ret;
	if (sen_if == NULL) {
		cam_err("sen_if is NULL\n");
		return -1;
	}
	if (sen_if->sensor_param_root == NULL)
		return -1;

	cam_dbg("sensor%d %s sensor_param_root valid parse %s\n",
		sen_if->port, sen_if->sensor_name, field_name);
	ret = camera_json_string_parse((const char *)sen_if->sensor_param_root,
				field_name, datatype, data);
	return ret;
}
#endif /* HOBOT_MCU_CAMSYS */

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor emode name string attr parse with flag char
 *
 * @param[in] str: the emode name string to parse
 * @param[in] flag: the flag char to parse
 *
 * @return 0:Success-parsed value, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_emode_string_parse(const char *str, const char flag)
{
	const char *s = NULL;
	int32_t v = 0, f = 0, ret = RET_OK;

	if (str == NULL) {
		cam_err("invalid args, str is NULL\n");
		return -1;
	 }
	s = str;
	while (*s) {
		if (v == 0) {
			if (*s == '_')
				v = 1;
		} else if (*s >= '0' && *s <= '9') {
			if (f)
				ret = (ret * 10) + (*s - '0');
		} else if (f) {
			break;
		} else if (*s == flag) {
			f = 1;
		}
		s++;
	}
	return (f) ? ret : -FLAG_NOT_FIND;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor emode parse by sen_if
 *
 * @param[in] sen_if: the sensor info struct to parse
 * @param[in] flag: the flag char to parse
 *
 * @return 0:Success-parsed value, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_emode_parse(sensor_info_t *sen_if, const char flag)
{
	int32_t ret = RET_OK;
	const char *sensor_emode_name;

	if ((sen_if == NULL) || (flag == 0)) {
		cam_err("invalid args\n");
		return -1;
	}
	sensor_emode_name = SENSOR_EMODE_NAME(sen_if);
	if (sensor_emode_name == NULL) {
		cam_warn("sensor %s extra_mode %d emode name is NULL\n",
			sen_if->sensor_name, sen_if->extra_mode);
		return -1;
	}
	ret = camera_sensor_emode_string_parse(sensor_emode_name, flag);
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor port mask get by sen_if
 *
 * @param[in] sen_if: the sensor info struct to parse
 *
 * @return 0:error, !0:bit mask of sensor port index
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
uint32_t camera_sensor_port_mask(sensor_info_t *sen_if)
{
	return (uint32_t)((sen_if != NULL) ? BIT(sen_if->dev_port) : 0U);
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor emode parse datatype as hex format
 *
 * @param[in] sen_if: the sensor info struct to parse
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_emode_datatype_hex(int32_t type)
{
	int32_t ret;
	switch (type) {
	case EMODE_YUV422:
		ret = 0x1E;
		break;
	case EMODE_RAW8:
		ret = 0x2A;
		break;
	case EMODE_RAW10:
		ret = 0x2B;
		break;
	case EMODE_RAW12:
		ret = 0x2C;
		break;
	default:
		ret = -RET_ERROR;
		break;
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor params get from camera module
 *
 * @param[in] sen_if: the sensor info struct to parse
 * @param[out] csp: camera sensor params struct to store
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_param_get(sensor_info_t *sen_if, cam_parameter_t *csp)
{
	/* sen_if & emode to cam_parameter_t */
	int32_t fov = 0, bayer_pattern = 0;
	int32_t ret = RET_OK;
	const char *emode_name = NULL;
	const char *calb_name = NULL;
	uint8_t i = 0;

	emode_name = SENSOR_EMODE_NAME(sen_if);
	calb_name = SENSOR_EMODE_CALIB_LNAME(sen_if);
	if ((sen_if == NULL) || (emode_name == NULL) || calb_name == NULL) {
		return -RET_ERROR;
	}

	while ((i < LEN_VENDOR_STR) &&
			emode_name[i] != '\0' &&
			emode_name[i] != '_') {
		csp->sns_param.vendor[i] = emode_name[i];
		i++;
	}

	strncpy(csp->sns_param.sensor_name, sen_if->sensor_name,
			sizeof(csp->sns_param.sensor_name) - 1);

	strncpy(csp->sns_param.calb_name, calb_name, sizeof(csp->sns_param.calb_name) - 1);

	fov = camera_sensor_emode_string_parse(emode_name, 'F');
	if (fov > 0) {
		snprintf(csp->sns_param.fov, LEN_FOV_STR, "%d", (uint16_t)fov);
	} else {
		ret |= fov;
	}

	char bayer_pattern_name[6][6] = {
		"RGGB",
		"RCCC",
		"RIRGB",
		"RGIRB",
		"RCCB",
		"RYYCY",
	};

	bayer_pattern = camera_sensor_emode_string_parse(emode_name, 'P');
	if (bayer_pattern >= 0) {
		strncpy(csp->sns_param.bayer_pattern, bayer_pattern_name[bayer_pattern],
				sizeof(csp->sns_param.bayer_pattern) - 1);
	} else {
		/* default RGGB */
		strncpy(csp->sns_param.bayer_pattern, bayer_pattern_name[0],
				sizeof(csp->sns_param.bayer_pattern) - 1);
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor config_index bit func call by mask
 *
 * @param[in] sen_if: the sensor info struct to used
 * @param[in] mask: the bit mask to enable call
 * @param[in] funcs: then func array to be called
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_config_do(sensor_info_t *sen_if, int32_t mask, sensor_config_func *funcs)
{
	int32_t i, ret = 0;
	int32_t config_mask = mask;
	const char *config_name[B_CONFIG_INDEX_MAX] = CONFIG_INDEX_BNAME;
	if ((sen_if == NULL) || (mask == 0) || (funcs == NULL))
		return -1;
	for (i = 0; i < B_CONFIG_INDEX_MAX; i++) {
		if ((((mask & BIT(i)) != 0) && SENSOR_CONFIG_ISEN(sen_if, BIT(i)))) {
			if (funcs[i] != NULL) {
				ret = funcs[i](sen_if);
			} else {
				cam_err("config_index[%d]: %s not support\n", i, config_name[i]);
				ret = -1;
			}
		}
		config_mask &= ~BIT(i);
		if ((ret < 0) || (config_mask == 0))
			break;
	}
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor lut table value byte swap
 *
 * @param[in] x: the value pointer to operation
 * @param[in] n: the whole byte number of value
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void camera_sensor_lut_byte_swap(uint32_t *x, uint32_t n)
{
	switch (n) {
	case 2:
	*x = ((((*x) & 0xff00) >> 8) | (((*x) & 0xff) << 8));
	break;
	case 3:
	*x = (((*x) & 0x0000ff00) + (((*x) & 0xff0000) >> 16) + (((*x) & 0xff) << 16));
	break;
	case 4:
	*x = ((((((*x) & 0xff000000) >> 8) + (((*x) & 0xff0000) << 8)) >> 16) |
		(((((*x) & 0xff00) >> 8) + (((*x) & 0xff) << 8)) << 16));
	break;
	}
}

