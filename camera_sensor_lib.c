/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_lib.c
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#define pr_mod	"sensor_lib"

#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "hb_camera_error.h"

#include "camera_log.h"
#include "camera_env.h"
#include "camera_i2c.h"
#include "camera_gpio.h"
#include "camera_sys.h"
#include "camera_sensor_dev.h"
#include "camera_sensor_common.h"

#include "cam_runtime.h"
#include "cam_module.h"
#include "cam_sensor_lib.h"
#include "cam_calib_lib.h"
#include "cam_vpf.h"
#include "cam_debug.h"

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get sensor extra_mode max config by lib module
 *
 * @param[in] module: the moudle struct of sensor lib
 *
 * @return >=0:Success-the max of extra_mode, <0:Failure-not limit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_sensor_extra_mode_max(const camera_module_t *module)
{
	int32_t i = 0;
	sensor_emode_type_t *emode;

	if (!CAMERA_MODULE_CHECK_VALID(module))
		return -RET_ERROR;
	emode = SENSOR_MODULE_GET_EMODE(module);
	if (emode == NULL)
		return -RET_ERROR;
	while (emode[i].name != NULL) {
		i++;
	}

	return i;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get sensor config_index bit mask by lib module
 *
 * @param[in] module: the module struct of sensor lib
 *
 * @return >=0:Success-the valid bit mask of config_index, <0:Failure-not limit
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_sensor_config_index_mask(const camera_module_t *module)
{
	int32_t i, mask = 0;
	sensor_config_func *cfuncs;

	if (!CAMERA_MODULE_CHECK_VALID(module))
		return -RET_ERROR;
	cfuncs = SENSOR_MODULE_GET_CFUNCS(module);
	if (cfuncs == NULL)
		return -RET_ERROR;
	for (i = 0; i < B_CONFIG_INDEX_MAX; i++) {
		if (cfuncs[i] != NULL)
			mask |= BIT(i);
	}

	return mask;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief check the sensor ko_version valid by lib module
 *
 * @param[in] module: the module struct of sensor lib
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
static int32_t camera_sensor_ko_version_check(const camera_module_t *module)
{
	int32_t ret = 0;
	const char *so_ver;
	camera_ko_version_t *ko_ver;

	if (!CAMERA_MODULE_CHECK_VALID(module))
		return 0;
	so_ver = CAMERA_MODULE_GET_VERSION(module);
	if (so_ver == NULL)
		so_ver = "unknown";
	ko_ver = CAMERA_MODULE_GET_KO_VERSION(module);
	if (ko_ver == NULL)
		return 0;
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		/* ko(of lib) version is valid? */
		if ((ko_ver->major == 0U) && (ko_ver->minor == 0U)) {
			cam_dbg("sensor %s v%s ko_ver skip check\n", module->name, so_ver);
			return 0;
		}
		/* lib driver version should >= ko(of lib) version */
		if ((SENSOR_VER_MAJOR < ko_ver->major) ||
			((SENSOR_VER_MAJOR == ko_ver->major) && (SENSOR_VER_MINOR < ko_ver->minor))) {
			cam_err("check %s v%s ko v%u.%u > v%u.%u error\n", module->name, so_ver,
				ko_ver->major, ko_ver->minor, SENSOR_VER_MAJOR, SENSOR_VER_MINOR);
			ret = -RET_ERROR;
		} else {
			cam_dbg("sensor %s v%s ko v%u.%u\n", module->name, so_ver,
				ko_ver->major, ko_ver->minor);
		}
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor config addr check is valid?
 *
 * @param[in] so_name: so name for error info show
 * @param[in] name: addr config name for error info show
 * @param[in] addr: addr config value to check
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
static int32_t camera_sensor_config_addr_check(char *so_name, const char *name, int32_t addr)
{
	if (!CAM_CONFIG_CHECK_RANGE(addr, CAM_SENSOR_CHECK_ADDR_MIN, CAM_SENSOR_CHECK_ADDR_MAX) &&
	    (addr != CAM_SENSOR_CHECK_ADDR_EXCEPT)) {
		cam_err("sensor %s check config %s 0x%x not in range [0x%x, 0x%x]/0x%x error\n",
			so_name, name, addr, CAM_SENSOR_CHECK_ADDR_MIN, CAM_SENSOR_CHECK_ADDR_MAX,
			CAM_SENSOR_CHECK_ADDR_EXCEPT);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor config value range check is valid?
 *
 * @param[in] so_name: so name for error info show
 * @param[in] name: config name for error info show
 * @param[in] val: config value to check
 * @param[in] min: config value min for check
 * @param[in] max: config value max for check
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
static int32_t camera_sensor_config_range_check(char *so_name, const char *name, int32_t value, int32_t min, int32_t max)
{
	if (!CAM_CONFIG_CHECK_RANGE(value, min, max)) {
		cam_err("sensor %s check config %s 0x%x not in range [0x%x, 0x%x] error\n",
			so_name, name,  value, min, max);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor module check cam_config if valid?
 *
 * @param[in] lib: the sensor module lib struct to used
 * @param[in] cam_config: the camera config stuct to check
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
int32_t camera_sensor_config_check(camera_module_lib_t *lib, camera_config_t *cam_config)
{
	int32_t ret = RET_OK;
	int32_t error = 0;
	int32_t extra_mode_max, config_index_mask;

	if ((lib == NULL) || (cam_config == NULL))
		return -RET_ERROR;

	/* no check? */
	if (camera_env_get_bool(CAMENV_CONFIG_NOCHECK, FALSE) == TRUE)
		return RET_OK;
	cam_dbg("sensor %s config check\n", cam_config->name);

	error += camera_sensor_config_addr_check(lib->so_name, "addr", cam_config->addr);
	error += camera_sensor_config_addr_check(lib->so_name, "isp_addr", cam_config->isp_addr);
	error += camera_sensor_config_addr_check(lib->so_name, "eeprom_addr", cam_config->eeprom_addr);
	error += camera_sensor_config_addr_check(lib->so_name, "serial_addr", cam_config->serial_addr);
	error += camera_sensor_config_range_check(lib->so_name, "sensor_mode", cam_config->sensor_mode,
				CAM_SENSOR_CHECK_SMODE_MIN, CAM_SENSOR_CHECK_SMODE_MAX);
	error += camera_sensor_config_range_check(lib->so_name, "fps", cam_config->fps,
				CAM_SENSOR_CHECK_FPS_MIN, CAM_SENSOR_CHECK_FPS_MAX);
	error += camera_sensor_config_range_check(lib->so_name, "width", cam_config->width,
				CAM_SENSOR_CHECK_WIDTH_MIN, CAM_SENSOR_CHECK_WIDTH_MAX);
	error += camera_sensor_config_range_check(lib->so_name, "height", cam_config->height,
				CAM_SENSOR_CHECK_HEIGHT_MIN, CAM_SENSOR_CHECK_HEIGHT_MAX);
	extra_mode_max = camera_sensor_extra_mode_max(lib->module);
	if (extra_mode_max >= 0) {
		error += camera_sensor_config_range_check(lib->so_name, "extra_mode", cam_config->extra_mode,
				0, extra_mode_max);
	}
	config_index_mask = camera_sensor_config_index_mask(lib->module);
	if ((config_index_mask >= 0) && ((cam_config->config_index & (~config_index_mask)) != 0)) {
		cam_err("sensor %s check config %s 0x%x not support bit 0x%x error\n",
			lib->so_name, "config_index",  cam_config->config_index,
			(cam_config->config_index & (~config_index_mask)));
		error += -RET_ERROR;
	}
	error += camera_sensor_ko_version_check(lib->module);

	if (error != 0) {
		cam_err("sensor %s check config has %d error\n", lib->so_name, -error);
		ret = -RET_ERROR;
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get the sensor emode name from camera handle config
 *
 * @param[in] hcam: the camera hanlde struct
 *
 * @return !NULL:the sensor emod name, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static const sensor_emode_type_t *camera_sensor_config_emode(camera_handle_st *hcam)
{
	sensor_emode_type_t *emode;
	int32_t extra_mode_max;
	int32_t extra_mode;

	if (hcam == NULL)
		return NULL;

	emode = SENSOR_MODULE_GET_EMODE(hcam->sensor_lib.module);
	extra_mode_max = camera_sensor_extra_mode_max(hcam->sensor_lib.module);
	extra_mode = hcam->cam_config.extra_mode;
	if ((emode == NULL) || (extra_mode >= extra_mode_max))
		return NULL;

	return &emode[extra_mode];
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get the sensor emode name from camera handle config
 *
 * @param[in] hcam: the camera hanlde struct
 *
 * @return !NULL:the sensor emod name, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static const char *camera_sensor_config_emode_name(camera_handle_st *hcam)
{
	const sensor_emode_type_t *emode;

	emode = camera_sensor_config_emode(hcam);
	if (emode == NULL)
		return NULL;

	return emode->name;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get the sensor link desp for deserial
 *
 * @param[in] hcam: the camera hanlde struct
 * @param[in] desp: the desp string buffer to store
 * @param[in] size: the desp buffer size
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
int32_t camera_sensor_get_link_desp(camera_handle_st *hcam, char *desp, int32_t size)
{
	const char *emode_name;

	if ((hcam == NULL) || (desp == NULL) || (size <= 0))
		return -RET_ERROR;
	emode_name = camera_sensor_config_emode_name(hcam);
	if (emode_name != NULL)
		snprintf(desp, size, "%s:%s", hcam->cam_config.name, emode_name);
	else
		strncpy(desp, hcam->cam_config.name, size - 1);

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get the calib so name from camera handle config
 *
 * @param[in] hcam: the camera hanlde struct
 *
 * @return !NULL:the calib so name, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
const char* camera_sensor_config_calib_lname(camera_handle_st *hcam)
{
	const char *calib_lname = NULL;
	const sensor_emode_type_t *emode;

	if (hcam == NULL)
		return NULL;

	if (strlen(hcam->cam_config.calib_lname) > 0) {
		calib_lname = hcam->cam_config.calib_lname;
	} else {
		emode = camera_sensor_config_emode(hcam);
		if (emode != NULL)
			calib_lname = emode->calib_lname;
	}

	if ((calib_lname == NULL) || (strlen(calib_lname) == 0) ||
		(strcasecmp(calib_lname, CAM_SENSOR_CALIB_STRING_INVALID) == 0) ||
		(strcasecmp(calib_lname, CAM_SENSOR_CALIB_LNAME_DISABLE) == 0))
		return NULL;

	return calib_lname;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get the calib so version from camera handle config
 *
 * @param[in] hcam: the camera hanlde struct
 *
 * @return !NULL:the calib version string, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
const char* camera_sensor_config_calib_version(camera_handle_st *hcam)
{
	const char *calib_version = NULL;
	const sensor_emode_type_t *emode;

	if (hcam == NULL)
		return NULL;

	if (strlen(hcam->cam_config.calib_lname) > 0) {
		calib_version = NULL;
	} else {
		emode = camera_sensor_config_emode(hcam);
		if (emode != NULL)
			calib_version = emode->calib_version;
	}

	if ((calib_version == NULL) || (strlen(calib_version) == 0) ||
		(strcasecmp(calib_version, CAM_SENSOR_CALIB_STRING_INVALID) == 0))
		return NULL;

	return calib_version;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief check the camera handle config if calib is valid
 *
 * @param[in] hcam: the camera handle struct
 *
 * @return 1:Yes-calib valid, 0:No-calib invalid
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensorl_config_has_calib(camera_handle_st *hcam)
{
	return ((camera_sensor_config_calib_lname(hcam) != NULL) ? 1 : 0);
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief bind sen_if ops info with camera module handle
 *
 * @param[in] hcam: the camera handle struct
 * @param[out] sen_if: the sensor_info stuct
 * @param[out] cal_if: the calib_info struct if need
 * @param[in] des_if: the deserial_info to bind
 * @param[in] link: the deserial link to bind
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
int32_t camera_sensor_ops_bind(camera_handle_st *hcam, sensor_info_t *sen_if, calib_info_t *cal_if, deserial_info_t *des_if, int32_t link)
{
	int32_t ret = RET_OK;
	camera_vin_attr_t *vin;
	camera_module_lib_t *lib;
	camera_config_t *cfg;
	const char *calib_lname;

	if ((hcam == NULL) || (sen_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hcam);
	cfg = &hcam->cam_config;
	vin = &hcam->vin_attr;
	lib = &hcam->sensor_lib;

	cam_dbg("sensor %s ops bind\n", cfg->name);

	sen_if->port = VIN_ATTR_TO_CAMERA_INDEX(vin);
	if (cal_if != NULL) {
		/* bind cal_info for calib lib call */
		ret = camera_calib_ops_bind(hcam, cal_if);
		if (ret < 0) {
			calib_lname = camera_sensor_config_calib_lname(hcam);
			cam_err("sensor%d calib %s ops bind error %d\n",
				sen_if->port, (calib_lname) ? calib_lname : "invalid", ret);
			return ret;
		}
	}
	sen_if->calib_info = cal_if;

	sen_if->reg_width = CAM_MODULE_GET_FLAG_ALEN(CAMERA_MODULE_GET_FLAGS(lib->module));
	sen_if->sensor_ops = lib->body;
	sen_if->sensor_fd = lib->so_fd;

	/* sensor_name, extra_mode and config_index parse when bind */
	sen_if->sensor_name = cfg->name;
	sen_if->extra_mode = cfg->extra_mode;
	sen_if->config_index = cfg->config_index;
	/* entry_num parse here for deserial diag id get */
	sen_if->entry_num = VIN_ATTR_TO_RX_INDEX(vin);

	if ((des_if != NULL) && (link < DES_LINK_NUM_MAX)) {
		sen_if->deserial_index = VIN_ATTR_TO_DESERIAL_INDEX(vin);
		sen_if->deserial_port = hcam->deserial_link;
		sen_if->deserial_info = des_if;
		des_if->sensor_info[hcam->deserial_link] = sen_if;
	} else {
		sen_if->deserial_index = -1;
		sen_if->deserial_port = -1;
		sen_if->deserial_info = NULL;
	}

	camera_debug_hcall_ro(hcam);
	return ret;
}

int32_t camera_sensor_set_cali_name(camera_handle_st *hcam, char *sensor_name, int32_t camera_index,
					char *new_calib_lname)
{
	int32_t ret = RET_OK;
	const char *calib_lname = NULL;
	camera_calib_t pcalib = {0};
	camera_module_lib_t *cal_lib = NULL;

	if (hcam == NULL)
		return -RET_ERROR;

	if ((camera_index < 0) || (camera_index >= CAM_CONFIG_CAMERA_MAX)) {
		cam_err("camera get as %d over %d error\n",
			camera_index, CAM_CONFIG_CAMERA_MAX);
		return -RET_ERROR;
	}

	if (new_calib_lname == NULL) {
		calib_lname = camera_sensor_config_calib_lname(hcam);
		if (calib_lname == NULL) {
			cam_warn("calib_lname is null, we will try sensor_name_tuning.json.\n");
			if (sensor_name != NULL) {
				snprintf(pcalib.name, sizeof(pcalib.name), "%s_tuning.json", sensor_name);
			} else {
				cam_err("calib_lname and sensor name all null, please check your code.\n");
				return -RET_ERROR;
			}
		} else {
			strncpy(pcalib.name, calib_lname, sizeof(pcalib.name));
		}
	} else {
		strncpy(pcalib.name, new_calib_lname, sizeof(pcalib.name));
	}

	pcalib.port = camera_index;

	cal_lib = &hcam->calib_lib;

	ret = camera_calib_set_cali_name_init(cal_lib);
	if (ret < 0) {
		cam_err("camera_calib_set_cali_name_init fail, ret=%d\n", ret);
		return ret;
	}
	ret = camera_calib_set_cali_name_put(cal_lib, &pcalib);
	if (ret < 0) {
		cam_err("camera_calib_set_cali_name_put fail, ret=%d\n", ret);
		return ret;
	}

	cam_info("%s port:%d, calib_lname:%s\n", __func__, camera_index, pcalib.name);

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief parse sen_if by camera_config and vin_attr of camera handle
 *
 * @param[in] hcam: the camera handle struct
 * @param[out] sen_if: the sensor_info struct
 * @param[out] cal_if: the calib_info struct if need
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
int32_t camera_sensor_config_parse(camera_handle_st *hcam, sensor_info_t *sen_if)
{
	camera_config_t *cfg;
	camera_vin_attr_t *vin;
	vcon_attr_t *vcon;
	int32_t i, ret;
	calib_info_t *cal_if;
	const char *calib_lname;

	if ((hcam == NULL) || (sen_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hcam);
	cfg = &hcam->cam_config;
	vin = &hcam->vin_attr;
	vcon = &vin->vcon_attr;
	cam_dbg("sensor %s 0x%02x config parse\n", cfg->name, cfg->addr);

	sen_if->bus_type = I2C_BUS;
	if ((cfg->bus_select == 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_MAIN) != 0)) {
		sen_if->bus_num = vcon->bus_main;
	} else if ((cfg->bus_select != 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_SEC) != 0)) {
		sen_if->bus_num = vcon->bus_second;
	} else {
		cam_err("vcon no valid %s bus attr error\n",
			(cfg->bus_select) ? "second" : "main");
		return -RET_ERROR;
	}
	sen_if->isp_addr = cfg->isp_addr;
	sen_if->sensor_addr = cfg->addr;
	sen_if->sensor1_addr = 0;
	sen_if->serial_addr = cfg->serial_addr;
	sen_if->serial_addr1 = 0;
	sen_if->imu_addr = 0;
	sen_if->sensor_clk = cfg->sensor_clk;
	sen_if->eeprom_addr = cfg->eeprom_addr;
	sen_if->power_mode = 0;
	sen_if->sensor_mode = cfg->sensor_mode;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	sen_if->gpio_num = 0;
	if (cfg->gpio_enable_bit != 0U) {
		if ((vcon->attr_valid & VCON_ATTR_V_GPIO_OTH) == 0) {
			cam_err("vcon no valid oth 0x%x gpio attr error\n",
				cfg->gpio_enable_bit);
			return -RET_ERROR;
		}
		for (i = VGPIO_OTH_BASE; i < (VGPIO_OTH_BASE + VGPIO_OTH_NUM); i++) {
			if ((cfg->gpio_enable_bit & BIT(i - VGPIO_OTH_BASE)) == 0U)
				continue;
			if (vcon->gpios[i] != 0) {
				sen_if->gpio_pin[sen_if->gpio_num] = vcon->gpios[i];
				sen_if->gpio_level[sen_if->gpio_num] =
				    (cfg->gpio_level_bit & BIT(i - VGPIO_OTH_BASE)) ? 1 : 0;
				sen_if->gpio_num++;
			}
		}
		if (sen_if->gpio_num == 0) {
			cam_warn("vcon no such oth 0x%x gpio attr\n",
				cfg->gpio_enable_bit);
		}
	}
#endif
	sen_if->fps = cfg->fps;
	sen_if->width = cfg->width;
	sen_if->height = cfg->height;
	sen_if->format = cfg->format;
	sen_if->resolution = cfg->height;
	/* sensor_name, extra_mode and config_index parsed see bind */
	sen_if->power_delay = 0;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	sen_if->config_path = NULL;
	sen_if->data_path_info = NULL;
#endif
	/* set dev_port as -1 for no sensor driver debug */
	sen_if->dev_port = (camera_env_get_bool(CAMENV_DRIVER_NOSENSOR, FALSE)) ?  -1 : sen_if->port;
	sen_if->init_cnt = 0;
	sen_if->start_cnt = 0;
	sen_if->bus_timeout = cfg->bus_timeout;
	sen_if->ts_compensate = cfg->ts_compensate;
	if (((camera_g_cfg()->diag_disable & CAMERA_DIAG_DISABLE_SENSOR) != 0U)) {
		cam_dbg("sensor %s auto disable all diag\n", cfg->name);
		sen_if->diag_mask_disabled = 0xFFFFFFFFU;
	} else {
		sen_if->diag_mask_disabled = cfg->flags;
	}
	/* param as json string */
	sen_if->sensor_param_root = (void *)cfg->sensor_param;

	cal_if = (calib_info_t *)sen_if->calib_info;
	if (cal_if != NULL) {
		/* parse cal_info for calib lib call */
		ret = camera_calib_config_parse(hcam, cal_if);
		if (ret < 0) {
			calib_lname = camera_sensor_config_calib_lname(hcam);
			cam_err("sensor%d calib %s config parse error %d\n",
				sen_if->port, (calib_lname) ? calib_lname : "invalid", ret);
			return ret;
		}
	}

	camera_debug_hcall_ro(hcam);
	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief parse the mipi csi attr by camera_config and sensor lib runtime
 *
 * @param[out] hcam: the camera handle struct with mipi_config struct
 * @param[in] sen_if: the sensor_info struct
 * @param[out] mipi_to: the mipi config struct to store
 * @param[out] bypass_to: the mipi bypass struct to store
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
int32_t camera_sensor_csi_attr_parse(camera_handle_st *hcam, sensor_info_t *sen_if,
				mipi_config_t *mipi_to, mipi_bypass_t *bypass_to)
{
	int32_t ret;
	camera_config_t *cfg;
	csi_attr_t csi_attr = { 0 };
	csi_attr_t *csi = NULL;

	if ((hcam == NULL) || (sen_if == NULL) || (mipi_to == NULL))
		return -RET_ERROR;
	cfg = &hcam->cam_config;

	ret = camera_sensor_get_csi_attr(sen_if, &csi_attr);
	if (ret == RET_OK) {
		csi = &csi_attr;
	}

	ret = camera_vpf_mipi_config_parse(mipi_to, bypass_to, cfg->mipi_cfg,
				csi, CAMERA_VPF_MIPI_IPI_VC_DEFAULT);
	if (ret < 0) {
		cam_err("camera%d %s csi attr parse error %d\n",
			hcam->vin_attr.flow_id, hcam->cam_config.name, ret);
		return ret;
	}

	if (mipi_to->rx_enable != 0)
		cam_dbg("camera%d %s csi attr rx parse: %ulane %uMbps 0x%02x\n",
			hcam->vin_attr.flow_id, hcam->cam_config.name,
			mipi_to->rx_attr.lane, mipi_to->rx_attr.mipiclk, mipi_to->rx_attr.datatype);
	if (mipi_to->bypass != NULL)
		cam_dbg("camera%d %s csi attr tx%d parse: %ulane %uMbps 0x%02x\n",
			hcam->vin_attr.flow_id, hcam->cam_config.name,
			mipi_to->bypass->tx_index, mipi_to->bypass->tx_attr.lane,
			mipi_to->bypass->tx_attr.mipiclk, mipi_to->bypass->tx_attr.datatype);
	if ((mipi_to->rx_enable == 0) && (mipi_to->bypass == NULL))
		cam_info("camera%d %s csi not enable\n",
			 hcam->vin_attr.flow_id, hcam->cam_config.name);

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief check if sensor userspace enable?
 *
 * @param[in] sen_if: sensor info struct
 *
 * @return 0:Success Disable, <0:Failure, >0:Sucess Enable
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_sensor_userspace_check(sensor_info_t *sen_if)
{
	int32_t ret = RET_OK;
	uint32_t userspace_enable = 0U;
	uint32_t ctrl_disable;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	if (m->userspace_control != NULL) {
		ret = m->userspace_control(sen_if->port, &userspace_enable);
		if (ret < 0) {
			cam_err("sensor%d %s userspace_control get error %d\n",
				sindex, sname, ret);
			return ret;
		}
	}
	if (userspace_enable == 0U)
		return RET_OK;

	ctrl_disable = camera_env_get_ulong(CAMENV_CTRL_DISABLE, 0U);
	if ((ctrl_disable & (0x1U << sen_if->port)) != 0U) {
		cam_info("sensor%d %s userspace_control 0x%x disable\n",
			sindex, sname, userspace_enable);
		return RET_OK;
	}

	if (((userspace_enable & HAL_LINE_CONTROL) != 0) && (m->aexp_line_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but aexp_line_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_GAIN_CONTROL) != 0) && (m->aexp_gain_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but aexp_gain_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_AWB_CONTROL) != 0) && (m->awb_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but awb_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_AF_CONTROL) != 0) && (m->af_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but af_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_ZOOM_CONTROL) != 0) && (m->zoom_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but zoom_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_AWB_CCT_CONTROL) != 0) && (m->awb_cct_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but awb_cct_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}
	if (((userspace_enable & HAL_AE_LINE_GAIN_CONTROL) != 0) && (m->aexp_line_gain_control == NULL)) {
		cam_err("sensor%d %s userspace_control 0x%x but aexp_line_gain_control NULL error\n",
			sindex, sname, userspace_enable);
		ret = -RET_ERROR;
	}

	if (ret < 0)
		return ret;

	return (int32_t)userspace_enable;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief the sensor userspace ctrl do here
 *
 * @param[in] m: the sensor modult struct
 * @param[in] enable: the ctrl enable mas
 * @param[in] ctrl: control hal struct to use
 * @param[in] info: control info struct to use
 *
 * @return >=0:Success(ops mask), <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_sensor_ctrl_do(sensor_module_t *m, uint32_t enable, hal_control_info_t *ctrl, sensor_ctrl_info_t *info)
{
	int32_t ret = RET_OK;
	int32_t ops = 0;

	if ((m == NULL) || (ctrl == NULL) || (info == NULL))
		return -RET_ERROR;
	camera_debug_loop_cami(ctrl->port, 2U, "ctrl_do");

	/* start control */
	if(m->start_control != NULL) {
		ret |= m->start_control(ctrl);
	}
	/* write gain */
	if (((enable & HAL_GAIN_CONTROL) != 0U) && (m->aexp_gain_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 3U, "aexp_gain_control");
		ops |= HAL_GAIN_CONTROL;
		ret |= m->aexp_gain_control(ctrl, info->mode, info->gain_buf, info->dgain_buf, info->gain_num);
		camera_debug_loop_camo(ctrl->port, 3U, "aexp_gain_control");
	}
	/* write line */
	if (((enable & HAL_LINE_CONTROL) != 0U) && (m->aexp_line_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 4U, "aexp_line_control");
		ops |= HAL_LINE_CONTROL;
		ret |= m->aexp_line_control(ctrl, info->mode, info->line_buf, info->line_num);
		camera_debug_loop_cami(ctrl->port, 4U, "aexp_line_control");
	}
	/* write line/gain */
	if (((enable & HAL_AE_LINE_GAIN_CONTROL) != 0U) && (m->aexp_line_gain_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 5U, "aexp_line_gain_control");
		ops |= HAL_AE_LINE_GAIN_CONTROL;
		ret |= m->aexp_line_gain_control(ctrl, info->mode, info->line_buf, info->line_num,
					info->gain_buf, info->dgain_buf, info->gain_num);
		camera_debug_loop_camo(ctrl->port, 5U, "aexp_line_gain_control");
	}
	/* write awb */
	if (((enable & HAL_AWB_CONTROL) != 0U) && (m->awb_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 6U, "awb_control");
		ops |= HAL_AWB_CONTROL;
		ret |= m->awb_control(ctrl, info->mode, info->rgain, info->bgain,
				info->grgain, info->gbgain);
		camera_debug_loop_camo(ctrl->port, 6U, "awb_control");
	}
	/* write awb cct */
	if (((enable & HAL_AWB_CCT_CONTROL) != 0U) && (m->awb_cct_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 7U, "awb_cct_control");
		ops |= HAL_AWB_CCT_CONTROL;
		ret |= m->awb_cct_control(ctrl, info->mode, info->rgain, info->bgain,
				info->grgain, info->gbgain, info->color_temper);
		camera_debug_loop_camo(ctrl->port, 7U, "awb_cct_control");
	}
	/* af control */
	if (((enable & HAL_AF_CONTROL) != 0U) && (m->af_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 8U, "af_control");
		ops |= HAL_AF_CONTROL;
		ret |= m->af_control(ctrl, info->mode, info->af_pos);
		camera_debug_loop_camo(ctrl->port, 8U, "af_control");
	}
	// zoom control
	if (((enable & HAL_ZOOM_CONTROL) != 0U) && (m->zoom_control != NULL)) {
		camera_debug_loop_cami(ctrl->port, 9U, "zoom_control");
		ops |= HAL_ZOOM_CONTROL;
		ret |= m->zoom_control(ctrl, info->mode, info->zoom_pos);
		camera_debug_loop_cami(ctrl->port, 9U, "zoom_control");
	}
	/* end control */
	if(m->end_control != NULL) {
		ret |= m->end_control(ctrl);
	}

	if (ret < 0) {
		cam_err("sensor%d ctrl 0x%x error\n", ctrl->port, ops);
		ret = -ops;
	} else {
		ret = ops;
	}

	camera_debug_loop_camo(ctrl->port, 2U, "ctrl_do");
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief the sensor userspace ctrl func for ctrl thread
 *
 * @param[in] arg: the sensor info struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void *camera_sensor_ctrl_func(void *arg)
{
	int32_t ret;
	uint32_t userspace_enable = 0;
	sensor_info_t *sen_if;
	sensor_module_t *m;
	hal_control_info_t ctrl = { 0 };
	sensor_ctrl_info_t info = { 0 };
	sensor_ctrl_result_t res = { 0 };
	int32_t sindex;
	char *sname;
	char tname[32];

	if (arg == NULL) {
		return NULL;
	}
	sen_if = (sensor_info_t *)(arg);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);
	if (m == NULL)
		return NULL;
	if (m->userspace_control != NULL)
		m->userspace_control(sen_if->port, &userspace_enable);
	if (userspace_enable == 0U) {
		cam_warn("sensor%d %s thread userspace_control invalid 0x%x\n",
			sen_if->port, sen_if->sensor_name, userspace_enable);
		return NULL;
	}

	snprintf(tname, sizeof(tname), "sen%d_ctrl:%s", sindex, sname);
	camera_prctl(PR_SET_NAME, tname);

	/* fill ctrl info */
	ctrl.port = sen_if->port;
	ctrl.sensor_mode = sen_if->sensor_mode;
	ctrl.bus_type = sen_if->bus_type;
	ctrl.bus_num = sen_if->bus_num;
	ctrl.sensor_addr = sen_if->sensor_addr;
	ctrl.sensor1_addr = sen_if->sensor1_addr;
	ctrl.serial_addr = sen_if->serial_addr;
	ctrl.serial_addr1 = sen_if->serial_addr1;
	ctrl.eeprom_addr = sen_if->eeprom_addr;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	memcpy(&ctrl.sensor_spi_info, &sen_if->spi_info, sizeof(spi_data_t));
#endif

	cam_info("thread %s work en 0x%x\n", tname, userspace_enable);
	while (sen_if->ctrl_thread_created <= 1) {
		camera_debug_loop_cami(ctrl.port, 1U, "ctrl_thread");
		memset(&info, 0, sizeof(info));
		info.port = ctrl.port;
		ret = camera_sensor_cdev_info_sync(sen_if, &info);
		if (ret < 0) {
			camera_debug_loop_camo(ctrl.port, 0U, "ctrl_thread");
			camera_sys_msleep(1);
			continue;
		}
		ret = camera_sensor_ctrl_do(m, userspace_enable, &ctrl, &info);
		res.port = ctrl.port;
		res.id = info.id;
		if (ret < 0) {
			res.ops = -ret;
			res.result = -1;
			(void)camera_sensor_cdev_result(sen_if, &res);
			camera_sys_msleep(10);
			continue;
		} else {
			res.ops = ret;
			res.result = 0;
			(void)camera_sensor_cdev_result(sen_if, &res);
		}
		camera_debug_loop_camo(ctrl.port, 1U, "ctrl_thread");
	}

	sen_if->ctrl_thread_created = 0;
	cam_info("thread %s exit\n", tname);

	return NULL;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor userspace ctrl init: create thread if need
 *
 * @param[in] sen_if: sensor info struct
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
static int32_t camera_sensor_ctrl_init(sensor_info_t *sen_if)
{
	int32_t ret;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;

	ret = camera_sensor_userspace_check(sen_if);
	if (ret <= 0)
		return ret;
	camera_debug_call_cami(sen_if->port);

	ret  = camera_sensor_cdev_open(sen_if);
	if (ret < 0) {
		cam_err("sensor%d %s ctrl open error %d\n",
			sen_if->port, sen_if->sensor_name, ret);
		return ret;
	}

	ret = camera_pthread_create(&sen_if->ctrl_thread_id, NULL,
				camera_sensor_ctrl_func, sen_if);
	if (ret < 0) {
		cam_err("sensor%d %s ctrl thread create error %d\n",
			sen_if->port, sen_if->sensor_name, ret);
		(void)camera_sensor_cdev_close(sen_if);
		return ret;
	}
	sen_if->ctrl_thread_created = 1;

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor userspace ctrl deinit: join thread if need
 *
 * @param[in] sen_if: sensor info struct
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
static int32_t camera_sensor_ctrl_deinit(sensor_info_t *sen_if)
{
	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);

	if (sen_if->ctrl_thread_created == 1) {
		sen_if->ctrl_thread_created = 2;
		camera_pthread_join(sen_if->ctrl_thread_id, NULL);

		(void)camera_sensor_cdev_close(sen_if);
	}

	camera_debug_call_camo(sen_if->port);
	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief the device event do something
 *
 * @param[in] sen_if: the sensor info struct
 * @param[in] event: the device event struct
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
static int32_t camera_sensor_event_do(sensor_info_t *sen_if, sensor_event_info_t *event)
{
	int32_t ret = RET_OK;

	if ((sen_if == NULL) || (event == NULL))
		return -RET_ERROR;

	switch (event->type) {
	case SEN_EVENT_TYPE_INVALID:
		camera_sys_msleep(500);
		break;
	case SEN_EVENT_TYPE_STREAM:
		if (event->data != 0)
			ret = camera_sensor_start(sen_if);
		else
			ret = camera_sensor_stop(sen_if);
		break;
	default:
		cam_err("sensor%d %s dev op type %d error\n",
			sen_if->port, sen_if->sensor_name, event->type);
		ret = -RET_ERROR;
		break;
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief the device operation func for event thread
 *
 * @param[in] arg: the sensor info struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void *camera_sensor_devop_func(void *arg)
{
	int32_t ret;
	sensor_info_t *sen_if;
	sensor_module_t *m;
	sensor_event_info_t event = { 0 };
	int32_t sindex;
	char *sname;
	char tname[32];

	if (arg == NULL) {
		return NULL;
	}
	sen_if = (sensor_info_t *)(arg);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);
	if (m == NULL)
		return NULL;

	snprintf(tname, sizeof(tname), "sen%d:%s", sindex, sname);
	camera_prctl(PR_SET_NAME, tname);

	cam_info("thread %s work\n", tname);
	while (sen_if->op_thread_created <= 1) {
		camera_debug_loop_cami(sindex, 0U, "op_thread");
		memset(&event, 0, sizeof(event));
		ret = camera_sensor_dev_event_get(sen_if, &event);
		if (ret < 0) {
			camera_sys_msleep(200);
			camera_debug_loop_camo(sindex, 0U, "op_thread");
			continue;
		}
		ret = camera_sensor_event_do(sen_if, &event);
		camera_sensor_dev_event_put(sen_if, ret);
		if (ret < 0)
			camera_sys_msleep(50);
		camera_debug_loop_camo(sindex, 0U, "op_thread");
	}

	sen_if->op_thread_created = 0;
	cam_info("thread %s exit\n", tname);

	return NULL;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief set the sensor devop thread as work in/out
 *
 * @param[in] sen_if: sensor info struct
 * @param[in] work: 1-work in, 0-work out
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
static int32_t camera_sensor_devop_thread(sensor_info_t *sen_if, int32_t work)
{
	int32_t ret = RET_OK;

	if (camera_sensor_dev_nodrv(sen_if))
		return RET_OK;
	camera_debug_call_cami(sen_if->port);

	if (work != 0) {
		ret = camera_pthread_create(&sen_if->op_thread_id, NULL,
				camera_sensor_devop_func, sen_if);
		if (ret < 0) {
			cam_err("sensor%d %s event thread create error %d\n",
				sen_if->port, sen_if->sensor_name, ret);
			return ret;
		}
		sen_if->op_thread_created = 1;
	} else {
		if (sen_if->op_thread_created == 1) {
			sen_if->op_thread_created = 2;
			camera_pthread_join(sen_if->op_thread_id, NULL);
		}
	}

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module is inited?
 *
 * @param[in] sen_if: sensor_info struct
 *
 * @return 1:inited, 0:deinited
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_is_inited(sensor_info_t *sen_if)
{
	return ((sen_if != NULL) && (sen_if->init_state == CAM_INIT)) ? 1 : 0;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module is started as streaming on?
 *
 * @param[in] sen_if: sensor_info struct
 *
 * @return 1:started, 0:stoped
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_sensor_is_started(sensor_info_t *sen_if)
{
	return ((sen_if != NULL) && (sen_if->start_state == CAM_START)) ? 1 : 0;
}

int32_t ae_share_flag;
/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module init the hardware to ready
 *
 * @param[in] sen_if: sensor module stuct
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
int32_t camera_sensor_init(sensor_info_t *sen_if)
{
	int32_t ret;
	int32_t real = 0;
	int32_t ae_share_flag;
	int32_t ae_src, ae_user;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;
	calib_info_t *cal_if;
	sensor_input_param_t input_param = { 0 };

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	cal_if = (calib_info_t *)sen_if->calib_info;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);
	if (m->init == NULL) {
		cam_err("sensor%d %s module init call invalid error\n",
			sindex, sname);
		return -RET_ERROR;
	}

	/* dev open */
	ret = camera_sensor_dev_open(sen_if);
	if (ret < 0) {
		cam_err("sensor%d %s dev open error %d\n", sindex, sname, ret);
		return ret;
	}

	/* i2c init */
	ret = camera_i2c_init(sen_if->bus_num);
	if (ret < 0) {
		cam_err("sensor%d %s i2c%d init error %d\n",
			sindex, sname, sen_if->bus_num, ret);
		goto init_error_devclose;
	}
	ret = camera_i2c_timeout_set(sen_if->bus_num, sen_if->bus_timeout);
	if (ret < 0) {
		cam_err("sensor%d %s i2c%d set timeout %dms error %d\n",
			sindex, sname, sen_if->bus_num, sen_if->bus_timeout, ret);
		goto init_error_i2cdeinit;
	}

	/* request init ? */
	ret = camera_sensor_dev_init_req(sen_if);
	if (ret == 0) {
		cam_dbg("sensor%d %s init real doing\n", sindex, sname);
		real = 1;

		/* ae share init */
		ae_share_flag = sen_if->ae_share_flag;
		ae_src = SENSOR_AE_SHARE_SRC(ae_share_flag);
		ae_user = SENSOR_AE_SHARE_SRC(ae_share_flag);
		if (ae_src == sindex) {
			ret = camera_sensor_dev_ae_share(sen_if, ae_share_flag);
			if (ret < 0) {
				cam_err("sensor%d %s ae share as src 0x%x error %d\n",
					sindex, sname, ae_share_flag, ret);
				goto init_error_resulterr;
			}
		}
		if (((ae_src == sindex) || (ae_user == sindex)) &&
			(m->ae_share_init != NULL)) {
			ret = m->ae_share_init(ae_share_flag);
			if (ret < 0) {
				cam_err("sensor%d %s ae share call 0x%x error %d\n",
					sindex, sname, ae_share_flag, ret);
				goto init_error_resulterr;
			}
		}

		/* input param set if need */
		if (sen_if->ts_compensate != 0) {
			input_param.ts_compensate = sen_if->ts_compensate;
			ret = camera_sensor_dev_input_param(sen_if, &input_param);
			if (ret < 0) {
				cam_err("sensor%d %s set ts_compensate %d error %d\n",
					sindex, sname, input_param.ts_compensate, ret);
				goto init_error_resulterr;
			}
		}

#ifndef HB_X5_CALI
		/* calib init if need */
		if (cal_if != NULL) {
			ret = camera_calib_init(cal_if);
			if (ret < 0) {
				cam_err("sensor%d %s calib init error %d\n",
					sindex, sname, ret);
				goto init_error_resulterr;
			}
		}
#endif

		/* do init */
		ret = m->init(sen_if);
		if (ret < 0) {
			cam_err("sensor%d %s init error %d\n",
				sindex, sname, ret);
#ifndef HB_X5_CALI
			goto init_error_calibdeinit;
#else
			goto init_error_resulterr;
#endif
		}

		/* ctrl init */
		ret = camera_sensor_ctrl_init(sen_if);
		if (ret < 0) {
			cam_err("sensor%d %s ctrl init error %d\n",
				sindex, sname, ret);
			goto init_error_deinit;
		}

		/* do diag_init if need */
		if (((camera_g_cfg()->diag_disable & CAMERA_DIAG_DISABLE_SENSOR) == 0U) &&
			(m->diag_nodes_init != NULL)) {
			ret = m->diag_nodes_init(sen_if);
			if (ret < 0) {
				cam_err("sensor%d %s diag init error %d\n",
					sindex, sname, ret);
				goto init_error_ctrldeinit;
			}
		}
	}
	/* event thread need work ? */
	if (!SENSOR_FIS_NO_OPTHREAD(sen_if)) {
		ret = camera_sensor_devop_thread(sen_if, 1);
		if (ret < 0) {
			cam_err("sensor%d %s op thread %d\n",
				sindex, sname, ret);
			if (real == 0)
				goto init_error_resulterr;
			else
				goto init_error_ctrldeinit;
		}
	}

	/* init done */
	if (real == 0)
		cam_info("sensor%d %s init req as ignore\n", sindex, sname);
	else
		cam_info("sensor%d %s init real done\n", sindex, sname);
	camera_sensor_dev_init_result(sen_if, ret);
	sen_if->init_state = CAM_INIT;

	camera_debug_call_camo(sen_if->port);
	return ret;

init_error_ctrldeinit:
	camera_sensor_ctrl_deinit(sen_if);
init_error_deinit:
	if (m->deinit != NULL)
		m->deinit(sen_if);
#ifndef HB_X5_CALI
init_error_calibdeinit:
	if (cal_if != NULL)
		camera_calib_deinit(cal_if);
#endif
init_error_resulterr:
	camera_sensor_dev_init_result(sen_if, ret);
init_error_i2cdeinit:
	camera_i2c_deinit(sen_if->bus_num);
init_error_devclose:
	camera_sensor_dev_close(sen_if);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module deinit to exit
 *
 * @param[in] sen_if: sensor module stuct
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
int32_t camera_sensor_deinit(sensor_info_t *sen_if)
{
	int32_t ret, real = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;
	calib_info_t *cal_if;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_camo(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	cal_if = (calib_info_t *)sen_if->calib_info;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	/* request deinit ? */
	ret = camera_sensor_dev_deinit(sen_if);
	if (ret == 0) {
		cam_dbg("sensor%d %s deinit real doing\n", sindex, sname);
		real = 1;
	}

	/* op thread exit ? */
	if (!SENSOR_FIS_NO_OPTHREAD(sen_if))
		camera_sensor_devop_thread(sen_if, 0);
	/* ctrl thread exit ? */
	camera_sensor_ctrl_deinit(sen_if);

	if (real != 0) {
		/* do deinit */
		if (m->deinit != NULL)
			m->deinit(sen_if);

#ifndef HB_X5_CALI
		if (cal_if != NULL)
			camera_calib_deinit(cal_if);
#endif
	}

	/* i2c deinit */
	camera_i2c_deinit(sen_if->bus_num);
	/* dev close */
	camera_sensor_dev_close(sen_if);

	/* deinit done */
	if (real == 0)
		cam_info("sensor%d %s deinit req as ignore\n", sindex, sname);
	else
		cam_info("sensor%d %s deinit real done\n", sindex, sname);
	sen_if->init_state = CAM_DEINIT;

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module start as streaming on
 *
 * @param[in] sen_if: sensor_info struct
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
int32_t camera_sensor_start(sensor_info_t *sen_if)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;
	uint64_t start_us, use_us;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for start error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	if (sen_if->start_state == CAM_START) {
		cam_info("sensor%d %s has started ignore\n", sindex, sname);
		camera_debug_call_camo(sen_if->port);
		return RET_OK;
	}

	start_us = camera_sys_gettime_us();
	if (m->start != NULL)
		ret = m->start(sen_if);
	else
		ret = camera_sensor_dev_start(sen_if);
	use_us = camera_sys_gettime_us() - start_us;

	if (ret < 0) {
		cam_err("sensor%d %s start error %d %lu.%03lums\n", sindex, sname, ret,
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));
		return ret;
	}

	ret = camera_addition_start(sindex);
	if (ret < 0) {
		cam_err("sensor%d %s addition start error %d\n", sindex, sname, ret);
		camera_sensor_stop(sen_if);
		return ret;
	}

	cam_info("sensor%d %s start done %lu.%03lums\n", sindex, sname,
		CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));
	sen_if->start_state = CAM_START;

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module stop as streaming off
 *
 * @param[in] sen_if: sensor_info struct
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
int32_t camera_sensor_stop(sensor_info_t *sen_if)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;
	uint64_t start_us, use_us;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for stop error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	if (sen_if->start_state == CAM_STOP) {
		cam_info("sensor%d %s has stoped ignore\n", sindex, sname);
		camera_debug_call_camo(sen_if->port);
		return RET_OK;
	}

	start_us = camera_sys_gettime_us();
	if (m->stop != NULL)
		ret = m->stop(sen_if);
	else
		ret = camera_sensor_dev_stop(sen_if);
	use_us = camera_sys_gettime_us() - start_us;

	if (ret < 0)
		cam_info("sensor%d %s stop error %d %lu.%03lums\n", sindex, sname, ret,
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));
	else
		cam_info("sensor%d %s stop done %lu.%03lums\n", sindex, sname,
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));
	if (camera_addition_stop(sindex) < 0)
		cam_info("sensor%d %s addition stop error\n", sindex, sname);
	sen_if->start_state = CAM_STOP;

	/* stop always return ok */
	camera_debug_call_camo(sen_if->port);
	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module init the hardware to ready
 *
 * @param[in] sen_if: sensor module stuct
 * @param[in] do_stop: 0-not stop, 1-do stop
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
int32_t camera_sensor_reset(sensor_info_t *sen_if, int32_t do_stop)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for reset error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	if (do_stop) {
		if (m->stop != NULL)
			ret = m->stop(sen_if);
		else
			ret = camera_sensor_dev_stop(sen_if);
		if (ret < 0)
			cam_dbg("camera%d %s reset: stop %d\n",
				sindex, sname, ret);
	}
	if (m->deinit != NULL) {
		ret = m->deinit(sen_if);
		if (ret < 0)
			cam_dbg("camera%d %s reset: deinit %d\n",
				sindex, sname, ret);
	}
	(void)camera_run_reset_ipi_by_index(sindex, 0U);
	if (m->init != NULL) {
		ret = m->init(sen_if);
		if (ret < 0) {
			cam_err("camera%d %s reset: init error %d\n",
				sindex, sname, ret);
			return ret;
		}
	}
	(void)camera_run_reset_ipi_by_index(sindex, 1U);
	if (m->start != NULL)
		ret = m->start(sen_if);
	else
		ret = camera_sensor_dev_start(sen_if);
	if (ret < 0) {
		cam_err("camera%d %s reset: start error %d\n",
			sindex, sname, ret);
		return ret;
	}

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module switch fps dynamically
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] fps: fps to set
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
int32_t camera_sensor_dynamic_switch_fps(sensor_info_t *sen_if, int32_t fps)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	if (m->dynamic_switch_fps == NULL) {
		cam_err("sensor%d %s call no dynamic_switch_fps error\n",
			sindex, sname);
		return -RET_ERROR;
	}

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for dynamic_switch_fps error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	ret = m->dynamic_switch_fps(sen_if, fps);
	if (ret < 0)
		cam_err("sensor%d %s dynamic_switch_fps error %d\n",
			sindex, sname, ret);

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module read register from hw
 *
 * @param[in] sen_if: sensor moudle struct
 * @param[in] type: the reg type to read
 * @param[in] reg_addr: the reg addr to read
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
int32_t camera_sensor_read_register(sensor_info_t *sen_if, camera_reg_type_t type, uint32_t reg_addr)
{
	int32_t ret, good = 0, error = 0;
	int32_t sindex;
	char *sname;
	int32_t dlen, alen;
	uint32_t bus;
	uint8_t addr;

	if (sen_if == NULL)
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	bus = (uint32_t)sen_if->bus_num;

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for read_register %d 0x%x error %d\n",
			sindex, sname, type, reg_addr, ret);
		return -RET_ERROR;
	}

	switch (type) {
			case CAMERA_SERIAL_REG:
			addr = (uint8_t)sen_if->serial_addr;
			alen = SENSOR_FV_ALEN(sen_if);
			dlen = SENSOR_FV_DLEN(sen_if);
			break;
		case CAMERA_EEPROM_REG:
			addr = (uint8_t)sen_if->eeprom_addr;
			alen = 16;
			dlen = 8;
			break;
		case CAMERA_SENSOR_REG:
			addr = (uint8_t)sen_if->sensor_addr;
			alen = SENSOR_FV_ALEN(sen_if);
			dlen = SENSOR_FV_DLEN(sen_if);
			break;
		default:
			cam_err("sensor%d %s read_register type %d error\n",
				sindex, sname, type);
			return -RET_ERROR;
	}

	if (reg_addr >= (0x1UL << alen)) {
		cam_err("sensor%d %s read_register reg_addr 0x%x error\n",
			sindex, sname, reg_addr);
		return -RET_ERROR;
	}

	if (alen == 8) {
		if (dlen == 8)
			ret = camera_i2c_read_reg8_data8(bus, addr, (uint16_t)reg_addr);
		else if (dlen == 16)
			ret = camera_i2c_read_reg8_data16(bus, addr, (uint16_t)reg_addr);
		else
			error = 1;
	} else if (alen == 16) {
		if (dlen == 8)
			ret = camera_i2c_read_reg16_data8(bus, addr, (uint16_t)reg_addr);
		else if (dlen == 16)
			ret = camera_i2c_read_reg16_data16(bus, addr, (uint16_t)reg_addr);
		else
			error = 1;
	} else {
		error = 1;
	}

	if (error != 0) {
		cam_err("sensor%d %s read_register type %d alen %d dlen %d error\n",
			sindex, sname, type, alen, dlen);
		return -RET_ERROR;
	}
	if (ret < 0)
		cam_err("sensor%d %s read_register type %d reg_addr 0x%x error %d\n",
			sindex, sname, type, reg_addr, ret);

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module parse embedded data to struct info
 *
 * @param[in] sen_if: sensor moudle struct
 * @param[in] embed_raw: embedded raw data buffer
 * @param[out] embed_info: embedded info struct to store
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
int32_t camera_sensor_parse_embed_data(sensor_info_t *sen_if, char* embed_raw, struct embed_data_info_s *embed_info)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	if (m->parse_embed_data == NULL) {
		cam_err("sensor%d %s call no parse_embed_data error\n",
			sindex, sname);
		return -RET_ERROR;
	}

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for parse_embed_data error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	ret = m->parse_embed_data(sen_if, embed_raw, embed_info);
	if (ret < 0)
		cam_err("sensor%d %s parse_embed_data error %d\n",
			sindex, sname, ret);

	camera_debug_call_camo(sen_if->port);
	return ret;
}

 /**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module update ae info
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] ae_info: camera ae info sturct to update
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
int32_t camera_sensor_update_ae_info(sensor_info_t *sen_if, camera_ae_info_t *ae_info)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for update_ae_info error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	ret = camera_sensor_dev_update_ae_info(sen_if, ae_info);
	if (ret < 0)
		cam_err("sensor%d %s update_ae_info error %d\n",
			sindex, sname, ret);

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor get sensor info from sensor lib
 *
 * @param[in] sen_if: sensor info struct
 * @param[in] sp: sensor param for usr
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void camera_sensor_get_iparam(sensor_info_t *sen_if, cam_usr_info_t *sp)
{
	int32_t ret = 0, sp_ret = 0;
	sensor_module_t *m;
	int32_t sindex;
	char *sname;
	const char *sp_state[CAM_SP_STATEA_MAX] = CAM_SP_STATE_NAMES;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL) || (sp == NULL))
		return;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	if (m->get_sns_params != NULL) {
		ret = m->get_sns_params(sen_if, &(sp->iparam), 3);
		if (ret < 0) {
			cam_err("sensor%d %s get_sns_params error %d\n",
				sindex, sname, ret);
		}
		sp_ret = camera_sensor_param_get(sen_if, &(sp->iparam));
		if (sp_ret < 0) {
			cam_warn("sensor%d %s common param get invalid ignore\n",
				sindex, sname, sp_ret);
		}
		if (ret < 0) {
			sp->state = CAM_SP_STATE_ERROR;
		} else {
			sp->state = CAM_SP_STATE_SUCCESS;
		}
	} else {
		sp->state = CAM_SP_STATE_NOT_SUPPROT;
	}
	if (sp->state == CAM_SP_STATE_SUCCESS)
		cam_info("sensor%d %s %dx%d@%.1fps state %d-%s\n", sindex, sname,
			sp->iparam.sns_param.width, sp->iparam.sns_param.height,
			sp->iparam.sns_param.fps, sp->state, sp_state[sp->state]);
	else
		cam_warn("sensor%d %s %dx%d@%.1fps state %d-%s\n", sindex, sname,
			sp->iparam.sns_param.width, sp->iparam.sns_param.height,
			sp->iparam.sns_param.fps, sp->state, sp_state[sp->state]);

	camera_debug_call_camo(sen_if->port);
	return ;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor set sensor info to driver
 *
 * @param[in] sen_devfd: sensor open fd
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
int32_t camera_sensor_set_sensor_info_to_driver(sensor_info_t *sen_if)
{
	int32_t ret;
	int32_t sindex;
	char *sname;
	cam_usr_info_t int_sp = { 0 };
	const char *sp_state[CAM_SP_STATEA_MAX] = CAM_SP_STATE_NAMES;

	if (sen_if == NULL)
		return -RET_ERROR;
	sindex = sen_if->port;
	sname = sen_if->sensor_name;

	if (sen_if->iparam_mode == 0) {
		camera_sensor_get_iparam(sen_if, &int_sp);
	} else {
		int_sp.state = CAM_SP_STATE_NOT_SUPPROT;
	}
	ret = camera_sensor_dev_set_intrinsic_param(sen_if, &int_sp);
	if (ret < 0) {
		cam_err("sensor%d %s set_intrinsic_param state %d-%s error %d\n",
			sindex, sname, int_sp.state, sp_state[int_sp.state], ret);
		return ret;
	}

	cam_dbg("sensor%d %s set_intrinsic_param state %d-%s\n",
		sindex, sname, int_sp.state, sp_state[int_sp.state]);
	return ret;
}

 /**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor get sensor info from driver
 *
 * @param[in] camera_index: sensor index
 * @param[in] sp: sensor param for usr
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
int32_t camera_sensor_get_sensor_info_from_driver(int32_t camera_index, cam_parameter_t *sp)
{
	int32_t ret, good = 0;
	cam_usr_info_t int_sp = { 0 };
	sensor_info_t *sen_if = NULL;
	const char *sp_state[CAM_SP_STATEA_MAX] = CAM_SP_STATE_NAMES;

	if ((camera_index < 0) || (camera_index >= CAM_CONFIG_CAMERA_MAX)) {
		cam_err("camera get as %d over %d error\n",
			camera_index, CAM_CONFIG_CAMERA_MAX);
		return -RET_ERROR;
	}

	if (sp == NULL) {
		return -HB_CAM_INVALID_PARAM;
	}
	ret = camera_run_cam_get(camera_index, &good, NULL, &sen_if, NULL);
	if ((ret < 0) || (good == 0) || (sen_if == NULL)) {
		ret = camera_run_cam_get_pre(camera_index, NULL, &sen_if, NULL);
		if ((ret < 0) || (sen_if == NULL)) {
			cam_err("sensor%d get sen_if error\n", camera_index);
			return -RET_ERROR;
		}
		/* fill port info */
		sen_if->port = camera_index;
		sen_if->dev_port = camera_index;
		/* open first if not good */
		ret = camera_sensor_dev_open(sen_if);
		if (ret < 0) {
			cam_err("sensor%d dev open fail\n", camera_index);
			return -RET_ERROR;
		}
		ret = camera_sensor_dev_get_intrinsic_param(sen_if, &int_sp);
		if (ret < 0) {
			cam_err("sensor%d get_intrinsic_param error %d\n",
				sen_if->port, ret);
			camera_sensor_dev_close(sen_if);
			return ret;
		}
		camera_sensor_dev_close(sen_if);
	} else {
		/* detect use if good */
		ret = camera_sensor_dev_get_intrinsic_param(sen_if, &int_sp);
		if (ret < 0) {
			cam_err("sensor%d %s get_intrinsic_param error %d\n",
				sen_if->port, sen_if->sensor_name, ret);
			return ret;
		}
	}

	if (int_sp.state == CAM_SP_STATE_SUCCESS) {
		(void)memcpy(sp, &(int_sp.iparam), sizeof(cam_parameter_t));
	} else if (int_sp.state == CAM_SP_STATE_FAILED) {
		ret = -HB_CAM_INVALID_OPERATION;
	} else if (int_sp.state == CAM_SP_STATE_ERROR) {
		ret = -HB_CAM_INVALID_PARAM;
	} else if (int_sp.state == CAM_SP_STATE_NOT_SUPPROT) {
		ret = -HB_CAM_OPS_NOT_SUPPORT;
	} else {
		ret = -HB_CAM_INVALID_PARAM;
	}

	cam_info("sensor%d get_intrinsic_param state %d-%s ret %d\n",
		camera_index, int_sp.state, sp_state[int_sp.state], ret);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module get sensor info
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] type: sensor info type as camera_param_type_e
 * @param[out] sp: sensor info struct to store
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
int32_t camera_sensor_get_sensor_info(sensor_info_t *sen_if, camera_param_type_t type, cam_parameter_t *sp)
{
	int32_t ret, good = 0;
	int32_t sindex;
	char *sname;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	sindex = sen_if->port;
	sname = sen_if->sensor_name;
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	if (m->get_sns_params == NULL) {
		cam_err("sensor%d %s call no get_sns_params error\n",
			sindex, sname);
		return -RET_ERROR;
	}

	ret = camera_run_cam_get(sindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("sensor%d %s not good for get_sns_params error %d\n",
			sindex, sname, ret);
		return -RET_ERROR;
	}

	ret = m->get_sns_params(sen_if, sp, type);
	if (ret < 0) {
		cam_err("sensor%d %s get_sns_params type %d error %d\n",
			sindex, sname, type, ret);
		return ret;
	}
	(void)camera_sensor_param_get(sen_if, sp);

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module get attr info of csi
 *
 * @param[in] sen_if: sensor_info struct
 * @param[out] csi_attr: csi attr stuct to store
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
int32_t camera_sensor_get_csi_attr(sensor_info_t *sen_if, csi_attr_t *csi_attr)
{
	int32_t ret = RET_OK;
	int32_t emode_sen_dt;
	sensor_module_t *m;

	if ((sen_if == NULL) || (sen_if->sensor_ops == NULL) || (csi_attr == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(sen_if->port);
	m = SENSOR_MODULE_T(sen_if->sensor_ops);

	emode_sen_dt = camera_sensor_emode_datatype_parse(sen_if);
	/* pre set for common default attr */
	csi_attr->phy = CSI_ATTR_PHY_DPHY;
	if (emode_sen_dt > 0)
		csi_attr->datatype[0] = emode_sen_dt;
	csi_attr->fps = sen_if->fps;
	csi_attr->width = sen_if->width;
	csi_attr->height = sen_if->height;

	/* shold fill: lane, mipiclk, settle */
	if (m->get_csi_attr != NULL) {
		ret = m->get_csi_attr(sen_if, csi_attr);
		if (ret < 0) {
			cam_err("camera%d %s get csi attr error %d\n",
				sen_if->port, sen_if->sensor_name, ret);
			return ret;
		}
	}

	camera_debug_call_camo(sen_if->port);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module get name and version string
 *
 * @param[in] sen_if: sensor_info struct
 * @param[out] name: sensor module name string to store
 * @param[out] version: sensor module version string to store
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
int32_t camera_sensor_get_version(sensor_info_t *sen_if, char *name, char *version)
{
	int32_t ret = RET_OK;
	camera_handle_st *hcam = NULL;
	const char *ver;

	if (sen_if == NULL)
		return -RET_ERROR;

	if (name != NULL) {
		ret = camera_run_cam_get(sen_if->port, NULL, &hcam, NULL, NULL);
		if ((ret == 0) && (hcam != NULL)) {
			strncpy(name, hcam->sensor_lib.so_name, CAMERA_MODULE_NAME_LEN);
		} else {
			ret = -RET_ERROR;
		}
	}
	if (version != NULL) {
		ver = CAMERA_MODULE_GET_VERSION(SENSOR_MODULE_M(sen_if));
		if (ver != NULL)
			strncpy(version, ver, CAMERA_VERSON_LEN_MAX);
		else
			ret = -RET_ERROR;
	}

	return 0;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor module dump regs for debug with link_mask
 *
 * @param[in] sen_if: sensor info struct
 * @param[in] link_mask: the link mask to dump, 0 for all
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
int32_t camera_sensor_dump(sensor_info_t *sen_if)
{
	int32_t ret = RET_OK;

	if (sen_if == NULL)
		return -RET_ERROR;

	return ret;
}
