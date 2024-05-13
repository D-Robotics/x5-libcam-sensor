/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_deserial_lib.c
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#define pr_mod	"deserial_lib"

#include <stdlib.h>
#include <string.h>

#include "hb_camera_error.h"

#include "camera_log.h"
#include "camera_env.h"
#include "camera_i2c.h"
#include "camera_gpio.h"
#include "camera_sys.h"
#include "camera_deserial_dev.h"
#include "camera_sensor_common.h"

#include "cam_runtime.h"
#include "cam_module.h"
#include "cam_deserial_lib.h"
#include "cam_poc_lib.h"
#include "cam_sensor_lib.h"
#include "cam_vpf.h"
#include "cam_debug.h"

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief check the deserial ko_version valid by lib module
 *
 * @param[in] module: the module struct of deserial lib
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
static int32_t camera_deserial_ko_version_check(const camera_module_t *module)
{
	int32_t ret = 0;
	const char *so_ver;
	camera_ko_version_t *ko_ver;

	if (!CAMERA_MODULE_CHECK_VALID(module))
		return 0;
	ko_ver = CAMERA_MODULE_GET_KO_VERSION(module);
	if (ko_ver == NULL)
		return 0;
	so_ver = CAMERA_MODULE_GET_VERSION(module);
	if (so_ver == NULL)
		so_ver = "unknown";
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		/* ko(of lib) version is valid? */
		if ((ko_ver->major == 0U) && (ko_ver->minor == 0U)) {
			cam_dbg("deserial %s v%s ko_ver skip check\n", module->name, so_ver);
			return 0;
		}
		/* lib driver version should >= ko(of lib) version */
		if ((DESERIAL_VER_MAJOR < ko_ver->major) ||
			((DESERIAL_VER_MAJOR == ko_ver->major) && (DESERIAL_VER_MINOR < ko_ver->minor))) {
			cam_err("deserial %s v%s check ko v%u.%u > v%u.%u error\n", module->name, so_ver,
				ko_ver->major, ko_ver->minor, DESERIAL_VER_MAJOR, DESERIAL_VER_MINOR);
			ret = -RET_ERROR;
		} else {
			cam_dbg("deserial %s v%s ko v%u.%u\n", module->name, so_ver,
				ko_ver->major, ko_ver->minor);
		}
	}

	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial config addr check is valid?
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
static int32_t camera_deserial_config_addr_check(char *so_name, const char *name, int32_t addr)
{
	if (!CAM_CONFIG_CHECK_RANGE(addr, CAM_DESERIAL_CHECK_ADDR_MIN, CAM_DESERIAL_CHECK_ADDR_MAX) &&
		(addr != CAM_DESERIAL_CHECK_ADDR_EXCEPT)) {
		cam_err("deserial %s check config %s 0x%x not in range [0x%x, 0x%x]/0x%x error\n",
			so_name, name, addr, CAM_DESERIAL_CHECK_ADDR_MIN, CAM_DESERIAL_CHECK_ADDR_MAX,
			CAM_DESERIAL_CHECK_ADDR_EXCEPT);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial config value range check is valid?
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
static int32_t camera_deserial_config_range_check(char *so_name, const char *name, int32_t value, int32_t min, int32_t max)
{
	if (!CAM_CONFIG_CHECK_RANGE(value, min, max)) {
		cam_err("deserial %s check config %s 0x%x not in range [0x%x, 0x%x] error\n",
			so_name, name,  value, min, max);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial module check des_config if valid?
 *
 * @param[in] lib: the deserial module lib struct to used
 * @param[in] des_config: the deserial config stuct to check
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
int32_t camera_deserial_config_check(camera_module_lib_t *lib, deserial_config_t *des_config)
{
	int32_t ret = RET_OK;
	int32_t error = 0;

	if ((lib == NULL) || (des_config == NULL))
		return -RET_ERROR;

	/* no check? */
	if (camera_env_get_bool(CAMENV_CONFIG_NOCHECK, FALSE) == TRUE)
		return RET_OK;
	cam_dbg("deserial %s config check\n", des_config->name);

	error += camera_deserial_config_addr_check(lib->so_name, "addr", des_config->addr);
	error += camera_deserial_config_range_check(lib->so_name, "reset_delay", des_config->reset_delay,
				CAM_DESERIAL_CHECK_RESET_DELAY_MIN, CAM_DESERIAL_CHECK_RESET_DELAY_MAX);
	error += camera_deserial_config_range_check(lib->so_name, "lane_speed", des_config->lane_speed,
				CAM_DESERIAL_CHECK_LANE_SPEED_MIN, CAM_DESERIAL_CHECK_LANE_SPEED_MAX);
	error += camera_deserial_ko_version_check(lib->module);

	if (error != 0) {
		cam_err("deserial %s check config has %d error\n", des_config->name, -error);
		ret = -RET_ERROR;
	}

	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief check the desrial handle config if poc is valid
 *
 * @param[in] hdes: the deserial handle struct
 *
 * @return 1:Yes-poc valid, 0:No-poc invalid
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_deserial_config_has_poc(deserial_handle_st *hdes)
{
	return (((hdes != NULL) && (hdes->des_config.poc_cfg != NULL) &&
		 (hdes->des_config.poc_cfg->name[0] != '\0')) ? 1 : 0);
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief bind des_if ops by deserial module handle
 *
 * @param[in] hdes: the deserial handle struct
 * @param[out] des_if: the desrial_info struct
 * @param[out] poc_if: the poc_info struct if need
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
int32_t camera_deserial_ops_bind(deserial_handle_st *hdes, deserial_info_t *des_if, poc_info_t *poc_if)
{
	int32_t ret = RET_OK;
	camera_module_lib_t *lib;

	if ((hdes == NULL) || (des_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hdes);
	cam_dbg("deserial %s ops bind\n", hdes->des_config.name);

	/* poc and deserial ops bind */
	if (poc_if != NULL) {
		/* bind poc_info for poc lib call */
		ret = camera_poc_ops_bind(hdes, poc_if);
		if (ret < 0) {
			cam_err("deserial%d poc %s ops bind error %d\n",
				des_if->index, hdes->poc_config.name, ret);
			return ret;
		}
	}
	des_if->poc_info = poc_if;

	lib = &hdes->deserial_lib;
	des_if->deserial_ops = lib->body;
	des_if->deserial_fd = lib->so_fd;

	camera_debug_hcall_ro(hdes);
	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief parse des_if by deserial_config and vin_attr of deserial handle
 *
 * @param[in] hdes: the deserial handle struct
 * @param[out] des_if: the desrial_info struct
 * @param[out] poc_if: the poc_info struct if need
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
int32_t camera_deserial_config_parse(deserial_handle_st *hdes, deserial_info_t *des_if)
{
	int32_t ret = RET_OK;
	deserial_config_t *cfg;
	camera_vin_attr_t *vin;
	vcon_attr_t *vcon;
	poc_info_t *poc_if;
	int32_t i;
	const uint32_t link_map_array[] = CAM_DESERIAL_LINK_MAP_ARRAY_DEFAULT;
	uint32_t link_mask = 0U;

	if ((hdes == NULL) || (des_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hdes);
	cfg = &hdes->des_config;
	vin = &hdes->vin_attr;
	vcon = &vin->vcon_attr;
	cam_dbg("deserial %s 0x%02x config parse\n", cfg->name, cfg->addr);

	des_if->index = VIN_ATTR_TO_DESERIAL_INDEX(vin);
	des_if->bus_type = I2C_BUS;
	if ((cfg->bus_select == 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_MAIN) != 0)) {
		des_if->bus_num = vcon->bus_main;
	} else if ((cfg->bus_select != 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_SEC) != 0)) {
		des_if->bus_num = vcon->bus_second;
	} else {
		cam_err("vcon no valid %s bus attr error\n",
			(cfg->bus_select) ? "second" : "main");
		return -RET_ERROR;
	}
	des_if->deserial_addr = cfg->addr;
	des_if->power_mode = 0;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	des_if->physical_entry = 0;
	des_if->mfp_index = 0;
#endif
	des_if->lane_mode = cfg->lane_mode;
	des_if->lane_speed = cfg->lane_speed;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	des_if->gpio_num = 0;
	if ((cfg->gpio_enable_bit != 0U) && ((cfg->gpio_enable_bit & BIT(31)) != 0U)) {
		if ((vcon->attr_valid & VCON_ATTR_V_GPIO_DES) == 0) {
			cam_err("vcon no valid des 0x%x gpio attr error\n",
				cfg->gpio_enable_bit);
			return -RET_ERROR;
		}
		for (i = VGPIO_DES_BASE; i < (VGPIO_DES_BASE + VGPIO_DES_NUM); i++) {
			if ((cfg->gpio_enable_bit & BIT(i - VGPIO_DES_BASE)) == 0U)
				continue;
			if (vcon->gpios[i] != 0) {
				des_if->gpio_pin[des_if->gpio_num] = vcon->gpios[i];
				des_if->gpio_level[des_if->gpio_num] =
				    (cfg->gpio_level_bit & BIT(i - VGPIO_DES_BASE)) ? 1 : 0;
				des_if->gpio_num++;
			}
		}
		if (des_if->gpio_num == 0) {
			cam_warn("vcon no such des 0x%x gpio attr\n",
				cfg->gpio_enable_bit);
		}
	}
#endif
	des_if->deserial_name = cfg->name;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	des_if->deserial_config_path = NULL;
#endif
	des_if->bus_timeout = cfg->bus_timeout;
	for (i = 0; i < DES_LINK_NUM_MAX; i++) {
		if (cfg->link_desp[i][0] != '\0') {
			des_if->port_desp[i] = cfg->link_desp[i];
			link_mask |= (0x1U << i);
		} else {
			des_if->port_desp[i] = NULL;
		}
	}
	if (cfg->link_map == 0U) {
		cfg->link_map = link_map_array[link_mask];
		cam_dbg("deserial %s link_map 0x%x as default 0x%x\n",
			cfg->name, link_mask, cfg->link_map);
	}
	for (i = 0; i < DES_CSI_NUM_MAX; i++) {
		des_if->deserial_csi[i] = DES_LINK_MAP_CSI(cfg->link_map, i);
	}
	if (cfg->reset_delay == 0U) {
		cfg->reset_delay = CAM_DESERIAL_RESET_DELAY_DEFAULT;
		cam_dbg("deserial %s reset_delay as default %d\n", cfg->name, cfg->reset_delay);
	}
	des_if->reset_delay = cfg->reset_delay;
	des_if->deserial_attr = cfg->flags;
	des_if->gpio_enable = cfg->gpio_enable_bit;
	des_if->gpio_levels = cfg->gpio_level_bit;
	for (i = 0; i < CAMERA_DES_GPIO_MAX; i++) {
		if (cfg->gpio_mfp[i] < CAMERA_DES_MFPMAX)
			des_if->deserial_gpio[i] = (int32_t)cfg->gpio_mfp[i];
		else
			des_if->deserial_gpio[i] = -1;
	}
	// TODO: others cfg
	// cfg->deserial_param;


	poc_if = (poc_info_t *)des_if->poc_info;
	if (poc_if != NULL) {
		/* parse poc_info for poc lib call */
		ret = camera_poc_config_parse(hdes, poc_if);
		if (ret < 0) {
			cam_err("deserial%d %s poc %s config parse error %d\n",
				des_if->index, cfg->name, hdes->poc_config.name, ret);
			return ret;
		}
	}
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	if (cfg->poc_cfg != NULL) {
		des_if->poc_addr = cfg->poc_cfg->addr;
		if (cfg->poc_cfg->poc_map == 0U) {
			cfg->poc_cfg->poc_map = CAM_POC_MAP_DEFAULT;
			cam_dbg("deserial%d %s poc_map as default 0x%x\n",
				des_if->index, cfg->name, cfg->poc_cfg->poc_map);
		}
		des_if->poc_map = cfg->poc_cfg->poc_map;
	}
#endif

	camera_debug_hcall_ro(hdes);
	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief sensor port mask of deserial by des_if and link_mask
 *		bit[31:28] as dummy port for deserial 4 link
 *		see: DIAG_SNR_PORT_BIT_DUMMY_LINK
 *
 * @param[in] des_if: the deserial info struct to parse
 * @param[in] link_mask: the link mask to get
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
uint32_t camera_deserial_port_mask(deserial_info_t *des_if, int32_t link_mask)
{
	int32_t i;
	uint32_t mask = 0U, port_bit;

	for (i = 0; i < DES_LINK_NUM_MAX; i++) {
		if ((link_mask & (0x1 << i)) != 0) {
			port_bit = camera_sensor_port_mask(des_if->sensor_info[i]);
#ifdef CAM_DIAG
			/* set dummy link bit if sen_if invalid but port_desp valid */
			if ((port_bit == 0U ) && (des_if->port_desp[i] != NULL))
				port_bit = BIT(DIAG_SNR_PORT_BIT_DUMMY_LINK + i);
#endif
			mask |= port_bit;
		}
	}
	return mask;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief parse the mipi csi attr by deserial_config and deserial lib runtime
 *
 * @param[in] hdes: the deserial handle struct with mipi_config struct
 * @param[in] des_if: the desrial_info struct
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
int32_t camera_deserial_csi_attr_parse(deserial_handle_st *hdes, deserial_info_t *des_if,
				mipi_config_t *mipi_to, mipi_bypass_t *bypass_to)
{
	int32_t ret, dindex;
	const char *dname;
	deserial_config_t *cfg;
	csi_attr_t csi_attr = { 0 };
	csi_attr_t *csi = NULL;

	if ((hdes == NULL) || (des_if == NULL) || (mipi_to == NULL))
		return -RET_ERROR;
	cfg = &hdes->des_config;
	dindex = VIN_ATTR_TO_DESERIAL_INDEX(&hdes->vin_attr);
	dname = cfg->name;

	ret = camera_deserial_get_csi_attr(des_if, &csi_attr);
	if (ret == RET_OK) {
		csi = &csi_attr;
	}

	ret = camera_vpf_mipi_config_parse(mipi_to, bypass_to, cfg->mipi_cfg,
				csi, CAMERA_VPF_MIPI_IPI_VC_DEFAULT);
	if (ret < 0) {
		cam_err("deserial%d %s csi attr parse error %d\n", dindex, dname, ret);
		return ret;
	}

	if (mipi_to->rx_enable != 0)
		cam_dbg("deserial%d %s csi attr rx parse: %ulane %uMbps 0x%02x\n",
			dindex, dname, mipi_to->rx_attr.lane,
			mipi_to->rx_attr.mipiclk, mipi_to->rx_attr.datatype);
	if (mipi_to->bypass != NULL)
		cam_dbg("deserial%d %s csi attr tx%d parse: %ulane %uMbps 0x%02x\n",
			dindex, dname, mipi_to->bypass->tx_index, mipi_to->bypass->tx_attr.lane,
			mipi_to->bypass->tx_attr.mipiclk, mipi_to->bypass->tx_attr.datatype);
	if ((mipi_to->rx_enable == 0) && (mipi_to->bypass == NULL))
		cam_info("deserial%d %s csi not enable\n", dindex, dname);

	return RET_OK;
}

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial power control for legacy
 *
 * @param[in] des_if: the deserial info struct
 * @param[in] work: go to working state at last?
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
static int32_t camera_deserial_power_legacy(deserial_info_t *des_if, int32_t work)
{
	int32_t ret = 0, i;

	camera_debug_call_desi(des_if->index);
	for(i = 0; i < des_if->gpio_num; i++) {
		if(des_if->gpio_pin[i] >=0) {
			ret |= camera_gpio_power_ctrl((uint32_t)des_if->gpio_pin[i],
					des_if->gpio_level[i]);
		}
	}
	if (work != 0) {
		camera_sys_msleep(des_if->reset_delay);
		for(i = 0; i < des_if->gpio_num; i++) {
			if(des_if->gpio_pin[i] >=0) {
				ret |= camera_gpio_power_ctrl((uint32_t)des_if->gpio_pin[i],
					1 - des_if->gpio_level[i]);
			}
		}
	}
	if(ret < 0) {
		cam_err("deserial power legacy error\n");
		return -RET_ERROR;
	}
	camera_debug_call_deso(des_if->index);
	return ret;
}
#endif

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial module power operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] on: 1-power on, 0-power off
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
static int32_t camera_deserial_power(deserial_info_t *des_if, int32_t on)
{
	deserial_handle_st *hdes = NULL;
	int32_t ret, pwren = 0, level = 0, v;

	ret = camera_run_des_get(des_if->index, NULL, &hdes, NULL, NULL);
	if ((ret < 0) || (hdes == NULL))
		return ret;

	ret = camera_vpf_vin_get_gpio(&hdes->vin_attr, des_if->gpio_enable, des_if->gpio_levels, VGPIO_DES_PWREN, &pwren, &level);
	if (ret < 0) {
		cam_err("power get PWREN gpio error %d\n", ret);
		return ret;
	}
	if (pwren == 0)
		return RET_OK;
	camera_debug_call_desi(des_if->index);

	v = (on) ? (1 - level) : level;
	ret = camera_gpio_power_ctrl((uint32_t)pwren, v);
	if (ret < 0) {
		cam_err("deserial%d %s power %s PWREN gpio%d=%d error %d\n",
			des_if->index, des_if->deserial_name, (on) ? "on" : "off",
			pwren, v, ret);
		return ret;
	}
	cam_info("deserial%d %s power %s PWREN gpio%d=%d\n",
		des_if->index, des_if->deserial_name, (on) ? "on" : "off",
		pwren, v);
	camera_sys_msleep(des_if->reset_delay * 5);

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial module reset operation
 *
 * @param[in] des_if: deserial info struct
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
static int32_t camera_deserial_reset(deserial_info_t *des_if)
{
	deserial_handle_st *hdes = NULL;
	int32_t ret, pwdn = 0, level = 0;

	ret = camera_run_des_get(des_if->index, NULL, &hdes, NULL, NULL);
	if ((ret < 0) || (hdes == NULL))
		return ret;

	ret = camera_vpf_vin_get_gpio(&hdes->vin_attr, des_if->gpio_enable, des_if->gpio_levels, VGPIO_DES_PWDN, &pwdn, &level);
	if (ret < 0) {
		cam_err("reset get PWDN gpio error %d\n", ret);
		return ret;
	}
	if (pwdn == 0)
		return RET_OK;
	camera_debug_call_desi(des_if->index);

	ret = camera_gpio_power_ctrl((uint32_t)pwdn, level);
	if (ret < 0) {
		cam_err("deserial%d %s reset_ PWDN gpio%d=%d error %d\n",
			des_if->index, des_if->deserial_name, pwdn, level, ret);
		return ret;
	}
	camera_sys_msleep(des_if->reset_delay);
	ret = camera_gpio_power_ctrl((uint32_t)pwdn, 1 - level);
	if (ret < 0) {
		cam_err("deserial%d %s reset- PWDN gpio%d=%d error %d\n",
			des_if->index, des_if->deserial_name, pwdn, 1 - level, ret);
		return ret;
	}

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief the device operation do something
 *
 * @param[in] des_if: the deserial info struct
 * @param[in] op: the device operation struct
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
static int32_t camera_deserial_devop_do(deserial_info_t *des_if, deserial_op_info_t *op)
{
	int32_t ret = RET_OK;

	if ((des_if == NULL) || (op == NULL))
		return -RET_ERROR;

	switch (op->type) {
	case DES_OP_TYPE_INVALID:
		camera_sys_msleep(500);
		break;
	case DES_OP_TYPE_STREAM:
		if (op->data != 0)
			ret = camera_deserial_stream_on(des_if, 0);
		else
			ret = camera_deserial_stream_off(des_if, 0);
		break;
	default:
		cam_err("deserial%d %s dev op type %d error\n",
			des_if->index, des_if->deserial_name, op->type);
		ret = -RET_ERROR;
		break;
	}

	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief the device operation func for devop thread
 *
 * @param[in] arg: the deserial info struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void *camera_deserial_devop_func(void *arg)
{
	int32_t ret;
	deserial_info_t *des_if;
	deserial_module_t *m;
	deserial_op_info_t op = { 0 };
	int32_t dindex;
	char *dname;
	char tname[32];

	if (arg == NULL) {
		return NULL;
	}
	des_if = (deserial_info_t *)(arg);
	dindex = des_if->index;
	dname = des_if->deserial_name;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);
	if (m == NULL)
		return NULL;
	if ((m->stream_on == NULL) || (m->stream_off == NULL)) {
		cam_err("deserial%d %s %s stream_on %s stream_off for thread error\n",
			dindex, dname, (m->stream_on) ? "has" : "no",
			(m->stream_off) ? "has" : "no");
		return NULL;
	}

	snprintf(tname, sizeof(tname), "des%d:%s", dindex, dname);
	camera_prctl(PR_SET_NAME, tname);

	cam_info("thread %s work\n", tname);
	while (des_if->thread_created <= 1) {
		camera_debug_loop_desi(dindex, 0U, "op_thread");
		memset(&op, 0, sizeof(op));
		ret = camera_deserial_dev_stream_get(des_if, &op);
		if (ret < 0) {
			camera_sys_msleep(200);
			camera_debug_loop_deso(dindex, 0U, "op_thread");
			continue;
		}
		ret = camera_deserial_devop_do(des_if, &op);
		camera_deserial_dev_stream_put(des_if, ret);
		if (ret < 0)
			camera_sys_msleep(50);
		camera_debug_loop_deso(dindex, 0U, "op_thread");
	}

	des_if->thread_created = 0;
	cam_info("thread %s exit\n", tname);

	return NULL;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief set the deserial devop thread as work in/out
 *
 * @param[in] des_if: deserial info struct
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
static int32_t camera_deserial_devop_thread(deserial_info_t *des_if, int32_t work)
{
	int32_t ret = RET_OK;

	if (camera_deserial_dev_nodrv(des_if))
		return RET_OK;

	camera_debug_call_desi(des_if->index);
	if (work != 0) {
		ret = camera_pthread_create(&des_if->init_thread_id, NULL,
				camera_deserial_devop_func, des_if);
		if (ret < 0) {
			cam_err("deserial%d %s op thread create error %d\n",
				des_if->index, des_if->deserial_name, ret);
			return ret;
		}
		des_if->thread_created = 1;
	} else {
		if (des_if->thread_created == 1) {
			des_if->thread_created = 2;
			camera_pthread_join(des_if->init_thread_id, NULL);
		}
	}

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module init operation
 *
 * @param[in] des_if: deserial info struct
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
int32_t camera_deserial_init(deserial_info_t *des_if)
{
	int32_t ret;
	int32_t gpio_done = 0, real = 0;
	int32_t dindex;
	char *dname;
	deserial_module_t *m;
	poc_info_t *poc_if;

	if ((des_if == NULL) || (des_if->deserial_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(des_if->index);
	dindex = des_if->index;
	dname = des_if->deserial_name;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);
	poc_if = (poc_info_t *)des_if->poc_info;
	if (m->init == NULL) {
		cam_err("deserial%d %s module init call invalid error\n",
			dindex, dname);
		return -RET_ERROR;
	}

	/* dev open */
	ret = camera_deserial_dev_open(des_if);
	if (ret < 0) {
		cam_err("deserial%d %s dev open error %d\n", dindex, dname, ret);
		return ret;
	}

	/* i2c init */
	ret = camera_i2c_init(des_if->bus_num);
	if (ret < 0) {
		cam_err("deserial%d %s i2c%d init error %d\n",
			dindex, dname, des_if->bus_num, ret);
		goto init_error_devclose;
	}
	ret = camera_i2c_timeout_set(des_if->bus_num, des_if->bus_timeout);
	if (ret < 0) {
		cam_err("deserial%d %s i2c%d set timeout %dms error %d\n",
			dindex, dname, des_if->bus_num, des_if->bus_timeout, ret);
		goto init_error_i2cdeinit;
	}

	/* request init ? */
	ret = camera_deserial_dev_init_req(des_if, 0);
	if (ret == 0) {
		cam_dbg("deserial%d %s init real doing\n", dindex, dname);
		real = 1;
		/* self gpio init */
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
		if (des_if->gpio_num > 0) {
			ret = camera_deserial_power_legacy(des_if, 1);
			if (ret < 0)
				goto init_error_resulterr;
			gpio_done = 1;
		}
#endif
		if (gpio_done == 0) {
			ret = camera_deserial_power(des_if, 1);
			if (ret < 0)
				goto init_error_resulterr;
			ret = camera_deserial_reset(des_if);
			if (ret < 0)
				goto init_error_poweroff;
		}

		/* poc init if need */
		if (poc_if != NULL) {
			ret = camera_poc_init(poc_if);
			if (ret < 0) {
				cam_err("deserial%d %s poc %s init error %d\n",
					dindex, dname, poc_if->poc_name, ret);
				goto init_error_poweroff;
			}
		}

		/* do init */
		ret = m->init(des_if);
		if (ret < 0) {
			cam_err("deserial%d %s init error %d\n",
				dindex, dname, ret);
			goto init_error_pocdeinit;
		}

		/* do data info init */
		if (des_if->data_info_inited == 0) {
			ret = camera_deserial_dev_info_init(des_if, NULL);
			if (ret < 0) {
				cam_err("deserial%d %s dev info init error %d\n",
					dindex, dname, ret);
				goto init_error_deinit;
			}
		}
	}
	/* do diag_init if need */
	if (((camera_g_cfg()->diag_disable & CAMERA_DIAG_DISABLE_DESERIAL) == 0U) &&
		(m->diag_nodes_init != NULL)) {
		ret = m->diag_nodes_init(des_if);
		if (ret < 0) {
			cam_err("deserial%d %s diag init error %d\n",
				dindex, dname, ret);
			if (real == 0)
				goto init_error_resulterr;
			else
				goto init_error_deinit;
		}
	}

	/* op thread need work ? */
	if (!DESERIAL_FIS_NO_OPTHREAD(des_if)) {
		ret = camera_deserial_devop_thread(des_if, 1);
		if (ret < 0) {
			cam_err("deserial%d %s op thread %d\n",
				dindex, dname, ret);
			if (real == 0)
				goto init_error_resulterr;
			else
				goto init_error_deinit;
		}
	}

	/* init done */
	if (real == 0)
		cam_info("deserial%d %s init req as ignore\n", dindex, dname);
	else
		cam_info("deserial%d %s init real done\n", dindex, dname);
	camera_deserial_dev_init_result(des_if, ret);

	camera_debug_call_deso(des_if->index);
	return ret;

init_error_deinit:
	des_if->data_info_inited = 0;
	if (m->deinit != NULL)
		m->deinit(des_if);
init_error_pocdeinit:
	if (poc_if != NULL)
		camera_poc_deinit(poc_if);
init_error_poweroff:
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	if (gpio_done == 1)
		camera_deserial_power_legacy(des_if, 0);
#endif
	if (gpio_done == 0)
		camera_deserial_power(des_if, 0);
init_error_resulterr:
	camera_deserial_dev_init_result(des_if, ret);
init_error_i2cdeinit:
	camera_i2c_deinit(des_if->bus_num);
init_error_devclose:
	camera_deserial_dev_close(des_if);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module deinit operation
 *
 * @param[in] des_if: deserial info struct
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
int32_t camera_deserial_deinit(deserial_info_t *des_if)
{
	int32_t ret, real = 0;
	int32_t gpio_done = 0;
	int32_t dindex;
	char *dname;
	deserial_module_t *m;
	poc_info_t *poc_if;

	if ((des_if == NULL) || (des_if->deserial_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(des_if->index);
	dindex = des_if->index;
	dname = des_if->deserial_name;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);
	poc_if = (poc_info_t *)des_if->poc_info;

	/* request deinit ? */
	ret = camera_deserial_dev_deinit(des_if);
	if (ret == 0) {
		cam_dbg("deserial%d %s deinit real doing\n", dindex, dname);
		real = 1;
	}

	/* op thread exit ? */
	if (!DESERIAL_FIS_NO_OPTHREAD(des_if))
		camera_deserial_devop_thread(des_if, 0);

	if (real != 0) {
		/* do deinit */
		if (m->deinit != NULL)
			m->deinit(des_if);

		/* poc deinit if need */
		if (poc_if != NULL)
			camera_poc_deinit(poc_if);

		/* self gpio deinit */
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
		if (des_if->gpio_num > 0) {
			camera_deserial_power_legacy(des_if, 0);
			gpio_done = 1;
		}
#endif
		if (gpio_done == 0)
			camera_deserial_power(des_if, 0);
	}

	des_if->data_info_inited = 0;
	/* i2c deinit */
	camera_i2c_deinit(des_if->bus_num);
	/* dev close */
	camera_deserial_dev_close(des_if);

	/* deinit done */
	if (real == 0)
		cam_info("deserial%d %s deinit req as ignore\n", dindex, dname);
	else
		cam_info("deserial%d %s deinit real done\n", dindex, dname);

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module stream on operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] link: the link of deserial to stream on
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
int32_t camera_deserial_stream_on(deserial_info_t *des_if, int32_t link)
{
	int32_t ret, good = 0;
	int32_t dindex;
	char *dname;
	deserial_module_t *m;
	uint64_t start_us, use_us;

	if ((des_if == NULL) || (des_if->deserial_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(des_if->index);
	dindex = des_if->index;
	dname = des_if->deserial_name;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);

	ret = camera_run_des_get(dindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("deserial%d %s not good for stream on error %d\n",
			dindex, dname, ret);
		return -RET_ERROR;
	}

	start_us = camera_sys_gettime_us();
	if (m->stream_on != NULL)
		ret = m->stream_on(des_if, link);
	else
		ret = camera_deserial_dev_stream_on(des_if, link);
	use_us = camera_sys_gettime_us() - start_us;

	if (ret < 0)
		cam_err("deserial%d %s %s stream on error %d %lu.%03lums\n",
			dindex, dname, (m->stream_on != NULL) ? "lib" : "drv", ret,
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));
	else
		cam_info("deserial%d %s %s stream on done %lu.%03lums\n",
			dindex, dname, (m->stream_on != NULL) ? "lib" : "drv",
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module stream off operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] link: the link of deserial to stream off
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
int32_t camera_deserial_stream_off(deserial_info_t *des_if, int32_t link)
{
	int32_t ret, good = 0;
	int32_t dindex;
	char *dname;
	deserial_module_t *m;
	uint64_t start_us, use_us;

	if ((des_if == NULL) || (des_if->deserial_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(des_if->index);
	dindex = des_if->index;
	dname = des_if->deserial_name;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);

	ret = camera_run_des_get(dindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("deserial%d %s not good for stream off error %d\n",
			dindex, dname, ret);
		return -RET_ERROR;
	}

	start_us = camera_sys_gettime_us();
	if (m->stream_off != NULL)
		ret = m->stream_off(des_if, link);
	else
		ret = camera_deserial_dev_stream_off(des_if, link);
	use_us = camera_sys_gettime_us() - start_us;

	if (ret < 0)
		cam_err("deserial%d %s %s stream off error %d %lu.%03lums\n",
			dindex, dname, (m->stream_off != NULL) ? "lib" : "drv", ret,
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));

	else
		cam_info("deserial%d %s %s stream off done %lu.%03lums\n",
			dindex, dname, (m->stream_off != NULL) ? "lib" : "drv",
			CAMERA_USE_MS(use_us), CAMERA_USE_RUS(use_us));

	camera_debug_call_deso(des_if->index);
	return ret;
}


/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module get csi attr info
 *
 * @param[in] des_if: deserial info struct
 * @param[in] link_mask: deserial valid link mask
 * @param[in] link_map: deserial link to csi:vc map
 * @param[out] csi_attr: csi attr info struct to store
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
static int32_t camera_deserial_get_csi_attr_default(deserial_info_t *des_if, uint32_t link_mask, uint32_t link_map, csi_attr_t *csi_attr)
{
	uint32_t i, vc, dt_set = 0U;
	int32_t link = 0, clk_set = 0;
	int32_t dindex = des_if->index;
	char *dname = des_if->deserial_name;

	for (i = 0U; i < DES_LINK_NUM_MAX; i++) {
		if ((link_mask & (0x1U << i)) == 0U)
			continue;
		vc = DES_LINK_MAP_VC(link_map, i);
		if (csi_attr->datatype[vc] == 0U) {
			csi_attr->datatype[vc] = CAM_DESERIAL_CSI_DATATYPE_DEFAULT;
			dt_set |= (0x1U << vc);
		}
		link++;
	}

	if (csi_attr->lane == 0U) {
		/* default as 4lane dphy */
		csi_attr->lane = CAM_DESERIAL_CSI_LANE_DEFAULT;
	}
	if (csi_attr->mipiclk == 0U) {
		switch (link) {
			case 4:
			case 3:
				csi_attr->mipiclk = csi_attr->lane * CAM_DESERIAL_CSI_SPEED_nV_DEFAULT;
				break;
			case 2:
				if (csi_attr->width >= 3840)
					csi_attr->mipiclk = csi_attr->lane * CAM_DESERIAL_CSI_SPEED_hV_DEFAULT;
				else
					csi_attr->mipiclk = csi_attr->lane * CAM_DESERIAL_CSI_SPEED_nV_DEFAULT;
				break;
			case 1:
				csi_attr->mipiclk = csi_attr->lane * CAM_DESERIAL_CSI_SPEED_1V_DEFAULT;
				break;
			case 0:
			default:
				return 0;
		}
		clk_set = 1;
	}

	if (dt_set && clk_set)
		cam_dbg("deserial%d %s default csi: 0x%02x/0x%x %dMbps/%dlane\n",
			dindex, dname, CAM_DESERIAL_CSI_DATATYPE_DEFAULT, dt_set, csi_attr->mipiclk, csi_attr->lane);
	else if (dt_set)
		cam_dbg("deserial%d %s default csi: 0x%02x/0x%x\n",
			dindex, dname, CAM_DESERIAL_CSI_DATATYPE_DEFAULT, dt_set);
	else if (clk_set)
		cam_dbg("deserial%d %s default csi: %dMbps/%dlane\n",
			dindex, dname, csi_attr->mipiclk, csi_attr->lane);
	return 0;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module get csi attr info
 *
 * @param[in] des_if: deserial info struct
 * @param[out] csi_attr: csi attr info struct to store
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
int32_t camera_deserial_get_csi_attr(deserial_info_t *des_if, csi_attr_t *csi_attr)
{
	int32_t ret = RET_OK;
	uint32_t i, vc, link_mask = 0U, link_map;
	int32_t emode_sen_dt;
	deserial_module_t *m;
	sensor_info_t *sen_if;
	csi_attr_t csi_sen = { 0 };
	deserial_handle_st *hdes = NULL;

	if ((des_if == NULL) || (des_if->deserial_ops == NULL) || (csi_attr == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(des_if->index);
	ret = camera_run_des_get(des_if->index, NULL, &hdes, NULL, NULL);
	if ((ret < 0) && (hdes == NULL))
		return -RET_ERROR;
	m = DESERIAL_MODULE_T(des_if->deserial_ops);
	link_map = hdes->des_config.link_map;

	/* pre set for common default attr */
	csi_attr->phy = CSI_ATTR_PHY_DPHY;
	for (i = 0U; i < DES_LINK_NUM_MAX; i++) {
		sen_if = des_if->sensor_info[i];
		if (sen_if != NULL) {
			memset(&csi_sen, 0, sizeof(csi_attr_t));
			if (camera_sensor_get_csi_attr(sen_if, &csi_sen) < 0)
				continue;
			csi_attr->fps = (csi_attr->fps > csi_sen.fps) ? csi_attr->fps : csi_sen.fps;
			csi_attr->width = (csi_attr->width > csi_sen.width) ? csi_attr->width : csi_sen.width;
			csi_attr->height = (csi_attr->height > csi_sen.height) ? csi_attr->height : csi_sen.height;
			vc = DES_LINK_MAP_VC(link_map, i);
			csi_attr->datatype[vc] = csi_sen.datatype[0];
			link_mask |= (0x1U << i);
		} else if ((des_if->port_desp[i] != NULL) && (des_if->port_desp[i][0] != '\0')) {
			emode_sen_dt = camera_sensor_string_datatype_parse(des_if->port_desp[i]);
			if (emode_sen_dt > 0) {
				vc = DES_LINK_MAP_VC(link_map, i);
				csi_attr->datatype[vc] = emode_sen_dt;
			}
			link_mask |= (0x1U << i);
		}
	}
	if (des_if->lane_speed != 0) {
		if (des_if->lane_mode == 0) {
			csi_attr->lane = 4;
			csi_attr->mipiclk = csi_attr->lane * des_if->lane_speed;
		}
	}

	/* shold fill: lane, mipiclk, settle */
	if (m->get_csi_attr != NULL)
		ret = m->get_csi_attr(des_if, csi_attr);
	else
		ret = camera_deserial_get_csi_attr_default(des_if, link_mask, link_map, csi_attr);

	camera_debug_call_deso(des_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module get name and version string
 *
 * @param[in] des_if: deserial info strct
 * @param[out] name: name string to store
 * @param[out] version: version string to store
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
int32_t camera_deserial_get_version(deserial_info_t *des_if, char *name, char *version)
{
	int32_t ret = RET_OK;
	deserial_handle_st *hdes = NULL;
	const char *ver;

	if (des_if == NULL)
		return -RET_ERROR;

	if (name != NULL) {
		ret = camera_run_des_get(des_if->index, NULL, &hdes, NULL, NULL);
		if ((ret == 0) && (hdes != NULL)) {
			strncpy(name, hdes->deserial_lib.so_name, CAMERA_MODULE_NAME_LEN);
		} else {
			ret = -RET_ERROR;
		}
	}
	if (version != NULL) {
		ver = CAMERA_MODULE_GET_VERSION(DESERIAL_MODULE_M(des_if));
		if (ver != NULL)
			strncpy(version, ver, CAMERA_VERSON_LEN_MAX);
		else
			ret = -RET_ERROR;
	}

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial module dump regs for debug with link_mask
 *
 * @param[in] des_if: deserial info struct
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
int32_t camera_deserial_dump(deserial_info_t *des_if, int32_t link_mask)
{
	int32_t ret = RET_OK;

	return ret;
}
