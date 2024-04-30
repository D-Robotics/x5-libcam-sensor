/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_poc_lib.c
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#define pr_mod	"poc_lib"

#include <stdlib.h>
#include <string.h>

#include "hb_camera_error.h"

#include "camera_log.h"
#include "camera_env.h"

#include "cam_runtime.h"
#include "cam_poc_lib.h"
#include "cam_debug.h"

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief check the poc ko_version valid by lib module
 *
 * @param[in] module: the module struct of poc lib
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
static int32_t camera_poc_ko_version_check(const camera_module_t *module)
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
			cam_dbg("poc %s v%s ko_ver skip check\n", module->name, so_ver);
			return 0;
		}
		/* ko(of lib) version show only here */
		cam_dbg("poc %s v%s ko v%u.%u\n", module->name, so_ver,
			ko_ver->major, ko_ver->minor);
	}

	return ret;
}


/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief poc config addr check is valid?
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
static int32_t camera_poc_config_addr_check(char *so_name, const char *name, int32_t addr)
{
	if (!CAM_CONFIG_CHECK_RANGE(addr, CAM_POC_CHECK_ADDR_MIN, CAM_POC_CHECK_ADDR_MAX) &&
	    (addr != CAM_POC_CHECK_ADDR_EXCEPT)) {
		cam_err("poc %s check config %s 0x%x not in range [0x%x, 0x%x]/0x%x error\n",
			so_name, name, addr, CAM_POC_CHECK_ADDR_MIN, CAM_POC_CHECK_ADDR_MAX,
			CAM_POC_CHECK_ADDR_EXCEPT);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief poc config value range check is valid?
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
static int32_t camera_poc_config_range_check(char *so_name, const char *name, int32_t value, int32_t min, int32_t max)
{
	if (!CAM_CONFIG_CHECK_RANGE(value, min, max)) {
		cam_err("poc %s check config %s 0x%x not in range [0x%x, 0x%x] error\n",
			so_name, name,  value, min, max);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief poc config value mask range check is valid?
 *
 * @param[in] so_name: so name for error info show
 * @param[in] name: config name for error info show
 * @param[in] val: config value to check
 * @param[in] mask: config value mask to check
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
static int32_t camera_poc_config_mask_range_check(char *so_name, const char *name, int32_t value,
						int32_t mask, int32_t min, int32_t max)
{
	if ((value & ~mask) != 0) {
		cam_err("poc %s check config %s 0x%x not in mask 0x%x error\n",
			so_name, name,  value, mask);
		return -RET_ERROR;
	}
	if (!CAM_CONFIG_CHECK_RANGE(value, min, max)) {
		cam_err("poc %s check config %s 0x%x not in range [0x%x, 0x%x] error\n",
			so_name, name,  value, min, max);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief poc module check poc_config if valid?
 *
 * @param[in] lib: the poc module lib struct to used
 * @param[in] poc_config: the poc config stuct to check
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
int32_t camera_poc_config_check(camera_module_lib_t *lib, poc_config_t *poc_config)
{
	int32_t ret = RET_OK;
	int32_t error = 0;
	const char *pname;

	if ((lib == NULL) || (poc_config == NULL))
		return -RET_ERROR;
	pname = (poc_config->name[0] != '\0') ? poc_config->name : CAM_POC_INSIDE_NAME;

	/* no check? */
	if (camera_env_get_bool(CAMENV_CONFIG_NOCHECK, FALSE) == TRUE)
		return RET_OK;
	cam_dbg("poc %s config check\n", pname);

	error += camera_poc_config_addr_check(lib->so_name, "addr", poc_config->addr);
	error += camera_poc_config_range_check(lib->so_name, "power_delay", poc_config->power_delay,
				CAM_POC_POWER_DELAY_MIN, CAM_POC_POWER_DELAY_MAX);
	error += camera_poc_config_mask_range_check(lib->so_name, "poc_map", poc_config->poc_map,
				CAM_POC_MAP_MASK_VAL, CAM_POC_MAP_MASK_MIN, CAM_POC_MAP_MASK_MAX);
	error += camera_poc_ko_version_check(lib->module);

	if (error != 0) {
		cam_err("poc %s check config has %d error\n", pname, -error);
		ret = -RET_ERROR;
	}

	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief bind poc_if ops by deserial handle
 *
 * @param[in] hdes: the deserial handle struct
 * @param[out] poc_if: the poc_info struct
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
int32_t camera_poc_ops_bind(deserial_handle_st *hdes, poc_info_t *poc_if)
{
	camera_module_lib_t *lib;

	if ((hdes == NULL) || (poc_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hdes);
	cam_dbg("poc %s ops bind\n", hdes->poc_config.name);

	lib = &hdes->poc_lib;

	poc_if->poc_ops = lib->body;
	poc_if->poc_fd = lib->so_fd;

	camera_debug_hcall_ro(hdes);
	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief parse poc_if by poc_config and vin_attr of deserial handle
 *
 * @param[in] hdes: the deserial handle struct
 * @param[out] poc_if: the poc_info struct
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
int32_t camera_poc_config_parse(deserial_handle_st *hdes, poc_info_t *poc_if)
{
	deserial_config_t *dcfg;
	poc_config_t *cfg;
	camera_vin_attr_t *vin;
	vcon_attr_t *vcon;
	int32_t i;

	if ((hdes == NULL) || (poc_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hdes);
	dcfg = &hdes->des_config;
	cfg = &hdes->poc_config;
	vin = &hdes->vin_attr;
	vcon = &vin->vcon_attr;
	cam_dbg("poc %s 0x%02x config parse\n", cfg->name, cfg->addr);

	poc_if->index = VIN_ATTR_TO_DESERIAL_INDEX(vin);
	poc_if->bus_type = I2C_BUS;
	if ((dcfg->bus_select == 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_MAIN) != 0)) {
		poc_if->bus_num = vcon->bus_main;
	} else if ((dcfg->bus_select != 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_SEC) != 0)) {
		poc_if->bus_num = vcon->bus_second;
	} else {
		cam_err("vcon no valid %s bus attr error\n",
			(dcfg->bus_select) ? "second" : "main");
		return -RET_ERROR;
	}
	poc_if->poc_addr = cfg->addr;
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	poc_if->gpio_num = 0;
	if (cfg->gpio_enable != 0U) {
		if ((vcon->attr_valid & VCON_ATTR_V_GPIO_POC) == 0) {
			cam_err("vcon no valid poc 0x%x gpio attr error\n",
				cfg->gpio_enable);
			return -RET_ERROR;
		}
		for (i = VGPIO_POC_BASE; i < VGPIO_POC_NUM; i++) {
			if ((cfg->gpio_enable & BIT(i - VGPIO_POC_BASE)) == 0)
				continue;
			if (vcon->gpios[i] != 0) {
				poc_if->gpio_pin[poc_if->gpio_num] = vcon->gpios[i];
				poc_if->gpio_level[poc_if->gpio_num] =
				    (cfg->gpio_level & BIT(i - VGPIO_POC_BASE)) ? 1 : 0;
				poc_if->gpio_num++;
			}
		}
		if (poc_if->gpio_num == 0) {
			cam_warn("vcon no such poc 0x%x gpio attr\n",
				cfg->gpio_enable);
		}
	}
#endif
	poc_if->poc_name = cfg->name;
	poc_if->power_delay = cfg->power_delay;
	if (cfg->poc_map == 0U) {
		cfg->poc_map = CAM_POC_MAP_DEFAULT;
		cam_dbg("poc %s poc_map as default 0x%x\n", cfg->name, cfg->poc_map);
	}

	camera_debug_hcall_ro(hdes);
	return RET_OK;
}


/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief poc module init operation
 *
 * @param[in] poc_if: poc info struct
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
int32_t camera_poc_init(poc_info_t *poc_if)
{
	int32_t ret = RET_OK;
	int32_t op_do = 0;
	int32_t pindex;
	char *pname;
	poc_module_t *m;

	if ((poc_if == NULL) || (poc_if->poc_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(poc_if->index);
	pindex = poc_if->index;
	pname = poc_if->poc_name;
	m = POC_MODULE_T(poc_if->poc_ops);

	cam_dbg("poc%d %s init doing\n", pindex, pname);
	if (m->init != NULL) {
		ret = m->init(poc_if);
		if (ret < 0) {
			cam_err("poc%d %s init error %d\n",
				pindex, pname, ret);
			return ret;
		}
		op_do = 1;
	}

	/* do diag_init if need */
	if (((camera_g_cfg()->diag_disable & CAMERA_DIAG_DISABLE_SENSOR) == 0U) &&
		(m->diag_nodes_init != NULL)) {
		ret = m->diag_nodes_init(poc_if);
		if (ret < 0) {
			cam_err("poc%d %s diag init error %d\n",
				pindex, pname, ret);
			if (m->deinit)
				m->deinit(poc_if);
			return ret;
		}
		op_do = 1;
	}

	if (op_do == 0)
		cam_dbg("poc %d %s module init call ignore\n",
			pindex, pname);
	else
		cam_info("poc%d %s init done\n", pindex, pname);

	camera_debug_call_deso(poc_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief poc module deinit operation
 *
 * @param[in] poc_if: poc info struct
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
int32_t camera_poc_deinit(poc_info_t *poc_if)
{
	int32_t ret = RET_OK;
	int32_t op_do = 0;
	int32_t pindex;
	char *pname;
	poc_module_t *m;

	if ((poc_if == NULL) || (poc_if->poc_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(poc_if->index);
	pindex = poc_if->index;
	pname = poc_if->poc_name;
	m = POC_MODULE_T(poc_if->poc_ops);

	cam_dbg("poc%d %s deinit doing\n", pindex, pname);
	if (m->deinit != NULL) {
		m->deinit(poc_if);
		op_do = 1;
	}

	if (op_do == 0)
		cam_dbg("poc %d %s module deinit call ignore\n",
		pindex, pname);
	else
		cam_info("poc%d %s deinit done\n", pindex, pname);

	camera_debug_call_deso(poc_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief poc module power on one link
 *
 * @param[in] poc_if: poc info struct
 * @param[in] link: link index to power on
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
int32_t camera_poc_on(poc_info_t *poc_if, int32_t link)
{
	int32_t ret, good = 0, port;
	int32_t pindex;
	char *pname;
	poc_module_t *m;

	if ((poc_if == NULL) || (poc_if->poc_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(poc_if->index);
	pindex = poc_if->index;
	pname = poc_if->poc_name;
	m = POC_MODULE_T(poc_if->poc_ops);

	if (m->power_on == NULL) {
		cam_err("poc%d %s no valid power_on call error\n",
			pindex, pname);
		return -RET_ERROR;
	}

	ret = camera_run_des_get(pindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("poc%d %s not good for power on error %d\n",
			pindex, pname, ret);
		return -RET_ERROR;
	}

	port = POC_LINK_TO_PORT(poc_if->poc_map, link);
	ret = m->power_on(poc_if, port);
	if (ret < 0)
		cam_err("poc%d %s power on error %d\n",
			pindex, pname, ret);

	camera_debug_call_deso(poc_if->index);
	return ret;

}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief poc module power off one link
 *
 * @param[in] poc_if: poc info struct
 * @param[in] link: link index to power off
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
int32_t camera_poc_off(poc_info_t *poc_if, int32_t link)
{
	int32_t ret, good = 0, port;
	int32_t pindex;
	char *pname;
	poc_module_t *m;

	if ((poc_if == NULL) || (poc_if->poc_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_desi(poc_if->index);
	pindex = poc_if->index;
	pname = poc_if->poc_name;
	m = POC_MODULE_T(poc_if->poc_ops);

	if (m->power_off == NULL) {
		cam_err("poc%d %s no valid power_off call error\n",
			pindex, pname);
		return -RET_ERROR;
	}

	ret = camera_run_des_get(pindex, &good, NULL, NULL, NULL);
	if ((ret < 0) || (good == 0)) {
		cam_err("poc%d %s not good for power off error %d\n",
			pindex, pname, ret);
		return -RET_ERROR;
	}

	port = POC_LINK_TO_PORT(poc_if->poc_map, link);
	ret = m->power_off(poc_if, port);
	if (ret < 0)
		cam_err("poc%d %s power off error %d\n",
			pindex, pname, ret);

	camera_debug_call_deso(poc_if->index);
	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief poc module get name and version string
 *
 * @param[in] poc_if: poc info struct
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
int32_t camera_poc_get_version(poc_info_t *poc_if, char *name, char *version)
{
	int32_t ret = RET_OK;
	deserial_handle_st *hdes = NULL;
	const char *ver;

	if (poc_if == NULL)
		return -RET_ERROR;

	if (name != NULL) {
		ret = camera_run_des_get(poc_if->index, NULL, &hdes, NULL, NULL);
		if ((ret == 0) && (hdes != NULL)) {
			strncpy(name, hdes->poc_lib.so_name, CAMERA_MODULE_NAME_LEN);
		} else {
			ret = -RET_ERROR;
		}
	}
	if (version != NULL) {
		ver = CAMERA_MODULE_GET_VERSION(POC_MODULE_M(poc_if));
		if (ver != NULL)
			strncpy(version, ver, CAMERA_VERSON_LEN_MAX);
		else
			ret = -RET_ERROR;
	}


	return ret;
}

