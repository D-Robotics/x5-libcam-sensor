/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_txser_lib.c
 *
 * @NO{S10E02C06}
 * @ASIL{B}
 */

#define pr_mod	"txser_lib"

#include <stdlib.h>
#include <string.h>

#include "hb_camera_error.h"

#include "camera_log.h"
#include "camera_env.h"
#include "camera_i2c.h"
#include "camera_gpio.h"
#include "camera_sys.h"

#include "cam_runtime.h"
#include "cam_module.h"
#include "cam_txser_lib.h"
#include "cam_vpf.h"
#include "cam_debug.h"

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief check the txser ko_version valid by lib module
 *
 * @param[in] module: the module struct of txser lib
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
static int32_t camera_txser_ko_version_check(const camera_module_t *module)
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
			cam_dbg("txser %s v%s ko_ver skip check\n", module->name, so_ver);
			return 0;
		}
		/* ko(of lib) version show only here */
		cam_dbg("txser %s v%s ko v%u.%u\n", module->name, so_ver,
			ko_ver->major, ko_ver->minor);
	}

	return ret;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser config addr check is valid?
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
static int32_t camera_txser_config_addr_check(char *so_name, const char *name, int32_t addr)
{
	if (!CAM_CONFIG_CHECK_RANGE(addr, CAM_TXSER_CHECK_ADDR_MIN, CAM_TXSER_CHECK_ADDR_MAX) &&
	    (addr != CAM_TXSER_CHECK_ADDR_EXCEPT)) {
		cam_err("txser %s check config %s 0x%x not in range [0x%x, 0x%x]/0x%x error\n",
			so_name, name, addr, CAM_TXSER_CHECK_ADDR_MIN, CAM_TXSER_CHECK_ADDR_MAX,
			CAM_TXSER_CHECK_ADDR_EXCEPT);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser config value range check is valid?
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
static int32_t camera_txser_config_range_check(char *so_name, const char *name, int32_t value, int32_t min, int32_t max)
{
	if (!CAM_CONFIG_CHECK_RANGE(value, min, max)) {
		cam_err("txser %s check config %s 0x%x not in range [0x%x, 0x%x] error\n",
			so_name, name,  value, min, max);
		return -RET_ERROR;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser module check txs_config if valid?
 *
 * @param[in] lib: the txser module lib struct to used
 * @param[in] txs_config: the camera config stuct to check
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
int32_t camera_txser_config_check(camera_module_lib_t *lib, txser_config_t *txs_config)
{
	int32_t ret = RET_OK;
	int32_t error = 0;

	if ((lib == NULL) || (txs_config == NULL))
		return -RET_ERROR;

	/* no check? */
	if (camera_env_get_bool(CAMENV_CONFIG_NOCHECK, FALSE) == TRUE)
		return RET_OK;

	error += camera_txser_config_addr_check(lib->so_name, "addr", txs_config->addr);
	error += camera_txser_config_range_check(lib->so_name, "reset_delay", txs_config->reset_delay,
				CAM_TXSER_CHECK_RESET_DELAY_MIN, CAM_TXSER_CHECK_RESET_DELAY_MAX);
	error += camera_txser_ko_version_check(lib->module);

	if (error != 0) {
		cam_err("txser %s check config has %d error\n", txs_config->name, -error);
		ret = -RET_ERROR;
	}

	return ret;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief bind txs_if ops by txser module handle
 *
 * @param[in] htxs: the txser handle struct
 * @param[out] txs_if: the desrial_info struct
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
int32_t camera_txser_ops_bind(txser_handle_st *htxs, txser_info_t *txs_if)
{
	int32_t ret = RET_OK;
	camera_module_lib_t *lib;

	if ((htxs == NULL) || (txs_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(htxs);
	cam_dbg("txser %s ops bind\n", htxs->txs_config.name);

	lib = &htxs->txser_lib;
	txs_if->txser_ops = lib->body;

	camera_debug_hcall_ro(htxs);
	return ret;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief parse txs_if by txser_config and vin_attr of txser handle
 *
 * @param[in] htxs: the txser handle struct
 * @param[out] txs_if: the desrial_info struct
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
int32_t camera_txser_config_parse(txser_handle_st *htxs, txser_info_t *txs_if)
{
	int32_t ret = RET_OK;
	txser_config_t *cfg;
	camera_vin_attr_t *vin;
	vcon_attr_t *vcon;
	int32_t i;

	if ((htxs == NULL) || (txs_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(htxs);
	cfg = &htxs->txs_config;
	vin = &htxs->vin_attr;
	vcon = &vin->vcon_attr;
	cam_dbg("txser %s 0x%02x config parse\n", cfg->name, cfg->addr);

	txs_if->index = VIN_ATTR_TO_TXSER_INDEX(vin);
	txs_if->bus_type = I2C_BUS;
	if ((cfg->bus_select == 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_MAIN) != 0)) {
		txs_if->bus_num = vcon->bus_main;
	} else if ((cfg->bus_select != 0) && ((vcon->attr_valid & VCON_ATTR_V_BUS_SEC) != 0)) {
		txs_if->bus_num = vcon->bus_second;
	} else {
		cam_err("vcon no valid %s bus attr error\n",
			(cfg->bus_select) ? "second" : "main");
		return -RET_ERROR;
	}
	txs_if->txser_addr = cfg->addr;
	txs_if->lane_mode = cfg->lane_mode;
	if (cfg->link_map == 0U) {
		cfg->link_map = CAM_TXSER_LINK_MAP_DEFAULT;
		cam_dbg("txser %s link_map as default 0x%x\n", cfg->name, cfg->link_map);
	}
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	txs_if->gpio_num = 0;
	if ((cfg->gpio_enable != 0U) && ((cfg->gpio_enable & BIT(31)) != 0U)) {
		if ((vcon->attr_valid & VCON_ATTR_V_GPIO_SER) == 0) {
			cam_err("vcon no valid ser 0x%x gpio attr error\n",
				cfg->gpio_enable);
			return -RET_ERROR;
		}
		for (i = VGPIO_SER_BASE; i < (VGPIO_SER_BASE + VGPIO_SER_NUM); i++) {
			if ((cfg->gpio_enable & BIT(i - VGPIO_SER_BASE)) == 0U)
				continue;
			if (vcon->gpios[i] != 0) {
				txs_if->gpio_pin[txs_if->gpio_num] = vcon->gpios[i];
				txs_if->gpio_level[txs_if->gpio_num] =
				    (cfg->gpio_level & BIT(i - VGPIO_SER_BASE)) ? 1 : 0;
				txs_if->gpio_num++;
			}
		}
		if (txs_if->gpio_num == 0) {
			cam_warn("vcon no such ser 0x%x gpio attr\n",
				cfg->gpio_enable);
		}
	}
#endif
	txs_if->txser_name = cfg->name;
	txs_if->bus_timeout = cfg->bus_timeout;
	if (cfg->reset_delay == 0U) {
		cfg->reset_delay = CAM_TXSER_RESET_DELAY_DEFAULT;
		cam_dbg("txser %s reset_delay as default %d\n", cfg->name, cfg->reset_delay);
	}
	txs_if->reset_delay = cfg->reset_delay;
	txs_if->txser_attr = cfg->flags;
	txs_if->gpio_enable = cfg->gpio_enable;
	txs_if->gpio_levels = cfg->gpio_level;
	// TODO: others cfg
	// cfg->txser_param;

	camera_debug_hcall_ro(htxs);
	return ret;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief parse the mipi csi attr by txser_config and txser lib runtime
 *
 * @param[in] htxs: the txser handle struct with mipi_config struct
 * @param[in] txs_if: the desrial_info struct
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
int32_t camera_txser_csi_attr_parse(txser_handle_st *htxs, txser_info_t *txs_if, mipi_config_t *mipi_from_rx,
				mipi_config_t *mipi_to, mipi_bypass_t *bypass_to)
{
	int32_t ret, tindex;
	const char *tname;
	txser_config_t *cfg;

	if ((htxs == NULL) || (txs_if == NULL) || (mipi_to == NULL))
		return -RET_ERROR;
	cfg = &htxs->txs_config;
	tindex = VIN_ATTR_TO_TXSER_INDEX(&htxs->vin_attr);
	tname = cfg->name;

	ret = camera_vpf_mipi_config_parse_tx(mipi_to, bypass_to, cfg->mipi_cfg, mipi_from_rx);
	if (ret < 0) {
		cam_err("txser%d %s csi attr parse error %d\n", tindex, tname, ret);
		return ret;
	}

	if (mipi_to->rx_enable != 0)
		cam_dbg("txser%d %s csi attr rx parse: %ulane %uMbps 0x%02x\n",
			tindex, tname, mipi_to->rx_attr.lane, mipi_to->rx_attr.mipiclk,
			mipi_to->rx_attr.datatype);
	if (mipi_to->bypass != NULL)
		cam_dbg("txser%d %s csi attr tx%d parse: %ulane %uMbps 0x%02x\n",
			tindex, tname, mipi_to->bypass->tx_index, mipi_to->bypass->tx_attr.lane,
			mipi_to->bypass->tx_attr.mipiclk, mipi_to->bypass->tx_attr.datatype);
	if ((mipi_to->rx_enable == 0) && (mipi_to->bypass == NULL))
		cam_info("txser%d %s csi not enable\n", tindex, tname);

	return RET_OK;
}

#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser power control for legacy
 *
 * @param[in] txs_if: the txser info struct
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
static int32_t camera_txser_power_legacy(txser_info_t *txs_if, int32_t work)
{
	int32_t ret = 0, i;

	camera_debug_call_txsi(txs_if->index);
	for(i = 0; i < txs_if->gpio_num; i++) {
		if(txs_if->gpio_pin[i] >=0) {
			ret |= camera_gpio_power_ctrl((uint32_t)txs_if->gpio_pin[i],
						      txs_if->gpio_level[i]);
		}
	}
	if (work != 0) {
		camera_sys_msleep(txs_if->reset_delay);
		for(i = 0; i < txs_if->gpio_num; i++) {
			if(txs_if->gpio_pin[i] >=0) {
				ret |= camera_gpio_power_ctrl((uint32_t)txs_if->gpio_pin[i],
							      1 - txs_if->gpio_level[i]);
			}
		}
	}
	if(ret < 0) {
		cam_err("txser power legacy error\n");
		return -RET_ERROR;
	}

	camera_debug_call_txso(txs_if->index);
	return ret;
}
#endif

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser module power operation
 *
 * @param[in] txs_if: txser info struct
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
static int32_t camera_txser_power(txser_info_t *txs_if, int32_t on)
{
	txser_handle_st *htxs = NULL;
	int32_t ret, pwren = 0, level = 0, v;

	ret = camera_run_txs_get(txs_if->index, NULL, &htxs, NULL);
	if ((ret < 0) || (htxs == NULL))
		return ret;
	camera_debug_call_txsi(txs_if->index);

	ret = camera_vpf_vin_get_gpio(&htxs->vin_attr, txs_if->gpio_enable, txs_if->gpio_levels, VGPIO_SER_PWREN, &pwren, &level);
	if (ret < 0) {
		cam_err("power get PWREN gpio error %d\n", ret);
		return ret;
	}
	if (pwren == 0)
		return RET_OK;

	v = (on) ? (1 - level) : level;
	ret = camera_gpio_power_ctrl((uint32_t)pwren, v);
	if (ret < 0) {

		cam_err("txser%d %s power %s PWREN gpio%d=%d error %d\n",
			txs_if->index, txs_if->txser_name, (on) ? "on" : "off",
			pwren, v, ret);
	}
	cam_info("txser%d %s power %s PWREN gpio%d=%d\n",
		txs_if->index, txs_if->txser_name, (on) ? "on" : "off",
		pwren, v);
	camera_sys_msleep(txs_if->reset_delay * 5);

	camera_debug_call_txso(txs_if->index);
	return ret;
}

/**
 * @NO{S10E02C06}
 * @ASIL{B}
 * @brief txser module reset operation
 *
 * @param[in] txs_if: txser info struct
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
static int32_t camera_txser_reset(txser_info_t *txs_if)
{
	txser_handle_st *htxs = NULL;
	int32_t ret, pwdn = 0, level = 0;

	ret = camera_run_txs_get(txs_if->index, NULL, &htxs, NULL);
	if ((ret < 0) || (htxs == NULL))
		return ret;
	camera_debug_call_txsi(txs_if->index);

	ret = camera_vpf_vin_get_gpio(&htxs->vin_attr, txs_if->gpio_enable, txs_if->gpio_levels, VGPIO_SER_PWDN, &pwdn, &level);
	if (ret < 0) {
		cam_err("reset get PWDN gpio error %d\n", ret);
		return ret;
	}
	if (pwdn == 0)
		return RET_OK;

	ret = camera_gpio_power_ctrl((uint32_t)pwdn, level);
	if (ret < 0) {
		cam_err("txser%d %s reset_ PWDN gpio%d=%d error %d\n",
			txs_if->index, txs_if->txser_name, pwdn, level, ret);
		return ret;
	}
	camera_sys_msleep(txs_if->reset_delay);
	ret = camera_gpio_power_ctrl((uint32_t)pwdn, 1 - level);
	if (ret < 0) {
		cam_err("txser%d %s reset- PWDN gpio%d=%d error %d\n",
			txs_if->index, txs_if->txser_name, pwdn, 1 - level, ret);
		return ret;
	}

	camera_debug_call_txso(txs_if->index);
	return ret;
}

/**
 * @NO{S10E02C06I}
 * @ASIL{B}
 * @brief txser module init operation
 *
 * @param[in] txs_if: txser info struct
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
int32_t camera_txser_init(txser_info_t *txs_if)
{
	int32_t ret;
	int32_t gpio_done = 0;
	int32_t tindex;
	char *tname;
	txser_module_t *m;

	if ((txs_if == NULL) || (txs_if->txser_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_txsi(txs_if->index);
	tindex = txs_if->index;
	tname = txs_if->txser_name;
	m = TXSER_MODULE_T(txs_if->txser_ops);
	if (m->init == NULL) {
		cam_err("txser%d %s module init call invalid error\n",
			tindex, tname);
		return -RET_ERROR;
	}

	/* i2c init */
	ret = camera_i2c_init(txs_if->bus_num);
	if (ret < 0) {
		cam_err("txser%d %s i2c%d init error %d\n",
			tindex, tname, txs_if->bus_num, ret);
		return ret;
	}
	ret = camera_i2c_timeout_set(txs_if->bus_num, txs_if->bus_timeout);
	if (ret < 0) {
		cam_err("txser%d %s i2c%d set timeout %dms error %d\n",
			tindex, tname, txs_if->bus_num, txs_if->bus_timeout, ret);
		goto init_error_i2cdeinit;
	}

	cam_dbg("txser%d %s init doing\n", tindex, tname);
	/* self gpio init */
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	if (txs_if->gpio_num > 0) {
		ret = camera_txser_power_legacy(txs_if, 1);
		if (ret < 0)
			goto init_error_i2cdeinit;
		gpio_done = 1;
	}
#endif
	if (gpio_done == 0) {
		ret = camera_txser_power(txs_if, 1);
		if (ret < 0)
			goto init_error_i2cdeinit;
		ret = camera_txser_reset(txs_if);
		if (ret < 0)
			goto init_error_poweroff;
	}

	/* do init */
	ret = m->init(txs_if);
	if (ret < 0) {
		cam_err("txser%d %s init error %d\n",
			tindex, tname, ret);
		goto init_error_poweroff;
	}

	/* do diag_init if need */
	if (((camera_g_cfg()->diag_disable & CAMERA_DIAG_DISABLE_TXSER) == 0U) &&
		(m->diag_nodes_init != NULL)) {
		ret = m->diag_nodes_init(txs_if);
		if (ret < 0) {
			cam_err("txser%d %s diag init error %d\n",
				tindex, tname, ret);
			goto init_error_deinit;
		}
	}

	cam_info("txser%d %s init done\n", tindex, tname);

	camera_debug_call_txso(txs_if->index);
	return ret;

init_error_deinit:
	if (m->deinit != NULL)
		m->deinit(txs_if);
init_error_poweroff:
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	if (gpio_done == 1)
		camera_txser_power_legacy(txs_if, 0);
#endif
	if (gpio_done == 0)
		camera_txser_power(txs_if, 0);
init_error_i2cdeinit:
	camera_i2c_deinit(txs_if->bus_num);

	return ret;
}

/**
 * @NO{S10E02C06I}
 * @ASIL{B}
 * @brief txser module deinit operation
 *
 * @param[in] txs_if: txser info struct
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
int32_t camera_txser_deinit(txser_info_t *txs_if)
{
	int32_t ret = 0;
	int32_t gpio_done = 0;
	int32_t dindex;
	char *dname;
	txser_module_t *m;

	if ((txs_if == NULL) || (txs_if->txser_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_txsi(txs_if->index);
	dindex = txs_if->index;
	dname = txs_if->txser_name;
	m = TXSER_MODULE_T(txs_if->txser_ops);

	cam_dbg("txser%d %s deinit doing\n", dindex, dname);

	/* do deinit */
	if (m->deinit != NULL)
		m->deinit(txs_if);

	/* self gpio deinit */
#ifdef CAM_CONFIG_INFO_LEGACY_COMPATIBLE
	if (txs_if->gpio_num > 0) {
		camera_txser_power_legacy(txs_if, 0);
		gpio_done = 1;
	}
#endif
	if (gpio_done == 0)
		camera_txser_power(txs_if, 0);

	/* i2c deinit */
	camera_i2c_deinit(txs_if->bus_num);

	/* deinit done */
	cam_info("txser%d %s deinit done\n", dindex, dname);

	camera_debug_call_txso(txs_if->index);
	return ret;
}

/**
 * @NO{S10E02C06I}
 * @ASIL{B}
 * @brief txser module get name and version string
 *
 * @param[in] txs_if: txser info struct
 * @param[in] name: name string to store
 * @param[in] version: versiong string to store
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
int32_t camera_txser_get_version(txser_info_t *txs_if, char *name, char *version)
{
	int32_t ret = RET_OK;
	txser_handle_st *htxs = NULL;
	const char *ver;

	if (txs_if == NULL)
		return -RET_ERROR;

	if (name != NULL) {
		ret = camera_run_txs_get(txs_if->index, NULL, &htxs, NULL);
		if ((ret == 0) && (htxs != NULL)) {
			strncpy(name, htxs->txser_lib.so_name, CAMERA_MODULE_NAME_LEN);
		} else {
			ret = -RET_ERROR;
		}
	}
	if (version != NULL) {
		ver = CAMERA_MODULE_GET_VERSION(TXSER_MODULE_M(txs_if));
		if (ver != NULL)
			strncpy(version, ver, CAMERA_VERSON_LEN_MAX);
		else
			ret = -RET_ERROR;
	}

	return ret;
}

