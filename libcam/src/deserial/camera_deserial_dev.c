/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_deserial_lib.c
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#define pr_mod	"deserial_dev"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "hb_camera_error.h"

#include "camera_log.h"
#include "camera_env.h"
#include "camera_deserial_dev.h"
#include "camera_deserial_dev_ioctl.h"

#include "cam_runtime.h"
#include "cam_module.h"
#include "cam_vpf.h"

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver open and perpare to work
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
int32_t camera_deserial_dev_open(deserial_info_t *des_if)
{
	int32_t ret;
	char des_dev[DESERIAL_DEV_NAME_LEN];
	const char *dname;
	deserial_version_info_t ver = { 0 };

	if (des_if == NULL)
		return -RET_ERROR;
	if (des_if->des_devfd > 0)
		return RET_OK;
	dname = des_if->deserial_name;
	snprintf(des_dev, sizeof(des_dev), DESERIAL_DEV_PATH, des_if->index);

	if (camera_env_get_bool(CAMENV_DRIVER_NODESERIAL, FALSE) != FALSE) {
		des_if->des_devfd = -2;
		cam_warn("open %s %s no driver as %d\n", des_dev, dname, des_if->des_devfd);
		return RET_OK;
	}

	ret = open((const char *)des_dev, O_RDWR);
	if (ret < 0) {
		cam_err("open %s %s error %d\n", des_dev, dname, ret);
		return ret;
	}
	des_if->des_devfd = ret;

	/* driver version check */
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		/* driver version should >= lib driver version */
		ret = camera_deserial_dev_get_version(des_if, &ver);
		if ((ret < 0) || ((ver.major < DESERIAL_VER_MAJOR)
#if DESERIAL_VER_MINOR
			|| ((ver.major == DESERIAL_VER_MAJOR) && (ver.minor < DESERIAL_VER_MINOR))
#endif
			)) {
			if (ret == 0) {
				cam_err("check %s driver v%u.%u < v%u.%u error\n", des_dev,
						ver.major, ver.minor, DESERIAL_VER_MAJOR, DESERIAL_VER_MINOR);
				ret = -RET_ERROR;
			}
			close(des_if->des_devfd);
			des_if->des_devfd = -1;
			return ret;
		}
		cam_dbg("open %s v%u.%u %s as %d\n", des_dev,
			ver.major, ver.minor, dname, des_if->des_devfd);
	} else {
		cam_dbg("open %s %s as %d\n", des_dev, dname, des_if->des_devfd);
	}

	return RET_OK;
}

/**
 * @NO{S10E02C07}
 * @ASIL{B}
 * @brief deserial driver is run in no driver mode?
 *
 * @param[in] des_if: deserial info struct
 *
 * @return 1:Yes, 0:No
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_deserial_dev_nodrv(deserial_info_t *des_if)
{
	return ((des_if == NULL) || (des_if->des_devfd < -1)) ? 1 : 0;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver close and exit
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
int32_t camera_deserial_dev_close(deserial_info_t *des_if)
{
	if (des_if == NULL)
		return -RET_ERROR;
	if (camera_deserial_dev_nodrv(des_if)) {
		cam_dbg("close " DESERIAL_DEV_PATH " %s no driver as %d\n",
			des_if->index, des_if->deserial_name, des_if->des_devfd);
		des_if->des_devfd = -1;
		return RET_OK;
	}
	if (des_if->des_devfd <= 0)
		return RET_OK;

	cam_dbg("close " DESERIAL_DEV_PATH " %s as %d\n",
		des_if->index, des_if->deserial_name, des_if->des_devfd);
	close(des_if->des_devfd);
	des_if->des_devfd = -1;

	return RET_OK;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief get deserial driver ioctl name by cmd
 *
 * @param[in] cmd: the ioctl cmd
 *
 * @return !NULL:the ioctl name string
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static const char *camera_deserial_dev_ioc_name(int32_t cmd)
{
	const char *deserial_ioc_names[] = DESERIAL_IOC_NAMES;

	int32_t nr = (_IOC_NR(cmd) < ARRAY_SIZE(deserial_ioc_names)) ? _IOC_NR(cmd) : -1;
	const char *ioc_name = (nr < 0) ? "unknown" : deserial_ioc_names[_IOC_NR(cmd)];

	return ioc_name;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief deserial driver ioctl operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] int32_t cmd: ioctl cmd
 * @param[in] void* arg: ioctl arg
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
static int32_t camera_deserial_dev_ioctl(deserial_info_t *des_if, int32_t cmd, void *arg)
{
	int32_t ret;

	if (camera_deserial_dev_nodrv(des_if)) {
		cam_dbg("deserial%d %s ioctl %s no driver as ok\n",
			des_if->index, des_if->deserial_name, camera_deserial_dev_ioc_name(cmd));
		return RET_OK;
	}
	if (des_if->des_devfd <= 0) {
		cam_err("deserial%d %s ioctl %s not open error\n",
			des_if->index, des_if->deserial_name, camera_deserial_dev_ioc_name(cmd));
		return -RET_ERROR;
	}

	ret = ioctl(des_if->des_devfd, cmd, arg);
	if (ret < 0) {
		ret = errno;
		cam_dbg("deserial%d %s ioctl %s %p ret %d: %s\n",
			des_if->index, des_if->deserial_name, camera_deserial_dev_ioc_name(cmd),
			arg, -ret, strerror(ret));
		return -ret;
	}

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver hardware info init
 *
 * @param[in] des_if: deserial info struct
 * @param[in] fill: deserial info data callback func for fill
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
int32_t camera_deserial_dev_info_init(deserial_info_t *des_if, DESERIAL_INFO_DATA_FILL_FUNC fill)
{
	int32_t ret = RET_OK;
	const char *des_gpio_names[] = { VCON_GPIO_NAMES_DES };
	deserial_info_data_t data = { 0 };
	deserial_handle_st *hdes = NULL;
	sensor_info_t *sen_if;
	poc_info_t *poc_if;
	int32_t i, gpio_num, gpio;
	int32_t dindex;
	char *dname;
	const char *emode_name;

	if (des_if == NULL)
		return -RET_ERROR;
	dindex = des_if->index;
	dname = des_if->deserial_name;
	poc_if = (poc_info_t *)des_if->poc_info;

	cam_dbg("deserial%d %s i2c%d@0x%02x dev info init\n",
		dindex, dname, des_if->bus_num, des_if->deserial_addr);

	ret = camera_run_des_get(dindex, NULL, &hdes, NULL, NULL);
	if ((ret < 0) || (hdes == NULL)) {
		cam_err("deserial%d %s not in error\n", dindex, dname);
		return -RET_ERROR;
	}

	data.index = des_if->index;
	strncpy(data.deserial_name, dname, sizeof(data.deserial_name) - 1);
	data.deserial_addr = des_if->deserial_addr;

	if (poc_if != NULL) {
		strncpy(data.poc_name, poc_if->poc_name, sizeof(data.poc_name) - 1);
		data.poc_addr = poc_if->poc_addr;
		data.poc_map = poc_if->poc_map;
	}
	data.bus_num = des_if->bus_num;
	data.bus_type = des_if->bus_type;
	data.reg_width = CAM_MODULE_GET_FLAG_ALEN(DESERIAL_FLAGS(des_if));
	gpio_num = (DESERIAL_GPIO_NUM_MAX < VGPIO_DES_NUM) ? DESERIAL_GPIO_NUM_MAX : VGPIO_DES_NUM;
	for (i = 0; i < gpio_num; i++) {
		ret = camera_vpf_vin_get_gpio(&hdes->vin_attr, des_if->gpio_enable,
					des_if->gpio_levels, VGPIO_DES_BASE + 1, &gpio, NULL);
		if (ret < 0) {
			cam_err("deserial%d %s gpio %s get error %d\n",
				dindex, dname, des_gpio_names[i], ret);
			return ret;
		}
	}
	data.gpio_enable = des_if->gpio_enable;
	data.gpio_level = des_if->gpio_levels;
	data.link_map = hdes->des_config.link_map;
	for (i = 0; i < DESERIAL_LINK_NUM_MAX; i++) {
		sen_if = (sensor_info_t *)des_if->sensor_info[i];
		if (sen_if != NULL) {
			data.sensor_index[i] = sen_if->port;
			emode_name = SENSOR_EMODE_NAME(sen_if);
			if (emode_name != NULL)
				snprintf(data.link_desp[i], DESERIAL_PORT_DESPLEN, "%s:%s",
					sen_if->sensor_name, emode_name);
			else
				strncpy(data.link_desp[i], sen_if->sensor_name, DESERIAL_PORT_DESPLEN);
		} else {
			data.sensor_index[i] = -1;
			if (des_if->port_desp[i] != NULL)
				strncpy(data.link_desp[i], des_if->port_desp[i], DESERIAL_PORT_DESPLEN);
		}
	}

	/* to fill by custom:
	 * chip_addr: the chip id reg addr
	 * chip_id: the chip id to read verify auto
	 * init/start/stop/deinit: operation reg list
	 */
	if (fill != NULL) {
		ret = fill(des_if, &data);
		if (ret < 0) {
			cam_err("deserial%d %s fill data_info error %d\n",
				dindex, des_if->deserial_name, ret);
			return ret;
		}
	}

	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_DATA_INIT, &data);
	if (ret < 0) {
		cam_err("deserial%d %s %s error %d\n", dindex, dname,
			camera_deserial_dev_ioc_name(DESERIAL_DATA_INIT), ret);
		return ret;
	}

	des_if->data_info_inited = 1;

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver request to init the hardware
 *
 * @param[in] des_if: deserial info struct
 * @param[in] timeout: request wait timeout ms
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
int32_t camera_deserial_dev_init_req(deserial_info_t *des_if, int32_t timeout)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_INIT_REQ, &timeout);
	if (ret == -RET_ERROR)
		cam_err("deserial%d %s %s %d error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_INIT_REQ), timeout, ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver set result back after init operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] result: init result: 0-init done, <0-init failed
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
int32_t camera_deserial_dev_init_result(deserial_info_t *des_if, int32_t result)
{
	int32_t ret;

	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_INIT_RESULT, &result);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_INIT_RESULT), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver requeset deinit and not need result
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
int32_t camera_deserial_dev_deinit(deserial_info_t *des_if)
{
	int32_t ret;

	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_DEINIT_REQ, NULL);
	if (ret == -RET_ERROR)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_DEINIT_REQ), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver request start to hardware
 *
 * @param[in] des_if: deserial info struct
 * @param[in] timeout: request wait timeout ms
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
int32_t camera_deserial_dev_start_req(deserial_info_t *des_if, int32_t timeout)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_START_REQ, &timeout);
	if (ret == -RET_ERROR)
		cam_err("deserial%d %s %s %d error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_START_REQ), timeout, ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver set result back after start operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] result: start result: 0-init done, <0-init failed
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
int32_t camera_deserial_dev_start_result(deserial_info_t *des_if, int32_t result)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_START_RESULT, &result);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_START_RESULT), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver request stop and not need result
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
int32_t camera_deserial_dev_stop(deserial_info_t *des_if)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STOP_REQ, NULL);
	if (ret == -RET_ERROR)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STOP_REQ), ret);

	return ret;
}

/**
 * @NO{S10E02C07}
 * @ASIL{B}
 * @brief deserial dev pre request operation
 *
 * @param[in] deserial_index: deerial info struct
 * @param[in] type: 0-init request, 1-start request
 * @param[in] timeout: the timeout ms
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
int32_t camera_deserial_dev_pre_req(int32_t deserial_index, int32_t type, int32_t timeout)
{
	int32_t ret;
	deserial_info_t *des_if;

	ret = camera_run_des_get(deserial_index, NULL, NULL, &des_if, NULL);
	if ((ret < 0) || (des_if == NULL))
		return RET_OK;
	if (type == 0)
		ret = camera_deserial_dev_init_req(des_if, timeout);
	else
		ret = camera_deserial_dev_start_req(des_if, timeout);

	return ret;
}

/**
 * @NO{S10E02C07}
 * @ASIL{B}
 * @brief deserial dev pre result operation
 *
 * @param[in] deserial_index: deserial info struct
 * @param[in] type: 0-init reseult, 1-start result
 * @param[in] result: 0-OK, <0-ERROR
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
int32_t camera_deserial_dev_pre_result(int32_t deserial_index, int32_t type, int32_t result)
{
	int32_t ret;
	deserial_info_t *des_if;

	ret = camera_run_des_get(deserial_index, NULL, NULL, &des_if, NULL);
	if ((ret < 0) || (des_if == NULL))
		return RET_OK;
	if (type == 0)
		ret = camera_deserial_dev_init_result(des_if, result);
	else
		ret = camera_deserial_dev_start_result(des_if, result);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver stream operation get (wait event return)
 *
 * @param[in] des_if: deserial info struct
 * @param[in] op_info: the deserial stream operation info struct
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
int32_t camera_deserial_dev_stream_get(deserial_info_t *des_if, deserial_op_info_t *op_info)
{
	int32_t ret;

	if ((des_if == NULL) || (op_info == NULL))
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STREAM_GET, op_info);
	if (ret < 0) {
		if (ret == (-ESRCH))
			cam_dbg("deserial%d %s %s cancel\n", des_if->index, des_if->deserial_name,
				camera_deserial_dev_ioc_name(DESERIAL_STREAM_GET));
		else
			cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
				camera_deserial_dev_ioc_name(DESERIAL_STREAM_GET), ret);
	}

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver stream operation put (set result back)
 *
 * @param[in] des_if: deserial info struct
 * @param[in] result: the result for get
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
int32_t camera_deserial_dev_stream_put(deserial_info_t *des_if, int32_t result)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STREAM_PUT, &result);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STREAM_PUT), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver stream on operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] link: the deserial link to operation
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
int32_t camera_deserial_dev_stream_on(deserial_info_t *des_if, int32_t link)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STREAM_ON, &link);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STREAM_ON), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver stream off operation
 *
 * @param[in] des_if: deserial info struct
 * @param[in] link: the deserial link to operation
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
int32_t camera_deserial_dev_stream_off(deserial_info_t *des_if, int32_t link)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STREAM_OFF, &link);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STREAM_OFF), ret);

	return ret;
}

/**
 * @NO{S10E02C05I}
 * @ASIL{B}
 * @brief deserial driver version info get
 *
 * @param[in] des_if: deserial info struct
 * @param[out] ver: the version info to store
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
int32_t camera_deserial_dev_get_version(deserial_info_t *des_if, deserial_version_info_t *ver)
{
	int32_t ret;

	if ((des_if == NULL) || (ver == NULL))
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_GET_VERSION, ver);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_GET_VERSION), ret);

	return ret;
}

int32_t camera_deserial_dev_state_check(deserial_info_t *des_if, uint32_t check_id)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STATE_CHECK, &check_id);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STATE_CHECK), ret);
	return ret;
}

int32_t camera_deserial_dev_state_confirm(deserial_info_t *des_if, uint32_t confirm_id)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STATE_CONFIRM, &confirm_id);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STATE_CONFIRM), ret);
	return ret;
}

int32_t camera_deserial_dev_state_clear(deserial_info_t *des_if, uint32_t clear_id)
{
	int32_t ret;

	if (des_if == NULL)
		return -RET_ERROR;
	ret = camera_deserial_dev_ioctl(des_if, DESERIAL_STATE_CLEAR, &clear_id);
	if (ret < 0)
		cam_err("deserial%d %s %s error %d\n", des_if->index, des_if->deserial_name,
			camera_deserial_dev_ioc_name(DESERIAL_STATE_CLEAR), ret);
	return ret;
}
