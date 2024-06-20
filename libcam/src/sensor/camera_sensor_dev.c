/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_sensor_dev.c
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#define pr_mod	"sensor_dev"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "hb_camera_data_info.h"
#include "hb_camera_error.h"

#include "cam_sensor_lib.h"

#include "camera_log.h"
#include "camera_env.h"
#include "camera_sensor_dev.h"
#include "camera_sensor_dev_ioctl.h"

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver open and prepare to work
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
int32_t camera_sensor_dev_open(sensor_info_t *sen_if)
{
	int32_t ret;
	const char *sname;
	char sen_dev[SENSOR_DEV_NAME_LEN];
	sensor_version_info_t ver = { 0 };

	if (sen_if == NULL)
		return -RET_ERROR;
	if (sen_if->sen_devfd > 0)
		return RET_OK;
	sname = sen_if->sensor_name;

	snprintf(sen_dev, sizeof(sen_dev), SENSOR_DEV_PATH, sen_if->port);

	if (camera_env_get_bool(CAMENV_DRIVER_NOSENSOR, FALSE) != FALSE) {
		sen_if->sen_devfd = -2;
		cam_warn("open %s %s no driver as %d\n", sen_dev, sname, sen_if->sen_devfd);
		return RET_OK;
	}

	ret = open((const char *)sen_dev, O_RDWR);
	if (ret < 0) {
		cam_err("open %s %s error %d\n", sen_dev, sname, ret);
		return ret;
	}
	sen_if->sen_devfd = ret;

	/* driver version check */
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		/* driver version should >= lib driver version */
		ret = camera_sensor_dev_get_version(sen_if, &ver);
		if ((ret < 0) || ((ver.major < SENSOR_VER_MAJOR)
#if SENSOR_VER_MINOR
			|| ((ver.major == SENSOR_VER_MAJOR) && (ver.minor < SENSOR_VER_MINOR))
#endif
			)) {
			if (ret == 0) {
				cam_err("check %s driver v%u.%u < v%u.%u error\n", sen_dev,
					ver.major, ver.minor, SENSOR_VER_MAJOR, SENSOR_VER_MINOR);
				ret = -RET_ERROR;
			}
			close(sen_if->sen_devfd);
			sen_if->sen_devfd = -1;
			return ret;
		}
		cam_dbg("open %s v%u.%u %s as %d\n", sen_dev,
			ver.major, ver.minor, sname, sen_if->sen_devfd);
	} else {
		cam_dbg("open %s %s as %d\n", sen_dev, sname, sen_if->sen_devfd);
	}
	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver is run in no driver mode?
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
int32_t camera_sensor_dev_nodrv(sensor_info_t *sen_if)
{
	return ((sen_if == NULL) || (sen_if->sen_devfd < -1)) ? 1 : 0;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver close and exit
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
int32_t camera_sensor_dev_close(sensor_info_t *sen_if)
{
	if (sen_if == NULL)
		return -RET_ERROR;
	if (camera_sensor_dev_nodrv(sen_if)) {
		cam_dbg("close " SENSOR_DEV_PATH " %s no driver as %d\n",
			sen_if->port, sen_if->sensor_name, sen_if->sen_devfd);
		sen_if->sen_devfd = -1;
		return RET_OK;
	}
	if (sen_if->sen_devfd <= 0)
		return RET_OK;

	cam_dbg("close " SENSOR_DEV_PATH " %s as %d\n",
		sen_if->port, sen_if->sensor_name, sen_if->sen_devfd);
	close(sen_if->sen_devfd);
	sen_if->sen_devfd = -1;

	return 0;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get sensor driver ioctl name by cmd
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
static const char *camera_sensor_dev_ioc_name(int32_t cmd)
{
	const char *sensor_ioc_names[] = SENSOR_IOC_NAMES;

	int32_t nr = (_IOC_NR(cmd) < ARRAY_SIZE(sensor_ioc_names)) ? _IOC_NR(cmd) : -1;
	const char *ioc_name = (nr < 0) ? "unknown" : sensor_ioc_names[_IOC_NR(cmd)];

	return ioc_name;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor driver ioctl operation
 *
 * @param[in] sen_if: sensor info struct
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
static int32_t camera_sensor_dev_ioctl(sensor_info_t *sen_if, int32_t cmd, void *arg)
{
	int32_t ret;

	if (camera_sensor_dev_nodrv(sen_if)) {
		cam_dbg("sensor%d %s ioctl %s no driver as ok\n",
			sen_if->port, sen_if->sensor_name, camera_sensor_dev_ioc_name(cmd));
		return RET_OK;
	}
	if (sen_if->sen_devfd <= 0) {
		cam_err("sensor%d %s ioctl %s not open error\n",
			sen_if->port, sen_if->sensor_name, camera_sensor_dev_ioc_name(cmd));
		return -RET_ERROR;
	}

	ret = ioctl(sen_if->sen_devfd, cmd, arg);
	if (ret < 0) {
		ret = errno;
		cam_dbg("sensor%d %s ioctl %s %p ret %d: %s\n",
			sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(cmd), arg, -ret, strerror(ret));
		return -ret;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro tuning data struct updata to driver
 *
 * @param[in] sen_if: sensor_info struct
 * @param[out] pdata: tuning data struct to update
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
int32_t camera_sensor_dev_tuning_init(sensor_info_t *sen_if, sensor_tuning_data_t *pdata)
{
	int32_t ret;

	if ((sen_if == NULL) || (pdata == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_TUNING_PARAM, pdata);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_TUNING_PARAM), ret);

	return ret;
}


/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro tuning data struct fill with self-defined function
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] snsf: sensor base info fill function
 * @param[in] awbf: sensor awb info fill function
 * @param[in] aef: sensor ae info fill function
 * @param[in] ctlf: sensor ctrl info fill function
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
int32_t camera_sensor_dev_tuning_fill_init(sensor_info_t *sen_if, sensor_data_fill snsf, sensor_awb_fill awbf, sensor_ae_fill aef, sensor_ctrl_fill ctlf)
{
	int32_t ret = 0;
	sensor_tuning_data_t pdata = { 0 };

	if (sen_if == NULL)
		return -RET_ERROR;

	if ((ret == 0) && (snsf != NULL))
		ret = snsf(sen_if, &pdata.sensor_data);
	if ((ret == 0) && (awbf != NULL))
		ret = awbf(sen_if, &pdata.sensor_awb);
	if ((ret == 0) && (aef != NULL))
		ret = aef(sen_if, NULL /*&pdata->sensor_ae*/);
	if ((ret == 0) && (aef != NULL)) {
		switch (sen_if->sensor_mode) {
		case NORMAL_M:
			ret = aef(sen_if, &pdata.normal);
			break;
		case DOL2_M:
			ret = aef(sen_if, &pdata.dol2);
			break;
		case DOL3_M:
			ret = aef(sen_if, &pdata.dol3);
			break;
		case PWL_M:
			ret = aef(sen_if, &pdata.pwl);
			break;
		case DOL4_M:
		default:
			cam_err("sensor%d %s tuning fill sensor_mode %d not support\n",
				sen_if->port, sen_if->sensor_name, sen_if->sensor_mode);
			return -RET_ERROR;
		}
	}

	if (ret < 0) {
		cam_err("sensor%d %s tuning fill sensor_mode %d error\n",
			sen_if->port, sen_if->sensor_name, sen_if->sensor_mode);
		return ret;
	}
	ret = camera_sensor_dev_tuning_init(sen_if, &pdata);
	if (ret < 0)
		cam_err("sensor%d %s tuning fill sensor_mode %d init error\n",
			sen_if->port, sen_if->sensor_name, sen_if->sensor_mode);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver set ae share
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] ae_share_flag: ae share flage to set
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
int32_t camera_sensor_dev_ae_share(sensor_info_t *sen_if, uint32_t ae_share_flag)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_AE_SHARE, &ae_share_flag);
	if (ret < 0)
		cam_err("sensor%d %s %s 0x%x error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_AE_SHARE), ae_share_flag, ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver set input param
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] ae_share_flag: ae share flage to set
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
int32_t camera_sensor_dev_input_param(sensor_info_t *sen_if, sensor_input_param_t *input_param)
{
	int32_t ret;

	if ((sen_if == NULL) || (input_param == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_INPUT_PARAM, input_param);
	if (ret < 0)
		cam_err("sensor%d %s %s %d error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_INPUT_PARAM), input_param->ts_compensate, ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver set intrinsic param
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] sp: the user sensor parm struct
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
int32_t camera_sensor_dev_set_intrinsic_param(sensor_info_t *sen_if,  cam_usr_info_t *sp)
{
	int32_t ret;

	if ((sen_if == NULL) || (sp == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_SET_INTRINSIC_PARAM, sp);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_SET_INTRINSIC_PARAM), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver get intrinsic param
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] sp: the user sensor parm struct
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
int32_t camera_sensor_dev_get_intrinsic_param(sensor_info_t *sen_if,  cam_usr_info_t *sp)
{
	int32_t ret;

	if ((sen_if == NULL) || (sp == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_GET_INTRINSIC_PARAM, sp);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_GET_INTRINSIC_PARAM), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver request to do hardware init operation
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
int32_t camera_sensor_dev_init_req(sensor_info_t *sen_if)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_INIT_REQ, NULL);
	if (ret == -RET_ERROR)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_INIT_REQ), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver set result of init operation to driver
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] result: 0-init done, <0-init failed
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
int32_t camera_sensor_dev_init_result(sensor_info_t *sen_if, int32_t result)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_INIT_RESULT, &result);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_INIT_RESULT), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver request to do hardware deinit operation and not need result
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
int32_t camera_sensor_dev_deinit(sensor_info_t *sen_if)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_DEINIT_REQ, NULL);
	if (ret == -RET_ERROR)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_DEINIT_REQ), ret);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver stream start operation to call
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
int32_t camera_sensor_dev_start(sensor_info_t *sen_if)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_START, NULL);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_START), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver stream stop operation to call
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
int32_t camera_sensor_dev_stop(sensor_info_t *sen_if)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_STOP, NULL);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_STOP), ret);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver get event call
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] event: event info struct
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
int32_t camera_sensor_dev_event_get(sensor_info_t *sen_if, sensor_event_info_t *event)
{
	int32_t ret;

	if ((sen_if == NULL) || (event == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_EVENT_GET, event);
	if (ret < 0) {
		if (ret == (-ESRCH))
			cam_dbg("sensor%d %s %s cancel\n", sen_if->port, sen_if->sensor_name,
				camera_sensor_dev_ioc_name(SENSOR_EVENT_GET));
		else
			cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
				camera_sensor_dev_ioc_name(SENSOR_EVENT_GET), ret);
	}

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver set event result call
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] result: event result to set
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
int32_t camera_sensor_dev_event_put(sensor_info_t *sen_if, int32_t result)
{
	int32_t ret;

	if (sen_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_EVENT_PUT, &result);
	if (ret < 0)
		cam_err("sensor%d %s %s %d error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_EVENT_PUT), result, ret);
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensro driver update ae info
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] ae_info: ae info struct to update
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
int32_t camera_sensor_dev_update_ae_info(sensor_info_t *sen_if, camera_ae_info_t *ae_info)
{
	int32_t ret;

	if ((sen_if == NULL) || (ae_info == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_UPDATE_AE_INFO, ae_info);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_UPDATE_AE_INFO), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor driver version info get
 *
 * @param[in] sen_if: sensor info struct
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
int32_t camera_sensor_dev_get_version(sensor_info_t *sen_if, sensor_version_info_t *ver)
{
	int32_t ret;

	if ((sen_if == NULL) || (ver == NULL))
		return -RET_ERROR;
	ret = camera_sensor_dev_ioctl(sen_if, SENSOR_GET_VERSION, ver);
	if (ret < 0)
		cam_err("sensor%d %s %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_dev_ioc_name(SENSOR_GET_VERSION), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor ctrl driver open to prepare to work for userspace control
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
int32_t camera_sensor_cdev_open(sensor_info_t *sen_if)
{
	int32_t ret;
	int32_t sindex;
	const char *sname;
	const char *cdev_path = SENSOR_CDEV_PATH;
	sensor_version_info_t ver = { 0 };

	if (sen_if == NULL)
		return -RET_ERROR;
	if (sen_if->sen_cdevfd > 0)
		return RET_OK;
	sindex = sen_if->port;
	sname = sen_if->sensor_name;

	ret = open(cdev_path, O_RDWR);
	if (ret < 0) {
		cam_err("open %s for sensor%d %s error %d\n",
			cdev_path, sindex, sname, ret);
		return ret;
	}
	sen_if->sen_cdevfd = ret;

	/* driver version check */
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		ret = camera_sensor_cdev_get_version(sen_if, &ver);
		if ((ret < 0) || ((ver.major < SENSOR_CTRL_VER_MAJOR)
#if SENSOR_CTRL_VER_MINOR
			|| ((ver.major == SENSOR_CTRL_VER_MAJOR) && (ver.minor < SENSOR_CTRL_VER_MINOR))
#endif
			)) {
			if (ret == 0) {
				cam_err("check %s driver v%u.%u < v%u.%u error\n", cdev_path,
					ver.major, ver.minor, SENSOR_CTRL_VER_MAJOR, SENSOR_CTRL_VER_MINOR);
				ret = -RET_ERROR;
			}
			close(sen_if->sen_cdevfd);
			sen_if->sen_cdevfd = -1;
			return ret;
		}
		cam_dbg("open %s v%u.%u for sensor%d %s as %d\n",
			cdev_path, ver.major, ver.minor, sindex, sname, sen_if->sen_cdevfd);
	} else {
		cam_dbg("open %s for sensor%d %s as %d\n",
			cdev_path, sindex, sname, sen_if->sen_cdevfd);
	}
	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor ctrl driver close to exit
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
int32_t camera_sensor_cdev_close(sensor_info_t *sen_if)
{
	if (sen_if == NULL)
		return -RET_ERROR;
	if (sen_if->sen_cdevfd <= 0)
		return RET_OK;

	cam_dbg("close " SENSOR_CDEV_PATH " for sensor%d %s as %d\n",
		sen_if->port, sen_if->sensor_name, sen_if->sen_cdevfd);
	close(sen_if->sen_cdevfd);
	sen_if->sen_cdevfd = -1;

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get sensor ctrl driver ioctl name by cmd
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
static const char *camera_sensor_cdev_ioc_name(int32_t cmd)
{
	const char *sensor_ctrl_ioc_names[] = SENSOR_CTRL_IOC_NAMES;
	int32_t nr = _IOC_NR(cmd) - CAMERA_CTRL_IOC_BASE;

	nr = (nr < ARRAY_SIZE(sensor_ctrl_ioc_names)) ? nr : -1;
	const char *ioc_name = (nr < 0) ? "unknown" : sensor_ctrl_ioc_names[nr];

	return ioc_name;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor ctrl driver ioctl operation
 *
 * @param[in] sen_if: sensor info struct
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
static int32_t camera_sensor_cdev_ioctl(sensor_info_t *sen_if, int32_t cmd, void *arg)
{
	int32_t ret;

	if (sen_if->sen_cdevfd <= 0) {
		cam_err("sensor%d %s ctrl ioctl %s not open error\n",
			sen_if->port, sen_if->sensor_name, camera_sensor_cdev_ioc_name(cmd));
		return -RET_ERROR;
	}

	ret = ioctl(sen_if->sen_cdevfd, cmd, arg);
	if (ret < 0) {
		ret = errno;
		cam_dbg("sensor%d %s ctrl ioctl %s %p ret %d: %s\n",
			sen_if->port, sen_if->sensor_name,
			camera_sensor_cdev_ioc_name(cmd), arg, -ret, strerror(ret));
		return -ret;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor ctrl wait control info update
 *
 * @param[in] sen_if: sensor_info struct
 * @param[out] info: control info struct to store
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
int32_t camera_sensor_cdev_info_sync(sensor_info_t *sen_if, sensor_ctrl_info_t *info)
{
	int32_t ret;

	if ((sen_if == NULL) || (info == NULL))
		return -RET_ERROR;
	ret = camera_sensor_cdev_ioctl(sen_if, SENSOR_CTRL_INFO_SYNC, info);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor ctrl set control result
 *
 * @param[in] sen_if: sensor_info struct
 * @param[in] res: control result struct to set

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
extern int32_t camera_sensor_cdev_result(sensor_info_t *sen_if, sensor_ctrl_result_t *res)
{
	int32_t ret;

	if ((sen_if == NULL) || (res == NULL))
		return -RET_ERROR;
	ret = camera_sensor_cdev_ioctl(sen_if, SENSOR_CTRL_RESULT, res);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor ctrl driver version info get
 *
 * @param[in] sen_if: sensor info struct
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
int32_t camera_sensor_cdev_get_version(sensor_info_t *sen_if, sensor_version_info_t *ver)
{
	int32_t ret;

	if ((sen_if == NULL) || (ver == NULL))
		return -RET_ERROR;
	ret = camera_sensor_cdev_ioctl(sen_if, SENSOR_CTRL_GET_VERSION, ver);
	if (ret < 0)
		cam_err("sensor%d %s ctrl %s error %d\n", sen_if->port, sen_if->sensor_name,
			camera_sensor_cdev_ioc_name(SENSOR_CTRL_GET_VERSION), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor calib(iq) dev open driver
 *
 * @param[in] cal_if: calib_info struct
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
int32_t camera_sensor_idev_open(calib_info_t *cal_if)
{
	int32_t ret;
	int32_t sindex;
	const char *sname;
	const char *idev_path = SENSOR_IDEV_PATH;
	sensor_version_info_t ver = { 0 };

	if (cal_if == NULL)
		return -RET_ERROR;
	if (cal_if->cal_idevfd > 0)
		return RET_OK;
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	ret = open(idev_path, O_RDWR);
	if (ret < 0) {
		cam_err("open %s for sensor%d %s error %d\n",
			idev_path, sindex, sname, ret);
		return ret;
	}
	cal_if->cal_idevfd = ret;

	/* driver version check */
	if (camera_env_get_bool(CAMENV_DRIVER_NOVERSION, FALSE) == FALSE) {
		ret = camera_sensor_idev_get_version(cal_if, &ver);
		if ((ret < 0) || ((ver.major < SENSOR_IQ_VER_MAJOR)
#if SENSOR_IQ_VER_MINOR
			((ver.major == SENSOR_IQ_VER_MAJOR) && (ver.minor < SENSOR_IQ_VER_MINOR))
#endif
			)) {
			if (ret == 0) {
				cam_err("check %s driver v%u.%u < v%u.%u error\n", idev_path,
					ver.major, ver.minor, SENSOR_IQ_VER_MAJOR, SENSOR_IQ_VER_MINOR);
				ret = -RET_ERROR;
			}
			close(cal_if->cal_idevfd);
			cal_if->cal_idevfd = -1;
			return ret;
		}
		cam_dbg("open %s v%u.%u for sensor%d %s as %d\n",
			idev_path, ver.major, ver.minor, sindex, sname, cal_if->cal_idevfd);
	} else {
		cam_dbg("open %s for sensor%d %s as %d\n",
			idev_path, sindex, sname, cal_if->cal_idevfd);
	}

	return RET_OK;

}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor calib(iq) dev close driver
 *
 * @param[in] cal_if: calib_info struct
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
int32_t camera_sensor_idev_close(calib_info_t *cal_if)
{
	if (cal_if == NULL)
		return -RET_ERROR;
	if (cal_if->cal_idevfd <= 0)
		return RET_OK;

	cam_dbg("close " SENSOR_IDEV_PATH " for sensor%d %s as %d\n",
		cal_if->port, cal_if->sensor_name, cal_if->cal_idevfd);
	close(cal_if->cal_idevfd);
	cal_if->cal_idevfd = -1;

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get sensor iq driver ioctl name by cmd
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
static const char *camera_sensor_idev_ioc_name(int32_t cmd)
{
	const char *sensor_iq_ioc_names[] = SENSOR_IQ_IOC_NAMES;

	int32_t nr = (_IOC_NR(cmd) < ARRAY_SIZE(sensor_iq_ioc_names)) ? _IOC_NR(cmd) : -1;
	const char *ioc_name = (nr < 0) ? "unknown" : sensor_iq_ioc_names[_IOC_NR(cmd)];

	return ioc_name;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief sensor iq driver ioctl operation
 *
 * @param[in] cal_if: calib_info struct
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
static int32_t camera_sensor_idev_ioctl(calib_info_t *cal_if, int32_t cmd, void *arg)
{
	int32_t ret;

	if (cal_if->cal_idevfd <= 0) {
		cam_err("sensor%d %s iq ioctl %s not open error\n",
			cal_if->port, cal_if->sensor_name, camera_sensor_idev_ioc_name(cmd));
		return -RET_ERROR;
	}

	ret = ioctl(cal_if->cal_idevfd, cmd, arg);
	if (ret < 0) {
		ret = errno;
		cam_dbg("sensor%d %s iq ioctl %s %p ret %d: %s\n",
			cal_if->port, cal_if->sensor_name,
			camera_sensor_idev_ioc_name(cmd), arg, -ret, strerror(ret));
		return -ret;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor calib(irq) gget total size and check if valid
 *
 * @param[in] cal_if: calib_info struct
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
int32_t camera_sensor_idev_totalsize(calib_info_t *cal_if)
{
	int32_t ret = 0u;
	uint32_t i = 0u;
	uint32_t total_size = 0u;

	if (cal_if == NULL)
		return -RET_ERROR;
	ret = camera_sensor_idev_ioctl(cal_if, AC_CALIB_TOTAL_SIZE, &total_size);
	if (ret < 0) {
		cam_err("sensor%d %s iq %s error %d\n", cal_if->port, cal_if->sensor_name,
			camera_sensor_idev_ioc_name(AC_CALIB_TOTAL_SIZE), ret);
		return ret;
	}
	if (total_size > LIBISP_CALIBRATION_TOTAL_SIZE) {
		cam_err("sensor%d %s iq total size %d > %d error\n",
			cal_if->port, cal_if->sensor_name,
			total_size, LIBISP_CALIBRATION_TOTAL_SIZE);
		return -RET_ERROR;
	}

	for (i = 0; i < CALIBRATION_MULTI_NUM; i++) {
		cal_if->calib_total_size[i] = total_size;
	}
	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor calib(irq) init by calib data update to driver
 *
 * @param[in] cal_if: calib_info struct
 * @param[in] pcalib: camera calib struct
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
int32_t camera_sensor_idev_init(calib_info_t *cal_if, camera_calib_t *pcalib)
{
	int32_t ret;

	if ((cal_if == NULL) || (pcalib == NULL))
		return -RET_ERROR;
	ret = camera_sensor_idev_ioctl(cal_if, AC_CALIB_INIT, pcalib);
	if (ret < 0)
		cam_err("sensor%d %s iq %s error %d\n", cal_if->port, cal_if->sensor_name,
			camera_sensor_idev_ioc_name(AC_CALIB_INIT), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor calib(irq) deinit by empty calib data update to driver
 *
 * @param[in] cal_if: calib_info struct
 * @param[in] pcalib: empty camera calib struct
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
int32_t camera_sensor_idev_deinit(calib_info_t *cal_if, camera_calib_t *pcalib)
{
	int32_t ret;

	if ((cal_if == NULL) || (pcalib == NULL))
		return -RET_ERROR;
	ret = camera_sensor_idev_ioctl(cal_if, AC_CALIB_RELEASE, pcalib);
	if (ret < 0)
		cam_err("sensor%d %s iq %s error %d\n", cal_if->port, cal_if->sensor_name,
			camera_sensor_idev_ioc_name(AC_CALIB_RELEASE), ret);

	return ret;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief sensor iq driver version info get
 *
 * @param[in] cal_if: calib_info struct
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
int32_t camera_sensor_idev_get_version(calib_info_t *cal_if, sensor_version_info_t *ver)
{
	int32_t ret;

	if ((cal_if == NULL) || (ver == NULL))
		return -RET_ERROR;
	ret = camera_sensor_idev_ioctl(cal_if, AC_CALIB_GET_VERSION, ver);
	if (ret < 0)
		cam_err("sensor%d %s iq %s error %d\n", cal_if->port, cal_if->sensor_name,
			camera_sensor_idev_ioc_name(AC_CALIB_GET_VERSION), ret);

	return ret;
}

int32_t camera_sensor_isi_dev_open(camera_module_lib_t *cal_lib)
{
	int32_t ret = RET_OK;

	if (cal_lib == NULL)
		return -RET_ERROR;
	if (cal_lib->so_fd > 0)
		return RET_OK;

	ret = open(SENSOR_IDEV_PATH, O_RDWR);
	if (ret < 0) {
		cam_err("open %s failed \n", SENSOR_IDEV_PATH);
		return -RET_ERROR;
	}

	cal_lib->so_fd = ret;

	return RET_OK;
}

int32_t camera_sensor_isi_dev_close(camera_module_lib_t *cal_lib)
{
	int32_t ret = RET_OK;

	if (cal_lib == NULL)
		return -RET_ERROR;
	if (cal_lib->so_fd <= 0)
		return RET_OK;

	close(cal_lib->so_fd);
	cal_lib->so_fd = -1;

	return ret;
}

int32_t camera_sensor_isi_dev_data_put(camera_module_lib_t *cal_lib, camera_calib_t *pcalib)
{
	int32_t ret = RET_OK;

	if (cal_lib == NULL || pcalib == NULL)
		return -RET_ERROR;

	if (cal_lib->so_fd <= 0) {
		cam_err("camera cali get so_fd failed \n");
		return -RET_ERROR;
	}

	ret = ioctl(cal_lib->so_fd, AC_CALIB_INIT, pcalib);
	if (ret < 0) {
		cam_err("%s ioctl fail, ret =%d, %s \n", __func__, ret, strerror(ret));
		return -RET_ERROR;
	}

	return ret;
}
