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

#define pr_mod	"sensor_calib"

#include <stdio.h>
#include <string.h>

#include "hb_camera_error.h"

#include "camera_mod_calib.h"
#include "camera_sensor_dev.h"
#include "camera_log.h"
#include "camera_env.h"

#include "cam_runtime.h"
#include "cam_debug.h"

/**
 * @var calibration_with_check_lut
 * calibration param check
 * total size is 200
 * uint8_t 1, uint16_t 2, uint32_t 4;
 */
static const uint32_t calibration_with_check_lut[] = CALIBRATION_WIDTH_CHECK_LUTS;

/**
 * @var calibration_name
 * calibration param name string array
 */
static const char *calibration_name[CALIBRATION_NAME_MAX + 1] = CALIBRATION_NAMES;

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief check the calib ko_version valid by lib module
 *
 * @param[in] module: the module struct of calib lib
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
static int32_t camera_calib_ko_version_check(const camera_module_t *module)
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
			cam_dbg("calib %s v%s ko_ver skip check\n", module->name, so_ver);
			return 0;
		}
		/* ko(of lib) version show only here */
		cam_dbg("calib %s v%s ko v%u.%u\n", module->name, so_ver,
			ko_ver->major, ko_ver->minor);
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief version string compare
 *
 * @param[in] ver1: version string 1
 * @param[in] ver2: version string 2
 *
 * @return 0:ver1 == ver2, <0:ver1 < ver2, >0:ver1 > ver2
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t camera_calib_version_cmp(const char *ver1, const char *ver2)
{
	int32_t i, n1 = 0, n2 = 0;
	int32_t v1[10] = { 0 };
	int32_t v2[10] = { 0 };
	const char *v;

	if ((ver1 == NULL) || (ver2 == NULL))
		return -1;

	if (strcmp(ver1, ver2) == 0)
		return 0;

	i = 0;
	v = ver1;
	while (i < ARRAY_SIZE(v1) && (*v != '\0')) {
		if ((*v >= '0') && (*v <= '9'))
			v1[i] = v1[i] * 10 + (*v - '0');
		else if ((*v == '.') || (*v == '-'))
			i++;
		else
			continue;
		v++;
	}
	n1 = i;

	i = 0;
	v = ver2;
	while (i < ARRAY_SIZE(v2) && (*v != '\0')) {
		if ((*v >= '0') && (*v <= '9'))
			v2[i] = v2[i] * 10 + (*v - '0');
		else if ((*v == '.') || (*v == '-'))
			i++;
		else
			continue;
		v++;
	}
	n2 = i;

	for (i = 0; (i < n1) || (i < n2); i++) {
		if (v1[i] == v2[i])
			continue;
		else
			return (v1[i] - v2[i]);
	}

	return 0;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief calib module check version if valid?
 *
 * @param[in] lib: the sensor module lib struct to used
 * @param[in] version: the calib base version to check
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
int32_t camera_calib_config_check(camera_module_lib_t *lib, const char *version)
{
	const char *lib_version = NULL, *lib_name = NULL;
	calib_module_legacy_t *ml = NULL;

	if (lib == NULL)
		return -RET_ERROR;

	if (version == NULL)
		return RET_OK;

	(void)camera_calib_ko_version_check(lib->module);

	if (camera_env_get_bool(CAMENV_CALVER_NOCHECK, FALSE) == TRUE)
		return RET_OK;

	cam_dbg("calib %s check with %s\n", lib->so_name, version);

	if (lib->module != NULL) {
		lib_version = CAMERA_MODULE_GET_VERSION(lib->module);
		lib_name = CALIB_MODULE_GET_SONAME(lib->module);
	} else {
		ml = (calib_module_legacy_t *)(lib->body);
		lib_name = ml->cname;
	}
	if ((lib_version == NULL) && (lib_name != NULL)) {
		lib_version = strrchr(lib_name, '_');
		if ((lib_version != NULL) && lib_version[1] == 'v')
			lib_version += 2;
		else
			lib_version = NULL;
	}

	if (camera_calib_version_cmp(lib_version, version) < 0) {
		cam_err("%s check %s error\n", lib_name, version);
		return -RET_ERROR;
	}
	cam_dbg("%s check %s done\n", lib_name, version);

	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camera calib ops info bind
 *
 * @param[in] hcam: camera handle struct
 * @param[out] cal_if: calib info struct
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
int32_t camera_calib_ops_bind(camera_handle_st *hcam, calib_info_t *cal_if)
{
	camera_module_lib_t *lib;

	if ((hcam == NULL) || (cal_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hcam);
	lib = &hcam->calib_lib;
	cam_dbg("calib %s ops bind\n", hcam->cam_config.name);

	cal_if->calib_ops = lib->body;
	if (lib->module == NULL)
		cal_if->is_legacy = 1;

	camera_debug_hcall_ro(hcam);
	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camera calib config parse
 *
 * @param[in] hcam: camera handle struct
 * @param[out] cal_if: calib info struct
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
int32_t camera_calib_config_parse(camera_handle_st *hcam, calib_info_t *cal_if)
{
	camera_vin_attr_t *vin;

	if ((hcam == NULL) || (cal_if == NULL))
		return -RET_ERROR;
	camera_debug_hcall_ri(hcam);
	vin = &hcam->vin_attr;
	cam_dbg("calib %s config parse\n", hcam->cam_config.name);

	cal_if->port = VIN_ATTR_TO_CAMERA_INDEX(vin);
	cal_if->sensor_name = hcam->cam_config.name;

	camera_debug_hcall_ro(hcam);
	return RET_OK;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief put calib data to free data of plut
 *
 * @param[in] cal_if: the calib info struct
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
static int32_t camera_calib_data_put(calib_info_t *cal_if)
{
	uint32_t j = 0u;
#ifdef CALIBRATION_GET_ASNEW
	uint32_t i;
#endif

	if (cal_if == NULL)
		return -RET_ERROR;

#ifdef CALIBRATION_GET_ASNEW
	for(j = 0U; j < cal_if->total_calib; j++) {
		for(i = 0U; i < cal_if->calib_total_size[j]; i++) {
			if (cal_if->plut[j][i].ptr != NULL)
				free(cal_if->plut[j][i].ptr);
		}
	}
#endif
	for(j = 0U; j < cal_if->total_calib; j++) {
		cal_if->tsize[j] = 0U;
	}

	return RET_OK;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief get the calibration name string
 *
 * @param[in] index: the calibration index
 *
 * @return !NULL:the name string
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static const char *camera_calib_name(int32_t index)
{
	if (index < CALIBRATION_NAME_MAX)
		return calibration_name[index];
	else
		return "CALIBRATION_UNKNOWN";
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief get calib data from module to plut
 *
 * @param[in] cal_if: the calib info struct
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
static int32_t camera_calib_data_get(calib_info_t *cal_if)
{
	int32_t ret = RET_OK;
	int32_t sindex;
	char *sname;
	calib_module_t *m = NULL;
	calib_module_legacy_t *ml = NULL;
	const char *calib_name;
	uint32_t i, j, dsize, over = 0;
	uint32_t total_width_size = sizeof(calibration_with_check_lut) / sizeof(uint32_t);
	uint32_t total_width_name_size = CALIBRATION_NAME_MAX;
	ACameraCalibrations pc = { 0 };
	LookupTable *lut;

	if ((cal_if == NULL) || (cal_if->calib_ops == NULL))
		return -RET_ERROR;
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	if (cal_if->is_legacy == 0) {
		m = CALIB_MODULE_T(cal_if->calib_ops);
		calib_name = CALIB_SONAME(cal_if);
		if (calib_name == NULL) {
			cam_err("sensor%d %s calib get name error\n",
				sindex, sname);
			return -RET_ERROR;
		}
		strncpy(cal_if->name, calib_name, sizeof(cal_if->name) - 1);
		if (m->get_calib_dynamic != NULL)
			m->get_calib_dynamic(&pc);
		if (m->get_calib_static != NULL)
			m->get_calib_static(&pc);
		if (m->get_calib_num != NULL) {
			m->get_calib_num(&cal_if->total_calib);
		} else {
			cal_if->total_calib = 1;
		}
	} else {
		ml = (calib_module_legacy_t *)(cal_if->calib_ops);
		strncpy(cal_if->name, ml->cname, sizeof(cal_if->name));
		if (ml->get_calib_dynamic != NULL)
			ml->get_calib_dynamic(&pc);
		if (ml->get_calib_static != NULL)
			ml->get_calib_static(&pc);
	}

	/* data get */
	cam_info("sensor%d %s calib get %s\n", sindex, sname, cal_if->name);
	memset(cal_if->tsize, 0, sizeof(cal_if->tsize));
	for(j = 0; j < cal_if->total_calib; j++) {
		for(i = 0; i < cal_if->calib_total_size[j]; i++) {
			lut = pc.calibrations[j][i];
			/* valid? */
			if (lut == NULL) {
				if (i < CALIBRATION_NAME_MAX)
					cam_dbg("sensor%d %s calib[%d]:%s not exist\n",
						sindex, sname, i, camera_calib_name(i));
				else if (over == 0)
					over = i;
				else if (i == (cal_if->calib_total_size[j] - 1))
					cam_dbg("sensor%d %s calib[%d-%d]:%s over null\n",
						sindex, sname, over, i, camera_calib_name(i));
				continue;
			}
			/* width check */
			if ((total_width_size >= cal_if->calib_total_size[j]) && (total_width_name_size >= cal_if->calib_total_size[j])) {
				if (lut->width != calibration_with_check_lut[i]) {
					cam_err("sensor%d %s calib[%d]:%s width %d error, should %d\n",
						sindex, sname, i, camera_calib_name(i), lut->width, calibration_with_check_lut[i]);
				}
			}
			/* ptr check */
			if(lut->ptr == NULL) {
				cam_dbg("sensor%d %s calib[%d]:%s ptr NULL error\n",
					sindex, sname, i, camera_calib_name(i));
				camera_calib_data_put(cal_if);
				return -RET_ERROR;
			}
			/* malloc and get data */
			dsize =  (uint32_t)(lut->width * lut->rows * lut->cols);
			cal_if->tsize[j] += dsize;
			(void)memcpy((void *)&(cal_if->plut[j][i]), (void *)lut, sizeof(LookupTable));
#ifdef CALIBRATION_GET_ASNEW
			cal_if->plut[j][i].ptr = malloc(dsize);
			if (cal_if->plut[j][i].ptr == NULL) {
				cam_dbg("sensor%d %s calib[%d]:%s malloc failed\n",
					sindex, sname, i, camera_calib_name(i));
				camera_calib_data_put(cal_if);
				return -RET_ERROR;
			}
			(void)memcpy(cal_if->plut[j][i].ptr, lut->ptr, dsize);
#endif
		}
		printf("calib[%d] -- tsize %d, calibr_total_size %d\n", j, cal_if->tsize[j], cal_if->calib_total_size[j]);
	}

	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camera calib data init update to idev
 *
 * @param[in] cal_if: the calib info struct
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
static int32_t camera_calib_data_init(calib_info_t *cal_if)
{
	int32_t ret;
	int32_t sindex;
	char *sname;
	uint32_t i = 0;
	camera_calib_t pcalib = { 0 };

	if (cal_if == NULL)
		return -RET_ERROR;
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	strncpy(pcalib.name, cal_if->name, sizeof(pcalib.name));
	pcalib.port = cal_if->port;
	pcalib.total_calib = cal_if->total_calib;
	for (i = 0; i < pcalib.total_calib; i++) {
		pcalib.calib_total_size[i] = cal_if->calib_total_size[i];
		pcalib.calib_mem_size[i] = cal_if->tsize[i];
		pcalib.plut[i] = &cal_if->plut[i][0];
	}

	ret = camera_sensor_idev_init(cal_if, &pcalib);
	if (ret < 0) {
		cam_err("sensor%d %s init calib[%d] num %d tsize: %d error %d\n",
			sindex, sname, pcalib.total_calib, cal_if->calib_total_size[0],
			pcalib.calib_mem_size[0], ret);
		return ret;
	}

	cam_info("sensor%d %s init calib[%d] num %d tsize: %d done\n",
		sindex, sname, pcalib.total_calib, cal_if->calib_total_size[0],
		pcalib.calib_mem_size[0], ret);
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camera calib data deiinit release to idev
 *
 * @param[in] cal_if: the calib info struct
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
static int32_t camera_calib_data_deinit(calib_info_t *cal_if)
{
	int32_t ret;
	int32_t sindex;
	char *sname;
	camera_calib_t pcalib = { 0 };

	if (cal_if == NULL)
		return -RET_ERROR;
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	pcalib.port = cal_if->port;
#ifndef HB_X5_CALI
	ret = camera_sensor_idev_deinit(cal_if, &pcalib);
	if (ret < 0) {
		cam_err("sensor%d %s calib idev deinit error %d\n",
			sindex, sname, ret);
		return ret;
	}
#endif
	cam_info("sensor%d %s calib idev deinit done\n", sindex, sname);
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camera calib so init do
 *
 * @param[in] cal_if: the calib info struct
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
int32_t camera_calib_init(calib_info_t *cal_if)
{
	int32_t ret;
	int32_t sindex;
	char *sname;

	if ((cal_if == NULL) || (cal_if->calib_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(cal_if->port);
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	/* idev open */
	ret = camera_sensor_idev_open(cal_if);
	if (ret < 0) {
		cam_err("sensor%d %s idev open error %d\n", sindex, sname, ret);
		return ret;
	}
	/* total size get and check */
	ret = camera_sensor_idev_totalsize(cal_if);
	if (ret < 0) {
		cam_err("sensor%d %s idev totalsize error %d\n", sindex, sname, ret);
		goto init_error_devclose;
	}
	/* get calib data */
	ret = camera_calib_data_get(cal_if);
	if (ret < 0) {
		cam_err("sensor%d %s calib data get error %d\n", sindex, sname, ret);
		goto init_error_devclose;
	}
	/* calib data init to idev */
	ret = camera_calib_data_init(cal_if);
	if (ret < 0) {
		cam_err("sensor%d %s calib data init error %d\n", sindex, sname, ret);
		goto init_error_dataput;
	}

	cam_info("sensor%d %s calib init done\n", sindex, sname);
	return ret;

init_error_dataput:
	camera_calib_data_put(cal_if);
init_error_devclose:
	camera_sensor_idev_close(cal_if);

	camera_debug_call_camo(cal_if->port);
	return ret;
}

/**
 * @NO{S10E02C04}
 * @ASIL{B}
 * @brief camrea calib so deinit and exit
 *
 * @param[in] cal_if: the calib inifo struct
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
int32_t camera_calib_deinit(calib_info_t *cal_if)
{
	int32_t sindex;
	char *sname;

	if ((cal_if == NULL) || (cal_if->calib_ops == NULL))
		return -RET_ERROR;
	camera_debug_call_cami(cal_if->port);
	sindex = cal_if->port;
	sname = cal_if->sensor_name;

	/* calib data deinit to idev */
	camera_calib_data_deinit(cal_if);
	/* calib data put and free */
	camera_calib_data_put(cal_if);
	/* idev close */
	camera_sensor_idev_close(cal_if);

	cam_info("sensor%d %s calib deinit done\n", sindex, sname);

	camera_debug_call_camo(cal_if->port);
	return RET_OK;
}

/**
 * @NO{S10E02C04I}
 * @ASIL{B}
 * @brief calib module get name and version string
 *
 * @param[in] cal_if: calib_info struct
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
int32_t camera_calib_get_version(calib_info_t *cal_if, char *name, char *version)
{
	int32_t ret = RET_OK;
	camera_handle_st *hcam = NULL;
	const char *ver;

	if (cal_if == NULL)
		return -RET_ERROR;

	if (name != NULL) {
		ret = camera_run_cam_get(cal_if->port, NULL, &hcam, NULL, NULL);
		if ((ret == 0) && (hcam != NULL)) {
			strncpy(name, hcam->sensor_lib.so_name, CAMERA_MODULE_NAME_LEN);
		} else {
			ret = -RET_ERROR;
		}
	}
	if (version != NULL) {
		ver = CAMERA_MODULE_GET_VERSION(CALIB_MODULE_M(cal_if));
		if (ver != NULL)
			strncpy(version, ver, CAMERA_VERSON_LEN_MAX);
		else
			ret = -RET_ERROR;
	}

	return 0;
}

int32_t camera_calib_set_cali_name_init(camera_module_lib_t *cal_lib)
{
       int32_t ret = RET_OK;

       if (cal_lib == NULL)
               return -RET_ERROR;

       ret = camera_sensor_isi_dev_open(cal_lib);
       if (ret < 0) {
               cam_err("sensor open isi dev fail, ret = %d\n", ret);
               return ret;
       }

       return ret;
}

int32_t camera_calib_set_cali_name_deinit(camera_module_lib_t *cal_lib)
{
	int32_t ret = RET_OK;

	if (cal_lib == NULL)
		return -RET_ERROR;

	ret = camera_sensor_isi_dev_close(cal_lib);
	if (ret < 0) {
		cam_err("sensor close isi dev fail, ret = %d\n", ret);
		return ret;
	}

	return ret;
}

int32_t camera_calib_set_cali_name_put(camera_module_lib_t *cal_lib, camera_calib_t *pcalib)
{
       int32_t ret = RET_OK;

       if (cal_lib == NULL || pcalib == NULL)
               return -RET_ERROR;

       ret = camera_sensor_isi_dev_data_put(cal_lib, pcalib);
       if (ret < 0) {
               cam_err("sensor put isi dev data fail, ret = %d\n", ret);
               return ret;
       }

       return ret;
}
