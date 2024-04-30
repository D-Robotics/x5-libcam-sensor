/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[dummy]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include "inc/hb_vin.h"
#include "hb_cam_utility.h"
#include "hb_camera_data_config.h"

static int32_t sensor_func(sensor_info_t *sensor_info, const char *func)
{
	vin_dbg("port%d: %s -- %s --\n",
		sensor_info->port, sensor_info->sensor_name, func);
	return RET_OK;
}

int dummy_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

    turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;
    turning_data.sensor_data.lines_per_second = 33750;//vts * fps, should be fixed = 1125 * 30
        //lines_per_second/fps = vts
        // from customer, max 10ms
        // 1000ms -lines_per_second - 33750
        // 10ms - 337, 33ms - 1125
	turning_data.sensor_data.exposure_time_max = 377; //from customer, max 10ms
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 2 * 1125 - 8;  //2*frame_length - 8  //linear not use
    turning_data.sensor_data.analog_gain_max = 251; //we use again + dig fine gain
	turning_data.sensor_data.digital_gain_max = 0;

	//sensor bit && bayer
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (ret < 0) {
		vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
		return -RET_ERROR;
	}

	return ret;
}

static int32_t sensor_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;

        switch(sensor_info->sensor_mode) {
			case NORMAL_M:	  // 1: normal
				vin_info("dummy sensor in normal/linear mode\n");
				vin_info("bus_num = %d, sensor_addr = 0x%0x\n", sensor_info->bus_num, sensor_info->sensor_addr);

				ret = dummy_linear_data_init(sensor_info);
				if (ret < 0) {
						vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
						return -HB_CAM_INIT_FAIL;
				}
				break;
			case DOL2_M:
			default:
				vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
				ret = -HB_CAM_INIT_FAIL;
				break;
        }
        vin_info("dummy config success under %d mode\n", sensor_info->sensor_mode);

	return ret;
}

static int32_t sensor_deinit(sensor_info_t *sensor_info)
{
	sensor_func(sensor_info, __func__);

	if (sensor_info->dev_port >= 0 && sensor_info->sen_devfd > 0) {
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}

	return RET_OK;
}

static int32_t sensor_stop(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);

	sensor_func(sensor_info, __func__);

	return ret;
}

static int32_t sensor_start(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK, req;
	deserial_info_t *des = (deserial_info_t *)(sensor_info->deserial_info);

	sensor_func(sensor_info, __func__);

	return RET_OK;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_(dummy);
sensor_module_t dummy = {
	.module = SENSOR_MNAME(dummy),
#else
sensor_module_t dummy = {
	.module = "dummy",
	.emode = NULL,
#endif
	.init = sensor_init,
	.deinit = sensor_deinit,
	.start = sensor_start,
	.stop = sensor_stop,
};

