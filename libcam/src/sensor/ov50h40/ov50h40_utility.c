/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2024 D-Robotics.
 * All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include "../hb_cam_utility.h"
#include "../hb_i2c.h"
#include "inc/ov50h40_setting.h"
#include "inc/sensor_effect_common.h"
#include "inc/sensorstd_common.h"
#include "hb_camera_data_config.h"

#define OV50H40_AGAIN_HIGH_BYTE  0x3508
#define OV50H40_AGAIN_LOW_BYTE  0x3509
#define OV50H40_EXP_HIGH_BYTE  0x3501 //[15:8]
#define OV50H40_EXP_LOW_BYTE  0x3502  //[7:0]
#define OV50H40_HTS_HI  0x380c
#define OV50H40_HTS_LO  0x380d
#define OV50H40_VTS_HI  0x380e
#define OV50H40_VTS_LO  0x380f

#define DW9800_VCM_ADDR 0x0c //ov50h VCM address

int sensor_af_init(sensor_info_t *info)
{
	camera_i2c_write_reg8_data8(info->bus_num, DW9800_VCM_ADDR, 0x02, 0x02);
	camera_i2c_write_reg8_data8(info->bus_num, DW9800_VCM_ADDR, 0x06, 0x40);
	camera_i2c_write_reg8_data8(info->bus_num, DW9800_VCM_ADDR, 0x07, 0x79);
	return 0;
}

static int ov50h40_linear_data_init(sensor_info_t *sensor_info);
static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps);
static int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info);
int sensor_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	vin_dbg("%s gpio_num = %d \n", sensor_info->sensor_name, sensor_info->gpio_num);
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			vin_dbg("%s gpio_pin[%d] = %d \n", sensor_info->sensor_name, gpio, sensor_info->gpio_pin[gpio]);
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio]);
				usleep(100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
						1 - sensor_info->gpio_level[gpio]);
				if(ret < 0) {
						vin_err("vin_power_ctrl fail\n");
						return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100 * 1000);  //100ms
			}
		}
	}

	return ret;
}

int sensor_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	int group_id = 0;
	uint8_t val = 0;
	int i;

	pr_debug("ov50h40 sensor_init \n");
	ret = sensor_poweron(sensor_info);
	if (ret < 0) {
			pr_err("%d : sensor reset %s fail\n",
						__LINE__, sensor_info->sensor_name);
			return ret;
	}

	// set resolution and format
	if (sensor_info->resolution == 3072) {
		pr_debug("ov50h40 resolution is 3072 \n");
		setting_size =
				sizeof(ov50h40_4096x3072_30fps_24MHz_linear_10bit_2000Mbps_4lane) / sizeof(uint32_t) / 2;
		ret = vin_write_array(sensor_info->bus_num,
								sensor_info->sensor_addr, 2,
								setting_size, ov50h40_4096x3072_30fps_24MHz_linear_10bit_2000Mbps_4lane);
		if (ret < 0) {
			pr_err("%d : init %s fail\n",
					__LINE__, sensor_info->sensor_name);
			return ret;
		}
	} else {
		pr_err("config mode is err\n");
		return -RET_ERROR;
	}

	ret = ov50h40_linear_data_init(sensor_info);
	if (ret < 0) {
		pr_err("%d : turning data init %s fail\n",
							__LINE__, sensor_info->sensor_name);
		return ret;
	}

	// Default 30fps
	// Switch frame rate based on application configuration
	//switch fps should be setted by user program, API: <hbn_camera_change_fps> !
	usleep(100 * 1000);  //100ms
	ret = sensor_dynamic_switch_fps(sensor_info, sensor_info->fps);
	if (ret < 0) {
		vin_err("ov50h40 dynamic switch fps fail, ret = %d \n", ret);
		ret = HB_CAM_DYNAMIC_SWITCH_FPS_FAIL;
	}

	sensor_af_init(sensor_info);

	return ret;
}

// start stream
int sensor_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	int group_id = 0;

	pr_debug("ov50h40 sensor start\n");
	setting_size = sizeof(ov50h40_stream_on_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, ov50h40_stream_on_setting);
	if (ret < 0) {
		pr_err("start %s fail\n", sensor_info->sensor_name);
		return ret;
	}

	return ret;
}

int sensor_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	printf("ov50h40 sensor stop \n");
	setting_size = sizeof(ov50h40_stream_off_setting) / sizeof(uint32_t) / 2;
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, ov50h40_stream_off_setting);
	if (ret < 0) {
		pr_err("stop %s fail\n", sensor_info->sensor_name);
		return ret;
	}
	return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
														sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -1;
				}
			}
		}
	}

	return ret;
}

int sensor_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sensor_poweroff(sensor_info);
	if (ret < 0)
	{
			pr_err("%d : deinit %s fail\n",
						__LINE__, sensor_info->sensor_name);
			return ret;
	}
	return ret;
}


void ov50h40_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
					sizeof(turning_data->sensor_name));
	return;
}

void ov50h40_normal_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
	turning_data->sensor_data.active_width = sensor_info->width;
	turning_data->sensor_data.active_height = sensor_info->height;
	// turning sensor_data
	int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV50H40_VTS_HI);
	int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV50H40_VTS_LO);
	uint32_t vts = vts_hi;
	vts = vts << 8 | vts_lo;
	printf("vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi,vts_lo, vts);//
	turning_data->sensor_data.conversion = 1;
	turning_data->sensor_data.turning_type = 6;
	turning_data->sensor_data.lines_per_second = vts * sensor_info->fps;
	turning_data->sensor_data.exposure_time_max = vts - 36;
	printf("exposure_time_max: %d\n",turning_data->sensor_data.exposure_time_max);
	turning_data->sensor_data.exposure_time_long_max = vts;
	turning_data->sensor_data.analog_gain_max = 191; //191
	turning_data->sensor_data.digital_gain_max = 159;//31
	turning_data->sensor_data.exposure_time_min = 2;
}

// turning data init
static int ov50h40_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	ov50h40_common_data_init(sensor_info, &turning_data);
	ov50h40_normal_data_init(sensor_info, &turning_data);

	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	// setting stream ctrl
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ov50h40_stream_on_setting)) {
			memcpy(stream_on, ov50h40_stream_on_setting, sizeof(ov50h40_stream_on_setting));
	} else {
			pr_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
	}

	if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ov50h40_stream_off_setting)) {
			memcpy(stream_off, ov50h40_stream_off_setting, sizeof(ov50h40_stream_off_setting));
	}
	else {
			pr_err("Number of registers on stream over 10\n");
			return -RET_ERROR;
	}
	// look-up table
	turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
			memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
			memcpy(turning_data.normal.again_lut, ov50h40_again_lut,
						sizeof(ov50h40_again_lut));
	}

	turning_data.normal.dgain_lut = malloc(256 * sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL)
	{
		memset(turning_data.normal.dgain_lut, 0xff, 256 * sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, ov50h40_dgain_lut,
						sizeof(ov50h40_dgain_lut));
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
			free(turning_data.normal.again_lut);
			turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut)
	{
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0) {
			pr_err("sensor_%s ioctl fail %d\n", sensor_info->sensor_name, ret);
			return -RET_ERROR;
	}

	return ret;
}


static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
#ifdef AE_DBG
	printf("test %s, mode = %d pos = %d\n", __FUNCTION__, mode, pos);
#endif
	//VCM CONTROL REG
	const uint16_t VCM_MSB = 0x03; //[9:8]
	const uint16_t VCM_LSB = 0x04; //[7:0]

	char temp0 = 0, temp1 = 0;
	uint32_t spos =  pos;
	if ( spos > 1023) {
		spos = 1023;
	}

	if ( spos < 0) {
		spos = 0;
	}

	temp0 = (spos >> 8) & 0x3;
	vin_i2c_write8(info->bus_num, 8, DW9800_VCM_ADDR, VCM_MSB, temp0);
	temp1 = (spos) & 0xff;
	vin_i2c_write8(info->bus_num, 8, DW9800_VCM_ADDR, VCM_LSB, temp1);
#ifdef AE_DBG
		printf("write spos = %d, 0x03 = 0x%x, 0x04 = 0x%x\n",spos, temp0, temp1);
#endif

	return 0;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
	printf("test %s, mode = %d gain_num = %d again[0] = %d, dgain[0] = %d\n", __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif

	//again
	const uint16_t COARSE_AGAIN_H = 0x3508; //[0:7] 64x
	const uint16_t FINE_AGAIN_L = 0x3509;   //[0:7]
	//dgain
	const uint16_t COARSE_DGAIN_L = 0x350A; //[0:4] 31x
	const uint16_t FINE_DGAIN_H = 0x350B;   //[0:7]
	const uint16_t FINE_DGAIN_L = 0x350C;   //[6:7]

	char again_reg_value_h = 0,again_reg_value_l = 0;
	char coarse_dgain_l = 0,fine_dgain_h = 0,fine_dgain_l = 0;
	int again_index = 0;
	int dgain_index = 0;

	if (mode == NORMAL_M) {
		if (again[0] >= sizeof(ov50h40_again_lut)/sizeof(uint32_t))
			again_index = sizeof(ov50h40_again_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(ov50h40_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(ov50h40_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];


		//config again
		again_reg_value_h = (ov50h40_again_lut[again_index] >> 8) & 0xFF;
		again_reg_value_l = (ov50h40_again_lut[again_index]) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, COARSE_AGAIN_H, again_reg_value_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, FINE_AGAIN_L, again_reg_value_l);
		//config dgain
		coarse_dgain_l = (ov50h40_dgain_lut[dgain_index] >> 16) & 0x1F;
		fine_dgain_h = (ov50h40_dgain_lut[dgain_index] >> 8) & 0xFF;
		fine_dgain_l = (ov50h40_dgain_lut[dgain_index]) & 0xC0;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, COARSE_DGAIN_L, coarse_dgain_l);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, FINE_DGAIN_H, fine_dgain_h);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, FINE_DGAIN_L, fine_dgain_l);
#ifdef AE_DBG
		printf("%s, gain_index: %d, COARSE_AGAIN_H:0x3508 = 0x%x, FINE_AGAIN_L:0x3509 = 0x%x, COARSE_DGAIN_L:0x350A = 0x%x, FINE_DGAIN_H:0x350B = 0x%x, FINE_DGAIN_L:0x350C = 0x%x\n",
		__FUNCTION__, again_index, again_reg_value_h, again_reg_value_l,coarse_dgain_l,fine_dgain_h,fine_dgain_l);
#endif
	} else	{
		vin_err(" unsupport mode %d\n", mode);
	}

	return 0;
}

/* input value:
 * line: exposure time value
 * line_num: linear mode: 1; dol2 mode: 2
 * */

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	printf("line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
	//Long exposure
	const uint16_t EXP_L_LINE0 = 0x3500; //[0:7]
	const uint16_t EXP_L_LINE1 = 0x3501; //[0:7]
	const uint16_t EXP_L_LINE2 = 0x3502; //[0:7]

	char temp0 = 0, temp1 = 0, temp2 = 0;
	char dynamic_vts_h = 0, dynamic_vts_l = 0;
	if (mode == NORMAL_M) {
		uint32_t sline =  line[0];

		// Support dynamic FPS based on ISP exposure time line
		uint32_t dynamic_vts = sline + 36; // from sensor FAE :dynamic_vts = sline + 36
		if(dynamic_vts <= 1084){
			dynamic_vts = 1084;
		}else{
			//NOTICE: fps = lines_per_second / vts
			uint32_t fps = 32540 / dynamic_vts;
#ifdef AE_DBG
			printf("%s set fps = %d, vts = 0x%x \n", __FUNCTION__, fps, dynamic_vts);
#endif
		}

		dynamic_vts_h = (dynamic_vts >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OV50H40_VTS_HI, dynamic_vts_h);
		dynamic_vts_l = dynamic_vts & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, OV50H40_VTS_LO, dynamic_vts_l);

		temp0 = (sline >> 16) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_L_LINE0, temp0);
		temp1 = (sline >> 8) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_L_LINE1, temp1);
		temp2 = (sline) & 0xFF;
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_L_LINE2, temp2);

#ifdef AE_DBG
		printf("write sline = %d, 0x3500 = 0x%x, 0x3501 = 0x%x,0x3502 = 0x%x\n",
						sline, temp0, temp1,temp2);
#endif
		} else {
			vin_err(" unsupport mode %d\n", mode);
		}

	return 0;
}


static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL| HAL_LINE_CONTROL | HAL_AF_CONTROL;
	return 0;
}

static int32_t sensor_update_fps_notify_driver(sensor_info_t *sensor_info)
{
	int32_t ret = RET_OK;

	switch(sensor_info->sensor_mode) {
		case (uint32_t)NORMAL_M:
				ret = ov50h40_linear_data_init(sensor_info);
				if (ret < 0) {
						vin_err("update fps ov50h40_linear_data_init fail\n");
						return ret;
				}
				break;
		default:
				vin_err("update fps not support %d mode \n", sensor_info->sensor_mode);
				break;
	}

	return ret;
}

/* input value:
 * fps: set fps
 *
 * we can use this function to dynamic switch fps in our program.
 * int32_t hbn_camera_change_fps(camera_handle_t cam_fd, int32_t fps)
 */
static int32_t sensor_dynamic_switch_fps(sensor_info_t *sensor_info, uint32_t fps)
{
	int32_t ret = RET_OK;
	int32_t vts;
	int32_t vts_h,vts_l;

	vin_info("%s %s %dfps \n", __FUNCTION__, sensor_info->sensor_name, fps);

	if (fps < 1 || sensor_info->fps > 30) {
			vin_err("%s %s %dfps not support\n", __FUNCTION__, sensor_info->sensor_name, fps);
			return -RET_ERROR;
	}

	switch (sensor_info->sensor_mode) {
			case NORMAL_M:
					//NOTICE:
					//vts = frame_length = lines_per_second / fps
					vts = 32540 / fps;
					break;
			default:
					vin_err("%s not support mode %d \n", __FUNCTION__, sensor_info->sensor_mode);
					return -RET_ERROR;
	}

#ifdef AE_DBG
	printf("%s set fps = %d, vts = 0x%x \n", __FUNCTION__, fps, vts);
#endif
	ret = hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
					OV50H40_VTS_HI, ((vts >> 8) & 0xff)); //0x380e[15:8]
	ret |= hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
					OV50H40_VTS_LO, (vts & 0xff)); 	  //0x380f[7:0]

	sensor_info->fps = fps;
	sensor_update_fps_notify_driver(sensor_info);
	vin_err("%s dynamic switch to %dfps success \n", sensor_info->sensor_name, fps);
	return RET_OK;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ov50h40, CAM_MODULE_FLAG_A16D8);
sensor_module_t ov50h40 = {
	.module = SENSOR_MNAME(ov50h40),
#else
sensor_module_t ov50h40 = {
	.module = "ov50h40",
#endif
	.init = sensor_init,
	.start = sensor_start,
	.stop = sensor_stop,
	.deinit = sensor_deinit,
	.power_on = sensor_poweron,
	.power_off = sensor_poweroff,
	.aexp_line_control = sensor_aexp_line_control,
	.dynamic_switch_fps = sensor_dynamic_switch_fps,
	.af_control = sensor_af_control,
	.aexp_gain_control = sensor_aexp_gain_control,
	.userspace_control = sensor_userspace_control,
};
