/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics.
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
#include "inc/ov2778_setting.h"
#include "inc/sensor_effect_common.h"

#define OV2778_AGAIN           0x30bb
#define OV2778_VTS_HI          0x30b2
#define OV2778_VTS_LO          0x30b3
#define OV2778_EXP_HIGH_BYTE   0x30b6
#define OV2778_EXP_LOW_BYTE    0x30b7

#define GPIO_BASE     0x34120000

#define MCLK (24000000)
static int power_ref;
static int ov2778_linear_data_init(sensor_info_t *sensor_info);
int sensor_poweron(sensor_info_t *sensor_info)
{
        uint32_t gpio_base, gpio_val, gpio_curr_val;
        int fd;
        unsigned char *gpio_addr;
        int32_t sensor_index = 0;

        sensor_index = sensor_info->entry_num;
        gpio_base = 1 << (11 + sensor_index);
        gpio_val  = 1 << (11 + sensor_index);

        fd = open("/dev/mem", O_RDWR);
        if (fd < 0) {
                printf("open /dev/mem failed\n");
                return -1;
        }

        gpio_addr = (unsigned char *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
        gpio_curr_val = *(unsigned int *)(gpio_addr + 0x04);
        *(unsigned int *)(gpio_addr + 0x04) = gpio_base | gpio_curr_val;

        usleep(200 * 1000);

        gpio_curr_val = *(unsigned int *)gpio_addr;
        *(unsigned int *)gpio_addr = gpio_val | gpio_curr_val;

        usleep(200 * 1000);

        if (fd)
                close(fd);

        munmap(gpio_addr, 0x1000);

        usleep(200 * 1000);

        return 0;
}

int sensor_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;
        int group_id = 0;
        uint8_t val = 0, chip_id;
        int i;

        pr_debug("ov2778 sensor_init \n");
        ret = sensor_poweron(sensor_info);
        if (ret < 0) {
                        pr_err("%d : sensor poweron %s fail\n",
                                                __LINE__, sensor_info->sensor_name);
                        return ret;
        }

        chip_id = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_CHIP_ID_MSB);
        pr_debug("CHIP ID MSB:%x\n", chip_id);
        if (chip_id != 0x27) {
                        pr_err("CHIP ID MSB CHECK FAILED\n");
                        return -1;
        }

        chip_id = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_CHIP_ID_LSB);
        pr_debug("CHIP ID LSB:%x\n", chip_id);
        if (chip_id != 0x70) {
                        pr_err("CHIP ID LSB CHECK FAILED\n");
                        return -1;
        }
        // set resolution and format
        if (sensor_info->resolution == 1080) {
                pr_debug("ov2778 resolution is 1080 \n");
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_SFW_CTRL1, 0x0);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x3013, 0x01);
                usleep(1000);
                setting_size =
                        sizeof(ov2778_init_settings) / sizeof(uint32_t) / 2;
                ret = vin_write_array(sensor_info->bus_num,
                                        sensor_info->sensor_addr, 2,
                                        setting_size, ov2778_init_settings);
                if (ret < 0) {
                                pr_err("%d : init %s fail\n",
                                         __LINE__, sensor_info->sensor_name);
                        return ret;
                }
        } else {
                pr_err("config mode is err\n");
                return -RET_ERROR;
        }

        if (sensor_info->format == 0x2c)
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_INTF_CTRL0, 0x08);
        else {
                pr_err("unsupported format\n");
                return -RET_ERROR;
        }
        // hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_ISP_PRE_CTL, 0x80);

        ret = ov2778_linear_data_init(sensor_info);
        if (ret < 0) {
                pr_err("%d : turning data init %s fail\n",
                         __LINE__, sensor_info->sensor_name);
                return ret;
        }

        return ret;
}

// start stream
int sensor_start(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        int setting_size = 0;
        int group_id = 0;

        pr_debug("ov2778 sensor start\n");
        setting_size = sizeof(ov2778_2lane_stream_on_setting) / sizeof(uint32_t) / 2;
        ret = vin_write_array(sensor_info->bus_num,
                                                        sensor_info->sensor_addr, 2,
                                                        setting_size, ov2778_2lane_stream_on_setting);
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
        printf("ov2778 sensor stop \n");
        setting_size = sizeof(ov2778_2lane_stream_off_setting) / sizeof(uint32_t) / 2;
        ret = vin_write_array(sensor_info->bus_num,
                                sensor_info->sensor_addr, 2,
                                setting_size, ov2778_2lane_stream_off_setting);
        if (ret < 0) {
                pr_err("stop %s fail\n", sensor_info->sensor_name);
                return ret;
        }
        return ret;
}

int sensor_poweroff(sensor_info_t *sensor_info)
{
        if (sensor_info->sen_devfd != 0) {
                close(sensor_info->sen_devfd);
                sensor_info->sen_devfd = -1;
        }
        return 0;
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
#define MAX_EXPO 1100

void ov2778_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
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

void ov2778_normal_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
        turning_data->sensor_data.active_width = sensor_info->width;
        turning_data->sensor_data.active_height = sensor_info->height;
        // turning sensor_data
        turning_data->sensor_data.turning_type = 6;
        int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_VTS_HI);
        int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV2778_VTS_LO);
        uint32_t vts = vts_hi;
        vts = vts << 8 | vts_lo;
        //pr_err("vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi,vts_lo, vts);
        turning_data->sensor_data.lines_per_second = vts * sensor_info->fps; // TBC
        // turning_data.sensor_data.lines_per_second = 33120;      // TBC
        turning_data->sensor_data.exposure_time_max = vts;              // TBC
        turning_data->sensor_data.exposure_time_long_max = vts; // TBC
}

// turning data init
static int ov2778_linear_data_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        uint32_t open_cnt = 0;
        sensor_turning_data_t turning_data;
        uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
        uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

        memset(&turning_data, 0, sizeof(sensor_turning_data_t));

        // common data
        ov2778_common_data_init(sensor_info, &turning_data);
        ov2778_normal_data_init(sensor_info, &turning_data);

        sensor_data_bayer_fill(&turning_data.sensor_data, 12, (uint32_t)BAYER_RGBIR_4x4_START_BGGIR, (uint32_t)BAYER_PATTERN_GRBIR_4X4);
        sensor_data_bits_fill(&turning_data.sensor_data, 12);

        turning_data.sensor_data.gain_max = 128 * 8192;            // TBC
        turning_data.sensor_data.analog_gain_max = 128 * 8192; // TBC
        turning_data.sensor_data.digital_gain_max = 0;
        turning_data.sensor_data.exposure_time_min = 1;

        turning_data.normal.s_line_length = 0;
        // aGain
        turning_data.normal.again_control_num = 0;

        // dGain ,ov2778 don't have dgain register
        turning_data.normal.dgain_control_num = 0;

        // setting stream ctrl
        turning_data.stream_ctrl.data_length = 1;
        if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ov2778_2lane_stream_on_setting)) {
                memcpy(stream_on, ov2778_2lane_stream_on_setting, sizeof(ov2778_2lane_stream_on_setting));
        } else {
                pr_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }

        if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ov2778_2lane_stream_off_setting)) {
                memcpy(stream_off, ov2778_2lane_stream_off_setting, sizeof(ov2778_2lane_stream_off_setting));
        }
        else {
                pr_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }
        // look-up table
        turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
        if (turning_data.normal.again_lut != NULL) {
                memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
                 memcpy(turning_data.normal.again_lut, ov2778_gain_lut,
                        sizeof(ov2778_gain_lut));
        }
        ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
        if (turning_data.normal.again_lut) {
                free(turning_data.normal.again_lut);
                turning_data.normal.again_lut = NULL;
        }
        if (ret < 0) {
                pr_err("sensor_%s ioctl fail %d\n", sensor_info->sensor_name, ret);
                return -RET_ERROR;
        }

        return ret;
}
static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
        //*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
        *enable = 0;
        return 0;
}

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
        int ret = 0;
        int bus = info->bus_num;
        int sensor_addr = info->sensor_addr;
        char temp = 0, temp1 = 0, temp2 = 0;
        uint32_t sline = line[0];
        const uint32_t max_lines = MAX_EXPO;
        if (sline > max_lines) {
                sline = max_lines;
        }
        if (sline < 1) {
                sline = 1;
        }

        ret = vin_i2c_write8(bus, 16, sensor_addr, OV2778_EXP_HIGH_BYTE, (sline >> 8) & 0xff);
        if (ret) {
                pr_err("set exp_high error \n");
                return ret;
        }
        ret = vin_i2c_write8(bus, 16, sensor_addr, OV2778_EXP_LOW_BYTE, sline & 0xff);
        if (ret) {
                pr_err("set exp_low error \n");
                return ret;
        }

        return ret;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
        int bus = info->bus_num;
        int sensor_addr = info->sensor_addr;
        char hi = 0, lo = 0;
        uint32_t Again = again[0];
        int ret = 0;

        if (Again > 255)
                return -1;


        ret = vin_i2c_write8(bus, 16, sensor_addr, OV2778_AGAIN, Again);
        if (ret) {
                pr_err("set again_high error \n");
                return ret;
        }

        return ret;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ov2778, CAM_MODULE_FLAG_A16D8);
sensor_module_t ov2778 = {
                .module = SENSOR_MNAME(ov2778),
#else
sensor_module_t ov2778 = {
                .module = "ov2778",
#endif
                .init = sensor_init,
                .start = sensor_start,
                .stop = sensor_stop,
                .deinit = sensor_deinit,
                .power_on = sensor_poweron,
                .power_off = sensor_poweroff,
                .aexp_line_control = sensor_aexp_line_control,
                .aexp_gain_control = sensor_aexp_gain_control,
                .userspace_control = sensor_userspace_control,
};
