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
#include "inc/ov5640_setting.h"
#include "inc/sensor_effect_common.h"
#define OV5640_EXP_REG_ADDR (0x3500)
#define OV5640_GAIN_ADDR_HI (0x350A)
#define OV5640_GAIN_ADDR_LO (0x350B)
#define OV5640_VTS_HI (0x380E)
#define OV5640_VTS_LO (0x380F)

#define GPIO_BASE     0x34120000

#define MCLK (24000000)
static int power_ref;
static int ov5640_linear_data_init(sensor_info_t *sensor_info);

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
                                        return -HB_CAM_SENSOR_POWEROFF_FAIL;
                                }
	                }
	        }
        }

        return ret;
}

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

        printf("ov5640 sensor_init \n");
        ret = sensor_poweron(sensor_info);
        if (ret < 0) {
                pr_err("%d : sensor reset %s fail\n",
                           __LINE__, sensor_info->sensor_name);
                return ret;
        }

        // init_settings
        setting_size =
                sizeof(ov5640_init_settings) / sizeof(uint32_t) / 2;
        pr_debug("%s write init_settings \n", sensor_info->sensor_name);
        ret = vin_write_array(sensor_info->bus_num,
                sensor_info->sensor_addr, 2,
                setting_size, ov5640_init_settings);
        if (ret < 0) {
                pr_err("%d : init %s fail\n",
                        __LINE__, sensor_info->sensor_name);
                return ret;
        }

        // set clock

        printf("ov5640 set clock \n");
        setting_size =
                sizeof(ov5640_clk_settings) / sizeof(uint32_t) / 2;
        pr_debug("%s write clk_settings \n", sensor_info->sensor_name);
        ret = vin_write_array(sensor_info->bus_num,
                sensor_info->sensor_addr, 2,
                setting_size, ov5640_clk_settings);
        if (ret < 0) {
                pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
                return ret;
        }

        // set tpg disable
        hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                        OV5640_TPG_CTRL, 0x0);
        hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                        OV5640_TPG_SET, 0);

        // set resolution and format
        if (sensor_info->resolution == 1080) {
                printf("ov5640 resolution is 1080 \n");
                setting_size =
                        sizeof(ov5640_1080p_settings) / sizeof(uint32_t) / 2;
                pr_debug("%s write 1080p setting, size = %d\n",
                                 sensor_info->sensor_name, setting_size);
                ret = vin_write_array(sensor_info->bus_num,
                                        sensor_info->sensor_addr, 2,
                                        setting_size, ov5640_1080p_settings);
                if (ret < 0) {
                        pr_err("%d : init %s fail\n",
                                         __LINE__, sensor_info->sensor_name);
                        return ret;
                }
        } else if (sensor_info->resolution == 720) {
                printf("ov5640 resolution is 720 \n");
                setting_size =
                        sizeof(ov5640_720p_settings) / sizeof(uint32_t) / 2;
                pr_debug("%s write 720p setting, size = %d\n",
                                 sensor_info->sensor_name, setting_size);
                ret = vin_write_array(sensor_info->bus_num,
                                        sensor_info->sensor_addr, 2,
                                        setting_size, ov5640_720p_settings);
                if (ret < 0) {
                        pr_err("%d : init %s fail\n",
                                         __LINE__, sensor_info->sensor_name);
                        return ret;
                }
        } else {
                pr_err("config mode is err\n");
                return -RET_ERROR;
        }

        if (sensor_info->format == 0x2B) {
                // 10-bit
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                OV5640_SC_PLL_CTRL0, 0x1a);
        } else if (sensor_info->format == 0x2A || sensor_info->format == 0x1c || sensor_info->format == 0x1e) {
                // 8-bit
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                OV5640_SC_PLL_CTRL0, 0x18);
        } else {
                pr_err("%s: Unsupported output format %d!\n", __func__, sensor_info->format);
                return -1;
        }


        // write format  RAW8 or RAW10
        if (sensor_info->format == 0x2A || sensor_info->format == 0x2B) {
                printf("ov5640 sensor format = 0x%02x \n", sensor_info->format);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FORMAT_CTRL00, 0x02);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FMT_MUX_CTRL, 0x03);
        } else if (sensor_info->format == 0x1e) {
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FORMAT_CTRL00, 0x3f);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FMT_MUX_CTRL, 0x00);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_SC_PLL_CTRL2, 0x2c);
        } else if (sensor_info->format == 0x1c) {
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FORMAT_CTRL00, 0x42);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_FMT_MUX_CTRL, 0x00);
                hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
                                    OV5640_SC_PLL_CTRL2, 0x2c);
        } else {
                pr_err("%s format 0x%02x is not support now!\n", sensor_info->sensor_name, sensor_info->format);
        }

        // hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
        //                 OV5640_TPG_CTRL, 0x80);
        // if (sensor_info->format == 0x2b) {
        //         hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
        //                         OV5640_TPG_SET, 0x04);
        // } else if (sensor_info->format == 0x2a || sensor_info->format == 0x1c || sensor_info->format == 0x1e) {
        //         hb_vin_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr,
        //                         OV5640_TPG_SET, 0x05);
        // } else {
        //         pr_err("%s: Unsupported output format %d!\n", __func__, sensor_info->format);
        //         return -1;
        // }

        ret = ov5640_linear_data_init(sensor_info);
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

        printf("ov5640 sensor start\n");
        setting_size = sizeof(ov5640_2lane_stream_on_setting) / sizeof(uint32_t) / 2;
        pr_debug("sensor_name %s, setting_size = %d\n",
                sensor_info->sensor_name, setting_size);
        ret = vin_write_array(sensor_info->bus_num,
                                sensor_info->sensor_addr, 2,
                                setting_size, ov5640_2lane_stream_on_setting);
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
        printf("ov5640 sensor stop \n");
        setting_size = sizeof(ov5640_2lane_stream_off_setting) / sizeof(uint32_t) / 2;
        pr_debug("sensor_name %s, setting_size = %d\n",
                sensor_info->sensor_name, setting_size);
        ret = vin_write_array(sensor_info->bus_num,
                                sensor_info->sensor_addr, 2,
                                setting_size, ov5640_2lane_stream_off_setting);
        if (ret < 0) {
                pr_err("stop %s fail\n", sensor_info->sensor_name);
                return ret;
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
#define MAX_EXPO 1100

void ov5640_common_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
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

void ov5640_normal_data_init(sensor_info_t *sensor_info, sensor_turning_data_t *turning_data)
{
        turning_data->sensor_data.active_width = sensor_info->width;
        turning_data->sensor_data.active_height = sensor_info->height;
        // turning sensor_data
        turning_data->sensor_data.turning_type = 6;
        int vts_hi = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV5640_VTS_HI);
        int vts_lo = hb_vin_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, OV5640_VTS_LO);
        uint32_t vts = vts_hi;
        vts = vts << 8 | vts_lo;
        //pr_err("vts_hi:0x%x,vts_lo:0x%x,vts:0x%x\n", vts_hi,vts_lo, vts);
        turning_data->sensor_data.lines_per_second = vts * sensor_info->fps; // TBC
        // turning_data.sensor_data.lines_per_second = 33120;      // TBC
        turning_data->sensor_data.exposure_time_max = vts;              // TBC
        turning_data->sensor_data.exposure_time_long_max = vts; // TBC
}

// turning data init
static int ov5640_linear_data_init(sensor_info_t *sensor_info)
{
        int ret = RET_OK;
        uint32_t open_cnt = 0;
        sensor_turning_data_t turning_data;
        uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
        uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

        memset(&turning_data, 0, sizeof(sensor_turning_data_t));

        // common data
        ov5640_common_data_init(sensor_info, &turning_data);
        ov5640_normal_data_init(sensor_info, &turning_data);

        sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R, (uint32_t)BAYER_PATTERN_RGGB);
        sensor_data_bits_fill(&turning_data.sensor_data, 12);

        turning_data.sensor_data.gain_max = 128 * 8192;            // TBC
        turning_data.sensor_data.analog_gain_max = 128 * 8192; // TBC
        turning_data.sensor_data.digital_gain_max = 0;
        turning_data.sensor_data.exposure_time_min = 1;

        turning_data.normal.s_line_length = 0;
        // aGain
        turning_data.normal.again_control_num = 0;

        // dGain ,ov5640 don't have dgain register
        turning_data.normal.dgain_control_num = 0;

        // setting stream ctrl
        turning_data.stream_ctrl.data_length = 1;
        if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(ov5640_2lane_stream_on_setting)) {
                memcpy(stream_on, ov5640_2lane_stream_on_setting, sizeof(ov5640_2lane_stream_on_setting));
        } else {
                pr_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }

        if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(ov5640_2lane_stream_off_setting)) {
                memcpy(stream_off, ov5640_2lane_stream_off_setting, sizeof(ov5640_2lane_stream_off_setting));
        }
        else {
                pr_err("Number of registers on stream over 10\n");
                return -RET_ERROR;
        }
        // look-up table
        turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
        if (turning_data.normal.again_lut != NULL) {
                memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
                memcpy(turning_data.normal.again_lut, ov5640_gain_lut,
                           sizeof(ov5640_gain_lut));
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
        temp1 = (sline << 4) & 0xff;
        vin_i2c_write8(bus, 16, sensor_addr, 0x3502, temp1);
        temp2 = ((sline >> 4) & 0xff);
        vin_i2c_write8(bus, 16, sensor_addr, 0x3501, temp2);
        temp2 = (sline >> 12) & 0xff;
        vin_i2c_write8(bus, 16, sensor_addr, 0x3500, temp2);

    return 0;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
        int bus = info->bus_num;
        int sensor_addr = info->sensor_addr;
        char hi = 0, lo = 0;
        uint32_t Again = again[0];
        if (Again > 255)
                return -1;
        int ret = 0;
        hi = (ov5640_gain_lut[Again] >> 8) & 0x03;
        ret = vin_i2c_write8(bus, 16, sensor_addr, OV5640_GAIN_ADDR_HI, hi);
        if (ret != 0) {
                printf("error while writing OV5640_GAIN_ADDR_HI!\n");
        }
        lo = (ov5640_gain_lut[Again] & 0xff);
        ret = vin_i2c_write8(bus, 16, sensor_addr, OV5640_GAIN_ADDR_LO, lo);
        if (ret != 0) {
                printf("error while writing ov5640_GAIN_ADDR_LO!\n");
        }
        return 0;
}
#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(ov5640, CAM_MODULE_FLAG_A16D8);
sensor_module_t ov5640 = {
        .module = SENSOR_MNAME(ov5640),
#else
sensor_module_t ov5640 = {
        .module = "ov5640",
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
