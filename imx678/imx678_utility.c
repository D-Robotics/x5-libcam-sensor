/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt) "[imx678]:" fmt

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
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/imx678_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"

#include "inc/tmi8150b_control.h"
#include <pthread.h>

#define REG_WIDTH 2   // reg16 data8

static int imx678_linear_data_init(sensor_info_t *sensor_info);
static int imx678_dol2_data_init(sensor_info_t *sensor_info);
static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos);
int sensor_af_init(sensor_info_t *info);

// Global variables to save initialization status and current target position
static unsigned char af_enable = 0; //to-do :We need to find a solution that allows for global control, requiring only the maintenance of a single switch.
static int af_initialized = 0;
unsigned char total_phase = 64;
unsigned char microstep = 6;  // 32 subdivisions
unsigned char speed = 30;
unsigned char direction = 1;  // 1 - clockwise ， 2 - counterclockwise
static pthread_t af_thread_id = 0;
unsigned int af_control_time = 0;
/* Actual measurement: motor using 128 subdivisions, 10 medium speed, 2 counterclockwise, 90-degree rotation requires 0x330 angle count. With speed setting 30, it takes about 38 seconds to reach the initial position */
/* Confirmed with motor control chip manufacturer, angle control indeed only has 0x330 angle */
/* For quick homing, can use microstep = 6. During actual debugging, to slow down, adjust the speed to */

static void* af_init_thread(void* arg) {
    sensor_info_t* sensor_info = (sensor_info_t*)arg;

    vin_info("AF initialization thread started");

    // 调用 AF 初始化
    int ret = sensor_af_init(sensor_info);
    if (ret < 0) {
        vin_err("AF initialization thread failed: %d\n", ret);
    } else {
        vin_info("AF initialization thread completed\n");
    }

    return NULL;
}


int sensor_af_init(sensor_info_t *info)
{
    int ret = 0;
    unsigned short target_angle = 0;
    short int current_angle_y = 0;
    short int current_angle_x = 0;
    int cnt = 0;

    if (af_initialized) {
        printf("TMI8150B AF already initialized\n");
        return 0;
    }

    // TMI8150B SPI initialization
    ret = Tmi8150_Api_init();
    if (ret < 0) {
        printf("TMI8150B AF initialization failed\n");
        return -1;
    }

    // TMI8150B global register initialization
    ret = Global_control();
    if (ret < 0) {
        printf("TMI8150B global control failed\n");
        return -1;
    }

    ret |= Yaxis_angel_phase_clear();
    usleep(500);
    if (ret < 0) {
        printf("Y-axis motor parameter configuration failed\n");
        return -1;
    }

    // Start synchronous initial position
    // Use manual mode for precise position control
    ret = Yaxis_angel_read(&current_angle_y);
    if (ret < 0) {
        printf("Failed to read Y-axis angle\n");
        return -1;
    }

    vin_dbg("Start initializing X-axis");
    ret = Xaxis_angel_read(&current_angle_x);
    vin_dbg("current_angle_x = %d\n" , current_angle_x);
    ret |= Xaxis_angel_phase_clear();
    usleep(500);

    unsigned char used_microstep = microstep;
    microstep = 6;
    direction = 1;  // Clockwise
    target_angle = 0x330;
    vin_dbg("microstep = %d , speed = %d\n", microstep , speed);
    ret = Yaxis_motor_solution(direction, microstep, speed);
    ret = Yaxis_motor_manual_start(target_angle, 0);  // 相位设为 0
    ret = Xaxis_motor_solution(direction, microstep, speed);
    ret = Xaxis_motor_manual_start(target_angle, 0);  // 相位设为 0

    sleep(6);

    ret = Yaxis_angel_read(&current_angle_y);
    if (ret < 0) {
        printf("Failed to read Y-axis angle\n");
        return -1;
    }


    direction = 2; // Counterclockwise
    target_angle = 0x0000;
    ret = Yaxis_motor_solution(direction, microstep, speed); 
    ret = Yaxis_motor_manual_start(target_angle, 0);  // Phase set to 0
    ret = Xaxis_motor_solution(direction, microstep, speed);
    ret = Xaxis_motor_manual_start(target_angle, 0);  // Phase set to 0

    sleep(6);


    ret = Yaxis_angel_read(&current_angle_y);
    if (ret < 0) {
        printf("Failed to read Y-axis angle\n");
        return -1;
    }
    ret = Yaxis_motor_lock();

    ret |= Yaxis_angel_phase_clear(); // Clear angle
    ret |= Xaxis_angel_phase_clear();

    // Return to set subdivision.
    microstep = used_microstep;
    // After counterclockwise rotation completes, return to initial position, can only first rotate clockwise.
    direction = 1;
    af_initialized = 1;




    //6.0 fixed focus
    target_angle = 0x00c5;
    ret = Xaxis_motor_solution(direction, microstep, speed);
    ret = Xaxis_motor_manual_start(target_angle, 0);  // Phase set to 0
    sleep(2);

    return 0;
}


int sensor_af_deinit(sensor_info_t *info)
{   int ret = 0;
    ret = Yaxis_motor_lock();
    return ret;
}

static uint32_t imx678_max(int32_t a, int32_t b)
{
    if (a > b)
        return a;
    else
        return b;
}

static int sensor_poweroff(sensor_info_t *sensor_info)
{
    int gpio, ret = RET_OK;
    vin_dbg("%s() : %s gpio_num = %d\n" , __FUNCTION__ , sensor_info->sensor_name, sensor_info->gpio_num);
    if (sensor_info->gpio_num > 0)
    {
        for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
        {
            vin_dbg("%s() , %s gpio_pin[%d] = %d\n", __FUNCTION__ , sensor_info->sensor_name, gpio,
                sensor_info->gpio_pin[gpio]);
            if (sensor_info->gpio_pin[gpio] != -1)
            {
                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
                if (ret < 0)
                {
                    vin_err("vin_power_ctrl fail\n");
                    return -HB_CAM_SENSOR_POWEROFF_FAIL;
                }
            }
        }
    }

    if(af_enable)
        sensor_af_deinit(sensor_info);

    return ret;
}

static int sensor_poweron(sensor_info_t *sensor_info)
{
    int gpio, ret = RET_OK;
    vin_dbg("%s gpio_num = %d\n", sensor_info->sensor_name, sensor_info->gpio_num);
    if (sensor_info->gpio_num > 0)
    {
        for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
        {

            vin_dbg("%s gpio_pin[%d] = %d\n", sensor_info->sensor_name, gpio,
                           sensor_info->gpio_pin[gpio]);
            if (sensor_info->gpio_pin[gpio] != -1)
            {
                ret = vin_power_ctrl(sensor_info->gpio_pin[gpio], sensor_info->gpio_level[gpio]);
                usleep(100 * 1000);   // 100ms
                ret |=
                    vin_power_ctrl(sensor_info->gpio_pin[gpio], 1 - sensor_info->gpio_level[gpio]);

                if (ret < 0)
                {
                    vin_err("vin_power_ctrl fail\n");
                    return -HB_CAM_SENSOR_POWERON_FAIL;
                }
                usleep(100 * 1000);   // 100ms
            }
        }
    }

    return ret;
}

static int sensor_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int setting_size = 0;

    int val = 0;

    ret = sensor_poweron(sensor_info);
    if (ret < 0)
    {
        vin_err("%d : sensor reset %s fail\n", __LINE__, sensor_info->sensor_name);
        return ret;   //-HB_CAM_SENSOR_POWERON_FAIL
    }

    vin_info("sensor_info->sensor_mode = %d", sensor_info->sensor_mode);
    switch (sensor_info->sensor_mode)
    {
    case NORMAL_M:   // 1: normal
        vin_info("imx678 in normal/linear mode\n");
        vin_info("bus_num = %d, sensor_addr = 0x%0x, fps = %d, config_index = %d\n",
                       sensor_info->bus_num, sensor_info->sensor_addr, sensor_info->fps,
                       sensor_info->config_index);
        if (sensor_info->fps == 30)
        {
            // setting_size = sizeof(imx678_init_3840x2160_linear_setting) / sizeof(uint32_t) / 2;
            setting_size = sizeof(imx678_init_3840x2160_linear_setting) /
                           sizeof(imx678_init_3840x2160_linear_setting[0]) / 2;
            ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
                                  setting_size, imx678_init_3840x2160_linear_setting);
            if (ret < 0)
            {
                vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
                return -HB_CAM_I2C_WRITE_FAIL;
            }

            // usleep(1000 * 1000);  //100ms

            ret = imx678_linear_data_init(sensor_info);
            if (ret < 0)
            {
                vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
                return -HB_CAM_INIT_FAIL;
            }
        }

        break;
    case DOL2_M:
        vin_info("imx678 in dol2/hdr mode\n");
        vin_info("bus_num = %d, sensor_addr = 0x%0x fps = %d\n", sensor_info->bus_num,
                       sensor_info->sensor_addr, sensor_info->fps);

        setting_size = sizeof(imx678_init_3840x2160_dol2_setting) /
                       sizeof(imx678_init_3840x2160_dol2_setting[0]) / 2;
        ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH,
                              setting_size, imx678_init_3840x2160_dol2_setting);
        if (ret < 0)
        {
            vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
            return -HB_CAM_I2C_WRITE_FAIL;
        }
        ret = imx678_dol2_data_init(sensor_info);
        if (ret < 0)
        {
            vin_err("%d : hdr data init %s fail\n", __LINE__, sensor_info->sensor_name);
            return -HB_CAM_INIT_FAIL;
        }

        break;
    default:
        vin_err("%d not support mode %d\n", __LINE__, sensor_info->sensor_mode);
        ret = -HB_CAM_INIT_FAIL;
        break;
    }
    vin_info("imx678 config success under %d mode\n", sensor_info->sensor_mode);


    if(af_enable)
    {
        // // not thread , blocking main process
        // sensor_af_init(sensor_info);

        // Use thread for AF initialization, not blocking main process
        pthread_t af_thread;
        int thread_ret = pthread_create(&af_thread, NULL, af_init_thread, sensor_info);
        if (thread_ret != 0) {
            vin_err("Failed to create AF initialization thread: %d\n", thread_ret);
            // Here can choose synchronous initialization or ignore error
            sensor_af_init(sensor_info); // Fallback to synchronous initialization
        } else {
            pthread_detach(af_thread); // Detach thread, let it end by itself
            vin_err("AF initialization thread started");
        }
    }
    return ret;
}

static int sensor_start(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int setting_size = 0;

    setting_size = sizeof(imx678_stream_on_setting) / sizeof(imx678_stream_on_setting[0]) / 2;
    vin_info("%s start normal/linear mode\n", sensor_info->sensor_name);
    ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH, setting_size,
                          imx678_stream_on_setting);
    if (ret < 0)
    {
        vin_err("start %s fail\n", sensor_info->sensor_name);
        return -HB_CAM_I2C_WRITE_FAIL;
    }

    return ret;
}

static int sensor_stop(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    int setting_size = 0;

    // linear and hdr mode use one stream_off setting
    setting_size =
        sizeof(imx678_stream_off_setting) / sizeof(imx678_stream_off_setting[0]) / 2;
    vin_info("%s sensor stop\n", sensor_info->sensor_name);
    ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, REG_WIDTH, setting_size,
                          imx678_stream_off_setting);
    if (ret < 0)
    {
        vin_err("start %s fail\n", sensor_info->sensor_name);
        return -HB_CAM_I2C_WRITE_FAIL;
    }

    return ret;
}

static int sensor_deinit(sensor_info_t *sensor_info)
{
    int ret = RET_OK;

    ret = sensor_poweroff(sensor_info);
    if (ret < 0)
    {
        vin_err("%d : deinit %s fail\n", __LINE__, sensor_info->sensor_name);
        return ret;   //-HB_CAM_SENSOR_POWEROFF_FAIL
    }
    return ret;
}

static int sensor_af_control(hal_control_info_t *info, uint32_t mode, uint32_t pos)
{
    /* After af_init, the initialized motor position is the state with lens retracted, at this point can only */
    int ret = 0;

    if (!af_initialized) {
        vin_dbg("%s():AF not initialized, please call sensor_af_init first\n",__FUNCTION__);
        return 0;
    }

    af_control_time ++;
    vin_dbg("sensor_af_control() start , sensor_af_control: af_control_time = %d , mode = %d, pos = %d\n", af_control_time , mode , pos);

    // // Position range limited to 0-816
    uint32_t spos = pos;
    if (spos > 816) {
        spos = 816;
        printf("Warning: Target position exceeds maximum limit 816, limited to 816\n");
    }
    unsigned short target_angle = (unsigned short)(spos);


    // Read current actual angle
    short int current_angle = 0;
    ret = Yaxis_angel_read(&current_angle);

    if (ret < 0) {
        printf("Failed to read Y-axis angle\n");
        return -1;
    }
    /* todo:
     * If the value turns to less than 0, but the motor rotation count becomes very high, 
     * then it needs to be recalibrated; otherwise, the rotation count will be inaccurate, 
     * or the correct number of rotations will be lost.
     */
    if(current_angle > 7000)// Means over-rotated, at this point motor must be at 0 position
    {
        ret |= Yaxis_angel_phase_clear(); // Clear angle
        vin_info(" current_angle =%d , over-rotated, angle cleared\n" , current_angle);
        current_angle = 0;
    }

#ifdef AE_DBG
    vin_dbg("current_angle: %d, target_angle: %d, spos: %d\n", 
           current_angle, target_angle, spos);
#endif

    // If already near target position, no need to move
    int angle_diff = abs(current_angle - target_angle);
    if (angle_diff <= 0) {  // 0 angle tolerance
#ifdef AE_DBG
    vin_dbg("Already near target position (tolerance %d), no need to move\n", angle_diff);
#endif
        current_angle = target_angle;
        ret = Yaxis_motor_lock();
        return 0;
    }

    // If target angle same as current target, also no need to move
    if (target_angle == current_angle) {
#ifdef AE_DBG
        vin_dbg("Target position unchanged, no need for other movement commands, just complete unfinished command\n");
        // ret = Yaxis_motor_lock();
#endif
        return 0;
    }

    // Determine direction
    if (target_angle < current_angle)
        direction = 2;
    else
        direction = 1;


    usleep(50);

    // Use manual mode for precise position control
    ret = Yaxis_motor_solution(direction, microstep, speed);  // 1= clockwise
    ret = Yaxis_motor_manual_start(target_angle, 0);  // Phase set to 0
    if (ret < 0) {
        printf("TMI8150B AF control failed\n");
        return -1;
    }

    vin_dbg("sensor_af_control() end , current_angle: %d, target_angle: %d, spos: %d\n", current_angle, target_angle, spos);

    return 0;
}

static int sensor_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again,
                                    uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
    printf("\n%s():mode = %d , gain_num = %d , again[0] = %d , dgain[0] = %d , again[1] = %d , dgain[1] = %d\n", __FUNCTION__ , mode , gain_num , again[0] , dgain[0] , again[1] , dgain[1]);
#endif
    // const uint16_t AGAIN = 0x3090;
    char again_reg_value = 0;
    int gain_index = 0;

    if (mode == NORMAL_M)
    {
        if (again[0] >= sizeof(imx678_gain_lut) / sizeof(uint32_t))
            gain_index = sizeof(imx678_gain_lut) / sizeof(uint32_t) - 1;
        else
            gain_index = again[0];

        again_reg_value = (imx678_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
        vin_dbg("%s, gain_index: %d, again:0x%04x = 0x%x\n", __FUNCTION__, gain_index, GAIN_PCG0,
               again_reg_value);
#endif
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, GAIN_PCG0, again_reg_value);
    }
    else if( mode == DOL2_M)
    {
        if (again[0] >= sizeof(imx678_gain_lut) / sizeof(uint32_t))
            gain_index = sizeof(imx678_gain_lut) / sizeof(uint32_t) - 1;
        else
            gain_index = again[0];

        again_reg_value = (imx678_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
        vin_dbg("%s, gain_index: %d, again:0x%04x = 0x%x\n", __FUNCTION__,
                gain_index, GAIN_PCG0, again_reg_value);
#endif
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, GAIN_PCG0,
                        again_reg_value);

        if (again[1] >= sizeof(imx678_gain_lut) / sizeof(uint32_t))
            gain_index = sizeof(imx678_gain_lut) / sizeof(uint32_t) - 1;
        else
            gain_index = again[1];

        again_reg_value = (imx678_gain_lut[gain_index] >> 0) & 0xFF;
#ifdef AE_DBG
        vin_dbg("%s, gain_index: %d, again:0x%04x = 0x%x\n", __FUNCTION__,
                gain_index, GAIN_PCG1, again_reg_value);
#endif
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, GAIN_PCG1,
                        again_reg_value);
    }
    else
    {
        vin_err(" unsupport mode %d\n", mode);
    }

    return 0;
}

/* input value:
 * line: exposure time value
 * line_num: linear mode: 1; dol2 mode: 2
 * */

static int sensor_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line,
                                    uint32_t line_num)
{
#ifdef AE_DBG
    printf("line mode %d, --line[0] : %d ,line[1] : %d ,  line_num:%d \n", mode, line[0], line[1], line_num);
#endif
    uint32_t tmp = 0;
    char temp0 = 0;
    char temp1 = 0;
    char temp2 = 0;

    int shr0 = 0;
    int shr1 = 0;
    uint32_t rhs1 = 0;
    uint32_t Vmax = 0x08CA;   // 2250
    uint32_t fsc = 0;

    int val_l = 0;
    int val_m = 0;
    int val_h = 0;

    val_l = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_VAMX);
    val_m = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_VAMX + 1);
    val_h = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_VAMX + 2);
    Vmax = ((val_h & 0xff) << 16) | (val_m << 8) | val_l;

    if ((mode == NORMAL_M) || (line_num == 1))
    {
        shr0 = Vmax - line[0];

        if (shr0 < 8)
            shr0 = 8;
        if (shr0 > (Vmax - 4))
            shr0 = Vmax - 4;

        tmp = shr0;
        temp2 = (tmp >> 16) & 0x0F;
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0 + 2, temp2);
        temp1 = (tmp >> 8) & 0xFF;
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0 + 1, temp1);
        temp0 = (tmp & 0xFF);
        vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0, temp0);
#ifdef AE_DBG
            printf("0x%x = 0x%x, 0x%x = 0x%x,0x%x = 0x%x\n", IMX678_SHR0 + 2, temp2, IMX678_SHR0 + 1, temp1,
                IMX678_SHR0, temp0);
#endif
    }
    else if (mode == DOL2_M)
    {
#ifdef AE_DBG
        printf("line[0] = %d , line[1] = %d \n", line[0], line[1]);
#endif

        val_l = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_RHS1);
        val_m = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_RHS1 + 1);
        val_h = camera_reg_i2c_read8(info->bus_num, REG_WIDTH_16bit, info->sensor_addr, IMX678_RHS1 + 2);
        rhs1 = ((val_h & 0xff) << 16) | (val_m << 8) | val_l;

        if (line_num == 2)
        {
            fsc = Vmax * 2;
            shr0 = fsc - line[0];
            shr1 = rhs1 - line[1];

            if(shr1 < 5)
                shr1 = 5;
            if(shr1 > (rhs1 - 2))
                shr1 = rhs1 - 2;
            if(shr0 < (rhs1 + 5))
                shr0 = rhs1 + 5;
            if(shr0 > (fsc - 2))
                shr0 = fsc - 2;
            shr1 |= 0x01; // 2x+1
            shr0 = ((shr0 >> 1) << 1); // 2x

#ifdef AE_DBG
            printf("shr0 = 0x%x, shr1 = 0x%x, rhs1 = 0x%x, Vmax = 0x%x, fsc = 0x%x\n",
                shr0, shr1, rhs1, Vmax, fsc);
#endif

            tmp = shr0;
            temp2 = (tmp >> 16) & 0x0F;
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0 + 2, temp2);
            temp1 = (tmp >> 8) & 0xFF;
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0 + 1, temp1);
            temp0 = (tmp & 0xFF);
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR0, temp0);

#ifdef AE_DBG
            printf("0x%x = 0x%x, 0x%x = 0x%x,0x%x = 0x%x\n", IMX678_SHR0 + 2, temp2, IMX678_SHR0 + 1, temp1,
                IMX678_SHR0, temp0);
#endif

            tmp = shr1;
            temp2 = (tmp >> 16) & 0x0F;
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR1 + 2, temp2);
            temp1 = (tmp >> 8) & 0xFF;
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR1 + 1, temp1);
            temp0 = (tmp & 0xFF);
            vin_i2c_write8(info->bus_num, 16, info->sensor_addr, IMX678_SHR1, temp0);

#ifdef AE_DBG
            printf("0x%x = 0x%x, 0x%x = 0x%x,0x%x = 0x%x\n", IMX678_SHR1 + 2, temp2, IMX678_SHR1 + 1, temp1,
                IMX678_SHR1, temp0);
#endif

        }
        else
        {
            vin_err(" unsupport line_num %d\n", line_num);
            return -1;
        }
    }
    else
    {
        vin_err(" unsupport mode %d\n", mode);
        return -1;
    }

    return 0;
}

static int imx678_linear_data_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    uint32_t open_cnt = 0;
    sensor_turning_data_t turning_data;
    uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
    uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

    int Hmax = 0x044C;   // 1100
    int Vmax = 0x08CA;   // 2250

    int val_l = 0;
    int val_m = 0;
    int val_h = 0;

    val_l = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX);
    val_m = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX + 1);
    val_h = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX + 2);
    Vmax = (val_h << 16) | (val_m << 8) | val_l;

    val_l = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_HAMX);
    val_m = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_HAMX + 1);
    Hmax = (val_m << 8) | val_l;
    vin_info("Vmax = 0x%x, Hmax = 0x%x", Vmax, Hmax);

    memset(&turning_data, 0, sizeof(sensor_turning_data_t));

    // common data
    turning_data.bus_num = sensor_info->bus_num;
    turning_data.bus_type = sensor_info->bus_type;
    turning_data.af_mode = 1;
    turning_data.port = sensor_info->port;
    turning_data.reg_width = sensor_info->reg_width;
    turning_data.mode = sensor_info->sensor_mode;
    turning_data.sensor_addr = sensor_info->sensor_addr;
    strncpy(turning_data.sensor_name, sensor_info->sensor_name, sizeof(turning_data.sensor_name));

    turning_data.sensor_data.active_width = 3840;
    turning_data.sensor_data.active_height = 2160;

    turning_data.sensor_data.lines_per_second = Vmax * sensor_info->fps; //67500;
    turning_data.sensor_data.exposure_time_max = Vmax - 2;
    turning_data.sensor_data.exposure_time_min = 1;
    turning_data.sensor_data.analog_gain_max = 255;
    turning_data.sensor_data.digital_gain_max = 0;
    turning_data.sensor_data.analog_gain_init = 32;
    turning_data.sensor_data.digital_gain_init = 0;
    turning_data.sensor_data.exposure_time_init = Vmax - 8;

    turning_data.sensor_data.turning_type = 6;         // gain calc
    turning_data.sensor_data.fps = sensor_info->fps;   // fps

    //   sensor turning data init

    turning_data.normal.param_hold = IMX678_PARAM_HOLD;
    turning_data.normal.param_hold_length = 1;
    turning_data.normal.s_line = IMX678_LINE;
    turning_data.normal.s_line_length = 3;

    // sensor bit && bayer
    sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R,
                           (uint32_t)BAYER_PATTERN_RGGB);
    // sensor exposure_max_bit, maybe not used ?  //FIXME
    sensor_data_bits_fill(&turning_data.sensor_data, 12);

    // some stress test case, we need kernel stream_ctrl.
    turning_data.stream_ctrl.data_length = 2;

    if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx678_stream_on_setting))
    {
        memcpy(stream_on, imx678_stream_on_setting, sizeof(imx678_stream_on_setting));
    }
    else
    {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx678_stream_off_setting))
    {
        memcpy(stream_off, imx678_stream_off_setting, sizeof(imx678_stream_off_setting));
    }
    else
    {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }

    // sync gain lut to kernel driver.
    turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
    if (turning_data.normal.again_lut != NULL)
    {
        memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
        memcpy(turning_data.normal.again_lut, imx678_gain_lut, sizeof(imx678_gain_lut));
    }

    ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

    if (turning_data.normal.again_lut)
    {
        free(turning_data.normal.again_lut);
        turning_data.normal.again_lut = NULL;
    }

    if (ret < 0)
    {
        vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
        return -RET_ERROR;
    }

    return ret;
}

static int imx678_dol2_data_init(sensor_info_t *sensor_info)
{
    int ret = RET_OK;
    uint32_t open_cnt = 0;
    sensor_turning_data_t turning_data;
    uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
    uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

    int Hmax = 0x044C;   // 1100
    int Vmax = 0x08CA;   // 2250

    int val_l = 0;
    int val_m = 0;
    int val_h = 0;

    val_l = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX);
    val_m = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX + 1);
    val_h = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_VAMX + 2);
    Vmax = (val_h << 16) | (val_m << 8) | val_l;

    val_l = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_HAMX);
    val_m = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_HAMX + 1);
    Hmax = (val_m << 8) | val_l;
    vin_info("Vmax = 0x%x, Hmax = 0x%x", Vmax, Hmax);

    val_l = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_RHS1);
    val_m = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_RHS1 + 1);
    val_h = camera_reg_i2c_read8(sensor_info->bus_num, REG_WIDTH_16bit, sensor_info->sensor_addr, IMX678_RHS1 + 2);
    uint32_t rhs1 = ((val_h & 0xff) << 16) | (val_m << 8) | val_l;

    memset(&turning_data, 0, sizeof(sensor_turning_data_t));

    // common data
    turning_data.bus_num = sensor_info->bus_num;
    turning_data.bus_type = sensor_info->bus_type;
    turning_data.port = sensor_info->port;
    turning_data.af_mode = 1;
    turning_data.reg_width = sensor_info->reg_width;
    turning_data.mode = sensor_info->sensor_mode;
    turning_data.sensor_addr = sensor_info->sensor_addr;
    strncpy(turning_data.sensor_name, sensor_info->sensor_name, sizeof(turning_data.sensor_name));

    turning_data.sensor_data.active_width = 3840;
    turning_data.sensor_data.active_height = 2160;

    turning_data.sensor_data.lines_per_second = Vmax * sensor_info->fps * 2; //81000;
    turning_data.sensor_data.exposure_time_max =  rhs1 -2;
    turning_data.sensor_data.exposure_time_min = 8;
    turning_data.sensor_data.exposure_time_long_max = (Vmax * 2) - 2;  //2*frame_length - 8  //linear not use
    turning_data.sensor_data.analog_gain_max = 255;
    turning_data.sensor_data.digital_gain_max = 0;

    turning_data.sensor_data.analog_gain_init = 32;
    turning_data.sensor_data.digital_gain_init = 0;
    turning_data.sensor_data.exposure_time_init = Vmax - 8;

    turning_data.sensor_data.turning_type = 6;         // gain calc
    turning_data.sensor_data.fps = sensor_info->fps;   // fps

    // sensor bit && bayer
    sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_R,
                           (uint32_t)BAYER_PATTERN_RGGB);
    // sensor exposure_max_bit, maybe not used ?  //FIXME
    sensor_data_bits_fill(&turning_data.sensor_data, 12);

    // some stress test case, we need kernel stream_ctrl.
    turning_data.stream_ctrl.data_length = 2;

    turning_data.dol2.line_p[0].ratio = 1 << 8;
    turning_data.dol2.line_p[0].offset = 0;
    turning_data.dol2.line_p[0].max = 66;
    turning_data.dol2.line_p[1].ratio = 1 << 8;
    turning_data.dol2.line_p[1].offset = 0;
    turning_data.dol2.line_p[1].max = 2176;


    if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(imx678_stream_on_setting))
    {
        memcpy(stream_on, imx678_stream_on_setting, sizeof(imx678_stream_on_setting));
    }
    else
    {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }
    if (sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(imx678_stream_off_setting))
    {
        memcpy(stream_off, imx678_stream_off_setting, sizeof(imx678_stream_off_setting));
    }
    else
    {
        vin_err("Number of registers on stream over 10\n");
        return -RET_ERROR;
    }

    // sync gain lut to kernel driver.
    turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
    if (turning_data.normal.again_lut != NULL)
    {
        memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
        memcpy(turning_data.normal.again_lut, imx678_gain_lut, sizeof(imx678_gain_lut));
    }

    ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);

    if (turning_data.normal.again_lut)
    {
        free(turning_data.normal.again_lut);
        turning_data.normal.again_lut = NULL;
    }

    if (ret < 0)
    {
        vin_err("%s sync gain lut ioctl fail %d\n", sensor_info->sensor_name, ret);
        return -RET_ERROR;
    }

    return ret;
}

static int sensor_userspace_control(uint32_t port, uint32_t *enable)
{
    vin_info("enable userspace gain control and line control\n");
    *enable = 0;	//imx678 use kernel space gain contrl and line control
    *enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
    // *enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL | HAL_AF_CONTROL;

    return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(imx678, CAM_MODULE_FLAG_A16D8);
sensor_module_t imx678 = {
    .module = SENSOR_MNAME(imx678),
#else
sensor_module_t imx678 = {
    .module = "imx678",
#endif
    .init = sensor_init,
    .start = sensor_start,
    .stop = sensor_stop,
    .deinit = sensor_deinit,
    .power_on = sensor_poweron,
    .power_off = sensor_poweroff,
    .aexp_gain_control = sensor_aexp_gain_control,
    .aexp_line_control = sensor_aexp_line_control,
    .af_control = sensor_af_control,
    .userspace_control = sensor_userspace_control,
};
