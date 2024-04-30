/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_DESERIAL_MAX_COMMON_H_
#define UTILITY_DESERIAL_MAX_COMMON_H_

#include "../hb_cam_utility.h"

#define GMSL_MODE3    3
#define GMSL_MODE6    6
#define SHIFT_8BIT    (0xff)
#define SHIFT_16BIT    (0xffff)

#define PIPE_VC_MASK		0xC0
#define PIPE_VC_SHFIT		6
#define PIPE_VC_SETTING_SHFIT	4

#define SENSOR_INFO(i) ((sensor_info_t *)(deserial_if->sensor_info[(i)]))

typedef struct device_info_s {
	int32_t dev_rev;
    uint16_t reg_addr;
}device_info_t;

/*
 * deserial_get_gmsl_speed() - Prase ser_type and link_speed form emode_name 'S' & 'L', than to determin the gmsl link speed
 * @gmsl_speed: assign the gmsl speed to the gmsl_speed
 */
int32_t deserial_get_gmsl_speed(deserial_info_t *deserial_if, int8_t *gmsl_speed, int32_t link_num);

/*
 * deserial_change_pipe_vc() - Change the vc num for deserial pipe config arry
 * @pdata: deserial pipe config arry
 * @pdata_vc_idx: deserial vc num index in pdata
 * @vc_num: change vc to vc_num
 */
void deserial_change_pipe_vc(uint32_t *pdata, int32_t pdata_vc_idx, int32_t vc_num);

int32_t deserial_get_dev_rev(deserial_info_t *deserial_if, device_info_t* dev_info);

#endif        // UTILITY_DESERIAL_MAX_COMMON_H_
