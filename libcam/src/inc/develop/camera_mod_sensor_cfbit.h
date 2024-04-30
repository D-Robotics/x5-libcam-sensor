/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_mod_sensor_cfbit.h
 *
 * @NO{S10E02C04}
 * @ASIL{B}
 */

#ifndef __CAMERA_MOD_SENSOR_CFBIT_H__
#define __CAMERA_MOD_SENSOR_CFBIT_H__

#include "camera_mod_sensor_data.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AE_DISABLE        BIT(B_AE_DISABLE)
#define AWB_DISABLE       BIT(B_AWB_DISABLE)
#define TEST_PATTERN      BIT(B_TEST_PATTERN)
#define DPHY_PORTB        BIT(B_DPHY_PORTB)
#define DPHY_COPY         BIT(B_DPHY_COPY)
#define EMBEDDED_MODE     BIT(B_EMBEDDED_MODE)
#define EMBEDDED_DATA     BIT(B_EMBEDDED_DATA)
#define TRIG_SOURCE       BIT(B_TRIG_SOURCE)
#define TRIG_STANDARD     BIT(B_TRIG_STANDARD)
#define TRIG_SHUTTER_SYNC BIT(B_TRIG_SHUTTER_SYNC)
#define TRIG_EXTERNAL     BIT(B_TRIG_EXTERNAL)
#define DUAL_ROI          BIT(B_DUAL_ROI)
#define MIRROR            BIT(B_MIRROR)
#define FLIP              BIT(B_FLIP)
#define PWL_24BIT         BIT(B_PWL_24BIT)
#define PDAF              BIT(B_PDAF)

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_MOD_SENSOR_CFBIT_H__ */

