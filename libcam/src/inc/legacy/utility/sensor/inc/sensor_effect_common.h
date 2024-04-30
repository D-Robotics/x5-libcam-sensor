/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file sensor_effect_common.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

/* dummy for legacy header */
#include "camera_mod_sensor_data.h"

#include "camera_sensor_dev.h"

/* des pre operation for sensor lib */
#include "camera_deserial_dev.h"
#undef hb_vin_mipi_pre_request
#define hb_vin_mipi_pre_request(entry_num, type, timeout)	camera_deserial_dev_pre_req(entry_num, type, timeout)
#undef hb_vin_mipi_pre_result
#define hb_vin_mipi_pre_result(entry_num, type, result)		camera_deserial_dev_pre_result(entry_num, type, result)
