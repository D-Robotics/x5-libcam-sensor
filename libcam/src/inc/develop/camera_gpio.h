/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_gpio.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_GPIO_H__
#define __CAMERA_GPIO_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_GPIO_IN		(0)
#define CAMERA_GPIO_OUT		(1)
#define CAMERA_GPIO_LOW		(0)
#define CAMERA_GPIO_HIGH	(1)

extern int32_t camera_gpio_export(uint32_t gpio);
extern int32_t camera_gpio_unexport(uint32_t gpio);
extern int32_t camera_gpio_set_dir(uint32_t gpio, uint32_t out_flag);
extern int32_t camera_gpio_set_value(uint32_t gpio, uint32_t value);
extern int32_t camera_gpio_get_value(uint32_t gpio, uint32_t *value);
extern int32_t camera_gpio_set_edge(uint32_t gpio, const char *edge);

extern int32_t camera_gpio_power_ctrl(uint32_t gpio, int32_t on_off);

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_GPIO_H__ */
