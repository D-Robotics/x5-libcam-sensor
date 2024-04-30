/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file camera_legacy.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAMERA_LEGACY_H__
#define __CAMERA_LEGACY_H__

#ifdef __cplusplus
extern "C" {
#endif

/* utility sensor apis */
#include "camera_sensor_common.h"

typedef sensor_config_func SENSOR_CONFIG_FUNC;

#define sensor_param_parse(s, f, t, d)		camera_sensor_param_parse(s, f, t, d)
#define vin_string_parse(str, flag)		camera_sensor_emode_string_parse(str, flag)
#define vin_sensor_emode_parse(s, flag)		camera_sensor_emode_parse(s, flag)
#define vin_port_mask_of_snr(s)			camera_sensor_port_mask(s)
#define vin_sensor_param_get(s, scp)		camera_sensor_param_get(s, csp)
#define sensor_config_do(s, mask, pf)		camera_sensor_config_do(s, mask, pf)
#define VIN_DOFFSET(x, n)			camera_sensor_lut_byte_swap(x, n)

/* utility deserial apis */
#include "camera_mod_deserial_data.h"
extern uint32_t camera_deserial_port_mask(deserial_info_t *des_if, int32_t link_mask);

#define vin_port_mask_of_des(d, m)			camera_deserial_port_mask(d, m)

/* utility reg i2c apis */
#include "camera_reg.h"

#define vin_i2c_read8(b, rw, sa, ra)		camera_reg_i2c_read8(b, rw, sa, ra)
#define vin_i2c_read16(b, rw, sa, ra)		camera_reg_i2c_read16(b, rw, sa, ra)
#define vin_i2c_read_block(b, rw, sa, ra, f, s) camera_reg_i2c_read_block(b, rw, sa, ra, f, s)
#define vin_i2c_write8(b, rw, sa, ra, v)	camera_reg_i2c_write8(b, rw, sa, ra, v)
#define vin_i2c_write16(b, rw, sa, ra, v)	camera_reg_i2c_write16(b, rw, sa, ra, v)
#define vin_i2c_write_block(b, rw, sa, ra, f, s) camera_reg_i2c_write_block(b, rw, sa, ra, f, s)

#define vin_i2c_write8_s(b, sa, ra, rw, bm, v)	camera_reg_i2c_write8_s(b, sa, ra, rw, bm, v)
#define vin_i2c_bit_write8(b, sa, ra, rw, bm, v) camera_reg_i2c_bit_write8(b, sa, ra, rw, bm, v)
#define vin_i2c_bit_array_write8(b, sa, rw, s, p) camera_reg_i2c_bit_array_write8(b, sa, rw, s, p)
#define vin_write_array(b, sa, rw, s, p)	camera_reg_i2c_write_array(b, sa, rw, s, p)

#define vin_i2c_read_retry(b, sa, rw, ra)	camera_reg_i2c_read_retry(b, sa, rw, ra)
#define vin_i2c_write_retry(b, sa, rw, ra, v)	camera_reg_i2c_write_retry(b, sa, rw, ra, v)

/* utility sys apis */
#include "camera_sys.h"

#define vin_get_board_id()			camera_sys_get_board_id()

/* gpio apis */
#include "camera_gpio.h"

#define vin_gpio_export(gpio)			camera_gpio_export(gpio)
#define vin_gpio_unexport(gpio)			camera_gpio_unexport(gpio)
#define vin_gpio_set_dir(gpio, out_flag)	camera_gpio_set_dir(gpio, out_flag)
#define vin_gpio_set_value(gpio, value)		camera_gpio_set_value(gpio, value)
#define vin_gpio_get_value(gpio, pvalue)	camera_gpio_get_value(gpio, pvalue)
#define vin_gpio_set_edge(gpio, pedge)		camera_gpio_set_edge(gpio, pedge)

#define GPIO_LOW				CAMERA_GPIO_LOW
#define GPIO_HIGH				CAMERA_GPIO_HIGH
#define vin_power_ctrl(gpio, on_off)		camera_gpio_power_ctrl(gpio, on_off)

/* i2c apis */
#include "camera_i2c.h"

#define hb_vin_i2c_init(bus)			camera_i2c_init(bus)
#define hb_vin_i2c_deinit(bus)			camera_i2c_deinit(bus)

#define hb_vin_i2c_lock(bus)			camera_i2c_lock(bus)
#define hb_vin_i2c_unlock(bus)			camera_i2c_unlock(bus)

#define hb_vin_i2c_timeout_set(bus, timeout_ms) camera_i2c_timeout_set(bus, timeout_ms)

#define hb_vin_i2c_read_reg16_data16(bus, i2c_addr, reg_addr) \
						camera_i2c_read_reg16_data16(bus, i2c_addr, reg_addr)
#define hb_vin_i2c_read_reg16_data8(bus, i2c_addr, reg_addr) \
						camera_i2c_read_reg16_data8(bus, i2c_addr, reg_addr)
#define hb_vin_i2c_read_reg8_data8(bus, i2c_addr, reg_addr) \
						camera_i2c_read_reg8_data8(bus, i2c_addr, reg_addr)
#define hb_vin_i2c_read_reg8_data16(bus, i2c_addr, reg_addr) \
						camera_i2c_read_reg8_data16(bus, i2c_addr, reg_addr)
#define hb_vin_i2c_write_reg16_data16(bus, i2c_addr, reg_addr, value) \
						camera_i2c_write_reg16_data16(bus, i2c_addr, reg_addr, value)
#define hb_vin_i2c_write_reg16_data8(bus, i2c_addr, reg_addr, value) \
						camera_i2c_write_reg16_data8(bus, i2c_addr, reg_addr, value)
#define hb_vin_i2c_write_reg8_data16(bus, i2c_addr, reg_addr, value) \
						camera_i2c_write_reg8_data16(bus, i2c_addr, reg_addr, value)
#define hb_vin_i2c_write_reg8_data8(bus, i2c_addr, reg_addr, value) \
						camera_i2c_write_reg8_data8(bus, i2c_addr, reg_addr, value)
#define hb_vin_i2c_write_block(bus, i2c_addr, reg_addr, value, cnt) \
						camera_i2c_write_block(bus, i2c_addr, reg_addr, value, cnt)

#define hb_vin_i2c_read_block_reg16(bus, i2c_addr, reg_addr, buf, count) \
						camera_i2c_read_block_reg16(bus, i2c_addr, reg_addr, buf, count)
#define hb_vin_i2c_read_block_reg8(bus, i2c_addr, reg_addr, buf, count) \
						camera_i2c_read_block_reg8(bus, i2c_addr, reg_addr, buf, count)
#define hb_vin_i2c_write_block_reg16(bus, i2c_addr, reg_addr, buf, count) \
						camera_i2c_write_block_reg16(bus, i2c_addr, reg_addr, buf, count)
#define hb_vin_i2c_write_block_reg8(bus, i2c_addr, reg_addr, buf, count) \
						camera_i2c_write_block_reg8(bus, i2c_addr, reg_addr, buf, count)

#define hb_i2c_write(bus, i2c_addr, reg_addr, reg_addr_len, buf, buf_len) \
						camera_i2c_write(bus, i2c_addr, reg_addr, reg_addr_len, buf, buf_len)
#define hb_i2c_read(bus, i2c_addr, reg_addr, reg_addr_len, buf, buf_len) \
						camera_i2c_read(bus, i2c_addr, reg_addr, reg_addr_len, buf, buf_len)
/* log apis */
#include "camera_log.h"

#define vin_err(fmt, ...)			cam_err(fmt, ##__VA_ARGS__)
#define vin_warn(fmt, ...)			cam_warn(fmt, ##__VA_ARGS__)
#define vin_dbg(fmt, ...)			cam_dbg(fmt, ##__VA_ARGS__)
#define vin_info(fmt, ...)			cam_info(fmt, ##__VA_ARGS__)

/* diag event apis */
extern int32_t camera_run_send_event(int32_t camera_index, uint32_t event_type, uint32_t module_id,
			uint32_t event_id, uint32_t status);
#define hb_cam_send_diag_event(port_index, mod_id, event_id, status) \
		camera_run_send_event(port_index, HB_CAM_EVENT_DIAG, mod_id, event_id, status)
extern int32_t camera_run_reset_by_index(int32_t camera_index);
#define hb_vin_reset(port) \
		camera_run_reset_by_index(port)

#ifdef HOBOT_MCU_CAMSYS
/* mcu adapt */
#include "osal_time.h"
#define usleep(s)    osal_usleep(s)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CAMERA_LEGACY_H__ */
