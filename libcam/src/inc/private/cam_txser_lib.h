/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_txser_lib.h
 *
 * @NO{S10E02C06}
 * @ASIL{B}
 */

#ifndef __CAM_TXSER_LIB_H__
#define __CAM_TXSER_LIB_H__

#include <stdint.h>

#include "hb_camera_data_config.h"

#include "camera_mod_txser.h"

#include "cam_module.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_TXSER_CHECK_ADDR_EXCEPT	(0xFF)
#define CAM_TXSER_CHECK_ADDR_MIN	(0x00)
#define CAM_TXSER_CHECK_ADDR_MAX	(0x7F)
#define CAM_TXSER_CHECK_RESET_DELAY_MIN	(0)
#define CAM_TXSER_CHECK_RESET_DELAY_MAX (10000)

#define CAM_TXSER_RESET_DELAY_DEFAULT	(20)
#define CAM_TXSER_LINK_MAP_DEFAULT	(0x3210)

/* internal apis */
extern int32_t camera_txser_config_check(camera_module_lib_t *lib, txser_config_t *txs_config);
extern int32_t camera_txser_ops_bind(txser_handle_st *htxs, txser_info_t *txs_if);
extern int32_t camera_txser_config_parse(txser_handle_st *htxs, txser_info_t *txs_if);
extern int32_t camera_txser_csi_attr_parse(txser_handle_st *htxs, txser_info_t *txs_if, mipi_config_t *mipi_from_rx,
				mipi_config_t *mipi_to, mipi_bypass_t *bypass_to);
extern int32_t camera_txser_init(txser_info_t *txs_if);
extern int32_t camera_txser_deinit(txser_info_t *txs_if);
extern int32_t camera_txser_get_version(txser_info_t *txs_if, char *name, char *version);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_TXSER_LIB_H__ */


