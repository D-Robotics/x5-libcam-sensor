/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_vpf.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_VPF_H__
#define __CAM_VPF_H__

#include <stdint.h>

#include "hb_camera_data_config.h"

#include "camera_mod_common.h"

#include "vpf_data_info.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_VPF_MIPI_VRATIO_DEFAULT		(1.5)
#define CAMERA_VPF_MIPI_IPI_VC_DEFAULT		(0x0)
#define CAMERA_VPF_MIPI_FPS_DEFAULT		(30)
#define CAMERA_VPF_MIPI_CLK_MBPS		(1000000UL)

extern vpf_handle_t camera_vpf_get_vin_fd(int32_t vflow_id);
extern int32_t camera_vpf_mipi_config_parse(mipi_config_t *mipi_to, mipi_bypass_t *bypass_to,
				mipi_config_t *mipi_from, csi_attr_t *csi_from, uint32_t ipi_vc);
extern int32_t camera_vpf_mipi_config_parse_tx(mipi_config_t *mipi_to, mipi_bypass_t *bypass_to,
				mipi_config_t *mipi_from, mipi_config_t *mipi_from_rx);
extern int32_t camera_vpf_mipi_ipi_reset(vpf_handle_t vin_fd, uint32_t ipi_index, uint32_t ipi_enable);

extern int32_t camera_vpf_vin_get_gpio(camera_vin_attr_t *vin_attr, int32_t m_enable, int32_t m_level,
				enum vcon_gpio_e name, int32_t *gpio, int32_t *level);

extern int32_t camera_vpf_get_vin_attr(vpf_handle_t vin_fd, camera_vin_attr_t *vin_attr);
extern int32_t camera_vpf_get_vin_attr_tx(vpf_handle_t vin_fd, camera_vin_attr_t *vin_attr);
extern int32_t camera_vpf_vin_attach_camera(vpf_handle_t vin_fd, int32_t camera_index, mipi_config_t *mipi_config);
extern int32_t camera_vpf_vin_detach_camera(vpf_handle_t vin_fd, int32_t camera_index, mipi_config_t *mipi_config);
extern int32_t camera_vpf_vin_attach_deserial(vpf_handle_t vin_fd, int32_t deserial_index, int32_t deserial_link,
			int32_t camera_index, mipi_config_t *mipi_config);
extern int32_t camera_vpf_vin_detach_deserial(vpf_handle_t vin_fd, int32_t deserial_index, int32_t deserial_link,
			int32_t camera_index, mipi_config_t *mipi_config);
extern int32_t camera_vpf_vin_attach_txser(vpf_handle_t vin_fd, int32_t txser_index, mipi_config_t *mipi_config);
extern int32_t camera_vpf_vin_detach_txser(vpf_handle_t vin_fd, int32_t txser_index, mipi_config_t *mipi_config);

extern int32_t camera_vpf_vflow_stop(hbn_vflow_handle_t vflow_fd);
extern int32_t camera_vpf_vflow_start(hbn_vflow_handle_t vflow_fd);
extern int32_t camera_vpf_update_cali_name(hbn_vnode_handle_t isp_vnode_fd);
extern hbn_vnode_handle_t camera_vpf_vflow_to_vin_fd(hbn_vflow_handle_t vflow_fd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAM_VPF_H__ */
