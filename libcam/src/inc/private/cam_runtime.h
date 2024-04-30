/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_runtime.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_RUNTIME_H__
#define __CAM_RUNTIME_H__

#include <stdint.h>

#include "camera_mod_sensor.h"
#include "camera_mod_deserial.h"
#include "camera_mod_poc.h"
#include "camera_mod_txser.h"
#include "camera_mod_calib.h"
#include "camera_sys.h"

#include "cam_handle.h"
#ifdef CAM_DIAG
#include "cam_diag.h"
#endif
#ifdef CAM_RECOV
#include "cam_recov.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_DIAG_DISABLE		(0x1U)
#define CAMERA_DIAG_DISABLE_SENSOR	(0x3U)
#define CAMERA_DIAG_DISABLE_DESERIAL	(0x5U)
#define CAMERA_DIAG_DISABLE_POC		(0x9U)
#define CAMERA_DIAG_DISABLE_TXSER	(0x11U)

/**
 * @struct camera_patial_s
 * camera partial info struct
 * @NO{S10E02C07}
 */
typedef struct camera_partial_s {
	camera_handle_st *cam_handle;
	deserial_handle_st *des_handle;
	int32_t des_link;
	vpf_handle_t vin_handle;
} camera_partial_t;

/**
 * @struct camera_runtime_s
 * camera runtime info struct
 * @NO{S10E02C07}
 */
typedef struct camera_runtime_s {
	uint32_t handle_cnt;
	uint32_t handle_mask;
	uint32_t good_mask;
	uint32_t partial_cnt;
	uint32_t partial_mask;
	camera_handle_st *handles[CAM_CONFIG_CAMERA_MAX];
	sensor_info_t sensor_info[CAM_CONFIG_CAMERA_MAX];
	calib_info_t calib_info[CAM_CONFIG_CAMERA_MAX];
	camera_partial_t partial[CAM_CONFIG_CAMERA_MAX];
	camera_mutex_t handle_mutex;
} camera_runtime_t;

/**
 * @struct deserial_runtime_s
 * deserial runtime info struct
 * @NO{S10E02C07}
 */
typedef struct deserial_runtime_s {
	uint32_t handle_cnt;
	uint32_t handle_mask;
	uint32_t good_mask;
	deserial_handle_st *handles[CAM_CONFIG_DESERIAL_MAX];
	deserial_info_t deserial_info[CAM_CONFIG_DESERIAL_MAX];
	poc_info_t poc_info[CAM_CONFIG_DESERIAL_MAX];
	camera_mutex_t handle_mutex;
} deserial_runtime_t;

/**
 * @struct txser_runtime_s
 * txser runtime info struct
 * @NO{S10E02C07}
 */
typedef struct txser_runtime_s {
	uint32_t handle_cnt;
	uint32_t handle_mask;
	uint32_t good_mask;
	txser_handle_st *handles[CAM_CONFIG_TXSER_MAX];
	txser_info_t txser_info[CAM_CONFIG_TXSER_MAX];
	camera_mutex_t handle_mutex;
} txser_runtime_t;

/**
 * @struct camera_global_handle_s
 * camera global config info struct
 * @NO{S10E02C07}
 */
typedef struct camera_handle_mgr_s {
	int32_t inited;
	int32_t index;
	int32_t count;
	camera_mutex_t *mutex;
	camera_handle_head_t *heads[CAM_HANDLE_NUM_MAX];
} camera_handle_mgr_t;

/**
 * @struct camera_global_runtime_s
 * camera runtime info struct
 * @NO{S10E02C07}
 */
typedef struct camera_global_runtime_s {
	camera_handle_mgr_t mgr;
	camera_runtime_t cam;
	deserial_runtime_t des;
	txser_runtime_t txs;
	camera_global_config_t cfg;
#ifdef CAM_DIAG
	cam_diag_mgr_s cam_diag_mgr;
#endif
#ifdef CAM_RECOV
	cam_recov_mgr_t cam_recov_mgr;
#endif
} camera_global_runtime_t;

/* internal apis */
extern camera_global_runtime_t *camera_global_runtime(void);
#define camera_global()		(camera_global_runtime())
#define camera_g_mgr()		(&((camera_global())->mgr))
#define camera_g_cam()		(&((camera_global())->cam))
#define camera_g_des()		(&((camera_global())->des))
#define camera_g_txs()		(&((camera_global())->txs))
#define camera_g_cfg()		(&((camera_global())->cfg))
#ifdef CAM_DIAG
#define camera_g_diag()		(&((camera_global())->cam_diag_mgr))
#endif
#ifdef CAM_RECOV
#define camera_g_recov()	(&((camera_global())->cam_recov_mgr))
#endif

extern int32_t camera_global_config_init(camera_global_config_t *gconfig);
extern int32_t camera_addition_start(int32_t camera_index);
extern int32_t camera_addition_stop(int32_t camera_index);

extern int32_t camera_handle_mgr_get_id(void);
extern int32_t camera_handle_mgr_set_id(int32_t id, camera_handle_head_t *head);
extern camera_handle_head_t* camera_handle_mgr_by_id(int32_t id);
extern int32_t camera_handle_mgr_put_id(int32_t id);

extern int32_t camera_run_cam_partial_get(int32_t camera_index, camera_partial_t *partial);
extern int32_t camera_run_cam_get(int32_t camera_index, int32_t *p_good, camera_handle_st **p_hcam,
			sensor_info_t **p_sen_if, calib_info_t **p_cal_if);
extern int32_t camera_run_cam_get_by_vin(vpf_handle_t vin_fd, int32_t *p_good, camera_handle_st **p_hcam,
			sensor_info_t **p_sen_if, calib_info_t **p_cal_if);
extern int32_t camera_run_cam_get_pre(int32_t camera_index, camera_handle_st *hcam,
			sensor_info_t **p_sen_if, calib_info_t **p_cal_if);
extern uint32_t camera_run_cam_get_good_mask(void);
extern int32_t camera_run_des_get(int32_t deserial_index, int32_t *p_good, deserial_handle_st ** p_hdes,
			deserial_info_t **p_deserial_info, poc_info_t **p_poc_info);
extern uint32_t camera_run_des_get_good_mask(void);
extern int32_t camera_run_txs_get(int32_t txser_index, int32_t *p_good,
			txser_handle_st **p_htxs, txser_info_t **p_txser_info);
extern uint32_t camera_run_txs_get_good_mask(void);

extern int32_t camera_run_set_event_callback(camera_handle_st *hcam, void (*event_callback)(cam_event_t* fault_info));
extern int32_t camera_run_send_event(int32_t camera_index, uint32_t event_type, uint32_t module_id,
			uint32_t event_id, uint32_t status);

extern int32_t camera_run_reset(camera_handle_st *hcam);
extern int32_t camera_run_reset_by_index(int32_t camera_index);
extern int32_t camera_run_reset_ipi_by_index(int32_t camera_index, uint32_t ipi_enable);

extern int32_t camera_attach_to_vin(camera_handle_st *hcam, vpf_handle_t vin_fd);
extern int32_t camera_detach_from_vin(camera_handle_st *hcam);
extern int32_t camera_attach_deserial_to_vin(deserial_handle_st *hdes, camera_des_link_t link, vpf_handle_t vin_fd);
extern int32_t camera_detach_deserial_from_vin(deserial_handle_st *hdes, camera_des_link_t link);
extern int32_t camera_attach_to_deserial(camera_handle_st *hcam, deserial_handle_st *hdes, camera_des_link_t link);
extern int32_t camera_detach_from_deserial(camera_handle_st *hcam);
extern int32_t camera_attach_txser_to_vin(txser_handle_st *htxs, camera_txs_csi_t csi, vpf_handle_t vin_fd);
extern int32_t camera_detach_txser_from_vin(txser_handle_st *htxs, camera_txs_csi_t csi);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CAM_RUNTIME_H__ */
