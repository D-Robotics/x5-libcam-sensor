/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_recov.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __HB_CAMERA_RECOV_H__
#define __HB_CAMERA_RECOV_H__
#include <bits/pthreadtypes.h>
#include <stdint.h>
#include <sys/types.h>
#include <semaphore.h>

#include "legacy/inc/cam_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_RECOV_DEFAULT_RETRY_COUNT	(3)
#define CAM_RECOV_DEFAULT_TIMEOUT_MS	(3000)

#define CAM_RECOV_RETRY_INTER_EN	(1)
#define CAM_RECOV_RETRY_INTER_CNT	(10)
#define CAM_RECOV_RETRY_INTER_SLEEP_MS	(1000)

enum {
	CAM_RECOV_LINK_DEFAULT,
	CAM_RECOV_LINK_DOWN,
	CAM_RECOV_LINK_DOWN_IGNORE,
	CAM_RECOV_LINK_UP,
	CAM_RECOV_LINK_UP_TIMEOUT,
	CAM_RECOV_LINK_UP_IGNORE,
};

typedef struct recov_node_info_s {
	struct recov_node_info_s *next;
	uint32_t port;
	uint32_t link_status;
	uint32_t recov_status;
	uint32_t recov_count;
	uint32_t recov_enable;
	struct timeval down_tv;
	struct timeval up_tv;
	struct timeval recov_tv;
	pthread_mutex_t lock;
} recov_node_info_t;

typedef struct recov_node_pool_s {
	recov_node_info_t nodes[CAM_MAX_NUM];
	uint32_t port_idle_mask;
} recov_node_pool_t;

typedef struct recov_list_s {
	recov_node_info_t *head;
	recov_node_info_t *tail;
	uint32_t count;
	pthread_mutex_t lock;
} recov_list_t;

typedef struct cam_recov_mgr_s cam_recov_mgr_t;

typedef struct recov_work_s {
	recov_list_t list;
	sem_t recov_sem;
	pthread_t thread_id;
	int32_t thread_running;
	int32_t port_mask;
	int32_t work_id;
	cam_recov_mgr_t *mgr;
} recov_work_t;

#define CAM_IIC_BUS_NUM	10u

typedef struct cam_recov_mgr_s {
	uint32_t recov_enable;
	int32_t recov_timeout;
	int32_t recov_retry;
	int32_t reserved[5];
	int32_t recov_inited;
	volatile int32_t recov_run;
	recov_node_pool_t pool;
	recov_work_t work[CAM_IIC_BUS_NUM];
} cam_recov_mgr_t;

extern int32_t cam_recov_init(void);
extern int32_t cam_recov_deinit(void);
extern int32_t cam_recov_enable(int32_t port);
extern int32_t cam_recov_disable(int32_t port);
extern int32_t cam_recov_status(int32_t port, uint32_t *link_status, uint32_t *recoving);
extern void cam_recov_link_notify(uint32_t port_mask, int32_t link_status);

#ifdef __cplusplus
}
#endif

#endif //__HB_CAMERA_RECOV_H__
