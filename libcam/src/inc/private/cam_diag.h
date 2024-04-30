/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_diag.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __HB_CAMERA_DIAG_H__
#define __HB_CAMERA_DIAG_H__
#include <bits/pthreadtypes.h>
#include <stdint.h>
#include <sys/types.h>
#include <semaphore.h>
#include "./cam_diag_list.h"
#include "legacy/inc/cam_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CAM_DIAG_DBG
/* diag debug */
#undef vin_dbg
#include "camera_env.h"
#define vin_dbg(fmt, ...) do { \
		if ((camera_env_get_ulong(CAMENV_DIAG_DEBUG, 0U) & CAM_DIAG_DBG) != 0U) \
			cam_dbg(fmt, ##__VA_ARGS__); \
	} while (0)
#endif

#define DIAG_BIT(i)	(1 << (i))
#define CAM_DIAG_DEFAULT_POLLING_MS	200
#define CAM_DIAG_POLLING_MS_MIN		50

#define DIAG_SNR_PORT_MASK	((1 << (CAM_MAX_NUM))-1)
#define DIAG_SNR_PORT_NUM_DUMMY_LINK	(4U)
#define DIAG_SNR_PORT_BIT_DUMMY_LINK	(32U - DIAG_SNR_PORT_NUM_DUMMY_LINK)
#if CAM_MAX_NUM >= DIAG_SNR_PORT_BIT_DUMMY_LINK
#error CAM_MAX_NUM too large to use dummy link bit
#endif

#define IS_GPIO0_GROUPA(i) (((i) >= 480 && (i) <= (511)) ? 1U : 0U)
#define IS_GPIO0_GROUPB(i) (((i) >= 448 && (i) <= (479)) ? 1U : 0U)
#define IS_GPIO0_GROUPC(i) (((i) >= 416 && (i) <= (447)) ? 1U : 0U)
#define IS_GPIO0_GROUPD(i) (((i) >= 384 && (i) <= (415)) ? 1U : 0U)

#define IS_GPIO1_GROUPA(i) (((i) >= 352 && (i) <= (383)) ? 1U : 0U)
#define IS_GPIO1_GROUPB(i) (((i) >= 320 && (i) <= (351)) ? 1U : 0U)
#define IS_GPIO1_GROUPC(i) (((i) >= 288 && (i) <= (319)) ? 1U : 0U)
#define IS_GPIO1_GROUPD(i) (((i) >= 256 && (i) <= (282)) ? 1U : 0U)


enum _diag_dev_type {
	CAM_DES = 1,
	CAM_POC,
	CAM_SER,
	CAM_SNR,
	CAM_MAX_DEV
};

#define CAM_DIAG_GPIO_HIGH	1
#define CAM_DIAG_GPIO_LOW	0

enum cam_diag_des_subtype {
	DES_UNLOCK = 0,
	DES_DEC_ERR_FLAG,
	DES_IDLE_ERR_FLAG,
	DES_MAX_RT_FLAG_0,
	DES_RT_CNT_FLAG_0,
	DES_VID_PXL_CRC_ERR_0,
	DES_LMO_ERR,
	DES_LCRC_ERR = 9,
	DES_MEM_ECC_ERR2,
	DES_LFLT_INT,
	DES_VDDBAD_INT,
	DES_PORZ_INT,
	DES_VDDCMP_INT,
	DES_ERRB,
	DES_SUBTYPE_MAX
};
enum cam_diag_snr_subtype {
	SNR_INT = 1,
	SNR_STREAM_OFF,
	SNR_VOLTAGE,
	SNR_FCNT_TEST,
	SNR_TEMP,
	SNR_ROW_COLUMN_ID,
	SNR_PLL_CLOCK,
	SNR_I2C_CRC,
	SNR_SCCB,
	SNR_RAM_CRC,
	SNR_ROM_CRC,
	SNR_ONLINE_PIXEL,
	SNR_TEST_PATTERN,
	SNR_ERRB_CHECK,
	SNR_SUBTYPE_MAX
};
enum cam_diag_poc_subtype {
	POC_UV_ERR = 0,
	POC_OV_ERR,
	POC_OC_ERR,
	POC_TS_ERR,
	POC_UVDD_ERR,
	POC_OVDD_ERR,
	POC_UVIN_ERR,
	POC_OVIN_ERR,
	POC_SUBTYPE_MAX
};

#define CAM_DIAG_PORT_NODES	(CAM_MAX_NUM*SNR_SUBTYPE_MAX + \
	DES_MAX_NUM*DES_SUBTYPE_MAX + DES_MAX_NUM*POC_SUBTYPE_MAX + 50U)

#define _DIAG_SUB_ID_BITS	4u
#define _DIAG_SUB_TYPE_BITS	8u
#define _DIAG_DEV_LINK_BITS	4u
#define _DIAG_DEV_RX_BITS	4u

#define CAM_DIAG_SUBTYPE_MAX	((1 << _DIAG_SUB_TYPE_BITS)-1)
#define _DIAG_SUB_TYPE_MASK	((1 << _DIAG_SUB_TYPE_BITS)-1)
#define _DIAG_SUB_ID_MASK	((1 << _DIAG_SUB_ID_BITS)-1)
#define _DIAG_DEV_RX_MASK	((1 << _DIAG_DEV_RX_BITS)-1)
#define _DIAG_DEV_LINK_MASK	((1 << _DIAG_DEV_LINK_BITS)-1)

#define _DIAG_SUB_ID_SHIFT	0u
#define _DIAG_SUB_TYPE_SHIFT	(_DIAG_SUB_ID_SHIFT+_DIAG_SUB_ID_BITS)
#define _DIAG_DEV_LINK_SHIFT	0u
#define _DIAG_DEV_RX_SHIFT	(_DIAG_DEV_LINK_SHIFT+_DIAG_DEV_RX_BITS)

#define CAM_DIAG_DEVINDEX(dev_rx, dev_link) \
	(((dev_rx)  << _DIAG_DEV_RX_SHIFT) | \
	 ((dev_link)   << _DIAG_DEV_LINK_SHIFT))
#define CAM_DIAG_SUBID(sub_type, sub_id) \
	(((sub_type)  << _DIAG_SUB_TYPE_SHIFT) | \
	 ((sub_id)   << _DIAG_SUB_ID_SHIFT))

#define _DIAG_SUBID_BITS	(_DIAG_SUB_TYPE_BITS+_DIAG_SUB_ID_BITS)
#define _DIAG_DEVINDEX_BITS	(_DIAG_DEV_RX_BITS+_DIAG_DEV_LINK_BITS)
#define _DIAG_DEVTYPE_BITS	8u

#define _DIAG_SUBID_MASK	((1 << _DIAG_SUBID_BITS)-1)
#define _DIAG_DEVINDEX_MASK	((1 << _DIAG_DEVINDEX_BITS)-1)
#define _DIAG_DEVTYPE_MASK	((1 << _DIAG_DEVTYPE_BITS)-1)

#define _DIAG_SUBID_SHIFT	0
#define _DIAG_DEVINDEX_SHIFT	(_DIAG_SUBID_SHIFT+_DIAG_SUBID_BITS)
#define _DIAG_DEVTYPE_SHIFT	(_DIAG_DEVINDEX_SHIFT+_DIAG_DEVINDEX_BITS)


#define DIAG_SUBID(diag_id)	(((diag_id) >> _DIAG_SUBID_SHIFT) & _DIAG_SUBID_MASK)
#define DIAG_DEVINDEX(diag_id)	(((diag_id) >> _DIAG_DEVINDEX_SHIFT) & _DIAG_DEVINDEX_MASK)
#define DIAG_DEVTYPE(diag_id)	(((diag_id) >> _DIAG_DEVTYPE_SHIFT) & _DIAG_DEVTYPE_MASK)

#define DIAG_SUBTYPE(diag_id)	(((DIAG_SUBID(diag_id)) >> _DIAG_SUB_TYPE_SHIFT) & _DIAG_SUB_TYPE_MASK)
#define DIAG_SUB_TYPE(diag_id)	((diag_id) >> _DIAG_SUB_TYPE_SHIFT)
#define DIAG_SUB_ID(diag_id)	(((DIAG_SUBID(diag_id)) >> _DIAG_SUB_ID_SHIFT) & _DIAG_SUB_ID_MASK)
#define DIAG_SNR_PORT(diag_id)	(((diag_id >> _DIAG_DEVINDEX_SHIFT) & 0xFF) - 1)

#define CAM_DIAG_ID(diag_dev_type, diag_dev_index, diag_subid) \
	(((diag_dev_type)  << _DIAG_DEVTYPE_SHIFT) | \
	 ((diag_dev_index) << _DIAG_DEVINDEX_SHIFT) | \
	 ((diag_subid)   << _DIAG_SUBID_SHIFT))

#define CAM_DIAG_ID_AD(diag_dev_type, diag_dev_index, diag_subtype, diag_sub_id) \
	(((diag_dev_type)  << _DIAG_DEVTYPE_SHIFT) | \
	 ((diag_dev_index) << _DIAG_DEVINDEX_SHIFT) | \
	 ((CAM_DIAG_SUBID(diag_subtype, diag_sub_id)) << _DIAG_SUBID_SHIFT))

#define CAM_SNR_DIAG_ID(port, subtype, subid) \
	(((CAM_SNR) << _DIAG_DEVTYPE_SHIFT) | \
	((port) << _DIAG_DEVINDEX_SHIFT) | \
	((subtype) << _DIAG_SUB_TYPE_SHIFT) | \
	((subid) << _DIAG_SUB_ID_SHIFT))

#define CAM_DES_DIAG_ID(rx, link, subtype, subid) \
	(((CAM_DES)  << _DIAG_DEVTYPE_SHIFT) | \
	(CAM_DIAG_DEVINDEX(rx, link) << _DIAG_DEVINDEX_SHIFT) | \
	((subtype)  << _DIAG_SUB_TYPE_SHIFT) | \
	((subid)   << _DIAG_SUB_ID_SHIFT))

#define CAM_POC_DIAG_ID(rx, link, subtype, subid) \
	(((CAM_POC)  << _DIAG_DEVTYPE_SHIFT) | \
	(CAM_DIAG_DEVINDEX(rx, link) << _DIAG_DEVINDEX_SHIFT) | \
	((subtype)  << _DIAG_SUB_TYPE_SHIFT) | \
	((subid)   << _DIAG_SUB_ID_SHIFT))

enum cam_diag_flag {
	MON_INIT = 1,
	MON_START,
	REPORT_EN,  // monitor only
	REPORT_ONCE,
	RECOVERY_EN,
	MON_DELAY,
	LINK_LOCK
};

#define DIAG_MON_INIT		DIAG_BIT(MON_INIT)
#define DIAG_MON_START		DIAG_BIT(MON_START)
#define DIAG_REPORT_EN		DIAG_BIT(REPORT_EN)
#define DIAG_REPORT_ONCE	DIAG_BIT(REPORT_ONCE)
#define DIAG_RECOVERY_EN	DIAG_BIT(RECOVERY_EN)
#define DIAG_MON_DELAY		DIAG_BIT(MON_DELAY)
#define DIAG_LINK_LOCK		DIAG_BIT(LINK_LOCK)

#define DIAG_FLAG_ISEN(flag, c) ((((flag)->diag_flag) & (c)) != 0)

enum cam_diag_type {
	CAM_DIAG_GPIO = 1,
	CAM_DIAG_EGPIO,
	CAM_DIAG_REG,
	CAM_DIAG_FCHM,
	CAM_DIAG_MAX
};

enum cam_diag_status {
	CAM_DIAG_NORMAL = 0,
	CAM_DIAG_FAULT = 1,
};

#define REG_IIC_TYPE(reg_type) ((reg_type)&(0xFF))
#define REG_JUDGE_TYPE(reg_type) ((reg_type)&0xFF>>8)

#define CAM_GPIO_TYPE(gpio_index)	\
	(((gpio_index) >= 480 && (gpio_index) <= 511) ||	\
	 ((gpio_index) >= 352 && (gpio_index) <= 383))		\
	? CAM_DIAG_EGPIO : CAM_DIAG_GPIO

enum _reg_value_type {
	VALUE_TYPE,
	BIT_TYPE,
};

#define DIAG_REG_BIG_ENDIAN 0
#define DIAG_REG_LITTLE_ENDIAN 1


#define _REG_LEN_BITS		4u
#define _REG_VALUE_TYPE_BITS	4u
#define _REG_BYTE_ORDER_BITS	2u
#define _REG_PRE_READ_BITS	2u

#define _REG_LEN_MASK	((1 << _REG_LEN_BITS) - 1)
#define _REG_VALUE_TYPE_MASK	((1 << _REG_VALUE_TYPE_BITS) - 1)
#define _REG_BYTE_ORDER_MASK	((1 << _REG_BYTE_ORDER_BITS) - 1)
#define _REG_PRE_READ_MASK	((1 << _REG_PRE_READ_BITS) - 1)

#define _REG_LEN_SHIFT	0
#define _REG_VALUE_TYPE_SHIFT	(_REG_LEN_SHIFT+_REG_LEN_BITS)
#define _REG_BYTE_ORDER_SHIFT	(_REG_VALUE_TYPE_SHIFT+_REG_VALUE_TYPE_BITS)
#define _REG_PRE_READ_SHIFT	(_REG_BYTE_ORDER_SHIFT+_REG_BYTE_ORDER_BITS)

#define DIAG_REGLEN(reg_type)	(((reg_type) >> _REG_LEN_SHIFT) & _REG_LEN_MASK)
#define DIAG_REGVTYPE(reg_type)	(((reg_type) >> _REG_VALUE_TYPE_SHIFT) & _REG_VALUE_TYPE_MASK)
#define DIAG_REG_BYTE_ORDER(reg_type)	(((reg_type) >> _REG_BYTE_ORDER_SHIFT) & _REG_BYTE_ORDER_MASK)
#define DIAG_REG_NEED_PREREAD(reg_type)	(((reg_type) >> _REG_PRE_READ_SHIFT) & _REG_PRE_READ_MASK)
#define DIAG_REG_IS_LITTLE_ENDIAN(reg_type) \
	((DIAG_REG_BYTE_ORDER(reg_type)) == DIAG_REG_LITTLE_ENDIAN)

#define DIAG_REG_TYPE(value_type, reg_len) DIAG_REG_TYPE_ALL(0, 0, value_type, reg_len)
#define DIAG_REG_TYPE_P(value_type, reg_len) DIAG_REG_TYPE_ALL(1, 0, value_type, reg_len)
#define DIAG_REG_TYPE_LITTLE_ENDIAN(value_type, reg_len) DIAG_REG_TYPE_ALL(0, 1, value_type, reg_len)

#define DIAG_REG_TYPE_ALL(reg_need_pre_read, reg_byte_order, reg_value_type, reg_len) \
	(((reg_need_pre_read)  << _REG_PRE_READ_SHIFT) | \
	 ((reg_byte_order) << _REG_BYTE_ORDER_SHIFT)) | \
	(((reg_value_type)  << _REG_VALUE_TYPE_SHIFT) | \
	 ((reg_len) << _REG_LEN_SHIFT))


/**
 * @struct gpio_diag_info
 * gpio diag node info sturct
 * @NO{S10E02C07}
 */
struct gpio_diag_info {
	uint8_t gpio_type;
	uint32_t gpio_index;
	uint8_t gpio_active;
	int32_t gpio_fd;
	int32_t gpio_value_last;
	int32_t gpio_value;
	uint32_t gpio_count;
	uint32_t gpio_nc;
};

struct reg_diag_info {
	uint16_t reg_type;
	uint8_t reg_bus;
	uint8_t reg_dev;
	int32_t reg_addr;
	int32_t reg_pmask;
	int32_t reg_value;
	int32_t reg_pvalue;
	uint16_t reg_mask;
	int16_t reg_min;
	uint16_t reg_active;
	uint16_t reg_max;
	uint16_t reg_test;
};

/**
 * @struct diag_info_s
 * camera diag node info struct as union
 * @NO{S10E02C07}
 */
struct diag_info_s {
	// uint8_t type;
	union {
		struct gpio_diag_info gpio;
		struct reg_diag_info reg;
	}t;
};

/**
 * @struct diag_node_info_s
 * camera diag node data struct
 * @NO{S10E02C07}
 */
typedef struct diag_node_info_s {
	struct list_node list_node;
	struct diag_node_info_s *stype_head;
	struct diag_node_info_s *node_next;
	struct diag_node_info_s *node_report_next;
	uint32_t diag_id;
	uint32_t diag_flag;
	uint32_t diag_status;
	struct timespec diag_tv;
	uint32_t diag_count;
	uint32_t diag_type;
	uint32_t port_mask;
	struct diag_info_s diag_info;
	int32_t (*fault_judging)(struct diag_node_info_s *node);
	int32_t (*fault_clear)(struct diag_node_info_s *node);
	int32_t (*test_fault_inject)(struct diag_node_info_s *node, int32_t inject);
	int32_t (*cb_inject)(struct diag_node_info_s *node);
	void *cb_data;
	int32_t inject_sfault;
	struct diag_node_info_s *diag_sub_list;
} diag_node_info_t;

/**
 * @struct diag_report_list
 * camera diag report work list
 * @NO{S10E02C07}
 */
struct diag_report_list {
	pthread_mutex_t lock;
	diag_node_info_t *head;
	diag_node_info_t *tail;
	uint32_t count;
};

/**
 * @struct diag_node_pool_s
 * camera diag node pool
 * @NO{S10E02C07}
 */
typedef struct diag_node_pool_s {
	diag_node_info_t dnode_array[CAM_DIAG_PORT_NODES];
	uint32_t dnode_array_index;
	pthread_mutex_t lock;
} diag_node_pool_t;

/**
 * @struct diag_des_link_dummy_s
 * camera diag node pool
 * @NO{S10E02C07}
 */
typedef struct diag_des_link_dummy_s {
	diag_node_info_t *dnode[DES_MAX_NUM][DIAG_SNR_PORT_NUM_DUMMY_LINK][DES_SUBTYPE_MAX];
	uint32_t count_link[DES_MAX_NUM][DIAG_SNR_PORT_NUM_DUMMY_LINK];
	uint32_t count_des[DES_MAX_NUM];
	uint32_t count_all;
	pthread_mutex_t lock;
} diag_des_link_dummy_t;

// #define CAM_DIAG_MON_NUM CAM_DIAG_MAX
// #define CAM_DIAG_REPORT_NUM 1
// #define CAM_DIAG_SET_NUM (CAM_DIAG_MAX+)
enum cam_diag_thread {
	GPIO_TH		= 0,
	EGPIO_TH	= 1,
	REG_TH		= 2,
	REPORT_TH	= 3,
	CAM_DIAG_TH_MAX,
};

/**
 * @struct diag_mon_list
 * camera diag monitor work list
 * @NO{S10E02C07}
 */
struct diag_mon_list {
	struct list_node dlist;
	uint32_t count;
	pthread_mutex_t list_lock;
};

struct cam_diag_mgr;

/**
 * @struct cam_diag_mon
 * camera diag monitor work struct
 * @NO{S10E02C07}
 */
typedef struct cam_diag_mon {
	// char name[];
	int32_t index;
	int32_t mon_type;
	int32_t mon_run;
	int32_t diag_fd;
	pthread_t thread_id;
	pthread_t sub_thid;
	int32_t thread_running;
	int32_t sub_thread_running;
	struct cam_diag_mgr *mgr;
	struct diag_mon_list dlist_nodes;
	int32_t reserved[10];
} cam_diag_mon_s;

/**
 * @struct cam_report
 * camera diag report work struct
 * @NO{S10E02C07}
 */
typedef struct cam_report {
	sem_t report_sem;
	pthread_t thread_id;
	int32_t thread_running;
	struct cam_diag_mgr *mgr;
	struct diag_report_list dlist_nodes;
} cam_report_s;

#define CAM_IIC_BUS_NUM	10u

/**
 * @struct cam_diag_mgr
 * camera diag manager struct
 * @NO{S10E02C07}
 */
typedef struct cam_diag_mgr {
	struct cam_diag_mon mon_egpio;
	struct cam_diag_mon mon_gpio;
	struct cam_diag_mon mon_reg[CAM_IIC_BUS_NUM];
	struct cam_report cam_diag_report;
	diag_node_pool_t node_pool;
	diag_des_link_dummy_t link_dummy;
	volatile int32_t diag_run;
	volatile int32_t port_mask;
	pthread_mutex_t lock;
	int32_t diag_polling_ms;
	void *event_mapping;
} cam_diag_mgr_s;

int32_t cam_diag_create(void);
int32_t cam_diag_info_dump(char *fname);
void cam_diag_add_subnode(diag_node_info_t *dnode, diag_node_info_t *sub_dnode);
int32_t cam_diag_destroy(void);
int32_t cam_diag_init(void);
int32_t cam_diag_deinit(void);
uint32_t cam_diag_get_port_mask();
uint32_t cam_diag_set_port_mask(uint32_t port_mask, int32_t enable);
diag_node_info_t *cam_diag_get_nodes(uint32_t nums);
int32_t cam_diag_node_register(diag_node_info_t *diag_node);
uint32_t cam_diag_get_snr_status(uint32_t *init_port_mask, uint32_t *start_port_mask);

int32_t cam_diag_fault_inject(int32_t diag_id, int32_t inject);
int32_t cam_diag_soft_fault_inject(int32_t diag_id, int32_t inject);
int32_t cam_diag_show_node_info(int32_t diag_id);
uint32_t cam_get_init_status(diag_node_info_t *dnode);
int32_t cam_diag_fault_inject_by_port(uint32_t port_mask, int32_t inject);
int32_t cam_diag_fault_inject_by_dtype(uint32_t dev_type, int32_t inject);

#ifdef __cplusplus
}
#endif

#endif  //__HB_CAMERA_DIAG_H__
