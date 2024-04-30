/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file vpf_data_info.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __VPF_DATA_INFO_H__
#define __VPF_DATA_INFO_H__

#include <stdint.h>

#include "cam_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CAM_CONFIG_LIBVPF_EN
#include "vpf_inter_interface.h"
#include "vin_cfg.h"

#define VCON_SENSOR_ERR_MAX	SENSOR_ERR_PIN_NUM
#define VCON_LPWM_CHN_MAX	LPWM_CHN_NUM
#else /* CAM_CONFIG_LIBVPF_EN */
/**
 * @def VCON_SENSOR_ERR_MAX
 * vcon attr number max of sensor_err
 */
#define VCON_SENSOR_ERR_MAX	(4)
/**
 * @def VCON_LPWM_CHN_MAX
 * vcon attr number max of lpwm channel
 */
#define VCON_LPWM_CHN_MAX	(4)

/**
 * @enum vcon_gpio_e
 * vcon gpio type enum
 * @NO{S10E01C02}
 */
enum vcon_gpio_e {
	/* poc gpios */
	VGPIO_POC_PWREN,
	VGPIO_POC_EN,
	VGPIO_POC_INT,
	VGPIO_POC_ENC0,
	VGPIO_POC_ENC1,
	VGPIO_POC_ENC2,
	VGPIO_POC_ENC3,
	VGPIO_POC_NC,
	/* des gpios */
	VGPIO_DES_PWREN,
	VGPIO_DES_PWDN,
	VGPIO_DES_LOCK,
	VGPIO_DES_ERRB,
	VGPIO_DES_CFG0,
	VGPIO_DES_CFG1,
	VGPIO_DES_ERROR0,
	VGPIO_DES_ERROR1,
	VGPIO_DES_ERROR2,
	VGPIO_DES_ERROR3,
	VGPIO_DES_NC1,
	VGPIO_DES_NC2,
	/* ser gpios */
	VGPIO_SER_PWREN,
	VGPIO_SER_PWDN,
	VGPIO_SER_LOCK,
	VGPIO_SER_ERRB,
	VGPIO_SER_CFG0,
	VGPIO_SER_CFG1,
	VGPIO_SER_CFG2,
	VGPIO_SER_NC,
	/* oth gpios */
	VGPIO_OTH_0,
	VGPIO_OTH_1,
	VGPIO_OTH_2,
	VGPIO_OTH_3,
	/* local info */
	VGPIO_NUM,
	VGPIO_POC_BASE = VGPIO_POC_PWREN,
	VGPIO_POC_NUM = VGPIO_DES_PWREN - VGPIO_POC_PWREN,
	VGPIO_DES_BASE = VGPIO_DES_PWREN,
	VGPIO_DES_NUM = VGPIO_SER_PWREN - VGPIO_DES_PWREN,
	VGPIO_SER_BASE = VGPIO_SER_PWREN,
	VGPIO_SER_NUM = VGPIO_OTH_0 - VGPIO_SER_PWREN,
	VGPIO_OTH_BASE = VGPIO_OTH_0,
	VGPIO_OTH_NUM = VGPIO_NUM - VGPIO_OTH_0,
};

/**
 * @struct vcon_attr_s
 * vcon attr config struct
 * @NO{S10E01C02}
 */
typedef struct vcon_attr_s {
	/* must be int32_t : do not modify it */
	int32_t attr_valid;	// 属性有效果性配置.
	int32_t bus_main;	// 主I2C主线索引.
	int32_t bus_second;	// 次I2C主线索引.
	int32_t gpios[VGPIO_NUM]; // gpio索引.
	int32_t sensor_err[VCON_SENSOR_ERR_MAX]; // sensor_err pin索引.
	int32_t lpwm_chn[VCON_LPWM_CHN_MAX]; // lpwm channel索引.
	int32_t rx_phy_mode;	// rx phy模式: 0-notuse,1-dphy,2-cphy.
	int32_t rx_phy_index;	// rx phy索引.
	int32_t rx_phy_link;	// rx phy链接索引-复合类型使用.
	int32_t tx_phy_mode;	// tx phy模式: 0-notuse,1-csi,2-dsi.
	int32_t tx_phy_index;	// tx phy索引.
	int32_t tx_phy_link;	// tx phy链接索引-复合类型使用.
	int32_t vcon_type;	// vcon类型: 0-独立,1-复合主,2-复合从.
	int32_t vcon_link;	// vcon链接索引-复合类型使用.
} vcon_attr_t;  // vcon属性结构体.

typedef struct lpwm_attr_s{
	uint32_t trigger_source;
	uint32_t trigger_mode;
	uint32_t period;
	uint32_t offset;
	uint32_t duty_time;
	uint32_t threshold;
	uint32_t adjust_step;
} lpwm_attr_t;  //Lpwm attribute structure

typedef struct cim_attr{
	uint32_t           mipi_en;             //enable mipi
	uint32_t           mipi_rx;
	uint32_t           vc_index;            // vx index
	uint32_t           ipi_channel;         // DOL2 use
	uint32_t           cim_pym_flyby;       // cim online pym  pym_sel Which way to choose
	uint32_t           cim_isp_flyby;       //cim online ISP  isp_chx
	uint32_t           cim_oters_reserved;
} cim_attr_t;

typedef struct vin_node_attr_s{
	cim_attr_t     cim_attr;    // Cim attribute
	lpwm_attr_t    lpwm_attr;   // Lpwm attribute
	vcon_attr_t    vcon_attr;   // Vcon Properties
	uint32_t       flow_id;
	uint32_t       magicNumber;
} vin_node_attr_t;  //Vin attribute structure

typedef struct vin_attr_s{
	vin_node_attr_t vin_node_attr;
	uint32_t        cim_oters_reserved;
	uint32_t	magicNumber;
} vin_attr_t;

/**
 * @struct mipi_attr_s
 * mipi base config struct as attr, include rx and tx
 * @NO{S10E02C07}
 */
typedef struct mipi_attr_s {
	int32_t attr_valid;
	int32_t attach;		// attach操作类型.
	int32_t rx_enable;	// RX设备使能.
	int32_t tx_enable;	// TX设备使能.
	int32_t tx_index;	// TX设备选择.

	struct mipi_host_cfg_s rx_attr;		// RX设备属性.
	struct mipi_dev_cfg_s tx_attr;		// TX设备属性.
} mipi_attr_t;  // mipi属性结构体.

/**
 * @struct mipi_attr_ex_s
 * mipi param config struct as extra attr, include rx and tx
 * @NO{S10E02C07}
 */
typedef struct mipi_attr_ex_s {
	int32_t attr_valid;
	int32_t reserved;
	uint64_t rx_ex_mask;	// RX增强属性掩码.
	uint64_t tx_ex_mask;	// TX增强属性掩码.

	struct mipi_host_param_s rx_attr_ex;// RX增强属性;
	struct mipi_dev_param_s tx_attr_ex; // TX增强属性;
} mipi_attr_ex_t;

/**
 * @enum vin_attr_ex_type_s
 * vin ex_attr type enum
 * @NO{S10E02C07}
 */
typedef enum vin_attr_ex_type_s{
	VIN_STATIC_CIM_ATTR,
	VIN_STATIC_MIPI_ATTR,
	VIN_DYNAMIC_FPS_CTRL,
	VIN_DYNAMIC_LPWM_TRIGGER_SOURCE,
	VIN_DYNAMIC_LPWM_FPS,
	VIN_DYNAMIC_IPI_RESET,
	VIN_DYNAMIC_BYPASS_ENABLE,
	VIN_ATTR_EX_INVALID,
} vin_attr_ex_type_e;

/**
 * @struct vin_attr_ex_s
 * vin ex_attr struct with mipi only for test without vpf
 * @NO{S10E02C07}
 */
typedef struct vin_attr_ex_s{
	vin_attr_ex_type_e      ex_attr_type;          // Extended Attribute Type
	mipi_attr_ex_t          mipi_ex_attr;          // Mipi extension attributes
	uint32_t                ipi_reset;             // mipi  ipi reset
	uint32_t                bypass_enable;         // bypass enable
} vin_attr_ex_t;

/**
 * @struct vcon_inter_attr_s
 * vcon internal attr for pipe and camera attach operation
 * @NO{S10E02C07}
 */
typedef struct vcon_inter_attr_s {
	int32_t attr_valid;
	int32_t flow_id;		// vflow id, VIN_NODE赋值.
	int32_t ctx_id;		// vctx id, VIN_NODE赋值.
	int32_t attach;			// attach操作类型.
	int32_t deserial_attach;	// 是否连上deserial.
	int32_t deserial_index;		// 所连deserial索引.
	int32_t deserial_link;		// 所连deserial上link.
	int32_t sensor_attach;		// 是否连上sensor.
	int32_t sensor_index;		// 所连sensor索引.
} vcon_inter_attr_t;  // vcon内部属性结构体.

typedef struct vin_inter_attr_s {
	mipi_attr_t mipi_inter_attr;
	vcon_inter_attr_t vcon_inter_attr;
} vin_inter_attr_t;
#endif /* CAM_CONFIG_LIBVPF_EN */

/**
 * @struct camera_vin_attr_s
 * the vin attr struct for camera attach
 * @NO{S10E02C07}
 */
typedef struct camera_vin_attr_s {
	uint32_t flow_id;
	uint32_t mipi_en;             //enable mipi
	uint32_t mipi_rx;
	uint32_t vc_index;            // vx index
	uint32_t ipi_channel;         // DOL2 use
	vcon_attr_t vcon_attr;
} camera_vin_attr_t;

#define VIN_ATTR_TO_CAMERA_INDEX(p)	((p)->flow_id)
#define VIN_ATTR_TO_DESERIAL_INDEX(p)	(((p)->vcon_attr.vcon_type != 2) ? (p)->vcon_attr.rx_phy_index : (p)->vcon_attr.vcon_link)
#define VIN_ATTR_TO_RX_INDEX(p)		((p)->vcon_attr.rx_phy_index)
#define VIN_ATTR_TO_TXSER_INDEX(p)	(((p)->vcon_attr.vcon_type != 2) ? (p)->vcon_attr.tx_phy_index : (p)->vcon_attr.vcon_link)

/**
 * @def VCON_ATTR_V_BUS_MAIN
 * vcon attr valid bit mask: bus_main
 */
#define VCON_ATTR_V_BUS_MAIN	(0x1 << 0)
/**
 * @def VCON_ATTR_V_BUS_SEC
 * vcon attr valid bit mask: bus_sec
 */
#define VCON_ATTR_V_BUS_SEC	(0x1 << 1)
/**
 * @def VCON_ATTR_V_GPIO_POC
 * vcon attr valid bit mask: gpio_poc
 */
#define VCON_ATTR_V_GPIO_POC	(0x1 << 2)
/**
 * @def VCON_ATTR_V_GPIO_DES
 * vcon attr valid bit mask: gpio_des
 */
#define VCON_ATTR_V_GPIO_DES	(0x1 << 3)
/**
 * @def VCON_ATTR_V_GPIO_SER
 * vcon attr valid bit mask: gpio_ser
 */
#define VCON_ATTR_V_GPIO_SER	(0x1 << 4)
/**
 * @def VCON_ATTR_V_GPIO_OTH
 * vcon attr valid bit mask: gpio_oth
 */
#define VCON_ATTR_V_GPIO_OTH	(0x1 << 5)
/**
 * @def VCON_ATTR_V_SENSOR_ERR
 * vcon attr valid bit mask: sensor_err
 */
#define VCON_ATTR_V_SENSOR_ERR	(0x1 << 6)
/**
 * @def VCON_ATTR_V_LPWM
 * vcon attr valid bit mask: lpwm index
 */
#define VCON_ATTR_V_LPWM	(0x1 << 7)
/**
 * @def VCON_ATTR_V_MIPI_RX
 * vcon attr valid bit mask: mipi rx index
 */
#define VCON_ATTR_V_MIPI_RX	(0x1 << 8)
/**
 * @def VCON_ATTR_V_MIPI_TX
 * vcon attr valid bit mask: mipi tx index
 */
#define VCON_ATTR_V_MIPI_TX	(0x1 << 9)
/**
 * @def VCON_ATTR_V_TYPE
 * vcon attr valid bit mask: type
 */
#define VCON_ATTR_V_TYPE	(0x1 << 10)
/**
 * @def VCON_ATTR_V_ALL
 * vcon attr valid bit mask: all bits valid
 */
#define VCON_ATTR_V_ALL		((0x1 << 11) - 1)
/**
 * @def VCON_ATTR_V_BUS
 * vcon attr valid bit mask: bus type: bus_main, bus_sec
 */
#define VCON_ATTR_V_BUS		(VCON_ATTR_V_BUS_MAIN | VCON_ATTR_V_BUS_SEC)
/**
 * @def VCON_ATTR_V_GPIO
 * vcon attr valid bit mask: gpio type: gpio_xx
 */
#define VCON_ATTR_V_GPIO	(VCON_ATTR_V_GPIO_POC | VCON_ATTR_V_GPIO_DES | \
				 VCON_ATTR_V_GPIO_SER | VCON_ATTR_V_GPIO_OTH)
/**
 * @def VCON_ATTR_V_MIPI
 * vcon attr valid bit mask: mipi type: rx, tx
 */
#define VCON_ATTR_V_MIPI	(VCON_ATTR_V_MIPI_RX | VCON_ATTR_V_MIPI_TX)

/**
 * @def VCON_ATTR_INVALID
 * vcon attr invalid value
 */
#define VCON_ATTR_INVALID	(-1)
/**
 * @def VCON_ATTR_INVGPIO
 * vcon gpio attr invalid value
 */
#define VCON_ATTR_INVGPIO	(0)

/**
 * @def VCON_GPIO_NAMES_POC
 * vcon poc gpio name string array
 */
#define VCON_GPIO_NAMES_POC  \
	/* poc gpios */ \
	"poc_pwren", \
	"poc_en", \
	"poc_int", \
	"poc_enc0", \
	"poc_enc1", \
	"poc_enc2", \
	"poc_enc3", \
	"poc_nc"

/**
 * @def VCON_GPIO_NAMES_DES
 * vcon des gpio name string array
 */
#define VCON_GPIO_NAMES_DES  \
	/* des gpios */ \
	"des_pwren", \
	"des_pwdn", \
	"des_lock", \
	"des_errb", \
	"des_cfg0", \
	"des_cfg1", \
	"des_error0", \
	"des_error1", \
	"des_error2", \
	"des_error3", \
	"des_nc1", \
	"des_nc2"

/**
 * @def VCON_GPIO_NAMES_SER
 * vcon ser gpio name string array
 */
#define VCON_GPIO_NAMES_SER  \
	/* ser gpios */ \
	"ser_pwren", \
	"ser_pwdn", \
	"ser_lock", \
	"ser_errb", \
	"ser_cfg0", \
	"ser_cfg1", \
	"ser_cfg2", \
	"ser_nc"

/**
 * @def VCON_GPIO_NAMES_OTH
 * vcon oth gpio name string array
 */
#define VCON_GPIO_NAMES_OTH  \
	/* oth gpios */ \
	"oth_0", \
	"oth_1", \
	"oth_2", \
	"oth_3"

/**
 * @def VCON_ATTR_NAMES
 * vcon attr name string array
 */
#define VCON_ATTR_NAMES { \
	"bus_main", \
	"bus_second", \
	VCON_GPIO_NAMES_POC, \
	VCON_GPIO_NAMES_DES, \
	VCON_GPIO_NAMES_SER, \
	VCON_GPIO_NAMES_OTH, \
	"sensor_err0", "sensor_err1", "sensor_err2", "sensor_err3", \
	"lpwm_chn0", "lpwm_chn1", "lpwm_chn2", "lpwm_chn3", \
	"rx_phy_mode", "rx_phy_index", "rx_phy_link", \
	"tx_phy_mode", "tx_phy_index", "tx_phy_link", \
	"vcon_type", "vcon_link", \
}

/**
 * @def VCON_ATTR_V_NUMS
 * vcon attr vflag number array
 */
#define VCON_ATTR_V_NUMS { \
	1, 1, \
	VGPIO_POC_NUM, \
	VGPIO_DES_NUM, \
	VGPIO_SER_NUM, \
	VGPIO_OTH_NUM, \
	4, 4, 3, 3, 2, \
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __VPF_DATA_INFO_H__ */
