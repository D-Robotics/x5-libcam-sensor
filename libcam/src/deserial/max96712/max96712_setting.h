/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_DESERIAL_MAX96712_SETTING_H_
#define UTILITY_DESERIAL_MAX96712_SETTING_H_

#define RESET_REG			0x13
#define RESET_VAL			0x40
#define RESET_ONESHOT_REG	0x18
#define GMSL_REG			0x10
#define GMSL_3GBPS			0x01
#define GMSL_6GBPS			0x02
#define GMSL_MASK			0x3
#define GMSL_LINK_SHIFT		4
#define REMTCH_REG			0x03
#define REMTCH_MASK(i)		(1 << ((i) * 2))  // reg 0x03: bit 0/2/4/6 -- link A/B/C/D
#define REMTCH_MASK_ALL		0x55
#define LINK_REG		0x06
#define LINK_ALL		0xF
#define LINK_MAP_REG1      0xF0
#define LINK_MAP_REG2      0xF1
#define LINK_NUM       4

#define LINKA_LOCK_REG		0x1a
#define LINKB_LOCK_REG		0xa
#define LINKC_LOCK_REG		0xb
#define LINKD_LOCK_REG		0xc
#define LINK_LOCK_MASK		BIT(3)
#define LINK_LOCK_TIMEOUT	200

#define PHY0_SPEED_REG 0x415
#define PHY1_SPEED_REG 0x418
#define PHY2_SPEED_REG 0x41B
#define PHY3_SPEED_REG 0x41E
#define PHY1_RESET_REG 0x1C00
#define MAXIM_PHYSPEED_MASK		0x1F
#define PHY_SPEED_MAX  2500
#define PHY2_RESET_REG 0x1D00
#define PHY3_RESET_REG 0x1E00
#define PHY4_RESET_REG 0x1F00
#define PHY_RESET_VAL0 0xf4
#define PHY_RESET_VAL1 0xf5
#define MIPI_OUTPUT_EN_REG        0x40b
#define MIPI_OUTPUT_EN                BIT[1]

#define PIPE_ENABLE_REG    0xF4
#define PIPE0_TO_MIPICTL_REG   0x92D
#define PIPE1_TO_MIPICTL_REG   0x96D
#define PIPE2_TO_MIPICTL_REG   0x9AD
#define PIPE3_TO_MIPICTL_REG   0x9ED

#define CSI_NUM			4
#define PIPE_MAP_TO_CSI0	0x55
#define PIPE_MAP_TO_CSI1	0xAA

#define MAX96712_MFP8	8U
#define MAX96712_REG1	0x1U
#define MAX96712_REG1_IIC_EN	0xCCU

#define GPIO_BASE_REG       0x300
#define GPIO_REG_MAX        0x333
#define REG_ADDR_GPIO(i)    ((GPIO_BASE_REG) + ((i)/5 * 0x10) + ((i)%5 * 3))
#define GPIO_INDEX_MAX	    16
#define GPIO_BASE_VAL       0x80
#define GPIO_OUT_DIS        BIT(0)
#define GPIO_TX_GMSL        BIT(1)
#define GPIO_RX_GMSL        BIT(2)
#define GPIO_OUT_HIGH       BIT(4)
#define GPIO_BCD_RX_EN		BIT(5)
#define GPIO_ID_MASK        0x1f
#define GPIO_TO_TRIG        0xeb
#define GPIO_TRIG_VAL0      0xa0
#define GPIO_TRIG_VAL       0xe0
#define LINK_A				0
#define GPIO_ID_OFFSET		4
#define GPIO_TO_DIAG_VAL        GPIO_RX_GMSL
#define MUXED_96712 	        0x041A
#define NOT_INIT_FLAG           2

#define DEC_ERR_CNT_A_96712REG 		0x35
#define DEC_ERR_CNT_B_96712REG		0x36
#define DEC_ERR_CNT_C_96712REG		0x37
#define DEC_ERR_CNT_D_96712REG		0x38
#define IDLE_ERR_CNT_A_96712REG 	0x39
#define IDLE_ERR_CNT_B_96712REG		0x3a
#define IDLE_ERR_CNT_C_96712REG		0x3b
#define IDLE_ERR_CNT_D_96712REG		0x3c
#define LCRC_ERR_CNT_0_96712REG		0x100
#define LCRC_ERR_CNT_1_96712REG		0x112
#define LCRC_ERR_CNT_2_96712REG		0x124
#define LCRC_ERR_CNT_3_96712REG		0x136
#define LCRC_ERR_CNT_4_96712REG		0x148
#define LCRC_ERR_CNT_5_96712REG		0x160
#define LCRC_ERR_CNT_6_96712REG		0x172
#define LCRC_ERR_CNT_7_96712REG		0x184
#define MAX_RT_ERR_CNT_A_96712REG	0x507
#define MAX_RT_ERR_CNT_B_96712REG	0x517
#define	MAX_RT_ERR_CNT_C_96712REG	0x527
#define MAX_RT_ERR_CNT_D_96712REG	0x537
#define VID_PXL_CRC_CNT_AZ_96712REG	0x11d2
#define VID_PXL_CRC_CNT_BZ_96712REG	0x11e3
#define VID_PXL_CRC_CNT_CZ_96712REG	0x11e7
#define VID_PXL_CRC_CNT_DZ_96712REG	0x11eb
#define MEM_ECC_ERR2_CNT_96712REG	0x1250

/* serial errg cfg */
#define	SER_ERRG_EN		0x29
#define SER_ERRG_CGF	0x2a
/* deserial errg cfg */
#define	MAX96712_GMSLA_ERRG_EN	0x1001
#define MAX96712_GMSLA_ERRG_CGF	0x1002
#define	MAX96712_GMSLB_ERRG_EN	0x1011
#define MAX96712_GMSLB_ERRG_CGF	0x1012
#define	MAX96712_GMSLC_ERRG_EN	0x1021
#define MAX96712_GMSLC_ERRG_CGF	0x1022
#define	MAX96712_GMSLD_ERRG_EN	0x1031
#define MAX96712_GMSLD_ERRG_CGF	0x1032

#define MAX96712_MAX_RT_LIMIT0	0x506
#define MAX96712_MAX_RT_LIMIT1	0x516
#define MAX96712_MAX_RT_LIMIT2	0x526
#define MAX96712_MAX_RT_LIMIT3	0x536
#define MAX96712_PYH0_DPLL		0x415
#define MAX96712_PYH1_DPLL		0x418
#define MAX96712_PYH2_DPLL		0x41b
#define MAX96712_PYH3_DPLL		0x41e

#define MEM_ECC_FAULT_INJECT_96712REG	0x1253

/* Max96712 reset*/
uint8_t max96712_reset_seting[]={
	0x04, 0x90, 0x00, 0x13, 0x40, 	// reset all
	0x00, 0x64,
	0x04, 0x90, 0x00, 0x18, 0x0F, 	// One-shot link reset for all
	0x00, 0x64,
};

uint8_t max96712_pipe_map[16][4] = {
	{0, 0, 0, 0},       // 0000
	{0, 1, 2, 3},       // 0001 Link-A
	{1, 0, 2, 3},       // 0010 Link-B
	{0, 1, 2, 3},       // 0011 Link-A+B
	{2, 1, 0, 3},       // 0100 Link-C
	{0, 2, 1, 3},       // 0101 Link-A+C
	{1, 2, 0, 3},       // 0110 Link-B+C
	{0, 1, 2, 3},       // 0111 Link-A+B+C
	{3, 1, 2, 0},       // 1000 Link-D
	{0, 3, 2, 1},       // 1001 Link-A+D
	{1, 3, 2, 0},       // 1010 Link-B+D
	{0, 1, 3, 2},       // 1011 Link-A+B+D
	{2, 3, 0, 1},       // 1100 Link-C+D
	{0, 2, 3, 1},       // 1101 Link-A+C+D
	{1, 2, 3, 0},       // 1110 Link-B+C+D
	{0, 1, 2, 3},       // 1111 Link-A+B+C+D
};

uint32_t max96712_base_setting[] = {
	0x1449, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x1549, 0x75,
	0x1649, 0x75,
	0x1749, 0x75,
	// solve unlock issue by excessive jitter on the GMSL link over a narrow temperature range
	0x06C2, 0x10,

	0x0973, 0x10,   // Un-double 8-bit data on controller 1
	// 0x0100, 0x23, 	// Disable sequence miss and packet detect on pipe 0
	// 0x0112, 0x23, 	// Disable sequence miss and packet detect on pipe 1
	// 0x0124, 0x23, 	// Disable sequence miss and packet detect on pipe 2
	// 0x0136, 0x23, 	// Disable sequence miss and packet detect on pipe 3

	0x040b, 0x00,    // Disable CSI output
	0x0006, 0xf0,    // Disable all link

	0x08A0, 0x04,    // 2x4 mode

	0x08A3, 0xE4,    // Map data lanes for PHY 1
	0x08A4, 0xE4,    // Map data lanes for PHY 0

	0x090A, 0xC0,    // 4 lanes
	0x094A, 0xC0,
	0x098A, 0xC0,
	0x09CA, 0xC0,

#if 0
// Hold DPLL in reset (config_soft_rst_n = 0) before changing the rate
	0x1C00, 0xF4,
	0x1D00, 0xF4,
	0x1E00, 0xF4,
	0x1F00, 0xF4,

// Set Data rate to be 2500Mbps/lane
	0x0415, 0x39,
	0x0418, 0x39,
	0x041B, 0x39,
	0x041E, 0x39,

// Release reset to DPLL (config_soft_rst_n = 1)
	0x1C00, 0xF5,
	0x1D00, 0xF5,
	0x1E00, 0xF5,
	0x1F00, 0xF5,
#endif
};

uint32_t phy_speed_reset_dpll_config[2][8] = {
	{
		0x1C00, 0xF4,
		0x1D00, 0xF4,
		0x1E00, 0xF4,
		0x1F00, 0xF4,
	},
	{
		0x1C00, 0xF5,
		0x1D00, 0xF5,
		0x1E00, 0xF5,
		0x1F00, 0xF5,
	}
};

uint16_t max96712_phy_speed_reg[] = {
	0x0415,
	0x0418,
	0x041B,
	0x041E,
};

uint32_t max96712_raw_pipe_to_mipi_config[4][20] = {
	{
		0x090B, 0x0F,    // Map source 0~2 for Link A
		0x092D, 0x55,    // Map source to controller 1
		0x090D, 0x2C,    // src vc && datatype, vc = 0, RAW12
		0x090E, 0x2C,    // dst vc && datatype, vc = 0, RAW12
		0x090F, 0x00,    // src frame start
		0x0910, 0x00,    // dst frame start
		0x0911, 0x01,    // src frame end
		0x0912, 0x01,    // dst frame end
		0x0913, 0x12,    // src embeded
		0x0914, 0x12,    // dst embeded
	},
	{
		0x094B, 0x0F,    // Map source 0~2 for Link B
		0x096D, 0x55,
		0x094D, 0x2C,
		0x094E, 0x6C,    // vc = 1
		0x094F, 0x00,
		0x0950, 0x40,
		0x0951, 0x01,
		0x0952, 0x41,
		0x0953, 0x12,
		0x0954, 0x52,
	},
	{
		0x098B, 0x0F,    // Map source 0~2 for Link C
		0x09AD, 0x55,
		0x098D, 0x2C,
		0x098E, 0xAC,    // vc = 2
		0x098F, 0x00,
		0x0990, 0x80,
		0x0991, 0x01,
		0x0992, 0x81,
		0x0993, 0x12,
		0x0994, 0x92,
	},
	{
		0x09CB, 0x0F,    // Map source 0~2 for Link D
		0x09ED, 0x55,
		0x09CD, 0x2C,
		0x09CE, 0xEC,    // vc = 3
		0x09CF, 0x00,
		0x09D0, 0xC0,
		0x09D1, 0x01,
		0x09D2, 0xC1,
		0x09D3, 0x12,
		0x09D4, 0xD2,
	},
};

uint32_t max96712_yuv_pipe_to_mipi_config[4][20] = {
	{
		0x090B, 0x0F,    // Map source 0~2 for Link A
		0x092D, 0x55,    // Map source to controller 1
		0x090D, 0x1E,    // src vc && datatype, vc = 0, YUV
		0x090E, 0x1E,    // dst vc && datatype, vc = 0, YUV
		0x090F, 0x00,    // src frame start
		0x0910, 0x00,    // dst frame start
		0x0911, 0x01,    // src frame end
		0x0912, 0x01,    // dst frame end
		0x0913, 0x12,    // src embeded
		0x0914, 0x12,    // dst embeded
	},
	{
		0x094B, 0x0F,    // Map source 0~2 for Link B
		0x096D, 0x55,
		0x094D, 0x1E,
		0x094E, 0x5E,    // vc = 1
		0x094F, 0x00,
		0x0950, 0x40,
		0x0951, 0x01,
		0x0952, 0x41,
		0x0953, 0x12,
		0x0954, 0x52,
	},
	{
		0x098B, 0x0F,    // Map source 0~2 for Link C
		0x09AD, 0x55,
		0x098D, 0x1E,
		0x098E, 0x9E,    // vc = 2
		0x098F, 0x00,
		0x0990, 0x80,
		0x0991, 0x01,
		0x0992, 0x81,
		0x0993, 0x12,
		0x0994, 0x92,
	},
	{
		0x09CB, 0x0F,    // Map source 0~2 for Link D
		0x09ED, 0x55,
		0x09CD, 0x1E,
		0x09CE, 0xDE,    // vc = 3
		0x09CF, 0x00,
		0x09D0, 0xC0,
		0x09D1, 0x01,
		0x09D2, 0xC1,
		0x09D3, 0x12,
		0x09D4, 0x92,
	},
};

uint32_t max96712_phy_copy_init_setting[] = {
	0x08A9, 0x88,
};
uint32_t max96712_stream_on_setting[] = {
	0x040b, 0x02, 0x02    // Enable CSI output
};
uint32_t max96712_stream_off_setting[] = {
	0x040b, 0x00,    // Disable CSI output
};
static uint16_t max96712_mfp_index[GPIO_INDEX_MAX + 1][4] = {
        // MFP0
	{
		0x0301,  // linkA
		0x0337,  // linkB
		0x036D,  // linkC
		0x03A4,  // linkD
	},
        // MFP1
	{
		0x0304,  // linkA
		0x033A,  // linkB
		0x0371,  // linkC
		0x03A7,  // linkD
	},
        // MFP2
	{
		0x0307,  // linkA
		0x033D,  // linkB
		0x0374,  // linkC
		0x03AA,  // linkD
	},
        // MFP3
	{
		0x030A,  // linkA
		0x0341,  // linkB
		0x0377,  // linkC
		0x03AD,  // linkD
	},
        // MFP4
	{
		0x030D,  // linkA
		0x0344,  // linkB
		0x037A,  // linkC
		0x03B1,  // linkD
	},
        // MFP5
	{
		0x0311,  // linkA
		0x0347,  // linkB
		0x037D,  // linkC
		0x03B4,  // linkD
	},
        // MFP6
	{
		0x0314,  // linkA
		0x034A,  // linkB
		0x0381,  // linkC
		0x03B7,  // linkD
	},
        // MFP7
	{
		0x0317,  // linkA
		0x034D,  // linkB
		0x0384,  // linkC
		0x03BA,  // linkD
	},
        // MFP8
	{
		0x031A,  // linkA
		0x0351,  // linkB
		0x0387,  // linkC
		0x03BD,  // linkD
	},
        // MFP9
	{
		0x031D,  // linkA
		0x0354,  // linkB
		0x038A,  // linkC
		0x03C1,  // linkD
	},
        // MFP10
	{
		0x0321,  // linkA
		0x0357,  // linkB
		0x038D,  // linkC
		0x03C4,  // linkD
	},
        // MFP11
	{
		0x0324,  // linkA
		0x035A,  // linkB
		0x0391,  // linkC
		0x03C7,  // linkD
	},
        // MFP12
	{
		0x0327,  // linkA
		0x035D,  // linkB
		0x0394,  // linkC
		0x03CA,  // linkD
	},
        // MFP13
	{
		0x032A,  // linkA
		0x0361,  // linkB
		0x0397,  // linkC
		0x03CD,  // linkD
	},
        // MFP14
	{
		0x032D,  // linkA
		0x0364,  // linkB
		0x039A,  // linkC
		0x03D1,  // linkD
	},
        // MFP15
	{
		0x0331,  // linkA
		0x0367,  // linkB
		0x039D,  // linkC
		0x03D4,  // linkD
	},
        // MFP16
	{
		0x0334,  // linkA
		0x036A,  // linkB
		0x03A1,  // linkC
		0x03D7,  // linkD
	},
};

static uint32_t max96712_datatype_bpp_mux_init_setting[4][18] = {
	{
		0x040E, 0x3F, 0x1E,	 // 0b00(DT1_H)-00111111, bit5-0, Set DT of Pipe0(0x1E)
		0x040B, 0xF8, 0x40,  // 11111000 bit7-3, Set bpp of pipe0(0x08)
		0x041A, 0x10, 0x10,	 // Set pipe0 YUV MUX mode
		0x0415, 0x40, 0x40,	 // Set pipe0
		DELAY_FLAG, 0x00, 0x00,
		DELAY_FLAG, 0x00, 0x00,
	},
	{
		0x040E, 0xC0, 0x40,	 // 0b00(DT1_H)-11000000, bit7-6,Set DT of Pipe1 High bit
		0x040F, 0x0F, 0x0E,	 // 00001111 bit3-0, Set DT of Pipe1 Low bit
		0x0411, 0x1F, 0x08,    // 00011111 bit4-0, Set bpp of pipe1(0x08)
		0x041A, 0x20, 0x20,	 // Set pipe1 YUV MUX mode
		0x0415, 0x80, 0x80,	 // Set pipe1
		DELAY_FLAG, 0x00, 0x00,
	},
	{
	   0x040F, 0xF0, 0x70,	 // 0b00(DT1_H)-11110000 bit7-4, Set DT of Pipe2 High bit
	   0x0410, 0x03, 0x02,	 // 0b00(DT1_H)-00000011, Set DT of Pipe2 Low bit
	   0x0411, 0xE0, 0x40,    // 11100000 bit7-5, Set bpp of pipe2(0x08)
	   0x0412, 0x03, 0x00,    // 00000011, Set bpp of pipe2(0x08)
	   0x041A, 0x40, 0x40,	 // Set pipe2 YUV MUX mode
	   0x0418, 0x40, 0x40,	 // Set pipe2
	},
	{
	   0x0410, 0xF3, 0x78,	 // 0b00(DT1_H)-11111100, Set DT of Pipe3
	   0x0412, 0x7C, 0x20,    // 01111100, Set bpp of pipe3(0x08)
	   0x041A, 0x80, 0x80,	 // Set pipe3 YUV MUX mode
	   0x0418, 0x80, 0x80,	 // Set pipe3
	   DELAY_FLAG, 0x00, 0x00,
	   DELAY_FLAG, 0x00, 0x00,
	},
};

static uint32_t max96712_diag_cfg[] = {
	0x0005, 0xc0,  // ERRB_EN(bit6, default 1)
	0x0029, 0x08,  // INTR6 LCRC_ERR_OEN(bit3), FSYNC_ERR_OEN bit(0)
	0x0025, 0x0f,  // INTR2 DEC_ERR_FLAG_A,B,C,D
	0x002b, 0x0f,  // INTR8 IDLE_ERR_FLAG_A,B,C,D
	0x002d, 0xff,  // INTR10 RT_CNT_FLAG_A,B,C,D  MAX_RT_FLAG_A,B,C,D
	0x0044, 0x80,  // MEM_ECC_ERR2_INT
	0x0506, 0x72,  // enadble linkA CC ARQ, MAX_RT_ERR_OEN_C(b1)
	0x0516, 0x72,  // enadble linkB CC ARQ, MAX_RT_ERR_OEN_C(b1)
	0x0526, 0x72,  // enadble linkC CC ARQ, MAX_RT_ERR_OEN_C(b1)
	0x0536, 0x72,  // enaable linkD CC ARQ, MAX_RT_ERR_OEN_C(b1)
};

enum max96712_subid_reg {
	MAX96712_UNLOCK			= 0,
	MAX96712_DEC_ERR		= 1,
	MAX96712_IDLE_ERR		= 2,
	MAX96712_MAX_RT			= 3,
	MAX96712_VID_PXL_CRC	= 5,
	MAX96712_LMO_ERR		= 6,
	MAX96712_LCRC_ERR		= 9,
	MAX96712_MEM_ECC		= 10,
};

enum max96712_subid_gpio {
	MAX96712_LOCK			= 14,
	MAX96712_ERRB			= 15,
};

enum max96712_dpll_stat {
	MAX96712_DPLL_HOLD,
	MAX96712_DPLL_REL
};

#if 0
/****************************** MAX96712 Config API *******************************/
/*
 * max96712_reset() - Reset max96712
 */
int32_t max96712_reset(deserial_info_t *deserial_if);

/*
 * max96712_gmsl_speed_set() - Set gmsl speed mode for max96712
 * @link_index: the link index that need set gmsl speed
 * @gmsl_val: 1: 3gbps, 2: 6gbps
 */
int32_t max96712_gmsl_speed_set(deserial_info_t *deserial_if,
                                uint16_t link_index, uint8_t gmsl_val);

/*
 * max96712_link_enable() - enable some links,and disable another link
 * @link_mask: need enable link mask,BIT[3:0] - link A~D
 */
int32_t max96712_link_enable(deserial_info_t *deserial_if, uint8_t link_mask);
/*
 * max96712_link_map() - max96712 link-pipe map
 * @vc_arr: vc_arr[0~3] - linkA~D vc_index
 */
int32_t max96712_link_map(deserial_info_t *deserial_if, uint8_t *pipe_arr);

/*
 * max96712_remote_control() - enable some one links, and disable another link
 * @link_index: need enable link, BIT[3:0] - link A~D
 */
int32_t max96712_remote_control(deserial_info_t *deserial_if,
                                               uint8_t link_index);

/*
 * max96712_phy_speed_cfg() - set max96712 mipi lane speed
 * @deserial_if->lane_speed: the mipi frequency, *100Mbps
 * @deserial_if->lane_mode: bit[5]-0: 2*4mode, 1: 4*2mode; bit[6]-need to set mipi csi port
*/
int32_t max96712_phy_speed_cfg(deserial_info_t *deserial_if);

/*
 * max96712_mfp_cfg() - set max96712 mipi lane speed
 * @deserial_if->deserial_gpio: [0]:trig_mfp, [1]: errb, [2]:sensor_err
 * @mfp_mode: trig_mode: GPIO_TO_TRIG, diag_mode: GPIO_TO_DIAG
 * @gpio_id: gpio transmit id
*/
int32_t max96712_mfp_cfg(deserial_info_t *deserial_if, uint8_t gpio_mode, uint8_t gpio_id);
#endif
#endif  // UTILITY_DESERIAL_MAX96712_SETTING_H_
