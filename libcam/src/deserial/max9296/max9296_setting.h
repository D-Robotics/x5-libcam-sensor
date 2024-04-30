/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_DESERIAL_MAX9296_SETTING_H_
#define UTILITY_DESERIAL_MAX9296_SETTING_H_

#define MAXIM_PHYSPEED_MASK		0x1F
#define MIPI_4LANE_MODE			0
#define MIPI_2LANE_MODE			1

#define RESET_REG		0x10
#define RESET_VAL		0xf1
#define GMSL_REG		0x01
#define GMSL_3GBPS		0x1
#define GMSL_6GBPS		0x2
#define GMSL_MASK		0x3
#define LINK_EN_REG		0x10
#define MIPICSIA_SPEED_REG		0x320
#define MIPICSIB_SPEED_REG		0x323
#define AUTO_LINK_MASK		BIT(4)
#define ONESHOTRESET_MASK	BIT(5)

#define LINK_LOCK_REG		0x13
#define LINK_LOCK_MASK		BIT(3)
#define LINK_LOCK_TIMEOUT	200

#define GPIO_INDEX_MAX		12
#define GPIO_BASE_REG		0x2b0
#define REG_ADDR_GPIO(i)    ((GPIO_BASE_REG) + ((i) * 3))
#define MAX9296_REG_ADDR_GPIO(i)	((GPIO_BASE_REG) + ((i) * 3))
#define MAX9296_ERRB_GPIO_RX_ADDR(i)	((GPIO_BASE_REG) + ((i) * 3) + 2)
#define GPIO_BASE_VAL		0x80
#define GPIO_OUT_DIS		BIT(0)
#define GPIO_TX_GMSL		BIT(1)
#define GPIO_RX_GMSL		BIT(2)
#define GPIO_OUT_HIGH		BIT(4)
#define GPIO_ID_MASK		0x1f
#define UART_1_EN		BIT(4)
#define LOCK_EN			BIT(7)

#define GPIO_TO_TRIG_VAL	0xeb
#define GPIO_TO_DIAG_VAL	GPIO_RX_GMSL

#define CSI_OUTPUT_EN_REG	0x313
#define CSI_NUM			2
#define PIPE_MAP_TO_CSI0	0x55
#define PIPE_MAP_TO_CSI1	0xAA
#define PIPE_VC0_SETTING_IDX	7
#define PIPE_VC1_SETTING_IDX	9
#define GPIO_ID_OFFSET  4
/* deserial fault count reg */
#define DEC_ERR_CNT_A_9296REG 		0x22
#define DEC_ERR_CNT_B_9296REG		0x23
#define IDLE_ERR_CNT_9296REG		0x24
#define LCRC_ERR_CNT_0_9296REG		0x100
#define LCRC_ERR_CNT_1_9296REG		0x112
#define LCRC_ERR_CNT_2_9296REG		0x124
#define LCRC_ERR_CNT_3_9296REG		0x136
#define MAX_RT_ERR_CNT_9296REG		0x77
#define VID_PXL_CRC_CNT_X_9296REG	0x55c
#define VID_PXL_CRC_CNT_Y_9296REG	0x55d
#define VID_PXL_CRC_CNT_Z_9296REG	0x55e
#define VID_PXL_CRC_CNT_U_9296REG	0x55f

/* serial errg cfg */
#define	SER_ERRG_EN		0x29
#define SER_ERRG_CGF	0x2a
/* deserial errg cfg */
#define	MAX9296_ERRG_EN	0x29
#define MAX9296_ERRG_CGF	0x2a

#define MAX9296_MAX_RT_LIMIT	0x76
#define MAX9296_PYH0_DPLL		0x31d
#define MAX9296_PYH1_DPLL		0x320
#define MAX9296_PYH2_DPLL		0x323
#define MAX9296_PYH3_DPLL		0x326

/* Max9296 reset */
uint8_t max9296_reset_setting[] = {
	0x04, 0x90, 0x00, 0x10, 0xf0,
	0x00, 0x64,
};

uint32_t max9296_errchpwrup_en[] = {
	0x1449, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x1549, 0x75,
};
uint32_t max9296_raw_single_link_config[] = {
	0x0313, 0x00,     // CSI output disabled
	0x0330, 0x04,     // 2x4 mode
	0x0319, 0x0c,     // Pipeline Y datatype 0x2c
	0x0320, 0x2c,     // MIPI CSI Port A 1200M
	0x0325, 0x80,     // Wait for a new frame
	0x0313, 0x02,     // MIPI output enable
};
uint32_t max9296_yuv_single_link_config[] = {
	0x1449, 0x75,  // Enable ErrChPwrUp, Enhance link stability
	0x1549, 0x75,
	0x0313, 0x00,     // CSI output disabled
	0x0330, 0x04,     // 2x4 mode
	0x0319, 0x10,     // Pipeline Y datatype 0x1E
	0x0320, 0x2c,     // MIPI CSI Port A 1200M
	0x0325, 0x80,     // Wait for a new frame
};

uint32_t max9296_raw_pipe_setting[2][20] = {
	{
		// Pipe y map vc0 to vc0
		0x044B, 0x0F,   // Enable 3 Mappings
		0x046D, 0x55,   // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
		// For the following MSB 2 bits = VC, LSB 6 bits =DT
		0x044D, 0x2C,   // SRC  0b00101100, DT = 0x2C VC=0
		0x044E, 0x2C,   // DEST 0b00101100, DT = 0x2C VC=0
		0x044F, 0x00,   // SRC  DT = Frame Start
		0x0450, 0x00,   // DEST DT = Frame Start
		0x0451, 0x01,   // SRC  DT = Frame End
		0x0452, 0x01,   // DEST DT = Frame End
		0x0453, 0x12,   // SRC  DT = Emb
		0x0454, 0x12,   // DEST DT = Emb
	},
	{
		// Pipe z map vc0 to vc1.
		0x048B, 0x0F,     // Enable 3 Mappings
		0x04AD, 0x55,     // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
		// For the following MSB 2 bits = VC, LSB 6 bits = DT
		0x048D, 0x2C,     // SRC  0b00101100, DT = 0x2C VC=0
		0x048E, 0x6C,     // DEST 0b01101100, DT = 0x2C VC=1
		0x048F, 0x00,     // SRC  DT = Frame Start
		0x0490, 0x40,     // DEST DT = Frame Start
		0x0491, 0x01,     // SRC  DT = Frame End
		0x0492, 0x41,     // DEST DT = Frame End
		0x0493, 0x12,     // SRC  DT = Emb
		0x0494, 0x52,     // DEST DT = Emb
	},
};

uint32_t max9296_raw_pipe_vc1_setting[2][22] = {
	{
		// Pipe y map vc 1
		0x044B, 0xFF,   // Enable 3 Mappings
		0x046E, 0x55,   // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
		0x030B, 0x03,   // Enable vc0~1 override pack
		// For the following MSB 2 bits = VC, LSB 6 bits =DT
		0x0455, 0x6C,   // SRC  0b00101100, DT = 0x2C VC=0
		0x0456, 0x6C,   // DEST 0b00101100, DT = 0x2C VC=0
		0x0457, 0x40,   // SRC  DT = Frame Start
		0x0458, 0x40,   // DEST DT = Frame Start
		0x0459, 0x41,   // SRC  DT = Frame End
		0x045a, 0x41,   // DEST DT = Frame End
		0x045b, 0x52,   // SRC  DT = Emb
		0x045c, 0x52,   // DEST DT = Emb
	},
	{
		// Pipe z map vc1.
		0x048B, 0xFF,     // Enable 3 Mappings
		0x04AE, 0x55,     // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
		0x030D, 0x03,     // Enable vc0~1 override pack
		// For the following MSB 2 bits = VC, LSB 6 bits = DT
		0x0495, 0x6C,     // SRC  0b00101100, DT = 0x2C VC=0
		0x0496, 0x6C,     // DEST 0b01101100, DT = 0x2C VC=1
		0x0497, 0x40,     // SRC  DT = Frame Start
		0x0498, 0x40,     // DEST DT = Frame Start
		0x0499, 0x41,     // SRC  DT = Frame End
		0x049a, 0x41,     // DEST DT = Frame End
		0x049b, 0x52,   // SRC  DT = Emb
		0x049c, 0x52,   // DEST DT = Emb
	},
};

uint32_t max9296_yuv_pipe_setting[2][20] = {
	{
		// Pipe y map to vc0
		0x044B, 0x0F,   // Enable 3 Mappings
		0x046D, 0x55,   // Destionation Controller = Controller 1. Controller 1 sends data to MIPI Port A
		// For the following MSB 2 bits = VC, LSB 6 bits =DT
		0x044D, 0x1E,   // SRC  0b00101100, DT = 0x2C VC=0
		0x044E, 0x1E,   // DEST 0b00101100, DT = 0x2C VC=0
		0x044F, 0x00,   // SRC  DT = Frame Start
		0x0450, 0x00,   // DEST DT = Frame Start
		0x0451, 0x01,   // SRC  DT = Frame End
		0x0452, 0x01,   // DEST DT = Frame End
		0x0453, 0x12,   // SRC  DT = Emb
		0x0454, 0x12,   // DEST DT = Emb
	},
	{
		// Pipe z map to vc1.
		0x048B, 0x0F,     // Enable 3 Mappings
		0x04AD, 0x55,     // Destionation Controller = Controller 1. Controller 2 sends data to MIPI Port A
		// For the following MSB 2 bits = VC, LSB 6 bits = DT
		0x048D, 0x1E,     // SRC  0b00101100, DT = 0x2C VC=0
		0x048E, 0x5E,     // DEST 0b01101100, DT = 0x2C VC=1
		0x048F, 0x00,     // SRC  DT = Frame Start
		0x0490, 0x40,     // DEST DT = Frame Start
		0x0491, 0x01,     // SRC  DT = Frame End
		0x0492, 0x41,     // DEST DT = Frame End
		0x0493, 0x12,   // SRC  DT = Emb
		0x0494, 0x52,   // DEST DT = Emb
	},
};

uint32_t max9296_stream_on_setting[] = {
	// MAX9296 - Deserializer config
	0x1449, 0x04, 0x04,  // Enable ErrChPwrUp, Enhance link stability
	0x1549, 0x04, 0x04,
	0x0313, 0x02, 0x00,
	0x0473, 0x10, 0x10,
	0x0325, 0x80, 0x80,
	0x0313, 0x02, 0x02,     // MIPI output enable
};
uint32_t max9296_stream_off_setting[] = {
	// MAX9296 - Deserializer config
	0x0313, 0x00,
};

uint8_t max9296_add_max96718_init_setting[] = {
	0x04, 0x90, 0x01, 0x61, 0x09,
};

uint8_t max9296_phy_portb_init_setting[] = {
	0x04, 0x90, 0x00, 0x51, 0x81,
	0x04, 0x90, 0x00, 0x52, 0x82,
};

uint8_t max9296_phy_portall_init_setting[] = {
	0x04, 0x90, 0x00, 0x51, 0x81,
	0x04, 0x90, 0x00, 0x52, 0x81,
};
uint32_t max9296_phy_copy_init_setting[] = {
	0x0339, 0x80,
	0x033a, 0x42,
};

static uint32_t max9296_diag_cfg[] = {
	0x0005, 0xc0,  // ERRB_EN(bit6, default 1), PU_LF3, PU_LF2
	0x001a, 0x0f,  // LFLT_INT(bit3), IDLE_ERR(bit2),DEC_ERR_B,A(bit1,0)
	0x001c, 0x08,  // MAX_RT(bit3), WM_ERR(bit0)
	0x001e, 0xe9,  // VDDCMP_INT(bit7),PORZ_INT(bit6),VDDBAD_INT(bit5),
				   // LCRC_ERR(bit3), VID_PXL_CRC_ERR(bit0)
};

enum max9296_subid_reg {
	MAX9296_UNLOCK			= 0,
	MAX9296_DEC_ERR			= 1,
	MAX9296_MAX_RT			= 3,
	MAX9296_VID_PXL_CRC		= 5,
	MAX9296_LMO_ERR			= 6,
	MAX9296_LCRC_ERR		= 9,
	MAX9296_MEM_ECC			= 10,
	MAX9296_LEFT_INT		= 11,
	MAX9296_VDDBAD_INT		= 12,
	MAX9296_PORZ_INT		= 13,
	MAX9296_VDDCMP_INT		= 14,
};

enum max9296_subid_gpio {
	MAX9296_LOCK			= 14,
	MAX9296_ERRB			= 15,
};

uint32_t max9296_datatype_bpp_mux_init_setting[2][18] = {
	{
		0x0316, 0xC0, 0x40,          // 11000000 H_bit6-7 pipeY datatype
	    0x0317, 0x0F, 0x0E,	         // 00001111 L_bit0-3  pipeY datatype
		0x0319, 0x1F, 0x08,          // 00011111 L_bit4-0, BPP Y
		0x031d, 0x80, 0x80,          // 10000000 H_bit7 override Y
		0x0322, 0x20, 0x20,          // 01010000 bit5
		DELAY_FLAG, 0x00, 0x00,
	},
	{
	 	0x0317, 0xF0, 0x70,           // 11110000 H_bit7-4 PIPEZ datatype
	 	0x0318, 0x03, 0x02,           // 00000011 L_bit2-1 PIPEZ datatype
		0x0319, 0xE0, 0x40,           // 11100000 H_bit7-5, BPP Z
		0x031a, 0x03, 0x00,           // 00000011 L_bit2-1 BPP Z
		0x0320, 0x40, 0x40,           // 01000000 bit6 override Z
		0x0322, 0x40, 0x40,           // 01000000 bit6
	},
};
/****************************** MAX9296 Config API *******************************/
/*
 * max9296_reset() - Reset max9296
 */
int32_t max9296_reset(deserial_info_t *deserial_if);

/*
 * max9296_gmsl_speed_set() - Set gmsl speed mode for max9296
 * @gmsl_val: 1 means 3gbps, 2means 6gbps
 */
int32_t max9296_gmsl_speed_set(deserial_info_t *deserial_if,
                               uint16_t link_index, uint8_t gmsl_val);

/*
 * max9296_link_enable() - Enable the link specified by the link_index
 * @link_index: Link that needs to be enabled
 */
int32_t max9296_link_enable(deserial_info_t *deserial_if, uint8_t link_index);

#endif  // UTILITY_DESERIAL_MAX9296_SETTING_H_
