/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_OV_COMMON_SETTING_H_
#define UTILITY_SENSOR_INC_OV_COMMON_SETTING_H_


#define OV_PARAM_HOLD        (0x3208)
	#define GROUP_0_START      (0x00)
	#define GROUP_0_END        (0x10)
	#define GROUP_0_Q_LUNCH    (0xE0)
#define OV_MANUAL_LUNCH      (0x3211)
	#define MANUAL_LUNCH_ON    (0x00)
#define OV_HCG_AGAIN         (0x3508)
#define OV_SPD_AGAIN         (0x3548)
#define OV_LCG_AGAIN         (0x3588)
#define OV_VS_AGAIN          (0x35C8)
#define OV_HCG_DGAIN         (0x350A)
#define OV_SPD_DGAIN         (0x354A)
#define OV_LCG_DGAIN         (0x358A)
#define OV_VS_DGAIN          (0x35CA)
#define OV_DCG_LINE          (0x3501)
#define OV_SPD_LINE          (0x3541)
#define OV_VS_LINE           (0x35C1)
#define OV_VTS               (0x380E)
#define OV_HTS               (0x386E)
#define OV_HTS_DCG           (0x380C)
#define OV_HTS_S             (0x384C)
#define OV_HTS_VS            (0x388C)
#define OV_X_OUTPUT          (0x3808)
#define OV_Y_OUTPUT          (0x380A)
#define OV_AWB_HCG_B         (0x5280)
#define OV_AWB_HCG_GB        (0x5282)
#define OV_AWB_HCG_GR        (0x5284)
#define OV_AWB_HCG_R         (0x5286)
#define OV_AWB_LCG_B         (0x5480)
#define OV_AWB_LCG_GB        (0x5482)
#define OV_AWB_LCG_GR        (0x5484)
#define OV_AWB_LCG_R         (0x5486)
#define OV_AWB_SPD_B         (0x5680)
#define OV_AWB_SPD_GB        (0x5682)
#define OV_AWB_SPD_GR        (0x5684)
#define OV_AWB_SPD_R         (0x5686)
#define OV_AWB_VS_B          (0x5880)
#define OV_AWB_VS_GB         (0x5882)
#define OV_AWB_VS_GR         (0x5884)
#define OV_AWB_VS_R          (0x5886)

#define OV_PLL2_PREDIV0      (0x0326)
#define OV_PLL2_PREDIV       (0x0323)
#define OV_PLL2_MULT         (0x0324)
#define OV_PLL2_DIVSYSPRE    (0x032A)
#define OV_PLL2_DIVSYS       (0x032B)
#define OV_MIRROR_FLIP       (0x3820)
#define OV_PWL0_1_2F         (0x5E2F)
#define OV_STREAMING         (0x0100)
#define OV_VFIFO_FCNT3       (0x4620)
#define OV_VFIFO_FCNT2       (0x4621)
#define OV_VFIFO_FCNT1       (0x4622)
#define OV_VFIFO_FCNT0       (0x4623)
#define OV_AVER_TEMPER       (0x4D2A)
#define OV_SCCB_CRC          (0x3180)

#define OV_TC_R_INIT_MAIN	(0x3826)
#define OV_SYNC_ROW_CNT_ADJ	(0x3882)

#if 0
// config_index bit[8~11] reserved
#define BIT(i)               (1 << (i))
#define AE_DISABLE           BIT(0)
#define AWB_DISABLE          BIT(1)
#define DUAL_LANE            BIT(2)
#define TEST_PATTERN         BIT(3)
#define CLK_SOURCE           (0x3 << 4)
#define TRIG_SOURCE          BIT(7)
#define TRIG_MODE            (0x3 << 8)
/* x8b/x3c only support TRIG_ARBITRARY */
#define TRIG_CONTINUOUS      BIT(8)
#define TRIG_ARBITRARY       BIT(9)
#define G_BOARD              BIT(10)
#define MAXFA_BOARD          BIT(11)
#define MIRROR               BIT(12)
#define FLIP                 BIT(13)
#define POC_DISABLE          BIT(14)
#define EMBEDDED_MODE        (0xf << 16)
/* add embedded data front 2 rows, resolution add 2 rows*/
#define EMBEDDED_MODE_0      BIT(16)
/* add embedded data front 2 rows, resulution same*/
#define EMBEDDED_MODE_1      BIT(17)
#define EMBEDDED_MODE_2      BIT(18)
#define EMBEDDED_MODE_3      BIT(19)
#define MAX9295_MFP3         BIT(20)
#endif

#define TUNING_LUT
#define SENSOR_REG_WIDTH     REG16_VAL8
#define SERDES_REG_WIDTH     REG16_VAL8
#define POC_REG_WIDTH        REG8_VAL8
#define DEFAULT_POC_ADDR     (0x28)
#define POC_I2C_ADDR_PILOT   (0x2a)
#define CAM_I2C_RETRY_MAX    10
#define FCNT_RETRY_MAX       4
#define FCNT_ERR_RANGE       2

// Sensor Diag Param begin
enum ov_subid {
	CAMERA_OV_ERRB_ERROR = 0,
	CAMERA_OV_AVDD_ERROR,
	CAMERA_OV_DOVDD_ERROR,
	CAMERA_OV_DVDD_ERROR,
	CAMERA_OV_TEMP_ERROR,
	CAMERA_OV_COLUMN_ID_ERROR,
	CAMERA_OV_ROW_ID_ERROR,
	CAMERA_OV_PLL_CLOCK_ERROR,
	CAMERA_OV_RAM_CRC_ERROR,
	CAMERA_OV_ROM_CRC_ERROR,
	CAMERA_OV_ONLINE_PIXEL_ERROR,
};

#define OV_FAULT_2_REG			0x4F09
#define OV_FAULT_1_REG			0x4F0A
#define OV_FAULT_0_REG			0x4F0B

// Voltage Check Info
#define OV_AVDD_VOLTAGE_REG     0x45A0
#define OV_DOVDD_VOLTAGE_REG    0x45A2
#define OV_DVDD_VOLTAGE_REG     0x45A4

#define AVDD_MAX_VOL_VALUE    0x0844  // 3.10v
#define AVDD_MIN_VOL_VALUE    0x0734  // 2.70v
#define DOVDD_MAX_VOL_VALUE   0x0547  // 1.98v
#define DOVDD_MIN_VOL_VALUE   0x0452  // 1.62v
#define DVDD_MAX_VOL_VALUE    0x0317  // 1.16v
#define DVDD_MIN_VOL_VALUE    0x02cd  // 1.05v

// Temperature Info
#define OV_AVER_TEMP_REG		0x4D2A
#define OV_TEMP_RATIO			1000
#define OV_MAX_TEMP_VALUE		125	 // 125
#define OV_MIN_TEMP_VALUE		-40	 // -40

// fault inject reg info
#define COLUMN_ROW_FAULT_REG	0x4640

#define PLL_CLOCK_ALLOW_INJECT	0x0410
#define PLL_CLOCK_FAULT_REG 	0x0408

#define RAM_CRC_ALLOW_INJECT	0x3219
#define RAM_CRC_FAULT_REG		0x322f

#define ROM_CRC_ALLOW_INJECT	0x2000
#define ROM_CRC_FAULT_REG		0x2007

#define ONLINE_PIXEL_ALLOW_INJECT	0x3B9E
#define ONLINE_PIXEL_FAULT_REG 	0x3B9D
// Sensor Diag Param end
typedef struct sensor_pll_data {
	uint32_t sclk;
	float    fps;
} sensor_pll_data_t;

uint32_t poc_init_setting[] = {
	0x01, 0x00,
	0x01, 0x1f,
};

uint32_t ov_test_pattern[] = {
	0x5240, 0x01,
	0x5440, 0x01,
	0x5640, 0x01,
	0x5840, 0x01,
	0x5004, 0x01,
	0x5005, 0x01,
	0x5006, 0x01,
	0x5007, 0x01,
};

uint32_t ov_stream_on_setting[] = {
	0x0100, 0x01,
};

uint32_t ov_stream_off_setting[] = {
	0x0100, 0x00,
};
uint32_t group_hold_start_setting[] = {
	0x3208, 0x00,  // group hold start
};

uint32_t group_hold_end_setting[] = {
	0x3208, 0x10,  // group hold end
	0x3211, 0x40,  // manual lunch on, group hold crc enable
	0x3208, 0xA0   // delay lunch
};

uint32_t awb_reg_array_base[] = {
	0x5280, 0x04,  // AWB_HCG_B_GAIN
	0x5281, 0x00,
	0x5282, 0x04,  // AWB_HCG_Gb_GAIN
	0x5283, 0x00,
	0x5284, 0x04,  // AWB_HCG_Gr_GAIN
	0x5285, 0x00,
	0x5286, 0x04,  // AWB_HCG_R_GAIN
	0x5287, 0x00,

	0x5480, 0x04,  // AWB_LCG_B_GAIN
	0x5481, 0x00,
	0x5482, 0x04,  // AWB_LCG_Gb_GAIN
	0x5483, 0x00,
	0x5484, 0x04,  // AWB_LCG_Gr_GAIN
	0x5485, 0x00,
	0x5486, 0x04,  // AWB_LCG_R_GAIN
	0x5487, 0x00,

	0x5680, 0x04,  // AWB_SPD_B_GAIN
	0x5681, 0x00,
	0x5682, 0x04,  // AWB_SPD_Gb_GAIN
	0x5683, 0x00,
	0x5684, 0x04,  // AWB_SPD_Gr_GAIN
	0x5685, 0x00,
	0x5686, 0x04,  // AWB_SPD_R_GAIN
	0x5687, 0x00,

	0x5880, 0x04,  // AWB_VS_B_GAIN
	0x5881, 0x00,
	0x5882, 0x04,  // AWB_VS_Gb_GAIN
	0x5883, 0x00,
	0x5884, 0x04,  // AWB_VS_Gr_GAIN
	0x5885, 0x00,
	0x5886, 0x04,  // AWB_VS_R_GAIN
	0x5887, 0x00,
};

uint32_t ae_reg_array_base[] = {
	0x3501, 0x01,  // AEC_HCG_EXPO
	0x3502, 0xc8,
	0x3541, 0x01,  // AEC_SPD_EXPO
	0x3542, 0xc8,
	0x35c1, 0x00,  // AEC_VS_EXPO
	0x35c2, 0x01,

	0x3508, 0x01,  // AEC_HCG_REAL_GAIN
	0x3509, 0x00,
	0x3548, 0x04,  // AEC_SPD_REAL_GAIN
	0x3549, 0x40,
	0x3588, 0x01,  // AEC_LCG_REAL_GAIN
	0x3589, 0x00,
	0x35C8, 0x01,  // AEC_VS_REAL_GAIN
	0x35C9, 0x00,

	0x350A, 0x01,  // AEC_HCG_DIGITAL_GAIN
	0x350B, 0x00,
	0x350C, 0x00,
	0x354A, 0x01,  // AEC_SPD_DIGITAL_GAIN
	0x354B, 0x00,
	0x354C, 0x00,
	0x358A, 0x01,  // AEC_LCG_DIGITAL_GAIN
	0x358B, 0x00,
	0x358C, 0x00,
	0x35CA, 0x01,  // AEC_VS_DIGITAL_GAIN
	0x35CB, 0x00,
	0x35CC, 0x00,
};

uint32_t emb_data_front_2rows_setting[] = {
	0x3208, 0x04,  // group 4 group hold start
	0x4620, 0x04,  // fcnt
	0x3501, 0x02,  // dcg_line
	0x3541, 0x02,  // spd_line
	0x35c1, 0x02,  // vs_line
	0x4D2A, 0x02,  // temperatur
	0x3208, 0x14,  // group 4 group hold end
	0x4317, 0x28,  // enable 2 rows of embedded data
	0x430D, 0x13,  // 93->12 13 embed type: default front 2 rows emb data not output
};

uint32_t emb_data_back_2rows_setting[] = {
	0x3208, 0x05,  // group 5 group hold start
	0x4620, 0x04,  // fcnt
	0x3501, 0x02,  // dcg_line
	0x3541, 0x02,  // spd_line
	0x35c1, 0x02,  // vs_line
	0x4D2A, 0x02,  // temperatur
	0x3208, 0x15,  // group 5 group hold end
	0x4317, 0x18,  // enable 2 rows of embedded data
};

typedef struct {
	float gain, again, dgain;
	float exp_value;
	uint16_t line;
} line_gain_control_t;

#define COMPARE_AND_ASSIGN(value, preset1, preset2)	\
	do {		\
		if ((value) <= (preset1))		\
			(value) = (preset1);		\
		else if ((value) >= (preset2))	\
			(value) = (preset2);		\
	} while (0)

#endif  //  UTILITY_SENSOR_INC_OV_COMMON_SETTING_H_
