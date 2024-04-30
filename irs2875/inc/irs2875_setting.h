/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2024 D-Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_IRS2875_SETTING_H_
#define UTILITY_SENSOR_INC_IRS2875_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_array)	(sizeof(_array) / sizeof(_array[0]))
#endif

struct regval {
	uint32_t addr;
	uint32_t val;
};
#define REG_NULL 0xFFFF

static const struct regval irs2875_208x1413_5_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1A13}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = Input)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0x9000, 0x12D3}, 	// S00_EXPOTIME (128us @ 37.65 MHz)
	{0x9002, 0x00A6}, 	// S00_SENSORCFG (sensor_code_index = 3)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x560F}, 	// S01_EXPOTIME (1000us @ 45.18 MHz)
	{0x9006, 0x0004}, 	// S01_SENSORCFG (sensor_code_index = 2)
	{0x9007, 0x0000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9008, 0x560F}, 	// S02_EXPOTIME (1000us @ 45.18 MHz)
	{0x900A, 0x0004}, 	// S02_SENSORCFG (sensor_code_index = 2)
	{0x900B, 0x0003}, 	// S02_ROCFG (sensor_rotates = 3)
	{0x900C, 0x560F}, 	// S03_EXPOTIME (1000us @ 45.18 MHz)
	{0x900E, 0x0004}, 	// S03_SENSORCFG (sensor_code_index = 2)
	{0x900F, 0x0006}, 	// S03_ROCFG (sensor_rotates = 6)
	{0x9010, 0x560F}, 	// S04_EXPOTIME (1000us @ 45.18 MHz)
	{0x9012, 0x0004}, 	// S04_SENSORCFG (sensor_code_index = 2)
	{0x9013, 0x0009}, 	// S04_ROCFG (sensor_rotates = 9)
	{0x9014, 0x5262}, 	// S05_EXPOTIME (1000us @ 37.65 MHz)
	{0x9016, 0x0006}, 	// S05_SENSORCFG (sensor_code_index = 3)
	{0x9017, 0x0000}, 	// S05_ROCFG (sensor_rotates = 0)
	{0x9018, 0x5262}, 	// S06_EXPOTIME (1000us @ 37.65 MHz)
	{0x901A, 0x0006}, 	// S06_SENSORCFG (sensor_code_index = 3)
	{0x901B, 0x0003}, 	// S06_ROCFG (sensor_rotates = 3)
	{0x901C, 0x5262}, 	// S07_EXPOTIME (1000us @ 37.65 MHz)
	{0x901E, 0x0006}, 	// S07_SENSORCFG (sensor_code_index = 3)
	{0x901F, 0x0006}, 	// S07_ROCFG (sensor_rotates = 6)
	{0x9020, 0x5262}, 	// S08_EXPOTIME (1000us @ 37.65 MHz)
	{0x9022, 0x0006}, 	// S08_SENSORCFG (sensor_code_index = 3)
	{0x9023, 0xC009}, 	// S08_ROCFG (sensor_rotates = 9)
	{0x9100, 0x12D3}, 	// S00_EXPOTIMEMAX (128us @ 37.65 MHz)
	{0x9102, 0x30A6}, 	// S00_ILLUCFG  Goff (pll_index = 3, illu_code_index = 3)
	{0x9103, 0x606D}, 	// S01_EXPOTIMEMAX (1470us @ 45.18 MHz)
	{0x9105, 0x6104}, 	// S01_ILLUCFG  0 deg (pll_index = 2, illu_code_index = 2)
	{0x9106, 0x606D}, 	// S02_EXPOTIMEMAX (1470us @ 45.18 MHz)
	{0x9108, 0x6104}, 	// S02_ILLUCFG  90 deg (pll_index = 2, illu_code_index = 2)
	{0x9109, 0x606D}, 	// S03_EXPOTIMEMAX (1470us @ 45.18 MHz)
	{0x910B, 0x6104}, 	// S03_ILLUCFG  180 deg (pll_index = 2, illu_code_index = 2)
	{0x910C, 0x606D}, 	// S04_EXPOTIMEMAX (1470us @ 45.18 MHz)
	{0x910E, 0x6104}, 	// S04_ILLUCFG  270 deg (pll_index = 2, illu_code_index = 2)
	{0x910F, 0x5B06}, 	// S05_EXPOTIMEMAX (1470us @ 37.65 MHz)
	{0x9111, 0x7106}, 	// S05_ILLUCFG  0 deg (pll_index = 3, illu_code_index = 3)
	{0x9112, 0x5B06}, 	// S06_EXPOTIMEMAX (1470us @ 37.65 MHz)
	{0x9114, 0x7106}, 	// S06_ILLUCFG  90 deg (pll_index = 3, illu_code_index = 3)
	{0x9115, 0x5B06}, 	// S07_EXPOTIMEMAX (1470us @ 37.65 MHz)
	{0x9117, 0x7106}, 	// S07_ILLUCFG  180 deg (pll_index = 3, illu_code_index = 3)
	{0x9118, 0x5B06}, 	// S08_EXPOTIMEMAX (1470us @ 37.65 MHz)
	{0x911A, 0x7106}, 	// S08_ILLUCFG  270 deg (pll_index = 3, illu_code_index = 3)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0042}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91C4, 0x0000}, 	// USECASEID
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x8810}, 	// PSOUT (IRS9102C output)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9102C output)
	{0x91DF, 0x0109}, 	// MB_CFG0 (MB0 = 9, LVDS0 active)
	{0x91E7, 0x249E}, 	// MB0_FRAMETIME_MIN (mb_framerate = 5.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9216, 0x1EA1}, 	// PLL_MODLUT2_CFG0 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9217, 0xE148}, 	// PLL_MODLUT2_CFG1 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9218, 0x0002}, 	// PLL_MODLUT2_CFG2 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9219, 0x0701}, 	// PLL_MODLUT2_CFG3 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921A, 0x0000}, 	// PLL_MODLUT2_CFG4 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921B, 0x0272}, 	// PLL_MODLUT2_CFG5 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921C, 0x0044}, 	// PLL_MODLUT2_CFG6 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921D, 0x0038}, 	// PLL_MODLUT2_CFG7 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921E, 0x16A1}, 	// PLL_MODLUT3_CFG0 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x921F, 0xCCCD}, 	// PLL_MODLUT3_CFG1 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9220, 0x0004}, 	// PLL_MODLUT3_CFG2 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9221, 0x2601}, 	// PLL_MODLUT3_CFG3 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9222, 0x6370}, 	// PLL_MODLUT3_CFG4 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9223, 0x0453}, 	// PLL_MODLUT3_CFG5 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9224, 0x0073}, 	// PLL_MODLUT3_CFG6 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9225, 0x0029}, 	// PLL_MODLUT3_CFG7 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x924E, 0x000B}, 	// SENSOR_LENGTH_CODE2 (codelength = 12, dc = 50.00 %)
	{0x924F, 0x0FC0}, 	// SENSOR_CODE2_0      (codelength = 12, dc = 50.00 %)
	{0x9257, 0x000B}, 	// SENSOR_LENGTH_CODE3 (codelength = 12, dc = 50.00 %)
	{0x9258, 0x0FC0}, 	// SENSOR_CODE3_0      (codelength = 12, dc = 50.00 %)
	{0x9272, 0x000B}, 	// ILLU_LENGTH_CODE2 (codelength = 12, dc = 37.50 %)
	{0x9273, 0x0F80}, 	// ILLU_CODE2_0      (codelength = 12, dc = 37.50 %)
	{0x927B, 0x000B}, 	// ILLU_LENGTH_CODE3 (codelength = 12, dc = 37.50 %)
	{0x927C, 0x0F80}, 	// ILLU_CODE3_0      (codelength = 12, dc = 37.50 %)
	{0x928E, 0x0A01}, 	// DIGCLKDIV_S2_PLL2 (f_fsm = 45.18 MHz)
	{0x9293, 0x0A01}, 	// DIGCLKDIV_S3_PLL3 (f_fsm = 37.65 MHz)
	{0x929E, 0x0A00}, 	// CHARGEPUMPDIV_S2_PLL2 (f_chargepump = 90.36 MHz)
	{0x92A3, 0x0A00}, 	// CHARGEPUMPDIV_S3_PLL3 (f_chargepump = 75.30 MHz)
	{0x92AE, 0x0B02}, 	// DLLREGDELAY_S2_PLL2
	{0x92B3, 0x0B05}, 	// DLLREGDELAY_S3_PLL3
	{0x92F3, 0x023C}, 	// EYE_FILT_FS6_11
	{0x92F4, 0x0246}, 	// EYE_FILT_FS6_10
	{0x92F5, 0x0246}, 	// EYE_FILT_FS6_9
	{0x92F6, 0x023C}, 	// EYE_FILT_FS6_8
	{0x92F7, 0x0246}, 	// EYE_FILT_FS6_7
	{0x92F8, 0x023C}, 	// EYE_FILT_FS6_6
	{0x92F9, 0x0246}, 	// EYE_FILT_FS6_5
	{0x92FA, 0x023C}, 	// EYE_FILT_FS6_4
	{0x92FB, 0x0246}, 	// EYE_FILT_FS6_3
	{0x92FC, 0x023C}, 	// EYE_FILT_FS6_2
	{0x92FD, 0x0246}, 	// EYE_FILT_FS6_1
	{0x92FE, 0x0246}, 	// EYE_FILT_FS6_0
	{0x92FF, 0x0128}, 	// EYE_FILT_FS5_7
	{0x9300, 0x0128}, 	// EYE_FILT_FS5_6
	{0x9301, 0x0100}, 	// EYE_FILT_FS5_5
	{0x9302, 0x0128}, 	// EYE_FILT_FS5_4
	{0x9303, 0x0128}, 	// EYE_FILT_FS5_3
	{0x9304, 0x0128}, 	// EYE_FILT_FS5_2
	{0x9305, 0x0128}, 	// EYE_FILT_FS5_1
	{0x9306, 0x0100}, 	// EYE_FILT_FS5_0
	{0x9307, 0x0080}, 	// EYE_FILT_FS4_7
	{0x9308, 0x0080}, 	// EYE_FILT_FS4_6
	{0x9309, 0x0080}, 	// EYE_FILT_FS4_5
	{0x930A, 0x0080}, 	// EYE_FILT_FS4_4
	{0x930B, 0x0080}, 	// EYE_FILT_FS4_3
	{0x930C, 0x0080}, 	// EYE_FILT_FS4_2
	{0x930D, 0x0080}, 	// EYE_FILT_FS4_1
	{0x930E, 0x0080}, 	// EYE_FILT_FS4_0
	{0x930F, 0x0040}, 	// EYE_FILT_FS3_7
	{0x9310, 0x0040}, 	// EYE_FILT_FS3_6
	{0x9311, 0x0040}, 	// EYE_FILT_FS3_5
	{0x9312, 0x0040}, 	// EYE_FILT_FS3_4
	{0x9313, 0x0040}, 	// EYE_FILT_FS3_3
	{0x9314, 0x0040}, 	// EYE_FILT_FS3_2
	{0x9315, 0x0040}, 	// EYE_FILT_FS3_1
	{0x9316, 0x0040}, 	// EYE_FILT_FS3_0
	{0x9317, 0x0040}, 	// EYE_FILT_FS2_7
	{0x9318, 0x0040}, 	// EYE_FILT_FS2_6
	{0x9319, 0x0040}, 	// EYE_FILT_FS2_5
	{0x931A, 0x0040}, 	// EYE_FILT_FS2_4
	{0x931B, 0x0040}, 	// EYE_FILT_FS2_3
	{0x931C, 0x0040}, 	// EYE_FILT_FS2_2
	{0x931D, 0x0040}, 	// EYE_FILT_FS2_1
	{0x931E, 0x0040}, 	// EYE_FILT_FS2_0
	{0x931F, 0x0040}, 	// EYE_FILT_FS1_7
	{0x9320, 0x0040}, 	// EYE_FILT_FS1_6
	{0x9321, 0x0040}, 	// EYE_FILT_FS1_5
	{0x9322, 0x0040}, 	// EYE_FILT_FS1_4
	{0x9323, 0x0040}, 	// EYE_FILT_FS1_3
	{0x9324, 0x0040}, 	// EYE_FILT_FS1_2
	{0x9325, 0x0040}, 	// EYE_FILT_FS1_1
	{0x9326, 0x0040}, 	// EYE_FILT_FS1_0
	{0x9327, 0x0040}, 	// EYE_FILT_FS0_7
	{0x9328, 0x0040}, 	// EYE_FILT_FS0_6
	{0x9329, 0x0040}, 	// EYE_FILT_FS0_5
	{0x932A, 0x0040}, 	// EYE_FILT_FS0_4
	{0x932B, 0x0040}, 	// EYE_FILT_FS0_3
	{0x932C, 0x0040}, 	// EYE_FILT_FS0_2
	{0x932D, 0x0040}, 	// EYE_FILT_FS0_1
	{0x932E, 0x0040}, 	// EYE_FILT_FS0_0
	{0x932F, 0x1B0D}, 	// EYE_THRES_FS0_CFG0 (Filter Stage 0, TH1 | TH0)
	{0x9330, 0x3729}, 	// EYE_THRES_FS0_CFG1 (Filter Stage 0, TH3 | TH2)
	{0x9331, 0x5345}, 	// EYE_THRES_FS0_CFG2 (Filter Stage 0, TH5 | TH4)
	{0x9332, 0x6E60}, 	// EYE_THRES_FS0_CFG3 (Filter Stage 0, TH7 | TH6)
	{0x9333, 0x291B}, 	// EYE_THRES_FS1_CFG0 (Filter Stage 1, TH1 | TH0)
	{0x9334, 0x4437}, 	// EYE_THRES_FS1_CFG1 (Filter Stage 1, TH3 | TH2)
	{0x9335, 0x594F}, 	// EYE_THRES_FS1_CFG2 (Filter Stage 1, TH5 | TH4)
	{0x9336, 0x0062}, 	// EYE_THRES_FS1_CFG3 (Filter Stage 1,     | TH6)
	{0x9337, 0x7354}, 	// EYE_THRES_FS2_CFG0 (Filter Stage 2, TH1 | TH0)
	{0x9338, 0xAB90}, 	// EYE_THRES_FS2_CFG1 (Filter Stage 2, TH3 | TH2)
	{0x9339, 0xDFC5}, 	// EYE_THRES_FS2_CFG2 (Filter Stage 2, TH5 | TH4)
	{0x933A, 0x00F7}, 	// EYE_THRES_FS2_CFG3 (Filter Stage 2,     | TH6)
	{0x933B, 0x4935}, 	// EYE_THRES_FS3_CFG0 (Filter Stage 3, TH1 | TH0)
	{0x933C, 0x6E5C}, 	// EYE_THRES_FS3_CFG1 (Filter Stage 3, TH3 | TH2)
	{0x933D, 0x907F}, 	// EYE_THRES_FS3_CFG2 (Filter Stage 3, TH5 | TH4)
	{0x933E, 0x00A0}, 	// EYE_THRES_FS3_CFG3 (Filter Stage 3,     | TH6)
	{0x933F, 0x6145}, 	// EYE_THRES_FS4_CFG0 (Filter Stage 4, TH1 | TH0)
	{0x9340, 0x937A}, 	// EYE_THRES_FS4_CFG1 (Filter Stage 4, TH3 | TH2)
	{0x9341, 0xC2AB}, 	// EYE_THRES_FS4_CFG2 (Filter Stage 4, TH5 | TH4)
	{0x9342, 0x00D8}, 	// EYE_THRES_FS4_CFG3 (Filter Stage 4,     | TH6)
	{0x9343, 0x2118}, 	// EYE_THRES_FS5_CFG0 (Filter Stage 5, TH1 | TH0)
	{0x9344, 0x332A}, 	// EYE_THRES_FS5_CFG1 (Filter Stage 5, TH3 | TH2)
	{0x9345, 0x453C}, 	// EYE_THRES_FS5_CFG2 (Filter Stage 5, TH5 | TH4)
	{0x9346, 0x004D}, 	// EYE_THRES_FS5_CFG3 (Filter Stage 5,     | TH6)
	{0x9347, 0x1811}, 	// EYE_THRES_FS6_CFG0 (Filter Stage 6, TH1 | TH0)
	{0x9348, 0x2620}, 	// EYE_THRES_FS6_CFG1 (Filter Stage 6, TH3 | TH2)
	{0x9349, 0x342D}, 	// EYE_THRES_FS6_CFG2 (Filter Stage 6, TH5 | TH4)
	{0x934A, 0x423A}, 	// EYE_THRES_FS6_CFG3 (Filter Stage 6, TH7 | TH6)
	{0x934B, 0x5049}, 	// EYE_THRES_FS6_CFG4 (Filter Stage 6, TH9 | TH8)
	{0x934C, 0x0058}, 	// EYE_THRES_FS6_CFG5 (Filter Stage 6,     | TH10)
	{0x934D, 0x0150}, 	// EYE_CMPMUX
	{0x934E, 0x0122}, 	// EYE_THRES_H_H (2.530 W)
	{0x934F, 0x0099}, 	// EYE_THRES_H_L (1.000 W)
	{0x9350, 0x0075}, 	// EYE_THRES_L_H (0.592 W)
	{0x9351, 0x0029}, 	// EYE_THRES_L_L (-0.250 W)
	{0x9352, 0x0006}, 	// EYE_CFG (11 plausibility samples, icm_idle enabled)
	{0x9353, 0x0235}, 	// EYE_IDLEFRAMETIME (180.80 ms)
	{0x9354, 0x0300}, 	// MODULEID0
	{0x9355, 0x0000}, 	// MODULEID1
	{0x9356, 0x0000}, 	// MODULEID2
	{0x9357, 0x0000}, 	// MODULEID3
	{0x9358, 0x0000}, 	// MODULEID4
	{0x9359, 0x0000}, 	// MODULEID5
	{0x935A, 0x0000}, 	// MODULEID6
	{0x935B, 0x010E}, 	// MODULEID7
	{0x935C, 0x8ED0}, 	// CRC
	{0x935D, 0x00A7}, 	// ROIROWMAX (156 rows)
	{0x935E, 0x000C}, 	// ROIROWMIN (156 rows)
	{0x935F, 0x00DF}, 	// ROICOLMAX (208 columns)
	{0x9360, 0x0010}, 	// ROICOLMIN (208 columns)
	{0x9362, 0x249E}, 	// MB0_FRAMETIME (mb_framerate = 5.0 fps)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

static const struct regval irs2875_208x1413_10_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1A13}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = Input)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0x9000, 0x12D3}, 	// S00_EXPOTIME (128us @ 37.65 MHz)
	{0x9002, 0x00A6}, 	// S00_SENSORCFG (sensor_code_index = 3)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x560F}, 	// S01_EXPOTIME (1000us @ 45.18 MHz)
	{0x9006, 0x0004}, 	// S01_SENSORCFG (sensor_code_index = 2)
	{0x9007, 0x0000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9008, 0x560F}, 	// S02_EXPOTIME (1000us @ 45.18 MHz)
	{0x900A, 0x0004}, 	// S02_SENSORCFG (sensor_code_index = 2)
	{0x900B, 0x0003}, 	// S02_ROCFG (sensor_rotates = 3)
	{0x900C, 0x560F}, 	// S03_EXPOTIME (1000us @ 45.18 MHz)
	{0x900E, 0x0004}, 	// S03_SENSORCFG (sensor_code_index = 2)
	{0x900F, 0x0006}, 	// S03_ROCFG (sensor_rotates = 6)
	{0x9010, 0x560F}, 	// S04_EXPOTIME (1000us @ 45.18 MHz)
	{0x9012, 0x0004}, 	// S04_SENSORCFG (sensor_code_index = 2)
	{0x9013, 0x0009}, 	// S04_ROCFG (sensor_rotates = 9)
	{0x9014, 0x5262}, 	// S05_EXPOTIME (1000us @ 37.65 MHz)
	{0x9016, 0x0006}, 	// S05_SENSORCFG (sensor_code_index = 3)
	{0x9017, 0x0000}, 	// S05_ROCFG (sensor_rotates = 0)
	{0x9018, 0x5262}, 	// S06_EXPOTIME (1000us @ 37.65 MHz)
	{0x901A, 0x0006}, 	// S06_SENSORCFG (sensor_code_index = 3)
	{0x901B, 0x0003}, 	// S06_ROCFG (sensor_rotates = 3)
	{0x901C, 0x5262}, 	// S07_EXPOTIME (1000us @ 37.65 MHz)
	{0x901E, 0x0006}, 	// S07_SENSORCFG (sensor_code_index = 3)
	{0x901F, 0x0006}, 	// S07_ROCFG (sensor_rotates = 6)
	{0x9020, 0x5262}, 	// S08_EXPOTIME (1000us @ 37.65 MHz)
	{0x9022, 0x0006}, 	// S08_SENSORCFG (sensor_code_index = 3)
	{0x9023, 0xC009}, 	// S08_ROCFG (sensor_rotates = 9)
	{0x9100, 0x12D3}, 	// S00_EXPOTIMEMAX (128us @ 37.65 MHz)
	{0x9102, 0x30A6}, 	// S00_ILLUCFG  Goff (pll_index = 3, illu_code_index = 3)
	{0x9103, 0x5D57}, 	// S01_EXPOTIMEMAX (1330us @ 45.18 MHz)
	{0x9105, 0x6104}, 	// S01_ILLUCFG  0 deg (pll_index = 2, illu_code_index = 2)
	{0x9106, 0x5D57}, 	// S02_EXPOTIMEMAX (1330us @ 45.18 MHz)
	{0x9108, 0x6104}, 	// S02_ILLUCFG  90 deg (pll_index = 2, illu_code_index = 2)
	{0x9109, 0x5D57}, 	// S03_EXPOTIMEMAX (1330us @ 45.18 MHz)
	{0x910B, 0x6104}, 	// S03_ILLUCFG  180 deg (pll_index = 2, illu_code_index = 2)
	{0x910C, 0x5D57}, 	// S04_EXPOTIMEMAX (1330us @ 45.18 MHz)
	{0x910E, 0x6104}, 	// S04_ILLUCFG  270 deg (pll_index = 2, illu_code_index = 2)
	{0x910F, 0x5873}, 	// S05_EXPOTIMEMAX (1330us @ 37.65 MHz)
	{0x9111, 0x7106}, 	// S05_ILLUCFG  0 deg (pll_index = 3, illu_code_index = 3)
	{0x9112, 0x5873}, 	// S06_EXPOTIMEMAX (1330us @ 37.65 MHz)
	{0x9114, 0x7106}, 	// S06_ILLUCFG  90 deg (pll_index = 3, illu_code_index = 3)
	{0x9115, 0x5873}, 	// S07_EXPOTIMEMAX (1330us @ 37.65 MHz)
	{0x9117, 0x7106}, 	// S07_ILLUCFG  180 deg (pll_index = 3, illu_code_index = 3)
	{0x9118, 0x5873}, 	// S08_EXPOTIMEMAX (1330us @ 37.65 MHz)
	{0x911A, 0x7106}, 	// S08_ILLUCFG  270 deg (pll_index = 3, illu_code_index = 3)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0042}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91C4, 0x0000}, 	// USECASEID
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x8810}, 	// PSOUT (IRS9102C output)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9102C output)
	{0x91DF, 0x0109}, 	// MB_CFG0 (MB0 = 9, LVDS0 active)
	{0x91E7, 0x124F}, 	// MB0_FRAMETIME_MIN (mb_framerate = 10.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9216, 0x1EA1}, 	// PLL_MODLUT2_CFG0 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9217, 0xE148}, 	// PLL_MODLUT2_CFG1 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9218, 0x0002}, 	// PLL_MODLUT2_CFG2 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9219, 0x0701}, 	// PLL_MODLUT2_CFG3 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921A, 0x0000}, 	// PLL_MODLUT2_CFG4 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921B, 0x0272}, 	// PLL_MODLUT2_CFG5 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921C, 0x0044}, 	// PLL_MODLUT2_CFG6 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921D, 0x0038}, 	// PLL_MODLUT2_CFG7 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921E, 0x16A1}, 	// PLL_MODLUT3_CFG0 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x921F, 0xCCCD}, 	// PLL_MODLUT3_CFG1 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9220, 0x0004}, 	// PLL_MODLUT3_CFG2 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9221, 0x2601}, 	// PLL_MODLUT3_CFG3 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9222, 0x6370}, 	// PLL_MODLUT3_CFG4 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9223, 0x0453}, 	// PLL_MODLUT3_CFG5 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9224, 0x0073}, 	// PLL_MODLUT3_CFG6 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9225, 0x0029}, 	// PLL_MODLUT3_CFG7 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x924E, 0x000B}, 	// SENSOR_LENGTH_CODE2 (codelength = 12, dc = 50.00 %)
	{0x924F, 0x0FC0}, 	// SENSOR_CODE2_0      (codelength = 12, dc = 50.00 %)
	{0x9257, 0x000B}, 	// SENSOR_LENGTH_CODE3 (codelength = 12, dc = 50.00 %)
	{0x9258, 0x0FC0}, 	// SENSOR_CODE3_0      (codelength = 12, dc = 50.00 %)
	{0x9272, 0x000B}, 	// ILLU_LENGTH_CODE2 (codelength = 12, dc = 37.50 %)
	{0x9273, 0x0F80}, 	// ILLU_CODE2_0      (codelength = 12, dc = 37.50 %)
	{0x927B, 0x000B}, 	// ILLU_LENGTH_CODE3 (codelength = 12, dc = 37.50 %)
	{0x927C, 0x0F80}, 	// ILLU_CODE3_0      (codelength = 12, dc = 37.50 %)
	{0x928E, 0x0A01}, 	// DIGCLKDIV_S2_PLL2 (f_fsm = 45.18 MHz)
	{0x9293, 0x0A01}, 	// DIGCLKDIV_S3_PLL3 (f_fsm = 37.65 MHz)
	{0x929E, 0x0A00}, 	// CHARGEPUMPDIV_S2_PLL2 (f_chargepump = 90.36 MHz)
	{0x92A3, 0x0A00}, 	// CHARGEPUMPDIV_S3_PLL3 (f_chargepump = 75.30 MHz)
	{0x92AE, 0x0B02}, 	// DLLREGDELAY_S2_PLL2
	{0x92B3, 0x0B05}, 	// DLLREGDELAY_S3_PLL3
	{0x92F3, 0x0276}, 	// EYE_FILT_FS6_11
	{0x92F4, 0x027B}, 	// EYE_FILT_FS6_10
	{0x92F5, 0x027A}, 	// EYE_FILT_FS6_9
	{0x92F6, 0x0276}, 	// EYE_FILT_FS6_8
	{0x92F7, 0x0276}, 	// EYE_FILT_FS6_7
	{0x92F8, 0x0276}, 	// EYE_FILT_FS6_6
	{0x92F9, 0x0276}, 	// EYE_FILT_FS6_5
	{0x92FA, 0x0276}, 	// EYE_FILT_FS6_4
	{0x92FB, 0x0276}, 	// EYE_FILT_FS6_3
	{0x92FC, 0x0276}, 	// EYE_FILT_FS6_2
	{0x92FD, 0x0279}, 	// EYE_FILT_FS6_1
	{0x92FE, 0x027C}, 	// EYE_FILT_FS6_0
	{0x92FF, 0x013B}, 	// EYE_FILT_FS5_7
	{0x9300, 0x0149}, 	// EYE_FILT_FS5_6
	{0x9301, 0x0125}, 	// EYE_FILT_FS5_5
	{0x9302, 0x0149}, 	// EYE_FILT_FS5_4
	{0x9303, 0x0149}, 	// EYE_FILT_FS5_3
	{0x9304, 0x0125}, 	// EYE_FILT_FS5_2
	{0x9305, 0x0149}, 	// EYE_FILT_FS5_1
	{0x9306, 0x0149}, 	// EYE_FILT_FS5_0
	{0x9307, 0x0080}, 	// EYE_FILT_FS4_7
	{0x9308, 0x0080}, 	// EYE_FILT_FS4_6
	{0x9309, 0x00FC}, 	// EYE_FILT_FS4_5
	{0x930A, 0x0096}, 	// EYE_FILT_FS4_4
	{0x930B, 0x0080}, 	// EYE_FILT_FS4_3
	{0x930C, 0x0080}, 	// EYE_FILT_FS4_2
	{0x930D, 0x0080}, 	// EYE_FILT_FS4_1
	{0x930E, 0x0080}, 	// EYE_FILT_FS4_0
	{0x930F, 0x0040}, 	// EYE_FILT_FS3_7
	{0x9310, 0x0040}, 	// EYE_FILT_FS3_6
	{0x9311, 0x0040}, 	// EYE_FILT_FS3_5
	{0x9312, 0x0040}, 	// EYE_FILT_FS3_4
	{0x9313, 0x0040}, 	// EYE_FILT_FS3_3
	{0x9314, 0x0040}, 	// EYE_FILT_FS3_2
	{0x9315, 0x0040}, 	// EYE_FILT_FS3_1
	{0x9316, 0x0040}, 	// EYE_FILT_FS3_0
	{0x9317, 0x0040}, 	// EYE_FILT_FS2_7
	{0x9318, 0x0040}, 	// EYE_FILT_FS2_6
	{0x9319, 0x0040}, 	// EYE_FILT_FS2_5
	{0x931A, 0x0040}, 	// EYE_FILT_FS2_4
	{0x931B, 0x0040}, 	// EYE_FILT_FS2_3
	{0x931C, 0x0040}, 	// EYE_FILT_FS2_2
	{0x931D, 0x0040}, 	// EYE_FILT_FS2_1
	{0x931E, 0x0040}, 	// EYE_FILT_FS2_0
	{0x931F, 0x0040}, 	// EYE_FILT_FS1_7
	{0x9320, 0x0040}, 	// EYE_FILT_FS1_6
	{0x9321, 0x0040}, 	// EYE_FILT_FS1_5
	{0x9322, 0x0040}, 	// EYE_FILT_FS1_4
	{0x9323, 0x0040}, 	// EYE_FILT_FS1_3
	{0x9324, 0x0040}, 	// EYE_FILT_FS1_2
	{0x9325, 0x0040}, 	// EYE_FILT_FS1_1
	{0x9326, 0x0040}, 	// EYE_FILT_FS1_0
	{0x9327, 0x0040}, 	// EYE_FILT_FS0_7
	{0x9328, 0x0040}, 	// EYE_FILT_FS0_6
	{0x9329, 0x0040}, 	// EYE_FILT_FS0_5
	{0x932A, 0x0040}, 	// EYE_FILT_FS0_4
	{0x932B, 0x0040}, 	// EYE_FILT_FS0_3
	{0x932C, 0x0040}, 	// EYE_FILT_FS0_2
	{0x932D, 0x0040}, 	// EYE_FILT_FS0_1
	{0x932E, 0x0040}, 	// EYE_FILT_FS0_0
	{0x932F, 0x1B0D}, 	// EYE_THRES_FS0_CFG0 (Filter Stage 0, TH1 | TH0)
	{0x9330, 0x3729}, 	// EYE_THRES_FS0_CFG1 (Filter Stage 0, TH3 | TH2)
	{0x9331, 0x5345}, 	// EYE_THRES_FS0_CFG2 (Filter Stage 0, TH5 | TH4)
	{0x9332, 0x6E60}, 	// EYE_THRES_FS0_CFG3 (Filter Stage 0, TH7 | TH6)
	{0x9333, 0x291B}, 	// EYE_THRES_FS1_CFG0 (Filter Stage 1, TH1 | TH0)
	{0x9334, 0x4437}, 	// EYE_THRES_FS1_CFG1 (Filter Stage 1, TH3 | TH2)
	{0x9335, 0x594F}, 	// EYE_THRES_FS1_CFG2 (Filter Stage 1, TH5 | TH4)
	{0x9336, 0x0062}, 	// EYE_THRES_FS1_CFG3 (Filter Stage 1,     | TH6)
	{0x9337, 0x7354}, 	// EYE_THRES_FS2_CFG0 (Filter Stage 2, TH1 | TH0)
	{0x9338, 0xAB90}, 	// EYE_THRES_FS2_CFG1 (Filter Stage 2, TH3 | TH2)
	{0x9339, 0xDFC5}, 	// EYE_THRES_FS2_CFG2 (Filter Stage 2, TH5 | TH4)
	{0x933A, 0x00F7}, 	// EYE_THRES_FS2_CFG3 (Filter Stage 2,     | TH6)
	{0x933B, 0x4935}, 	// EYE_THRES_FS3_CFG0 (Filter Stage 3, TH1 | TH0)
	{0x933C, 0x6E5C}, 	// EYE_THRES_FS3_CFG1 (Filter Stage 3, TH3 | TH2)
	{0x933D, 0x907F}, 	// EYE_THRES_FS3_CFG2 (Filter Stage 3, TH5 | TH4)
	{0x933E, 0x00A0}, 	// EYE_THRES_FS3_CFG3 (Filter Stage 3,     | TH6)
	{0x933F, 0x6145}, 	// EYE_THRES_FS4_CFG0 (Filter Stage 4, TH1 | TH0)
	{0x9340, 0x937A}, 	// EYE_THRES_FS4_CFG1 (Filter Stage 4, TH3 | TH2)
	{0x9341, 0xC2AB}, 	// EYE_THRES_FS4_CFG2 (Filter Stage 4, TH5 | TH4)
	{0x9342, 0x00D8}, 	// EYE_THRES_FS4_CFG3 (Filter Stage 4,     | TH6)
	{0x9343, 0x2118}, 	// EYE_THRES_FS5_CFG0 (Filter Stage 5, TH1 | TH0)
	{0x9344, 0x332A}, 	// EYE_THRES_FS5_CFG1 (Filter Stage 5, TH3 | TH2)
	{0x9345, 0x453C}, 	// EYE_THRES_FS5_CFG2 (Filter Stage 5, TH5 | TH4)
	{0x9346, 0x004D}, 	// EYE_THRES_FS5_CFG3 (Filter Stage 5,     | TH6)
	{0x9347, 0x1811}, 	// EYE_THRES_FS6_CFG0 (Filter Stage 6, TH1 | TH0)
	{0x9348, 0x2620}, 	// EYE_THRES_FS6_CFG1 (Filter Stage 6, TH3 | TH2)
	{0x9349, 0x342D}, 	// EYE_THRES_FS6_CFG2 (Filter Stage 6, TH5 | TH4)
	{0x934A, 0x423A}, 	// EYE_THRES_FS6_CFG3 (Filter Stage 6, TH7 | TH6)
	{0x934B, 0x5049}, 	// EYE_THRES_FS6_CFG4 (Filter Stage 6, TH9 | TH8)
	{0x934C, 0x0058}, 	// EYE_THRES_FS6_CFG5 (Filter Stage 6,     | TH10)
	{0x934D, 0x0150}, 	// EYE_CMPMUX
	{0x934E, 0x0122}, 	// EYE_THRES_H_H (2.530 W)
	{0x934F, 0x0099}, 	// EYE_THRES_H_L (1.000 W)
	{0x9350, 0x0075}, 	// EYE_THRES_L_H (0.592 W)
	{0x9351, 0x0029}, 	// EYE_THRES_L_L (-0.250 W)
	{0x9352, 0x0006}, 	// EYE_CFG (11 plausibility samples, icm_idle enabled)
	{0x9353, 0x0100}, 	// EYE_IDLEFRAMETIME (81.92 ms)
	{0x9354, 0x0300}, 	// MODULEID0
	{0x9355, 0x0000}, 	// MODULEID1
	{0x9356, 0x0000}, 	// MODULEID2
	{0x9357, 0x0000}, 	// MODULEID3
	{0x9358, 0x0000}, 	// MODULEID4
	{0x9359, 0x0000}, 	// MODULEID5
	{0x935A, 0x0000}, 	// MODULEID6
	{0x935B, 0x010E}, 	// MODULEID7
	{0x935C, 0x450A}, 	// CRC
	{0x935D, 0x00A7}, 	// ROIROWMAX (156 rows)
	{0x935E, 0x000C}, 	// ROIROWMIN (156 rows)
	{0x935F, 0x00DF}, 	// ROICOLMAX (208 columns)
	{0x9360, 0x0010}, 	// ROICOLMIN (208 columns)
	{0x9362, 0x124F}, 	// MB0_FRAMETIME (mb_framerate = 10.0 fps)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

static const struct regval irs2875_208x1413_15_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1A13}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = Input)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0x9000, 0x12D3}, 	// S00_EXPOTIME (128us @ 37.65 MHz)
	{0x9002, 0x00A6}, 	// S00_SENSORCFG (sensor_code_index = 3)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x560F}, 	// S01_EXPOTIME (1000us @ 45.18 MHz)
	{0x9006, 0x0004}, 	// S01_SENSORCFG (sensor_code_index = 2)
	{0x9007, 0x0000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9008, 0x560F}, 	// S02_EXPOTIME (1000us @ 45.18 MHz)
	{0x900A, 0x0004}, 	// S02_SENSORCFG (sensor_code_index = 2)
	{0x900B, 0x0003}, 	// S02_ROCFG (sensor_rotates = 3)
	{0x900C, 0x560F}, 	// S03_EXPOTIME (1000us @ 45.18 MHz)
	{0x900E, 0x0004}, 	// S03_SENSORCFG (sensor_code_index = 2)
	{0x900F, 0x0006}, 	// S03_ROCFG (sensor_rotates = 6)
	{0x9010, 0x560F}, 	// S04_EXPOTIME (1000us @ 45.18 MHz)
	{0x9012, 0x0004}, 	// S04_SENSORCFG (sensor_code_index = 2)
	{0x9013, 0x0009}, 	// S04_ROCFG (sensor_rotates = 9)
	{0x9014, 0x5262}, 	// S05_EXPOTIME (1000us @ 37.65 MHz)
	{0x9016, 0x0006}, 	// S05_SENSORCFG (sensor_code_index = 3)
	{0x9017, 0x0000}, 	// S05_ROCFG (sensor_rotates = 0)
	{0x9018, 0x5262}, 	// S06_EXPOTIME (1000us @ 37.65 MHz)
	{0x901A, 0x0006}, 	// S06_SENSORCFG (sensor_code_index = 3)
	{0x901B, 0x0003}, 	// S06_ROCFG (sensor_rotates = 3)
	{0x901C, 0x5262}, 	// S07_EXPOTIME (1000us @ 37.65 MHz)
	{0x901E, 0x0006}, 	// S07_SENSORCFG (sensor_code_index = 3)
	{0x901F, 0x0006}, 	// S07_ROCFG (sensor_rotates = 6)
	{0x9020, 0x5262}, 	// S08_EXPOTIME (1000us @ 37.65 MHz)
	{0x9022, 0x0006}, 	// S08_SENSORCFG (sensor_code_index = 3)
	{0x9023, 0xC009}, 	// S08_ROCFG (sensor_rotates = 9)
	{0x9100, 0x12D3}, 	// S00_EXPOTIMEMAX (128us @ 37.65 MHz)
	{0x9102, 0x30A6}, 	// S00_ILLUCFG  Goff (pll_index = 3, illu_code_index = 3)
	{0x9103, 0x5B22}, 	// S01_EXPOTIMEMAX (1230us @ 45.18 MHz)
	{0x9105, 0x6104}, 	// S01_ILLUCFG  0 deg (pll_index = 2, illu_code_index = 2)
	{0x9106, 0x5B22}, 	// S02_EXPOTIMEMAX (1230us @ 45.18 MHz)
	{0x9108, 0x6104}, 	// S02_ILLUCFG  90 deg (pll_index = 2, illu_code_index = 2)
	{0x9109, 0x5B22}, 	// S03_EXPOTIMEMAX (1230us @ 45.18 MHz)
	{0x910B, 0x6104}, 	// S03_ILLUCFG  180 deg (pll_index = 2, illu_code_index = 2)
	{0x910C, 0x5B22}, 	// S04_EXPOTIMEMAX (1230us @ 45.18 MHz)
	{0x910E, 0x6104}, 	// S04_ILLUCFG  270 deg (pll_index = 2, illu_code_index = 2)
	{0x910F, 0x569C}, 	// S05_EXPOTIMEMAX (1230us @ 37.65 MHz)
	{0x9111, 0x7106}, 	// S05_ILLUCFG  0 deg (pll_index = 3, illu_code_index = 3)
	{0x9112, 0x569C}, 	// S06_EXPOTIMEMAX (1230us @ 37.65 MHz)
	{0x9114, 0x7106}, 	// S06_ILLUCFG  90 deg (pll_index = 3, illu_code_index = 3)
	{0x9115, 0x569C}, 	// S07_EXPOTIMEMAX (1230us @ 37.65 MHz)
	{0x9117, 0x7106}, 	// S07_ILLUCFG  180 deg (pll_index = 3, illu_code_index = 3)
	{0x9118, 0x569C}, 	// S08_EXPOTIMEMAX (1230us @ 37.65 MHz)
	{0x911A, 0x7106}, 	// S08_ILLUCFG  270 deg (pll_index = 3, illu_code_index = 3)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0042}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91C4, 0x0000}, 	// USECASEID
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x8810}, 	// PSOUT (IRS9102C output)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9102C output)
	{0x91DF, 0x0109}, 	// MB_CFG0 (MB0 = 9, LVDS0 active)
	{0x91E7, 0x0C34}, 	// MB0_FRAMETIME_MIN (mb_framerate = 15.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9216, 0x1EA1}, 	// PLL_MODLUT2_CFG0 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9217, 0xE148}, 	// PLL_MODLUT2_CFG1 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9218, 0x0002}, 	// PLL_MODLUT2_CFG2 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9219, 0x0701}, 	// PLL_MODLUT2_CFG3 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921A, 0x0000}, 	// PLL_MODLUT2_CFG4 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921B, 0x0272}, 	// PLL_MODLUT2_CFG5 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921C, 0x0044}, 	// PLL_MODLUT2_CFG6 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921D, 0x0038}, 	// PLL_MODLUT2_CFG7 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921E, 0x16A1}, 	// PLL_MODLUT3_CFG0 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x921F, 0xCCCD}, 	// PLL_MODLUT3_CFG1 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9220, 0x0004}, 	// PLL_MODLUT3_CFG2 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9221, 0x2601}, 	// PLL_MODLUT3_CFG3 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9222, 0x6370}, 	// PLL_MODLUT3_CFG4 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9223, 0x0453}, 	// PLL_MODLUT3_CFG5 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9224, 0x0073}, 	// PLL_MODLUT3_CFG6 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9225, 0x0029}, 	// PLL_MODLUT3_CFG7 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x924E, 0x000B}, 	// SENSOR_LENGTH_CODE2 (codelength = 12, dc = 50.00 %)
	{0x924F, 0x0FC0}, 	// SENSOR_CODE2_0      (codelength = 12, dc = 50.00 %)
	{0x9257, 0x000B}, 	// SENSOR_LENGTH_CODE3 (codelength = 12, dc = 50.00 %)
	{0x9258, 0x0FC0}, 	// SENSOR_CODE3_0      (codelength = 12, dc = 50.00 %)
	{0x9272, 0x000B}, 	// ILLU_LENGTH_CODE2 (codelength = 12, dc = 37.50 %)
	{0x9273, 0x0F80}, 	// ILLU_CODE2_0      (codelength = 12, dc = 37.50 %)
	{0x927B, 0x000B}, 	// ILLU_LENGTH_CODE3 (codelength = 12, dc = 37.50 %)
	{0x927C, 0x0F80}, 	// ILLU_CODE3_0      (codelength = 12, dc = 37.50 %)
	{0x928E, 0x0A01}, 	// DIGCLKDIV_S2_PLL2 (f_fsm = 45.18 MHz)
	{0x9293, 0x0A01}, 	// DIGCLKDIV_S3_PLL3 (f_fsm = 37.65 MHz)
	{0x929E, 0x0A00}, 	// CHARGEPUMPDIV_S2_PLL2 (f_chargepump = 90.36 MHz)
	{0x92A3, 0x0A00}, 	// CHARGEPUMPDIV_S3_PLL3 (f_chargepump = 75.30 MHz)
	{0x92AE, 0x0B02}, 	// DLLREGDELAY_S2_PLL2
	{0x92B3, 0x0B05}, 	// DLLREGDELAY_S3_PLL3
	{0x92F3, 0x029F}, 	// EYE_FILT_FS6_11
	{0x92F4, 0x02A8}, 	// EYE_FILT_FS6_10
	{0x92F5, 0x02A8}, 	// EYE_FILT_FS6_9
	{0x92F6, 0x029F}, 	// EYE_FILT_FS6_8
	{0x92F7, 0x02A8}, 	// EYE_FILT_FS6_7
	{0x92F8, 0x02A8}, 	// EYE_FILT_FS6_6
	{0x92F9, 0x029F}, 	// EYE_FILT_FS6_5
	{0x92FA, 0x02A8}, 	// EYE_FILT_FS6_4
	{0x92FB, 0x02A8}, 	// EYE_FILT_FS6_3
	{0x92FC, 0x029F}, 	// EYE_FILT_FS6_2
	{0x92FD, 0x02A8}, 	// EYE_FILT_FS6_1
	{0x92FE, 0x02A8}, 	// EYE_FILT_FS6_0
	{0x92FF, 0x0148}, 	// EYE_FILT_FS5_7
	{0x9300, 0x0160}, 	// EYE_FILT_FS5_6
	{0x9301, 0x0143}, 	// EYE_FILT_FS5_5
	{0x9302, 0x0165}, 	// EYE_FILT_FS5_4
	{0x9303, 0x0143}, 	// EYE_FILT_FS5_3
	{0x9304, 0x0165}, 	// EYE_FILT_FS5_2
	{0x9305, 0x0143}, 	// EYE_FILT_FS5_1
	{0x9306, 0x0165}, 	// EYE_FILT_FS5_0
	{0x9307, 0x00B3}, 	// EYE_FILT_FS4_7
	{0x9308, 0x00D4}, 	// EYE_FILT_FS4_6
	{0x9309, 0x0080}, 	// EYE_FILT_FS4_5
	{0x930A, 0x0087}, 	// EYE_FILT_FS4_4
	{0x930B, 0x0100}, 	// EYE_FILT_FS4_3
	{0x930C, 0x0080}, 	// EYE_FILT_FS4_2
	{0x930D, 0x0080}, 	// EYE_FILT_FS4_1
	{0x930E, 0x0080}, 	// EYE_FILT_FS4_0
	{0x930F, 0x0040}, 	// EYE_FILT_FS3_7
	{0x9310, 0x0040}, 	// EYE_FILT_FS3_6
	{0x9311, 0x0040}, 	// EYE_FILT_FS3_5
	{0x9312, 0x0040}, 	// EYE_FILT_FS3_4
	{0x9313, 0x0040}, 	// EYE_FILT_FS3_3
	{0x9314, 0x0040}, 	// EYE_FILT_FS3_2
	{0x9315, 0x0040}, 	// EYE_FILT_FS3_1
	{0x9316, 0x0040}, 	// EYE_FILT_FS3_0
	{0x9317, 0x0040}, 	// EYE_FILT_FS2_7
	{0x9318, 0x0040}, 	// EYE_FILT_FS2_6
	{0x9319, 0x0040}, 	// EYE_FILT_FS2_5
	{0x931A, 0x0040}, 	// EYE_FILT_FS2_4
	{0x931B, 0x0040}, 	// EYE_FILT_FS2_3
	{0x931C, 0x0040}, 	// EYE_FILT_FS2_2
	{0x931D, 0x0040}, 	// EYE_FILT_FS2_1
	{0x931E, 0x0040}, 	// EYE_FILT_FS2_0
	{0x931F, 0x0040}, 	// EYE_FILT_FS1_7
	{0x9320, 0x0040}, 	// EYE_FILT_FS1_6
	{0x9321, 0x0040}, 	// EYE_FILT_FS1_5
	{0x9322, 0x0040}, 	// EYE_FILT_FS1_4
	{0x9323, 0x0040}, 	// EYE_FILT_FS1_3
	{0x9324, 0x0040}, 	// EYE_FILT_FS1_2
	{0x9325, 0x0040}, 	// EYE_FILT_FS1_1
	{0x9326, 0x0040}, 	// EYE_FILT_FS1_0
	{0x9327, 0x0040}, 	// EYE_FILT_FS0_7
	{0x9328, 0x0040}, 	// EYE_FILT_FS0_6
	{0x9329, 0x0040}, 	// EYE_FILT_FS0_5
	{0x932A, 0x0040}, 	// EYE_FILT_FS0_4
	{0x932B, 0x0040}, 	// EYE_FILT_FS0_3
	{0x932C, 0x0040}, 	// EYE_FILT_FS0_2
	{0x932D, 0x0040}, 	// EYE_FILT_FS0_1
	{0x932E, 0x0040}, 	// EYE_FILT_FS0_0
	{0x932F, 0x1B0D}, 	// EYE_THRES_FS0_CFG0 (Filter Stage 0, TH1 | TH0)
	{0x9330, 0x3729}, 	// EYE_THRES_FS0_CFG1 (Filter Stage 0, TH3 | TH2)
	{0x9331, 0x5345}, 	// EYE_THRES_FS0_CFG2 (Filter Stage 0, TH5 | TH4)
	{0x9332, 0x6E60}, 	// EYE_THRES_FS0_CFG3 (Filter Stage 0, TH7 | TH6)
	{0x9333, 0x291B}, 	// EYE_THRES_FS1_CFG0 (Filter Stage 1, TH1 | TH0)
	{0x9334, 0x4437}, 	// EYE_THRES_FS1_CFG1 (Filter Stage 1, TH3 | TH2)
	{0x9335, 0x594F}, 	// EYE_THRES_FS1_CFG2 (Filter Stage 1, TH5 | TH4)
	{0x9336, 0x0062}, 	// EYE_THRES_FS1_CFG3 (Filter Stage 1,     | TH6)
	{0x9337, 0x7354}, 	// EYE_THRES_FS2_CFG0 (Filter Stage 2, TH1 | TH0)
	{0x9338, 0xAB90}, 	// EYE_THRES_FS2_CFG1 (Filter Stage 2, TH3 | TH2)
	{0x9339, 0xDFC5}, 	// EYE_THRES_FS2_CFG2 (Filter Stage 2, TH5 | TH4)
	{0x933A, 0x00F7}, 	// EYE_THRES_FS2_CFG3 (Filter Stage 2,     | TH6)
	{0x933B, 0x4935}, 	// EYE_THRES_FS3_CFG0 (Filter Stage 3, TH1 | TH0)
	{0x933C, 0x6E5C}, 	// EYE_THRES_FS3_CFG1 (Filter Stage 3, TH3 | TH2)
	{0x933D, 0x907F}, 	// EYE_THRES_FS3_CFG2 (Filter Stage 3, TH5 | TH4)
	{0x933E, 0x00A0}, 	// EYE_THRES_FS3_CFG3 (Filter Stage 3,     | TH6)
	{0x933F, 0x6145}, 	// EYE_THRES_FS4_CFG0 (Filter Stage 4, TH1 | TH0)
	{0x9340, 0x937A}, 	// EYE_THRES_FS4_CFG1 (Filter Stage 4, TH3 | TH2)
	{0x9341, 0xC2AB}, 	// EYE_THRES_FS4_CFG2 (Filter Stage 4, TH5 | TH4)
	{0x9342, 0x00D8}, 	// EYE_THRES_FS4_CFG3 (Filter Stage 4,     | TH6)
	{0x9343, 0x2118}, 	// EYE_THRES_FS5_CFG0 (Filter Stage 5, TH1 | TH0)
	{0x9344, 0x332A}, 	// EYE_THRES_FS5_CFG1 (Filter Stage 5, TH3 | TH2)
	{0x9345, 0x453C}, 	// EYE_THRES_FS5_CFG2 (Filter Stage 5, TH5 | TH4)
	{0x9346, 0x004D}, 	// EYE_THRES_FS5_CFG3 (Filter Stage 5,     | TH6)
	{0x9347, 0x1811}, 	// EYE_THRES_FS6_CFG0 (Filter Stage 6, TH1 | TH0)
	{0x9348, 0x2620}, 	// EYE_THRES_FS6_CFG1 (Filter Stage 6, TH3 | TH2)
	{0x9349, 0x342D}, 	// EYE_THRES_FS6_CFG2 (Filter Stage 6, TH5 | TH4)
	{0x934A, 0x423A}, 	// EYE_THRES_FS6_CFG3 (Filter Stage 6, TH7 | TH6)
	{0x934B, 0x5049}, 	// EYE_THRES_FS6_CFG4 (Filter Stage 6, TH9 | TH8)
	{0x934C, 0x0058}, 	// EYE_THRES_FS6_CFG5 (Filter Stage 6,     | TH10)
	{0x934D, 0x0150}, 	// EYE_CMPMUX
	{0x934E, 0x0122}, 	// EYE_THRES_H_H (2.530 W)
	{0x934F, 0x0099}, 	// EYE_THRES_H_L (1.000 W)
	{0x9350, 0x0075}, 	// EYE_THRES_L_H (0.592 W)
	{0x9351, 0x0029}, 	// EYE_THRES_L_L (-0.250 W)
	{0x9352, 0x0006}, 	// EYE_CFG (11 plausibility samples, icm_idle enabled)
	{0x9353, 0x009A}, 	// EYE_IDLEFRAMETIME (49.28 ms)
	{0x9354, 0x0300}, 	// MODULEID0
	{0x9355, 0x0000}, 	// MODULEID1
	{0x9356, 0x0000}, 	// MODULEID2
	{0x9357, 0x0000}, 	// MODULEID3
	{0x9358, 0x0000}, 	// MODULEID4
	{0x9359, 0x0000}, 	// MODULEID5
	{0x935A, 0x0000}, 	// MODULEID6
	{0x935B, 0x010E}, 	// MODULEID7
	{0x935C, 0xD19C}, 	// CRC
	{0x935D, 0x00A7}, 	// ROIROWMAX (156 rows)
	{0x935E, 0x000C}, 	// ROIROWMIN (156 rows)
	{0x935F, 0x00DF}, 	// ROICOLMAX (208 columns)
	{0x9360, 0x0010}, 	// ROICOLMIN (208 columns)
	{0x9362, 0x0C34}, 	// MB0_FRAMETIME (mb_framerate = 15.0 fps)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

static const struct regval irs2875_208x1413_20_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1A13}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = Input)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0x9000, 0x12D3}, 	// S00_EXPOTIME (128us @ 37.65 MHz)
	{0x9002, 0x00A6}, 	// S00_SENSORCFG (sensor_code_index = 3)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x560F}, 	// S01_EXPOTIME (1000us @ 45.18 MHz)
	{0x9006, 0x0004}, 	// S01_SENSORCFG (sensor_code_index = 2)
	{0x9007, 0x0000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9008, 0x560F}, 	// S02_EXPOTIME (1000us @ 45.18 MHz)
	{0x900A, 0x0004}, 	// S02_SENSORCFG (sensor_code_index = 2)
	{0x900B, 0x0003}, 	// S02_ROCFG (sensor_rotates = 3)
	{0x900C, 0x560F}, 	// S03_EXPOTIME (1000us @ 45.18 MHz)
	{0x900E, 0x0004}, 	// S03_SENSORCFG (sensor_code_index = 2)
	{0x900F, 0x0006}, 	// S03_ROCFG (sensor_rotates = 6)
	{0x9010, 0x560F}, 	// S04_EXPOTIME (1000us @ 45.18 MHz)
	{0x9012, 0x0004}, 	// S04_SENSORCFG (sensor_code_index = 2)
	{0x9013, 0x0009}, 	// S04_ROCFG (sensor_rotates = 9)
	{0x9014, 0x5262}, 	// S05_EXPOTIME (1000us @ 37.65 MHz)
	{0x9016, 0x0006}, 	// S05_SENSORCFG (sensor_code_index = 3)
	{0x9017, 0x0000}, 	// S05_ROCFG (sensor_rotates = 0)
	{0x9018, 0x5262}, 	// S06_EXPOTIME (1000us @ 37.65 MHz)
	{0x901A, 0x0006}, 	// S06_SENSORCFG (sensor_code_index = 3)
	{0x901B, 0x0003}, 	// S06_ROCFG (sensor_rotates = 3)
	{0x901C, 0x5262}, 	// S07_EXPOTIME (1000us @ 37.65 MHz)
	{0x901E, 0x0006}, 	// S07_SENSORCFG (sensor_code_index = 3)
	{0x901F, 0x0006}, 	// S07_ROCFG (sensor_rotates = 6)
	{0x9020, 0x5262}, 	// S08_EXPOTIME (1000us @ 37.65 MHz)
	{0x9022, 0x0006}, 	// S08_SENSORCFG (sensor_code_index = 3)
	{0x9023, 0xC009}, 	// S08_ROCFG (sensor_rotates = 9)
	{0x9100, 0x12D3}, 	// S00_EXPOTIMEMAX (128us @ 37.65 MHz)
	{0x9102, 0x30A6}, 	// S00_ILLUCFG  Goff (pll_index = 3, illu_code_index = 3)
	{0x9103, 0x595E}, 	// S01_EXPOTIMEMAX (1150us @ 45.18 MHz)
	{0x9105, 0x6104}, 	// S01_ILLUCFG  0 deg (pll_index = 2, illu_code_index = 2)
	{0x9106, 0x595E}, 	// S02_EXPOTIMEMAX (1150us @ 45.18 MHz)
	{0x9108, 0x6104}, 	// S02_ILLUCFG  90 deg (pll_index = 2, illu_code_index = 2)
	{0x9109, 0x595E}, 	// S03_EXPOTIMEMAX (1150us @ 45.18 MHz)
	{0x910B, 0x6104}, 	// S03_ILLUCFG  180 deg (pll_index = 2, illu_code_index = 2)
	{0x910C, 0x595E}, 	// S04_EXPOTIMEMAX (1150us @ 45.18 MHz)
	{0x910E, 0x6104}, 	// S04_ILLUCFG  270 deg (pll_index = 2, illu_code_index = 2)
	{0x910F, 0x5524}, 	// S05_EXPOTIMEMAX (1150us @ 37.65 MHz)
	{0x9111, 0x7106}, 	// S05_ILLUCFG  0 deg (pll_index = 3, illu_code_index = 3)
	{0x9112, 0x5524}, 	// S06_EXPOTIMEMAX (1150us @ 37.65 MHz)
	{0x9114, 0x7106}, 	// S06_ILLUCFG  90 deg (pll_index = 3, illu_code_index = 3)
	{0x9115, 0x5524}, 	// S07_EXPOTIMEMAX (1150us @ 37.65 MHz)
	{0x9117, 0x7106}, 	// S07_ILLUCFG  180 deg (pll_index = 3, illu_code_index = 3)
	{0x9118, 0x5524}, 	// S08_EXPOTIMEMAX (1150us @ 37.65 MHz)
	{0x911A, 0x7106}, 	// S08_ILLUCFG  270 deg (pll_index = 3, illu_code_index = 3)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0042}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91C4, 0x0000}, 	// USECASEID
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x8810}, 	// PSOUT (IRS9102C output)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9102C output)
	{0x91DF, 0x0109}, 	// MB_CFG0 (MB0 = 9, LVDS0 active)
	{0x91E7, 0x0927}, 	// MB0_FRAMETIME_MIN (mb_framerate = 20.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9216, 0x1EA1}, 	// PLL_MODLUT2_CFG0 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9217, 0xE148}, 	// PLL_MODLUT2_CFG1 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9218, 0x0002}, 	// PLL_MODLUT2_CFG2 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9219, 0x0701}, 	// PLL_MODLUT2_CFG3 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921A, 0x0000}, 	// PLL_MODLUT2_CFG4 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921B, 0x0272}, 	// PLL_MODLUT2_CFG5 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921C, 0x0044}, 	// PLL_MODLUT2_CFG6 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921D, 0x0038}, 	// PLL_MODLUT2_CFG7 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921E, 0x16A1}, 	// PLL_MODLUT3_CFG0 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x921F, 0xCCCD}, 	// PLL_MODLUT3_CFG1 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9220, 0x0004}, 	// PLL_MODLUT3_CFG2 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9221, 0x2601}, 	// PLL_MODLUT3_CFG3 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9222, 0x6370}, 	// PLL_MODLUT3_CFG4 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9223, 0x0453}, 	// PLL_MODLUT3_CFG5 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9224, 0x0073}, 	// PLL_MODLUT3_CFG6 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9225, 0x0029}, 	// PLL_MODLUT3_CFG7 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x924E, 0x000B}, 	// SENSOR_LENGTH_CODE2 (codelength = 12, dc = 50.00 %)
	{0x924F, 0x0FC0}, 	// SENSOR_CODE2_0      (codelength = 12, dc = 50.00 %)
	{0x9257, 0x000B}, 	// SENSOR_LENGTH_CODE3 (codelength = 12, dc = 50.00 %)
	{0x9258, 0x0FC0}, 	// SENSOR_CODE3_0      (codelength = 12, dc = 50.00 %)
	{0x9272, 0x000B}, 	// ILLU_LENGTH_CODE2 (codelength = 12, dc = 37.50 %)
	{0x9273, 0x0F80}, 	// ILLU_CODE2_0      (codelength = 12, dc = 37.50 %)
	{0x927B, 0x000B}, 	// ILLU_LENGTH_CODE3 (codelength = 12, dc = 37.50 %)
	{0x927C, 0x0F80}, 	// ILLU_CODE3_0      (codelength = 12, dc = 37.50 %)
	{0x928E, 0x0A01}, 	// DIGCLKDIV_S2_PLL2 (f_fsm = 45.18 MHz)
	{0x9293, 0x0A01}, 	// DIGCLKDIV_S3_PLL3 (f_fsm = 37.65 MHz)
	{0x929E, 0x0A00}, 	// CHARGEPUMPDIV_S2_PLL2 (f_chargepump = 90.36 MHz)
	{0x92A3, 0x0A00}, 	// CHARGEPUMPDIV_S3_PLL3 (f_chargepump = 75.30 MHz)
	{0x92AE, 0x0B02}, 	// DLLREGDELAY_S2_PLL2
	{0x92B3, 0x0B05}, 	// DLLREGDELAY_S3_PLL3
	{0x92F3, 0x02CC}, 	// EYE_FILT_FS6_11
	{0x92F4, 0x02CF}, 	// EYE_FILT_FS6_10
	{0x92F5, 0x02CF}, 	// EYE_FILT_FS6_9
	{0x92F6, 0x02CC}, 	// EYE_FILT_FS6_8
	{0x92F7, 0x02CC}, 	// EYE_FILT_FS6_7
	{0x92F8, 0x02CC}, 	// EYE_FILT_FS6_6
	{0x92F9, 0x02D1}, 	// EYE_FILT_FS6_5
	{0x92FA, 0x02CE}, 	// EYE_FILT_FS6_4
	{0x92FB, 0x02CC}, 	// EYE_FILT_FS6_3
	{0x92FC, 0x02CC}, 	// EYE_FILT_FS6_2
	{0x92FD, 0x02CE}, 	// EYE_FILT_FS6_1
	{0x92FE, 0x02D1}, 	// EYE_FILT_FS6_0
	{0x92FF, 0x0174}, 	// EYE_FILT_FS5_7
	{0x9300, 0x015E}, 	// EYE_FILT_FS5_6
	{0x9301, 0x015E}, 	// EYE_FILT_FS5_5
	{0x9302, 0x0172}, 	// EYE_FILT_FS5_4
	{0x9303, 0x016A}, 	// EYE_FILT_FS5_3
	{0x9304, 0x015E}, 	// EYE_FILT_FS5_2
	{0x9305, 0x0162}, 	// EYE_FILT_FS5_1
	{0x9306, 0x017A}, 	// EYE_FILT_FS5_0
	{0x9307, 0x00EE}, 	// EYE_FILT_FS4_7
	{0x9308, 0x0080}, 	// EYE_FILT_FS4_6
	{0x9309, 0x00E8}, 	// EYE_FILT_FS4_5
	{0x930A, 0x0096}, 	// EYE_FILT_FS4_4
	{0x930B, 0x00A0}, 	// EYE_FILT_FS4_3
	{0x930C, 0x00DE}, 	// EYE_FILT_FS4_2
	{0x930D, 0x0080}, 	// EYE_FILT_FS4_1
	{0x930E, 0x0080}, 	// EYE_FILT_FS4_0
	{0x930F, 0x0040}, 	// EYE_FILT_FS3_7
	{0x9310, 0x0040}, 	// EYE_FILT_FS3_6
	{0x9311, 0x0040}, 	// EYE_FILT_FS3_5
	{0x9312, 0x0040}, 	// EYE_FILT_FS3_4
	{0x9313, 0x0040}, 	// EYE_FILT_FS3_3
	{0x9314, 0x0040}, 	// EYE_FILT_FS3_2
	{0x9315, 0x0040}, 	// EYE_FILT_FS3_1
	{0x9316, 0x0040}, 	// EYE_FILT_FS3_0
	{0x9317, 0x0040}, 	// EYE_FILT_FS2_7
	{0x9318, 0x0040}, 	// EYE_FILT_FS2_6
	{0x9319, 0x0040}, 	// EYE_FILT_FS2_5
	{0x931A, 0x0040}, 	// EYE_FILT_FS2_4
	{0x931B, 0x0040}, 	// EYE_FILT_FS2_3
	{0x931C, 0x0040}, 	// EYE_FILT_FS2_2
	{0x931D, 0x0040}, 	// EYE_FILT_FS2_1
	{0x931E, 0x0040}, 	// EYE_FILT_FS2_0
	{0x931F, 0x0040}, 	// EYE_FILT_FS1_7
	{0x9320, 0x0040}, 	// EYE_FILT_FS1_6
	{0x9321, 0x0040}, 	// EYE_FILT_FS1_5
	{0x9322, 0x0040}, 	// EYE_FILT_FS1_4
	{0x9323, 0x0040}, 	// EYE_FILT_FS1_3
	{0x9324, 0x0040}, 	// EYE_FILT_FS1_2
	{0x9325, 0x0040}, 	// EYE_FILT_FS1_1
	{0x9326, 0x0040}, 	// EYE_FILT_FS1_0
	{0x9327, 0x0040}, 	// EYE_FILT_FS0_7
	{0x9328, 0x0040}, 	// EYE_FILT_FS0_6
	{0x9329, 0x0040}, 	// EYE_FILT_FS0_5
	{0x932A, 0x0040}, 	// EYE_FILT_FS0_4
	{0x932B, 0x0040}, 	// EYE_FILT_FS0_3
	{0x932C, 0x0040}, 	// EYE_FILT_FS0_2
	{0x932D, 0x0040}, 	// EYE_FILT_FS0_1
	{0x932E, 0x0040}, 	// EYE_FILT_FS0_0
	{0x932F, 0x1B0D}, 	// EYE_THRES_FS0_CFG0 (Filter Stage 0, TH1 | TH0)
	{0x9330, 0x3729}, 	// EYE_THRES_FS0_CFG1 (Filter Stage 0, TH3 | TH2)
	{0x9331, 0x5345}, 	// EYE_THRES_FS0_CFG2 (Filter Stage 0, TH5 | TH4)
	{0x9332, 0x6E60}, 	// EYE_THRES_FS0_CFG3 (Filter Stage 0, TH7 | TH6)
	{0x9333, 0x291B}, 	// EYE_THRES_FS1_CFG0 (Filter Stage 1, TH1 | TH0)
	{0x9334, 0x4437}, 	// EYE_THRES_FS1_CFG1 (Filter Stage 1, TH3 | TH2)
	{0x9335, 0x594F}, 	// EYE_THRES_FS1_CFG2 (Filter Stage 1, TH5 | TH4)
	{0x9336, 0x0062}, 	// EYE_THRES_FS1_CFG3 (Filter Stage 1,     | TH6)
	{0x9337, 0x7354}, 	// EYE_THRES_FS2_CFG0 (Filter Stage 2, TH1 | TH0)
	{0x9338, 0xAB90}, 	// EYE_THRES_FS2_CFG1 (Filter Stage 2, TH3 | TH2)
	{0x9339, 0xDFC5}, 	// EYE_THRES_FS2_CFG2 (Filter Stage 2, TH5 | TH4)
	{0x933A, 0x00F7}, 	// EYE_THRES_FS2_CFG3 (Filter Stage 2,     | TH6)
	{0x933B, 0x4935}, 	// EYE_THRES_FS3_CFG0 (Filter Stage 3, TH1 | TH0)
	{0x933C, 0x6E5C}, 	// EYE_THRES_FS3_CFG1 (Filter Stage 3, TH3 | TH2)
	{0x933D, 0x907F}, 	// EYE_THRES_FS3_CFG2 (Filter Stage 3, TH5 | TH4)
	{0x933E, 0x00A0}, 	// EYE_THRES_FS3_CFG3 (Filter Stage 3,     | TH6)
	{0x933F, 0x6145}, 	// EYE_THRES_FS4_CFG0 (Filter Stage 4, TH1 | TH0)
	{0x9340, 0x937A}, 	// EYE_THRES_FS4_CFG1 (Filter Stage 4, TH3 | TH2)
	{0x9341, 0xC2AB}, 	// EYE_THRES_FS4_CFG2 (Filter Stage 4, TH5 | TH4)
	{0x9342, 0x00D8}, 	// EYE_THRES_FS4_CFG3 (Filter Stage 4,     | TH6)
	{0x9343, 0x2118}, 	// EYE_THRES_FS5_CFG0 (Filter Stage 5, TH1 | TH0)
	{0x9344, 0x332A}, 	// EYE_THRES_FS5_CFG1 (Filter Stage 5, TH3 | TH2)
	{0x9345, 0x453C}, 	// EYE_THRES_FS5_CFG2 (Filter Stage 5, TH5 | TH4)
	{0x9346, 0x004D}, 	// EYE_THRES_FS5_CFG3 (Filter Stage 5,     | TH6)
	{0x9347, 0x1811}, 	// EYE_THRES_FS6_CFG0 (Filter Stage 6, TH1 | TH0)
	{0x9348, 0x2620}, 	// EYE_THRES_FS6_CFG1 (Filter Stage 6, TH3 | TH2)
	{0x9349, 0x342D}, 	// EYE_THRES_FS6_CFG2 (Filter Stage 6, TH5 | TH4)
	{0x934A, 0x423A}, 	// EYE_THRES_FS6_CFG3 (Filter Stage 6, TH7 | TH6)
	{0x934B, 0x5049}, 	// EYE_THRES_FS6_CFG4 (Filter Stage 6, TH9 | TH8)
	{0x934C, 0x0058}, 	// EYE_THRES_FS6_CFG5 (Filter Stage 6,     | TH10)
	{0x934D, 0x0150}, 	// EYE_CMPMUX
	{0x934E, 0x0122}, 	// EYE_THRES_H_H (2.530 W)
	{0x934F, 0x0099}, 	// EYE_THRES_H_L (1.000 W)
	{0x9350, 0x0075}, 	// EYE_THRES_L_H (0.592 W)
	{0x9351, 0x0029}, 	// EYE_THRES_L_L (-0.250 W)
	{0x9352, 0x0006}, 	// EYE_CFG (11 plausibility samples, icm_idle enabled)
	{0x9353, 0x0068}, 	// EYE_IDLEFRAMETIME (33.28 ms)
	{0x9354, 0x0300}, 	// MODULEID0
	{0x9355, 0x0000}, 	// MODULEID1
	{0x9356, 0x0000}, 	// MODULEID2
	{0x9357, 0x0000}, 	// MODULEID3
	{0x9358, 0x0000}, 	// MODULEID4
	{0x9359, 0x0000}, 	// MODULEID5
	{0x935A, 0x0000}, 	// MODULEID6
	{0x935B, 0x010E}, 	// MODULEID7
	{0x935C, 0x5E03}, 	// CRC
	{0x935D, 0x00A7}, 	// ROIROWMAX (156 rows)
	{0x935E, 0x000C}, 	// ROIROWMIN (156 rows)
	{0x935F, 0x00DF}, 	// ROICOLMAX (208 columns)
	{0x9360, 0x0010}, 	// ROICOLMIN (208 columns)
	{0x9362, 0x0927}, 	// MB0_FRAMETIME (mb_framerate = 20.0 fps)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

static const struct regval irs2875_208x1413_30_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1A13}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = Input)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0x9000, 0x12D3}, 	// S00_EXPOTIME (128us @ 37.65 MHz)
	{0x9002, 0x00A6}, 	// S00_SENSORCFG (sensor_code_index = 3)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x4DAD}, 	// S01_EXPOTIME (620us @ 45.18 MHz)
	{0x9006, 0x0004}, 	// S01_SENSORCFG (sensor_code_index = 2)
	{0x9007, 0x0000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9008, 0x4DAD}, 	// S02_EXPOTIME (620us @ 45.18 MHz)
	{0x900A, 0x0004}, 	// S02_SENSORCFG (sensor_code_index = 2)
	{0x900B, 0x0003}, 	// S02_ROCFG (sensor_rotates = 3)
	{0x900C, 0x4DAD}, 	// S03_EXPOTIME (620us @ 45.18 MHz)
	{0x900E, 0x0004}, 	// S03_SENSORCFG (sensor_code_index = 2)
	{0x900F, 0x0006}, 	// S03_ROCFG (sensor_rotates = 6)
	{0x9010, 0x4DAD}, 	// S04_EXPOTIME (620us @ 45.18 MHz)
	{0x9012, 0x0004}, 	// S04_SENSORCFG (sensor_code_index = 2)
	{0x9013, 0x0009}, 	// S04_ROCFG (sensor_rotates = 9)
	{0x9014, 0x4B65}, 	// S05_EXPOTIME (620us @ 37.65 MHz)
	{0x9016, 0x0006}, 	// S05_SENSORCFG (sensor_code_index = 3)
	{0x9017, 0x0000}, 	// S05_ROCFG (sensor_rotates = 0)
	{0x9018, 0x4B65}, 	// S06_EXPOTIME (620us @ 37.65 MHz)
	{0x901A, 0x0006}, 	// S06_SENSORCFG (sensor_code_index = 3)
	{0x901B, 0x0003}, 	// S06_ROCFG (sensor_rotates = 3)
	{0x901C, 0x4B65}, 	// S07_EXPOTIME (620us @ 37.65 MHz)
	{0x901E, 0x0006}, 	// S07_SENSORCFG (sensor_code_index = 3)
	{0x901F, 0x0006}, 	// S07_ROCFG (sensor_rotates = 6)
	{0x9020, 0x4B65}, 	// S08_EXPOTIME (620us @ 37.65 MHz)
	{0x9022, 0x0006}, 	// S08_SENSORCFG (sensor_code_index = 3)
	{0x9023, 0xC009}, 	// S08_ROCFG (sensor_rotates = 9)
	{0x9100, 0x12D3}, 	// S00_EXPOTIMEMAX (128us @ 37.65 MHz)
	{0x9102, 0x30A6}, 	// S00_ILLUCFG  Goff (pll_index = 3, illu_code_index = 3)
	{0x9103, 0x4DAD}, 	// S01_EXPOTIMEMAX (620us @ 45.18 MHz)
	{0x9105, 0x6104}, 	// S01_ILLUCFG  0 deg (pll_index = 2, illu_code_index = 2)
	{0x9106, 0x4DAD}, 	// S02_EXPOTIMEMAX (620us @ 45.18 MHz)
	{0x9108, 0x6104}, 	// S02_ILLUCFG  90 deg (pll_index = 2, illu_code_index = 2)
	{0x9109, 0x4DAD}, 	// S03_EXPOTIMEMAX (620us @ 45.18 MHz)
	{0x910B, 0x6104}, 	// S03_ILLUCFG  180 deg (pll_index = 2, illu_code_index = 2)
	{0x910C, 0x4DAD}, 	// S04_EXPOTIMEMAX (620us @ 45.18 MHz)
	{0x910E, 0x6104}, 	// S04_ILLUCFG  270 deg (pll_index = 2, illu_code_index = 2)
	{0x910F, 0x4B65}, 	// S05_EXPOTIMEMAX (620us @ 37.65 MHz)
	{0x9111, 0x7106}, 	// S05_ILLUCFG  0 deg (pll_index = 3, illu_code_index = 3)
	{0x9112, 0x4B65}, 	// S06_EXPOTIMEMAX (620us @ 37.65 MHz)
	{0x9114, 0x7106}, 	// S06_ILLUCFG  90 deg (pll_index = 3, illu_code_index = 3)
	{0x9115, 0x4B65}, 	// S07_EXPOTIMEMAX (620us @ 37.65 MHz)
	{0x9117, 0x7106}, 	// S07_ILLUCFG  180 deg (pll_index = 3, illu_code_index = 3)
	{0x9118, 0x4B65}, 	// S08_EXPOTIMEMAX (620us @ 37.65 MHz)
	{0x911A, 0x7106}, 	// S08_ILLUCFG  270 deg (pll_index = 3, illu_code_index = 3)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0042}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91C4, 0x0000}, 	// USECASEID
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x8810}, 	// PSOUT (IRS9102C output)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9102C output)
	{0x91DF, 0x0109}, 	// MB_CFG0 (MB0 = 9, LVDS0 active)
	{0x91E7, 0x061A}, 	// MB0_FRAMETIME_MIN (mb_framerate = 30.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9216, 0x1EA1}, 	// PLL_MODLUT2_CFG0 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9217, 0xE148}, 	// PLL_MODLUT2_CFG1 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9218, 0x0002}, 	// PLL_MODLUT2_CFG2 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x9219, 0x0701}, 	// PLL_MODLUT2_CFG3 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921A, 0x0000}, 	// PLL_MODLUT2_CFG4 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921B, 0x0272}, 	// PLL_MODLUT2_CFG5 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921C, 0x0044}, 	// PLL_MODLUT2_CFG6 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921D, 0x0038}, 	// PLL_MODLUT2_CFG7 (for f_illu = 45.18 MHz [f_mod = 542.16 MHz])
	{0x921E, 0x16A1}, 	// PLL_MODLUT3_CFG0 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x921F, 0xCCCD}, 	// PLL_MODLUT3_CFG1 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9220, 0x0004}, 	// PLL_MODLUT3_CFG2 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9221, 0x2601}, 	// PLL_MODLUT3_CFG3 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9222, 0x6370}, 	// PLL_MODLUT3_CFG4 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9223, 0x0453}, 	// PLL_MODLUT3_CFG5 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9224, 0x0073}, 	// PLL_MODLUT3_CFG6 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x9225, 0x0029}, 	// PLL_MODLUT3_CFG7 (for f_illu = 37.65 MHz [f_mod = 451.80 MHz])
	{0x924E, 0x000B}, 	// SENSOR_LENGTH_CODE2 (codelength = 12, dc = 50.00 %)
	{0x924F, 0x0FC0}, 	// SENSOR_CODE2_0      (codelength = 12, dc = 50.00 %)
	{0x9257, 0x000B}, 	// SENSOR_LENGTH_CODE3 (codelength = 12, dc = 50.00 %)
	{0x9258, 0x0FC0}, 	// SENSOR_CODE3_0      (codelength = 12, dc = 50.00 %)
	{0x9272, 0x000B}, 	// ILLU_LENGTH_CODE2 (codelength = 12, dc = 37.50 %)
	{0x9273, 0x0F80}, 	// ILLU_CODE2_0      (codelength = 12, dc = 37.50 %)
	{0x927B, 0x000B}, 	// ILLU_LENGTH_CODE3 (codelength = 12, dc = 37.50 %)
	{0x927C, 0x0F80}, 	// ILLU_CODE3_0      (codelength = 12, dc = 37.50 %)
	{0x928E, 0x0A01}, 	// DIGCLKDIV_S2_PLL2 (f_fsm = 45.18 MHz)
	{0x9293, 0x0A01}, 	// DIGCLKDIV_S3_PLL3 (f_fsm = 37.65 MHz)
	{0x929E, 0x0A00}, 	// CHARGEPUMPDIV_S2_PLL2 (f_chargepump = 90.36 MHz)
	{0x92A3, 0x0A00}, 	// CHARGEPUMPDIV_S3_PLL3 (f_chargepump = 75.30 MHz)
	{0x92AE, 0x0B02}, 	// DLLREGDELAY_S2_PLL2
	{0x92B3, 0x0B05}, 	// DLLREGDELAY_S3_PLL3
	{0x92F3, 0x02A5}, 	// EYE_FILT_FS6_11
	{0x92F4, 0x02A6}, 	// EYE_FILT_FS6_10
	{0x92F5, 0x02A7}, 	// EYE_FILT_FS6_9
	{0x92F6, 0x02A5}, 	// EYE_FILT_FS6_8
	{0x92F7, 0x02A6}, 	// EYE_FILT_FS6_7
	{0x92F8, 0x02A8}, 	// EYE_FILT_FS6_6
	{0x92F9, 0x02A5}, 	// EYE_FILT_FS6_5
	{0x92FA, 0x02A5}, 	// EYE_FILT_FS6_4
	{0x92FB, 0x02A8}, 	// EYE_FILT_FS6_3
	{0x92FC, 0x02A5}, 	// EYE_FILT_FS6_2
	{0x92FD, 0x02A5}, 	// EYE_FILT_FS6_1
	{0x92FE, 0x02A9}, 	// EYE_FILT_FS6_0
	{0x92FF, 0x0154}, 	// EYE_FILT_FS5_7
	{0x9300, 0x0155}, 	// EYE_FILT_FS5_6
	{0x9301, 0x0155}, 	// EYE_FILT_FS5_5
	{0x9302, 0x0155}, 	// EYE_FILT_FS5_4
	{0x9303, 0x0155}, 	// EYE_FILT_FS5_3
	{0x9304, 0x0155}, 	// EYE_FILT_FS5_2
	{0x9305, 0x0155}, 	// EYE_FILT_FS5_1
	{0x9306, 0x0155}, 	// EYE_FILT_FS5_0
	{0x9307, 0x0089}, 	// EYE_FILT_FS4_7
	{0x9308, 0x00BB}, 	// EYE_FILT_FS4_6
	{0x9309, 0x00B3}, 	// EYE_FILT_FS4_5
	{0x930A, 0x0091}, 	// EYE_FILT_FS4_4
	{0x930B, 0x00C4}, 	// EYE_FILT_FS4_3
	{0x930C, 0x0095}, 	// EYE_FILT_FS4_2
	{0x930D, 0x00AF}, 	// EYE_FILT_FS4_1
	{0x930E, 0x0080}, 	// EYE_FILT_FS4_0
	{0x930F, 0x0040}, 	// EYE_FILT_FS3_7
	{0x9310, 0x0040}, 	// EYE_FILT_FS3_6
	{0x9311, 0x0040}, 	// EYE_FILT_FS3_5
	{0x9312, 0x0040}, 	// EYE_FILT_FS3_4
	{0x9313, 0x0040}, 	// EYE_FILT_FS3_3
	{0x9314, 0x0040}, 	// EYE_FILT_FS3_2
	{0x9315, 0x0040}, 	// EYE_FILT_FS3_1
	{0x9316, 0x0040}, 	// EYE_FILT_FS3_0
	{0x9317, 0x0040}, 	// EYE_FILT_FS2_7
	{0x9318, 0x0040}, 	// EYE_FILT_FS2_6
	{0x9319, 0x0040}, 	// EYE_FILT_FS2_5
	{0x931A, 0x0040}, 	// EYE_FILT_FS2_4
	{0x931B, 0x0040}, 	// EYE_FILT_FS2_3
	{0x931C, 0x0040}, 	// EYE_FILT_FS2_2
	{0x931D, 0x0040}, 	// EYE_FILT_FS2_1
	{0x931E, 0x0040}, 	// EYE_FILT_FS2_0
	{0x931F, 0x0040}, 	// EYE_FILT_FS1_7
	{0x9320, 0x0040}, 	// EYE_FILT_FS1_6
	{0x9321, 0x0040}, 	// EYE_FILT_FS1_5
	{0x9322, 0x0040}, 	// EYE_FILT_FS1_4
	{0x9323, 0x0040}, 	// EYE_FILT_FS1_3
	{0x9324, 0x0040}, 	// EYE_FILT_FS1_2
	{0x9325, 0x0040}, 	// EYE_FILT_FS1_1
	{0x9326, 0x0040}, 	// EYE_FILT_FS1_0
	{0x9327, 0x0040}, 	// EYE_FILT_FS0_7
	{0x9328, 0x0040}, 	// EYE_FILT_FS0_6
	{0x9329, 0x0040}, 	// EYE_FILT_FS0_5
	{0x932A, 0x0040}, 	// EYE_FILT_FS0_4
	{0x932B, 0x0040}, 	// EYE_FILT_FS0_3
	{0x932C, 0x0040}, 	// EYE_FILT_FS0_2
	{0x932D, 0x0040}, 	// EYE_FILT_FS0_1
	{0x932E, 0x0040}, 	// EYE_FILT_FS0_0
	{0x932F, 0x1B0D}, 	// EYE_THRES_FS0_CFG0 (Filter Stage 0, TH1 | TH0)
	{0x9330, 0x3729}, 	// EYE_THRES_FS0_CFG1 (Filter Stage 0, TH3 | TH2)
	{0x9331, 0x5345}, 	// EYE_THRES_FS0_CFG2 (Filter Stage 0, TH5 | TH4)
	{0x9332, 0x6E60}, 	// EYE_THRES_FS0_CFG3 (Filter Stage 0, TH7 | TH6)
	{0x9333, 0x291B}, 	// EYE_THRES_FS1_CFG0 (Filter Stage 1, TH1 | TH0)
	{0x9334, 0x4437}, 	// EYE_THRES_FS1_CFG1 (Filter Stage 1, TH3 | TH2)
	{0x9335, 0x594F}, 	// EYE_THRES_FS1_CFG2 (Filter Stage 1, TH5 | TH4)
	{0x9336, 0x0062}, 	// EYE_THRES_FS1_CFG3 (Filter Stage 1,     | TH6)
	{0x9337, 0x7354}, 	// EYE_THRES_FS2_CFG0 (Filter Stage 2, TH1 | TH0)
	{0x9338, 0xAB90}, 	// EYE_THRES_FS2_CFG1 (Filter Stage 2, TH3 | TH2)
	{0x9339, 0xDFC5}, 	// EYE_THRES_FS2_CFG2 (Filter Stage 2, TH5 | TH4)
	{0x933A, 0x00F7}, 	// EYE_THRES_FS2_CFG3 (Filter Stage 2,     | TH6)
	{0x933B, 0x4935}, 	// EYE_THRES_FS3_CFG0 (Filter Stage 3, TH1 | TH0)
	{0x933C, 0x6E5C}, 	// EYE_THRES_FS3_CFG1 (Filter Stage 3, TH3 | TH2)
	{0x933D, 0x907F}, 	// EYE_THRES_FS3_CFG2 (Filter Stage 3, TH5 | TH4)
	{0x933E, 0x00A0}, 	// EYE_THRES_FS3_CFG3 (Filter Stage 3,     | TH6)
	{0x933F, 0x6145}, 	// EYE_THRES_FS4_CFG0 (Filter Stage 4, TH1 | TH0)
	{0x9340, 0x937A}, 	// EYE_THRES_FS4_CFG1 (Filter Stage 4, TH3 | TH2)
	{0x9341, 0xC2AB}, 	// EYE_THRES_FS4_CFG2 (Filter Stage 4, TH5 | TH4)
	{0x9342, 0x00D8}, 	// EYE_THRES_FS4_CFG3 (Filter Stage 4,     | TH6)
	{0x9343, 0x2118}, 	// EYE_THRES_FS5_CFG0 (Filter Stage 5, TH1 | TH0)
	{0x9344, 0x332A}, 	// EYE_THRES_FS5_CFG1 (Filter Stage 5, TH3 | TH2)
	{0x9345, 0x453C}, 	// EYE_THRES_FS5_CFG2 (Filter Stage 5, TH5 | TH4)
	{0x9346, 0x004D}, 	// EYE_THRES_FS5_CFG3 (Filter Stage 5,     | TH6)
	{0x9347, 0x1811}, 	// EYE_THRES_FS6_CFG0 (Filter Stage 6, TH1 | TH0)
	{0x9348, 0x2620}, 	// EYE_THRES_FS6_CFG1 (Filter Stage 6, TH3 | TH2)
	{0x9349, 0x342D}, 	// EYE_THRES_FS6_CFG2 (Filter Stage 6, TH5 | TH4)
	{0x934A, 0x423A}, 	// EYE_THRES_FS6_CFG3 (Filter Stage 6, TH7 | TH6)
	{0x934B, 0x5049}, 	// EYE_THRES_FS6_CFG4 (Filter Stage 6, TH9 | TH8)
	{0x934C, 0x0058}, 	// EYE_THRES_FS6_CFG5 (Filter Stage 6,     | TH10)
	{0x934D, 0x0150}, 	// EYE_CMPMUX
	{0x934E, 0x0122}, 	// EYE_THRES_H_H (2.530 W)
	{0x934F, 0x0099}, 	// EYE_THRES_H_L (1.000 W)
	{0x9350, 0x0075}, 	// EYE_THRES_L_H (0.592 W)
	{0x9351, 0x0029}, 	// EYE_THRES_L_L (-0.250 W)
	{0x9352, 0x0006}, 	// EYE_CFG (11 plausibility samples, icm_idle enabled)
	{0x9353, 0x0041}, 	// EYE_IDLEFRAMETIME (20.80 ms)
	{0x9354, 0x0300}, 	// MODULEID0
	{0x9355, 0x0000}, 	// MODULEID1
	{0x9356, 0x0000}, 	// MODULEID2
	{0x9357, 0x0000}, 	// MODULEID3
	{0x9358, 0x0000}, 	// MODULEID4
	{0x9359, 0x0000}, 	// MODULEID5
	{0x935A, 0x0000}, 	// MODULEID6
	{0x935B, 0x010E}, 	// MODULEID7
	{0x935C, 0x2A8A}, 	// CRC
	{0x935D, 0x00A7}, 	// ROIROWMAX (156 rows)
	{0x935E, 0x000C}, 	// ROIROWMIN (156 rows)
	{0x935F, 0x00DF}, 	// ROICOLMAX (208 columns)
	{0x9360, 0x0010}, 	// ROICOLMIN (208 columns)
	{0x9362, 0x061A}, 	// MB0_FRAMETIME (mb_framerate = 30.0 fps)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

static const struct regval irs2875_240x362_30_regs[] = {
	{0xA009, 0x1313}, 	// PADGPIOCFG0 (GPIO0 = HighZ, GPIO1 = HighZ)
	{0xA00A, 0x1313}, 	// PADGPIOCFG1 (GPIO2 = HighZ, GPIO3 = HighZ)
	{0xA00B, 0x1313}, 	// PADGPIOCFG2 (GPIO4 = HighZ, GPIO5 = HighZ)
	{0xA02F, 0x16A1}, 	// PLL_SYSLUT_CFG0 (for fref = 24.00 MHz)
	{0xA030, 0x5555}, 	// PLL_SYSLUT_CFG1 (for fref = 24.00 MHz)
	{0xA031, 0x0005}, 	// PLL_SYSLUT_CFG2 (for fref = 24.00 MHz)
	{0xA032, 0x0000}, 	// PLL_SYSLUT_CFG3 (for fref = 24.00 MHz)
	{0xA033, 0x04D0}, 	// PLL_SYSLUT_CFG4 (for fref = 24.00 MHz)
	{0xA034, 0x0000}, 	// PLL_SYSLUT_CFG5 (for fref = 24.00 MHz)
	{0xA035, 0x000F}, 	// PLL_SYSLUT_CFG6 (for fref = 24.00 MHz)
	{0xA038, 0x00A8}, 	// CP_CFG (Bugfix for A11)
	{0xA047, 0x0F07}, 	// DPHYDLANECFG_1GB_2 (Bugfix for A11)
	{0xA064, 0x4EC6}, 	// PIXREFTRIM0 (Bugfix for A11)
	{0xA068, 0x0600}, 	// PIXREFEN1 (Bugfix for A11)
	{0xA069, 0xF222}, 	// SENSREFI (Bugfix for A11 with Pixel-Interface-Buffer ADC )
	{0x9000, 0x0BC4}, 	// S00_EXPOTIME (50us @ 60.24 MHz)
	{0x9002, 0x00A0}, 	// S00_SENSORCFG (sensor_code_index = 0)
	{0x9003, 0x0000}, 	// S00_ROCFG (sensor_rotates = 0)
	{0x9004, 0x5D6A}, 	// S01_EXPOTIME (1000us @ 60.24 MHz)
	{0x9006, 0x00A0}, 	// S01_SENSORCFG (sensor_code_index = 0)
	{0x9007, 0xC000}, 	// S01_ROCFG (sensor_rotates = 0)
	{0x9100, 0x7AD4}, 	// S00_EXPOTIMEMAX (2000us @ 60.24 MHz)
	{0x9102, 0x00A0}, 	// S00_ILLUCFG  Goff (pll_index = 0, illu_code_index = 0)
	{0x9103, 0x7AD4}, 	// S01_EXPOTIMEMAX (2000us @ 60.24 MHz)
	{0x9105, 0x00A0}, 	// S01_ILLUCFG  Goff (pll_index = 0, illu_code_index = 0)
	{0x91C0, 0x0112}, 	// CSICFG (superframe enabled)
	{0x91C1, 0x2140}, 	// ROS
	{0x91C3, 0x0012}, 	// MODULECFG (NTC meas enabled [internal PU])
	{0x91CB, 0x0008}, 	// EXPCFG0
	{0x91CC, 0x0020}, 	// EXPCFG1
	{0x91CD, 0x0810}, 	// PSOUT (IRS9100 output, Gating disabled)
	{0x91D6, 0x0035}, 	// PADMODCFG (IRS9100 output)
	{0x91DF, 0x0102}, 	// MB_CFG_0 (MB0 = 2, LVDS0 active)
	{0x91E7, 0x061A}, 	// MB0_FRAMETIME_MIN (mb_framerate = 30.0 fps)
	{0x91EF, 0x0008}, 	// MBSEQ0_CFG0 (1x MB0)
	{0x9206, 0x1AA1}, 	// PLL_MODLUT0_CFG0 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x9207, 0xD70A}, 	// PLL_MODLUT0_CFG1 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x9208, 0x0003}, 	// PLL_MODLUT0_CFG2 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x9209, 0x0C01}, 	// PLL_MODLUT0_CFG3 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x920A, 0x0000}, 	// PLL_MODLUT0_CFG4 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x920B, 0x0362}, 	// PLL_MODLUT0_CFG5 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x920C, 0x0044}, 	// PLL_MODLUT0_CFG6 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x920D, 0x0030}, 	// PLL_MODLUT0_CFG7 (for f_illu = 60.24 MHz [f_mod = 722.88 MHz])
	{0x923C, 0x000B}, 	// SENSOR_LENGTH_CODE0 (codelength = 12, dc = 50.00 %)
	{0x923D, 0x0FC0}, 	// SENSOR_CODE0_0      (codelength = 12, dc = 50.00 %)
	{0x9260, 0x000B}, 	// ILLU_LENGTH_CODE0 (codelength = 12, dc = 33.33 %)
	{0x9261, 0x0F00}, 	// ILLU_CODE0_0      (codelength = 12, dc = 33.33 %)
	{0x9284, 0x0A01}, 	// DIGCLKDIV_S0_PLL0 (f_fsm = 60.24 MHz)
	{0x9294, 0x0A00}, 	// CHARGEPUMPDIV_S0_PLL0 (f_chargepump = 120.48 MHz)
	{0x92A4, 0x0902}, 	// DLLREGDELAY_S0_PLL0
	{0x935C, 0x0000}, 	// CRC (invalid)
	{0x935D, 0x00B3}, 	// ROIROWMAX (180 rows)
	{0x935E, 0x0000}, 	// ROIROWMIN (180 rows)
	{0x935F, 0x00EF}, 	// ROICOLMAX (240 columns)
	{0x9360, 0x0000}, 	// ROICOLMIN (240 columns)
	{0x9362, 0x061A}, 	// MB0_FRAMETIME (mb_framerate = 30.0 fps)
	{0x93DD, 0x0020}, 	// WPPIXREFEN1 (Bugfix for A11)
	{0x9401, 0x0002}, 	// SEQ_MODE (MBSEQ0, Embedded Data selection: App Std)
	{REG_NULL, 0x00},
};

//master
static struct regval irs2875_stream_on_setting[] = {
	{0x9400, 0x0001},	 /* stream on */
};
//master
static struct regval irs2875_stream_off_setting[] = {
	{0x9400, 0x0000},	 /* stream off */
};

static struct regval irs2875_slave_mode_stream_on_setting[] = {
	{0x9402, 0x0000},
	{0xA001, 0x0005},
	{0xA00A, 0x1A13},
	{0x9402, 0x0001},
};

static struct regval irs2875_slave_mode_stream_off_setting[] = {
	{0x9402, 0x0007},
	{REG_NULL, 100},
	{0xA001, 0x0000},
	{0x8609, 0xBB5B},
	{0x9402, 0x0000},
};

#ifdef __cplusplus
}
#endif

#endif  //UTILITY_SENSOR_INC_IRS2875_SETTING_H_
