/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_SENSOR_INC_ISX031STD_SETTING_H_
#define UTILITY_SENSOR_INC_ISX031STD_SETTING_H_

#define ISX031_VMAX_OFFSET	0x8a70
#define ISX031_VMAX			0x8a74
#define ISX031_DEF_VMAX		0x06d6


enum MODE_TYPE {
	HK_M24F217D4_S2R0T8E5,
	SENSING_M24F190D4_S0R0T7E5,  // H190X
	SENSING_M24F190D4_S2R0T8E5,  // H190XA
	WHETRON_M24F190D4_S2R0T8,
	SENSING_M24F190D4_S2R0T7E5,  // H190XA-1
    MODE_TYPE_MAX,
	MODE_TYPE_NUM,
};

// Sensor Diag Param begin
enum sony_subid {
	CAMERA_SONY_ERRB_ERROR = 0,
	CAMERA_SONY_AVDD_ERROR,
	CAMERA_SONY_DOVDD_ERROR,
	CAMERA_SONY_DVDD_ERROR,
	CAMERA_SONY_FCNT_ERROR,
	CAMERA_SONY_TEMP_ERROR,
	CAMERA_SONY_ROW_COLUMN_ID_ERROR,
	CAMERA_SONY_PLL_CLOCK_ERROR,
	CAMERA_SONY_RAM_CRC_1BIT_DATA_ERROR,
	CAMERA_SONY_RAM_CRC_2BIT_DATA_ERROR,
	CAMERA_SONY_RAM_CRC_1BIT_ADDRE_ERROR,
	CAMERA_SONY_ROM_CRC_1BIT_DATA_ERROR,
	CAMERA_SONY_ROM_CRC_2BIT_DATA_ERROR,
	CAMERA_SONY_ROM_CRC_1BIT_ADDRE_ERROR,
};

#define SONY_FAULT_0_REG		0x1E4C
#define SONY_FAULT_1_REG		0x1E4D
#define SONY_FAULT_2_REG		0x1E4E
#define SONY_FAULT_3_REG		0x1E4F

// Voltage Check Info
#define SONY_AVDD_VOLTAGE_REG	0x1E40
#define SONY_DOVDD_VOLTAGE_REG	0x1E42
#define SONY_DVDD_VOLTAGE_REG	0x1E44

#define SONY_AVDD_MAX_VOL		0x0E4C  // 3.66v
#define SONY_AVDD_MIN_VOL		0x0B7C  // 2.94v
#define SONY_DOVDD_MAX_VOL		0x0816  // 2.07v
#define SONY_DOVDD_MIN_VOL		0x05FA  // 1.53v
#define SONY_DVDD_MAX_VOL		0x0500  // 1.28v
#define SONY_DVDD_MIN_VOL		0x0398  // 0.92v

// Stream state
#define SONY_STREAM_STATE		0x6005
#define SONY_STREAMING_ON_VAL	0x05

// Frame Count Info
#define SONY_FCNT_REG_ADDR		0x7DC8
#define FPS_MONITOR_PERIOD_US	1000000

#define FCNT_WARN_RANGE			1
#define FCNT_ERR_RANGE			2

#define SHT_CTRL_UNIT_MIN		0xAC4C
#define SHT_MIN_UNIT_IN_USEC	0x03
#define SHT_MIN_UNIT_IN_FRAME	0x04

// Temperature Info
#define SONY_TEMP_SEN0_REG		0x1F40
#define SONY_MAX_TEMP_VALUE		120	 // 120
#define SONY_MIN_TEMP_VALUE		-35	 // -35

// fault inject reg info
#define SONY_ECM_UPDATE_REG		0x8A11

#define SONY_FAULT_INJECT_REG0	0xBF40
#define SONY_FAULT_INJECT_REG1	0xBF41
#define SONY_FAULT_INJECT_REG2	0xBF42
#define SONY_FAULT_INJECT_REG3	0xBF43

#define SONY_FAULT_CLEAR_REG0	0xBF48
#define SONY_FAULT_CLEAR_REG1	0xBF49
#define SONY_FAULT_CLEAR_REG2	0xBF4A
#define SONY_FAULT_CLEAR_REG3	0xBF4B

// fault notification mode select
uint32_t fault_notification_mode_setting[] = {
    0xBF3A, 0x55,  // avdd, row column, internal bus, register monitor set Mode 1
    0xBF3B, 0x11,  // Communication CRC, dovdd monitor set Mode 1
};
// Sensor Diag Param end

uint32_t isx031_trigger_shutter_mode_setting[] = {
	0x8AF0, 0x02,  // shutter trigger-based
	0xBF14, 0x02,  // SG_MODE_APL
};
uint32_t isx031_trigger_external_mode_setting[] = {
	0x8AF0, 0x01,  // shutter trigger-based  SG_MODE_ =
	0xBF14, 0x01,  // SG_MODE_APL
};

static uint32_t max96712_stream_on_setting[] = {
	0x040b, 0x82,  	// MIPI output enable
};

static uint32_t max96712_stream_off_setting[] = {
	0x040b, 0x00,  	// MIPI output enable
};

static uint32_t max9296_stream_on_setting[] = {
	0x0313, 0x42,
};

static uint32_t max9296_stream_off_setting[] = {
	0x0313, 0x00,
};

static uint32_t isx031_stream_on_setting[] = {
	0x8A01, 0x80,
};

static uint32_t isx031_stream_off_setting[] = {
	0x8A01, 0x00,
};

static uint32_t isx031_vmax_setting[] = {
	0x8A70, 0x0000,   // fps setting
};

static uint32_t isx031_pattern_mode_setting[] = {
	0xBE14, 0x01,  // DIF_PG_EN_
	0xBF60, 0x01,  // DIF_PG_EN_APL
};

#endif  // UTILITY_SENSOR_INC_ISX031STD_SETTING_H_
