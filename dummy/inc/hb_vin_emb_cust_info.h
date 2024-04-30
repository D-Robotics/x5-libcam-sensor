/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2020 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_VIN_EMB_CUST_INFO_H__
#define __HB_VIN_EMB_CUST_INFO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct embed_isx021_info_s {
	uint16_t DataCountInfo;// 1 2
	uint16_t chipid0;//13-14
	uint32_t chipid1;//17 18 19 20
	uint8_t HostSeletedRegs[10];//21 -30
	uint32_t SP1ExpTLineExtPulse;//31 33
	uint32_t SP2ExpTLineExtPulse;//34 36
	uint64_t frameCount; //37 -40
	uint8_t frameCountEnable; //41
	uint8_t FrameIDIndicate; // 42
	uint8_t FrameMaskEnable;//43
	uint8_t TRCTypeReplacementEnable;//43
	uint8_t HsyncSingleMaskEnable;//43
	uint8_t VsyncSingleMaskEnable;//43
	int16_t AdjLevelVsyncSignalExtFsync;//49 -50 CompensationLevelInExternalPulseBasedSync
	uint16_t TemperatureSensor0;//53 54
	uint16_t TemperatureSensor1;//55 56
	uint64_t SafetyReverse0; //61-64 safety
	uint16_t SafetyReverse1; //65- 66 safety
	uint8_t ExternalFsyncDetector;//99
	uint32_t SP1ExpTLineShutterTrigger;//101 103
	uint32_t SP2ExpTLineShutterTrigger;//104 106
	uint16_t AGainSP1H;
	uint16_t AGainSP1L;
	uint16_t AGainSP2H;
	uint16_t AGainSP2L;
	uint32_t DGain;//141 143
	uint16_t HCompenstationLevelOpticalCenter; //165 166
	uint16_t VCompenstationLevelOpticalCenter;  //168 167
	uint8_t HFlippingEnable; //169
	uint8_t VFlippingEnable; //170
	uint16_t HCropOffset; //181 182
	uint16_t VCropOffset; //183 184
	uint16_t HCropSize; // 185 186
	uint16_t VCropSize; // 187 188
	uint8_t DriveMode; //197
	uint8_t SyncMethod; //201
	uint8_t AutoModeEnabel; //202
	uint16_t DriveClkHorizontal;//205 206
	uint32_t DriveClkVertical; // 209 211
	uint8_t NumExtFrames; //213
	uint32_t NumExtLines;//214 216
	uint8_t CropEnable;//217
	uint8_t PGModeEnable;//218
	uint8_t HostCommunicationMaskEnable;//219
	uint8_t ReplacementDataTypeHostCommunicationMaskEnable;//219
	uint8_t HsyncSignalHostCommunicationMaskEnable;//219
	uint8_t VsyncSignalHostCommunicationMaskEnable;//219
	uint16_t TypeOfSensor;//225 226
	uint16_t chipid2;//226
	// uint16_t WBGainCF0; //249 250
	// uint16_t WBGainCF1; //251 252
	// uint16_t WBGainCF2; //253 254
	// uint16_t WBGainCF3; //255 256
	// uint32_t CompositinGainSP1CF0; //477 479
	// uint32_t CompositinGainSP1CF1; //480 472
	// uint32_t CompositinGainSP1CF2; //483 485
	// uint32_t CompositinGainSP1CF3; //486 488
	// uint32_t CompositinGainSP2CF0; //489 491
	// uint32_t CompositinGainSP2CF1; //492 494
	// uint32_t CompositinGainSP2CF2; //495 497
	// uint32_t CompositinGainSP2CF3; //498 500
} embed_isx021_info_t;

#ifdef __cplusplus
}
#endif

#endif
