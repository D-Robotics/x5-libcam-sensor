/*
 *    COPYRIGHT NOTICE
 *   Copyright 2019 Horizon Robotics, Inc.
 *    All rights reserved.
 */

#ifndef UTILITY_SENSOR_INC_DS953_SETTING_H_
#define UTILITY_SENSOR_INC_DS953_SETTING_H_

#ifdef __cplusplus
extern "C" {
#endif

static uint32_t ds953_imx390_init_setting[] = {
	0x01, 0x03,	 // digital reset 0/1
	0x02, 0x33,	 // 4-lane, generator crc, i2c strap mode 1.8v
	0x0E, 0xBC,	 // gpio0/1 output enable
	0x33, 0x02,	 // enable two gpios for forward channel
};

static uint32_t ds953_ar0233_init_setting[] = {
	0x01, 0x03,	 // digital reset 0/1
	0x02, 0x33,	 // 4-lane, generator crc, i2c strap mode 1.8v
	0x0E, 0xBC,  // gpio0/1 output enable
	0x33, 0x02,  // enable two gpios for forward channel
};

static uint32_t ds953_ar0233_x3_init_setting[] = {
	0x01, 0x03,  // digital reset 0/1.
	0x00, 0x00,
	0x02, 0x73,  // 4-lane, Continuous Clock, gen crc, i2c 1.8v.
	0x32, 0x49,  // i2c pass-through, parity check.
	0x00, 0x00,
	0x39, 0x20,  // slave_id[0]=0x10.
	0x3A, 0xD0,  // slave_id[1]=0x68.
	0x41, 0x20,  // slave_id_alias[0]=0x10.
	0x42, 0xD0,  // slave_id_alias[1]=0x68.
	0x0e, 0x2D,  // gpio-dir: 3-i,2-i,1-o,0-i.
	0x0d, 0x00,  // gpio-val: 3-0,2-0,1-0,0-0.
	0x00, 0x00,  // reset: gpio1
	0x0d, 0x02,  // gpio-val: 3-0,2-0,1-1,0-0.
	0x00, 0x00,
};

static uint32_t ds953_dummy_init_setting[] = {
	0x01, 0x03,  // digital reset 0/1.
	0x00, 0x00,
	0x02, 0x73,  // 4-lane, Continuous Clock, gen crc, i2c 1.8v.
	0x32, 0x49,  // i2c pass-through, parity check.
	0x00, 0x00,
};

static uint32_t ds953_ix019_bypass_init_setting[] = {
	0x01, 0x03,	  // digital reset 0/1
	0x02, 0x33,	  // 2-lane, generator crc, i2c strap mode 1.8v
	// 0x0E, 0xBC,	// gpio0/1 output enable
	// 0x33, 0x02,	// enable two gpios for forward channel
};


#ifdef __cplusplus
}
#endif
#endif // UTILITY_SENSOR_INC_DS953_SETTING_H_
