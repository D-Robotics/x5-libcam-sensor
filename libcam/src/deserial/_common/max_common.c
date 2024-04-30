/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#include "./max_common.h"

#define LINK_MAX	4

struct serial_link_info {
      int8_t mode_num;         // The number of supported gmsl speed
      int8_t mode_dflt;        // The defult gmsl speed
      int8_t select[8];        // gmsl speed mode select
};

/* GMSL speed mode is  */
const struct serial_link_info serial_links[] = {
	[MAX9295A] = { 2, 0, {GMSL_MODE6, GMSL_MODE3}},
	[MAX96717] = { 2, 0, {GMSL_MODE6, GMSL_MODE3}},
	[MAX96717F] = { 1, 0, {GMSL_MODE3}},
};

static int32_t  deserial_get_emode_name(deserial_info_t *deserial_if, const char *emode_name[])
{
	int32_t i, ret = RET_OK;

	for (i = 0; i < LINK_MAX; i++) {
		if (deserial_if->sensor_info[i] != NULL) {
			emode_name[i] = SENSOR_EMODE_NAME(SENSOR_INFO(i));
			if (emode_name[i] == NULL) {
				vin_err("deserial %s link %d emode name is NULLL\n",
					deserial_if->deserial_name, i);
				return -1;
			}
		} else if (deserial_if->port_desp[i] != NULL) {
			emode_name[i] = deserial_if->port_desp[i];
		} else {
			emode_name[i] = NULL;
		}
	}
	return ret;
}
int32_t deserial_get_gmsl_speed(deserial_info_t *deserial_if, int8_t *gmsl_speed, int32_t link_num)
{
	int32_t ret = RET_OK, i, j;
	const char *emode_name[LINK_MAX];
	int32_t ser_type[LINK_MAX];
	int32_t gmsl_speed_index;

	// Get emode_name
	ret = deserial_get_emode_name(deserial_if, emode_name);
	if (ret < 0) {
		vin_err("deserial get emode name fail!!!\n");
		return ret;
	}
	// Parse ser_type
	for (i = 0; i < link_num; i++) {
		if (emode_name[i] != NULL) {
			ret = vin_string_parse(emode_name[i], EMODE_F_SER_TYPE);
			if (ret < 0) {
				vin_err("emode_name[%d] %s ser type parse fail!!!\n", i, emode_name[i]);
				return ret;
			}
			if (ret >= SER_TYPE_BUTT) {
				vin_err("Ser type is %d, morethen the maximum %d", ret, SER_TYPE_BUTT-1);
				return -1;
			}
			ser_type[i] = ret;
			ret = vin_string_parse(emode_name[i], EMODE_F_SER_LINK_SPEED);
			if (ret > 0) {
				gmsl_speed[i] = ret;
			} else if (ret == -FLAG_NOT_FIND) {
				gmsl_speed[i] = 0;
			} else {
				vin_err("ser gmsl speed parse fail!!!\n");
				return ret;
			}
		} else {
			ser_type[i] = -1;
			continue;
		}
		/* Emode name 'L' is not set, gmsl speed is defult value */
		if (gmsl_speed[i] == 0) {
			gmsl_speed_index = serial_links[ser_type[i]].mode_dflt;
			gmsl_speed[i] = serial_links[ser_type[i]].select[gmsl_speed_index];
			continue;
		}
		/* Emode name 'L' is set, to determine if gmsl speed is valid */
		for (j = 0; j < serial_links[ser_type[i]].mode_num; j ++) {
			if (gmsl_speed[i] == serial_links[ser_type[i]].select[j])
				break;
		}
		/* Gmsl speed is invalid return  */
		if (j >= serial_links[ser_type[i]].mode_num) {
			vin_err("the serial type %d don't support gmsl speed %dgbps",
				ser_type, gmsl_speed[i]);
			return -1;
		}
	}
	return RET_OK;
}

void deserial_change_pipe_vc(uint32_t *pdata, int32_t pdata_vc_idx, int32_t vc_num)
{
	int32_t vc_idx = pdata_vc_idx;
	int32_t i;

	for (i = 0; i < 4; i ++) {
		pdata[vc_idx] &= ~PIPE_VC_MASK;
		pdata[vc_idx] |= (vc_num << PIPE_VC_SHFIT);
		vc_idx += PIPE_VC_SETTING_SHFIT;
	}
}

int32_t deserial_get_dev_rev(deserial_info_t *deserial_if, device_info_t* dev_info) {
	int32_t ret = RET_OK;
	uint32_t bus = deserial_if->bus_num;
	uint8_t i2c_addr = deserial_if->deserial_addr;
	uint16_t reg_addr = dev_info->reg_addr;

	ret = vin_i2c_read_retry(bus, i2c_addr, REG16_VAL8, reg_addr);
	if (ret < 0) {
		dev_info->dev_rev = -1;
		vin_err("read deserial revision reg 0x%x fail!!!\n", reg_addr);
		return -1;
	}
	ret &= 0x0f;
	dev_info->dev_rev = ret;
	return RET_OK;
}
