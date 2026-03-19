/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2023 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)             "[sc132gsstd]:" fmt

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "hb_i2c.h"
#include "hb_cam_utility.h"
#include "inc/sc132gsstd_setting.h"
#include "inc/sensor_effect_common.h"
#include "hb_camera_data_config.h"
#include "hsmt_ns6607.h"

//#define AE_DBG

#define SC1320GS_PROGRAM_GAIN	(0x3e08)
#define SC132GS_DIGITAL_GAIN	(0x3e06)
#define SC132GS_EXP_LINE		(0x3e00)
#define SC132GS_DOL2_SHORT_EXP_LINE		(0x3e04)

#define DESER_I2C_ADDRESS 0x29  // deserializer I2C address

// Lock: Obtain i2 c bus operation permission
static pthread_mutex_t g_i2c_bus_mutex = PTHREAD_MUTEX_INITIALIZER;

static int i2c_bus_lock(void)
{
	int ret = pthread_mutex_lock(&g_i2c_bus_mutex);
	if (ret != 0) {
		vin_err("I2C bus lock failed! errno=%d\n", ret);
		return RET_ERROR;
	}
	return RET_OK;
}

// Unlock: Release i2c bus operation permission
static int i2c_bus_unlock(void)
{
	int ret = pthread_mutex_unlock(&g_i2c_bus_mutex);
	if (ret != 0) {
		vin_err("I2C bus unlock failed! errno=%d\n", ret);
		return RET_ERROR;
	}
	return RET_OK;
}

typedef struct serdes_ctx_s {
	int      fd;           // Fd opened by/dev/i2c-X
	uint8_t  local_addr;   // The I2C address of the deserializer is currently equal to DESER-I2C_DDRESS
	uint8_t  bus_id;       // SoC internal I2C bus number (passed to set_i2c_mandle)
	uint8_t  tx_map;       // The currently valid tx bitmap (such as 0x1/0x2/0x4/0x8)
	uint8_t  tx_num;       // The tx index (0~3) bound to the current sensor
} serdes_ctx_t;
static serdes_ctx_t g_serdes_ctx;

// turning data init
int sc132gsstd_linear_data_init_1088x1280(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint32_t  open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;

	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	if (sensor_info->sensor_mode == SLAVE_M)
		turning_data.mode = NORMAL_M;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
		sizeof(turning_data.sensor_name));

	// turning sensor_data
	// lines_per_second = fps * vts, vts = {16‘h320e,16’h320f} = 1400
	// If trigger is enabled, the configuration before trigger will still be used.
	turning_data.sensor_data.lines_per_second = 84000;
	// form customer, exposure time max = 10ms
	turning_data.sensor_data.exposure_time_max = 2560;

	turning_data.sensor_data.active_width = 1088;
	turning_data.sensor_data.active_height = 1280;
	turning_data.sensor_data.analog_gain_max = 154;		//154
	turning_data.sensor_data.digital_gain_max = 0;   //159
	turning_data.sensor_data.exposure_time_min = 1;
	// No setting is required in linear mode
	turning_data.sensor_data.exposure_time_long_max = 4000;

	// raw10
	sensor_data_bayer_fill(&turning_data.sensor_data, 10, (uint32_t)BAYER_START_B, (uint32_t)BAYER_PATTERN_RGGB);
	sensor_data_bits_fill(&turning_data.sensor_data, 12);

	turning_data.stream_ctrl.data_length = 1;
	if(sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc132gs_stream_on_setting)) {
		memcpy(stream_on, sc132gs_stream_on_setting, sizeof(sc132gs_stream_on_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if(sizeof(turning_data.stream_ctrl.stream_off) >= sizeof(sc132gs_stream_off_setting)) {
		memcpy(stream_off, sc132gs_stream_off_setting, sizeof(sc132gs_stream_off_setting));
	} else {
		vin_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	turning_data.normal.again_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.again_lut != NULL) {
		memset(turning_data.normal.again_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.again_lut, sc132gs_gain_lut,
			sizeof(sc132gs_gain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc132gs_gain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.again_lut[open_cnt], 2);
		}
	}

	turning_data.normal.dgain_lut = malloc(256*sizeof(uint32_t));
	if (turning_data.normal.dgain_lut != NULL) {
		memset(turning_data.normal.dgain_lut, 0xff, 256*sizeof(uint32_t));
		memcpy(turning_data.normal.dgain_lut, sc132gs_dgain_lut,
			sizeof(sc132gs_dgain_lut));
		for (open_cnt =0; open_cnt <
			sizeof(sc132gs_dgain_lut)/sizeof(uint32_t); open_cnt++) {
				// DOFFSET(&turning_data.normal.dgain_lut[open_cnt], 2);
		}
	}

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut) {
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut) {
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0) {
		vin_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

/*
 * Unified hsmt serdes send: one API for deserializer, serializer, and select-TX operations.
 * - Deserializer (RX init, mipi on/off, select_tx_sensor): tx_map=0, port_id=0, stream with opcode 4/2 only.
 * - Serializer (TX init, may mix local+remote): tx_map=channel bit, port_id=1 (user reg) or 5 (sensor port).
 * - Each packet: opcode 4 or 2 -> write local (deserializer); opcode 0 -> write remote (serializer).
 * For sensor register array after select-TX, use serdes_select_tx_and_write_sensor_array().
 *
 * NOTE: This variant is SerDes-context aware and uses serdes_ctx_t
 *       instead of raw (fd, local_addr) parameters.
 */
static int serdes_write_n_packets_mixed(serdes_ctx_t *ctx,
								 uint8_t tx_map, uint8_t port_id,
								 int buf_len, const uint8_t *cmds_buf,
								 int *fail_line_no, int *err_offset)
{
	int fd = ctx->fd;
	uint8_t local_addr = ctx->local_addr;
	int cur_offset = 0;
	unsigned short payload_len = 0;
	int ret = RET_PARAM_ERR;
	int retry = 0;
	int tx_num = 0;
	int failcode_len;
	int tmp = tx_map;
	uint8_t opcode;

	if (fail_line_no != NULL)
		*fail_line_no = 1;

	if (cmds_buf == NULL) {
		buf_len = 0;
		if (err_offset != NULL)
			*err_offset = 0;
		return RET_PARAM_ERR;
	}

	while (tmp) {
		tx_num++;
		tmp = tmp & (tmp - 1);
	}
	if (tx_num > 1)
		failcode_len = tx_num * 3 + 4;
	else
		failcode_len = 1;

	while (cur_offset < buf_len) {
		if (buf_len - cur_offset < 3) {
			ret = RET_PARAM_ERR;
			vin_err("write_n_packets_mixed: buf_len - cur_offset < 3, buf_len=%d cur_offset=%d\n",
				buf_len, cur_offset);
			break;
		}

		payload_len = (cmds_buf[cur_offset + 1] & 8)
			? (((cmds_buf[cur_offset + 2] & 0x7f) << 4) | (cmds_buf[cur_offset + 1] >> 4)) + 4
			: (cmds_buf[cur_offset + 1] >> 4) + 3;

		if (payload_len > MAX_PACKET_SIZE + 4 || payload_len + cur_offset + 1 > buf_len) {
			ret = RET_PARAM_ERR;
			vin_err("write_n_packets_mixed: payload_len invalid, payload_len=%u cur_offset=%d\n",
				(unsigned int)payload_len, cur_offset);
			break;
		}

		opcode = cmds_buf[cur_offset + 1] & 7;

		if (opcode == 4 || opcode == 2) {
			/* write local (deserializer) */
			ret = i2c_write_local_msg(fd, local_addr, payload_len + 1,
				&cmds_buf[cur_offset], 7, g_failcode_buf);
		} else if (opcode == 0 && tx_map != 0) {
			/* write remote (serializer / port) */
			ret = i2c_write_remote_msg(fd, local_addr, tx_map, port_id,
				payload_len + 1, &cmds_buf[cur_offset], failcode_len, g_failcode_buf);
		} else {
			vin_err("write_n_packets_mixed: unsupported opcode %u at offset %d (cmd_buf[1]=0x%02x)\n",
				(unsigned int)opcode, cur_offset, cmds_buf[cur_offset + 1]);
			ret = RET_PARAM_ERR;
			break;
		}

		if (ret == RET_OK) {
			cur_offset += payload_len + 1;
			retry = 0;
			if (fail_line_no != NULL)
				(*fail_line_no)++;
			if (tx_num == 1)
				failcode_len = 1;
		} else if (ret != RET_MSG_ERR || tx_num > 1 || retry >= RETRY_TIMES) {
			break;
		} else {
			if (++retry == RETRY_TIMES)
				failcode_len = 7;
		}
	}

	error_handling(ret, (ret == RET_MSG_ERR && failcode_len <= ACK_DATA_LEN) ? failcode_len : 7);
	if (err_offset != NULL)
		*err_offset = cur_offset;
	return ret;
}

/*
 * Select the TX channel then write sensor register array via I2C tunneling.
 * Unifies "select serializer for sensor" + vin_write_array so driver can do one call for sensor init/control.
 * select_buf/select_buf_len: Packet stream that selects which TX's I2C master is used (e.g. select_tx_sensor[tx_num]).
 *
 * NOTE: This variant is SerDes-context aware and uses serdes_ctx_t
 *       instead of raw (fd, local_addr) parameters.
 */
int serdes_select_tx_and_write_sensor_array(serdes_ctx_t *ctx,
	const uint8_t *select_buf, int select_buf_len,
	uint32_t bus, uint32_t sensor_i2c_addr, int reg_width, int setting_size, uint32_t *cam_setting)
{
	int fail_line_no;
	int ret = serdes_write_n_packets_mixed(ctx, 0, 0, select_buf_len, select_buf, &fail_line_no, NULL);
	if (ret != RET_OK) {
		vin_err("serdes_select_tx_and_write_sensor_array: select TX failed line %d\n", fail_line_no);
		return ret;
	}
	return vin_write_array(bus, sensor_i2c_addr, reg_width, setting_size, cam_setting);
}

static int sc132gsstd_poweroff(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;

	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
									sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -RET_ERROR;
				}
			}
		}
	}

	return ret;
}

static int sc132gsstd_poweron(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if(sensor_info->gpio_num > 0) {
		for(gpio = 0; gpio < sensor_info->gpio_num; gpio++) {
			if(sensor_info->gpio_pin[gpio] != -1) {
				ret = vin_power_ctrl(sensor_info->gpio_pin[gpio],
						sensor_info->gpio_level[gpio]);
				usleep(1 * 100 * 1000);  //100ms
				ret |= vin_power_ctrl(sensor_info->gpio_pin[gpio],
						1 - sensor_info->gpio_level[gpio]);
				if(ret < 0) {
					vin_err("vin_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
				usleep(100*1000);  //100ms
			}
		}
	}

	return ret;
}

static int sc132gsstd_init(sensor_info_t *sensor_info)
{
	int setting_size = 0;
	int channel = 0, i = 0, tx_map = 0xf, ret = RET_OK;
	int fail_line_no;
	const uint8_t *tx_id_array[4] = {SET_I2C0_TX0_ID, SET_I2C0_TX1_ID, SET_I2C0_TX2_ID, SET_I2C0_TX3_ID};
	const uint8_t *tx_init_array[4] = {tx0_init_txt_tunneling, tx1_init_txt_tunneling, tx2_init_txt_tunneling, tx3_init_txt_tunneling};
	int tx_init_size[4] = {sizeof(tx0_init_txt_tunneling), sizeof(tx1_init_txt_tunneling), sizeof(tx2_init_txt_tunneling), sizeof(tx3_init_txt_tunneling)};
	const uint8_t *select_tx_sensor[4] = {select_tx0_sensor, select_tx1_sensor, select_tx2_sensor, select_tx3_sensor};
	int vc_num = 0;  // virtual channel number
	int tx_num = 0;
	uint8_t local_addr = DESER_I2C_ADDRESS;
	uint8_t current_sensor_name[128] = {0};
	char serdes_i2c_bus[32] = {0};
	int fd = -1;
	int xfer_started = 0;

	ret = sc132gsstd_poweron(sensor_info);
	if (ret < 0) {
		vin_err("%d : sensor reset %s fail\n",
			__LINE__, sensor_info->sensor_name);
		goto end;
	}

	snprintf(serdes_i2c_bus, sizeof(serdes_i2c_bus), "/dev/i2c-%d", sensor_info->bus_num);
	fd = open(serdes_i2c_bus, O_RDWR);
	if (fd < 0)
	{
		vin_err("open i2c error, bus: %s\r\n", serdes_i2c_bus);
		ret = -RET_ERROR;
		goto end;
	}

	vc_num = sensor_info->extra_mode;  // vc0 bind to tx0, vc1 bind to tx1, vc2 bind to tx2
	tx_num = vc_num;  // tx_num is the number of tx channels to be used, 0 for tx0, 1 for tx1, 2 for tx2, 3 for tx3
	vin_info("Current use virtual channel %d.\n", vc_num);
	switch (vc_num) {
		case 0:
			tx_map = 0x1;  // tx0
			break;
		case 1:
			tx_map = 0x2;  // tx1
			break;
		case 2:
			tx_map = 0x4;  // tx2
			break;
		case 3:
			tx_map = 0x8;  // tx3
			break;
		default:
			vin_err("Invalid virtual channel %d\n", vc_num);
			ret = -RET_ERROR;
			goto end;
	}
	vin_info("Current tx_map is 0x%x\n", tx_map);

	/* Initialize SerDes context (single-SerDes scenario).
	 * In the future, this can be extended to support multiple SerDes
	 * by maintaining one ctx per (bus, local_addr) pair. */
	g_serdes_ctx.fd = fd;
	g_serdes_ctx.local_addr = local_addr;
	g_serdes_ctx.bus_id = (uint8_t)sensor_info->bus_num;
	g_serdes_ctx.tx_map = tx_map;
	g_serdes_ctx.tx_num = tx_num;

	// Call this method once at the beginning of a process.
	i2c_start_xfer();
	xfer_started = 1;

	// Initialize I2C handle for each local bus/local address combination after i2c_start_xfer().
	set_i2c_handle(g_serdes_ctx.fd, 1, local_addr, 0);

	// initialize rx ,deserial
	vin_info("start initial rx\r\n");
	if ((ret = serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(rx_init_4tx), rx_init_4tx, &fail_line_no, NULL)) != RET_OK)
	{
		// Users can handle the error according to their own preference.
		vin_err("initial rx error\r\n");
		goto end;
	}

	vin_info("rx finish\r\n");

	usleep(200 * 1000);  //200ms
	// check if the specified tx channels are locked
	for (channel = 0; channel < 4; channel++)
	{
		for(i = 0; i < 50; i++)
		{
			if (check_lock(g_serdes_ctx.fd, local_addr, channel) == RET_OK)
			{
				// tx_map |= (1 << channel);
				vin_info("channel %x lock\r\n",channel);
				break;
			}

			usleep(10000); // 10ms
		}

		if(i == 50)
		{
			vin_info("channel %x unlock\r\n",channel);
		}
	}

	// Set tx's hsmtid before tx initialization
	if (tx_map & (1 << tx_num))
	{
		vin_info("set_tx_hsmtid tx%d\r\n", tx_num);
		if ((ret = set_tx_hsmtid(g_serdes_ctx.fd, local_addr, sizeof(SET_I2C0_TX0_ID), tx_id_array[tx_num], g_failcode_buf)) != RET_OK)
		{
			tx_map &= ~(1 << tx_num);
			error_handling(ret, 7);
			// Users can handle the error according to their own preference.
		}
	}

	// initialize tx
	vin_info("start initial tx%d with tx_map 0x%x\r\n", tx_num, tx_map);
	if ((ret = serdes_write_n_packets_mixed(&g_serdes_ctx, tx_map, PORT_SER, tx_init_size[tx_num], tx_init_array[tx_num], &fail_line_no, NULL)) != RET_OK)
	{
		tx_map &= ~(1 << tx_num);
		// Users can handle the error according to their own preference.
		vin_info("initial tx%d error\r\n", tx_num);
		ret = -RET_ERROR;
	}
	else
	{
		vin_info("initial tx%d OK\r\n", tx_num);
	}

	if (tx_map == 0)
	{
		vin_info("Error!!! No tx is valid!\r\n");
		ret = -RET_ERROR;
		goto end;
	}

	vin_info("sc132gsstd config success under %d mode\n\n", sensor_info->sensor_mode);

	// Select TX and init sensor (write sensor array via I2C tunneling)
	vin_info("Via I2C TX%d sensor and init\n", tx_num);
	strncpy(current_sensor_name, sensor_info->sensor_name, sizeof(current_sensor_name) - 1);
	current_sensor_name[sizeof(current_sensor_name) - 1] = '\0';
	vin_info("current sensor is %s\n", current_sensor_name);
	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			vin_info("init %s in normal mode.\n", current_sensor_name);
			vin_info("sensor_info->sensor_addr is 0x%x\n", sensor_info->sensor_addr);
			setting_size = sizeof(sc132gs_linear_init_1088x1280_30fps_setting_master) / sizeof(uint32_t) / 2;
			ret = serdes_select_tx_and_write_sensor_array(&g_serdes_ctx,
				select_tx_sensor[tx_num], sizeof(select_tx0_sensor),
				sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size,
				sc132gs_linear_init_1088x1280_30fps_setting_master);
			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				goto end;
			}

			ret = sc132gsstd_linear_data_init_1088x1280(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				goto end;
			}
			break;

		case SLAVE_M:	  // 6:slave
			vin_info("init %s in slave mode\n", current_sensor_name);
			vin_info("sensor_info->sensor_addr is 0x%x\n", sensor_info->sensor_addr);
			setting_size = sizeof(sc132gs_linear_init_1088x1280_30fps_setting_slave) / sizeof(uint32_t) / 2;
			ret = serdes_select_tx_and_write_sensor_array(&g_serdes_ctx,
				select_tx_sensor[tx_num], sizeof(select_tx0_sensor),
				sensor_info->bus_num, sensor_info->sensor_addr, 2, setting_size,
				sc132gs_linear_init_1088x1280_30fps_setting_slave);

			if (ret < 0) {
				vin_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
				goto end;
			}

			ret = sc132gsstd_linear_data_init_1088x1280(sensor_info);
			if (ret < 0) {
				vin_err("%d : linear data init %s fail\n", __LINE__, sensor_info->sensor_name);
				goto end;
			}
			break;
		default:
			vin_err("not support mode %d\n", sensor_info->sensor_mode);
			ret = -RET_ERROR;
			goto end;
	}

	/* Success path: keep SerDes fd/xfer alive for start/stop/AE controls. */
	return 0;

end:
	/* Error path cleanup only */
	if (xfer_started) {
		i2c_stop_xfer();
	}
	if (fd >= 0) {
		close(fd);
	}
	g_serdes_ctx.fd = -1;
	return ret == RET_OK ? -RET_ERROR : ret;
}

// start stream
static int sc132gsstd_start(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t local_addr = DESER_I2C_ADDRESS;
	int fail_line_no ;
	int tx_map = 0xf;
	int vc_num = 0;  // virtual channel number
	int tx_num = 0;
	const uint8_t *select_tx_sensor[4] = {select_tx0_sensor, select_tx1_sensor, select_tx2_sensor, select_tx3_sensor};
	int i = 0;

	vin_info("%s: serdes_i2c_bus fd is  %d\n", __FUNCTION__, g_serdes_ctx.fd);
	if (g_serdes_ctx.fd < 0) {
		vin_err("%s: serdes ctx not initialized (fd=%d)\n", __FUNCTION__, g_serdes_ctx.fd);
		return -RET_ERROR;
	}

	vc_num = sensor_info->extra_mode;  // vc0 bind to tx0, vc1 bind to tx1, vc2 bind to tx2
	tx_num = vc_num;  // tx_num is the number of tx channels to be used, 0 for tx0, 1 for tx1, 2 for tx2, 3 for tx3
	vin_info("Current start frame on virtual channel %d.\n", vc_num);
	switch (vc_num) {
		case 0:
			tx_map = 0x1;  // tx0
			break;
		case 1:
			tx_map = 0x2;  // tx1
			break;
		case 2:
			tx_map = 0x4;  // tx2
			break;
		case 3:
			tx_map = 0x8;  // tx3
			break;
		default:
			vin_err("Invalid virtual channel %d\n", vc_num);
			goto end;
	}
	vin_info("Current start tx_map is 0x%x\n", tx_map);

	switch(sensor_info->sensor_mode) {
		case NORMAL_M:
			vin_info("Start sc132gsstd in normal mode.\n");
			// set mipi0 on
			vin_info("set mipi0 on\r\n");
			if ((ret = serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(mipi0_on), mipi0_on, &fail_line_no, NULL)) != RET_OK)
		{
			vin_err("set mipi0 on error!!!\r\n");
			return ret;
		}

		break;
	case DOL2_M:
		setting_size = sizeof(sc132gs_stream_on_setting)/sizeof(uint32_t)/2;
		vin_info("start hdr mode, sensor_name %s, setting_size = %d\n", sensor_info->sensor_name, setting_size);
		ret = vin_write_array(sensor_info->bus_num, sensor_info->sensor_addr, 2,
				setting_size, sc132gs_stream_on_setting);
		if(ret < 0) {
				vin_err("start %s fail\n", sensor_info->sensor_name);
				return ret;
		}
		break;

		case SLAVE_M:	  // 6: slave
		vin_info("Start sc132gsstd in slave mode\n");

		usleep(100 * 1000);  //100ms
			vin_info("set mipi0 on\r\n");
			if ((ret = serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(mipi0_on), mipi0_on, &fail_line_no, NULL)) != RET_OK)
		{
			vin_err("set mipi0 on error!!!\r\n");
			return ret;
		}
		break;

	default:
		vin_err("not support mode %d\n", sensor_info->sensor_mode);
		ret = -RET_ERROR;
		break;
	}

	return ret;

end:
	return -RET_ERROR;
}

static int sc132gsstd_stop(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0;
	uint8_t local_addr = DESER_I2C_ADDRESS;
	int fail_line_no ;
	int tx_map = 0xf;
	int vc_num = 0;  // virtual channel number
	int tx_num = 0;
	const uint8_t *select_tx_sensor[4] = {select_tx0_sensor, select_tx1_sensor, select_tx2_sensor, select_tx3_sensor};

	vin_info("%s: serdes_i2c_bus fd is  %d\n", __FUNCTION__, g_serdes_ctx.fd);
	if (g_serdes_ctx.fd < 0) {
		vin_err("%s: serdes ctx not initialized (fd=%d)\n", __FUNCTION__, g_serdes_ctx.fd);
		return -RET_ERROR;
	}

	vc_num = sensor_info->extra_mode;  // vc0 bind to tx0, vc1 bind to tx1, vc2 bind to tx2
	tx_num = vc_num;  // tx_num is the number of tx channels to be used, 0 for tx0, 1 for tx1, 2 for tx2, 3 for tx3
	vin_info("Current stop frame virtual channel %d.\n", vc_num);
	switch (vc_num) {
		case 0:
			tx_map = 0x1;  // tx0
			break;
		case 1:
			tx_map = 0x2;  // tx1
			break;
		case 2:
			tx_map = 0x4;  // tx2
			break;
		case 3:
			tx_map = 0x8;  // tx3
			break;
		default:
			vin_err("Invalid virtual channel %d\n", vc_num);
			goto end;
	}
	vin_info("Current stop tx_map is 0x%x\n", tx_map);

	// Select the serializer for sensor (Bulk)
	vin_info("Via I2C TX%d sensor\r\n", tx_num);
	if (serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(select_tx0_sensor), select_tx_sensor[tx_num], &fail_line_no, NULL) != RET_OK)
	{
		vin_err("Via I2C TX%d sensor error!\r\n", tx_num);
		goto end;
	}
	setting_size =
			sizeof(sc132gs_stream_off_setting) / sizeof(uint32_t) / 2;
	vin_info("sensor stop sensor_name %s, setting_size = %d\n",
			sensor_info->sensor_name, setting_size);
	ret = vin_write_array(sensor_info->bus_num,
							sensor_info->sensor_addr, 2,
							setting_size, sc132gs_stream_off_setting);
	if (ret < 0)
	{
			vin_err("start %s fail\n", sensor_info->sensor_name);
			return ret;
	}

	// set mipi0 off
	vin_info("set mipi0 off\r\n");
	if ((ret = serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(mipi0_off), mipi0_off, &fail_line_no, NULL)) != RET_OK)
	{
		vin_err("set mipi0 off error!!!\r\n");
		return ret;
	}

	return ret;

end:
	return -RET_ERROR;
}

static  int sc132gsstd_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;

	ret = sc132gsstd_poweroff(sensor_info);
	if (ret < 0)
	{
		vin_err("%d : deinit %s fail\n",
			__LINE__, sensor_info->sensor_name);
	}
	if (g_serdes_ctx.fd >= 0) {
		i2c_stop_xfer();
		close(g_serdes_ctx.fd);
		g_serdes_ctx.fd = -1;
	}
	return ret;
}

static int sc132gsstd_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{
#ifdef AE_DBG
	vin_info("%s %s mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, mode, gain_num, again[0], dgain[0]);
#endif
	const uint16_t AGAIN_LOW = 0x3e08;
	const uint16_t AGAIN_HIGH = 0x3e09;
	const uint16_t DGAIN_LOW = 0x3e06;
	const uint16_t DGAIN_HIGH = 0x3e07;
	char ana_gain = 0, ana_fine_gain = 0;
	char dig_gain = 0, dig_fine_gain = 0;
	int again_index = 0, dgain_index = 0;
	int vc_num = -1;
	int fail_line_no;
	const uint8_t *select_tx_sensor[4] = {select_tx0_sensor, select_tx1_sensor, select_tx2_sensor, select_tx3_sensor};
	int ret = RET_OK;

	if (!info) {
		vin_err("%s: info is NULL\n", __FUNCTION__);
		return RET_ERROR;
	}
	if (!again || !dgain || gain_num == 0) {
		vin_err("%s: invalid gain params, gain_num=%u\n", __FUNCTION__, gain_num);
		return RET_ERROR;
	}
	if (g_serdes_ctx.fd < 0) {
		vin_err("%s: serdes ctx not initialized (fd=%d)\n", __FUNCTION__, g_serdes_ctx.fd);
		return RET_ERROR;
	}

	/*
	 * NOTE:
	 *  VC/TX binding must be per-sensor, not a process-global runtime state.
	 */
	vc_num = (int)info->extra_mode;
	if (vc_num < 0 || vc_num > 3) {
		vin_err("Global vc_num is invalid! vc_num=%d\n", vc_num);
		return RET_ERROR;
	}
	vin_info("[%s](%s) vc_%d mode:%d gain_num:%d again[0]:%x, dgain[0]:%x\n", __FILE__, __FUNCTION__, vc_num, mode, gain_num, again[0], dgain[0]);

	// Select the serializer to control the sensor
	vin_info("Gain control: select TX%d sensor\r\n", vc_num);

	if (i2c_bus_lock() != RET_OK) {
		return RET_ERROR;
	}
	if (serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(select_tx0_sensor), select_tx_sensor[vc_num], &fail_line_no, NULL) != RET_OK)
	{
		vin_err("Gain control: select TX%d sensor error!\r\n", vc_num);
		i2c_bus_unlock();
		return RET_ERROR;
	}

	if ((mode == NORMAL_M) || (mode == SLAVE_M) || (mode == DOL2_M)) {
		if (again[0] >= sizeof(sc132gs_gain_lut)/sizeof(uint32_t))
			again_index = sizeof(sc132gs_gain_lut)/sizeof(uint32_t) - 1;
		else
			again_index = again[0];

		if (dgain[0] >= sizeof(sc132gs_dgain_lut)/sizeof(uint32_t))
			dgain_index = sizeof(sc132gs_dgain_lut)/sizeof(uint32_t) - 1;
		else
			dgain_index = dgain[0];

		ana_gain = (sc132gs_gain_lut[again_index] >> 8) & 0x000000FF;
		ana_fine_gain = sc132gs_gain_lut[again_index] & 0x000000FF;

		dig_gain = (sc132gs_dgain_lut[dgain_index] >> 8) & 0x000000FF;
		dig_fine_gain = sc132gs_dgain_lut[dgain_index] & 0x000000FF;
#ifdef AE_DBG
		vin_info("%s vc_%d again(0x3e08/0x3e09):%x,%x; dgain(0x3e06x3e07):%x,%x\n",
				__FUNCTION__, vc_num, ana_gain, ana_fine_gain, dig_gain, dig_fine_gain);
#endif
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_LOW, ana_gain);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN_HIGH, ana_fine_gain);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_LOW, dig_gain);
		vin_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN_HIGH, dig_fine_gain);
	} else	{
		vin_err(" unsupport mode %d\n", mode);
		ret = RET_ERROR;
	}
	i2c_bus_unlock();

	return ret;
}
static int sc132gs_ae_set(uint32_t bus, uint32_t addr, uint32_t line)
{
	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	const uint16_t S_EXP_LINE0 = 0x3e04;
	const uint16_t S_EXP_LINE1 = 0x3e05;
	char temp0 = 0, temp1 = 0, temp2 = 0;

	uint32_t sline = line;
	/*
	* NOTICE: trigger mode: sline = line(from isp)
	* from customer, exposure time max is 10ms，sline = exposure_time_max = 420
	*/
	if (sline >= 840)
		sline = 840;

	temp0 = (sline & 0xF000) >> 12;
	temp1 = (sline & 0xFF0) >> 4;
	temp2 = (sline & 0x0F) << 4;
	vin_i2c_write8(bus, 16, addr, EXP_LINE0, temp0);
	vin_i2c_write8(bus, 16, addr, EXP_LINE1, temp1);
	vin_i2c_write8(bus, 16, addr, EXP_LINE2, temp2);

#ifdef AE_DBG
	vin_info("%s sline = %d, 0x3e00 = %x, 0x3e01 = %x, 0x3e02 = %x \n",
		__FUNCTION__, sline, temp0, temp1, temp2);
#endif

	return 0;
}

#define SAMPLECNT 8
static int sc132gsstd_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
#ifdef AE_DBG
	vin_info(" line mode %d, --line %d , line_num:%d \n", mode, line[0], line_num);
#endif
	uint32_t val;
	int vc_num = -1;
	int fail_line_no;
	const uint8_t *select_tx_sensor[4] = {select_tx0_sensor, select_tx1_sensor, select_tx2_sensor, select_tx3_sensor};
	int ret = RET_OK;

	if (info == NULL || line == NULL) {
		vin_err("info or line is NULL!\n");
		return RET_ERROR;
	}
	if (g_serdes_ctx.fd < 0) {
		vin_err("%s: serdes ctx not initialized (fd=%d)\n", __FUNCTION__, g_serdes_ctx.fd);
		return RET_ERROR;
	}

	/*
	 * NOTE:
	 *  VC/TX binding must be per-sensor, not a process-global runtime state.
	 */
	vc_num = (int)info->extra_mode;
	if (vc_num < 0 || vc_num > 3) {
		vin_err("Global vc_num is invalid! vc_num=%d\n", vc_num);
		return RET_ERROR;
	}
	vin_info("vc_%d line mode %d, --line %d , line_num:%d \n", vc_num, mode, line[0], line_num);

	// Select the serializer to control the sensor
	vin_info("Expose control: select TX%d sensor\r\n", vc_num);

	if (i2c_bus_lock() != RET_OK) {
		return RET_ERROR;
	}
	if (serdes_write_n_packets_mixed(&g_serdes_ctx, 0, 0, sizeof(select_tx0_sensor), select_tx_sensor[vc_num], &fail_line_no, NULL) != RET_OK)
	{
		vin_err("Expose control: select TX%d sensor error!\r\n", vc_num);
		i2c_bus_unlock();
		return RET_ERROR;
	}

	if ((mode == NORMAL_M) || (mode == SLAVE_M)) {
		val = line[0];
		sc132gs_ae_set(info->bus_num, info->sensor_addr, val);
	} else if (mode == DOL2_M) {
		//todo
	} else {
		vin_err(" unsupported mode %d\n", mode);
		ret = RET_ERROR;
	}

	i2c_bus_unlock();

	return ret;
}

static int sc132gsstd_userspace_control(uint32_t port, uint32_t *enable)
{
	vin_info("enable userspace gain control and line control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	return 0;
}

#ifdef CAMERA_FRAMEWORK_HBN
SENSOR_MODULE_F(sc132gsstd, CAM_MODULE_FLAG_A16D8);
sensor_module_t sc132gsstd = {
		.module = SENSOR_MNAME(sc132gsstd),
#else
sensor_module_t sc132gsstd = {
		.module = "sc132gsstd",
#endif
		.init = sc132gsstd_init,
		.start = sc132gsstd_start,
		.stop = sc132gsstd_stop,
		.deinit = sc132gsstd_deinit,
		.power_on = sc132gsstd_poweron,
		.power_off = sc132gsstd_poweroff,
		.aexp_gain_control = sc132gsstd_aexp_gain_control,
		.aexp_line_control = sc132gsstd_aexp_line_control,
		.userspace_control = sc132gsstd_userspace_control,
};
