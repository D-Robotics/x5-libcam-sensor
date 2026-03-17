/*
* Copyright [C] 2025 Norel Systems Limited
* All rights reserved.
* This file contains confidential and/or privileged information of Norel Systems Ltd.
* If you are not the intended recipient, please delete this file immediately.
* Any unauthorized copying, disclosure or distribution of the information in this file is strictly forbidden.
* The software is provided on an "AS IS" basis, without warranties of any kind, either expressed or implied.
*/

#define pr_fmt(fmt)             "[hsmt_ns6607]:" fmt
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
#include "hsmt_ns6607.h"


struct SHARED_ADDR_ATTR
{
	uint16_t cur_id;
};


struct I2C_LOCAL_ADDR{
	struct SHARED_ADDR_ATTR * addr_attr;
	char shm_name[sizeof(SHARED_ATTR_NAME) + 4];
	int shmfd;
	uint8_t local_addr;
	uint8_t is_i2c1;   // is_i2c1 = 2  means SPI
	char sem_name[sizeof(SEM_NAME) + 4];
	sem_t * semfd;
};


struct I2C_LOCAL_BUS_PARAM{
	int fd;
	struct I2C_LOCAL_ADDR local_addrs[I2C_LOCAL_ADDR_NUM];
};

// I2C_BUS_NUM = number of i2c buses(host/master) for the SOC.
struct I2C_LOCAL_BUS_PARAM g_devs[TOTAL_BUS_NUM];   // 1 for SPI, the SPI slave number must <= 4
uint8_t g_ack_buf[MAX_PACKET_SIZE + 5] = {0};
double g_spi_timeout = 2; // default 2s
uint8_t g_failcode_buf[ACK_DATA_LEN] = {0};

static inline int max(int a, int b) {
    return a > b ? a : b;
}

uint16_t look_table_crc16(const uint8_t * data, int len)
{
	int i;
	uint16_t crc = 0xffff;
	uint8_t index;
	for(i = 0; i < len; i++)
	{
		index = (uint8_t)((crc & 0xff) ^ data[i]);
		crc >>= 8;
		crc ^= crc_table[index];
	}

	return crc;
}

void print_stack_trace() {
	void *buffer[100];
	int size = backtrace(buffer, 100);
	char **symbols = backtrace_symbols(buffer, size);
	for (int i = 0; i < size; i++) {
		vin_err("%s\n", symbols[i]);
	}
	free(symbols);
}

unsigned int calc_crc(unsigned int width, unsigned int  poly, unsigned int initial,
					const uint8_t *buffer,  unsigned int size_bits)
{
	unsigned int i;
	unsigned int crc = initial;

	for(i=0; i< size_bits; i++)
	{
		unsigned int bit = (((uint8_t*)buffer)[i >> 3] >> (i & 0x07)) & 1;

		unsigned int last = (crc >> (width - 1)) & 1;

		bit = bit ^ last;

		crc <<= 1;

		if(bit)
			crc ^= poly;
	}
	return width >=32 ? crc : (crc & ((1 << width) - 1));
}
int find_i2c_handle(int fd, uint8_t local_addr)
{
	int i, j;

	for (i = 0; i < TOTAL_BUS_NUM; i++)
	{
		if (g_devs[i].fd == fd)
		{
			for (j = 0; j < I2C_LOCAL_ADDR_NUM; j++)
			{
				if (g_devs[i].local_addrs[j].local_addr == local_addr)
				{
					return i * I2C_LOCAL_ADDR_NUM + j;
				}
			}
		}
	}

	return -1;
}

// flags: 0 = write  1 = read
int send_i2c_xfer_data(int fd, uint8_t local_addr, uint16_t flags, uint16_t cmd_len, const uint8_t *cmd_buf)
{
	struct i2c_rdwr_ioctl_data hsmt_msg;
	int ret = 0;

	if (find_i2c_handle(fd, local_addr) == -1)
	{
		vin_err("send_i2c_xfer_data failed, fd = %d, local_addr = 0x%02x\r\n", fd, local_addr);
		return RET_PARAM_ERR;
	}

	hsmt_msg.nmsgs = 1;
	hsmt_msg.msgs = (struct i2c_msg*)malloc(hsmt_msg.nmsgs * sizeof(struct i2c_msg));
	if (!hsmt_msg.msgs)
	{
		vin_err("send_i2c_xfer_data failed, malloc failed\r\n");
		return RET_MEM_ERR;
	}

	// write cmd to i2c
	hsmt_msg.msgs[0].addr = local_addr; //i2c slave address
	hsmt_msg.msgs[0].flags = flags;     //write
	hsmt_msg.msgs[0].len = cmd_len;
	hsmt_msg.msgs[0].buf = (uint8_t *)cmd_buf;

	ret = ioctl(fd, I2C_RDWR, (unsigned long)&hsmt_msg);
	free(hsmt_msg.msgs);
	return (ret < 0) ? RET_IO_ERR : RET_OK;
}

int i2c_write_cmd(int fd, uint8_t local_addr, int buf_len, const uint8_t *cmd_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret;
	uint16_t payload_len;

	if (fd <= 0 || cmd_buf == NULL || buf_len < 3 || failcode_len > ACK_DATA_LEN || failcode_len < 1 || failcode_buf == NULL)
	{
		vin_err("i2c_write_cmd failed,fd = %d, cmd_buf = %p, buf_len = %d, failcode_len = %d\r\n", fd, cmd_buf, buf_len, failcode_len);
		return RET_PARAM_ERR;
	}

	payload_len = (cmd_buf[1] & 8) ? (((cmd_buf[2] & 0x7f) << 4) | (cmd_buf[1] >> 4)) + 4 : (cmd_buf[1] >> 4) + 3;
	if (payload_len + 1 > buf_len || payload_len > MAX_PACKET_SIZE + 4)
	{
		vin_err("i2c_write_cmd failed, payload_len = %d, buf_len = %d\r\n", payload_len, buf_len);
		return RET_PARAM_ERR;
	}

	// I2C
	ret = send_i2c_xfer_data(fd, local_addr, 0, payload_len, &cmd_buf[1]);


	if (ret == RET_OK)
	{
		ret = send_i2c_xfer_data(fd, local_addr, 1, failcode_len, failcode_buf);

		if (ret == RET_OK && failcode_buf[0] != 0xEA)
		{
			vin_err("i2c_write_cmd failed, failcode_buf[0] = 0x%02x\r\n", failcode_buf[0]);
			vin_err("i2c_write_cmd failed, failcode_buf[1] = 0x%02x\r\n", failcode_buf[1]);
			vin_err("i2c_write_cmd failed, failcode_buf[2] = 0x%02x\r\n", failcode_buf[2]);
			vin_err("i2c_write_cmd failed, failcode_buf[3] = 0x%02x\r\n", failcode_buf[3]);
			vin_err("i2c_write_cmd failed, failcode_buf[4] = 0x%02x\r\n", failcode_buf[4]);
			// getchar(); // wait for user input to see the error
			return RET_MSG_ERR;
		}
	}

	return ret;
}

uint16_t calc_crc16(const uint8_t *buffer, unsigned int size)
{
	return look_table_crc16(buffer, size);
}
int lock_i2c(int fd, uint8_t local_addr)
{
	int i = find_i2c_handle(fd, local_addr);
	if (i != -1 && g_devs[i / I2C_LOCAL_ADDR_NUM].local_addrs[i % I2C_LOCAL_ADDR_NUM].semfd != NULL)
	{
		sem_wait(g_devs[i / I2C_LOCAL_ADDR_NUM].local_addrs[i % I2C_LOCAL_ADDR_NUM].semfd);
	}
	else
	{
		vin_err("lock_i2c fd = %d, local_addr = 0x%02x\r\n", fd, local_addr);
		print_stack_trace();
		return RET_LOCK_ERR;
	}

	return RET_OK;
}

int i2c_read_cmd(int fd, uint8_t local_addr, int cmd_len, const uint8_t *cmd_buf, int read_len, int failcode_len, uint8_t *failcode_buf)
{
	int ret;
	uint16_t payload_len, crc16;

	if (fd <= 0 || cmd_buf == NULL || cmd_len < 3 || read_len < 5 || ((unsigned int)read_len > sizeof(g_ack_buf)))
	{
		return RET_PARAM_ERR;
	}

	payload_len = (cmd_buf[1] & 8) ? (((cmd_buf[2] & 0x7f) << 4) | (cmd_buf[1] >> 4)) + 4 : (cmd_buf[1] >> 4) + 3;
	if (payload_len + 1 > cmd_len)
	{
		return RET_PARAM_ERR;
	}


	ret = send_i2c_xfer_data(fd, local_addr, 0, payload_len, &cmd_buf[1]);


	if (ret == RET_OK)
	{

		ret = send_i2c_xfer_data(fd, local_addr, 1, max(read_len, failcode_len), g_ack_buf);

		crc16 = calc_crc16(g_ack_buf, read_len - 2);
		if (ret == RET_OK && (g_ack_buf[0] != 0xE5 || (g_ack_buf[1] & 7) != 2
			|| ((g_ack_buf[1] & 8) ? (((g_ack_buf[2] & 0x7f) << 4) | (g_ack_buf[1] >> 4)) + 5 : (g_ack_buf[1] >> 4) + 4) != read_len
			|| ((g_ack_buf[read_len - 2] << 8) | g_ack_buf[read_len - 1]) != crc16))
		{
			memcpy(failcode_buf, g_ack_buf, failcode_len);
			return RET_MSG_ERR;
		}
	}

	return ret;
}

int unlock_i2c(int fd, uint8_t local_addr)
{
	int i = find_i2c_handle(fd, local_addr);
	if (i != -1 && g_devs[i / I2C_LOCAL_ADDR_NUM].local_addrs[i % I2C_LOCAL_ADDR_NUM].semfd != NULL)
	{
		sem_post(g_devs[i / I2C_LOCAL_ADDR_NUM].local_addrs[i % I2C_LOCAL_ADDR_NUM].semfd);
	}
	else
	{
		return RET_LOCK_ERR;
	}

	return RET_OK;
}

int i2c_read_local_reg16(int fd, uint8_t local_addr, int cmd_buf_len, const uint8_t *cmd_buf, uint16_t *reg16_data, int failcode_len, uint8_t *failcode_buf)
{
	int ret;

	if (cmd_buf == NULL || cmd_buf_len < 3)
	{
		vin_err("cmd_buf_len is %d\r\n", cmd_buf_len);
		return RET_PARAM_ERR;
	}

	if ((cmd_buf[1] & 7) != 5)  // local reg read
	{
		vin_err("(cmd_buf[1] & 7) is %d\r\n", cmd_buf[1] & 7);
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		vin_err("lock_i2c failed, ret = %d\r\n", ret);
		return ret;
	}

	if ((ret = i2c_read_cmd(fd, local_addr, cmd_buf_len, cmd_buf, 6, failcode_len, failcode_buf)) == RET_OK)
	{
		*reg16_data = g_ack_buf[2] | (g_ack_buf[3] << 8);
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

int check_lock(int fd, uint8_t local_addr, int tx)
{
	unsigned short reg_data;

	if (tx > 3 || tx < 0)
	{
		return -1;
	}

	if (i2c_read_local_reg16(fd, local_addr, sizeof(READ_CHANNEL_LOCK[0]), READ_CHANNEL_LOCK[tx], &reg_data, 7, g_failcode_buf) == RET_OK
		&& (reg_data & 0x40) == 0)   // lock
	{
		return 0;
	}

	return -1;
}

void error_handling(int ret, int failcode_len)
{
	int i;

	if (ret == RET_IO_ERR || ret == RET_MEM_ERR)
	{
		vin_err("ret = %d, fatal error\r\n", ret);
	}
	else if (ret == RET_PARAM_ERR)
	{
		vin_err("ret = %d, parameter error, user shall check if input parameters are correct.\r\n", ret);
	}
	else if (ret == RET_MSG_ERR)
	{
		printf("failcode: ");
		for (i = 0; i < failcode_len; i++)
		{
			printf("%x ", g_failcode_buf[i]);
		}

		printf("\r\n");
	}
}

int create_shm(int i, int j, uint8_t bus, uint8_t local_addr, uint8_t is_i2c1)
{
	sprintf(g_devs[i].local_addrs[j].shm_name, "%s%d%02x", SHARED_ATTR_NAME, bus, local_addr);
	g_devs[i].local_addrs[j].shmfd = shm_open(g_devs[i].local_addrs[j].shm_name, O_RDWR | O_CREAT, 0644);
	if (g_devs[i].local_addrs[j].shmfd == -1)
	{
		return RET_SHM_ERR;
	}

	ftruncate(g_devs[i].local_addrs[j].shmfd, sizeof(struct SHARED_ADDR_ATTR));
	g_devs[i].local_addrs[j].addr_attr = (struct SHARED_ADDR_ATTR *)mmap(NULL, sizeof(struct SHARED_ADDR_ATTR), PROT_READ | PROT_WRITE, MAP_SHARED, g_devs[i].local_addrs[j].shmfd, 0);
	if (g_devs[i].local_addrs[j].addr_attr == NULL)
	{
		close(g_devs[i].local_addrs[j].shmfd);
		shm_unlink(g_devs[i].local_addrs[j].shm_name);
		return RET_SHM_ERR;
	}

	sprintf(g_devs[i].local_addrs[j].sem_name, "%s%d%02x", SEM_NAME, bus, local_addr);
	g_devs[i].local_addrs[j].semfd = sem_open(g_devs[i].local_addrs[j].sem_name, O_CREAT, 0644, 1);
	if (g_devs[i].local_addrs[j].semfd == SEM_FAILED)
	{
		munmap(g_devs[i].local_addrs[j].addr_attr, sizeof(struct SHARED_ADDR_ATTR));
		close(g_devs[i].local_addrs[j].shmfd);
		shm_unlink(g_devs[i].local_addrs[j].shm_name);
		g_devs[i].local_addrs[j].addr_attr = NULL;
		return RET_LOCK_ERR;
	}

	sem_wait(g_devs[i].local_addrs[j].semfd);
	g_devs[i].local_addrs[j].addr_attr->cur_id = 0;
	sem_post(g_devs[i].local_addrs[j].semfd);

	g_devs[i].local_addrs[j].local_addr = local_addr;
	g_devs[i].local_addrs[j].is_i2c1 = is_i2c1;
	return RET_OK;
}

int set_i2c_handle(int fd, uint8_t bus, uint8_t local_addr, uint8_t is_i2c1)
{
	int i, j;

	if (I2C_LOCAL_BUS_NUM < 1 || fd <= 0 || local_addr == 0 || (is_i2c1 != 1 && is_i2c1 != 0))
	{
		return RET_PARAM_ERR;
	}

	for (i = 0; i < I2C_LOCAL_BUS_NUM; i++)
	{
		if (g_devs[i].fd == fd || g_devs[i].fd == -1)
		{
			g_devs[i].fd = fd;
			for (j = 0; j < I2C_LOCAL_ADDR_NUM; j++)
			{
				if (g_devs[i].local_addrs[j].local_addr == local_addr)
				{
					return RET_OK;
				}
				else if (g_devs[i].local_addrs[j].local_addr == 0 && g_devs[i].local_addrs[j].addr_attr == NULL)
				{
					return create_shm(i, j, bus, local_addr, is_i2c1);
				}
			}

			break;
		}
	}

	return RET_PARAM_ERR;
}

int set_spi_handle(int fd, uint8_t bus, uint8_t cs_pin)
{
	int i = I2C_LOCAL_BUS_NUM;
	int j;

	if (g_devs[i].fd == fd || g_devs[i].fd == -1)
	{
		g_devs[i].fd = fd;
		for (j = 0; j < I2C_LOCAL_ADDR_NUM; j++)
		{
			if (g_devs[i].local_addrs[j].local_addr == cs_pin)
			{
				return RET_OK;
			}
			else if (g_devs[i].local_addrs[j].local_addr == 0 && g_devs[i].local_addrs[j].addr_attr == NULL)
			{
				return create_shm(i, j, bus, cs_pin, 2);
			}
		}
	}

	return RET_PARAM_ERR;
}

void i2c_start_xfer(void)
{
	int i;
	for (i = 0; i < TOTAL_BUS_NUM; i++)
	{
		g_devs[i].fd = -1;
		memset(g_devs[i].local_addrs, 0, sizeof(g_devs[i].local_addrs));
	}
}

void i2c_stop_xfer(void)
{
	int i, j;

	for (i = 0; i < TOTAL_BUS_NUM; i++)
	{
		if (g_devs[i].fd != -1)
		{
			for (j = 0; j < I2C_LOCAL_ADDR_NUM; j++)
			{
				if (g_devs[i].local_addrs[j].addr_attr != NULL)
				{
					munmap(g_devs[i].local_addrs[j].addr_attr, sizeof(struct SHARED_ADDR_ATTR));
					close(g_devs[i].local_addrs[j].shmfd);
					shm_unlink(g_devs[i].local_addrs[j].shm_name);
					g_devs[i].local_addrs[j].addr_attr = NULL;
				}

				if (g_devs[i].local_addrs[j].semfd != NULL)
				{
					sem_close(g_devs[i].local_addrs[j].semfd);
					sem_unlink(g_devs[i].local_addrs[j].sem_name);
					g_devs[i].local_addrs[j].semfd = NULL;
				}
			}

			g_devs[i].fd = -1;
		}
	}
}

int set_destination(int fd, uint8_t local_addr, uint8_t tx_map, uint8_t port_id, int failcode_len, uint8_t *failcode_buf)
{
	const uint8_t (*sel_buf)[32] = NULL;
	int cur_index, sub_index, index, ret;

	/*
	* port_id must be one of the well-defined HSMT ports:
	*  - PORT_STD    : Standard remote device (HSMT std registers)
	*  - PORT_SER    : Serializer device
	*  - PORT_SENSOR : Remote I2C master (sensor side)
	*  - PORT_SPI    : Remote SPI device
	*/
	if (tx_map == 0 || tx_map > 15 ||
	(port_id != PORT_SENSOR && port_id != PORT_SER &&
		port_id != PORT_STD && port_id != PORT_SPI))
	{
		vin_info("RET_PARAM_ERR %d %s\r\n", __LINE__, __func__);
		return RET_PARAM_ERR;
	}

	if ((sub_index = find_i2c_handle(fd, local_addr)) == -1)
	{
		vin_info("RET_PARAM_ERR %d %s\r\n", __LINE__, __func__);
		return RET_PARAM_ERR;
	}

	cur_index = sub_index / I2C_LOCAL_ADDR_NUM;
	sub_index = sub_index % I2C_LOCAL_ADDR_NUM;
	index = g_devs[cur_index].local_addrs[sub_index].is_i2c1;   // 0: i2c

	if (tx_map != 0 && ((tx_map << 8) | port_id) != g_devs[cur_index].local_addrs[sub_index].addr_attr->cur_id)
	{
		if (port_id == PORT_SER)  // serializer control port
		{
			sel_buf = &CHANGE_SER_CMD[index][tx_map - 1];
		}
		else if (port_id == PORT_SENSOR)
		{
			sel_buf = &CHANGE_SENSOR_CMD[index][tx_map - 1];
		}
		else if (port_id == PORT_SPI)  // only use i2c now
		{
			vin_warn("RET_PARAM_ERR %d %s, SPI port is not supported yet\r\n", __LINE__, __func__);
			// sel_buf = &CHANGE_SPI_CMD[0][tx_map - 1];
		}
		else // port_id == PORT_STD
		{
			sel_buf = &CHANGE_STD_CMD[index][tx_map - 1];
		}

		if ((ret = i2c_write_cmd(fd, local_addr, sizeof(sel_buf[0]), (const uint8_t *)&(*sel_buf)[0], failcode_len, failcode_buf)) != RET_OK)
		{
			g_devs[cur_index].local_addrs[sub_index].addr_attr->cur_id = 0;
			return ret;
		}

		g_devs[cur_index].local_addrs[sub_index].addr_attr->cur_id = (tx_map << 8) | port_id;
	}

	return RET_OK;
}


int i2c_write_local_msg(int fd, uint8_t local_addr, int cmd_buf_len, const uint8_t *cmd_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret;
	uint8_t opcode;

	if (cmd_buf == NULL) {
		vin_err("cmd_buf == NULL\r\n");
		return RET_PARAM_ERR;
	} else if (cmd_buf_len < 3)	{
		vin_err("cmd_buf_len < 3, is %d\r\n", cmd_buf_len);
		return RET_PARAM_ERR;
	}

	opcode = cmd_buf[1] & 7;
	// vin_info("current cmd_buf is 0x%x\r\n", cmd_buf[1]);
	if (opcode != 4 && opcode != 2) // local reg write or hsmt reg write
	{
		vin_err("opcode != 4 && opcode != 2, cmd_buf[1] is 0x%x,opcode = %d\r\n", cmd_buf[1], opcode);
		for (int i = 0; i < cmd_buf_len; i++)
		{
			vin_err("cmd_buf[%d] = 0x%x\r\n", i, cmd_buf[i]);
		}
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		vin_err("lock_i2c failed, ret = %d\r\n", ret);
		return ret;
	}

	ret = i2c_write_cmd(fd, local_addr, cmd_buf_len, cmd_buf, failcode_len, failcode_buf);

	unlock_i2c(fd, local_addr);
	return ret;
}

int i2c_write_remote_msg(int fd, uint8_t local_addr, uint8_t tx_map, uint8_t port_id, int cmd_buf_len, const uint8_t *cmd_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret;
	if (cmd_buf == NULL || cmd_buf_len < 3)
	{
		vin_info("RET_PARAM_ERR %d %s\r\n", __LINE__, __func__);
		return RET_PARAM_ERR;
	}

	if ((cmd_buf[1] & 7) != 0) // remote write
	{
		vin_info("RET_PARAM_ERR %d %s cmd_buf[1] = 0x%x\r\n", __LINE__, __func__, cmd_buf[1]);
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		return ret;
	}

	if ((ret = set_destination(fd, local_addr, tx_map, port_id, failcode_len, failcode_buf)) != RET_OK)
	{
		unlock_i2c(fd, local_addr);
		return ret;
	}

	ret = i2c_write_cmd(fd, local_addr, cmd_buf_len, cmd_buf, failcode_len, failcode_buf);

	unlock_i2c(fd, local_addr);
	return ret;
}

int i2c_write_remote_reg_data16(int fd, uint8_t local_addr, uint8_t tx_map, uint32_t reg_addr, uint16_t reg16_data, int failcode_len, uint8_t *failcode_buf)
{
	uint8_t cmd_buf[16];
	uint8_t i = 0;
	uint16_t crc16 = 0;
	uint16_t reg_base_addr = reg_addr >> 16;
	uint16_t reg_offset_addr = reg_addr & 0xFFFF;

	cmd_buf[i++] = 0x52;
	cmd_buf[i++] = 0x50;
	cmd_buf[i++] = ((reg_base_addr & 0x03) << 6) | (0x00 << 3);
	cmd_buf[i++] = ((reg_base_addr & 0xFC) >> 2) | ((reg_offset_addr & 0x03) << 6);
	cmd_buf[i++] = ((reg_offset_addr & 0x3FC) >> 2);
	cmd_buf[i++] = LSB(reg16_data);
	cmd_buf[i++] = MSB(reg16_data);

	crc16 = calc_crc16(cmd_buf+1, i-1);
	cmd_buf[i++] = MSB(crc16);
cmd_buf[i++] = LSB(crc16);

/* Remote write to serializer registers (PORT_SER). */
return i2c_write_remote_msg(fd, local_addr, tx_map, PORT_SER, i, cmd_buf, failcode_len, failcode_buf);
}

int write_n_msgs(int fd, uint8_t local_addr, int buf_len, const uint8_t* cmds_buf, int* fail_line_no) {
	int cur_offset = 0;
	unsigned short payload_len = 0;
	int ret = RET_PARAM_ERR;
	int retry = 0;
	if (fail_line_no != NULL) {
		*fail_line_no = 1;
	}

	if (cmds_buf == NULL) {
		ret = RET_PARAM_ERR;
		buf_len = 0;
	}

	while (cur_offset < buf_len) {
		if (buf_len - cur_offset < 3) {
			ret = RET_PARAM_ERR;
			vin_err("buf_len - cur_offset < 3, buf_len = %d, cur_offset = %d\r\n", buf_len, cur_offset);
			break;
		}

		payload_len = (cmds_buf[cur_offset + 1] & 8) ? (((cmds_buf[cur_offset + 2] & 0x7f) << 4) |
						(cmds_buf[cur_offset + 1] >> 4)) + 4 : (cmds_buf[cur_offset + 1] >> 4) + 3;
		if (payload_len > MAX_PACKET_SIZE + 4 || payload_len + cur_offset + 1 > buf_len) {
			ret = RET_PARAM_ERR;
			vin_err("payload_len > MAX_PACKET_SIZE + 4, payload_len = %d, cur_offset = %d\r\n", payload_len, cur_offset);
			break;
		}


		ret = i2c_write_local_msg(fd, local_addr, payload_len + 1, &cmds_buf[cur_offset], 7, g_failcode_buf);

		if (ret == RET_OK) {
			cur_offset += payload_len + 1;
			retry = 0;
			if (fail_line_no != NULL) {
				(*fail_line_no)++;
			}
		} else if (ret != RET_MSG_ERR || retry++ >= RETRY_TIMES) {
			break;
		}
	}

	error_handling(ret, 7);
	return ret;
}

int i2c_read_remote_reg16(int fd, uint8_t local_addr, uint8_t tx_map, int cmd_buf_len, const uint8_t *cmd_buf, uint16_t *reg16_data, int failcode_len, uint8_t *failcode_buf)
{
	int ret;

	if (cmd_buf == NULL || cmd_buf_len < 3)
	{
		return RET_PARAM_ERR;
	}

	if ((cmd_buf[1] & 7) != 1)  // remote read
	{
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		return ret;
	}

	/* Remote read from serializer registers (PORT_SER). */
	if ((ret = set_destination(fd, local_addr, tx_map, PORT_SER, failcode_len, failcode_buf)) != RET_OK)
	{
		unlock_i2c(fd, local_addr);
		return ret;
	}

	if ((ret = i2c_read_cmd(fd, local_addr, cmd_buf_len, cmd_buf, 6, failcode_len, failcode_buf)) == RET_OK)
	{
		*reg16_data = g_ack_buf[2] | (g_ack_buf[3] << 8);
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

int i2c_read_local_hsmt_reg16(int fd, uint8_t local_addr, int cmd_buf_len, const uint8_t *cmd_buf,  uint16_t *rsp_reg16_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret, len, i, start = 2;
	uint16_t payload_len;

	if (cmd_buf == NULL || cmd_buf_len < 6)
	{
		vin_err("cmd_buf_len is %d\r\n", cmd_buf_len);
		return RET_PARAM_ERR;
	}

	payload_len = (cmd_buf[4] & 8) ? (((cmd_buf[5] & 0x7f) << 4) | (cmd_buf[4] >> 4)) : (cmd_buf[4] >> 4);
	if ((cmd_buf[1] & 7) != 3 || (payload_len & 1) == 1)  // local hsmt reg read
	{
		vin_err("(cmd_buf[1] & 7) is %d, payload_len is %d\r\n", cmd_buf[1], payload_len);
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		vin_err("lock_i2c failed, ret = %d\r\n", ret);
		return ret;
	}

	len = payload_len + 4;
	if (len > 19)
	{
		len++;
		start++;
	}

	if ((ret = i2c_read_cmd(fd, local_addr, cmd_buf_len, cmd_buf, len, failcode_len, failcode_buf)) == RET_OK)
	{
		for (i = 0; i < payload_len; i += 2)
		{
			*rsp_reg16_buf = g_ack_buf[start + i] | (g_ack_buf[start + i + 1] << 8);
			rsp_reg16_buf++;
		}
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

int i2c_read_remote_hsmt_reg16(int fd, uint8_t local_addr, uint8_t tx_map, int cmd_buf_len, const uint8_t *cmd_buf, uint16_t *rsp_reg16_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret, len, i, start = 2;
	uint16_t payload_len;

	if (cmd_buf == NULL || cmd_buf_len < 6)
	{
		return RET_PARAM_ERR;
	}

	payload_len = (cmd_buf[4] & 8) ? (((cmd_buf[5] & 0x7f) << 4) | (cmd_buf[4] >> 4)) : (cmd_buf[4] >> 4);
	if ((cmd_buf[1] & 7) != 1 || (payload_len & 1) == 1)
	{
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		return ret;
	}

	/* Remote read from standard HSMT registers (PORT_STD). */
	if ((ret = set_destination(fd, local_addr, tx_map, PORT_STD, failcode_len, failcode_buf)) != RET_OK)
	{
		unlock_i2c(fd, local_addr);
		return ret;
	}

	len = payload_len + 4;
	if (len > 19)
	{
		len++;
		start++;
	}

	if ((ret = i2c_read_cmd(fd, local_addr, cmd_buf_len, cmd_buf, len, failcode_len, failcode_buf)) == RET_OK)
	{
		for (i = 0; i < payload_len; i += 2)
		{
			*rsp_reg16_buf = g_ack_buf[start + i] | (g_ack_buf[start + i + 1] << 8);
			rsp_reg16_buf++;
		}
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

int base_read_remote_device(int fd, uint8_t local_addr, uint8_t tx_map, uint8_t port_id, int cmd_buf_len, const uint8_t *cmd_buf, uint8_t *rsp_byte_buf, int failcode_len, uint8_t *failcode_buf)
{
	int ret, len, start = 2;
	uint16_t payload_len;

	if (cmd_buf == NULL || cmd_buf_len < 5)
	{
		return RET_PARAM_ERR;
	}

	if ((cmd_buf[1] & 7) != 1)
	{
		return RET_PARAM_ERR;
	}

	payload_len = (cmd_buf[3] & 8) ? (((cmd_buf[4] & 0x7f) << 4) | (cmd_buf[3] >> 4)) : (cmd_buf[3] >> 4);

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		return ret;
	}

	if ((ret = set_destination(fd, local_addr, tx_map, port_id, failcode_len, failcode_buf)) != RET_OK)
	{
		unlock_i2c(fd, local_addr);
		return ret;
	}

	len = payload_len + 4;
	if (len > 19)
	{
		len++;
		start++;
	}

	if ((ret = i2c_read_cmd(fd, local_addr, cmd_buf_len, cmd_buf, len, failcode_len, failcode_buf)) == RET_OK)
	{
		memcpy(rsp_byte_buf, g_ack_buf + start, payload_len);
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

int i2c_read_remote_device(int fd, uint8_t local_addr, uint8_t tx_map, int cmd_buf_len, const uint8_t *cmd_buf, uint8_t *rsp_byte_buf, int failcode_len, uint8_t *failcode_buf)
{
	/* Read from remote sensor via I2C master port (PORT_SENSOR). */
	return base_read_remote_device(fd, local_addr, tx_map, PORT_SENSOR, cmd_buf_len, cmd_buf, rsp_byte_buf, failcode_len, failcode_buf);
}

int set_tx_hsmtid(int fd, uint8_t local_addr, int cmd_buf_len, const uint8_t *cmd_buf, uint8_t *failcode_buf)
{
	int cur_offset = 0, i;
	uint16_t payload_len = 0, crc16;
	int ret = 0, retry = 0;

	if (cmd_buf == NULL || cmd_buf_len < 3)
	{
		return RET_PARAM_ERR;
	}

	if ((ret = lock_i2c(fd, local_addr)) != RET_OK)
	{
		return ret;
	}

	if ((i = find_i2c_handle(fd, local_addr)) != -1)
	{
		g_devs[i / I2C_LOCAL_ADDR_NUM].local_addrs[i % I2C_LOCAL_ADDR_NUM].addr_attr->cur_id = 0;
	}

	while (cur_offset < cmd_buf_len)
	{
		if (cmd_buf_len - cur_offset < 3)
		{
			ret = RET_PARAM_ERR;
			vin_err("cmd_buf_len - cur_offset < 3, cmd_buf_len = %d, cur_offset = %d\r\n", cmd_buf_len, cur_offset);
			break;
		}

		payload_len = (cmd_buf[cur_offset + 1] & 8) ? (((cmd_buf[cur_offset + 2] & 0x7f) << 4) | (cmd_buf[cur_offset + 1] >> 4)) + 4 : (cmd_buf[cur_offset + 1] >> 4) + 3;
		if (payload_len + cur_offset + 1 > cmd_buf_len)
		{
			ret = RET_PARAM_ERR;
			vin_err("payload_len + cur_offset + 1 > cmd_buf_len, payload_len = %d, cmd_buf_len = %d, cur_offset = %d\r\n", payload_len, cmd_buf_len, cur_offset);
			break;
		}

		ret = i2c_write_cmd(fd, local_addr, payload_len + 1, &cmd_buf[cur_offset], 7, failcode_buf);
		// check local or remote, if remote check failcode_buf[2] & 3f == 7  set hsmtid already
		if (ret == RET_IO_ERR || ret == RET_MEM_ERR || ret == RET_PARAM_ERR)
		{
			vin_err("set_tx_hsmtid failed, ret = %d (%s)\r\n", ret,
				ret == RET_IO_ERR ? "IO Error" : (ret == RET_MEM_ERR ? "Memory Error" : "Parameter Error"));
			unlock_i2c(fd, local_addr);
			return ret;
		}
		else if (ret == RET_MSG_ERR)
		{
			if ((cmd_buf[cur_offset + 1] & 7) == 0 && (failcode_buf[2] & 0x3f) == 7) // remote write
			{
				crc16 = calc_crc16(failcode_buf, 5);
				if (((failcode_buf[5] << 8) | failcode_buf[6]) == crc16)
				{
					ret = RET_OK; // set hsmtid already means OK
				}
			}
		}

		if (ret == RET_OK)
		{
			cur_offset += payload_len + 1;
		}
		else if (++retry > RETRY_TIMES)
		{
			break;
		}
	}

	unlock_i2c(fd, local_addr);
	return ret;
}

uint8_t * wrap_buff(uint8_t *buf, int buf_len, int opcode)
{
	uint16_t crc16;

	if (buf_len > 15)
	{
		*--buf = (buf_len >> 4) & 0x7f;
		*--buf = (buf_len << 4) | 8 | opcode;
		buf_len += 2;
	}
	else
	{
		*--buf = (buf_len << 4) | opcode;
		buf_len++;
	}

	crc16 = calc_crc16(buf, buf_len);
	buf[buf_len] = crc16 >> 8;
	buf[buf_len + 1] = crc16 & 0xff;
	*--buf = 0x52;
	return buf;
}

int convert_sensor_write_cmd(int dev_addr, const int * sensor_data, int sensor_data_len, int mode, uint8_t *cmd_buf)
{
	int i, start_addr, data_len, len_offset = 0, cmd_buf_len = 0, offset = 3;
	uint8_t * p_buf;

	if (sensor_data == NULL || cmd_buf == NULL || mode > 1 || mode < 0 || (sensor_data_len & 1) != 0 || sensor_data_len < 1)
	{
		return -1;
	}

	start_addr = sensor_data[0] + 1;
	vin_info("start_addr = 0x%x, sensor_data_len is %d\n", sensor_data_len, start_addr);

	if (mode == 1)
	{
		vin_info("use mode 1\n");
		cmd_buf[offset++] = dev_addr | 1;
		vin_info("dev_addr = 0x%x, cmd_buf[%d] = 0x%x\n", dev_addr, (offset - 1),cmd_buf[offset - 1]);


		for (i = 0; i < sensor_data_len; i += 2)
		{
			if (start_addr != sensor_data[i] || offset - len_offset - 1 == 256)  // not successive or reach the max size
			{
				cmd_buf[len_offset] = (offset - len_offset - 1) % 256;

				start_addr = sensor_data[i];
				len_offset = offset++;
				cmd_buf[offset++] = start_addr >> 8;
				cmd_buf[offset++] = start_addr & 0xff;
			}

			cmd_buf[offset++] = sensor_data[i + 1];
			start_addr++;

			if (offset >= MAX_PACKET_SIZE - 3 || i == sensor_data_len - 2)  // reach the max size or the last packet
			{
				cmd_buf[len_offset] = (offset - len_offset - 1) % 256;
				p_buf = wrap_buff(cmd_buf + 3, offset - 3, 0);
				data_len = (offset > 18) ? offset + 2 : offset + 1;
				if (offset <= 18)
				{
					memmove(cmd_buf, p_buf, data_len);
				}

				cmd_buf_len += data_len;
				cmd_buf += data_len;
				len_offset = 0;
				offset = 3;
				cmd_buf[offset++] = dev_addr | 1;
				start_addr = sensor_data[i + 2] + 1;
			}
			// vin_info("cmd_buf[%d] is 0x%x\n", i, cmd_buf[i]);
		}

		vin_info("cmd_buf_len is %d\n", cmd_buf_len);
		// for (int i = 0; i < cmd_buf_len; i++) {
		// 	vin_info("cmd_buf[%d] is 0x%x\n", i, cmd_buf[i]);
		// }
	}
	else  // mode == 0
	{
		for (i = 0; i < sensor_data_len; i += 2)
		{
			start_addr = sensor_data[i];
			offset = 2;
			cmd_buf[offset++] = dev_addr;
			cmd_buf[offset++] = start_addr >> 8;
			cmd_buf[offset++] = start_addr & 0xff;
			cmd_buf[offset++] = sensor_data[i + 1];
			wrap_buff(cmd_buf + 2, offset - 2, 0);
			cmd_buf_len += 8;
			cmd_buf += 8;
		}
	}

	// for (int i = 0; i < cmd_buf_len; i++) {
	// 	vin_info("cmd_buf[%d] is 0x%x\n", i, cmd_buf[i]);
	// }

	return cmd_buf_len;
}

int write_n_remote_msgs(int fd, uint8_t local_addr, uint8_t tx_map, uint8_t port_id, int buf_len,
						const uint8_t *cmds_buf, int *fail_line_no, int *err_offset)
{
	int i, cur_offset = 0;
	unsigned short payload_len = 0;
	int ret = RET_PARAM_ERR;
	int retry = 0;
	uint8_t failcode[ACK_DATA_LEN];
	int tx_num = 0;
	int failcode_len = 1;
	int tmp = tx_map;

	if (fail_line_no) {
		*fail_line_no = 0;
	}

	if (cmds_buf == NULL)
	{
		ret = RET_PARAM_ERR;
		buf_len = 0;
		vin_info("RET_PARAM_ERR %d %s\r\n", __LINE__, __func__);
	}

	while (tmp)
	{
		tx_num++;
		tmp = tmp & (tmp - 1);
	}

	if (tx_num > 1) // multi tx
	{
		failcode_len = tx_num * 3 + 4;  // add crc
	}

	while (cur_offset < buf_len)
	{

		if (buf_len - cur_offset < 3)
		{
			ret = RET_PARAM_ERR;
			vin_info("RET_PARAM_ERR %d %s, buf_len is %d, cur_offset is %d (buf_len - cur_offset < 3)\r\n",
				__LINE__, __func__, buf_len, cur_offset);
			vin_info("cmds_buf[%d] is 0x%x\r\n", cur_offset, cmds_buf[cur_offset]);
			break;
		}

		payload_len = (cmds_buf[cur_offset + 1] & 8) ? (((cmds_buf[cur_offset + 2] & 0x7f) << 4) | (cmds_buf[cur_offset + 1] >> 4)) + 4 : (cmds_buf[cur_offset + 1] >> 4) + 3;
		if (payload_len > MAX_PACKET_SIZE + 4 || payload_len + cur_offset + 1 > buf_len)
		{
			ret = RET_PARAM_ERR;
			vin_info("RET_PARAM_ERR %d %s\r\n", __LINE__, __func__);
			break;
		}

		// vin_info("i2c_write_remote_msg ret = %d, payload_len = %d, cur_offset = %d\r\n", ret, payload_len, cur_offset);
		ret = i2c_write_remote_msg(fd, local_addr, tx_map, port_id, payload_len + 1, &cmds_buf[cur_offset], failcode_len, g_failcode_buf);

		if (ret == RET_OK)
		{
			cur_offset += payload_len + 1;
			retry = 0;
			if (fail_line_no) (*fail_line_no)++;
			failcode_len = (tx_num == 1) ? 1 : failcode_len;
		}
		else if (ret != RET_MSG_ERR || tx_num > 1 || retry >= RETRY_TIMES)
		{
			break;
		}
		else
		{
			if (++retry == RETRY_TIMES)   // the last try
			{
				failcode_len = 7;
			}
		}
	}

	error_handling(ret, failcode_len);
	if (err_offset != NULL)
	{
		*err_offset = cur_offset;
	}

	return ret;
}
