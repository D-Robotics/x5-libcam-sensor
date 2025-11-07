
#ifndef _TMI8150B_CONTROL_H_
#define _TMI8150B_CONTROL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define TMI8150B_LOG_LEVEL_DBG 1
#define TMI8150B_LOG_LEVEL_INF 2
#define TMI8150B_LOG_LEVEL_ERR 3

#define TMI8150B_LOG_LEVEL TMI8150B_LOG_LEVEL_ERR

#define tmi8150b_log(level, fmt, args...)                                                          \
    do                                                                                             \
    {                                                                                              \
        if (level >= TMI8150B_LOG_LEVEL)                                                           \
        {                                                                                          \
            printf("[tmi8150b_log_info] [ %s, Line: %d ]  " fmt "\n", __func__, __LINE__, ##args); \
        }                                                                                          \
    } while (0)

    typedef struct
    {
        char devspi[128];
        unsigned int mode;
        unsigned int speed;   // unit: hz
        char lsb;
        char bits;
    } Tmi8150b_Spi_Params_st;

#define TMI8150B_SPI_DEV_PATH "/dev/spidev2.0"
#define TMI8150B_SPI_MODE                                                                          \
    SPI_MODE_0
#define TMI8150B_SPI_LSB 1
#define TMI8150B_SPI_MSB 0
// #define TMI8150B_SPI_BITS 16                 // 16bits data
#define TMI8150B_SPI_BITS 8                 // 8bits data
#define TMI8150B_SPI_SPEED 2 * 1000 * 1000   // units: hz, tmi8150b max speed is 5Mhz

int Global_control(void);
int Tmi8150_Api_init(void);
int Tmi8150_Api_deinit(void);
int Yaxis_motor_disable(void);
int Yaxis_motor_lock(void);
int Yaxis_motor_manual_start(unsigned short int angle, unsigned short int phase);
int Yaxis_motor_solution(unsigned char dir, unsigned char microstep, unsigned char speed);
int Yaxis_angel_phase_clear(void);
int Yaxis_angel_read(short int *angle_now);
int Xaxis_angel_read(short int* angle_now);
int Xaxis_angel_phase_clear(void);
int Xaxis_motor_solution(unsigned char dir,unsigned char microstep,unsigned char speed);
int Xaxis_motor_manual_start(unsigned short int angle,unsigned short int phase);
int XY_sin_factor(unsigned char xfactor, unsigned char yfactor);
int Tmi8150b_Spi_Init(Tmi8150b_Spi_Params_st *pSpiParams);
int Tmi8150b_Spi_Deinit(void);
int Tmi8150b_Spi_Write_register(unsigned char addr, unsigned char val);
int Tmi8150b_Spi_Read_register(unsigned char addr, unsigned char *val);

#endif /* _TMI8150B_CONTROL_H_ */