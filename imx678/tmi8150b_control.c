#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <signal.h>
#include <ctype.h>
// #include <spi.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "inc/tmi8150b_control.h"
#include "inc/spi.h"


//*****************************************************************************
//*****************************************************************************
//** Global Data
//*****************************************************************************
//*****************************************************************************
static unsigned char u8CurCh12Dir = 0;
static unsigned char u8CurCh34Dir = 0;

static unsigned short int u8Ch12AlignVal = 0;
static unsigned short int u8Ch34AlignVal = 0;

typedef enum
{
    TMI8150B_ROTATE_CLOCKWISE = 1,
    TMI8150B_ROTATE_ANTI_CLOCKWISE,
} TMI8150B_ROTATE_MOD_E;

typedef enum
{
    TMI8150B_MICROSTEP_128 = 1,
    TMI8150B_MICROSTEP_64,
    TMI8150B_MICROSTEP_32,
    TMI8150B_MICROSTEP_16,
    TMI8150B_MICROSTEP_8,
    TMI8150B_MICROSTEP_4,
    TMI8150B_MICROSTEP_2,
    TMI8150B_MICROSTEP_1,
} TMI8150B_MICROSTEP_E;

typedef enum
{
    TMI8150B_IR_CTRL_DISABLE = 1,
    TMI8150B_IR_CTRL_CLOCKWISE,
    TMI8150B_IR_CTRL_ANTI_CLOCKWISE,
    TMI8150B_IR_CTRL_BRAKE,
} TMI8150B_IR_CTRL_E;



int Global_control(void)
{
    int ret = 0;
    ret |= Tmi8150b_Spi_Write_register(
        0x82, 0x03);   // bit1 = 1 : angel & phase can write, bit0 = 1: CLK_RDY
    ret |= Tmi8150b_Spi_Write_register(0x82, 0x8B);   // global clear soft reset
    return ret;
}


/*!
*******************************************************************************
** \brief :
** -tmi8150b spi init and globle register init
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Tmi8150_Api_init(void)
{
    int ret = 0;
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "write mode");
    Tmi8150b_Spi_Params_st stTmi8150bSpiParam;
    memset(&stTmi8150bSpiParam, 0, sizeof(Tmi8150b_Spi_Params_st));

    // tmi8150b spi init
    strncpy(stTmi8150bSpiParam.devspi, TMI8150B_SPI_DEV_PATH, sizeof(stTmi8150bSpiParam.devspi));
    stTmi8150bSpiParam.mode = TMI8150B_SPI_MODE;
    stTmi8150bSpiParam.lsb = TMI8150B_SPI_MSB;   // MSB: MSB First
    stTmi8150bSpiParam.bits = TMI8150B_SPI_BITS;
    stTmi8150bSpiParam.speed = TMI8150B_SPI_SPEED;
    ret |= Tmi8150b_Spi_Init(&stTmi8150bSpiParam);
    usleep(100 * 1000);

    return ret;
}






/*!
*******************************************************************************
** \brief :
** -disable enable ch12 & ch34 and tmi8150b spi deinit
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Tmi8150_Api_deinit(void)
{
    int ret = 0;
    ret |= Tmi8150b_Spi_Deinit();

    return ret;
}



/*!
*******************************************************************************
** \brief :
** -CH34 driven stepper motor stop
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_motor_disable(void)
{
    int ret = 0;
    ret |= Tmi8150b_Spi_Write_register(0x94, 0x07);   // bit7: 1 = disable enable ch34
    return ret;
}



/*!
*******************************************************************************
** \brief :
** -CH34 driven stepper motor stop
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_motor_lock(void)
{
    int ret = 0;
    ret |= Tmi8150b_Spi_Write_register(0x95, 0x7F);   // bit7: 1 = disable enable ch34
    return ret;
}



/*!
*******************************************************************************
** \brief :
** -CH12 driven stepper motor rotates to set angle, must first globally enable 
** 'Global_control' and configure CH12 parameters 'Xaxis_motor_solution'
**
** \param[in] angle	 angle determines number of electrical cycles
** \param[in] phase	 phase determines phase, phase value range is 0~1023
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Xaxis_motor_manual_start(unsigned short int angle,unsigned short int phase)
{
    int ret = 0;

    ret |= Tmi8150b_Spi_Write_register(0x84,0x07);  // bit0~2: 110 =  ch12 manual spwm ctrl
    // ret |= Xaxis_angel_phase_clear();

    // Correcting phase
    unsigned short int tmp_phase = 0;
    if(0 != (phase%u8Ch12AlignVal)){
        tmp_phase = phase + (u8Ch12AlignVal - (phase%u8Ch12AlignVal));
        if(tmp_phase > 0x3ff)
        {
            tmp_phase = phase - (phase%u8Ch12AlignVal);
        }
    }
    else{
        tmp_phase = phase;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG,"phase = 0x%04x, tmp_phase = 0x%04x",phase,tmp_phase);

    if(TMI8150B_ROTATE_CLOCKWISE == u8CurCh12Dir) //clockwise
    {
        unsigned char angle_low = angle;
        unsigned char angle_high = angle >> 8;
        ret |= Tmi8150b_Spi_Write_register(0x8B, angle_low);
        ret |= Tmi8150b_Spi_Write_register(0x8C, angle_high);
        unsigned char phase_low = tmp_phase;
        unsigned char phase_high = tmp_phase >> 8;
        phase_high = phase_high << 4;
        ret |= Tmi8150b_Spi_Write_register(0x8A, phase_low);
        ret |= Tmi8150b_Spi_Write_register(0x8F, phase_high);
    }
     else if(TMI8150B_ROTATE_ANTI_CLOCKWISE == u8CurCh12Dir) //counterclockwise
     {
        if(tmp_phase == 0)//phase is 0
        {
            unsigned char angle_low = 8192 - angle;
            unsigned char angle_high = (8192 - angle) >> 8;
            // unsigned char angle_low = angle;
            // unsigned char angle_high = angle >> 8;
            ret |= Tmi8150b_Spi_Write_register(0x8B, angle_low);
            ret |= Tmi8150b_Spi_Write_register(0x8C, angle_high);
            ret |= Tmi8150b_Spi_Write_register(0x8A,0);
            ret |= Tmi8150b_Spi_Write_register(0x8F,0);
        }
        else if(tmp_phase != 0)//phase is not 0
        {
            unsigned char angle_low = 8191 - angle;
            unsigned char angle_high = (8191 - angle) >> 8;
            // unsigned char angle_low = angle;
            // unsigned char angle_high = angle >> 8;
            ret |= Tmi8150b_Spi_Write_register(0x8B, angle_low);
            ret |= Tmi8150b_Spi_Write_register(0x8C, angle_high);

            unsigned char phase_low = (1024 - tmp_phase);
            unsigned char phase_high = (1024 - tmp_phase) >> 8;
            phase_high = phase_high << 4;
            ret |= Tmi8150b_Spi_Write_register(0x8A, phase_low);
            ret |= Tmi8150b_Spi_Write_register(0x8F, phase_high);
        }
    }
    ret |= Tmi8150b_Spi_Write_register(0x84,0x87);  // bit7 = 1 : ch12 enable

    return ret;

}



/*!
*******************************************************************************
** \brief :
** -CH34 driven stepper motor rotates to set angle, must first globally enable 
** 'Global_control' and configure CH34 parameters 'Yaxis_motor_solution'
**
** \param[in] angle	 angle determines number of electrical cycles
** \param[in] phase	 phase determines phase, phase value range is 0~1023
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_motor_manual_start(unsigned short int angle, unsigned short int phase)
{
    int ret = 0;
    ret |= Tmi8150b_Spi_Write_register(0x94, 0x07);   // bit0~2: 110 =  ch34 manual spwm ctrl
    // ret |= Yaxis_angel_phase_clear();

    // Correcting phase
    unsigned short int tmp_phase = 0;
    if (0 != (phase % u8Ch34AlignVal))
    {
        tmp_phase = phase + (u8Ch34AlignVal - (phase % u8Ch34AlignVal));
        if (tmp_phase > 0x3ff)
        {
            tmp_phase = phase - (phase % u8Ch34AlignVal);
        }
    }
    else
    {
        tmp_phase = phase;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "phase = 0x%04x, tmp_phase = 0x%04x", phase, tmp_phase);

    // printf("%s() : angle = %d ,  phase = %d\n",__FUNCTION__ , angle , phase);
    if (TMI8150B_ROTATE_CLOCKWISE == u8CurCh34Dir)   //clockwise
    {
        // printf("%s() TMI8150B_ROTATE_CLOCKWISE\n",__FUNCTION__);

        unsigned char angle_low = angle;
        unsigned char angle_high = angle >> 8;
        ret |= Tmi8150b_Spi_Write_register(0x9B, angle_low);
        ret |= Tmi8150b_Spi_Write_register(0x9C, angle_high);
        unsigned char phase_low = tmp_phase;
        unsigned char phase_high = tmp_phase >> 8;
        phase_high = phase_high << 4;
        ret |= Tmi8150b_Spi_Write_register(0x9A, phase_low);
        ret |= Tmi8150b_Spi_Write_register(0x92, phase_high);
        // printf("CLOCKWISE , angle_low = 0x%02x , angle_high = 0x%02x\n" , angle_low , angle_high);
    }
    else if (TMI8150B_ROTATE_ANTI_CLOCKWISE == u8CurCh34Dir)   //counterclockwise
    {
        // printf("%s() TMI8150B_ROTATE_ANTI_CLOCKWISE\n",__FUNCTION__);

        if (tmp_phase == 0)   //相位为 0
        {
            // unsigned char angle_low = 8192 - angle;
            // unsigned char angle_high = (8192 - angle) >> 8;
            unsigned char angle_low = angle;
            unsigned char angle_high = angle >> 8;
            ret |= Tmi8150b_Spi_Write_register(0x9B, angle_low);
            ret |= Tmi8150b_Spi_Write_register(0x9C, angle_high);
            ret |= Tmi8150b_Spi_Write_register(0x9A, 0);
            ret |= Tmi8150b_Spi_Write_register(0x92, 0);
            // printf("ANTI_CLOCKWISE , angle_low = 0x%02x , angle_high = 0x%02x\n" , angle_low , angle_high);
        }
        else if (tmp_phase != 0)   //phase is not 0
        {
            // unsigned char angle_low = 8191 - angle;
            // unsigned char angle_high = (8191 - angle) >> 8;
            unsigned char angle_low = angle;
            unsigned char angle_high = angle >> 8;
            ret |= Tmi8150b_Spi_Write_register(0x9B, angle_low);
            ret |= Tmi8150b_Spi_Write_register(0x9C, angle_high);

            unsigned char phase_low = (1024 - tmp_phase);
            unsigned char phase_high = (1024 - tmp_phase) >> 8;
            phase_high = phase_high << 4;
            ret |= Tmi8150b_Spi_Write_register(0x9A, phase_low);
            ret |= Tmi8150b_Spi_Write_register(0x92, phase_high);
            // printf("ANTI_CLOCKWISE , angle_low = 0x%02x , angle_high = 0x%02x\n" , angle_low , angle_high);
        }
    }
    // printf("%s() after spi write()\n",__FUNCTION__);


    ret |= Tmi8150b_Spi_Write_register(0x94, 0x87);   // bit7 = 1 : ch34 enable
    return ret;
}


/*!
*******************************************************************************
** \brief :
** -Configure CH12 channel stepper motor working mode, including direction, microstep, rotation speed
**
** \param[in] dir	 		determine direction (1-clockwise; 2-counterclockwise)
** \param[in] microstep	 	determine microstep (1-128 microstep; 2-64 microstep; 3-32 microstep; 4-16 microstep)
** \param[in] speed	 	    determine frequency division to affect rotation speed (1~30 frequency division)
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Xaxis_motor_solution(unsigned char dir,unsigned char microstep,unsigned char speed)
{
    int ret = 0;
    unsigned char  dir_bit,micro_bit;

    // Rotate Direction
    switch(dir)
    {
        case TMI8150B_ROTATE_CLOCKWISE:
            dir_bit = 5;
            u8CurCh12Dir = TMI8150B_ROTATE_CLOCKWISE;
            break;
        case TMI8150B_ROTATE_ANTI_CLOCKWISE:
            dir_bit = 10;
            u8CurCh12Dir = TMI8150B_ROTATE_ANTI_CLOCKWISE;
            break;
        default:
            dir_bit = 5;
            u8CurCh12Dir = TMI8150B_ROTATE_CLOCKWISE;
            break;
        //other inputs default to clockwise
    }

    // MicroStep
    switch(microstep)
    {
        case TMI8150B_MICROSTEP_128:
            micro_bit = 7;
            u8Ch12AlignVal = 256 /128;
            break;
        case TMI8150B_MICROSTEP_64 :
            micro_bit = 6;
            u8Ch12AlignVal = 256 /64;
            break;
        case TMI8150B_MICROSTEP_32 :
            micro_bit = 5;
            u8Ch12AlignVal = 256 /32;
            break;
        case TMI8150B_MICROSTEP_16 :
            micro_bit = 4;
            u8Ch12AlignVal = 256 /16;
            break;
        case TMI8150B_MICROSTEP_8 :
            micro_bit = 3;
            u8Ch12AlignVal = 256 /8;
            break;
        case TMI8150B_MICROSTEP_4 :
            micro_bit = 2;
            u8Ch12AlignVal = 256 /4;
            break;
        case TMI8150B_MICROSTEP_2 :
            micro_bit = 1;
            u8Ch12AlignVal = 256 /2;
            break;
        case TMI8150B_MICROSTEP_1 :
            micro_bit = 0;
            u8Ch12AlignVal = 256 /1;
            break;
        default :
            micro_bit = 7;
            u8Ch12AlignVal = 256 /128;
            break;
        //other inputs default to 128 microstep
    }
    ret |= Tmi8150b_Spi_Write_register(0x85, micro_bit*16+dir_bit);

    // Frequency Division (Rotation Speed)
    if(speed > 30)
    {
        speed = 30;
    }
    else if(0 == speed)
    {
        speed = 1;
    }
    ret |= Tmi8150b_Spi_Write_register(0x86, speed-1);
    return ret;
}



/*!
*******************************************************************************
** \brief :
** -Configure CH34 channel stepper motor working mode, including direction, microstep, rotation speed
**
** \param[in] dir	 		determine direction (1-clockwise; 2-counterclockwise)
** \param[in] microstep	 	determine microstep (1-128 microstep; 2-64 microstep; 3-32 microstep; 4-16 microstep)
** \param[in] speed	 	    etermine frequency division to affect rotation speed (1~30 frequency division, maximum 30 frequency division)
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_motor_solution(unsigned char dir, unsigned char microstep, unsigned char speed)
{
    int ret = 0;
    unsigned char dir_bit, micro_bit;

    // Rotate Direction
    switch (dir)
    {
    case TMI8150B_ROTATE_CLOCKWISE:
        dir_bit = 5;
        u8CurCh34Dir = TMI8150B_ROTATE_CLOCKWISE;
        break;
    case TMI8150B_ROTATE_ANTI_CLOCKWISE:
        dir_bit = 10;
        u8CurCh34Dir = TMI8150B_ROTATE_ANTI_CLOCKWISE;
        break;
    default:
        dir_bit = 5;
        u8CurCh34Dir = TMI8150B_ROTATE_CLOCKWISE;
        break;
        //other inputs default to clockwise
    }

    // MicroStep
    switch (microstep)
    {
    case TMI8150B_MICROSTEP_128:
        micro_bit = 7;
        u8Ch34AlignVal = 256 / 128;
        break;
    case TMI8150B_MICROSTEP_64:
        micro_bit = 6;
        u8Ch34AlignVal = 256 / 64;
        break;
    case TMI8150B_MICROSTEP_32:
        micro_bit = 5;
        u8Ch34AlignVal = 256 / 32;
        break;
    case TMI8150B_MICROSTEP_16:
        micro_bit = 4;
        u8Ch34AlignVal = 256 / 16;
        break;
    case TMI8150B_MICROSTEP_8:
        micro_bit = 3;
        u8Ch34AlignVal = 256 / 8;
        break;
    case TMI8150B_MICROSTEP_4:
        micro_bit = 2;
        u8Ch34AlignVal = 256 / 4;
        break;
    case TMI8150B_MICROSTEP_2:
        micro_bit = 1;
        u8Ch34AlignVal = 256 / 2;
        break;
    case TMI8150B_MICROSTEP_1:
        micro_bit = 0;
        u8Ch34AlignVal = 256 / 1;
        break;
    default:
        micro_bit = 7;
        u8Ch34AlignVal = 256 / 128;
        break;
        //other inputs default to 128 microstep
    }
    ret |= Tmi8150b_Spi_Write_register(0x95, micro_bit * 16 + dir_bit);

    // Frequency Division (Rotation Speed)
    if (speed > 30)
    {
        speed = 30;
    }
    else if (0 == speed)
    {
        speed = 1;
    }
    ret |= Tmi8150b_Spi_Write_register(0x96, speed - 1);
    return ret;
}

/*!
*******************************************************************************
** \brief :
** -Reset CH12 channel turn count and phase counter to zero, phase setting high two bits will also be cleared
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Xaxis_angel_phase_clear(void)
{
    int ret = 0;
    // clear ch12 angel register
    ret |= Tmi8150b_Spi_Write_register(0x88, 0x00);
    ret |= Tmi8150b_Spi_Write_register(0x89, 0x00);
    // clear ch12 phase register
    ret |= Tmi8150b_Spi_Write_register(0x87, 0x00);
    ret |= Tmi8150b_Spi_Write_register(0x8F, 0x00);
    return ret;
}


/*!
*******************************************************************************
** \brief :
** -Reset CH34 channel turn count and phase counter to zero, phase setting high two bits will also be cleared
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_angel_phase_clear(void)
{
    int ret = 0;
    // clear ch34 angel register
    ret |= Tmi8150b_Spi_Write_register(0x98, 0x00);
    ret |= Tmi8150b_Spi_Write_register(0x99, 0x00);
    // clear ch34 phase register
    ret |= Tmi8150b_Spi_Write_register(0x97, 0x00);
    ret |= Tmi8150b_Spi_Write_register(0x92, 0x00);
    return ret;
}


/*!
*******************************************************************************
** \brief :
** -Read CH12 channel current turn count counter value
**
** \param[in] angle_now	 	current motor angle
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Xaxis_angel_read(short int* angle_now)
{
    int ret = 0;
    unsigned char  angle_low;
    unsigned char  angle_high;

    ret |= Tmi8150b_Spi_Read_register(0x08, &angle_low);
    ret |= Tmi8150b_Spi_Read_register(0x09, &angle_high);
    *angle_now = (angle_high&0x1F)*256+angle_low;
    return ret;
}



/*!
*******************************************************************************
** \brief :
** -Read CH34 channel current turn count counter value
**
** \param[in] angle_now     current motor angle
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Yaxis_angel_read(short int *angle_now)
{
    int ret = 0;
    unsigned char angle_low;
    unsigned char angle_high;

    ret |= Tmi8150b_Spi_Read_register(0x18, &angle_low);
    ret |= Tmi8150b_Spi_Read_register(0x19, &angle_high);
    *angle_now = (angle_high & 0x1F) * 256 + angle_low;
    // printf("%s() : angle_low = %x , angle_high = %x\n" , __FUNCTION__ , angle_low , angle_high);

    return ret;
}


/*!
*******************************************************************************
** \brief :
** -Change CH12, CH34 PWM update frequency, can be used to reduce rotation speed
**
** \param[in] xfactor	 determine CH12 update frequency, 1~8 times speed reduction, default is 1
** \param[in] yfactor	 determine CH34 update frequency, 1~8 times speed reduction, default is 1
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int XY_sin_factor(unsigned char xfactor, unsigned char yfactor)
{
    int ret = 0;

    // check and change ch12 pwm update mode
    if (xfactor >= 8)
    {
        xfactor = 8;
    }
    else if (0 == xfactor)
    {
        xfactor = 1;
    }
    xfactor = xfactor - 1;

    // check and change ch34 pwm update mode
    if (yfactor > 8)
    {
        yfactor = 8;
    }
    else if (0 == yfactor)
    {
        yfactor = 1;
    }
    yfactor = yfactor - 1;

    // set ch12 & ch34 pwm update mode
    yfactor = (yfactor << 4);
    ret |= Tmi8150b_Spi_Write_register(0x9F, xfactor + yfactor);

    return ret;
}





//*****************************************************************************
//*****************************************************************************
//** Global Data
//*****************************************************************************
//*****************************************************************************
static int tmi8150bSpiFd = 0;
static Tmi8150b_Spi_Params_st stTmi8150bSpiParams;

/*!
*******************************************************************************
** \brief :
** -Tmi8150b Spi Init
**
** \param[in] pSpiParams : Tmi8150b_Spi_Params_st
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Tmi8150b_Spi_Init(Tmi8150b_Spi_Params_st *pSpiParams)
{
    int ret = 0;
    unsigned int mode = 0;
    unsigned int speed = 0;
    unsigned char lsb = 0;
    unsigned char bits = 0;

    memset(&stTmi8150bSpiParams, 0, sizeof(stTmi8150bSpiParams));
    if (NULL == pSpiParams)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "pSpiParams is invalid, usd default value !!!");
        strncpy(stTmi8150bSpiParams.devspi, "/dev/spidev2.0", sizeof(stTmi8150bSpiParams.devspi));
        stTmi8150bSpiParams.mode = SPI_MODE_0;
        stTmi8150bSpiParams.lsb = 0;     // MSB: MSB First
        // stTmi8150bSpiParams.bits = 16;   // 16bits data
        stTmi8150bSpiParams.speed = 1 * 1000 * 1000;
        stTmi8150bSpiParams.bits = 8;   // 16bits data
        // stTmi8150bSpiParams.speed = 2 * 1000 * 1000;
    }
    else
    {
        memcpy((void *)&stTmi8150bSpiParams, (void *)pSpiParams, sizeof(Tmi8150b_Spi_Params_st));
    }

    if (tmi8150bSpiFd != 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Tmi8150b spi had inited !!!");
        return -1;
    }

    tmi8150bSpiFd = open(stTmi8150bSpiParams.devspi, O_RDWR);
    if (tmi8150bSpiFd < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "open %s failed !!!", stTmi8150bSpiParams.devspi);
        return -1;
    }

// spi work mode check
#if 0
	if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_MODE, &mode) < 0) {
		tmi8150b_log(TMI8150B_LOG_LEVEL_ERR,"SPI rd_mode");
		return -1;
	}
    mode = (mode&0xFFFFFFFC)|stTmi8150bSpiParams.mode;
#endif
    mode = stTmi8150bSpiParams.mode;
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "write mode = %d", mode);
    // if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_MODE, &mode) < 0)
    if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_MODE32, &mode) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_mode");
        return -1;
    }
    if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_MODE, &mode) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_mode");
        return -1;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "read mode = %d", mode);

    // Msb first check
    lsb = stTmi8150bSpiParams.lsb;
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "write lsb = %d", lsb);
    if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_LSB_FIRST, &lsb) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_lsb_fist");
        return -1;
    }
    if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_LSB_FIRST, &lsb) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_lsb_fist");
        return -1;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "read lsb = %d", lsb);

    // 8bits data send check
    bits = stTmi8150bSpiParams.bits;
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "write bits = %d", bits);
    if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_lsb_fist");
        return -1;
    }
    if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI bits_per_word");
        return -1;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "read bits = %d", bits);

    // SCLK check
    speed = stTmi8150bSpiParams.speed;
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "write speed = %d", speed);
    if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_lsb_fist");
        return -1;
    }
    if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI bits_per_word");
        return -1;
    }
    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "read speed = %d", speed);

    tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "%s: spi mode 0x%x, %d bits, %s per word, %d Hz max\n",
                 stTmi8150bSpiParams.devspi, mode, bits, lsb ? "(lsb first) " : "(msb first)",
                 speed);

    return ret;
}

/*!
*******************************************************************************
** \brief :
** -Deinit tmi8150b spi
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Tmi8150b_Spi_Deinit(void)
{
    if (0 == tmi8150bSpiFd)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Tmi8150b spi don't init !!!");
        return -1;
    }

    close(tmi8150bSpiFd);
    tmi8150bSpiFd = 0;

    return 0;
}

/*!
*******************************************************************************
** \brief :
** -Write tmi8150b register
**
** \param[in] void
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
// int Tmi8150b_Spi_Write_register(unsigned char addr, unsigned char val)
// {
//     int ret = 0;
//     unsigned char txBuf[3];

//     if (0 == tmi8150bSpiFd)
//     {
//         tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Tmi8150b spi don't init !!!");
//         return -1;
//     }
//     memset(txBuf, 0, sizeof(txBuf));
//     txBuf[1] = addr;
//     txBuf[0] = val;
//     ret = write(tmi8150bSpiFd, txBuf, 2);
//     if (ret < 0)
//     {
//         tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI Write addr: 0x%02x = 0x%02x error\n", addr, val);
//     }
//     else
//     {
//         tmi8150b_log(TMI8150B_LOG_LEVEL_DBG,
//                      "SPI Write addr: 0x%02x = 0x%02x Success ret = 0x%02x\n", addr, val, ret);
//     }
//     return ret;
// }

int Tmi8150b_Spi_Write_register(unsigned char addr, unsigned char val)
{
    int ret = 0;
    unsigned char txBuf[2];
    unsigned char rxBuf[2]; // SPI is full duplex, even writes need receive buffer

    if (0 == tmi8150bSpiFd)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Tmi8150b spi don't init !!!");
        return -1;
    }

    memset(txBuf, 0, sizeof(txBuf));
    memset(rxBuf, 0, sizeof(rxBuf));

    // Set transmit data: usually write operation requires address+data
    // According to your device protocol, may need specific command format
    txBuf[0] = addr;    // register address
    txBuf[1] = val;     // value to write

    // Use ioctl for SPI transfer
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (size_t)txBuf;
    tr.rx_buf = (size_t)rxBuf; // Even writes need receive buffer
    tr.len = 2;
    tr.delay_usecs = 10;
    tr.speed_hz = stTmi8150bSpiParams.speed; // Use speed set during initialization
    tr.bits_per_word = stTmi8150bSpiParams.bits; // Use bits set during initialization

    ret = ioctl(tmi8150bSpiFd, SPI_IOC_MESSAGE(1), &tr);

    // printf("SPI Write addr: 0x%02x = 0x%02x\n", addr, val);

    if (ret < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI Write addr: 0x%02x = 0x%02x error", addr, val);
    }
    else
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_DBG,
                     "SPI Write addr: 0x%02x = 0x%02x Success", addr, val);

        // Optional: print debug information
        tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, 
                     "SPI Write: tx[0]=0x%02x, tx[1]=0x%02x, rx[0]=0x%02x, rx[1]=0x%02x",
                     txBuf[0], txBuf[1], rxBuf[0], rxBuf[1]);
    }
    return ret;
}

/*!
*******************************************************************************
** \brief :
** -Read tmi8150b register
**
** \param[in] addr
** \param[in] val
**
** \return
** - #0         success
** - #other     failed
**
*******************************************************************************
*/
int Tmi8150b_Spi_Read_register(unsigned char addr, unsigned char *val)
{
    int ret = 0;
    unsigned char txBuf[2];
    unsigned char rxBuf[2];
    unsigned int mode = 0;

    if (0 == tmi8150bSpiFd)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Tmi8150b spi don't init !!!");
        return -1;
    }

    if (NULL == val)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "Recive data buff is null !!!");
        return -1;
    }

    memset(txBuf, 0, sizeof(txBuf));
    memset(rxBuf, 0, sizeof(rxBuf));

// #if 1
//     struct spi_ioc_transfer tr[1];
//     memset(tr, 0, sizeof(tr));
//     tr[0].tx_buf = (size_t)txBuf;
//     tr[0].rx_buf = (size_t)rxBuf;
//     tr[0].len = 2;
//     txBuf[1] = addr;
//     ret = ioctl(tmi8150bSpiFd, SPI_IOC_MESSAGE(1), tr);
// // tmi8150b_log(TMI8150B_LOG_LEVEL_ERR,"rxBuf[0] = 0x%02x, rxBuf[1] = 0x%02x\n",rxBuf[0],rxBuf[1]);
// #else
//     struct spi_ioc_transfer tr[2];
//     memset(tr, 0, sizeof(tr));
//     tr[0].tx_buf = (unsigned long long)txBuf;
//     tr[0].len = 1;
//     tr[1].rx_buf = (unsigned long long)val;
//     tr[1].len = 1;
//     txBuf[0] = addr;
//     ret = ioctl(tmi8150bSpiFd, SPI_IOC_MESSAGE(2), tr);
// #endif

    struct spi_ioc_transfer tr[1];
    memset(tr, 0, sizeof(tr));

    tr[0].tx_buf = (size_t)txBuf;
    tr[0].rx_buf = (size_t)rxBuf;
    tr[0].len = 2;
    // tr[0].delay_usecs = 10;           // add delay
    // tr[0].speed_hz = 2000000;         // add speed(2MHz)
    // tr[0].bits_per_word = 8;          // add bits

    // Set transmit data: usually read operation requires sending address first
    // txBuf[0] = addr | 0x80;  // Assume highest bit indicates read operation
    txBuf[0] = addr;
    txBuf[1] = 0x00;         //  dummy byte

    mode = SPI_MODE_0;
    // if (ioctl(tmi8150bSpiFd, SPI_IOC_RD_MODE, &mode) < 0)
    // {
    //     tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI rd_mode");
    //     return -1;
    // }
    ret = ioctl(tmi8150bSpiFd, SPI_IOC_MESSAGE(1), tr);
    // *val = rxBuf[0];
    //
    *val = rxBuf[1];
    // printf("txBuf[0] = 0x%02x , txBuf[1] = 0x%02x ,  rxBuf[0] = 0x%02x, rxBuf[1] = 0x%02x\n",
    //     txBuf[0] , txBuf[1] , rxBuf[0], rxBuf[1]);
    if (ret < 0)
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI Read addr: 0x%02x error\n", addr);
    }
    else
    {
        tmi8150b_log(TMI8150B_LOG_LEVEL_DBG, "SPI Read addr: 0x%02x = 0x%02x Success\n", addr,
                     *val);
    }

    // if (ioctl(tmi8150bSpiFd, SPI_IOC_WR_MODE, &mode) < 0)
    // {
    //     tmi8150b_log(TMI8150B_LOG_LEVEL_ERR, "SPI wd_mode");
    //     return -1;
    // }
    return ret;
}

