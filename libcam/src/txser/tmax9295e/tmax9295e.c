/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file tmax9295e.c
 *
 * @NO{S10E02C05}
 * @ASIL{B}
 */

#define pr_fmt(fmt)		"[tmax9295e]:" fmt

#include "hb_camera_error.h"

#include "camera_log.h"

#include "camera_mod_txser.h"

/**
 * @var txser_dump_regs
 * txser dump registers for debug
 */
static const int32_t txser_dump_regs[] = {
	0x0,
};

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief txser init operation: keep stream off after done
 *
 * @param[in] txser_info: txser module struct
 *
 * @return 0:Success, <0:Failure
 */
static int32_t txser_init(txser_info_t *txser_info)
{
	int32_t ret = RET_OK;
	cam_info("txser%d i2c%d@0x%02x init\n", txser_info->index,
		txser_info->bus_num, txser_info->txser_addr);

	return ret;
}

/**
 * @NO{S10E02C05}
 * @ASIL{B}
 * @brief txser deinit operation: restore the status befor init
 *
 * @param[in] txser_info: txser module struct
 *
 * @return 0:Success, <0:Failure
 */
static int32_t txser_deinit(txser_info_t *txser_info)
{
	int32_t ret = RET_OK;
	cam_info("txser%d deinit\n", txser_info->index);

	return ret;
}

TXSER_MODULE_DF(tmax9295e, txser_dump_regs, CAM_MODULE_FLAG_A16D8);
txser_module_t tmax9295e = {
	.module = TXSER_MNAME(tmax9295e),
	.init = txser_init,
	.deinit = txser_deinit,
};
