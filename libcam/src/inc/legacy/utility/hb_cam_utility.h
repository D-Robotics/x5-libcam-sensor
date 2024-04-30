/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file hb_cam_utility.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

/* common data type */
#include "cam_common.h"

/* dummy for legacy header */
#include "camera_legacy.h"

#ifdef CAM_DIAG
/* diag enable */
#include "cam_diag.h"
#endif

// pre board id
#define	BOARD_ID_MATRIXDUO_A	(0x631)
#define	BOARD_ID_MATRIXDUO_B	(0x632)
#define	BOARD_ID_MATRIXDSOLO	(0x641)
// a sample board id
#define BOARD_ID_MATRIXDUO_A_V2	(0X651)
#define	BOARD_ID_MATRIXDUO_B_V2	(0X652)
#define	BOARD_ID_MATRIXDSOLO_V2	(0X642)
// b sample board id
#define BOARD_ID_MATRIXDUO_A_V3	(0X653)
#define BOARD_ID_MATRIXDUO_B_V3	(0X654)
#define BOARD_ID_MATRIXSOLO_V3	(0X643)
