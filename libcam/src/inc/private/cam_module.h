/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_module.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef __CAM_MODULE_H__
#define __CAM_MODULE_H__

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAM_MODULE_VERSION_MAJOR	(1)
#define CAM_MODULE_VERSION_MINOR	(0)
#define CAM_MODULE_VERSION_PATCH	(0)

/**
 * @def CAM_MODULE_VERSION
 * the version of camera module base
 */
#define CAM_MODULE_VERSION		(((CAM_MODULE_VERSION_MAJOR) << 16) | \
					 ((CAM_MODULE_VERSION_MINOR) << 8) | \
					 ((CAM_MODULE_VERSION_PATCH) << 0))
/**
 * @def CAM_MODULE_MAGIC_CODE
 * the magic code of camera module to verify
 * define as: Hcam
 */
#define CAM_MODULE_MAGIC_CODE		(0x4863616D)
/**
 * @def CAM_MODULE_NAME_LEN
 * the max length of camera module name string
 */
#define CAM_MODULE_NAME_LEN		(100)
/**
 * @def CAM_MODULE_DATA_MAX
 * the max of camera module name string
 */
#define CAM_MODULE_DATA_MAX		(4)

/**
 * @def CAM_MODULE_SOLIB_PRE
 * the so lib name prefix of camera module to used
 * define as: Hcam
 */
#define CAM_MODULE_SOLIB_PRE		"lib"
/**
 * @def CAM_MODULE_SOLIB_NAME
 * the so lib name of camera module to used
 * define as: Hcam
 */
#define CAM_MODULE_SOLIB_NAME		CAM_MODULE_SOLIB_PRE "%s.so"
/**
 * @def CAM_MODULE_SOLIB_LEN
 * the length of so lib name string of camera module to used
 * define as: Hcam
 */
#define CAM_MODULE_SOLIB_LEN		(CAM_MODULE_NAME_LEN + 8)

/**
 * @def CAM_MODULE_FLAG_DLEN
 * get thg flags value of data length
 */
#define CAM_MODULE_FLAG_DLEN(d)		((((d) / 8) & 0xF) << 0)
/**
 * @def CAM_MODULE_FLAG_ALEN
 * get thg flags value of address length
 */
#define CAM_MODULE_FLAG_ALEN(a)		((((a) / 8) & 0xF) << 4)
/**
 * @def CAM_MODULE_FLAG_EXT
 * get thg flags value of others extern
 */
#define CAM_MODULE_FLAG_EXT(e)		((e) << 16)

/**
 * @def CAM_MODULE_FLAG_A8D8
 * the flags for device with 8bit addr 8bit data
 */
#define CAM_MODULE_FLAG_A8D8		(CAM_MODULE_FLAG_ALEN(8) | CAM_MODULE_FLAG_DLEN(8))
/**
 * @def CAM_MODULE_FLAG_A16D8
 * the flags for device with 16bit addr 8bit data
 */
#define CAM_MODULE_FLAG_A16D8		(CAM_MODULE_FLAG_ALEN(16) | CAM_MODULE_FLAG_DLEN(8))
/**
 * @def CAM_MODULE_FLAG_A16D16
 * the flags for device with 16bit addr 16bit data
 */
#define CAM_MODULE_FLAG_A16D16		(CAM_MODULE_FLAG_ALEN(16) | CAM_MODULE_FLAG_DLEN(16))


/**
 * @def CAM_MODULE_GET_FLAG_DLEN
 * get data bit length config from module flags
 */
#define CAM_MODULE_GET_FLAG_DLEN(f)	(((f) & 0xF) * 8)
/**
 * @def CAM_MODULE_GET_FLAG_ALEN
 * get address bit length config from module flags
 */
#define CAM_MODULE_GET_FLAG_ALEN(f)	((((f) >> 4) & 0xF) * 8)
/**
 * @def CAM_MODULE_GET_FLAG_EXT
 * get ohters extern config from module flags
 */
#define CAM_MODULE_GET_FLAG_EXT(f)	(((f) >> 16) & 0xFFFF)

/**
 * @def CAM_MODULE_GET_VER_MAJOR
 * get majon version info from module version
 */
#define CAM_MODULE_GET_VER_MAJOR(v)	(((v) >> 16) & 0xFF)
/**
 * @def CAM_MODULE_GET_VER_MINOR
 * get minon version info from module version
 */
#define CAM_MODULE_GET_VER_MINOR(v)	(((v) >> 8) & 0xFF)
/**
 * @def CAM_MODULE_GET_VER_PATCH
 * get pathc version info from module version
 */
#define CAM_MODULE_GET_VER_PATCH(v)	(((v) >> 0) & 0xFF)

/**
 * @enum camera_module_type_e
 * camera sub module type enum
 * @NO{S10E02C07}
 */
typedef enum camera_module_type_e {
	CAM_MODULE_INVALID = 0,
	CAM_MODULE_SENSOR,
	CAM_MODULE_CALIB,
	CAM_MODULE_DESERIAL,
	CAM_MODULE_POC,
	CAM_MODULE_TXSER,
	CAM_MODULE_UNSUPPORTED,
} camera_module_type_t;

/**
 * @def CAMERA_MODULE_TYPE_NAMES
 * get pathc version info from module version
 */
#define CAMERA_MODULE_TYPE_NAMES { \
	"invalid", \
	"sensor", \
	"calib", \
	"deserial", \
	"poc", \
	"txser", \
	"unsupported", \
}

/**
 * @struct camera_ko_version_s
 * camera sub module ko version struct
 * @NO{S10E02C07}
 */
typedef struct camera_ko_version_s {
	uint32_t major;
	uint32_t minor;
} camera_ko_version_t;

/**
 * @struct camera_module_s
 * camera sub module struct
 * @NO{S10E02C07}
 */
typedef struct camera_module_s {
	char name[CAM_MODULE_NAME_LEN];	// module name string, eg: ar0820.
	uint32_t magic;			// magic code to verify module, see: CAM_MODULE_MAGIC_CODE.
	uint32_t version;		// module version, see: CAM_MODULE_VERSION.
	uint16_t module_type;		// module type, see: camera_module_type_e.
	uint16_t module_size;		// module struct whole size to check.
	uint16_t info_size;		// module main info struct size to check.
	uint16_t data_size;		// module so_data valid size.
	uint32_t flags;			// module flags.
	camera_ko_version_t ko_version;	// module depend ko version;
	uint32_t reserved[14];		// reserved.
	const char *so_version;		// so file version, as: "1.0.0" for libxxx.1.0.0.so.
	const void *so_data[CAM_MODULE_DATA_MAX]; // so private data, as: emode, ops, etc.
} camera_module_t;

/**
 * @def CAMERA_MODULE_HEADER
 * all sub standard module struct should use this as header.
 * use union for compatibility to the legacy version module source.
 */
#define CAMERA_MODULE_HEADER \
    union { \
	const char *module; \
	const camera_module_t *module_st; \
    }

/**
 * @struct camera_module_lib_s
 * camera sub module lib so process struct
 * @NO{S10E02C07}
 */
typedef struct camera_module_lib_s {
	char so_name[CAM_MODULE_SOLIB_LEN]; // module solib name string, eg: libar0820.so.
	void *so_fd;			// so dlopen fd.
	const camera_module_t *module;	// module struct pointer.
	void *body;			// module body struct pointer.
} camera_module_lib_t;

/**
 * @def CAMERA_MODULE_T
 * covert point to camera_module_t struct pointer
 */
#define CAMERA_MODULE_T(m)		((camera_module_t *)(m))

/**
 * @def CAMERA_MODULE_CHECK_VALID
 * check point if a valid module struct pointer by magic code
 */
#define CAMERA_MODULE_CHECK_VALID(m)	(((m) != NULL) && (CAMERA_MODULE_T(m)->magic == CAM_MODULE_MAGIC_CODE))
/**
 * @def CAMERA_MODULE_CHECK_VERSION
 * check point if a valid module struct pointer with matched major version
 */
#define CAMERA_MODULE_CHECK_VERSION(m)	(CAMERA_MODULE_CHECK_VALID(m) && \
			(CAM_MODULE_GET_VER_MAJOR((CAMERA_MODULE_T(m)->version)) == CAM_MODULE_VERSION_MAJOR))
/**
 * @def CAMERA_MODULE_CHECK
 * check point if a valid module struct pointer with more verify
 */
#define CAMERA_MODULE_CHECK(m, t, s, i)	(CAMERA_MODULE_CHECK_VERSION(m) && \
			(CAMERA_MODULE_T(m)->module_type == (t)) && \
			(CAMERA_MODULE_T(m)->module_size == sizeof(s)) && \
			(CAMERA_MODULE_T(m)->info_size == sizeof(i)))

/**
 * @def CAMERA_MODULE_GET_FLAGS
 * get flags value from a module struct pointer
 */
#define CAMERA_MODULE_GET_FLAGS(m)	(CAMERA_MODULE_CHECK_VERSION(m) ? (CAMERA_MODULE_T(m)->flags) : 0U)
/**
 * @def CAMERA_MODULE_GET_KO_VERSION
 * get so version string from a module struct pointer
 */
#define CAMERA_MODULE_GET_KO_VERSION(m)	(CAMERA_MODULE_CHECK_VERSION(m) ? &(CAMERA_MODULE_T(m)->ko_version) : NULL)
/**
 * @def CAMERA_MODULE_GET_VERSION
 * get so version string from a module struct pointer
 */
#define CAMERA_MODULE_GET_VERSION(m)	(CAMERA_MODULE_CHECK_VERSION(m) ? (CAMERA_MODULE_T(m)->so_version) : NULL)
/**
 * @def CAMERA_MODULE_GET_DATA
 * get so data pointer from a module struct pointer
 */
#define CAMERA_MODULE_GET_DATA(m, i)	((CAMERA_MODULE_CHECK_VERSION(m) && (i < CAMERA_MODULE_T(m)->data_size)) ? \
			(CAMERA_MODULE_T(m)->so_data[i]) : NULL)


/**
 * @def CAMERA_MODULE
 * define a module struct with initialization config
 */
#define CAMERA_MODULE(n, t, m, i, f, kva, kvi, v, d, d0, d1, d2, d3) \
const camera_module_t cammod_##n = { \
	.name = #n, \
	.magic = CAM_MODULE_MAGIC_CODE, \
	.version = CAM_MODULE_VERSION, \
	.module_type = (uint16_t)t, \
	.module_size = sizeof(m), \
	.info_size = sizeof(i), \
	.data_size = (d), \
	.flags = (f), \
	.ko_version = { (kva), (kvi) }, \
	.so_version = (v), \
	.so_data = { \
		(const void *)(d0), \
		(const void *)(d1), \
		(const void *)(d2), \
		(const void *)(d3) }, \
	.reserved = { 0 }, \
}

/**
 * @def CAMERA_MODULE_NAME
 * get the module name string from CAM_MODULE() defined module struct
 */
#define CAMERA_MODULE_NAME(n)              (cammod_##n.name)

/* internal apis */
extern int32_t camera_module_lib_pre(const char *name, camera_module_type_t type, camera_module_lib_t *lib);
void camera_module_lib_post(camera_module_lib_t *lib);

#ifdef __cplusplus
}
#endif

#endif /* __CAM_MODULE_H__ */


