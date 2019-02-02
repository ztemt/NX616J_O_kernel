#ifndef MSM_ULTRASOUND_H
#define MSM_ULTRASOUND_H

#pragma once

#include <linux/types.h>
#include <dsp/apr_audio-v2.h>
#include <linux/delay.h>

#define NUBIA_ULTRASOUND_DISABLE                0
#define NUBIA_ULTRASOUND_ENABLE                  1
#define NUBIA_ULTRASOUND_SET_PARAMS         2
#define NUBIA_ULTRASOUND_GET_PARAMS         3
#define NUBIA_ULTRASOUND_RAMP_DOWN          4

/** Param ID definition */
#define NUBIA_ULTRASOUND_PARAM_ID_UPS_DATA                   3
#define NUBIA_ULTRASOUND_PARAM_ID_CALIBRATION_DATA   11
#define NUBIA_ULTRASOUND_PARAM_ID_ENGINE_VERSION       12


#define NUBIA_ENABLE_APR_SIZE                  16
#define NUBIA_COFIG_SET_PARAM_SIZE        96

#define NUBIA_ULTRASOUND_MODULE_TX      0x0F010201
#define NUBIA_ULTRASOUND_MODULE_RX      0x0FF10202
#define ULTRASOUND_OPCODE                        0x0FF10204


#define NUBIA_GET_PARAMS_SIZE               128
#define NUBIA_SET_PARAMS_SIZE               108

/** register */
#define NUBIA_SYSTEM_CONFIGURATION      0
/** bits */
#define NUBIA_SYSTEM_CONFIGURATION_OPERATION_MODE   1
#define NUBIA_SYSTEM_CONFIGURATION_LOG_LEVEL               2

#define NUBIA_CALIBRATION                       1
#define NUBIA_CALIBRATION_STATE           0
#define NUBIA_CALIBRATION_PROFILE        1
#define NUBIA_ULTRASOUND_GAIN              2

#define NUBIA_SYSTEM_CONFIGURATION_SIZE  96
#define NUBIA_CALIBRATION_DATA_SIZE           16
#define NUBIA_VERSION_INFO_SIZE                   16


#define NUBIA_OBJ_ID_CALIBRATION_DATA   1
#define NUBIA_OBJ_ID_VERSION_INFO            2
#define NUBIA_OBJ_ID_BRANCH_INFO             3

#define NUBIA_PORT_ID                                   SLIMBUS_2_TX

struct afe_ultrasound_set_params_t {
        uint32_t  payload[NUBIA_SET_PARAMS_SIZE];
} __packed;

struct afe_ultrasound_config_command {
        struct apr_hdr                      hdr;
        struct afe_port_cmd_set_param_v2    param;
        struct afe_port_param_data_v2       pdata;
        struct afe_ultrasound_set_params_t  prot_config;
} __packed;

struct afe_ultrasound_get_params_t {
        uint32_t payload[NUBIA_GET_PARAMS_SIZE];
} __packed;

struct afe_ultrasound_get_calib {
        struct afe_port_cmd_get_param_v2   get_param;
        struct afe_port_param_data_v2      pdata;
        struct afe_ultrasound_get_params_t res_cfg;
} __packed;

struct afe_ultrasound_calib_get_resp {
        struct afe_ultrasound_get_params_t res_cfg;
} __packed;


int32_t ultrasound_apr_set(int32_t port_id, uint32_t *param_id,
        u8 *user_params, int32_t length);

int32_t nubia_process_apr_payload(uint32_t *payload);


typedef struct afe_ultrasound_state {
        atomic_t us_apr_state;
        void **ptr_apr;
        atomic_t *ptr_status;
        atomic_t *ptr_state;
        wait_queue_head_t *ptr_wait;
        int timeout_ms;
        struct afe_ultrasound_calib_get_resp *ptr_ultrasound_calib_data;
} afe_ultrasound_state_t;


struct nubia_engine_version_info {
        uint32_t major;
        uint32_t minor;
        uint32_t build;
        uint32_t revision;
};

struct nubia_shared_data_block {
        uint32_t object_id;
        size_t size;
        void *buffer;
};

struct nubia_shared_data_block *nubia_get_shared_obj(uint32_t object_id);

extern afe_ultrasound_state_t ultra_afe;

unsigned int nubia_add_platform_controls(void *platform);
int ultrasonic_data_push(const char *buffer, size_t buffer_size);

struct nubia_system_configuration {
        union {
                uint8_t reserved[NUBIA_SYSTEM_CONFIGURATION_SIZE];
        };
};


enum nubia_system_configuration_parameter_type {

        ESCPT_OPERATION_MODE = 1,
        ESCPT_CALIBRATION_STATE,
        ESCPT_ENGINE_VERSION,
        ESCPT_LOG_LEVEL,
};

struct nubia_system_configuration_parameter {
        enum nubia_system_configuration_parameter_type type;
        union {
                int32_t operation_mode;
                int32_t calibration_state;
                int32_t engine_version;
                int32_t log_level;
        };
};

struct nubia_system_configuration_parameters_cache {
        int32_t operation_mode;
        int32_t calibration_state;
        int32_t engine_version;
        int32_t log_level;
};

struct ultrasound_calibration_data {
        union {
                uint8_t reserved[NUBIA_CALIBRATION_DATA_SIZE];
        };
};

#endif
