
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <sound/asound.h>
#include <sound/soc.h>
#include <sound/control.h>
#include "../asoc/msm-pcm-routing-v2.h"
#include <dsp/q6audio-v2.h>
#include <dsp/apr_audio-v2.h>
#include <dsp/msm-ultrasound.h>
//#include <linux/wakelock.h>
#include <wakelock_ultrasound.h>


static struct wake_lock ultrasound_wake_lock;


#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

int afe_ultrasound_set_calib_data(int port, int param_id, int module_id, struct afe_ultrasound_set_params_t *prot_config, uint32_t length)
{
    int ret = -EINVAL;
    int index = 0;
    struct afe_ultrasound_config_command configV;
    struct afe_ultrasound_config_command *config;

    config = &configV;
    pr_debug("[ultrasonic]: inside %s\n", __func__);
    memset(config, 0, sizeof(struct afe_ultrasound_config_command));
    if (!prot_config) {
        pr_err("%s Invalid params\n", __func__);
        goto fail_cmd;
    }
    if ((q6audio_validate_port(port) < 0)) {
        pr_err("%s invalid port %d\n", __func__, port);
        goto fail_cmd;
    }
    index = q6audio_get_port_index(port);
    config->pdata.module_id = module_id;
    config->hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
                                            APR_HDR_LEN(APR_HDR_SIZE),
                                            APR_PKT_VER);
    config->hdr.pkt_size = sizeof(struct afe_ultrasound_config_command);
    config->hdr.src_port = 0;
    config->hdr.dest_port = 0;
    config->hdr.token = index;
    config->hdr.opcode = AFE_PORT_CMD_SET_PARAM_V2;
    config->param.port_id = q6audio_get_port_id(port);
    config->param.payload_size =
                sizeof(struct afe_ultrasound_config_command) -
                sizeof(config->hdr) - sizeof(config->param);
    config->pdata.param_id = param_id;
    config->pdata.param_size = length;
    //pr_debug("[ultrasonic]: param_size %d\n", length);
    memcpy(config->prot_config.payload, prot_config, sizeof(struct afe_ultrasound_set_params_t));
    atomic_set(ultra_afe.ptr_state, 1);
    ret = apr_send_pkt(*ultra_afe.ptr_apr, (uint32_t *) config);
    if (ret < 0) {
        pr_err("%s: Setting param for port %d param[0x%x]failed\n",
                    __func__, port, param_id);
    goto fail_cmd;
    }
    ret = wait_event_timeout(ultra_afe.ptr_wait[index],
                                        (atomic_read(ultra_afe.ptr_state) == 0),
                                        msecs_to_jiffies(ultra_afe.timeout_ms*10));
    if (!ret) {
        pr_err("%s: wait_event timeout\n", __func__);
        ret = -EINVAL;
        goto fail_cmd;
    }
    if (atomic_read(ultra_afe.ptr_status) != 0) {
        pr_err("%s: config cmd failed\n", __func__);
        ret = -EINVAL;
        goto fail_cmd;
    }
    ret = 0;
    
fail_cmd:
    pr_err("%s config->pdata.param_id %x status %d\n", __func__, config->pdata.param_id, ret);
    
    return ret;
}


int32_t ultrasound_apr_set(int32_t port_id, uint32_t *param_id, u8 *user_params, int32_t length)
{
    int32_t  ret = 0;
    uint32_t module_id;

    if (port_id == NUBIA_PORT_ID)
        module_id = NUBIA_ULTRASOUND_MODULE_TX;
    else
        module_id = NUBIA_ULTRASOUND_MODULE_RX;

    switch (*param_id) {
        case NUBIA_ULTRASOUND_ENABLE:
        {
            int32_t array[4] = {1, 0, 0, 0};
            printk("enable ultrasound\n");
            wake_lock_init(&ultrasound_wake_lock, WAKE_LOCK_SUSPEND, "ultrasound_wake_lock");

            if(!wake_lock_active(&ultrasound_wake_lock))
                wake_lock(&ultrasound_wake_lock);

            ret = afe_ultrasound_set_calib_data(port_id,
                                                *param_id, module_id,
                                                (struct afe_ultrasound_set_params_t *)array,
                                                NUBIA_ENABLE_APR_SIZE);
        }
        break;
        case NUBIA_ULTRASOUND_DISABLE:
        {
            int32_t array[4] = {0, 0, 0, 0};
            printk("disable ultrasound\n");

            ret = afe_ultrasound_set_calib_data(port_id,
                                                *param_id, module_id,
                                                (struct afe_ultrasound_set_params_t *)array,
                                                NUBIA_ENABLE_APR_SIZE);
            if(wake_lock_active(&ultrasound_wake_lock))
                wake_unlock(&ultrasound_wake_lock);

            wake_lock_destroy(&ultrasound_wake_lock);
        }
        break;
        case NUBIA_ULTRASOUND_SET_PARAMS:
        {
            printk("ultrasound set params\n");
            ret = afe_ultrasound_set_calib_data(port_id,
                                                *param_id, module_id,
                                                (struct afe_ultrasound_set_params_t *)user_params,
                                                length);

        }
        break;
        
        default:
            goto fail_cmd;
    }
    return ret;

fail_cmd:
    pr_err("%s param_id %x status %d\n", __func__, *param_id, ret);
    return -1;
}

int32_t nubia_process_apr_payload(uint32_t *payload)
{
        uint32_t payload_size = 0;
        int32_t  ret = -1;
        struct nubia_shared_data_block *data_block = NULL;
        size_t copy_size = 0;
    
        if (payload[0] == NUBIA_ULTRASOUND_MODULE_TX) {
            /* payload format
            *   payload[0] = Module ID
            *   payload[1] = Param ID
            *   payload[2] = LSB - payload size
            *   MSB - reserved(TBD)
            *   payload[3] = US data payload starts from here
            */
            payload_size = payload[2] & 0xFFFF;
            /* pr_debug("[ultrasonic]: playload type=%d size = %d, data 0x%x 0x%x 0x%x ...\n",
            *   payload[1], payload_size, payload[3], payload[4], payload[5]);
            */
            switch (payload[1]) {

            case NUBIA_ULTRASOUND_PARAM_ID_CALIBRATION_DATA:
                    if (payload_size >= NUBIA_CALIBRATION_DATA_SIZE) {
                            pr_debug("ultrasonic calibration data copied to local AP cache");

                            data_block = nubia_get_shared_obj(NUBIA_OBJ_ID_CALIBRATION_DATA);
                            copy_size = min_t(size_t, data_block->size, (size_t)NUBIA_CALIBRATION_DATA_SIZE);
                            memcpy((u8 *)data_block->buffer, &payload[3], (size_t)NUBIA_CALIBRATION_DATA_SIZE);
                            ret = (int32_t)copy_size;
                    }
                    break;
            case NUBIA_ULTRASOUND_PARAM_ID_UPS_DATA:
            default:
                    if (payload_size <= sizeof(struct afe_ultrasound_calib_get_resp)) {
                            ret = ultrasonic_data_push((const char *)&payload[3], (size_t)payload_size);
                            if (ret != 0) {
                                    pr_err("[ultrasonic] : failed to push apr payload to elliptic device");
                                    return ret;
                            }
                            //printk("[ultrasonic]: sizeof(struct afe_ultrasound_calib_get_resp)=%ld\n",sizeof(struct afe_ultrasound_calib_get_resp));
                            ret = payload_size;
                    }
                    break;
            }
        } else {
            pr_debug("[ultrasonic]: Invalid Ultrasound Module ID %d\n", payload[0]);
        }
        return ret;
}

