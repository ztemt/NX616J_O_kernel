
#define DEBUG
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

int msm_routing_set_ultrasound_enable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int msm_routing_get_ultrasound_enable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_system_configuration_param_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_system_configuration_param_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_calibration_param_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_calibration_param_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_calibration_data_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);
int nubia_calibration_data_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol);


static struct nubia_system_configuration_parameters_cache nubia_system_configuration_cache = { 0, 0 };

static struct nubia_engine_version_info nubia_engine_version_cache = { 0xde, 0xad, 0xbe, 0xef };

static struct ultrasound_calibration_data ultrasound_calibration_data_cache = { .reserved = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0} };

static struct nubia_shared_data_block shared_data_blocks[] = {
        { NUBIA_OBJ_ID_CALIBRATION_DATA, NUBIA_CALIBRATION_DATA_SIZE, &ultrasound_calibration_data_cache },
        { NUBIA_OBJ_ID_VERSION_INFO,     NUBIA_VERSION_INFO_SIZE,     &nubia_engine_version_cache },
};


static const size_t NUM_SHARED_RW_OBJS = sizeof(shared_data_blocks) / sizeof(struct nubia_shared_data_block);

struct nubia_shared_data_block *nubia_get_shared_obj(uint32_t object_id)
{
        size_t i;

        for (i = 0; i < NUM_SHARED_RW_OBJS; ++i) {
        if (shared_data_blocks[i].object_id == object_id)
            return &shared_data_blocks[i];
        }

        return NULL;
}

static const char * const ultrasound_enable_texts[] = {"Off", "On"};

static const struct soc_enum nubia_enum[] = {
        SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(ultrasound_enable_texts),
        ultrasound_enable_texts),
};

static const struct snd_kcontrol_new ultrasound_filter_mixer_controls[] = {
        SOC_ENUM_EXT("Ultrasound Enable",
        nubia_enum[0],
        msm_routing_get_ultrasound_enable,
        msm_routing_set_ultrasound_enable),
        SOC_SINGLE_EXT("Ultrasound Mode",
        NUBIA_SYSTEM_CONFIGURATION,
        NUBIA_SYSTEM_CONFIGURATION_OPERATION_MODE,
        255,
        0,
        nubia_system_configuration_param_get,
        nubia_system_configuration_param_put),
        SOC_SINGLE_EXT("Ultrasound Calibration State",
        NUBIA_CALIBRATION,
        NUBIA_CALIBRATION_STATE,
        256,
        0,
        nubia_calibration_param_get,
        nubia_calibration_param_put),
        SND_SOC_BYTES_EXT("Ultrasound Calibration Data",
        NUBIA_CALIBRATION_DATA_SIZE,
        nubia_calibration_data_get,
        nubia_calibration_data_put),
        SOC_SINGLE_EXT("Ultrasound Log Level",
        NUBIA_SYSTEM_CONFIGURATION,
        NUBIA_SYSTEM_CONFIGURATION_LOG_LEVEL,
        7,
        0,
        nubia_system_configuration_param_get,
        nubia_system_configuration_param_put),
};


static uint32_t ultrasound_enable_cache;

int msm_routing_get_ultrasound_enable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        ucontrol->value.integer.value[0] = ultrasound_enable_cache;
        return 0;
}

int msm_routing_set_ultrasound_enable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        int32_t ret = 0;

        ultrasound_enable_cache = ucontrol->value.integer.value[0];

        msm_pcm_routing_acquire_lock();
        ret = ultrasound_apr_set(NUBIA_PORT_ID, &ultrasound_enable_cache, NULL, 0);
        msm_pcm_routing_release_lock();
        return 0;
}


int nubia_system_configuration_param_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

        pr_debug("%s: reg: %d shift: %d\n", __func__, mc->reg, mc->shift);

        if (mc->reg != NUBIA_SYSTEM_CONFIGURATION)
                return -EINVAL;

        switch (mc->shift) {
                case NUBIA_SYSTEM_CONFIGURATION_OPERATION_MODE:
                ucontrol->value.integer.value[0] =
                nubia_system_configuration_cache.operation_mode;
                break;
        case NUBIA_SYSTEM_CONFIGURATION_LOG_LEVEL:
                ucontrol->value.integer.value[0] =
                nubia_system_configuration_cache.log_level;
                break;

        default:
                return -EINVAL;
        }
        return 1;
}

int nubia_system_configuration_param_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
        struct nubia_system_configuration_parameter param;
        uint32_t param_id = NUBIA_ULTRASOUND_SET_PARAMS;

        pr_debug("%s: reg: %d shift: %d\n", __func__, mc->reg, mc->shift);

        if (mc->reg != NUBIA_SYSTEM_CONFIGURATION)
                return -EINVAL;

        switch (mc->shift) {
        case NUBIA_SYSTEM_CONFIGURATION_OPERATION_MODE:
                nubia_system_configuration_cache.operation_mode = ucontrol->value.integer.value[0];
                param.type = ESCPT_OPERATION_MODE;
                param.operation_mode = nubia_system_configuration_cache.operation_mode;
                break;
        case NUBIA_SYSTEM_CONFIGURATION_LOG_LEVEL:
                nubia_system_configuration_cache.log_level = ucontrol->value.integer.value[0];
                param.type = ESCPT_LOG_LEVEL;
                param.log_level = nubia_system_configuration_cache.log_level;
                break;


        default:
                return -EINVAL;
        }

        return ultrasound_apr_set(NUBIA_PORT_ID, &param_id, (u8 *)&param, sizeof(param));
        
}

int nubia_calibration_param_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;

        pr_debug("%s: reg: %d shift: %d \n", __func__, mc->reg, mc->shift);

        if (mc->reg != NUBIA_CALIBRATION)
                return -EINVAL;

        switch (mc->shift) {
        case NUBIA_CALIBRATION_STATE:
                ucontrol->value.integer.value[0] = nubia_system_configuration_cache.calibration_state;
                break;

        default:
                return -EINVAL;
        }

        return 1;
}

int nubia_calibration_param_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
        struct nubia_system_configuration_parameter param;
        uint32_t param_id = NUBIA_ULTRASOUND_SET_PARAMS;

        if (mc->reg != NUBIA_CALIBRATION)
        return -EINVAL;

        switch (mc->shift) {
        case NUBIA_CALIBRATION_STATE:
                nubia_system_configuration_cache.calibration_state = ucontrol->value.integer.value[0];
                param.type = ESCPT_CALIBRATION_STATE;
                param.calibration_state =
                nubia_system_configuration_cache.calibration_state;
                pr_debug("%s: set calibration state:%ld\n", __func__, ucontrol->value.integer.value[0]);
                break;

        default:
                return -EINVAL;
        }

        return ultrasound_apr_set(NUBIA_PORT_ID, &param_id, (u8 *)&param, sizeof(param));
}

int nubia_calibration_data_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
        pr_debug("%s: reg: %d shift: %d \n", __func__, mc->reg, mc->shift);
        memcpy(ucontrol->value.bytes.data, &ultrasound_calibration_data_cache, NUBIA_CALIBRATION_DATA_SIZE);

        return 0;
}

int nubia_calibration_data_put(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
        struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
        pr_debug("%s: reg: %d shift: %d \n", __func__, mc->reg, mc->shift);
        return 0;
}

unsigned int nubia_add_platform_controls(void *platform)
{
        const unsigned int num_controls = ARRAY_SIZE(ultrasound_filter_mixer_controls);

        if (platform != NULL) {
                snd_soc_add_platform_controls((struct snd_soc_platform *)platform,
                                                                        ultrasound_filter_mixer_controls,
                                                                        num_controls);
        } else {
                pr_err("[ULTRASOUND]: pointer is NULL %s\n", __func__);
        }

        return num_controls;
}
