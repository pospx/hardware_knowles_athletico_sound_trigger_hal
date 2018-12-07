/*
 * Copyright (C) 2018 Knowles Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "SoundTriggerHALUtil"
#define LOG_NDEBUG 0

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <cutils/log.h>

#include <errno.h>
#include <linux/errno.h>
#include <sys/ioctl.h>

#include "cvq_ioctl.h"

#define HOTWORD_EVT_SRC_ID            IAXXX_SYSID_PLUGIN_INSTANCE_0
#define OSLO_EVT_SRC_ID               IAXXX_SYSID_PLUGIN_INSTANCE_3
#define CHRE_EVT_SRC_ID               IAXXX_SYSID_PLUGIN_INSTANCE_6
#define AMBIENT_ENTITY_EVT_SRC_ID     IAXXX_SYSID_PLUGIN_INSTANCE_5

#define HOTWORD_PKG_ID      11
#define HOTWORD_PLUGIN_IDX  0
#define HOTWORD_INSTANCE_ID 0
#define HOTWORD_PRIORITY    1

#define AMBIENT_ENTITY_PKG_ID          12
#define AMBIENT_ENTITY_PLUGIN_IDX      0
#define AMBIENT_ENTITY_INSTANCE_ID     5
#define AMBIENT_ENTITY_PRIORITY        1

#define SENSOR_PKG_ID           0
#define SENSOR_PLUGIN_IDX       0
#define SENSOR_INSTANCE_ID      3
#define SENSOR_PRIORITY         1
#define SENSOR_PRESENCE_MODE    0
#define SENSOR_DETECTED_MODE    1
#define SENSOR_MAX_MODE         2

#define AEC_PKG_ID       7
#define AEC_PLUGIN_IDX   0
#define AEC_BLOCK_ID     1
#define AEC_INSTANCE_ID  7
#define AEC_PRIORITY     1

#define CHRE_PLUGIN_IDX      0
#define CHRE_INSTANCE_ID     6
#define CHRE_EVT_ID          3
#define CHRE_EVT_PARAM_ID    8

#define BUF_PACKAGE_ID      4
#define BUF_PLUGIN_IDX      0
#define BUF_PRIORITY        1

#define HOTWORD_BUF_INSTANCE_ID     1
#define OSLO_BUF_INSTANCE_ID        2
#define AMBIENT_BUF_INSTANCE_ID     4

#define HOTWORD_DETECTION   0
#define AMBIENT_DETECTION   1
#define ENTITY_DETECTION    2

#define HOTWORD_SLOT_ID     1
#define AMBIENT_SLOT_ID     3
#define ENTITY_SLOT_ID      4

#define HOTWORD_UNLOAD_PARAM_ID             1
#define AMBIENT_ENTITY_UNLOAD_PARAM_ID      1
#define AMBIENT_ENTITY_RESET_PARAM_ID       2

#define HOTWORD_UNLOAD_PARAM_VAL            1
#define AMBIENT_UNLOAD_PARAM_VAL            3
#define ENTITY_UNLOAD_PARAM_VAL             4
#define AMBIENT_ENTITY_RESET_PARAM_VAL      3

#define BUFFER_PACKAGE              "BufferPackage.bin"
#define BUFFER_CONFIG_VAL           "BufferConfigVal.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_AMBIENT_VAL   "BufferConfigValAmbient.bin"
#define OK_GOOGLE_PACKAGE           "OkGooglePackage.bin"
#define AMBIENT_HOTWORD_PACKAGE     "AmbientHotwordPackage.bin"
#define SENSOR_PACKAGE              "OsloSensorPackage.bin"
#define AEC_PASSTHROUGH_PACKAGE     "PassthruPackage.bin"

#define MIC_ROUTE                            "mic1-route"
#define BARGEIN_ROUTE                        "bargein-route"
#define SENSOR_ROTUE                         "oslo-route"
#define HOTWORD_WITH_BARGEIN_ROUTE           "hotword-route-with-bargein"
#define HOTWORD_WITHOUT_BARGEIN_ROUTE        "hotword-route-without-bargein"
#define CHRE_WITH_BARGEIN_ROUTE              "chre-route-with-bargein"
#define CHRE_WITHOUT_BARGEIN_ROUTE           "chre-route-without-bargein"
#define AMBIENT_ENTITY_WITH_BARGEIN_ROUTE    "ambient-entity-route-with-bargein"
#define AMBIENT_ENTITY_WITHOUT_BARGEIN_ROUTE "ambient-entity-route-without-bargein"

int write_model(struct iaxxx_odsp_hw *odsp_hdl, unsigned char *data,
                int length, int kw_type)
 {
    int err = 0;

    switch(kw_type)
    {
        case 0: //HOTWORD
            ALOGV("+%s+ OK_GOOGLE_KW_ID", __func__);
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                            HOTWORD_INSTANCE_ID, 0,
                                            0, IAXXX_HMD_BLOCK_ID);
            if (err < 0) {
                ALOGE("%s: Failed to set hotword plgin reset param %s\n",
                    __func__, strerror(errno));
                goto exit;
            }

            err = iaxxx_odsp_plugin_set_parameter_blk(odsp_hdl,
                                        HOTWORD_INSTANCE_ID, HOTWORD_SLOT_ID,
                                        IAXXX_HMD_BLOCK_ID, data, length);
            break;
        case 1: //AMBIENT
            ALOGV("+%s+ AMBIENT_KW_ID", __func__);
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                            AMBIENT_ENTITY_INSTANCE_ID, 0,
                                            0, IAXXX_HMD_BLOCK_ID);
            if (err < 0) {
                ALOGE("%s: Failed to set ambient plgin reset param %s\n",
                    __func__, strerror(errno));
                goto exit;
            }

            err = iaxxx_odsp_plugin_set_parameter_blk(odsp_hdl,
                                    AMBIENT_ENTITY_INSTANCE_ID, AMBIENT_SLOT_ID,
                                    IAXXX_HMD_BLOCK_ID, data, length);
            break;
        case 2: //ENTITY
            ALOGV("+%s+ Entity_KW_ID", __func__);
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                            AMBIENT_ENTITY_INSTANCE_ID, 0,
                                            0, IAXXX_HMD_BLOCK_ID);
            if (err < 0) {
                ALOGE("%s: Failed to set entity plgin reset param %s\n",
                    __func__, strerror(errno));
                goto exit;
            }
            err = iaxxx_odsp_plugin_set_parameter_blk(odsp_hdl,
                                    AMBIENT_ENTITY_INSTANCE_ID, ENTITY_SLOT_ID,
                                    IAXXX_HMD_BLOCK_ID, data, length);
            break;
        default:
            ALOGE("%s: Unknown KW_ID\n", __func__);
            err = -1;
            errno = -EINVAL;
            break;
    }

    if (err < 0) {
        ALOGE("%s: Failed to load the keyword with error %s\n",
            __func__, strerror(errno));
    }
exit:
    ALOGV("-%s-", __func__);
    return err;
}

int get_event(struct iaxxx_odsp_hw *odsp_hdl, struct iaxxx_get_event_info *ge)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    err = iaxxx_odsp_evt_getevent(odsp_hdl, ge);
    if (err == -1) {
        ALOGE("%s: ERROR Failed to get event with error %d(%s)",
            __func__, errno, strerror(errno));
    }

    ALOGV("-%s-", __func__);
    return err;
}

int reset_ambient_plugin(struct iaxxx_odsp_hw *odsp_hdl)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                        AMBIENT_ENTITY_INSTANCE_ID,
                                        AMBIENT_ENTITY_RESET_PARAM_ID,
                                        AMBIENT_ENTITY_RESET_PARAM_VAL,
                                        IAXXX_HMD_BLOCK_ID);
    if (err != 0) {
            ALOGE("%s: ERROR: Set param for ambient lib reset failed %d(%s)",
                __func__, errno, strerror(errno));
    }
    ALOGV("-%s-", __func__);
    return err;
}

/*
 * Provide an interface for Testing binary
 * to control route.
 */
static struct mixer* open_mixer_ctl()
{
    return mixer_open(0);
}

static void close_mixer_ctl(struct mixer *mixer)
{
    if (mixer) {
        mixer_close(mixer);
    }
}

static int set_mixer_ctl_val(struct mixer *mixer, char *id, int value)
{
    struct mixer_ctl *ctl = NULL;
    int err = 0;

    if ((mixer == NULL) || (id == NULL)) {
        ALOGE("%s: ERROR Null argument passed", __func__);
        err = -EINVAL;
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(mixer, id);
    if (ctl == NULL) {
        ALOGE("%s: ERROR Invalid control name: %s", __func__, id);
        err = -1;
        goto exit;
    }

    if (mixer_ctl_set_value(ctl, 0, value)) {
        ALOGE("%s: ERROR Invalid value for %s", __func__, id);
        err = -1;
        goto exit;
    }

exit:
    return err;
}

static int set_mixer_ctl_string(struct mixer *mixer,
                                char *id, const char *string)
{
    struct mixer_ctl *ctl = NULL;
    int err = 0;

    if ((mixer == NULL) || (id == NULL)) {
        ALOGE("%s: ERROR Null argument passed", __func__);
        err = -EINVAL;
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(mixer, id);
    if (ctl == NULL) {
        ALOGE("%s: ERROR Invalid control name: %s", __func__, id);
        err = -1;
        goto exit;
    }

    if (mixer_ctl_set_enum_by_string(ctl, string)) {
        ALOGE("%s: ERROR Invalid string for %s", __func__, id);
        err = -1;
        goto exit;
    }

exit:
    return err;
}

/*
 * Provide an interface for Oslo binary
 * to control route.
 */
int force_set_sensor_route(bool enable)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    struct mixer *mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        err = -1;
        goto exit;
    }

    if (enable) {
        set_mixer_ctl_string(mixer, "sensor0 Ip Conf", "plugin3Out1");
        set_mixer_ctl_val(mixer, "sensor0 En", 1);
        set_mixer_ctl_string(mixer, "Plgin2Ip Ep0 Conf", "SensorOut0");
        set_mixer_ctl_string(mixer, "Plgin3Ip Ep0 Conf", "plugin2Out0");
        set_mixer_ctl_val(mixer, "Plgin2Blk1En", 1);
        set_mixer_ctl_val(mixer, "Plgin3Blk1En", 1);
    } else {
        set_mixer_ctl_val(mixer, "Plgin3Blk1En", 0);
        set_mixer_ctl_val(mixer, "Plgin2Blk1En", 0);
        set_mixer_ctl_val(mixer, "sensor0 En", 0);
        set_mixer_ctl_string(mixer, "sensor0 Ip Conf", "UNKNOWN");
    }

exit:
    close_mixer_ctl(mixer);
    ALOGV("-%s-", __func__);
    return err;
}

int set_sensor_route(struct audio_route *route_hdl, bool enable)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    if (enable)
        err = audio_route_apply_and_update_path(route_hdl, SENSOR_ROTUE);
    else
        err = audio_route_reset_and_update_path(route_hdl, SENSOR_ROTUE);
    if (err)
        ALOGE("%s: route fail %d", __func__, err);

    ALOGV("-%s-", __func__);
    return err;
}

int set_ambient_entity_state(struct iaxxx_odsp_hw *odsp_hdl,
                    unsigned int current)
{
    int err = 0;
    ALOGV("+%s+ enable models %x", __func__, current & PLUGIN2_MASK);

    err = iaxxx_odsp_plugin_setevent(odsp_hdl, AMBIENT_ENTITY_INSTANCE_ID,
                                    current & PLUGIN2_MASK, IAXXX_HMD_BLOCK_ID);
    if (err < 0) {
        ALOGE("%s: ERROR: ambient_entity set event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        goto exit;
    }
    if (current & AMBIENT_MASK) {
        err = iaxxx_odsp_evt_subscribe(odsp_hdl, AMBIENT_ENTITY_EVT_SRC_ID,
                                    AMBIENT_DETECTION, IAXXX_SYSID_HOST, 0);
        if (err < 0) {
            ALOGE("%s: ERROR: Ambient subscrive event failed"
                " with error %d(%s)", __func__,
                errno, strerror(errno));
            goto exit;
        }

    }
    if (current & ENTITY_MASK) {
        err = iaxxx_odsp_evt_subscribe(odsp_hdl, AMBIENT_ENTITY_EVT_SRC_ID,
                                    ENTITY_DETECTION, IAXXX_SYSID_HOST, 0);
        if (err < 0) {
            ALOGE("%s: ERROR: Entity subscrive event failed"
                " with error %d(%s)", __func__,
                errno, strerror(errno));
            goto exit;
        }
    }

exit:
    ALOGV("-%s-", __func__);
    return err;
}

int tear_ambient_entity_state(struct iaxxx_odsp_hw *odsp_hdl,
                    unsigned int current)
{
    int err = 0;
    ALOGV("+%s+ current %x", __func__, current & PLUGIN2_MASK);
    if (current & AMBIENT_MASK) {
        err = iaxxx_odsp_evt_unsubscribe(odsp_hdl, AMBIENT_ENTITY_EVT_SRC_ID,
                                        AMBIENT_DETECTION, IAXXX_SYSID_HOST);
        if (err < 0) {
            ALOGE("%s: ERROR: Ambient unsubscrive event failed"
                " with error %d(%s)", __func__,
                errno, strerror(errno));
            goto exit;
        }
        err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                            AMBIENT_ENTITY_INSTANCE_ID,
                                            AMBIENT_ENTITY_UNLOAD_PARAM_ID,
                                            AMBIENT_UNLOAD_PARAM_VAL,
                                            IAXXX_HMD_BLOCK_ID);
        if (err < 0) {
            ALOGE("%s: ERROR: Ambient model unload failed with error %d(%s)",
                __func__, errno, strerror(errno));
            goto exit;
        }
    }
    if (current & ENTITY_MASK) {
        err = iaxxx_odsp_evt_unsubscribe(odsp_hdl, AMBIENT_ENTITY_EVT_SRC_ID,
                                        ENTITY_DETECTION, IAXXX_SYSID_HOST);
        if (err < 0) {
            ALOGE("%s: ERROR: Entity unsubscrive event failed"
                " with error %d(%s)", __func__,
                errno, strerror(errno));
            goto exit;
        }
        err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                            AMBIENT_ENTITY_INSTANCE_ID,
                                            AMBIENT_ENTITY_UNLOAD_PARAM_ID,
                                            ENTITY_UNLOAD_PARAM_VAL,
                                            IAXXX_HMD_BLOCK_ID);
        if (err < 0) {
            ALOGE("%s: ERROR: Entity model unload failed with error %d(%s)",
                __func__, errno, strerror(errno));
            goto exit;
        }
    }

exit:
    ALOGV("-%s-", __func__);
    return err;
}

int set_ambient_entity_route(struct audio_route *route_hdl, bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);

    if (bargein == true)
        err = audio_route_apply_and_update_path(route_hdl,
                                        AMBIENT_ENTITY_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_apply_and_update_path(route_hdl,
                                        AMBIENT_ENTITY_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route apply fail %d", __func__, err);

    ALOGV("-%s-", __func__);
    return err;
}

int tear_ambient_entity_route(struct audio_route *route_hdl, bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);
    /* check cvq node to send ioctl */
    if (bargein == true)
        err = audio_route_reset_and_update_path(route_hdl,
                                        AMBIENT_ENTITY_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl,
                                        AMBIENT_ENTITY_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route reset fail %d", __func__, err);

    ALOGV("-%s-", __func__);
    return err;
}

int set_hotword_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl, bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);
    // Set the events and params
    err = iaxxx_odsp_plugin_setevent(odsp_hdl, HOTWORD_INSTANCE_ID,
                                    0x1, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Hotword set event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        goto exit;
    }

    ALOGD("Registering for Hotword event\n");

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, HOTWORD_EVT_SRC_ID,
                                HOTWORD_DETECTION, IAXXX_SYSID_HOST, 0);
    if (err == -1) {
        ALOGE("%s: ERROR: Hotword subscribe event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        goto exit;
    }

    if (bargein == true)
        err = audio_route_apply_and_update_path(route_hdl,
                                            HOTWORD_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_apply_and_update_path(route_hdl,
                                            HOTWORD_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route apply fail %d", __func__, err);

exit:
    ALOGV("-%s-", __func__);
    return err;
}

int tear_hotword_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl,
                    bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);
    /* check cvq node to send ioctl */
    if (bargein == true)
        err = audio_route_reset_and_update_path(route_hdl,
                                            HOTWORD_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl,
                                            HOTWORD_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route reset fail %d", __func__, err);

    err = iaxxx_odsp_evt_unsubscribe(odsp_hdl, HOTWORD_EVT_SRC_ID,
                                    HOTWORD_DETECTION, IAXXX_SYSID_HOST);
    if (err == -1) {
        ALOGE("%s: ERROR: Hotword unsubscrive event failed with error %d(%s)",
            __func__, errno, strerror(errno));
    }

    ALOGV("-%s-", __func__);
    return err;
}

int set_chre_audio_route(struct audio_route *route_hdl, bool bargein)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    if (bargein)
        err = audio_route_apply_and_update_path(route_hdl,
                                        CHRE_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_apply_and_update_path(route_hdl,
                                        CHRE_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route apply fail %d", __func__, err);

    ALOGV("-%s-", __func__);
    return err;
}

int tear_chre_audio_route(struct audio_route *route_hdl, bool bargein)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    if (bargein == true)
        err = audio_route_reset_and_update_path(route_hdl,
                                        CHRE_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl,
                                        CHRE_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route reset fail %d", __func__, err);

    ALOGV("-%s-", __func__);
    return err;
}

int sensor_event_init_params(struct iaxxx_odsp_hw *odsp_hdl)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    // Set the events and params
    err = iaxxx_odsp_plugin_setevent(odsp_hdl, SENSOR_INSTANCE_ID, 0x7,
                                    IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Sensor set event with error %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    ALOGD("Registering for 3 sensor mode switch events\n");

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, OSLO_EVT_SRC_ID,
                                SENSOR_PRESENCE_MODE, IAXXX_SYSID_SCRIPT_MGR,
                                0x1201);
    if (err == -1) {
        ALOGE("%s: ERROR: Oslo event subscription (presence mode) failed with"
            " error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, OSLO_EVT_SRC_ID,
                                SENSOR_DETECTED_MODE, IAXXX_SYSID_SCRIPT_MGR,
                                0x1202);
    if (err == -1) {
        ALOGE("%s: ERROR: Oslo event subscription (detection mode) failed with"
            " error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, OSLO_EVT_SRC_ID,
                                SENSOR_MAX_MODE, IAXXX_SYSID_HOST, 0);
    if (err == -1) {
        ALOGE("%s: ERROR: Oslo event subscription (max mode) failed with"
            " error %d(%s)", __func__, errno, strerror(errno));
    }

    ALOGV("-%s-", __func__);
    return err;
}

int flush_model(struct iaxxx_odsp_hw *odsp_hdl, int kw_type)
{
    int err = 0;

    ALOGV("+%s+", __func__);

    switch(kw_type)
    {
        case 0: //HOTWORD
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                                HOTWORD_INSTANCE_ID,
                                                HOTWORD_UNLOAD_PARAM_ID,
                                                HOTWORD_UNLOAD_PARAM_VAL,
                                                IAXXX_HMD_BLOCK_ID);
            break;
        case 1: //AMBIENT
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                                AMBIENT_ENTITY_INSTANCE_ID,
                                                AMBIENT_ENTITY_UNLOAD_PARAM_ID,
                                                AMBIENT_UNLOAD_PARAM_VAL,
                                                IAXXX_HMD_BLOCK_ID);
            break;
        case 2: //ENTITY
            err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                                AMBIENT_ENTITY_INSTANCE_ID,
                                                AMBIENT_ENTITY_UNLOAD_PARAM_ID,
                                                ENTITY_UNLOAD_PARAM_VAL,
                                                IAXXX_HMD_BLOCK_ID);
            break;
        default:
            ALOGE("%s: Unknown KW_ID\n", __func__);
            err = -1;
            errno = -EINVAL;
            break;
    }

    if (err < 0) {
        ALOGE("%s: ERROR: model unload set param failed with error %d(%s)",
            __func__, errno, strerror(errno));
    }

    ALOGV("-%s-", __func__);
    return err;
}


int setup_chip(struct iaxxx_odsp_hw *odsp_hdl)
{
    int err = 0;
    struct iaxxx_create_config_data cdata;

    ALOGV("+%s+", __func__);
    /* AMBIENT_HOTWORD_PACKAGE */
    // Download packages for ok google
    err = iaxxx_odsp_package_load(odsp_hdl, AMBIENT_HOTWORD_PACKAGE,
                                HOTWORD_PKG_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load SoundTrigger %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Download packages for ambient & entity
    err = iaxxx_odsp_package_load(odsp_hdl, AMBIENT_HOTWORD_PACKAGE,
                                AMBIENT_ENTITY_PKG_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load ENTITY %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    err = iaxxx_odsp_package_load(odsp_hdl, AEC_PASSTHROUGH_PACKAGE,
                                AEC_PKG_ID);
    if (-1 == err && EEXIST != errno) {
        ALOGE("%s: ERROR: Failed to load AEC %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    err = iaxxx_odsp_package_load(odsp_hdl, BUFFER_PACKAGE, BUF_PACKAGE_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load Buffer %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    /* Create plugins */
    cdata.type = CONFIG_FILE;
    cdata.data.fdata.filename = BUFFER_CONFIG_VAL;
    err = iaxxx_odsp_plugin_set_creation_config(odsp_hdl,
                                                HOTWORD_BUF_INSTANCE_ID,
                                                IAXXX_HMD_BLOCK_ID,
                                                cdata);
    if (err == -1) {
        ALOGE("%s: ERROR: Hotword buffer configuration %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, HOTWORD_BUF_INSTANCE_ID,
                                BUF_PRIORITY, BUF_PACKAGE_ID, BUF_PLUGIN_IDX,
                                IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Hotword Buffer%d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Hotword plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, HOTWORD_INSTANCE_ID, HOTWORD_PRIORITY,
                                HOTWORD_PKG_ID, HOTWORD_PLUGIN_IDX,
                                IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Hotword plugin%d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Ambient & Entity plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, AMBIENT_ENTITY_INSTANCE_ID,
                                AMBIENT_ENTITY_PRIORITY, AMBIENT_ENTITY_PKG_ID,
                                AMBIENT_ENTITY_PLUGIN_IDX, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Entity plugin%d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    /* SENSOR MANAGER PACKAGE LOAD AND ROUTE SETUP */
    // Download packages
    err = iaxxx_odsp_package_load(odsp_hdl, SENSOR_PACKAGE, SENSOR_PKG_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load Oslo %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    /* TODO Need to have a check if buffer package is not loaded then we have to load it here */
    /* if buffer plugin Loaded */
    /* Create plugins */
    cdata.type = CONFIG_FILE;
    cdata.data.fdata.filename = BUFFER_CONFIG_OSLO_VAL;
    err = iaxxx_odsp_plugin_set_creation_config(odsp_hdl,
                                                OSLO_BUF_INSTANCE_ID,
                                                IAXXX_HMD_BLOCK_ID,
                                                cdata);
    if (err == -1) {
        ALOGE("%s: ERROR: Oslo buffer configuration %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, OSLO_BUF_INSTANCE_ID, BUF_PRIORITY,
                                BUF_PACKAGE_ID, BUF_PLUGIN_IDX,
                                IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Oslo Buffer %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Dummy sensor plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, SENSOR_INSTANCE_ID,
                                SENSOR_PRIORITY, SENSOR_PKG_ID,
                                SENSOR_PLUGIN_IDX, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Oslo Plugin %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    err = sensor_event_init_params(odsp_hdl);
    if (err) {
        ALOGE("%s: ERROR: Sensor event init failed %d", __func__, err);
        return err;
    }
    /* SENSOR MANAGER ROUTE END */


    /* Ambient & Entity 10 sec Q15 buffer plugin */
    /* TODO Need to have a check if buffer package is not
     * loaded then we have to load it here
     */
    /* if buffer plugin Loaded */
    /* Create plugins */
    cdata.type = CONFIG_FILE;
    cdata.data.fdata.filename = BUFFER_CONFIG_AMBIENT_VAL;
    err = iaxxx_odsp_plugin_set_creation_config(odsp_hdl,
                                                AMBIENT_BUF_INSTANCE_ID,
                                                IAXXX_HMD_BLOCK_ID,
                                                cdata);
    if (err == -1) {
        ALOGE("%s: ERROR: Ambient buffer configuration %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, AMBIENT_BUF_INSTANCE_ID,
                                BUF_PRIORITY, BUF_PACKAGE_ID, BUF_PLUGIN_IDX,
                                IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Ambient Buffer %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // AEC PT Plugin Create
    err = iaxxx_odsp_plugin_create(odsp_hdl, AEC_INSTANCE_ID, AEC_PRIORITY,
                                AEC_PKG_ID, AEC_PLUGIN_IDX, AEC_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create AEC Plugin %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    /* Create CHRE plugins */
    cdata.type = CONFIG_FILE;
    cdata.data.fdata.filename = BUFFER_CONFIG_VAL;
    err = iaxxx_odsp_plugin_set_creation_config(odsp_hdl,
                                                CHRE_INSTANCE_ID,
                                                IAXXX_HMD_BLOCK_ID,
                                                cdata);
    if (err == -1) {
        ALOGE("%s: ERROR: CHRE Buffer configuration %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Create CHRE Buffer plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, CHRE_INSTANCE_ID, BUF_PRIORITY,
                                   BUF_PACKAGE_ID, CHRE_PLUGIN_IDX,
                                   IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create CHRE Buffer %d(%s)",
           __func__, errno, strerror(errno));
        return err;
    }

    /* Param ID is 8 for Buffer package
     * 60480 is in bytes calculated for 1.8sec buffer threshold
     * 640/2 = 320 for 10ms frame in Q15 format.
     * 320+16 = 336 is each frame plus tunnel header
     * 336 * 180 = 60480 which is 1.8 sec buffer in bytes
     */
    err = iaxxx_odsp_plugin_set_parameter(odsp_hdl, CHRE_INSTANCE_ID,
                                        CHRE_EVT_PARAM_ID, 60480,
                                        IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: CHRE buffer set param failed with error %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    err = iaxxx_odsp_plugin_setevent(odsp_hdl, CHRE_INSTANCE_ID,
                                    CHRE_EVT_PARAM_ID, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: CHRE set event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, CHRE_EVT_SRC_ID,
                                CHRE_EVT_ID, IAXXX_SYSID_HOST_1, 0);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
            "  IOCTL failed with error %d(%s)", __func__, CHRE_EVT_ID,
            CHRE_EVT_SRC_ID, errno, strerror(errno));
    }

    ALOGV("-%s-", __func__);
    return err;
}

int enable_bargein_route(struct audio_route *route_hdl, bool enable)
{
    int err = 0;

    ALOGV("+%s+ %d", __func__, enable);
    if (enable)
        err = audio_route_apply_and_update_path(route_hdl, BARGEIN_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl, BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route fail %d", __func__, err);;


    ALOGD("-%s-", __func__);
    return 0;
}

int enable_mic_route(struct audio_route *route_hdl, int enable)
{
    int err = 0;

    ALOGD("+%s+ %d ", __func__, enable);
    if (enable)
        err = audio_route_apply_and_update_path(route_hdl, MIC_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl, MIC_ROUTE);
    if (err)
        ALOGE("%s: route fail %d", __func__, err);

    ALOGD("-%s-", __func__);
    return err;
}

int get_entity_param_blk(struct iaxxx_odsp_hw *odsp_hdl, void *payload,
                unsigned int payload_size)
{
    int err = 0;
    err = iaxxx_odsp_plugin_get_parameter_blk(odsp_hdl,
                                            AMBIENT_ENTITY_INSTANCE_ID,
                                            IAXXX_HMD_BLOCK_ID,
                                            100, payload,
                                            payload_size);

    if (err < 0) {
        ALOGE("%s: Failed to get param blk error %s\n",
            __func__, strerror(errno));
    }
    return err;
}