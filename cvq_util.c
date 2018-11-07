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

#define HOTWORD_EVT_SRC_ID    IAXXX_SYSID_PLUGIN_INSTANCE_0
#define AMBIENT_EVT_SRC_ID    IAXXX_SYSID_PLUGIN_INSTANCE_5
#define OSLO_EVT_SRC_ID       IAXXX_SYSID_PLUGIN_INSTANCE_3

#define HOTWORD_PKG_ID      11
#define HOTWORD_PLUGIN_IDX  0
#define HOTWORD_INSTANCE_ID 0
#define HOTWORD_PRIORITY    1
#define HOTWORD_DETECTION   0
#define HOTWORD_SLOT_ID     1

#define AMBIENT_PKG_ID          12
#define AMBIENT_PLUGIN_IDX      0
#define AMBIENT_INSTANCE_ID     5
#define AMBIENT_PRIORITY        1
#define AMBIENT_DETECTION       1
#define AMBIENT_SLOT_ID         3
#define AMBIENT_RESET_PARAM_ID  2
#define AMBIENT_RESET_PARAM_VAL 0

#define BUF_PACKAGE_ID      4
#define BUF_PLUGIN_IDX      0
#define BUF_PRIORITY        1

#define HOTWORD_BUF_INSTANCE_ID     1
#define OSLO_BUF_INSTANCE_ID        2
#define AMBIENT_BUF_INSTANCE_ID     4

#define SENSOR_PKG_ID           0
#define SENSOR_PLUGIN_IDX       0
#define SENSOR_INSTANCE_ID      3
#define SENSOR_PRIORITY         1
#define SENSOR_PRESENCE_MODE    0
#define SENSOR_DETECTED_MODE    1
#define SENSOR_MAX_MODE         2

#define CHRE_PLUGIN_IDX      0
#define CHRE_INSTANCE_ID     6

#define SND_MODEL_UNLOAD_PARAM_ID   5
#define HOTWORD_UNLOAD_PARAM_VAL    1
#define AMBIENT_UNLOAD_PARAM_VAL    3

#define AEC_PKG_ID       7
#define AEC_PLUGIN_IDX   0
#define AEC_BLOCK_ID     1
#define AEC_INSTANCE_ID  7
#define AEC_PRIORITY     1

#define BUFFER_PACKAGE              "BufferPackage.bin"
#define BUFFER_CONFIG_VAL           "BufferConfigVal.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_AMBIENT_VAL   "BufferConfigValAmbient.bin"
#define OK_GOOGLE_PACKAGE           "OkGooglePackage.bin"
#define SOUND_TRIGGER_PACKAGE       "SoundTriggerPackage.bin"
#define SENSOR_PACKAGE              "DummySensorPackage.bin"
#define AEC_PASSTHROUGH_PACKAGE     "PassthruPackage.bin"

#define MIC_ROUTE                           "mic1-route"
#define BARGEIN_ROUTE                       "bargein-route"
#define SENSOR_ROTUE                        "oslo-route"
#define AMBIENT_AUDIO_WITH_BARGEIN_ROUTE    "ambient-audio-route-with-bargein"
#define AMBIENT_AUDIO_WITHOUT_BARGEIN_ROUTE "ambient-audio-route-without-bargein"
#define HOTWORD_WITH_BARGEIN_ROUTE          "hotword-route-with-bargein"
#define HOTWORD_WITHOUT_BARGEIN_ROUTE       "hotword-route-without-bargein"
#define CHRE_WITH_BARGEIN_ROUTE             "chre-route-with-bargein"
#define CHRE_WITHOUT_BARGEIN_ROUTE          "chre-route-without-bargein"

int write_model(struct iaxxx_odsp_hw *odsp_hdl, unsigned char *data,
                int length, bool kw_type)
{
    int err = 0;

    if (kw_type) {
        ALOGV("+%s+ AMBIENT_KW_ID", __func__);
        err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                                        AMBIENT_INSTANCE_ID, 0,
                                        0, IAXXX_HMD_BLOCK_ID);
        if (err < 0) {
            ALOGE("%s: Failed to set ambient plgin reset param %s\n",
                __func__, strerror(errno));
            goto exit;
        }

        err = iaxxx_odsp_plugin_set_parameter_blk(odsp_hdl,
                                        AMBIENT_INSTANCE_ID, AMBIENT_SLOT_ID,
                                        IAXXX_HMD_BLOCK_ID, data, length);
    } else {
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
                                        AMBIENT_INSTANCE_ID,
                                        AMBIENT_RESET_PARAM_ID,
                                        AMBIENT_RESET_PARAM_VAL,
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

int set_ambient_audio_route(struct iaxxx_odsp_hw *odsp_hdl,
                            struct audio_route *route_hdl,
                            bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);
    // Set the events and params
    err = iaxxx_odsp_plugin_setevent(odsp_hdl, AMBIENT_INSTANCE_ID,
                                0x2, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Ambient set event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        goto exit;
    }

    ALOGD("Registering for Ambient event\n");

    // Subscribe for events
    err = iaxxx_odsp_evt_subscribe(odsp_hdl, AMBIENT_EVT_SRC_ID,
                                AMBIENT_DETECTION, IAXXX_SYSID_HOST, 0);
    if (err == -1) {
        ALOGE("%s: ERROR: Ambient subscribe event failed with error %d(%s)",
            __func__, errno, strerror(errno));
        goto exit;
    }

    if (bargein == true)
        err = audio_route_apply_and_update_path(route_hdl,
                                        AMBIENT_AUDIO_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_apply_and_update_path(route_hdl,
                                        AMBIENT_AUDIO_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route apply fail %d", __func__, err);

exit:
    ALOGV("-%s-", __func__);
    return err;
}

int tear_ambient_audio_route(struct iaxxx_odsp_hw *odsp_hdl,
                            struct audio_route *route_hdl,
                            bool bargein)
{
    int err = 0;

    ALOGV("+%s bargein %d+", __func__, bargein);
    if (bargein == true)
        err = audio_route_reset_and_update_path(route_hdl,
                                        AMBIENT_AUDIO_WITH_BARGEIN_ROUTE);
    else
        err = audio_route_reset_and_update_path(route_hdl,
                                        AMBIENT_AUDIO_WITHOUT_BARGEIN_ROUTE);
    if (err)
        ALOGE("%s: route reset fail %d", __func__, err);

    err = iaxxx_odsp_evt_unsubscribe(odsp_hdl, AMBIENT_EVT_SRC_ID,
                                    AMBIENT_DETECTION, IAXXX_SYSID_HOST);
    if (err == -1) {
        ALOGE("%s: ERROR: Ambient unsubscrive event failed with error %d(%s)",
            __func__, errno, strerror(errno));
    }

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

int flush_model(struct iaxxx_odsp_hw *odsp_hdl, bool kw_type)
{
    int err = 0;

    ALOGV("+%s+", __func__);
    if (kw_type) {
        err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                    AMBIENT_INSTANCE_ID, SND_MODEL_UNLOAD_PARAM_ID,
                    AMBIENT_UNLOAD_PARAM_VAL, IAXXX_HMD_BLOCK_ID);
    } else {
        err = iaxxx_odsp_plugin_set_parameter(odsp_hdl,
                    HOTWORD_INSTANCE_ID, SND_MODEL_UNLOAD_PARAM_ID,
                    HOTWORD_UNLOAD_PARAM_VAL, IAXXX_HMD_BLOCK_ID);
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
    /* SOUND_TRIGGER_PACKAGE */
    // Download packages for ok google
    err = iaxxx_odsp_package_load(odsp_hdl, SOUND_TRIGGER_PACKAGE,
                                HOTWORD_PKG_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load SoundTrigger %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    // Download packages for ambient audio
    err = iaxxx_odsp_package_load(odsp_hdl, SOUND_TRIGGER_PACKAGE,
                                AMBIENT_PKG_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to load Ambient %d(%s)",
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

    /* Ambient 10 sec Q15 buffer plugin */
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

    // Create Ambient plugin
    err = iaxxx_odsp_plugin_create(odsp_hdl, AMBIENT_INSTANCE_ID,
                                AMBIENT_PRIORITY, AMBIENT_PKG_ID,
                                AMBIENT_PLUGIN_IDX, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed to create Ambient Plugin %d(%s)",
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

    err = iaxxx_odsp_plugin_set_parameter(odsp_hdl, HOTWORD_INSTANCE_ID, 0,
                                        0, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Ok google set param %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    err = iaxxx_odsp_plugin_set_parameter(odsp_hdl, AMBIENT_INSTANCE_ID, 0,
                                        0, IAXXX_HMD_BLOCK_ID);
    if (err == -1) {
        ALOGE("%s: ERROR: Ambient set param %d(%s)",
            __func__, errno, strerror(errno));
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