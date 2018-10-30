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
#include <tinyalsa/asoundlib.h>

#include "iaxxx-odsp.h"
#include "iaxxx-system-identifiers.h"
#include "cvq_ioctl.h"

static FILE *cvq_node;

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

#define AEC_PKG_ID       7
#define AEC_PLUGIN_IDX   0
#define AEC_BLOCK_ID     1
#define AEC_INSTANCE_ID  7
#define AEC_PRIORITY     1

#define CARD_NUM         0

#define CVQ_NODE                    "/dev/iaxxx-odsp-celldrv"
#define BUFFER_PACKAGE              "BufferPackage.bin"
#define BUFFER_CONFIG_VAL           "BufferConfigVal.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_AMBIENT_VAL   "BufferConfigValAmbient.bin"
#define OK_GOOGLE_PACKAGE           "OkGooglePackage.bin"
#define SOUND_TRIGGER_PACKAGE       "SoundTriggerPackage.bin"
#define SENSOR_PACKAGE              "DummySensorPackage.bin"
#define AEC_PASSTHROUGH_PACKAGE     "PassthruPackage.bin"

int write_model(unsigned char *data, int length, bool kw_type)
{
    int err = 0;
    struct iaxxx_plugin_param_blk ppb;
    ALOGD("+%s+", __func__);
    if (kw_type) {
        ALOGD("+%s+ AMBIENT_KW_ID", __func__);
        ppb.block_id = IAXXX_HMD_BLOCK_ID;
        ppb.inst_id = AMBIENT_INSTANCE_ID;
        ppb.param_size = length;
        ppb.param_blk = (uintptr_t)data;
        ppb.id = (uint32_t) AMBIENT_SLOT_ID;
    } else {
        ALOGD("+%s+ OK_GOOGLE_KW_ID", __func__);
        ppb.block_id = IAXXX_HMD_BLOCK_ID;
        ppb.inst_id = HOTWORD_INSTANCE_ID;
        ppb.param_size = length;
        ppb.param_blk = (uintptr_t)data;
        ppb.id = (uint32_t) HOTWORD_SLOT_ID;
    }
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM_BLK, &ppb);
    if (err < 0) {
        ALOGE("err. Failed to load the keyword with error %s\n",
            strerror(errno));
    }
    ALOGD("-%s-", __func__);
    return err;
}

int get_event(struct iaxxx_get_event *ge)
{
    int err = 0;
    ALOGD("+%s+", __func__);
    err = ioctl(fileno(cvq_node), ODSP_GET_EVENT, (unsigned long) ge);
    if (err == -1) {
        ALOGE("%s: ERROR ODSP_GET_EVENT failed with error %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    ALOGD("-%s-", __func__);
    return err;
}

int reset_ambient_plugin()
{
    struct iaxxx_plugin_param pp;
    int err = 0;

    ALOGD("+%s+ reset ambient algo lib", __func__);
    pp.inst_id = AMBIENT_INSTANCE_ID;
    pp.block_id = IAXXX_HMD_BLOCK_ID;
    pp.param_id = AMBIENT_RESET_PARAM_ID;
    pp.param_val = AMBIENT_RESET_PARAM_VAL;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM for ambient lib reset failed %d(%s)",
            __func__, errno, strerror(errno));
    }
    ALOGD("-%s-", __func__);
    return err;
}

static struct mixer* open_mixer_ctl()
{
    return mixer_open(CARD_NUM);
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

int set_sensor_route(bool enable)
{
    ALOGD("+%s+", __func__);
    int err = 0;
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
    ALOGD("-%s-", __func__);
    return err;
}

int set_ambient_audio_route(bool bargein)
{
    struct mixer *mixer;
    struct iaxxx_set_event se;
    struct iaxxx_evt_info ei;
    int err = 0;

    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s+", __func__);
    /* check cvq node to send ioctl */
    if (cvq_node == NULL) {
        ALOGE("file %s is NULL\n", CVQ_NODE);
        err = -EIO;
        goto exit;
    }

    // Set the events and params
    se.inst_id = AMBIENT_INSTANCE_ID;
    se.event_enable_mask = 0x2;
    se.block_id = IAXXX_HMD_BLOCK_ID;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with error"
            " %d(%s)", __func__, errno, strerror(errno));
        goto exit;
    }

    ALOGD("Registering for Ambient event\n");

    // Subscribe for events
    ei.src_id = AMBIENT_EVT_SRC_ID;
    ei.event_id = AMBIENT_DETECTION;    // 1 - Keyword detection
    ei.dst_id = IAXXX_SYSID_HOST;
    ei.dst_opaque = 0;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE,
                (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
            " IOCTL failed with error %d(%s)",
            __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        goto exit;
    }

    if (bargein == true) {
        set_mixer_ctl_string(mixer, "Plgin5Ip Ep0 Conf", "plugin7Out0");
        set_mixer_ctl_string(mixer, "Plgin5Ip Ep1 Conf", "plugin7Out0");

        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin4Ip Ep0 Conf", "plugin7Out0");
    } else {
        set_mixer_ctl_string(mixer, "Plgin5Ip Ep0 Conf", "RX0_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin5Ip Ep1 Conf", "RX1_ChanMgr");

        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin4Ip Ep0 Conf", "RX0_ChanMgr");
    }
    set_mixer_ctl_string(mixer, "Plgin4En", "Rx0Plgin4On");
    set_mixer_ctl_val(mixer, "Plgin4Blk1En", 1);
    set_mixer_ctl_string(mixer, "Plgin5En", "Rx0Plgin5On");
    set_mixer_ctl_val(mixer, "Plgin5Blk1En", 1);

exit:
    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return err;
}

int tear_ambient_audio_route(void)
{
    struct mixer *mixer;
    struct iaxxx_evt_info ei;
    int err = 0;

    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s+", __func__);
    /* check cvq node to send ioctl */
    if (cvq_node == NULL) {
        ALOGE("file %s is NULL\n", CVQ_NODE);
        err = -EIO;
        goto exit;
    }

    set_mixer_ctl_val(mixer, "Plgin4Blk1En", 0);
    set_mixer_ctl_val(mixer, "Plgin5Blk1En", 0);

    ei.src_id = AMBIENT_EVT_SRC_ID;
    ei.event_id = AMBIENT_DETECTION;
    ei.dst_id = IAXXX_SYSID_HOST;
    ei.dst_opaque = 0;
    err = ioctl(fileno(cvq_node),
                ODSP_EVENT_UNSUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_UNSUBSCRIBE (for event_id %d,"
            " src_id %d) IOCTL failed with error %d(%s)",
            __func__, ei.event_id, ei.src_id, errno, strerror(errno));
    }

exit:
    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return err;
}

int set_hotword_route(bool bargein)
{
    struct mixer *mixer;
    struct iaxxx_set_event se;
    struct iaxxx_evt_info ei;
    int err = 0;

    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s bargein %d+", __func__, bargein);
    /* check cvq node to send ioctl */
    if (cvq_node == NULL) {
        ALOGE("file %s is NULL\n", CVQ_NODE);
        err = -EIO;
        goto exit;
    }

    // Set the events and params
    se.inst_id = HOTWORD_INSTANCE_ID;
    se.event_enable_mask = 0x1;
    se.block_id = IAXXX_HMD_BLOCK_ID;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with "
            "  error %d(%s)", __func__, errno, strerror(errno));
        goto exit;
    }

    ALOGD("Registering for ok google event\n");

    // Subscribe for events
    ei.src_id = HOTWORD_EVT_SRC_ID;
    ei.event_id = HOTWORD_DETECTION; // 0 - Keyword detection
    ei.dst_id = IAXXX_SYSID_HOST;
    ei.dst_opaque = 0;
    err = ioctl(fileno(cvq_node),
                ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
            "  IOCTL failed with error %d(%s)",
             __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        goto exit;
    }

    if (bargein == true) {
        set_mixer_ctl_string(mixer, "Plgin0Ip Ep0 Conf", "plugin7Out0");
        set_mixer_ctl_string(mixer, "Plgin0Ip Ep1 Conf", "plugin7Out0");

        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin1Ip Ep0 Conf", "plugin7Out0");
    } else {
        set_mixer_ctl_string(mixer, "Plgin0Ip Ep0 Conf", "RX0_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin0Ip Ep1 Conf", "RX1_ChanMgr");
        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin1Ip Ep0 Conf", "RX0_ChanMgr");
    }

    set_mixer_ctl_string(mixer, "Plgin1En", "Rx0Plgin1On");
    set_mixer_ctl_string(mixer, "Plgin0En", "Rx0Plgin0On");
    set_mixer_ctl_val(mixer, "Plgin0Blk1En", 1);
    set_mixer_ctl_val(mixer, "Plgin1Blk1En", 1);

exit:
    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return err;
}

int tear_hotword_route(void)
{
    struct mixer *mixer;
    struct iaxxx_evt_info ei;
    int err = 0;

    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s+", __func__);
    /* check cvq node to send ioctl */
    if (cvq_node == NULL) {
        ALOGE("file %s is NULL\n", CVQ_NODE);
        err = -EIO;
        goto exit;
    }

    set_mixer_ctl_val(mixer, "Plgin1Blk1En", 0);
    set_mixer_ctl_val(mixer, "Plgin0Blk1En", 0);

    ei.src_id = HOTWORD_EVT_SRC_ID;
    ei.event_id = HOTWORD_DETECTION;
    ei.dst_id = IAXXX_SYSID_HOST;
    ei.dst_opaque = 0;
    err = ioctl(fileno(cvq_node),
                ODSP_EVENT_UNSUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_UNSUBSCRIBE (for event_id %d, "
            "  src_id %d) IOCTL failed with error %d(%s)",
            __func__, ei.event_id, ei.src_id, errno, strerror(errno));
    }

exit:
    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return err;
}

int set_chre_audio_route(bool bargein)
{
    struct mixer *mixer = open_mixer_ctl();

    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s+ %d", __func__, bargein);
    if (bargein) {
        // CHRE Buffer Plugin OP End point  with Bargein
        set_mixer_ctl_string(mixer, "Plgin6Ip Ep0 Conf", "plugin7Out0");
    } else {
        // CHRE Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin6Ip Ep0 Conf", "RX0_ChanMgr");
    }
    set_mixer_ctl_string(mixer, "Plgin6En", "Rx0Plgin6On");
    set_mixer_ctl_val(mixer, "Plgin6Blk1En", 1);

    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return 0;
}

int tear_chre_audio_route(void)
{
    struct mixer *mixer = open_mixer_ctl();

    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }
    ALOGD("+%s+", __func__);

    /* Disable CHRE 2 sec buffer plugin */
    set_mixer_ctl_val(mixer, "Plgin6Blk1En", 0);

    close_mixer_ctl(mixer);
    ALOGD("-%s-", __func__);
    return 0;
}

int sensor_event_init_params()
{
    struct iaxxx_set_event se;
    struct iaxxx_evt_info ei;
    int err = 0;

    ALOGD("+%s+", __func__);

    // Set the events and params
    se.inst_id = SENSOR_INSTANCE_ID;
    se.event_enable_mask = 0x7;
    se.block_id = IAXXX_HMD_BLOCK_ID;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with error %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }

    ALOGD("Registering for 3 sensor mode switch events\n");

    // Subscribe for events
    ei.src_id = OSLO_EVT_SRC_ID;
    ei.event_id = SENSOR_PRESENCE_MODE; // 0 - Mode switch to presence mode
    ei.dst_id = IAXXX_SYSID_SCRIPT_MGR;
    ei.dst_opaque = 0x1201;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
            " IOCTL failed with error %d(%s)",
            __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    ei.src_id = OSLO_EVT_SRC_ID;
    ei.event_id = SENSOR_DETECTED_MODE; // 1 - Mode switch to detected mode.
    ei.dst_id = IAXXX_SYSID_SCRIPT_MGR;
    ei.dst_opaque = 0x1202;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
              " IOCTL failed with error %d(%s)",
                __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    ei.src_id = OSLO_EVT_SRC_ID;
    ei.event_id = SENSOR_MAX_MODE; // 2 - Mode switch to max mode
    ei.dst_id = IAXXX_SYSID_HOST; // update this to HOST_1 for Customer
    ei.dst_opaque = 0;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d)"
            " IOCTL failed with error %d(%s)",
            __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    return err;
}

int setup_chip() {
    struct iaxxx_plugin_info pi;
    struct iaxxx_plugin_create_cfg pcc;
    struct iaxxx_pkg_mgmt_info pkg_info;
    struct iaxxx_plugin_param pp;
    int err = 0;
    uint32_t buffer_id;
    uint32_t ok_google_id;
    uint32_t ambient_id;
    uint32_t sensor_pkg_id;
    uint32_t aec_pt_id;

    ALOGE("Entering %s", __func__);

    /* open the cvq node to send ioctl */
    if (cvq_node == NULL) {
        if((cvq_node = fopen(CVQ_NODE, "rw")) == NULL) {
            ALOGE("file %s open for write error: %s\n", CVQ_NODE,
                strerror(errno));
            return -EIO;
        }
    }
    /* SOUND_TRIGGER_PACKAGE */
    // Download packages for ok google
    strlcpy(pkg_info.pkg_name, SOUND_TRIGGER_PACKAGE, NAME_MAX_SIZE);
    pkg_info.pkg_id = HOTWORD_PKG_ID;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_LOAD_PACKAGE failed %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }
    ALOGE("%s: ok google ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    ok_google_id = pkg_info.proc_id;

    // Download packages for ambient audio
    strlcpy(pkg_info.pkg_name, SOUND_TRIGGER_PACKAGE, NAME_MAX_SIZE);
    pkg_info.pkg_id = AMBIENT_PKG_ID;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_LOAD_PACKAGE failed %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }
    ALOGE("%s: ambient ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    ambient_id = pkg_info.proc_id;

    strlcpy(pkg_info.pkg_name, AEC_PASSTHROUGH_PACKAGE, NAME_MAX_SIZE);
    pkg_info.pkg_id = AEC_PKG_ID;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (err == -1 && EEXIST != errno) {
        ALOGE("%s: ERROR: ODSP_LOAD_PACKAGE failed %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: AEC Pass through package loaded 0x%x\n",
        __func__, pkg_info.proc_id);
    aec_pt_id = pkg_info.proc_id;

    strlcpy(pkg_info.pkg_name, BUFFER_PACKAGE, NAME_MAX_SIZE);
    pkg_info.pkg_id = BUF_PACKAGE_ID;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (err == -1) {
        ALOGE("%s: ERROR: Buffer ODSP_LOAD_PACKAGE failed %d(%s)",
             __func__, errno, strerror(errno));
        return err;
    }
    ALOGE("%s: Buffer ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    buffer_id = pkg_info.proc_id;

    /* Create plugins */
    strlcpy(pcc.file_name, BUFFER_CONFIG_VAL, NAME_MAX_SIZE);
    pcc.inst_id = HOTWORD_BUF_INSTANCE_ID;
    pcc.block_id = IAXXX_HMD_BLOCK_ID;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGE("%s: hot word buffer Configuration size is %u",
        __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node),
                ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create"
            " config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx = BUF_PLUGIN_IDX;
    pi.pkg_id = buffer_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = HOTWORD_BUF_INSTANCE_ID;
    pi.priority = BUF_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create ok google plugin
    pi.plg_idx = HOTWORD_PLUGIN_IDX;
    pi.pkg_id = ok_google_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = HOTWORD_INSTANCE_ID;
    pi.priority = HOTWORD_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create ok google"
            " Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    /* SENSOR MANAGER PACKAGE LOAD AND ROUTE SETUP */
    // Download packages
    strlcpy(pkg_info.pkg_name, SENSOR_PACKAGE, NAME_MAX_SIZE);
    pkg_info.pkg_id = SENSOR_PKG_ID;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (err == -1) {
        ALOGE("%s: ERROR: SENSOR ODSP_LOAD_PACKAGE failed %d(%s)",
            __func__, errno, strerror(errno));
        return err;
    }
    ALOGE("%s: SENSOR ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    sensor_pkg_id = pkg_info.proc_id;

    /* TODO Need to have a check if buffer package is not loaded then we have to load it here */
    /* if buffer plugin Loaded */
    /* Create plugins */
    strlcpy(pcc.file_name, BUFFER_CONFIG_OSLO_VAL, NAME_MAX_SIZE);
    pcc.inst_id = OSLO_BUF_INSTANCE_ID;
    pcc.block_id = IAXXX_HMD_BLOCK_ID;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGE("%s: OSLO Buffer Configuration size is %u", __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node),
                ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create"
            " config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx = BUF_PLUGIN_IDX;
    pi.pkg_id = buffer_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = OSLO_BUF_INSTANCE_ID;
    pi.priority = BUF_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Dummy sensor plugin
    pi.plg_idx = SENSOR_PLUGIN_IDX;
    pi.pkg_id = sensor_pkg_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = SENSOR_INSTANCE_ID;
    pi.priority = SENSOR_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create sensor Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    err = sensor_event_init_params();
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
    strlcpy(pcc.file_name, BUFFER_CONFIG_AMBIENT_VAL, NAME_MAX_SIZE);
    pcc.inst_id = AMBIENT_BUF_INSTANCE_ID;
    pcc.block_id = IAXXX_HMD_BLOCK_ID;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGE("%s: Ambient Buffer Configuration size is %u",
        __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node),
                ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create"
            " config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx = BUF_PLUGIN_IDX;
    pi.pkg_id = buffer_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = AMBIENT_BUF_INSTANCE_ID;
    pi.priority = BUF_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Ambient plugin
    pi.plg_idx = AMBIENT_PLUGIN_IDX;
    pi.pkg_id = ambient_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = AMBIENT_INSTANCE_ID;
    pi.priority = AMBIENT_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Ambient Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // AEC PT Plugin Create
    pi.plg_idx = AEC_PLUGIN_IDX;
    pi.pkg_id = aec_pt_id;
    pi.block_id = AEC_BLOCK_ID;
    pi.inst_id = AEC_INSTANCE_ID;
    pi.priority = AEC_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err == -1 && errno != EEXIST) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create AEC Plugin"
            "with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: AEC Pass through plugin created", __func__);

    /* Create CHRE plugins */
    strlcpy(pcc.file_name, BUFFER_CONFIG_VAL, NAME_MAX_SIZE);
    pcc.inst_id = CHRE_INSTANCE_ID;
    pcc.block_id = IAXXX_HMD_BLOCK_ID;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGE("%s: CHRE 2sec buffer Configuration size is %u",
        __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node),
                ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create"
            "config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create CHRE Buffer plugin
    pi.plg_idx = CHRE_PLUGIN_IDX;
    pi.pkg_id = buffer_id;
    pi.block_id = IAXXX_HMD_BLOCK_ID;
    pi.inst_id = CHRE_INSTANCE_ID;
    pi.priority = BUF_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (err== -1) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin"
            " with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    pp.inst_id = HOTWORD_INSTANCE_ID;
    pp.block_id = IAXXX_HMD_BLOCK_ID;
    pp.param_id = 0;
    pp.param_val = 0;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s) for"
            " Ok google set param", __func__, errno, strerror(errno));
    }

    pp.inst_id = AMBIENT_INSTANCE_ID;
    pp.block_id = IAXXX_HMD_BLOCK_ID;
    pp.param_id = 0;
    pp.param_val = 0;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (err == -1) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s) for"
            " Ambient set param", __func__, errno, strerror(errno));
    }
    return err;
}

int enable_bargein_route(bool enable)
{
    struct mixer *mixer;

    ALOGD("+Entering %s+ %d", __func__, enable);
    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    if (enable) {
        /* enable aec plugin */
        set_mixer_ctl_val(mixer, "PCM PortE Setup", 1);
        set_mixer_ctl_val(mixer, "PCM PortE Start", 0x1FEF);

        // STREAM 0 INPUT Channels
        set_mixer_ctl_string(mixer, "strm4 ASRC Mode", "ASRC_ENABLE");
        set_mixer_ctl_string(mixer, "strm4 Master Strm Id", "STREAMID_4");
        set_mixer_ctl_string(mixer, "strm4 Format FrLn", "16K_10MS");
        set_mixer_ctl_string(mixer, "strm4 Format Sr", "RATE_16K");
        set_mixer_ctl_string(mixer, "strm4 Format Enc", "ENCODING_AFLOAT");
        set_mixer_ctl_val(mixer, "strm4 Dir", 0);
        set_mixer_ctl_val(mixer, "strm4 inter strm delay", 0);
        set_mixer_ctl_string(mixer, "strm4 Port", "PCM4");
        set_mixer_ctl_string(mixer, "strm4 Port Enc", "ENCODING_Q15");
        set_mixer_ctl_val(mixer, "strm4 CH Mask En", 0x3F0);
        set_mixer_ctl_val(mixer,"strm4 En", 1);

        set_mixer_ctl_string(mixer, "Plgin7Ip Ep0 Conf", "RX0_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin7Ip Ep1 Conf", "RX4_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin7Ip Ep2 Conf", "RX7_ChanMgr");

        set_mixer_ctl_string(mixer, "Plgin7En", "Rx0Plgin7On");
        set_mixer_ctl_val(mixer, "Plgin7Blk1En", 1);
    } else {
        /* disable aec plugin */
        set_mixer_ctl_val(mixer, "Plgin7Blk1En", 0);
        set_mixer_ctl_val(mixer,"strm4 En", 0);

        set_mixer_ctl_val(mixer, "PCM PortE Setup", 0);
        set_mixer_ctl_val(mixer, "PCM PortE Start", 0);
    }

    close_mixer_ctl(mixer);
    ALOGD("-Exiting %s-", __func__);
    return 0;
}

int enable_mic_route(int enable)
{
    struct mixer *mixer;
    int err = 0;

    ALOGD("Entering enable_mic_route %d ", enable);

    mixer = open_mixer_ctl();
    if (mixer == NULL) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        err = -1;
        goto exit;
    }

    if (enable) {
        /* check cvq node to send ioctl */
        if (cvq_node == NULL) {
            ALOGE("file %s is NULL\n", CVQ_NODE);
            err = -EIO;
            goto exit;
        }

        // Setup the PDM port config
        set_mixer_ctl_val(mixer, "PortB MicBias", 1);
        set_mixer_ctl_val(mixer, "PortC MicBias", 1);
        set_mixer_ctl_string(mixer, "PDM BCLK", "IAXXX_PDM_CLK_3P_072MHZ");
        set_mixer_ctl_string(mixer, "PDM Port ACLK", "IAXXX_AUD_PORT_32K");
        set_mixer_ctl_val(mixer, "Pdm PortB Setup", 1);
        set_mixer_ctl_val(mixer, "Pdm PortC Setup", 1);
        set_mixer_ctl_val(mixer, "Pdm CDC Setup", 1);
        set_mixer_ctl_val(mixer, "Pdm PortC CDC DMic4 En", 1);
        set_mixer_ctl_val(mixer, "Pdm PortB CDC DMic6 En", 1);
        set_mixer_ctl_val(mixer, "PDM Hos", 0x50);
        set_mixer_ctl_val(mixer, "PDM CDC0 Start", 0x50);

        // Channel 0
        set_mixer_ctl_string(mixer, "Rx0Chan GnRmp", "STEP_2000");
        set_mixer_ctl_val(mixer, "Rx0Ch EpGain", 40);
        set_mixer_ctl_val(mixer, "Rx0Chan Gain En", 1);

        // STREAM 0 INPUT Channels
        set_mixer_ctl_string(mixer, "strm0 ASRC Mode", "REDBOX_2-1");
        set_mixer_ctl_string(mixer, "strm0 Master Strm Id", "STREAMID_0");
        set_mixer_ctl_string(mixer, "strm0 Format FrLn", "16K_10MS");
        set_mixer_ctl_string(mixer, "strm0 Format Sr", "RATE_16K");
        set_mixer_ctl_string(mixer, "strm0 Format Enc", "ENCODING_AFLOAT");
        set_mixer_ctl_val(mixer, "strm0 Dir", 0);
        set_mixer_ctl_val(mixer, "strm0 inter strm delay", 0);
        set_mixer_ctl_string(mixer, "strm0 Port", "PDMI4");
        set_mixer_ctl_string(mixer, "strm0 Port Enc", "ENCODING_Q23");
        set_mixer_ctl_val(mixer, "strm0 CH Mask En", 1);

        set_mixer_ctl_string(mixer, "strm1 ASRC Mode", "REDBOX_2-1");
        set_mixer_ctl_string(mixer, "strm1 Master Strm Id", "STREAMID_1");
        set_mixer_ctl_string(mixer, "strm1 Format FrLn", "16K_10MS");
        set_mixer_ctl_string(mixer, "strm1 Format Sr", "RATE_16K");
        set_mixer_ctl_string(mixer, "strm1 Format Enc", "ENCODING_AFLOAT");
        set_mixer_ctl_val(mixer, "strm1 Dir", 0);
        set_mixer_ctl_val(mixer, "strm1 inter strm delay", 0);
        set_mixer_ctl_string(mixer, "strm1 Port", "PDMI6");
        set_mixer_ctl_string(mixer, "strm1 Port Enc", "ENCODING_Q23");
        set_mixer_ctl_val(mixer, "strm1 CH Mask En", 2);

        set_mixer_ctl_string(mixer, "Rx0 Mux Port", "PDMI4");
        set_mixer_ctl_val(mixer, "Route Status", 1);
        set_mixer_ctl_val(mixer, "strm0 En", 1);
        set_mixer_ctl_val(mixer, "strm1 En", 1);
    } else {
        set_mixer_ctl_val(mixer, "strm1 En", 0);
        set_mixer_ctl_val(mixer, "strm0 En", 0);
        set_mixer_ctl_val(mixer, "PDM CDC0 Start", 0);
        set_mixer_ctl_val(mixer, "Pdm PortB Setup", 0);
        set_mixer_ctl_val(mixer, "Pdm PortC Setup", 0);
        set_mixer_ctl_val(mixer, "Pdm CDC Setup", 0);
        set_mixer_ctl_val(mixer, "Pdm PortC CDC DMic4 En", 0);
        set_mixer_ctl_val(mixer, "Pdm PortB CDC DMic6 En", 0);
        set_mixer_ctl_val(mixer, "PDM Hos", 0);
        set_mixer_ctl_val(mixer, "PortC MicBias", 0);
        set_mixer_ctl_val(mixer, "PortB MicBias", 0);
    }

exit:
    close_mixer_ctl(mixer);
    return err;
}