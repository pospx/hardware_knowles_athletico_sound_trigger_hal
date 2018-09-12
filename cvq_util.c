#define LOG_TAG "SoundTriggerKnowles_HAL_util"
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
#include "ia_constants.h"

#define CVQ_NODE "/dev/iaxxx-odsp-celldrv"

static FILE *cvq_node;

#define OK_GOOGLE_EVT_SRC_ID 0x303F //"SYSID_PLUGIN_0"
#define AMBIENT_EVT_SRC_ID 0x317F  //"SYSID_PLUGIN_5"
#define OSLO_EVT_SRC_ID 0x30FF //"SYSID_PLUGIN_3"

#define DSP_VQ_PROCESS_MODE                     (0)
#define DSP_VQ_RESET                            (4)

#define CARD_NUM            (0)
#define DEV_NODE            "/dev/iaxxx-odsp-celldrv"
#define BUFFER_PACKAGE      "BufferPackage.bin"
#define BUFFER_CONFIG_VAL      "BufferConfigVal.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_AMBIENT_VAL      "BufferConfigValAmbient.bin"
#define OK_GOOGLE_PACKAGE   "OkGooglePackage.bin"
#define SOUND_TRIGGER_PACKAGE   "SoundTriggerPackage.bin"
#define SENSOR_PACKAGE   "DummySensorPackage.bin"

int start_cvq(void)
{
    ALOGE("%s: Entering", __func__);
    int err = 0;
    struct iaxxx_plugin_param pp;
    int kw_id = 0;

    ALOGI("Setting PROCESSMODE to 0");
    ALOGD("+%s+", __func__);

    pp.inst_id      = 0;
    pp.param_id     = 1;
    pp.param_val    = 1;    // 1 is detect_KW
    pp.block_id     = 1;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    //err = ioctl(fileno(cvq_node), ODSP_START_RECOGNITION, (unsigned long) &kw_id);
    //if (-1 == err) {
    //    printf("%s: ERROR: ODSP_START_RECOGNITION IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
    //}

    ALOGD("-%s-", __func__);
    return err;
}

int stop_cvq(void)
{
    ALOGE("%s: Entering", __func__);
    int err = 0;
    struct iaxxx_plugin_param pp;
    int kw_id = 0;

    ALOGI("Setting PROCESSMODE to 1");
    ALOGD("+%s+", __func__);
    pp.inst_id      = 0;
    pp.param_id     = 1;
    pp.param_val    = 0;    // 1 is detect_KW
    pp.block_id     = 1;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    //err = ioctl(fileno(cvq_node), ODSP_STOP_RECOGNITION, (unsigned long) &kw_id);
    //if (-1 == err) {
    //    printf("%s: ERROR: ODSP_STOP_RECOGNITION IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
    //}

    ALOGD("-%s-", __func__);
    return err;
}

int init_params()
{
    ALOGD("+%s+", __func__);
    /* open the cvq node to send ioctl */
    if (NULL == cvq_node) {
        if((cvq_node = fopen(CVQ_NODE, "rw")) == NULL) {
            ALOGE("file %s open for write error: %s\n", CVQ_NODE,
            strerror(errno));
            return -EIO;
        }
    }

    ALOGD("-%s-", __func__);
    return 0;
}

int write_model(unsigned char *data, int length, bool kw_type)
{
    int ret;
    struct iaxxx_plugin_param_blk ppb;

    ALOGD("+%s+", __func__);

    if (kw_type) {
        //kw_type:AMBIENT_KW_ID
        ALOGD("+%s+ AMBIENT_KW_ID", __func__);
        ppb.block_id = 1;
        ppb.inst_id = 5;
        ppb.param_size = length;
        ppb.param_blk = (uintptr_t)data;
        ppb.id = (uint32_t) 3;
    } else {
        ALOGD("+%s+ OK_GOOGLE_KW_ID", __func__);
        //kw_type:OK_GOOGLE_KW_ID
        ppb.block_id = 1;
        ppb.inst_id = 0;
        ppb.param_size = length;
        ppb.param_blk = (uintptr_t)data;
        ppb.id = (uint32_t) 1;
    }

    ret = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM_BLK, &ppb);

    if (ret < 0)
        ALOGE("err. Failed to load the keyword with error %s\n", strerror(errno));

    ALOGD("-%s-", __func__);
    return ret;
}

int flush_model(void)
{
#if 1
        ALOGE("%s: Ignore for now", __func__);
        return 0;
#else
    int rc;

    rc = ioctl(fileno(cvq_node), IA_UNLOAD_KEYWORDS, NULL);
    if (rc < 0)
        ALOGE("err. IA_UNLOAD_KEYWORDS: %s\n", strerror(errno));

    return rc;
#endif
}

int get_event(struct iaxxx_get_event *ge)
{
    int err = 0;
    ALOGD("+%s+", __func__);
    err = ioctl(fileno(cvq_node), ODSP_GET_EVENT, (unsigned long) ge);
    if (-1 == err) {
        ALOGE("%s: ERROR ODSP_GET_EVENT failed with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("-%s-", __func__);
    return err;
}

int unload_all_models()
{
    struct iaxxx_plugin_param pp;
    int err;

    pp.inst_id      = 0;
    pp.block_id     = 2;
    pp.param_id     = DSP_VQ_RESET;
    pp.param_val    = 1;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s) for DSP_VQ_RESET", __func__, errno, strerror(errno));
    }

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

    if ((NULL == mixer) || (NULL == id)) {
        ALOGE("%s: ERROR Null argument passed", __func__);
        err = -EINVAL;
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(mixer, id);
    if (NULL == ctl) {
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

static int set_mixer_ctl_string(struct mixer *mixer, char *id, const char *string)
{
    struct mixer_ctl *ctl = NULL;
    int err = 0;

    if ((NULL == mixer) || (NULL == id)) {
        ALOGE("%s: ERROR Null argument passed", __func__);
        err = -EINVAL;
        goto exit;
    }

    ctl = mixer_get_ctl_by_name(mixer, id);
    if (NULL == ctl) {
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
    struct mixer *mixer = open_mixer_ctl();
    if (NULL == mixer) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    if (enable) {
        set_mixer_ctl_val(mixer, "sensor0 En", 1);

        set_mixer_ctl_string(mixer, "Plgin2Ip Ep0 Conf", "SensorOut0");
        set_mixer_ctl_string(mixer, "Plgin3Ip Ep0 Conf", "plugin2Out0");

        set_mixer_ctl_val(mixer, "Plgin2Blk1En", 1);
        set_mixer_ctl_val(mixer, "Plgin3Blk1En", 1);
    } else {
        // TODO: Add teardown route
    }

    close_mixer_ctl(mixer);

    ALOGD("-%s-", __func__);
    return 0;
}

int set_ambient_audio_route(bool enable) {
    struct mixer *mixer = open_mixer_ctl();
    struct iaxxx_set_event se;
    struct iaxxx_evt_info ei;
    int err = 0;

    if (NULL == mixer) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }

    ALOGD("+%s+", __func__);
    if (enable) {

        /* check cvq node to send ioctl */
        if (NULL == cvq_node) {
                ALOGE("file %s is NULL\n", CVQ_NODE);
                return -EIO;
        }

        // Set the events and params
        se.inst_id           = 5;
        se.event_enable_mask = 0x7;
        se.block_id          = 1;
        err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
            return err;
        }

        ALOGD("Registering for Ambient event\n");

        // Subscribe for events
        ei.src_id       = AMBIENT_EVT_SRC_ID;
        ei.event_id     = 1;    // 1 - Keyword detection
        ei.dst_id       = IAXXX_SYSID_HOST;
        ei.dst_opaque   = 0;
        err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                        __func__, ei.event_id, ei.src_id, errno, strerror(errno));
            return err;
        }

        set_mixer_ctl_string(mixer, "Plgin5Ip Ep0 Conf", "RX0_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin5Ip Ep1 Conf", "RX1_ChanMgr");
        set_mixer_ctl_string(mixer, "Plgin5En", "Rx0Plgin5On");
        set_mixer_ctl_val(mixer, "Plgin5Blk1En", 1);

        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, "Plgin4Ip Ep0 Conf", "RX0_ChanMgr");
        set_mixer_ctl_val(mixer, "Plgin4Blk1En", 1);
        set_mixer_ctl_string(mixer, "Plgin4En", "Rx0Plgin4On");

    } else {
        set_mixer_ctl_val(mixer, "Plgin4Blk1En", 0);
        set_mixer_ctl_val(mixer, "Plgin5Blk1En", 0);

        ei.src_id       = AMBIENT_EVT_SRC_ID;
        ei.event_id     = 1;
        ei.dst_id       = IAXXX_SYSID_HOST;
        ei.dst_opaque   = 0;
        err = ioctl(fileno(cvq_node), ODSP_EVENT_UNSUBSCRIBE, (unsigned long) &ei);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_EVENT_UNSUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                                __func__, ei.event_id, ei.src_id, errno, strerror(errno));
            return err;
        }
    }
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
    se.inst_id           = 3;
    se.event_enable_mask = 0x7;
    se.block_id          = 1;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    ALOGD("Registering for 3 sensor mode switch events\n");

    // Subscribe for events
    ei.src_id       = OSLO_EVT_SRC_ID;
    ei.event_id     = 0;    // 0 - Mode switch to presence mode
    ei.dst_id       = IAXXX_SYSID_SCRIPT_MGR;
    ei.dst_opaque   = 0x1201;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                    __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    ei.src_id       = OSLO_EVT_SRC_ID;
    ei.event_id     = 1;    // 1 - Mode switch to detected mode.
    ei.dst_id       = IAXXX_SYSID_SCRIPT_MGR;
    ei.dst_opaque   = 0x1202;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                    __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    // Subscribe for events
    ei.src_id       = OSLO_EVT_SRC_ID;
    ei.event_id     = 2;    // 2 - Mode switch to max mode
    ei.dst_id       = IAXXX_SYSID_HOST;  // update this to HOST_1 for Customer
    ei.dst_opaque   = 0;
    err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                    __func__, ei.event_id, ei.src_id, errno, strerror(errno));
        return err;
    }

    return err;
}

int setup_mic_routes()
{
    struct iaxxx_plugin_info pi;
    struct iaxxx_plugin_create_cfg pcc;
    struct iaxxx_pkg_mgmt_info pkg_info;
    struct iaxxx_plugin_param pp;
    int err;
    uint32_t buffer_id;
    uint32_t ok_google_id;
    uint32_t ambient_id;
    uint32_t sensor_pkg_id;

    ALOGD("Entering setup_mic_routes");

	/* open the cvq node to send ioctl */
	if (NULL == cvq_node) {
        if((cvq_node = fopen(CVQ_NODE, "rw")) == NULL) {
            ALOGE("file %s open for write error: %s\n", CVQ_NODE,
                strerror(errno));
            return -EIO;
        }
    }
    /* SOUND_TRIGGER_PACKAGE */
    // Download packages for ok google
    strcpy(pkg_info.pkg_name, SOUND_TRIGGER_PACKAGE);
    pkg_info.pkg_id = 11;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_LOAD_PACKAGE failed %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: Ok google ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    ok_google_id = pkg_info.proc_id;

    // Download packages for ambient audio
    strcpy(pkg_info.pkg_name, SOUND_TRIGGER_PACKAGE);
    pkg_info.pkg_id = 12;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_LOAD_PACKAGE failed %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: ambient ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    ambient_id = pkg_info.proc_id;

    strcpy(pkg_info.pkg_name, BUFFER_PACKAGE);
    pkg_info.pkg_id = 4;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (-1 == err) {
        ALOGE("%s: ERROR: Buffer ODSP_LOAD_PACKAGE failed %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: Buffer ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    buffer_id = pkg_info.proc_id;

    /* Create plugins */
    strcpy(pcc.file_name, BUFFER_CONFIG_VAL);
    pcc.inst_id = 1;
    pcc.block_id = 1;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGD("%s: hot word buffer Configuration size is %u", __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx  = 0;
    pi.pkg_id = buffer_id;
    pi.block_id = 1;
    pi.inst_id  = 1;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create ok google plugin
    pi.plg_idx  = 0;
    pi.pkg_id = ok_google_id;
    pi.block_id = 1;
    pi.inst_id  = 0;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create ok google Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    /* SENSOR MANAGER PACKAGE LOAD AND ROUTE SETUP */
    // Download packages
    strcpy(pkg_info.pkg_name, SENSOR_PACKAGE);
    pkg_info.pkg_id = 0;
    pkg_info.proc_id = 0;
    err = ioctl(fileno(cvq_node), ODSP_LOAD_PACKAGE, (unsigned long) &pkg_info);
    if (-1 == err) {
        ALOGE("%s: ERROR: SENSOR ODSP_LOAD_PACKAGE failed %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGD("%s: SENSOR ODSP_LOAD_PACKAGE %x\n", __func__, pkg_info.proc_id);
    sensor_pkg_id = pkg_info.proc_id;

    /* TODO Need to have a check if buffer package is not loaded then we have to load it here */
    /* if buffer plugin Loaded */
    /* Create plugins */
    strcpy(pcc.file_name, BUFFER_CONFIG_OSLO_VAL);
    pcc.inst_id = 2;
    pcc.block_id = 1;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGD("%s: Oslo Buffer Configuration size is %u", __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx  = 0;
    pi.pkg_id = buffer_id;
    pi.block_id = 1;
    pi.inst_id  = 2;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Dummy sensor plugin
    pi.plg_idx  = 0;
    pi.pkg_id = sensor_pkg_id;
    pi.block_id = 1;
    pi.inst_id  = 3;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create sensor Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    err = sensor_event_init_params();
    if (err) {
        ALOGE("%s: ERROR: Sensor event init failed %d", __func__, err);
        return err;
    }
    /* SENSOR MANAGER ROUTE END */

    /* Ambient 10 sec Ulaw buffer plugin */
    /* TODO Need to have a check if buffer package is not loaded then we have to load it here */
    /* if buffer plugin Loaded */
    /* Create plugins */
    strcpy(pcc.file_name, BUFFER_CONFIG_AMBIENT_VAL);
    pcc.inst_id = 4;
    pcc.block_id = 1;
    pcc.cfg_size = 12;
    pcc.cfg_val = 0;

    ALOGE("%s: Ambient Buffer Configuration size is %u", __func__, pcc.cfg_size);
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_CREATE_CFG, (unsigned long) &pcc);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_CREATE_CFG IOCTL failed to set create config %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Buffer plugin
    pi.plg_idx  = 0;
    pi.pkg_id = buffer_id;
    pi.block_id = 1;
    pi.inst_id  = 4;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Buffer Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }

    // Create Ambient plugin
    pi.plg_idx  = 0;
    pi.pkg_id = ambient_id;
    pi.block_id = 1;
    pi.inst_id  = 5;
    pi.priority = BUFFER_PRIORITY;
    err = ioctl(fileno(cvq_node), ODSP_PLG_CREATE, (unsigned long) &pi);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_CREATE IOCTL failed to create Ambient Plugin with error %d(%s)", __func__, errno, strerror(errno));
        return err;
    }
    ALOGE("%s: set param for hotword", __func__);
    pp.inst_id      = 0;
    pp.block_id     = 1;
    pp.param_id     = 0;
    pp.param_val    = 0;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s) for Ok google set param", __func__, errno, strerror(errno));
    }
    ALOGE("%s: set param for ambient", __func__);
    pp.inst_id      = 5;
    pp.block_id     = 1;
    pp.param_id     = 0;
    pp.param_val    = 0;
    err = ioctl(fileno(cvq_node), ODSP_PLG_SET_PARAM, (unsigned long) &pp);
    if (-1 == err) {
        ALOGE("%s: ERROR: ODSP_PLG_SET_PARAM IOCTL failed with error %d(%s) for Ambient set param", __func__, errno, strerror(errno));
    }
    return 0;
}

int enable_mic_route(int enable)
{
    struct mixer *mixer = open_mixer_ctl();
    struct iaxxx_set_event se;
    struct iaxxx_evt_info ei;
    int err = 0;

     if (NULL == mixer) {
        ALOGE("%s: ERROR: Failed to open the mixer control", __func__);
        return -1;
    }
    ALOGD("Entering enable_mic_route %d ", enable);

    if (enable)
    {
        /* check cvq node to send ioctl */
        if (NULL == cvq_node) {
                ALOGE("file %s is NULL\n", CVQ_NODE);
                return -EIO;
        }

        // Set the events and params
        se.inst_id           = 0;
        se.event_enable_mask = 0x7;
        se.block_id          = 1;
        err = ioctl(fileno(cvq_node), ODSP_PLG_SET_EVENT, (unsigned long) &se);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_PLG_SET_EVENT IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
            return err;
        }

        ALOGD("Registering for ok google event\n");

        // Subscribe for events
        ei.src_id       = OK_GOOGLE_EVT_SRC_ID;
        ei.event_id     = 0;    // 0 - Keyword detection
        ei.dst_id       = IAXXX_SYSID_HOST;
        ei.dst_opaque   = 0;
        err = ioctl(fileno(cvq_node), ODSP_EVENT_SUBSCRIBE, (unsigned long) &ei);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_EVENT_SUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                        __func__, ei.event_id, ei.src_id, errno, strerror(errno));
            return err;
        }

        // Setup the PDM port config
        set_mixer_ctl_val(mixer, PORTB_MICBIAS, 1);
        set_mixer_ctl_val(mixer, PORTC_MICBIAS, 1);
        set_mixer_ctl_string(mixer, PDM_BCLK, "IAXXX_PDM_CLK_3P_072MHZ");
        set_mixer_ctl_string(mixer, PDM_PORT_ACLK, "IAXXX_AUD_PORT_32K");
        set_mixer_ctl_val(mixer, PDM_PORTB_SETUP,1);
        set_mixer_ctl_val(mixer, PDM_PORTC_SETUP,1);
        set_mixer_ctl_val(mixer, "Pdm CDC Setup", 1);
        set_mixer_ctl_val(mixer, PDM_PORTC_DMIC0_EN, 1);
        set_mixer_ctl_val(mixer, PDM_PORTB_DMIC0_EN, 1);
        set_mixer_ctl_val(mixer, PDM_HOS, 0x50);
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

        set_mixer_ctl_string(mixer, PLGIN0IP_EP0_CONF, "RX0_ChanMgr");
        set_mixer_ctl_string(mixer, PLGIN0IP_EP1_CONF, "RX1_ChanMgr");
        set_mixer_ctl_string(mixer, PLGIN0EN, "Rx0Plgin0On");
        set_mixer_ctl_val(mixer, PLGIN0BLK1EN, 1);

        // Buffer Plugin OP End point
        set_mixer_ctl_string(mixer, PLGIN1IP_EP0_CONF, "RX0_ChanMgr");
        set_mixer_ctl_val(mixer, PLGIN1BLK1EN, 1);
        set_mixer_ctl_string(mixer, PLGIN1EN, "Rx0Plgin1On");
        set_mixer_ctl_string(mixer, "Rx0 Mux Port", "PDMI4");
        set_mixer_ctl_val(mixer, "Route Status", 1);
        set_mixer_ctl_val(mixer, "strm0 En", 1);
        set_mixer_ctl_val(mixer, "strm1 En", 1);
    }
    else {
        set_mixer_ctl_val(mixer, "strm1 En", 0);
        set_mixer_ctl_val(mixer, "strm0 En", 0);
        set_mixer_ctl_val(mixer, PLGIN1BLK1EN, 0);
        set_mixer_ctl_val(mixer, PLGIN0BLK1EN, 0);
        set_mixer_ctl_val(mixer, "PDM CDC0 Start", 0);
        set_mixer_ctl_val(mixer, PDM_PORTB_SETUP,0);
        set_mixer_ctl_val(mixer, PDM_PORTC_SETUP,0);
        set_mixer_ctl_val(mixer, "Pdm CDC Setup", 0);
        set_mixer_ctl_val(mixer, PDM_PORTC_DMIC0_EN, 0);
        set_mixer_ctl_val(mixer, PDM_PORTB_DMIC0_EN, 0);
        set_mixer_ctl_val(mixer, PDM_HOS, 0);
        set_mixer_ctl_val(mixer, PORTC_MICBIAS, 0);
        set_mixer_ctl_val(mixer, PORTB_MICBIAS, 0);

        ei.src_id       = OK_GOOGLE_EVT_SRC_ID;
        ei.event_id     = 0;
        ei.dst_id       = IAXXX_SYSID_HOST;
        ei.dst_opaque   = 0;
        err = ioctl(fileno(cvq_node), ODSP_EVENT_UNSUBSCRIBE, (unsigned long) &ei);
        if (-1 == err) {
            ALOGE("%s: ERROR: ODSP_EVENT_UNSUBSCRIBE (for event_id %d, src_id %d) IOCTL failed with error %d(%s)",
                                __func__, ei.event_id, ei.src_id, errno, strerror(errno));
            return err;
        }
    }

    close_mixer_ctl(mixer);
    return 0;
}
