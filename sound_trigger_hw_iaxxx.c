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

#define LOG_TAG "SoundTriggerHAL"
#define LOG_NDEBUG 0

#include <errno.h>
#include <fcntl.h>
#include <malloc.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <cutils/log.h>
#include <cutils/uevent.h>
#include <math.h>
#include <dlfcn.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/timerfd.h>

#include <hardware/hardware.h>
#include <system/sound_trigger.h>
#include <hardware/sound_trigger.h>

#include "iaxxx-odsp.h"
#include "cvq_ioctl.h"

#include "sound_trigger_intf.h"

#define ENABLE_KEYPHRASE_DETECTION

#define MAX_GENERIC_SOUND_MODELS    (3)
#define MAX_KEY_PHRASES             (1)
#define MAX_MODELS                  (MAX_GENERIC_SOUND_MODELS + MAX_KEY_PHRASES)

#define MAX_USERS                   (1)
#define MAX_BUFFER_MS               (3000)
#define POWER_CONSUMPTION           (0) // TBD
#define ST_HAL_VERSION              (1)

#define UEVENT_MSG_LEN              (1024)

#define OK_GOOGLE_KW_ID             (0)
#define AMBIENT_KW_ID               (1)

#define IAXXX_VQ_EVENT_STR          "IAXXX_VQ_EVENT"
#define IAXXX_RECOVERY_EVENT_STR    "IAXXX_RECOVERY_EVENT"

#ifdef __LP64__
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib64/hw/adnc_strm.primary.default.so"
#else
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib/hw/adnc_strm.primary.default.so"
#endif

#define HOTWORD_AUDIO_MODEL  "7038ddc8-30f2-11e6-b0ac-40a8f03d3f15"
#define SENSOR_MANAGER_MODEL "5c0c296d-204c-4c2b-9f85-e50746caf914"
#define AMBIENT_AUDIO_MODEL  "6ac81359-2dc2-4fea-a0a0-bd378ed6da4f"
#define CHRE_AUDIO_MODEL     "7e0d396d-02ac-4524-58f9-a67056ebe847"

#define HOTWORD_MODEL (0)
#define AMBIENT_MODEL (1)

static const struct sound_trigger_properties hw_properties = {
    "Knowles Electronics",      // implementor
    "Continous VoiceQ",         // description
    1,                          // version
    // Version UUID
    { 0x80f7dcd5, 0xbb62, 0x4816, 0xa931, {0x9c, 0xaa, 0x52, 0x5d, 0xf5, 0xc7}},
    MAX_MODELS,                 // max_sound_models
    MAX_KEY_PHRASES,            // max_key_phrases
    MAX_USERS,                  // max_users
    RECOGNITION_MODE_VOICE_TRIGGER | // recognition_mode
    RECOGNITION_MODE_GENERIC_TRIGGER,
    true,                       // capture_transition
    MAX_BUFFER_MS,              // max_capture_ms
    false,                      // concurrent_capture
    false,                      // trigger_in_event
    POWER_CONSUMPTION           // power_consumption_mw
};

struct model_info {
    void *recognition_cookie;
    void *sound_model_cookie;
    sound_model_handle_t model_handle;
    sound_trigger_uuid_t uuid;
    recognition_callback_t recognition_callback;
    sound_model_callback_t sound_model_callback;
    struct sound_trigger_recognition_config *config;
    int kw_id;
    sound_trigger_sound_model_type_t type;

    void *data;
    int data_sz;
    bool is_loaded;
    bool is_active;
};

struct knowles_sound_trigger_device {
    struct sound_trigger_hw_device device;
    struct model_info models[MAX_MODELS];
    sound_trigger_uuid_t authkw_model_uuid;
    pthread_t callback_thread;
    pthread_mutex_t lock;
    int opened;
    int send_sock;
    int recv_sock;
    struct sound_trigger_recognition_config *last_keyword_detected_config;

    // Information about streaming
    int is_streaming;
    void *adnc_cvq_strm_lib;
    int (*adnc_strm_open)(bool, int, int);
    size_t (*adnc_strm_read)(long, void *, size_t);
    int (*adnc_strm_close)(long);
    long adnc_strm_handle;

    sound_trigger_uuid_t hotword_model_uuid;
    sound_trigger_uuid_t sensor_model_uuid;
    sound_trigger_uuid_t ambient_model_uuid;
    sound_trigger_uuid_t chre_model_uuid;

    int last_detected_model_type;
    bool is_mic_route_enabled;

    void *audio_hal_handle;
    audio_hw_call_back_t audio_hal_cb;
    unsigned int sthal_prop_api_version;
};

/*
 * Since there's only ever one sound_trigger_device, keep it as a global so
 * that other people can dlopen this lib to get at the streaming audio.
 */
static struct knowles_sound_trigger_device g_stdev = {
    .lock = PTHREAD_MUTEX_INITIALIZER
};

static bool check_uuid_equality(sound_trigger_uuid_t uuid1,
                                sound_trigger_uuid_t uuid2)
{
    if (uuid1.timeLow != uuid2.timeLow ||
        uuid1.timeMid != uuid2.timeMid ||
        uuid1.timeHiAndVersion != uuid2.timeHiAndVersion ||
        uuid1.clockSeq != uuid2.clockSeq) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        if(uuid1.node[i] != uuid2.node[i]) {
            return false;
        }
    }

    return true;
}

bool str_to_uuid(char* uuid_str, sound_trigger_uuid_t* uuid)
{
    if (uuid_str == NULL) {
        ALOGI("Invalid str_to_uuid input.");
        return false;
    }

    int tmp[10];
    if (sscanf(uuid_str, "%08x-%04x-%04x-%04x-%02x%02x%02x%02x%02x%02x",
            tmp, tmp+1, tmp+2, tmp+3, tmp+4, tmp+5,
            tmp+6, tmp+7, tmp+8, tmp+9) < 10) {
        ALOGI("Invalid UUID, got: %s", uuid_str);
        return false;
    }
    uuid->timeLow = (unsigned int)tmp[0];
    uuid->timeMid = (unsigned short)tmp[1];
    uuid->timeHiAndVersion = (unsigned short)tmp[2];
    uuid->clockSeq = (unsigned short)tmp[3];
    uuid->node[0] = (unsigned char)tmp[4];
    uuid->node[1] = (unsigned char)tmp[5];
    uuid->node[2] = (unsigned char)tmp[6];
    uuid->node[3] = (unsigned char)tmp[7];
    uuid->node[4] = (unsigned char)tmp[8];
    uuid->node[5] = (unsigned char)tmp[9];

    return true;
}

static int find_empty_model_slot(struct knowles_sound_trigger_device *st_dev)
{
    int i = -1;
    for (i = 0; i < MAX_MODELS; i++) {
        if (st_dev->models[i].is_loaded == false)
            break;
    }

    if (i >= MAX_MODELS) {
        i = -1;
    }

    return i;
}

static int find_handle_for_kw_id(
                        struct knowles_sound_trigger_device *st_dev, int kw_id)
{
    int i = 0;
    for (i = 0; i < MAX_MODELS; i++) {
        if (kw_id == st_dev->models[i].kw_id)
            break;
    }

    return i;
}

static void reg_hal_event_session(
                                struct sound_trigger_recognition_config *config,
                                sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    struct sound_trigger_event_info event_info;
    /*
     * Register config and capture_handle of trigger sound model to audio hal
     * It only register while request capturing buffer.
     */
    if (config->capture_requested && stdev->audio_hal_cb) {
        ALOGD("%s: ST_EVENT_SESSION_REGISTER capture_handle %d model %p",
            __func__, config->capture_handle, &stdev->models[handle]);
        event_info.st_ses.p_ses = (void *)&stdev->models[handle];
        event_info.st_ses.config = stdev_hotword_pcm_config;
        event_info.st_ses.capture_handle = config->capture_handle;
        event_info.st_ses.pcm = NULL;
        stdev->audio_hal_cb(ST_EVENT_SESSION_REGISTER, &event_info);
    }
}

static void dereg_hal_event_session(
                                struct sound_trigger_recognition_config *config,
                                sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    struct sound_trigger_event_info event_info;
    /*
     * Indicate to audio hal that streaming is stopped.
     * Stop capturing data from STHAL.
     */
    if (config->capture_requested && stdev->audio_hal_cb) {
        ALOGD("%s: ST_EVENT_SESSION_DEREGISTER capture_handle %d model %p",
            __func__, config->capture_handle, &stdev->models[handle]);
        event_info.st_ses.p_ses = (void *)&stdev->models[handle];
        event_info.st_ses.capture_handle = config->capture_handle;
        event_info.st_ses.pcm = NULL;
        stdev->audio_hal_cb(ST_EVENT_SESSION_DEREGISTER, &event_info);
    }
}


static char *stdev_keyphrase_event_alloc(sound_model_handle_t handle,
                                struct sound_trigger_recognition_config *config,
                                int recognition_status)
{
    char *data;
    struct sound_trigger_phrase_recognition_event *event;
    data = (char *)calloc(1,
                        sizeof(struct sound_trigger_phrase_recognition_event));
    if (!data)
        return NULL;
    event = (struct sound_trigger_phrase_recognition_event *)data;
    event->common.status = recognition_status;
    event->common.type = SOUND_MODEL_TYPE_KEYPHRASE;
    event->common.model = handle;
    event->common.capture_available = false;

    if (config) {
        unsigned int i;

        event->num_phrases = config->num_phrases;
        if (event->num_phrases > SOUND_TRIGGER_MAX_PHRASES)
            event->num_phrases = SOUND_TRIGGER_MAX_PHRASES;
        for (i = 0; i < event->num_phrases; i++)
            memcpy(&event->phrase_extras[i],
                &config->phrases[i],
                sizeof(struct sound_trigger_phrase_recognition_extra));
    }

    event->num_phrases = 1;
    event->phrase_extras[0].confidence_level = 100;
    event->phrase_extras[0].num_levels = 1;
    event->phrase_extras[0].levels[0].level = 100;
    event->phrase_extras[0].levels[0].user_id = 0;
    /*
     * Signify that all the data is comming through streaming
     * and not through the buffer.
     */
    event->common.capture_available = true;
    event->common.capture_delay_ms = 0;
    event->common.capture_preamble_ms = 0;
    event->common.audio_config = AUDIO_CONFIG_INITIALIZER;
    event->common.audio_config.sample_rate = 16000;
    event->common.audio_config.channel_mask = AUDIO_CHANNEL_IN_MONO;
    event->common.audio_config.format = AUDIO_FORMAT_PCM_16_BIT;

    return data;
}

static char *stdev_generic_event_alloc(int model_handle)
{
    char *data;
    struct sound_trigger_generic_recognition_event *event;

    data = (char *)calloc(1,
                        sizeof(struct sound_trigger_generic_recognition_event));
    if (!data) {
        ALOGE("%s: Failed to allocate memory for recog event", __func__);
        return NULL;
    }

    event = (struct sound_trigger_generic_recognition_event *)data;
    event->common.status = RECOGNITION_STATUS_SUCCESS;
    event->common.type = SOUND_MODEL_TYPE_GENERIC;
    event->common.model = model_handle;

    /*
     * Signify that all the data is comming through streaming and
     * not through the buffer.
     */
    event->common.capture_available = true;
    event->common.audio_config = AUDIO_CONFIG_INITIALIZER;
    event->common.audio_config.sample_rate = 16000;
    event->common.audio_config.channel_mask = AUDIO_CHANNEL_IN_MONO;
    event->common.audio_config.format = AUDIO_FORMAT_PCM_16_BIT;

    return data;
}

static void stdev_close_term_sock(struct knowles_sound_trigger_device *stdev)
{
    if (stdev->send_sock >= 0) {
        close(stdev->send_sock);
        stdev->send_sock = -1;
    }

    if (stdev->recv_sock >= 0) {
        close(stdev->recv_sock);
        stdev->recv_sock = -1;
    }
}

// stdev needs to be locked before calling this function
static int restart_recognition(struct knowles_sound_trigger_device *stdev)
{
    int err = 0, i = 0;

    if (stdev->is_mic_route_enabled == true) {
        if (enable_mic_route(true)) {
            ALOGE("failed to enable mic route");
        }
    }
    // [TODO] Recovery function still TBD.
    // Download all the keyword models files that were previously loaded
    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_loaded == true) {
            err = write_model(stdev->models[i].data,
                            stdev->models[i].data_sz,
                            stdev->models[i].kw_id);
            if (err == -1) {
                ALOGE("%s: Failed to load the keyword model error - %d (%s)",
                        __func__, errno, strerror(errno));
                // How do we handle error during a recovery?
            }

            if (check_uuid_equality(stdev->models[i].uuid,
                                    stdev->sensor_model_uuid)) {
                set_sensor_route(true);
            }
            if (check_uuid_equality(stdev->models[i].uuid,
                                    stdev->chre_model_uuid)) {
                // set ambient with out bargein
                set_chre_audio_route(false);
            }
        }

        if (stdev->models[i].is_active == true) {
            if (check_uuid_equality(stdev->models[i].uuid,
                                    stdev->ambient_model_uuid)) {
                // set ambient with out bargein
                set_ambient_audio_route(false);
            }
            if (check_uuid_equality(stdev->models[i].uuid,
                                    stdev->hotword_model_uuid)) {
                // set hotword with out bargein
                set_hotword_route(false);
            }
        }
    }

    return err;
}

// stdev needs to be locked before calling this function
static int fw_crash_recovery(struct knowles_sound_trigger_device *stdev)
{
    int err;

    err = setup_chip();
    if (err != 0) {
        ALOGE("%s: ERROR: Failed to download packages and setup routes",
            __func__);
        goto exit;
    }

    // Redownload the keyword model files and start recognition
    err = restart_recognition(stdev);
    if (err != 0) {
        ALOGE("%s: ERROR: Failed to download the keyword models and restarting"
            " recognition", __func__);
        goto exit;
    }

exit:
    return err;
}

static void *callback_thread_loop(void *context)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)context;
    struct pollfd fds[2];
    char msg[UEVENT_MSG_LEN];
    int exit_sockets[2];
    int err = 0;
    int i, n;
    int kwid = 0;
    struct iaxxx_get_event ge;

    ALOGI("%s", __func__);
    prctl(PR_SET_NAME, (unsigned long)"sound trigger callback", 0, 0, 0);

    pthread_mutex_lock(&stdev->lock);

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, exit_sockets) == -1) {
        ALOGE("%s: Failed to create termination socket", __func__);
        goto exit;
    }

    stdev_close_term_sock(stdev);
    stdev->send_sock = exit_sockets[0];
    stdev->recv_sock = exit_sockets[1];

    memset(fds, 0, 2 * sizeof(struct pollfd));
    int timeout = -1; // Wait for event indefinitely
    fds[0].events = POLLIN;
    fds[0].fd = uevent_open_socket(64*1024, true);
    if (fds[0].fd == -1) {
        ALOGE("Error opening socket for hotplug uevent errno %d(%s)",
            errno, strerror(errno));
        goto exit;
    }
    fds[1].events = POLLIN;
    fds[1].fd = stdev->recv_sock;

    ge.event_id = -1;

    pthread_mutex_unlock(&stdev->lock);

    while (1) {
        poll (fds, 2, timeout);

        pthread_mutex_lock(&stdev->lock);
        if (err < 0) {
            ALOGE("%s: Error in poll: %d (%s)",
                __func__, errno, strerror(errno));
            break;
        }

        if (fds[0].revents & POLLIN) {
            n = uevent_kernel_multicast_recv(fds[0].fd, msg, UEVENT_MSG_LEN);
            if (n <= 0) {
                pthread_mutex_unlock(&stdev->lock);
                continue;
            }
            for (i = 0; i < n;) {
                if (strstr(msg + i, IAXXX_VQ_EVENT_STR)) {
                    ALOGI("%s", IAXXX_VQ_EVENT_STR);

                    err = get_event(&ge);
                    if (err == 0) {
                        if (ge.event_id == OK_GOOGLE_KW_ID) {
                            ALOGD("Eventid received is OK_GOOGLE_KW_ID %d",
                                OK_GOOGLE_KW_ID);
                            kwid = OK_GOOGLE_KW_ID;
                            stdev->last_detected_model_type = HOTWORD_MODEL;
                            break;
                        } else if (ge.event_id == AMBIENT_KW_ID) {
                            ALOGD("Eventid received is AMBIENT_KW_ID %d",
                                AMBIENT_KW_ID);
                            kwid = AMBIENT_KW_ID;
                            stdev->last_detected_model_type = AMBIENT_MODEL;
                            reset_ambient_plugin();
                            break;
                        } else {
                            ALOGE("Unknown event id received, ignoring %d",
                                ge.event_id);
                        }
                    } else {
                        ALOGE("get_event failed with error %d", err);
                    }
                } else if (strstr(msg + i, IAXXX_RECOVERY_EVENT_STR)) {
                    ALOGD("Firmware has crashed, start the recovery");
                    int err = fw_crash_recovery(stdev);
                    if (err != 0) {
                        ALOGE("Firmware crash recovery failed");
                    }
                }

                i += strlen(msg + i) + 1;
            }

            if (ge.event_id == OK_GOOGLE_KW_ID ||
                ge.event_id == AMBIENT_KW_ID) {
                ALOGD("%s: Keyword ID %d", __func__, kwid);

                int idx = find_handle_for_kw_id(stdev, kwid);
                if (idx < MAX_MODELS) {
                    if (stdev->models[idx].type == SOUND_MODEL_TYPE_KEYPHRASE) {
                        struct sound_trigger_phrase_recognition_event *event;
                        event = (struct sound_trigger_phrase_recognition_event*)
                                    stdev_keyphrase_event_alloc(
                                                stdev->models[idx].model_handle,
                                                stdev->models[idx].config,
                                                RECOGNITION_STATUS_SUCCESS);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d",
                                kwid);
                            model->recognition_callback(&event->common,
                                                    model->recognition_cookie);
                            // Update the config so that it will be used
                            // during the streaming
                            stdev->last_keyword_detected_config = model->config;

                            free(event);
                        } else {
                            ALOGE("Failed to allocate memory for the event");
                        }
                    } else if (stdev->models[idx].type == SOUND_MODEL_TYPE_GENERIC) {
                        struct sound_trigger_generic_recognition_event *event;
                        event = (struct sound_trigger_generic_recognition_event*)
                                    stdev_generic_event_alloc(
                                            stdev->models[idx].model_handle);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d",
                                kwid);
                            model->recognition_callback(&event->common,
                                                    model->recognition_cookie);
                            // Update the config so that it will be used
                            // during the streaming
                            stdev->last_keyword_detected_config = model->config;

                            free(event);
                        } else {
                            ALOGE("Failed to allocate memory for the event");
                        }
                    }
                }
                ge.event_id = -1;
            }
        } else if (fds[1].revents & POLLIN) {
            read(fds[1].fd, &n, sizeof(n)); /* clear the socket */
            ALOGD("%s: Termination message", __func__);
            break;
        }
        else {
            ALOGI("%s: Message ignored", __func__);
        }
        pthread_mutex_unlock(&stdev->lock);
    }

exit:
    stdev_close_term_sock(stdev);
    pthread_mutex_unlock(&stdev->lock);

    return (void *)(long)err;
}

static int stdev_get_properties(
                            const struct sound_trigger_hw_device *dev __unused,
                            struct sound_trigger_properties *properties)
{
    ALOGD("+%s+", __func__);
    if (properties == NULL)
        return -EINVAL;
    memcpy(properties, &hw_properties, sizeof(struct sound_trigger_properties));
    ALOGD("-%s-", __func__);
    return 0;
}

static int stdev_load_sound_model(const struct sound_trigger_hw_device *dev,
                                struct sound_trigger_sound_model *sound_model,
                                sound_model_callback_t callback,
                                void *cookie,
                                sound_model_handle_t *handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int ret = 0, err = 0;
    int kw_model_sz = 0;
    int i = 0;

    unsigned char *kw_buffer = NULL;


    ALOGD("+%s+", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (handle == NULL || sound_model == NULL) {
        ALOGE("%s: handle/sound_model is NULL", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (sound_model->data_size == 0 ||
        sound_model->data_offset < sizeof(struct sound_trigger_sound_model)) {
        ALOGE("%s: Invalid sound model data", __func__);
        ret = -EINVAL;
        goto exit;
    }

    kw_buffer = (unsigned char *) sound_model + sound_model->data_offset;
    kw_model_sz = sound_model->data_size;
    ALOGV("%s: kw_model_sz %d", __func__, kw_model_sz);

    // Load the keyword model file
    // Find an empty slot to load the model
    i = find_empty_model_slot(stdev);
    if (i == -1) {
        ALOGE("%s: Can't load model no free slots available", __func__);
        ret = -ENOSYS;
        goto exit;
    }

    *handle = i;
    ALOGV("%s: Loading keyword model handle(%d) type(%d)", __func__,
        *handle, sound_model->type);
    // This will need to be replaced with UUID once they are fixed
    stdev->models[i].kw_id = (i + 1);
    stdev->models[i].model_handle = *handle;
    stdev->models[i].type = sound_model->type;
    stdev->models[i].uuid = sound_model->uuid;
    stdev->models[i].sound_model_callback = callback;
    stdev->models[i].sound_model_cookie = cookie;
    stdev->models[i].recognition_callback = NULL;
    stdev->models[i].recognition_cookie = NULL;

    stdev->models[i].data = malloc(kw_model_sz);
    if (stdev->models[i].data == NULL) {
        ALOGE("%s: Warning, could not allocate memory for keyword model data,"
            " cannot redownload on crash", __func__);
        stdev->models[i].data_sz = 0;
    } else {
        memcpy(stdev->models[i].data, kw_buffer, kw_model_sz);
        stdev->models[i].data_sz = kw_model_sz;
    }

    // Send the keyword model to the chip only for hotword and ambient audio
    if (check_uuid_equality(sound_model->uuid,
                            stdev->hotword_model_uuid)) {
        err = write_model(kw_buffer, kw_model_sz, OK_GOOGLE_KW_ID);
        stdev->models[i].kw_id = OK_GOOGLE_KW_ID;
    } else if (check_uuid_equality(sound_model->uuid,
                                stdev->ambient_model_uuid)) {
        err = write_model(kw_buffer, kw_model_sz, AMBIENT_KW_ID);
        stdev->models[i].kw_id = AMBIENT_KW_ID;
    } else if (check_uuid_equality(sound_model->uuid,
                                stdev->sensor_model_uuid)) {
        // Don't download the keyword model file, just setup the
        // sensor route
        set_sensor_route(true);
    } else if (check_uuid_equality(sound_model->uuid,
                                stdev->chre_model_uuid)) {
        // Don't download the keyword model file, just setup the
        // CHRE route
        set_chre_audio_route(false);
    } else {
        ALOGE("%s: ERROR: unknown keyword model file", __func__);
        err = -1;
        errno = -EINVAL;
    }
    if (err == -1) {
        ALOGE("%s: Failed to load the keyword model error - %d (%s)",
            __func__, errno, strerror(errno));
        ret = errno;
        if (stdev->models[i].data) {
            free(stdev->models[i].data);
            stdev->models[i].data = NULL;
            stdev->models[i].data_sz = 0;
        }
        goto exit;
    }

    stdev->models[i].is_loaded = true;

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s handle %d-", __func__, *handle);
    return ret;
}

static int stdev_unload_sound_model(const struct sound_trigger_hw_device *dev,
                                    sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int ret = 0;

    ALOGD("+%s handle %d+", __func__, handle);
    pthread_mutex_lock(&stdev->lock);

    // Just confirm the model was previously loaded
    if (stdev->models[handle].is_loaded == false) {
        ALOGE("%s: Invalid model(%d) being called for unload",
                __func__, handle);
        ret = -EINVAL;
        goto exit;
    }

    if (check_uuid_equality(stdev->models[handle].uuid,
                            stdev->sensor_model_uuid)) {
        // Disable the sensor route
        set_sensor_route(false);
    } else if (check_uuid_equality(stdev->models[handle].uuid,
                                stdev->chre_model_uuid)) {
        // Disable the CHRE route
        tear_chre_audio_route();
    }

    stdev->models[handle].sound_model_callback = NULL;
    stdev->models[handle].sound_model_cookie = NULL;
    stdev->models[handle].is_loaded = false;
    if (stdev->models[handle].data) {
        free(stdev->models[handle].data);
        stdev->models[handle].data = NULL;
        stdev->models[handle].data_sz = 0;
    }

    ALOGD("%s: Successfully unloaded the model, handle - %d",
        __func__, handle);

    // [TODO] Need to unload the keyword model files from the chip
exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s handle %d-", __func__, handle);
    return ret;
}

static int stdev_start_recognition(
                        const struct sound_trigger_hw_device *dev,
                        sound_model_handle_t handle,
                        const struct sound_trigger_recognition_config *config,
                        recognition_callback_t callback,
                        void *cookie)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0;
    struct model_info *model = &stdev->models[handle];

    ALOGD("%s stdev %p, sound model %d", __func__, stdev, handle);

    pthread_mutex_lock(&stdev->lock);

    if (callback == NULL) {
        ALOGE("%s: recognition_callback is null", __func__);
        status = -EINVAL;
        goto exit;
    }

    if (model->config != NULL) {
        dereg_hal_event_session(model->config, handle);
        free(model->config);
        model->config = NULL;
    }

    if (config != NULL) {
        model->config = (struct sound_trigger_recognition_config *)
                            malloc(sizeof(*config));
        if (model->config == NULL) {
            ALOGE("%s: Failed to allocate memory for model config", __func__);
            status = -ENOMEM;
            goto exit;
        }

        memcpy(model->config, config, sizeof(*config));
        reg_hal_event_session(model->config, handle);

        ALOGD("%s: Is capture requested %d",
            __func__, config->capture_requested);
    } else {
        ALOGD("%s: config is null", __func__);
        model->config = NULL;
    }

    model->recognition_callback = callback;
    model->recognition_cookie = cookie;
    if (model->is_active == true) {
        // This model is already active, do nothing except updating callbacks,
        // configs and cookie
        goto exit;
    }
    model->is_active = true;

    if (stdev->is_mic_route_enabled == false) {
        stdev->is_mic_route_enabled = true;
        if (enable_mic_route(true)) {
            ALOGE("failed to enable mic route");
            stdev->is_mic_route_enabled = false;
        }
    }

    if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        // set ambient with out bargein
        set_ambient_audio_route(false);
    } else if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid)) {
        // set hotword with out bargein
        set_hotword_route(false);
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s sound model %d-", __func__, handle);
    return status;
}

static int stdev_stop_recognition(
                        const struct sound_trigger_hw_device *dev,
                        sound_model_handle_t handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0, i = 0;
    struct model_info *model = &stdev->models[handle];
    ALOGD("+%s sound model %d+", __func__, handle);
    pthread_mutex_lock(&stdev->lock);

    if (model->config != NULL) {
        dereg_hal_event_session(model->config, handle);
        free(model->config);
        model->config = NULL;
    }

    model->recognition_callback = NULL;
    model->recognition_cookie = NULL;
    model->is_active = false;

    if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        tear_ambient_audio_route();
    } else if (check_uuid_equality(model->uuid, stdev->hotword_model_uuid)) {
        tear_hotword_route();
    }

    for (i = 0; i < MAX_MODELS; i++) {
        if (stdev->models[i].is_active == true) {
            break;
        }
    }

    if (i == MAX_MODELS) {
        ALOGD("None of keywords are active so disable mic route");
        if (stdev->is_mic_route_enabled == true) {
            stdev->is_mic_route_enabled = false;
            if (enable_mic_route(false)) {
                ALOGE("failed to disable mic route");
                stdev->is_mic_route_enabled = true;
            }
        }
    }

    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s sound model %d-", __func__, handle);
    return status;
}

static int stdev_close(hw_device_t *device)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)device;
    int ret = 0;
    ALOGD("+%s+", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (!stdev->opened) {
        ALOGE("%s: device already closed", __func__);
        ret = -EFAULT;
        goto exit;
    }
    stdev->opened = false;

    if (stdev->send_sock >= 0)
        write(stdev->send_sock, "T", 1);
    pthread_join(stdev->callback_thread, (void **)NULL);

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s-", __func__);
    return ret;
}

__attribute__ ((visibility ("default")))
audio_io_handle_t stdev_get_audio_handle()
{
    if (g_stdev.last_keyword_detected_config == NULL) {
        ALOGI("%s: Config is NULL so returning audio handle as 0", __func__);
        return 0;
    }

    ALOGI("%s: Audio Handle is %d",
        __func__, g_stdev.last_keyword_detected_config->capture_handle);

    return g_stdev.last_keyword_detected_config->capture_handle;
}

__attribute__ ((visibility ("default")))
void stdev_audio_record_state(bool enabled) {
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int i = 0;

    pthread_mutex_lock(&stdev->lock);
    ALOGI("%s: Entering enabled %d", __func__, enabled);

    if (enabled == true) {
        // Recording is about to start so disable our mic route and
        // model files that are still active.
        for (i = 0; i < MAX_MODELS; i++) {
            if (stdev->models[i].is_active == true) {
                if (check_uuid_equality(stdev->models[i].uuid,
                                        stdev->ambient_model_uuid)) {
                    tear_ambient_audio_route();
                } else if (check_uuid_equality(stdev->models[i].uuid,
                                            stdev->hotword_model_uuid)) {
                    tear_hotword_route();
                }
                stdev->models[i].is_active = false;
            }
        }

        if (stdev->is_mic_route_enabled == true) {
            ALOGE("Disabling the mic route");
            stdev->is_mic_route_enabled = false;
            if (enable_mic_route(false)) {
                ALOGE("failed to disable mic route");
                stdev->is_mic_route_enabled = true;
            }
        }
    } else {
        // Recording is about to end so enable mic route if it was previously
        // disabled
        // TBD check Is this required? Because it will get enabled again in
        // start recognition
    }

    ALOGI("%s: Exiting", __func__);

    pthread_mutex_unlock(&stdev->lock);
}

__attribute__ ((visibility ("default")))
void stdev_set_bargein(bool enable) {
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int i;

    pthread_mutex_lock(&stdev->lock);
    ALOGI("%s: Entering enable %d", __func__, enable);

    if (enable == true) {
        ALOGI("Bargein should be enabled");
        enable_bargein_route(true);
        for (i = 0; i < MAX_MODELS; i++) {
            if (stdev->models[i].is_active == true) {
                if (check_uuid_equality(stdev->models[i].uuid,
                                        stdev->ambient_model_uuid)) {
                    // teardown the current ambient route
                    tear_ambient_audio_route();
                    // resetup the ambient route with bargein
                    set_ambient_audio_route(true);
                } else if (check_uuid_equality(stdev->models[i].uuid,
                                            stdev->chre_model_uuid)) {
                    // teardown the current chre route
                    tear_chre_audio_route();
                    // resetup the chre route with bargein
                    set_chre_audio_route(true);
                } else if (check_uuid_equality(stdev->models[i].uuid,
                                            stdev->hotword_model_uuid)) {
                    // teardown the current hotword route
                    tear_hotword_route();
                    // resetup the hotword route with bargein
                    set_hotword_route(true);
                }
            }
        }
    } else {
        ALOGI("Bargein should be disabled");
        for (i = 0; i < MAX_MODELS; i++) {
            if (stdev->models[i].is_active == true) {
                if (check_uuid_equality(stdev->models[i].uuid,
                                        stdev->ambient_model_uuid)) {
                    // teardown the current ambient route with bargein
                    tear_ambient_audio_route();
                    // resetup the ambient route with out bargein
                    set_ambient_audio_route(false);
                } else if (check_uuid_equality(stdev->models[i].uuid,
                                            stdev->chre_model_uuid)) {
                    // teardown the current chre route with bargein
                    tear_chre_audio_route();
                    // resetup the chre route with out bargein
                    set_chre_audio_route(false);
                } else if (check_uuid_equality(stdev->models[i].uuid,
                                            stdev->hotword_model_uuid)) {
                    // teardown the current hotword route with bargein
                    tear_hotword_route();
                    // resetup the hotword route with out bargein
                    set_hotword_route(false);
                }
            }
        }
        enable_bargein_route(false);
    }

    ALOGI("%s: Exiting", __func__);

    pthread_mutex_unlock(&stdev->lock);
}

static int open_streaming_lib(struct knowles_sound_trigger_device *stdev) {
    int ret = 0;

    if (access(ADNC_STRM_LIBRARY_PATH, R_OK) == 0) {
        stdev->adnc_cvq_strm_lib = dlopen(ADNC_STRM_LIBRARY_PATH, RTLD_NOW);
        if (stdev->adnc_cvq_strm_lib == NULL) {
            char const *err_str = dlerror();
            ALOGE("%s: module = %s error = %s", __func__,
                ADNC_STRM_LIBRARY_PATH, err_str ? err_str : "unknown");
            ALOGE("%s: DLOPEN failed for %s", __func__, ADNC_STRM_LIBRARY_PATH);
        } else {
            ALOGV("%s: DLOPEN successful for %s",
                __func__, ADNC_STRM_LIBRARY_PATH);
            stdev->adnc_strm_handle = 0;
            stdev->adnc_strm_open =
                (int (*)(bool, int, int))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_open");
            stdev->adnc_strm_read =
               (size_t (*)(long, void *, size_t))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_read");
            stdev->adnc_strm_close =
                (int (*)(long))dlsym(stdev->adnc_cvq_strm_lib,
                "adnc_strm_close");
            if (!stdev->adnc_strm_open || !stdev->adnc_strm_read ||
                !stdev->adnc_strm_close) {
                ALOGE("%s: Error grabbing functions in %s", __func__,
                    ADNC_STRM_LIBRARY_PATH);
                stdev->adnc_strm_open = 0;
                stdev->adnc_strm_read = 0;
                stdev->adnc_strm_close = 0;
            }
        }
    }

    return ret;
}

static int load_audio_hal()
{
    char audio_hal_lib[100];
    void *sthal_prop_api_version = NULL;
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int ret = 0;

    snprintf(audio_hal_lib, sizeof(audio_hal_lib), "%s/%s.%s.so",
            AUDIO_HAL_LIBRARY_PATH, AUDIO_HAL_NAME_PREFIX,
            SOUND_TRIGGER_PLATFORM);
    if (access(audio_hal_lib, R_OK)) {
        ALOGE("%s: ERROR. %s not found", __func__, audio_hal_lib);
        return -ENOENT;
    }

    stdev->audio_hal_handle = dlopen(audio_hal_lib, RTLD_NOW);
    if (stdev->audio_hal_handle == NULL) {
        ALOGE("%s: ERROR. %s", __func__, dlerror());
        return -ENODEV;
    }

    stdev->audio_hal_cb = dlsym(stdev->audio_hal_handle, "audio_hw_call_back");
    if (stdev->audio_hal_cb == NULL) {
        ALOGE("%s: ERROR. %s", __func__, dlerror());
        ret = -ENODEV;
        goto error;
    }

    sthal_prop_api_version = dlsym(stdev->audio_hal_handle,
                                "sthal_prop_api_version");
    if (sthal_prop_api_version == NULL) {
        stdev->sthal_prop_api_version = 0;
        ret = 0; /* passthru for backward compability */
    } else {
        stdev->sthal_prop_api_version = *(int *)sthal_prop_api_version;
        if (MAJOR_VERSION(stdev->sthal_prop_api_version) !=
            MAJOR_VERSION(STHAL_PROP_API_CURRENT_VERSION)) {
            ALOGE("%s: Incompatible API versions sthal:0x%x != ahal:0x%x",
                __func__, STHAL_PROP_API_CURRENT_VERSION,
                stdev->sthal_prop_api_version);
            goto error;
        }
        ALOGD("%s: ahal is using proprietary API version 0x%04x", __func__,
            stdev->sthal_prop_api_version);
    }

    ALOGD("%s: load AHAL successfully.", __func__);
    return ret;

error:
    dlclose(stdev->audio_hal_handle);
    stdev->audio_hal_handle = NULL;
    return ret;
}

static int stdev_open(const hw_module_t *module, const char *name,
        hw_device_t **device)
{
    struct knowles_sound_trigger_device *stdev;
    int ret = 0, i = 0;

    ALOGE("!! Knowles SoundTrigger v1!!");

    if (strcmp(name, SOUND_TRIGGER_HARDWARE_INTERFACE) != 0)
        return -EINVAL;

    stdev = &g_stdev;
    pthread_mutex_lock(&stdev->lock);

    if (stdev->opened) {
        ALOGE("%s: Only one sountrigger can be opened at a time", __func__);
        ret = -EBUSY;
        goto exit;
    }

    ret = open_streaming_lib(stdev);
    if (ret != 0) {
        ALOGE("%s: Couldnot open the streaming library", __func__);
        goto exit;
    }

    ret = load_audio_hal();
    if (ret != 0) {
        if (stdev->adnc_cvq_strm_lib)
            dlclose(stdev->adnc_cvq_strm_lib);
        ALOGE("%s: Couldn't load AHAL", __func__);
        goto exit;
    }

    stdev->device.common.tag = HARDWARE_DEVICE_TAG;
    stdev->device.common.version = SOUND_TRIGGER_DEVICE_API_VERSION_1_0;
    stdev->device.common.module = (struct hw_module_t *)module;
    stdev->device.common.close = stdev_close;
    stdev->device.get_properties = stdev_get_properties;
    stdev->device.load_sound_model = stdev_load_sound_model;
    stdev->device.unload_sound_model = stdev_unload_sound_model;
    stdev->device.start_recognition = stdev_start_recognition;
    stdev->device.stop_recognition = stdev_stop_recognition;

    stdev->opened = true;
    /* Initialize all member variable */
    for (i = 0; i < MAX_MODELS; i++) {
        stdev->models[i].config = NULL;
        stdev->models[i].data = NULL;
        stdev->models[i].data_sz = 0;
        stdev->models[i].is_loaded = false;
        stdev->models[i].is_active = false;
        stdev->last_keyword_detected_config = NULL;
    }

    stdev->is_mic_route_enabled = false;

    str_to_uuid(HOTWORD_AUDIO_MODEL, &stdev->hotword_model_uuid);
    str_to_uuid(SENSOR_MANAGER_MODEL, &stdev->sensor_model_uuid);
    str_to_uuid(AMBIENT_AUDIO_MODEL, &stdev->ambient_model_uuid);
    str_to_uuid(CHRE_AUDIO_MODEL, &stdev->chre_model_uuid);

    *device = &stdev->device.common; /* same address as stdev */

    ALOGD("%s: Wait for Firmware download to be completed", __func__);
    /* [TODO] Temp workaround for different hw interface
     * we will add a mechanism to handle that instead hard-code */
    for (int max_attempts = 0; max_attempts < 300; max_attempts++) {
        struct stat ready_stat;
        const char *ready_path1 = "/sys/bus/spi/devices/spi1.0/iaxxx/fw_dl_complete";
        const char *ready_path2 = "/sys/bus/spi/devices/spi2.0/iaxxx/fw_dl_complete";
        const char *ready_path3 = "/sys/bus/spi/devices/spi3.0/iaxxx/fw_dl_complete";
        const char *ready_path4 = "/sys/bus/spi/devices/spi4.0/iaxxx/fw_dl_complete";

        int ready_result1 = stat(ready_path1, &ready_stat);
        int ready_result2 = stat(ready_path2, &ready_stat);
        int ready_result3 = stat(ready_path3, &ready_stat);
        int ready_result4 = stat(ready_path4, &ready_stat);

        if (0 == ready_result1 || 0 == ready_result2 || 0 == ready_result3 || 0 == ready_result4) {
            ALOGD("%s: Found", __func__);
            break;
        } else {
            ALOGD("%s: Didn't find fw_dl_complete, sleeping for one sec", __func__);
            sleep(1);
        }
    }

    setup_chip();

    ALOGD("stdev before pthread_create %p", stdev);
    // Create a thread to handle all events from kernel
    pthread_create(&stdev->callback_thread, (const pthread_attr_t *) NULL,
                callback_thread_loop, stdev);

exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

/* AHAL calls this callback to communicate with STHAL */
int sound_trigger_hw_call_back(audio_event_type_t event,
                            struct audio_event_info *config)
{
    int ret = 0;
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    if (!stdev)
        return -ENODEV;

    if (!stdev->opened) {
        ALOGE("%s: Error SoundTrigger has not been opened", __func__);
        return -EINVAL;
    }

    switch (event) {
    case AUDIO_EVENT_CAPTURE_DEVICE_INACTIVE:
    case AUDIO_EVENT_CAPTURE_DEVICE_ACTIVE:
    case AUDIO_EVENT_PLAYBACK_STREAM_INACTIVE:
    case AUDIO_EVENT_PLAYBACK_STREAM_ACTIVE:
    case AUDIO_EVENT_CAPTURE_STREAM_INACTIVE:
    case AUDIO_EVENT_CAPTURE_STREAM_ACTIVE:
        /*
         * [TODO] handle playback stream on/off event
         * That need to enable AEC plugin in codec
         */

        /*
         * [TODO] handle capture stream on/off event
         * That pause all recognition in codec
         * and resume after capture stream off
         */
        ALOGD("%s: handle audio concurrency %d", __func__, event);
        break;

    case AUDIO_EVENT_STOP_LAB:
        /* Close Stream Driver */
        ALOGD("%s: close streaming %d", __func__, event);
        pthread_mutex_lock(&stdev->lock);
        if (stdev->adnc_strm_handle) {
            stdev->adnc_strm_close(stdev->adnc_strm_handle);
            stdev->adnc_strm_handle = 0;
            stdev->is_streaming = 0;
        }
        pthread_mutex_unlock(&stdev->lock);

        break;

    case AUDIO_EVENT_SSR:
        /*[TODO] Do we need to handle adsp SSR event ? */
        ALOGD("%s: handle audio subsystem restart %d", __func__, event);
        break;

    case AUDIO_EVENT_READ_SAMPLES:
        /* Open Stream Driver */
        pthread_mutex_lock(&stdev->lock);
        if (stdev->is_streaming == false) {
            if (stdev->adnc_strm_open == NULL) {
                ALOGE("%s: Error adnc streaming not supported", __func__);
            } else {
                bool keyword_stripping_enabled = false;
                stdev->adnc_strm_handle = stdev->adnc_strm_open(
                                            keyword_stripping_enabled, 0,
                                            stdev->last_detected_model_type);
                if (stdev->adnc_strm_handle) {
                    ALOGD("Successfully opened adnc streaming");
                    stdev->is_streaming = true;
                } else {
                    ALOGE("%s: DSP is currently not streaming", __func__);
                }
            }
        }
        /* Read pcm data from tunnel */
        if (stdev->is_streaming == true) {
            //ALOGD("%s: soundtrigger HAL adnc_strm_read", __func__);
            stdev->adnc_strm_read(stdev->adnc_strm_handle,
                                config->u.aud_info.buf,
                                config->u.aud_info.num_bytes);
        } else {
            ALOGE("%s: soundtrigger is not streaming", __func__);
        }
        pthread_mutex_unlock(&stdev->lock);

        break;

    case AUDIO_EVENT_NUM_ST_SESSIONS:
    case AUDIO_EVENT_DEVICE_CONNECT:
    case AUDIO_EVENT_DEVICE_DISCONNECT:
    case AUDIO_EVENT_SVA_EXEC_MODE:
    case AUDIO_EVENT_SVA_EXEC_MODE_STATUS:
        ALOGV("%s: useless event %d", __func__, event);
        break;

    default:
        ALOGW("%s: Unknown event %d", __func__, event);
        break;
    }

    return ret;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = stdev_open,
};

struct sound_trigger_module HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = SOUND_TRIGGER_MODULE_API_VERSION_1_0,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = SOUND_TRIGGER_HARDWARE_MODULE_ID,
        .name = "Knowles Sound Trigger HAL",
        .author = "Knowles Electronics",
        .methods = &hal_module_methods,
    },
};