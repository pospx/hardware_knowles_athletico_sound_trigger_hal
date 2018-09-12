/*
 * Copyright (C) 2014 The Android Open Source Project
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

#define LOG_TAG "SoundTriggerKnowles_HAL"
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

#define ENABLE_KEYPHRASE_DETECTION

#define MAX_GENERIC_SOUND_MODELS    (3)
#define MAX_KEY_PHRASES             (1)
#define MAX_MODELS                  (MAX_GENERIC_SOUND_MODELS + MAX_KEY_PHRASES)

#define MAX_USERS                   (1)
#define MAX_BUFFER_MS               (3000)
#define POWER_CONSUMPTION           (0) // TBD
#define ST_HAL_VERSION              (1)

#define UEVENT_MSG_LEN              (1024)

#define OK_GOOGLE_KW_ID              (0)
#define AMBIENT_KW_ID              (1)

#define IAXXX_VQ_EVENT_STR          "IAXXX_VQ_EVENT"
#define IAXXX_RECOVERY_EVENT_STR    "IAXXX_RECOVERY_EVENT"

#ifdef __LP64__
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib64/hw/adnc_strm.primary.default.so"
#else
#define ADNC_STRM_LIBRARY_PATH "/vendor/lib/hw/adnc_strm.primary.default.so"
#endif

#define SENSOR_MANAGER_MODEL "5c0c296d-204c-4c2b-9f85-e50746caf914"
#define AMBIENT_AUDIO_MODEL "6ac81359-2dc2-4fea-a0a0-bd378ed6da4f"

#define HOTWORD_MODEL (0)
#define AMBIENT_MODEL (1)

// Define this macro to enable test stub to simulate the Generic Sound Model
// form the SoundTrigger HAL.
//#define SIMULATE_GSM_TEST_STUB

static const struct sound_trigger_properties hw_properties = {
    "Knowles Electronics",      // implementor
    "Continous VoiceQ",         // description
    1,                          // version
    { 0x80f7dcd5, 0xbb62, 0x4816, 0xa931, { 0x9c, 0xaa, 0x52, 0x5d, 0xf5, 0xc7 } }, // Version UUID
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
};

struct knowles_sound_trigger_device {
    struct sound_trigger_hw_device device;
    struct model_info models[MAX_MODELS];
    sound_trigger_uuid_t authkw_model_uuid;
    pthread_t callback_thread;
    pthread_mutex_t lock;
    bool is_recog_in_prog;
    int opened;
    int send_sock;
    int recv_sock;
    struct sound_trigger_recognition_config *last_keyword_detected_config;

    // Information about streaming
    int is_streaming;
    void *adnc_cvq_strm_lib;
    int    (*adnc_strm_open)(bool, int, int);
    size_t (*adnc_strm_read)(long, void*, size_t);
    int    (*adnc_strm_close)(long);
    long adnc_strm_handle;
#ifdef SIMULATE_GSM_TEST_STUB
    int last_recog_model_id;
#endif

    sound_trigger_uuid_t sensor_model_uuid;
    sound_trigger_uuid_t ambient_model_uuid;

    int last_detected_model_type;
};

// Since there's only ever one sound_trigger_device, keep it as a global so that other people can
// dlopen this lib to get at the streaming audio.
static struct knowles_sound_trigger_device g_stdev = { .lock = PTHREAD_MUTEX_INITIALIZER };

static bool check_uuid_equality(sound_trigger_uuid_t uuid1, sound_trigger_uuid_t uuid2) {
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
        tmp, tmp+1, tmp+2, tmp+3, tmp+4, tmp+5, tmp+6, tmp+7, tmp+8, tmp+9) < 10) {
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
        if (false == st_dev->models[i].is_loaded)
            break;
    }

    if (MAX_MODELS <= i) {
        i = -1;
    }

    return i;
}

static int find_handle_for_kw_id(struct knowles_sound_trigger_device *st_dev, int kw_id)
{
    int i = 0;
    for (i = 0; i < MAX_MODELS; i++) {
        if (kw_id == st_dev->models[i].kw_id)
            break;
    }

    return i;
}

static char *stdev_keyphrase_event_alloc(sound_model_handle_t handle,
                                         struct sound_trigger_recognition_config *config,
                                         int recognition_status)
{
    char *data;
    struct sound_trigger_phrase_recognition_event *event;
    data = (char *)calloc(1, sizeof(struct sound_trigger_phrase_recognition_event));
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
    // Signify that all the data is comming through streaming, not through the buffer.
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

    data = (char *)calloc(1, sizeof(struct sound_trigger_generic_recognition_event));
    if (!data) {
        ALOGE("%s: Failed to allocate memory for recog event", __func__);
        return NULL;
    }

    event = (struct sound_trigger_generic_recognition_event *)data;
    event->common.status = RECOGNITION_STATUS_SUCCESS;
    event->common.type   = SOUND_MODEL_TYPE_GENERIC;
    event->common.model  = model_handle;

    // Signify that all the data is comming through streaming, not through the buffer.
    event->common.capture_available         = true;
    event->common.audio_config              = AUDIO_CONFIG_INITIALIZER;
    event->common.audio_config.sample_rate  = 16000;
    event->common.audio_config.channel_mask = AUDIO_CHANNEL_IN_MONO;
    event->common.audio_config.format       = AUDIO_FORMAT_PCM_16_BIT;

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

    // Download all the keyword models files that were previously loaded
    for (i = 0; i < MAX_MODELS; i++) {
        if (true == stdev->models[i].is_loaded) {
            err = write_model(stdev->models[i].data, stdev->models[i].data_sz, stdev->models[i].kw_id);
            if (-1 == err) {
                ALOGE("%s: Failed to load the keyword model error - %d (%s)",
                                            __func__, errno, strerror(errno));
                // How do we handle error during a recovery?
            }
        }
    }

    if (true == stdev->is_recog_in_prog) {
        err = start_cvq();
        if (enable_mic_route(true)) {
            ALOGE("failed to enable mic route");
        }
    }

    return err;
}

// stdev needs to be locked before calling this function
static int fw_crash_recovery(struct knowles_sound_trigger_device *stdev)
{
    int err;

    err = setup_mic_routes();
    if (0 != err) {
        ALOGE("%s: ERROR: Failed to download packages and setup routes", __func__);
        goto exit;
    }

    // Setup the VQ plugin
    err = init_params();
    if (err < 0) {
        ALOGE("%s: ERROR: Failed to setup the chip", __func__);
        goto exit;
    }

    // Redownload the keyword model files and start recognition
    err = restart_recognition(stdev);
    if (0 != err) {
        ALOGE("%s: ERROR: Failed to download the keyword models and restarting recognition", __func__);
        goto exit;
    }

exit:
    return err;
}

static void *callback_thread_loop(void *context)
{

    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)context;
#ifdef SIMULATE_GSM_TEST_STUB
    struct pollfd fds[3];
    struct itimerspec ts;
    struct timespec now;
#else
    struct pollfd fds[2];
#endif
    char msg[UEVENT_MSG_LEN];
    int exit_sockets[2];
    int err = 0;
    int i, n;
    int kwid = 0;
    struct iaxxx_get_event ge;

    ALOGI("%s", __func__);
    prctl(PR_SET_NAME, (unsigned long)"sound trigger callback", 0, 0, 0);

    pthread_mutex_lock(&stdev->lock);

    ALOGD("%s stdev %p", __func__, stdev);

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, exit_sockets) == -1) {
        ALOGE("%s: Failed to create termination socket", __func__);
        goto exit;
    }

    stdev_close_term_sock(stdev);
    stdev->send_sock = exit_sockets[0];
    stdev->recv_sock = exit_sockets[1];

#ifdef SIMULATE_GSM_TEST_STUB
    memset(fds, 0, 3 * sizeof(struct pollfd));
#else
    memset(fds, 0, 2 * sizeof(struct pollfd));
#endif // SIMULATE_GSM_TEST_STUB
    int timeout = -1; // Wait for event indefinitely
    fds[0].events = POLLIN;
    fds[0].fd = uevent_open_socket(64*1024, true);
    if (fds[0].fd == -1) {
        ALOGE("Error opening socket for hotplug uevent errno %d(%s)", errno, strerror(errno));
        goto exit;
    }
    fds[1].events = POLLIN;
    fds[1].fd = stdev->recv_sock;

#ifdef SIMULATE_GSM_TEST_STUB
    if (clock_gettime(CLOCK_REALTIME, &now) == -1)  {
        ALOGE("Failed to get the realtime clock");
        goto exit;
    }

    fds[2].events = POLLIN;
    fds[2].fd = timerfd_create(CLOCK_REALTIME, 0);
    if (-1 == fds[2].fd) {
        ALOGE("Failed to create the timer fd");
        goto exit;
    }

    ts.it_interval.tv_sec = 5;
    ts.it_interval.tv_nsec = 0;
    ts.it_value.tv_sec = now.tv_sec + 5;
    ts.it_value.tv_nsec = now.tv_nsec;

    if (timerfd_settime(fds[2].fd, TFD_TIMER_ABSTIME, &ts, NULL) < 0) {
        ALOGE("timerfd_settime() failed: errno=%d\n", errno);
        goto exit;
    }
#endif // SIMULATE_GSM_TEST_STUB

    ge.event_id = -1;

    pthread_mutex_unlock(&stdev->lock);

    while (1) {
#ifdef SIMULATE_GSM_TEST_STUB
        err = poll (fds, 3, timeout);
#else
        err = poll (fds, 2, timeout);
#endif // SIMULATE_GSM_TEST_STUB

        if (0 == err) {
            ALOGE("Timeout YAY!!");
        }

        pthread_mutex_lock(&stdev->lock);
        if (err < 0) {
            ALOGE("%s: Error in poll: %d (%s)", __func__, errno, strerror(errno));
            break;
        }

        if (fds[0].revents & POLLIN) {
            n = uevent_kernel_multicast_recv(fds[0].fd, msg, UEVENT_MSG_LEN);
            if (n <= 0) {
                pthread_mutex_unlock(&stdev->lock);
                continue;
            }
            ALOGI("uevent msg is %s", msg);
            for (i = 0; i < n;) {
                if (strstr(msg + i, IAXXX_VQ_EVENT_STR)) {
                    ALOGI("%s", IAXXX_VQ_EVENT_STR);

                    err = get_event(&ge);
                    if (0 == err) {
                        if (OK_GOOGLE_KW_ID == ge.event_id) {
                            ALOGD("Eventid received is OK_GOOGLE_KW_ID %d", OK_GOOGLE_KW_ID);
                            kwid = OK_GOOGLE_KW_ID;
                            stdev->last_detected_model_type = HOTWORD_MODEL;
                            break;
                        } else if (AMBIENT_KW_ID == ge.event_id) {
                            ALOGD("Eventid received is AMBIENT_KW_ID %d", AMBIENT_KW_ID);
                            kwid = AMBIENT_KW_ID;
                            stdev->last_detected_model_type = AMBIENT_MODEL;
                            break;
                        } else {
                            ALOGE("Unknown event id received, ignoring %d", ge.event_id);
                        }
                    } else {
                        ALOGE("get_event failed with error %d", err);
                    }
                } else if (strstr(msg + i, IAXXX_RECOVERY_EVENT_STR)) {
                    ALOGE("Firmware has crashed, start the recovery");
                    int err = fw_crash_recovery(stdev);
                    if (0 != err) {
                        ALOGE("Firmware crash recovery failed");
                    }
                }

                i += strlen(msg + i) + 1;
            }

            if (OK_GOOGLE_KW_ID == ge.event_id ||
                AMBIENT_KW_ID == ge.event_id) {
                ALOGE("%s: Keyword ID %d", __func__, kwid);
                int idx = find_handle_for_kw_id(stdev, kwid);

                if (idx < MAX_MODELS) {
                    if (SOUND_MODEL_TYPE_KEYPHRASE == stdev->models[idx].type) {
                        struct sound_trigger_phrase_recognition_event *event;
                        event = (struct sound_trigger_phrase_recognition_event *)
                                    stdev_keyphrase_event_alloc(stdev->models[idx].model_handle,
                                                                stdev->models[idx].config,
                                                                RECOGNITION_STATUS_SUCCESS);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d", kwid);
                            model->recognition_callback(&event->common, model->recognition_cookie);
                            // Update the config so that it will be used during the streaming
                            stdev->last_keyword_detected_config = model->config;

                            free(event);
                        } else {
                            ALOGE("Failed to allocate memory for the event");
                        }
                    } else if (SOUND_MODEL_TYPE_GENERIC == stdev->models[idx].type) {
                        struct sound_trigger_generic_recognition_event *event;
                        event = (struct sound_trigger_generic_recognition_event *)
                                stdev_generic_event_alloc(stdev->models[idx].model_handle);
                        if (event) {
                            struct model_info *model;
                            model = &stdev->models[idx];

                            ALOGD("Sending recognition callback for id %d", kwid);
                            model->recognition_callback(&event->common, model->recognition_cookie);
                            // Update the config so that it will be used during the streaming
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
            ALOGI("%s: Termination message", __func__);
            break;
        }
#ifdef SIMULATE_GSM_TEST_STUB
        else if (fds[2].revents & POLLIN) {
            uint64_t temp;
        // This means that the poll timed out, so check if we have any generic
        // models, if yes then send and a recognition event for that
            int k = 0;
            for (k = 0; k < MAX_MODELS; k++) {
                if (SOUND_MODEL_TYPE_GENERIC == stdev->models[k].type &&
                    k != stdev->last_recog_model_id) {
                        break;
                }
            }

            if (MAX_MODELS == k && -1 != stdev->last_recog_model_id)
                k = stdev->last_recog_model_id;
            else
                stdev->last_recog_model_id = k;

            if (k < MAX_MODELS) {
                struct sound_trigger_generic_recognition_event *event;
                event = (struct sound_trigger_generic_recognition_event *)
                    stdev_generic_event_alloc(stdev->models[k].model_handle);
                if (event) {
                    struct model_info *model;
                    model = &stdev->models[k];

                    if (NULL != model->recognition_callback) {
                        ALOGD("Sending recognition callback for id %d", kwid);
                        model->recognition_callback(&event->common, model->recognition_cookie);
                        // Update the config so that it will be used during the streaming
                        stdev->last_keyword_detected_config = model->config;
                    }
                    free(event);
                } else {
                    ALOGE("Failed to allocate memory for the event");
                }
            }

            read(fds[2].fd, &temp, sizeof(uint64_t));
        }
#endif // SIMULATE_GSM_TEST_STUB
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

static int stdev_get_properties(const struct sound_trigger_hw_device *dev __unused,
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

    ALOGD("%s stdev %p", __func__, stdev);

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
    if (-1 == i) {
        ALOGE("%s: Can't load model no free slots available", __func__);
        ret = -ENOSYS;
        goto exit;
    }

    *handle = i;
    ALOGV("%s: Loading keyword model handle(%d) type(%d)", __func__, *handle, sound_model->type);
    // This will need to be replaced with UUID once they are fixed
    stdev->models[i].kw_id          = (i + 1);
    stdev->models[i].model_handle   = *handle;
    stdev->models[i].type           = sound_model->type;
    stdev->models[i].uuid           = sound_model->uuid;
    stdev->models[i].sound_model_callback   = callback;
    stdev->models[i].sound_model_cookie     = cookie;
    stdev->models[i].recognition_callback   = NULL;
    stdev->models[i].recognition_cookie     = NULL;

    stdev->models[i].data = malloc(kw_model_sz);
    if (NULL == stdev->models[i].data) {
        ALOGE("%s: Warning, could not allocate memory for keyword model data, cannot redownload on crash", __func__);
        stdev->models[i].data_sz = 0;
    } else {
        memcpy(stdev->models[i].data, kw_buffer, kw_model_sz);
        stdev->models[i].data_sz = kw_model_sz;
    }

    if (SOUND_MODEL_TYPE_KEYPHRASE == sound_model->type ||
        true == check_uuid_equality(sound_model->uuid, stdev->ambient_model_uuid)) {
        // Send the keyword model to the chip only for hotword and ambient audio
        if (SOUND_MODEL_TYPE_KEYPHRASE == sound_model->type) {
            err = write_model(kw_buffer, kw_model_sz, OK_GOOGLE_KW_ID);
            stdev->models[i].kw_id = OK_GOOGLE_KW_ID;
        }
        else if (true == check_uuid_equality(sound_model->uuid, stdev->ambient_model_uuid)) {
            err = write_model(kw_buffer, kw_model_sz, AMBIENT_KW_ID);
            stdev->models[i].kw_id = AMBIENT_KW_ID;
        }
        if (-1 == err) {
            ALOGE("%s: Failed to load the keyword model error - %d (%s)", __func__, errno, strerror(errno));
            ret = errno;
            if (stdev->models[i].data) {
                free(stdev->models[i].data);
                stdev->models[i].data = NULL;
                stdev->models[i].data_sz = 0;
            }
            goto exit;
        }
    } else if (SOUND_MODEL_TYPE_GENERIC == sound_model->type &&
               !check_uuid_equality(sound_model->uuid, stdev->sensor_model_uuid)) {
        /*
         * [TODO]Temp for Sound Trigger Test ApK verify.
         *       Will remove in the future.
         */
        err = write_model(kw_buffer, kw_model_sz, OK_GOOGLE_KW_ID);
        stdev->models[i].kw_id = OK_GOOGLE_KW_ID;
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
    if (false == stdev->models[handle].is_loaded) {
        ALOGE("%s: Invalid model(%d) being called for unload", __func__, handle);
        ret = -EINVAL;
        goto exit;
    }

    stdev->models[handle].sound_model_callback = NULL;
    stdev->models[handle].sound_model_cookie   = NULL;
    stdev->models[handle].is_loaded = false;
    if (stdev->models[handle].data) {
        free(stdev->models[handle].data);
        stdev->models[handle].data = NULL;
        stdev->models[handle].data_sz = 0;
    }

    ALOGE("%s: Successfully unloaded the model, handle - %d", __func__, handle);

    // To Do Need to unload the keyword model files
    //unload_all_models();
exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s handle %d-", __func__, handle);
    return ret;
}

static int stdev_start_recognition(const struct sound_trigger_hw_device *dev,
        sound_model_handle_t sound_model_handle,
        const struct sound_trigger_recognition_config *config,
        recognition_callback_t callback,
        void *cookie)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0;
    struct model_info *model = &stdev->models[sound_model_handle];

    ALOGD("%s stdev %p", __func__, stdev);
    ALOGD("+%s sound model %d+", __func__, sound_model_handle);
    pthread_mutex_lock(&stdev->lock);

    if (NULL == callback) {
        ALOGE("%s: recognition_callback is null", __func__);
        status = -EINVAL;
        goto exit;
    }

    if (NULL != model->config) {
        free(model->config);
        model->config = NULL;
    }

    if (NULL != config) {
        model->config = (struct sound_trigger_recognition_config *) malloc (sizeof(*config));
        if (NULL == model->config) {
            ALOGE("%s: Failed to allocate memory for model config", __func__);
            status = -ENOMEM;
            goto exit;
        }

        memcpy(model->config, config, sizeof(*config));

        ALOGE("%s: Is capture requested %d", __func__, config->capture_requested);
    } else {
        ALOGV("%s: config is null", __func__);
        model->config = NULL;
    }

    model->recognition_callback = callback;
    model->recognition_cookie   = cookie;

    if (check_uuid_equality(model->uuid, stdev->sensor_model_uuid)) {
        set_sensor_route(true);
    } else if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        set_ambient_audio_route(true);
    } else {
#ifdef SIMULATE_GSM_TEST_STUB
        if (SOUND_MODEL_TYPE_GENERIC != model->type) {
#endif // SIMULATE_GSM_TEST_STUB
        if (false == stdev->is_recog_in_prog) {
            ALOGE("This is the first keyword so send the start recoginition to the chip");
            status = start_cvq();

            stdev->is_recog_in_prog = true;

            if (enable_mic_route(true)) {
                ALOGE("failed to enable mic route");
            }
        }
#ifdef SIMULATE_GSM_TEST_STUB
        }
#endif // SIMULATE_GSM_TEST_STUB
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s sound model %d-", __func__, sound_model_handle);
    return status;
}

static int stdev_stop_recognition(const struct sound_trigger_hw_device *dev,
        sound_model_handle_t sound_model_handle)
{
    struct knowles_sound_trigger_device *stdev =
        (struct knowles_sound_trigger_device *)dev;
    int status = 0;
    struct model_info *model = &stdev->models[sound_model_handle];
    ALOGD("+%s sound model %d+", __func__, sound_model_handle);
    pthread_mutex_lock(&stdev->lock);

    if (NULL != model->config) {
        free(model->config);
        model->config       = NULL;
    }

    model->recognition_callback = NULL;
    model->recognition_cookie   = NULL;

    if (check_uuid_equality(model->uuid, stdev->sensor_model_uuid)) {
        set_sensor_route(false);
    } else if (check_uuid_equality(model->uuid, stdev->ambient_model_uuid)) {
        set_ambient_audio_route(false);
    } else {
#ifdef SIMULATE_GSM_TEST_STUB
        if (SOUND_MODEL_TYPE_GENERIC != model->type) {
#endif // SIMULATE_GSM_TEST_STUB
        if (true == stdev->is_recog_in_prog) {
            ALOGE("None of keywords are active so send the stop recoginition to the chip");
            if (enable_mic_route(false)) {
                ALOGE("failed to disable mic route");
            }
            status = stop_cvq();
            stdev->is_recog_in_prog = false;
        }
#ifdef SIMULATE_GSM_TEST_STUB
        }
#endif // SIMULATE_GSM_TEST_STUB
    }

    pthread_mutex_unlock(&stdev->lock);
    ALOGD("-%s sound model %d-", __func__, sound_model_handle);
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
int stdev_open_for_streaming()
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int ret = 0;

    ALOGI("%s: Entering", __func__);
    pthread_mutex_lock(&stdev->lock);
    if (false == stdev->opened) {
        ALOGE("%s: Error SoundTrigger has not been opened", __func__);
        goto exit;
    }

    if (false == stdev->is_streaming) {
        if (NULL == stdev->adnc_strm_open) {
            ALOGE("%s: Error adnc streaming not supported", __func__);
        } else {
            bool keyword_stripping_enabled = false;
            stdev->adnc_strm_handle = stdev->adnc_strm_open(keyword_stripping_enabled, 0,
                                                            stdev->last_detected_model_type);
            if (stdev->adnc_strm_handle) {
                ALOGD("Successfully opened adnc streaming");
                stdev->is_streaming = true;
                ret = stdev->adnc_strm_handle;
            } else {
                ALOGE("%s: DSP is currently not streaming", __func__);
            }
        }
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

__attribute__ ((visibility ("default")))
size_t stdev_read_samples(int audio_handle, void *buffer, size_t  buffer_len)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    size_t ret = 0;

    //ALOGI("%s: Entering", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (false == stdev->opened || false == stdev->is_streaming) {
        ALOGE("%s: Error SoundTrigger has not been opened or DSP is not streaming", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (0 == audio_handle || (audio_handle != stdev->adnc_strm_handle)) {
        ALOGE("%s: Error - Invalid audio handle", __func__);
        ret = -EINVAL;
        goto exit;
    }

    ALOGE("%s: soundtrigger HAL adnc_strm_read", __func__);
    ret = stdev->adnc_strm_read(stdev->adnc_strm_handle, buffer, buffer_len);
    ALOGV("%s: Sent %zu bytes to buffer", __func__, ret);

exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

__attribute__ ((visibility ("default")))
int stdev_close_for_streaming(int audio_handle)
{
    struct knowles_sound_trigger_device *stdev = &g_stdev;
    int ret = 0;

    ALOGI("%s: Entering", __func__);
    pthread_mutex_lock(&stdev->lock);

    if (0 == audio_handle || (audio_handle != stdev->adnc_strm_handle)) {
        ALOGE("%s: Error - Invalid audio handle", __func__);
        ret = -EINVAL;
        goto exit;
    }

    if (stdev->adnc_strm_handle) {
        stdev->adnc_strm_close(stdev->adnc_strm_handle);
        stdev->adnc_strm_handle = 0;
        stdev->is_streaming = 0;
    }

exit:
    pthread_mutex_unlock(&stdev->lock);
    return ret;
}

__attribute__ ((visibility ("default")))
audio_io_handle_t stdev_get_audio_handle()
{
    if (NULL == g_stdev.last_keyword_detected_config) {
        ALOGI("%s: Config is NULL so returning audio handle as 0", __func__);
        return 0;
    }

    ALOGI("%s: Audio Handle is %d", __func__, g_stdev.last_keyword_detected_config->capture_handle);

    return g_stdev.last_keyword_detected_config->capture_handle;
}


static int open_streaming_lib(struct knowles_sound_trigger_device *stdev)
{
    int ret = 0;

    if (access(ADNC_STRM_LIBRARY_PATH, R_OK) == 0) {
        stdev->adnc_cvq_strm_lib = dlopen(ADNC_STRM_LIBRARY_PATH, RTLD_NOW);
        if (stdev->adnc_cvq_strm_lib == NULL) {
            char const *err_str = dlerror();
            ALOGE("%s: module = %s error = %s", __func__, ADNC_STRM_LIBRARY_PATH, err_str ? err_str : "unknown");
            ALOGE("%s: DLOPEN failed for %s", __func__, ADNC_STRM_LIBRARY_PATH);
        } else {
            ALOGV("%s: DLOPEN successful for %s", __func__, ADNC_STRM_LIBRARY_PATH);
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
            if (!stdev->adnc_strm_open || !stdev->adnc_strm_read || !stdev->adnc_strm_close) {
                ALOGE("%s: Error grabbing functions in %s", __func__, ADNC_STRM_LIBRARY_PATH);
                stdev->adnc_strm_open = 0;
                stdev->adnc_strm_read = 0;
                stdev->adnc_strm_close = 0;
            }
        }
    }

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
    if (0 != ret) {
        ALOGE("%s: Couldnot open the streaming library", __func__);
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

    stdev->is_recog_in_prog = false;
    stdev->opened = true;
    /* Initialize all member variable */
    for (i = 0; i < MAX_MODELS; i++) {
        stdev->models[i].config = NULL;
        stdev->models[i].data   = NULL;
        stdev->models[i].data_sz = 0;
        stdev->models[i].is_loaded = false;
        stdev->last_keyword_detected_config = NULL;
    }

#ifdef SIMULATE_GSM_TEST_STUB
    stdev->last_recog_model_id = -1;
#endif // SIMULATE_GSM_TEST_STUB

    str_to_uuid(SENSOR_MANAGER_MODEL, &stdev->sensor_model_uuid);
    str_to_uuid(AMBIENT_AUDIO_MODEL, &stdev->ambient_model_uuid);

    *device = &stdev->device.common; /* same address as stdev */

    ALOGD("%s: Wait for Firmware download to be completed", __func__);
    for(int max_attempts = 0; max_attempts < 300; max_attempts++) {
        struct stat ready_stat;
        const char *ready_path = "/sys/bus/spi/devices/spi1.0/iaxxx/fw_dl_complete";
        ALOGD("%s: Checking for %s", __func__, ready_path);
        int ready_result = stat(ready_path, &ready_stat);
        if (0 == ready_result) {
            ALOGD("%s: Found %s", __func__, ready_path);
            break;
        } else {
            ALOGD("%s: Didn't find %s, sleeping for one sec", __func__, ready_path);
            sleep(1);
        }
    }

    setup_mic_routes();
    init_params();

    ALOGD("stdev before pthread_create %p", stdev);
    // Create a thread to handle all events from kernel
    pthread_create(&stdev->callback_thread, (const pthread_attr_t *) NULL,
            callback_thread_loop, stdev);

exit:
    pthread_mutex_unlock(&stdev->lock);
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

