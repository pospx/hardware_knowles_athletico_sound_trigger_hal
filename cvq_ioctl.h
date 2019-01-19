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

#ifndef _CVQ_IOCTL_H
#define _CVQ_IOCTL_H

#include "iaxxx_odsp_hw.h"
#include <audio_route/audio_route.h>
#include <tinyalsa/asoundlib.h>

#define HOTWORD_MASK 0x1
#define AMBIENT_MASK 0x2
#define ENTITY_MASK  0x4
#define PLUGIN1_MASK (HOTWORD_MASK)
#define PLUGIN2_MASK (AMBIENT_MASK | ENTITY_MASK)

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
#define ENTITY_SLOT_ID      5

#define HOTWORD_UNLOAD_PARAM_ID                 1
#define AMBIENT_ENTITY_UNLOAD_PARAM_ID          1
#define AMBIENT_ENTITY_RESET_PARAM_ID           2
#define AMBIENT_ENTITY_GET_MODEL_STATE_PARAM_ID 6

#define HOTWORD_UNLOAD_PARAM_VAL            1
#define AMBIENT_UNLOAD_PARAM_VAL            3
#define ENTITY_UNLOAD_PARAM_VAL             5
#define AMBIENT_ENTITY_RESET_PARAM_VAL      3

#define BUFFER_PACKAGE              "BufferPackage.bin"
#define BUFFER_CONFIG_VAL           "BufferConfigVal.bin"
#define BUFFER_CONFIG_OSLO_VAL      "BufferConfigValOslo.bin"
#define BUFFER_CONFIG_AMBIENT_VAL   "BufferConfigValAmbient.bin"
#define OK_GOOGLE_PACKAGE           "OkGooglePackage.bin"
#define AMBIENT_EC_PACKAGE          "AmbientECPackage.bin"
#define AMBIENT_DA_PACKAGE          "AmbientDAPackage.bin"
#define SENSOR_PACKAGE              "OsloSensorPackage.bin"
#define SENSOR_CONFIG_VAL           "OsloSensorConfig.bin"
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

int force_set_sensor_route(bool enable);
int write_model(struct iaxxx_odsp_hw *odsp_hdl, unsigned char *data,
                int length, int kw_type);
int get_model_state(struct iaxxx_odsp_hw *odsp_hdl, const uint32_t inst_id,
                    const uint32_t param_val);
int get_event(struct iaxxx_odsp_hw *odsp_hdl,
            struct iaxxx_get_event_info *ge);
int setup_chip(struct iaxxx_odsp_hw *odsp_hdl);
int enable_mic_route(struct audio_route *route_hdl, int enable);
int set_sensor_route(struct audio_route *route_hdl, bool enable);
int set_ambient_entity_state(struct iaxxx_odsp_hw *odsp_hdl,
                        unsigned int current);
int tear_ambient_entity_state(struct iaxxx_odsp_hw *odsp_hdl,
                        unsigned int current);
int set_ambient_entity_route(struct audio_route *route_hdl, bool bargein);
int tear_ambient_entity_route(struct audio_route *route_hdl, bool bargein);
int set_hotword_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl, bool bargein);
int tear_hotword_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl, bool bargein);
int set_chre_audio_route(struct audio_route *route_hdl, bool bargein);
int tear_chre_audio_route(struct audio_route *route_hdl, bool bargein);
int reset_ambient_plugin(struct iaxxx_odsp_hw *odsp_hdl);
int enable_bargein_route(struct audio_route *route_hdl, bool enable);
int flush_model(struct iaxxx_odsp_hw *odsp_hdl, int kw_type);
int get_entity_param_blk(struct iaxxx_odsp_hw *odsp_hdl, void *payload, unsigned int payload_size);

#endif /* _CVQ_IOCTL_H */