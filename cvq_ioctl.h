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

int force_set_sensor_route(bool enable);
int write_model(struct iaxxx_odsp_hw *odsp_hdl, unsigned char *data,
                int length, int kw_type);
int get_event(struct iaxxx_odsp_hw *odsp_hdl,
            struct iaxxx_get_event_info *ge);
int setup_chip(struct iaxxx_odsp_hw *odsp_hdl);
int enable_mic_route(struct audio_route *route_hdl, int enable);
int set_sensor_route(struct audio_route *route_hdl, bool enable);
int set_ambient_audio_route(struct iaxxx_odsp_hw *odsp_hdl,
                            struct audio_route *route_hdl,
                            bool bargein);
int tear_ambient_audio_route(struct iaxxx_odsp_hw *odsp_hdl,
                            struct audio_route *route_hdl,
                            bool bargein);
int set_entity_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl, bool bargein);
int tear_entity_route(struct iaxxx_odsp_hw *odsp_hdl,
                    struct audio_route *route_hdl, bool bargein);
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
