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

int write_model(unsigned char *data, int length, bool kw_type);
int get_event(struct iaxxx_get_event *ge);
int setup_chip();
int enable_mic_route(int enable);
int set_sensor_route(bool enable);
int set_ambient_audio_route(bool bargein);
int tear_ambient_audio_route(void);
int set_hotword_route(bool bargein);
int tear_hotword_route(void);
int set_chre_audio_route(bool bargein);
int tear_chre_audio_route(void);
int reset_ambient_plugin(void);
int enable_bargein_route(bool enable);

#endif /* _CVQ_IOCTL_H */
