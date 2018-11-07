/************************************************************************//**
 * @file oslo_config_test.c
 *
 * @brief
 *
 * Revision Information:
 *
 * Copyright 2017 Knowles Corporation. All rights reserved.
 *
 * All information, including software, contained herein is and remains
 * the property of Knowles Corporation. The intellectual and technical
 * concepts contained herein are proprietary to Knowles Corporation
 * and may be covered by U.S. and foreign patents, patents in process,
 * and/or are protected by trade secret and/or copyright law.
 * This information may only be used in accordance with the applicable
 * Knowles SDK License. Dissemination of this information or distribution
 * of this material is strictly forbidden unless in accordance with the
 * applicable Knowles SDK License.
 *
 *
 * KNOWLES SOURCE CODE IS STRICTLY PROVIDED "AS IS" WITHOUT ANY WARRANTY
 * WHATSOEVER, AND KNOWLES EXPRESSLY DISCLAIMS ALL WARRANTIES,
 * EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. KNOWLES
 * SHALL NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF
 * USING, MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
 * IN CERTAIN STATES, THE LAW MAY NOT ALLOW KNOWLES TO DISCLAIM OR EXCLUDE
 * WARRANTIES OR DISCLAIM DAMAGES, SO THE ABOVE DISCLAIMERS MAY NOT APPLY.
 * IN SUCH EVENT, KNOWLES' AGGREGATE LIABILITY SHALL NOT EXCEED
 * FIFTY DOLLARS ($50.00).
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "iaxxx-module.h"
#include "cvq_ioctl.h"

#define LOG_TAG "ia_sensor_param_test"

#include <cutils/log.h>

#define GETOPT_HELP_CHAR (CHAR_MIN - 2)

#define DEV_NODE "/dev/iaxxx-module-celldrv"

#define BGT60TR24C_NUM_REGISTERS        0x60

/* copy of SensorDriver.h from athletico repository */
#define OSLO_PRESET_CONFIG_START_INDEX  100
#define OSLO_CONTROL_START_INDEX        200
#define OSLO_SETTING_START_INDEX        300
#define OSLO_REGISTER_RW_START_INDEX    400

typedef enum
{
    SENSOR_PARAM_SENSOR_SPEC = 0,
    SENSOR_PARAM_SAMP_RATE,
    SENSOR_PARAM_SAMP_SIZE,
    SENSOR_PARAM_INTF_SPEED,
    SENSOR_PARAM_DRIVER_STATE,
    SENSOR_PARAM_FRAMES_PROCESSED,

    /* oslo preset configurations */
    OSLO_CONFIG_DEFAULT = OSLO_PRESET_CONFIG_START_INDEX,
    OSLO_CONFIG_PRESENCE,
    OSLO_CONFIG_CONTINUOUS,
    OSLO_CONFIG_PRESENCE_SLOW,
    OSLO_CONFIG_FACTORY_PRESENCE,
    OSLO_CONFIG_FACTORY_REACH,

    /* oslo control - restart oslo when settings change */
    OSLO_CONTROL_RESTART = OSLO_CONTROL_START_INDEX,
    OSLO_CONTROL_STRIP_HEADERS,
    OSLO_CONTROL_SLPI_INT,
    OSLO_CONTROL_STOP,

    /* oslo settings */
    OSLO_PARAM_REQUEST_RATE = OSLO_SETTING_START_INDEX,
    OSLO_PARAM_REQUEST_ANTENNA_MASK,
    OSLO_PARAM_TX_POWER,
    OSLO_PARAM_LOWER_FREQ,
    OSLO_PARAM_UPPER_FREQ,
    OSLO_PARAM_SAMPLES_PER_CHIRP,
    OSLO_PARAM_BASEBAND_VGA_GAIN,
    OSLO_PARAM_BURST_CHIRP_COUNT,
    OSLO_PARAM_BURST_CHIRP_RATE,
    OSLO_PARAM_BURST_POWER_MODE,
    OSLO_PARAM_BURST_INTERCHIRP_POWER_MODE,
    OSLO_PARAM_STARTUP_TIMING_WAKE_UP_TIME_100NS,
    OSLO_PARAM_STARTUP_TIMING_PLL_SETTLE_TIME_COARSE_100NS,
    OSLO_PARAM_STARTUP_TIMING_PLL_SETTLE_TIME_FINE_100NS,
    OSLO_PARAM_STARTUP_TIMING_OSCILLATOR_USEC,
    OSLO_PARAM_PRE_CHIRP_DELAY_100NS,
    OSLO_PARAM_POST_CHIRP_DELAY_100NS,
    OSLO_PARAM_CHIRP_PA_DELAY_100NS,
    OSLO_PARAM_CHIRP_ADC_DELAY_100NS,
    OSLO_PARAM_VISUALIZER_DATA_TYPE,
    OSLO_PARAM_OSCILLATOR_MODE,
    OSLO_PARAM_HP_GAIN_CH1,
    OSLO_PARAM_HP_GAIN_CH2,
    OSLO_PARAM_HP_GAIN_CH3,
    OSLO_PARAM_HP_GAIN_CH4,
    OSLO_PARAM_BASEBAND_RESET_PERIOD_1NS,
    OSLO_PARAM_HP_CUTOFF_CH1,
    OSLO_PARAM_HP_CUTOFF_CH2,
    OSLO_PARAM_HP_CUTOFF_CH3,
    OSLO_PARAM_HP_CUTOFF_CH4,
    OSLO_PARAM_PHASE_CONFIG,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_PLL,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_VCO,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_FDIV,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_BASEBAND,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_RF,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_MADC,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_MADC_BANDGAP,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_SADC,
    OSLO_PARAM_IDLE_SETTINGS_ENABLE_SADC_BANDGAP,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_PLL,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_VCO,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_FDIV,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_BASEBAND,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_RF,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_MADC,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_MADC_BANDGAP,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_SADC,
    OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_SADC_BANDGAP,
    OSLO_PARAM_TOSS_COUNT,
    OSLO_PARAM_CHIRP_DIRECTION,
    OSLO_PARAM_ADC_SAMPLE_RATE,
    OSLO_PARAM_CHARGE_PUMP,

    /* oslo register r/w */
    OSLO_REGISTER_MIN = OSLO_REGISTER_RW_START_INDEX,
    OSLO_REGISTER_MAX = OSLO_REGISTER_RW_START_INDEX + 99,

    SENSOR_PARAM_NUM,

    /* Force enums to be of size int */
    SENSOR_PARAM_FORCE_SIZE = INT_MAX,
} sensor_param_t;

typedef struct {
    int setting_id;
    char *setting_name;
} oslo_settings_t;

/* map setting name to param id */
static const oslo_settings_t oslo_settings[] =
{
    {OSLO_CONFIG_DEFAULT,                                   "config_default"},
    {OSLO_CONFIG_PRESENCE,                                  "config_presence"},
    {OSLO_CONFIG_CONTINUOUS,                                "config_continuous"},
    {OSLO_CONFIG_PRESENCE_SLOW,                             "config_presence_slow"},
    {OSLO_CONFIG_FACTORY_PRESENCE,                          "config_factory_presence"},
    {OSLO_CONFIG_FACTORY_REACH,                             "config_factory_reach"},
    {OSLO_CONTROL_RESTART,                                  "oslo_control_restart"},
    {OSLO_CONTROL_STRIP_HEADERS,                            "oslo_control_strip_headers"},
    {OSLO_CONTROL_SLPI_INT,                                 "oslo_control_slpi_interrupt"},
    {OSLO_CONTROL_STOP,                                     "oslo_control_stop"},
    {OSLO_PARAM_REQUEST_RATE,                               "param_request_rate"},
    {OSLO_PARAM_REQUEST_ANTENNA_MASK,                       "param_request_antenna_mask"},
    {OSLO_PARAM_TX_POWER,                                   "param_tx_power"},
    {OSLO_PARAM_LOWER_FREQ,                                 "param_lower_freq"},
    {OSLO_PARAM_UPPER_FREQ,                                 "param_upper_freq"},
    {OSLO_PARAM_SAMPLES_PER_CHIRP,                          "param_samples_per_chirp"},
    {OSLO_PARAM_BASEBAND_VGA_GAIN,                          "param_baseband_vga_gain"},
    {OSLO_PARAM_BURST_CHIRP_COUNT,                          "param_burst_chirp_count"},
    {OSLO_PARAM_BURST_CHIRP_RATE,                           "param_burst_chirp_rate"},
    {OSLO_PARAM_BURST_POWER_MODE,                           "param_burst_power_mode"},
    {OSLO_PARAM_BURST_INTERCHIRP_POWER_MODE,                "param_burst_interchirp_power_mode"},
    {OSLO_PARAM_STARTUP_TIMING_WAKE_UP_TIME_100NS,          "param_startup_timing_wake_up_time_100ns"},
    {OSLO_PARAM_STARTUP_TIMING_PLL_SETTLE_TIME_COARSE_100NS,"param_startup_timing_pll_settle_time_coarse_100ns"},
    {OSLO_PARAM_STARTUP_TIMING_PLL_SETTLE_TIME_FINE_100NS,  "param_startup_timing_pll_settle_time_fine_100ns"},
    {OSLO_PARAM_STARTUP_TIMING_OSCILLATOR_USEC,             "param_startup_timing_oscillator_usec"},
    {OSLO_PARAM_PRE_CHIRP_DELAY_100NS,                      "param_pre_chirp_delay_100ns"},
    {OSLO_PARAM_POST_CHIRP_DELAY_100NS,                     "param_post_chirp_delay_100ns"},
    {OSLO_PARAM_CHIRP_PA_DELAY_100NS,                       "param_chirp_pa_delay_100ns"},
    {OSLO_PARAM_CHIRP_ADC_DELAY_100NS,                      "param_chirp_adc_delay_100ns"},
    {OSLO_PARAM_VISUALIZER_DATA_TYPE,                       "param_visualizer_data_type"},
    {OSLO_PARAM_OSCILLATOR_MODE,                            "param_oscillator_mode"},
    {OSLO_PARAM_HP_GAIN_CH1,                                "param_hp_gain_ch1"},
    {OSLO_PARAM_HP_GAIN_CH2,                                "param_hp_gain_ch2"},
    {OSLO_PARAM_HP_GAIN_CH3,                                "param_hp_gain_ch3"},
    {OSLO_PARAM_HP_GAIN_CH4,                                "param_hp_gain_ch4"},
    {OSLO_PARAM_BASEBAND_RESET_PERIOD_1NS,                  "param_baseband_reset_period_1ns"},
    {OSLO_PARAM_HP_CUTOFF_CH1,                              "param_hp_cutoff_ch1"},
    {OSLO_PARAM_HP_CUTOFF_CH2,                              "param_hp_cutoff_ch2"},
    {OSLO_PARAM_HP_CUTOFF_CH3,                              "param_hp_cutoff_ch3"},
    {OSLO_PARAM_HP_CUTOFF_CH4,                              "param_hp_cutoff_ch4"},
    {OSLO_PARAM_PHASE_CONFIG,                               "param_phase_config"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_PLL,                   "param_idle_settings_enable_pll"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_VCO,                   "param_idle_settings_enable_vco"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_FDIV,                  "param_idle_settings_enable_fdiv"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_BASEBAND,              "param_idle_settings_enable_baseband"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_RF,                    "param_idle_settings_enable_rf"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_MADC,                  "param_idle_settings_enable_madc"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_MADC_BANDGAP,          "param_idle_settings_enable_madc_bandgap"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_SADC,                  "param_idle_settings_enable_sadc"},
    {OSLO_PARAM_IDLE_SETTINGS_ENABLE_SADC_BANDGAP,          "param_idle_settings_enable_sadc_bandgap"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_PLL,             "param_deep_sleep_settings_enable_pll"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_VCO,             "param_deep_sleep_settings_enable_vco"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_FDIV,            "param_deep_sleep_settings_enable_fdiv"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_BASEBAND,        "param_deep_sleep_settings_enable_baseband"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_RF,              "param_deep_sleep_settings_enable_rf"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_MADC,            "param_deep_sleep_settings_enable_madc"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_MADC_BANDGAP,    "param_deep_sleep_settings_enable_madc_bandgap"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_SADC,            "param_deep_sleep_settings_enable_sadc"},
    {OSLO_PARAM_DEEP_SLEEP_SETTINGS_ENABLE_SADC_BANDGAP,    "param_deep_sleep_settings_enable_sadc_bandgap"},
    {OSLO_PARAM_CHIRP_DIRECTION,                            "param_chirp_direction"},
    {OSLO_PARAM_ADC_SAMPLE_RATE,                            "param_adc_sample_rate"},
    {OSLO_PARAM_CHARGE_PUMP,                                "param_charge_pump"},
};

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))
#define OSLO_SETTINGS_SIZE COUNT_OF(oslo_settings)

struct ia_sensor_mgr {
    FILE *dev_node;
};

static struct option const long_options[] =
{
 {"setparamid", required_argument, NULL, 's'},
 {"value", required_argument, NULL, 'v'},
 {"getparamid", required_argument, NULL, 'g'},
 {"ping", required_argument, NULL, 'p'},
 {"route", required_argument, NULL, 'r'},
 {"readregister", required_argument, NULL, 'd'},
 {"writeregister", required_argument, NULL, 'w'},
 {"help", no_argument, NULL, GETOPT_HELP_CHAR},
 {NULL, 0, NULL, 0}
};

void usage() {
    fputs("\
    USAGE -\n\
    -------\n\
    1) oslo_config_test -s <param_name> -v <param_val>\n\
    2) oslo_config_test -g <param_name>\n\
    \n\
    In the first form, set a parameter with a value.\n\
    In the second form, get a value of a parameter\n\
    \n\
    3) oslo_config_test -p <timeout>\n\
    4) oslo_config_test -r <1/0>\n\
    5) oslo_config_test -d <reg_addr>\n\
    6) oslo_config_test -w <reg_addr> -v <reg_val>\n\
    ", stdout);

    fputs("\n\
    OPTIONS - \n\
    ---------\n\
    -s          Set a parameter using its <param_name>.\n\
    -v          Set this value for the parameter ID that was passed with\n\
                the option '-s'. Using this option alone is invalid.\n\
    -g          Get the value of a parameter using its <param_name>.\n\
    -p          Ping oslo sensor.\n\
    -r          Set sensor route.\n\
    -d          Read register.\n\
    -w          Write register.\n\
    ", stdout);

    fputs("\n\
    List of all <param_name>\n\
    ---------\n\
    ", stdout);

    unsigned int i;
    for (i = 0; i < OSLO_SETTINGS_SIZE; i ++) {
        fprintf(stderr, "    %s\n", oslo_settings[i].setting_name, stdout);
    }

    exit(EXIT_FAILURE);
}

void set_param(struct ia_sensor_mgr *smd, int param_id, float param_val) {
    int err = 0;
    struct iaxxx_sensor_param sp;

    if (NULL == smd) {
        ALOGE("%s: NULL handle passed", __func__);
        return;
    }

    sp.inst_id      = 0;
    sp.block_id     = 0;

    sp.param_id = param_id;
    sp.param_val = param_val;

    ALOGD("Set sensor param 0x%X with value %f",
          param_id, param_val);
    fprintf(stdout, "Set sensor param 0x%X with value %f\n",
          param_id, param_val);

    err = ioctl(fileno(smd->dev_node), MODULE_SENSOR_SET_PARAM, (unsigned long) &sp);
    if (-1 == err) {
        ALOGE("%s: ERROR: MODULE_SENSOR_SET_PARAM IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
        return;
    }

}

uint32_t get_param(struct ia_sensor_mgr *smd, int param_id) {
    struct iaxxx_sensor_param sp;
    int err = 0;
    ALOGD ("Get param - param_id 0x%X", param_id);

    if (NULL == smd) {
        ALOGE("%s: NULL handle passed", __func__);
        return 0;
    }

    sp.inst_id      = 0;
    sp.block_id     = 0;

    sp.param_id = param_id;
    sp.param_val = 0;
    err = ioctl(fileno(smd->dev_node), MODULE_SENSOR_GET_PARAM, (unsigned long) &sp);
    if (-1 == err) {
        ALOGE("%s: ERROR: MODULE_SENSOR_GET_PARAM IOCTL failed with error %d(%s)", __func__, errno, strerror(errno));
        return 0;
    } else {
        ALOGD("Value of param 0x%X is %zu",
              sp.param_id, sp.param_val);
        fprintf(stdout, "Value of param 0x%X is %zu\n",
              sp.param_id, sp.param_val);
    }

    return sp.param_val;
}

int oslo_setting_lookup(char *in)
{
    unsigned int i;
    int ret = -1;

    for (i = 0; i < OSLO_SETTINGS_SIZE; i++)
    {
        if (strcmp(in, oslo_settings[i].setting_name) == 0)
        {
            ret = oslo_settings[i].setting_id;
            break;
        }
    }

    return ret;
}

bool ping_test(struct ia_sensor_mgr *smd, uint32_t ping_timeout_sec) {
    bool ret = false;
    uint32_t radar_frames_initial;
    time_t start_time;

    force_set_sensor_route(true);

    start_time = time(NULL);
    radar_frames_initial = get_param(smd, SENSOR_PARAM_FRAMES_PROCESSED);

    do {
        uint32_t radar_frames = get_param(smd, SENSOR_PARAM_FRAMES_PROCESSED);
        if (radar_frames > radar_frames_initial) {
            ALOGD("%s: frame number increased (%d, %d)",
                  __func__, radar_frames_initial, radar_frames);
            ret = true;
            break;
        }
        else
            usleep(50 * 1000); // 50ms
    } while (difftime(time(NULL), start_time) <= ping_timeout_sec);

    force_set_sensor_route(false);

    ALOGD("%s: %s", __func__, (ret ? "PASS" : "FAIL"));
    fprintf(stdout, "%s: %s\n", __func__, (ret ? "PASS" : "FAIL"));

    return ret;
}

void read_register(struct ia_sensor_mgr *smd, uint32_t reg_addr) {
    uint32_t reg_val;

    if (reg_addr >= BGT60TR24C_NUM_REGISTERS) {
        fprintf(stdout, "Invalid reg addr:0x%02x\n", reg_addr);
        return;
    }

    reg_val = get_param(smd, OSLO_REGISTER_MIN + reg_addr);

    ALOGD("reg[0x%02x]: 0x%06x\n", reg_addr, reg_val);
    fprintf(stdout, "reg[0x%02x]: 0x%06x\n", reg_addr, reg_val);
}

void write_register(struct ia_sensor_mgr *smd, uint32_t reg_addr, uint32_t reg_val) {
    if (reg_addr >= BGT60TR24C_NUM_REGISTERS) {
        fprintf(stdout, "Invalid reg addr:0x%02x\n", reg_addr);
        return;
    }

    set_param(smd, OSLO_REGISTER_MIN + reg_addr, reg_val);

    ALOGD("Write reg[0x%02x] val:0x%06x\n", reg_addr, reg_val);
    fprintf(stdout, "Write reg[0x%02x] val:0x%06x\n", reg_addr, reg_val);
}

int main(int argc, char *argv[]) {
    struct ia_sensor_mgr * smd;
    char use_case;
    int param_id = -1;
    int c;
    float param_val = 0.0;
    uint32_t ping_timeout_sec;
    bool route_enable;
    uint32_t reg_addr;
    uint32_t reg_val;
    bool reg_val_set = false;

    if (argc <= 1) {
        usage();
    }

    while ((c = getopt_long (argc, argv, "s:v:g:p:r:d:w:", long_options, NULL)) != -1) {
        switch (c) {
        case 's':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, s option requires an argument");
                usage();
            } else {
                param_id = oslo_setting_lookup(optarg);
                if (param_id == -1)
                {
                    fprintf(stderr, "Invalid setting %s", optarg);
                    usage();
                } else {
                    use_case = 's';
                }
            }
        break;
        case 'v':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, v option requires an argument");
                usage();
            } else {
                if ('s' == use_case) {
                    param_val = strtof(optarg, NULL);
                    use_case = 'v';
                } else if ('w' == use_case) {
                    reg_val = strtoul(optarg, NULL, 0);
                    reg_val_set = true;
                } else {
                    fprintf(stderr, "Incorrect usage, v option should be the second option");
                    usage();
                }
            }
        break;
        case 'g':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, g option requires an argument");
                usage();
            } else {
                param_id = oslo_setting_lookup(optarg);
                if (param_id == -1)
                {
                    fprintf(stderr, "Invalid setting %s", optarg);
                    usage();
                } else {
                    use_case = 'g';
                }
            }
        break;
        case 'p':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, p option requires an argument");
                usage();
            } else {
                ping_timeout_sec = strtoul(optarg, NULL, 0);
                use_case = 'p';
            }
        break;
        case 'r':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, r option requires an argument");
                usage();
            } else {
                route_enable = (strtoul(optarg, NULL, 0) != 0) ? true : false;
                use_case = 'r';
            }
        break;
        case 'd':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, d option requires an argument");
                usage();
            } else {
                reg_addr = strtoul(optarg, NULL, 0);
                use_case = 'd';
            }
        break;
        case 'w':
            if (NULL == optarg) {
                fprintf(stderr, "Incorrect usage, w option requires an argument");
                usage();
            } else {
                reg_addr = strtoul(optarg, NULL, 0);
                use_case = 'w';
            }
        break;
        case GETOPT_HELP_CHAR:
        default:
            usage();
        break;
        }

        smd = (struct ia_sensor_mgr*) malloc(sizeof(struct ia_sensor_mgr));
        if (NULL == smd) {
            ALOGE("%s: ERROR Failed to allocated memory for ia_sensor_mgr", __func__);
            return -EINVAL;
        }

        if((smd->dev_node = fopen(DEV_NODE, "rw")) == NULL) {
            ALOGE("%s: ERROR file %s open for write error: %s\n", __func__, DEV_NODE, strerror(errno));
            free(smd);
            return -EINVAL;
        }

        if ('v' == use_case) {
            set_param(smd, param_id, param_val);
        } else if ('g' == use_case) {
            get_param(smd, param_id);
        } else if ('p' == use_case) {
            ping_test(smd, ping_timeout_sec);
        } else if ('r' == use_case) {
            force_set_sensor_route(route_enable);
        } else if ('d' == use_case) {
           read_register(smd, reg_addr);
        } else if ('w' == use_case) {
           if (reg_val_set)
             write_register(smd, reg_addr, reg_val);
        }

        if (smd->dev_node) {
            fclose(smd->dev_node);
        }

        free(smd);
    }

    return 0;
}
