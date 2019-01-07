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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>

#define LOG_TAG "odsp_api_test"

#include <cutils/log.h>
#include "iaxxx_odsp_hw.h"

#define GETOPT_HELP_CHAR (CHAR_MIN - 2)

static struct option const long_options[] =
{
 {"setparamid", required_argument, NULL, 's'},
 {"getparamid", required_argument, NULL, 'g'},
 {"help", no_argument, NULL, GETOPT_HELP_CHAR},
 {NULL, 0, NULL, 0}
};

void usage() {
    fputs("\
    USAGE -\n\
    -------\n\
    1) odsp_api_test -s <param_id> <param_val> <inst_id> <block_id>\n\
    2) odsp_api_test -g <param_id> <inst_id> <block_id>\n\
    \n\
    In the first form, set a parameter with a value, needs instance and block id.\n\
    In the second form, get a parameters value, needs instance and block id.\n\
    ", stdout);

    exit(EXIT_FAILURE);
}

void set_param(struct iaxxx_odsp_hw *ioh, unsigned int param_id,
                unsigned int param_val, unsigned int inst_id,
                unsigned int block_id) {
    int err = 0;

    err = iaxxx_odsp_plugin_set_parameter(ioh, inst_id, param_id, param_val,
                                           block_id);
    if (err != 0) {
        ALOGE("Failed to set parameter id %u with error %d", param_id, err);
    }
}

void get_param(struct iaxxx_odsp_hw *ioh, unsigned int param_id,
                   unsigned int inst_id, unsigned int block_id) {
    int err = 0;
    unsigned int param_val = 0;

    err = iaxxx_odsp_plugin_get_parameter(ioh, inst_id, param_id, block_id,
                                          &param_val);
    if (err != 0) {
        ALOGE("Failed to get parameter value for id %u with error %d",
            param_id, err);
    } else {
        ALOGD("Value of parameter id %u is %u", param_id, param_val);
    }
}

int main(int argc, char *argv[]) {
    struct iaxxx_odsp_hw *ioh = NULL;
    char use_case;
    unsigned int param_id, param_val, inst_id, block_id;
    int c, err = 0;
    // Set param option needs 4 arguments
    const int set_param_req_arg = 4;
    // Get param option needs 3 arguments
    const int get_param_req_arg = 3;

    if (argc <= 1) {
        usage();
    }

    while ((c = getopt_long (argc, argv, "s:g:", long_options, NULL)) != -1) {
        switch (c) {
        case 's':
            use_case = 's';
            // reset optind by 1
            --optind;
            if (optind + set_param_req_arg > argc) {
                usage();
            }

            param_id = strtol(argv[optind], NULL, 0);
            optind++;
            param_val = strtol(argv[optind], NULL, 0);
            optind++;
            inst_id = strtol(argv[optind], NULL, 0);
            optind++;
            block_id = strtol(argv[optind], NULL, 0);
            optind++;

            ALOGE("Set parameter - param_id %d param_val %d inst_id %d "
                  "block_id %d", param_id, param_val, inst_id, block_id);
        break;
        case 'g':
            use_case = 'g';
            // reset optind by 1
            --optind;
            if (optind + get_param_req_arg > argc) {
                usage();
            }

            param_id = strtol(argv[optind], NULL, 0);
            optind++;
            inst_id = strtol(argv[optind], NULL, 0);
            optind++;
            block_id = strtol(argv[optind], NULL, 0);
            optind++;

            ALOGE("Get parameter - param_id %d inst_id %d block_id %d",
                    param_id, inst_id, block_id);
        break;
        default:
            usage();
        break;
        }
    }

    ioh = iaxxx_odsp_init();
    if (ioh == NULL) {
        ALOGE("ERROR: Failed to init odsp HAL");
        err = -1;
        goto exit;
    }

    if (use_case == 's') {
        set_param(ioh, param_id, param_val, inst_id, block_id);
    } else if (use_case == 'g') {
        get_param(ioh, param_id, inst_id, block_id);
    }

exit:
    if (ioh) {
        err = iaxxx_odsp_deinit(ioh);
        if (err != 0) {
            ALOGE("Failed to deinit the odsp HAL");
        }
    }

    return err;
}
