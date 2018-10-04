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

#define LOG_TAG "iaxxx_odsp_hw"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <cutils/log.h>
#include <sys/ioctl.h>
#include <inttypes.h>

#include "iaxxx-odsp.h"

#include "iaxxx_odsp_hw.h"

#define DEV_NODE "/dev/iaxxx-odsp-celldrv"
#define FUNCTION_ENTRY_LOG ALOGV("Entering %s", __func__);
#define FUNCTION_EXIT_LOG ALOGV("Exiting %s", __func__);

#define NAME_MAX_SIZE 256

struct iaxxx_odsp_hw {
    FILE *dev_node;
};

/**
 * Initialize the ODSP HAL
 *
 * Input  - NA
 * Output - Handle to iaxxx_odsp_hw on success, NULL on failure
 */
struct iaxxx_odsp_hw* iaxxx_odsp_init()
{
    struct iaxxx_odsp_hw *ioh = NULL;

    FUNCTION_ENTRY_LOG;

    ioh = (struct iaxxx_odsp_hw *)malloc(sizeof(struct iaxxx_odsp_hw));
    if (ioh == NULL) {
        ALOGE("%s: ERROR: Failed to allocate memory for iaxxx_odsp_hw",
                __func__);
        goto func_exit;
    }

    ioh->dev_node = fopen(DEV_NODE, "rw");
    if (ioh->dev_node == NULL) {
        ALOGE("%s: ERROR: Failed to open %s", __func__, DEV_NODE);
        free(ioh);
        ioh = NULL;
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return ioh;
}

/**
 * De-Initialize the ODSP HAL
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_deinit(struct iaxxx_odsp_hw *odsp_hw_hdl)
{

    int err = 0;

    FUNCTION_ENTRY_LOG;
    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    if (odsp_hw_hdl->dev_node) {
        fclose(odsp_hw_hdl->dev_node);
    }

    free(odsp_hw_hdl);
func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Load a package
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          pkg_name - Relative path to the Package binary (Should be placed in
 *                     the firmware location)
 *          pkg_id - Package ID
 *          proc_id - Process ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_package_load(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const char *pkg_name, const uint32_t pkg_id,
                            const uint32_t proc_id)
{
    int err = 0;
    struct iaxxx_pkg_mgmt_info pkg_info;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    if (pkg_name == NULL) {
        ALOGE("%s: ERROR: Package name cannot be null", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Package name %s, package id %u, process id %u",
        __func__, pkg_name, pkg_id, proc_id);

    strlcpy(pkg_info.pkg_name, pkg_name, NAME_MAX_SIZE);
    pkg_info.pkg_id = pkg_id;
    pkg_info.proc_id = proc_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_LOAD_PACKAGE, (unsigned long)&pkg_info);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Unload a package
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          pkg_name - Relative path to the Package binary (Should be placed in
 *                     the firmware location)
 *          pkg_id - Package ID
 *          proc_id - Process ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_package_unload(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t pkg_id,
                            const uint32_t proc_id)
{
    int err = 0;
    struct iaxxx_pkg_mgmt_info pkg_info;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: package id %u, process id %u", __func__, pkg_id, proc_id);

    pkg_info.pkg_id = pkg_id;
    pkg_info.proc_id = proc_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_UNLOAD_PACKAGE, (unsigned long)&pkg_info);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Create a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          priority - Priority of the plugin
 *          pkg_id - Package ID
 *          plg_idx - Plugin Index*
 *          block_id - Block ID
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_create(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t inst_id,
                            const uint32_t priority,
                            const uint32_t pkg_id,
                            const uint32_t plg_idx,
                            const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Plugin index %u, package id %u, block id %u, instance id %u"
        " priority %u", __func__, plg_idx, pkg_id, block_id, inst_id, priority);

    pi.plg_idx = plg_idx;
    pi.pkg_id = pkg_id;
    pi.block_id = block_id;
    pi.inst_id = inst_id;
    pi.priority = priority;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_CREATE, (unsigned long)&pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set the creation configuration on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 *          cdata - Creation configuration data
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_creation_config(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t block_id,
                                        struct iaxxx_create_config_data cdata)
{
    int err = 0;
    struct iaxxx_plugin_create_cfg pcc;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    pcc.inst_id = inst_id;
    pcc.block_id = block_id;
    pcc.cfg_size = 0;
    switch (cdata.type) {
    case CONFIG_FILE:
        strlcpy(pcc.file_name, cdata.data.fdata.filename, NAME_MAX_SIZE);
        ALOGV("%s: Configuration file name %s", __func__, pcc.file_name);
        break;
    case CONFIG_VALUE:
        pcc.cfg_size = cdata.data.vdata.config_val_sz;
        pcc.cfg_val = cdata.data.vdata.config_val;
        ALOGV("%s: Configuration value %"PRId64, __func__, pcc.cfg_val);
        break;
    default:
        ALOGE("%s: ERROR: Invalid type of configuration type", __func__);
        err = -1;
        goto func_exit;
        break;
    }

    ALOGV("%s: Instance id %u, block id %u", __func__, inst_id, block_id);

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_CREATE_CFG, (unsigned long)&pcc);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Destroy the plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_destroy(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t inst_id,
                            const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: instance id %u, block id %u", __func__, inst_id, block_id);

    pi.block_id = block_id;
    pi.inst_id  = inst_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_DESTROY, (unsigned long) &pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Enable the plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_enable(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t inst_id,
                            const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: instance id %u, block id %u", __func__, inst_id, block_id);

    pi.block_id = block_id;
    pi.inst_id = inst_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_ENABLE, (unsigned long)&pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Disable the plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_disable(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t inst_id,
                            const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: instance id %u, block id %u", __func__, inst_id, block_id);

    pi.block_id = block_id;
    pi.inst_id = inst_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_DISABLE, (unsigned long)&pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Reset the plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_reset(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint32_t inst_id,
                            const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: instance id %u, block id %u", __func__, inst_id, block_id);

    pi.block_id = block_id;
    pi.inst_id = inst_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_DISABLE, (unsigned long)&pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set a parameter on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          param_id - Parameter ID
 *          param_val - Parameter Value*
 *          block_id  - Block ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_parameter(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                    const uint32_t inst_id,
                                    const uint32_t param_id,
                                    const uint32_t param_val,
                                    const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_param pp;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u param_id %u param_val %u",
        __func__, inst_id, block_id, param_id, param_val);

    pp.inst_id = inst_id;
    pp.block_id = block_id;
    pp.param_id = param_id;
    pp.param_val = param_val;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_PARAM, (unsigned long)&pp);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Get the value of parameter on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          param_id - Parameter ID
 *          block_id - Block ID*
 *          param_val - Parameter Value
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_get_parameter(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                    const uint32_t inst_id,
                                    const uint32_t param_id,
                                    const uint32_t block_id,
                                    uint32_t *param_val)
{
    int err = 0;
    struct iaxxx_plugin_param pp;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    pp.inst_id = inst_id;
    pp.block_id = block_id;
    pp.param_id = param_id;
    pp.param_val = 0;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_GET_PARAM, (unsigned long)&pp);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
        goto func_exit;
    }

    *param_val = pp.param_val;

    ALOGV("%s: Instance id %u, block id %u param_id %u param_val %u",
        __func__, inst_id, block_id, param_id, *param_val);

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set a parameter block on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          param_blk_id - Parameter block id
 *          block_id - Block ID
 *          param_buf - Pointer to the parameter block
 *          param_buf_sz - Parameter block size
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_parameter_blk(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t param_blk_id,
                                        const uint32_t block_id,
                                        const void *param_buf,
                                        const uint32_t param_buf_sz)
{
    int err = 0;
    struct iaxxx_plugin_param_blk ppb;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u, param_buf_sz %u, parm_blk_id %u",
        __func__, inst_id, block_id, param_buf_sz, param_blk_id);

    ppb.inst_id = inst_id;
    ppb.block_id = block_id;
    ppb.param_size = param_buf_sz;
    ppb.param_blk = (uintptr_t)param_buf;
    ppb.id = param_blk_id;
    ppb.file_name[0] = '\0';
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_PARAM_BLK, (unsigned long)&ppb);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set a parameter block on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          param_blk_id - Parameter block id
 *          block_id - Block ID
 *          file_name - Relative path to the Parameter File(Should be placed in
 *                     the firmware location)
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_parameter_blk_from_file(
                                        struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t param_blk_id,
                                        const uint32_t block_id,
                                        const char *file_name)
{
    int err = 0;
    struct iaxxx_plugin_param_blk ppb;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u, file_name %s, parm_blk_id %u",
        __func__, inst_id, block_id, file_name, param_blk_id);

    ppb.param_size = 0;
    ppb.param_blk = (uintptr_t) NULL;
    ppb.inst_id = inst_id;
    ppb.block_id = block_id;
    ppb.id = param_blk_id;
    strlcpy(ppb.file_name, file_name, NAME_MAX_SIZE);
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_PARAM_BLK, (unsigned long)&ppb);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}



/**
 * Set custom configuration for plugin
 *
 * Input  - odsp_hw_hdl         - Handle to odsp hw structure
 *          inst_id             - Instance ID
 *          block_id            - Block ID
 *          param_blk_id        - Parameter block id
 *          custom_config_id    - Id for what type of custom configuration
 *          filename            - Name of file with custom config data
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_custom_cfg(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                    const uint32_t inst_id,
                                    const uint32_t block_id,
                                    const uint32_t param_blk_id,
                                    const uint32_t custom_config_id,
                                    const char *filename)
{
    int err = 0;
    struct iaxxx_plugin_custom_cfg pcc;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u, param blk id %u"
        " custom-config id %u filename %s",
        __func__, inst_id, block_id, param_blk_id,custom_config_id, filename);

    strlcpy(pcc.file_name, filename, NAME_MAX_SIZE);
    pcc.inst_id = inst_id;
    pcc.block_id = block_id;
    pcc.param_blk_id = param_blk_id;
    pcc.custom_config_id = custom_config_id;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_CUSTOM_CFG, (unsigned long)&pcc);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Subscribe to an event
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          src_id      - System Id of event source
 *          event_id    - Event Id
 *          dst_id  - System Id of event destination
 *          dst_opaque  - Info sought by destination task when even occurs
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_evt_subscribe(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint16_t src_id,
                            const uint16_t event_id,
                            const uint16_t dst_id,
                            const uint32_t dst_opaque)
{
    int err = 0;
    struct iaxxx_evt_info ei;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: src id %u, event id %u, dst_id %u dst_opq %u",
        __func__, src_id, event_id, dst_id, dst_opaque);

    ei.src_id = src_id;
    ei.event_id = event_id;
    ei.dst_id = dst_id;
    ei.dst_opaque = dst_opaque;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_EVENT_SUBSCRIBE, (unsigned long)&ei);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Unsubscribe an event
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          src_id      - System Id of event source
 *          event_id    - Event Id
 *          dst_id  - System Id of event destination
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_evt_unsubscribe(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint16_t src_id,
                            const uint16_t event_id,
                            const uint16_t dst_id)
{
    int err = 0;
    struct iaxxx_evt_info ei;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: src id %u, event id %u, dst_id %u",
        __func__, src_id, event_id, dst_id);

    ei.src_id = src_id;
    ei.event_id = event_id;
    ei.dst_id = dst_id;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_EVENT_UNSUBSCRIBE, (unsigned long)&ei);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}


/**
 * Create a plugin for a statically loaded package
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          priority - Priority of the plugin
 *          pkg_id - Package ID
 *          plg_idx - Plugin Index*
 *          block_id - Block ID
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_create_static_package(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t priority,
                                        const uint32_t pkg_id,
                                        const uint32_t plg_idx,
                                        const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_plugin_info pi;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Plugin index %u, package id %u, block id %u, instance id %u"
         " priority %u", __func__, plg_idx, pkg_id, block_id, inst_id, priority);

    pi.plg_idx = plg_idx;
    pi.pkg_id = pkg_id;
    pi.block_id = block_id;
    pi.inst_id = inst_id;
    pi.priority = priority;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_CREATE_STATIC_PACKAGE, (unsigned long)&pi);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Get a parameter block from a plugin
 *
 * Input  - odsp_hw_hdl     - Handle to odsp hw structure
 *          inst_id         - Instance ID
 *          block_id        - Block ID
 *          param_blk_id    - Parameter block id
 *          param_buf       - Pointer to the parameter block
 *          param_buf_sz    - Parameter block size in words
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_get_parameter_blk(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t block_id,
                                        const uint32_t param_blk_id,
                                        uint32_t *param_buf,
                                        const uint32_t param_buf_sz)
{
    int err = 0;
    struct iaxxx_plugin_param_blk ppb;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u, param_buf_sz %u, id %u",
        __func__, inst_id, block_id, param_buf_sz, param_blk_id);

    ppb.inst_id = inst_id;
    ppb.block_id = block_id;
    ppb.id = param_blk_id;
    ppb.param_size = param_buf_sz;
    ppb.param_blk = (uintptr_t)param_buf;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_GET_PARAM_BLK, (unsigned long)&ppb);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set Event for the plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          eventEnableMask - event Mask
 *          block_id - Block ID
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_setevent(struct iaxxx_odsp_hw *odsp_hw_hdl,
                              const uint32_t inst_id,
                              const uint32_t eventEnableMask,
                              const uint32_t block_id)
{
    int err = 0;
    struct iaxxx_set_event se;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: instance id %u, eventEnableMask %x, block id %u",
        __func__, inst_id, eventEnableMask, block_id);

    se.block_id = block_id;
    se.event_enable_mask = eventEnableMask;
    se.inst_id  = inst_id;
    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_EVENT, (unsigned long)&se);
    if (err == -1) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
        return err;
    }

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/* Read Plugin Error Info
 *
 * Input  - odsp_hw_hdl     - Handle to odsp hw structure
 *          error_code      - Pointer to uint32 to return error code
 *          error_instance  - Pointer to uint8 to return plugin instance
 *                            where error occurred.
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_read_error(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                const uint32_t block_id,
                                uint32_t *error_code,
                                uint8_t *error_instance)
{
    int err = 0;
    struct iaxxx_plugin_error_info pei;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Block id %u", __func__, block_id);

    pei.block_id = block_id;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_READ_PLUGIN_ERROR, (unsigned long)&pei);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
        goto func_exit;
    }

    *error_code = pei.error_code;
    *error_instance = pei.error_instance;
    ALOGV("%s: Plugin error code:%x instance=%d",
        __func__, pei.error_code, pei.error_instance);

func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}

/**
 * Set a parameter block on a plugin and get ack to
 * ensure the data has been sent and retry if not.
 *
 * Input  - odsp_hw_hdl       - Handle to odsp hw structure
 *          inst_id           - Instance ID
 *          param_blk_id      - Parameter block id
 *          block_id          - Block ID
 *          set_param_buf     - Pointer to the parameter block
 *          set_param_buf_sz  - Parameter block size in bytes
 *          response_data_buf - Buffer for response data from plugin
 *          response_data_sz  - Size of Buffer in uint32 words for
 *                              response data from plugin
 *          max_no_retries    - Max number of retries in case of busy
 *                              response from plugin.
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_parameter_blk_with_ack(
                                        struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t param_blk_id,
                                        const uint32_t block_id,
                                        const void *set_param_buf,
                                        const uint32_t set_param_buf_sz,
                                        uint32_t* response_data_buf,
                                        const uint32_t response_data_sz,
                                        const uint32_t max_no_retries)
{

    int err = 0;
    struct iaxxx_plugin_set_param_blk_with_ack_info pspbwa;

    FUNCTION_ENTRY_LOG;

    if (odsp_hw_hdl == NULL) {
        ALOGE("%s: ERROR: Invalid handle to iaxxx_odsp_hw", __func__);
        err = -1;
        goto func_exit;
    }

    ALOGV("%s: Instance id %u, block id %u, param blk id %u max-retries:%u",
            __func__, inst_id, block_id, param_blk_id, max_no_retries);

    pspbwa.inst_id = inst_id;
    pspbwa.block_id = block_id;
    pspbwa.param_blk_id = param_blk_id;
    pspbwa.set_param_blk_buffer = (uintptr_t) set_param_buf;
    pspbwa.set_param_blk_size = set_param_buf_sz;
    pspbwa.response_buffer = (uintptr_t) response_data_buf;
    pspbwa.response_buf_size = response_data_sz;
    pspbwa.max_retries = max_no_retries;

    err = ioctl(fileno(odsp_hw_hdl->dev_node),
                ODSP_PLG_SET_PARAM_BLK_WITH_ACK, (unsigned long)&pspbwa);
    if (err < 0) {
        ALOGE("%s: ERROR: Failed with error %s", __func__, strerror(errno));
    }


func_exit:
    FUNCTION_EXIT_LOG;
    return err;
}
