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

#ifndef _IAXXX_ODSP_HW_H_
#define _IAXXX_ODSP_HW_H_

#if __cplusplus
extern "C"
{
#endif

struct iaxxx_odsp_hw;

struct iaxxx_config_file {
    const char *filename;
};

struct iaxxx_config_value {
    uint64_t config_val;
    uint32_t config_val_sz;
};

union iaxxx_config_data {
    struct iaxxx_config_file fdata;
    struct iaxxx_config_value vdata;
};

enum iaxxx_config_type {
    CONFIG_FILE,
    CONFIG_VALUE
};

struct iaxxx_create_config_data {
    enum iaxxx_config_type type;
    union iaxxx_config_data data;
};

/**
 * Initialize the ODSP HAL
 *
 * Input  - NA
 * Output - Handle to iaxxx_odsp_hw on success, NULL on failure
 */
struct iaxxx_odsp_hw* iaxxx_odsp_init();

/**
 * De-Initialize the ODSP HAL
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_deinit(struct iaxxx_odsp_hw *odsp_hw_hdl);

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
                            const char *pkg_name,
                            const uint32_t pkg_id,
                            const uint32_t proc_id);

/**
 * Unload a package
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          pkg_id - Package ID
 *          proc_id - Process ID
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_package_unload(struct iaxxx_odsp_hw *odsp_hw_hdl,
			                const uint32_t pkg_id,
			                const uint32_t proc_id);

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
                             const uint32_t block_id);

/**
 * Set the creation configuration on a plugin
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          inst_id - Instance ID
 *          block_id - Block ID
 *          cdata - Creation configuration data
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_creation_config(
                                        struct iaxxx_odsp_hw *odsp_hw_hdl,
                                        const uint32_t inst_id,
                                        const uint32_t block_id,
                                        struct iaxxx_create_config_data cdata);

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
                              const uint32_t block_id);

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
                            const uint32_t block_id);

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
                              const uint32_t block_id);

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
                            const uint32_t block_id);

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
                                    const uint32_t block_id);

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
                                    uint32_t *param_val);

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
                                        const uint32_t param_buf_sz);

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
                                        const char *file_name);

/**
 * Set custom configuration for plugin
 *
 * Input  - odsp_hw_hdl 	    - Handle to odsp hw structure
 *          inst_id  		    - Instance ID
 *          block_id 		    - Block ID
 *          param_blk_id  	    - Parameter block id
 *          custom_config_id    - Id for what type of custom configuration
 *          filename		    - Name of file with custom config data
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_plugin_set_custom_cfg(struct iaxxx_odsp_hw *odsp_hw_hdl,
                                    const uint32_t inst_id,
                                    const uint32_t block_id,
                                    const uint32_t param_blk_id,
                                    const uint32_t custom_config_id,
                                    const char *filename);

/**
 * Subscribe to an event
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          src_id  	- System Id of event source
 *          event_id 	- Event Id
 *          dst_id	- System Id of event destination
 *          dst_opaque	- Info sought by destination task when even occurs
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_evt_subscribe(struct iaxxx_odsp_hw *odsp_hw_hdl,
                            const uint16_t src_id,
                            const uint16_t event_id,
                            const uint16_t dst_id,
                            const uint32_t dst_opaque);

/**
 * Unsubscribe an event
 *
 * Input  - odsp_hw_hdl - Handle to odsp hw structure
 *          src_id  	- System Id of event source
 *          event_id 	- Event Id
 *          dst_id	- System Id of event destination
 *
 * Output - 0 on success, on failure < 0
 */
int iaxxx_odsp_evt_unsubscribe(struct iaxxx_odsp_hw *odsp_hw_hdl,
                              const uint16_t src_id,
                              const uint16_t event_id,
                              const uint16_t dst_id);

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
                                            const uint32_t block_id);

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
                                        const uint32_t param_buf_sz);

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
                              const uint32_t block_id);

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
                                uint8_t *error_instance);


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
                                        const uint32_t max_no_retries);


#if __cplusplus
} // extern "C"
#endif

#endif // #ifndef _IAXXX_ODSP_HW_H_
