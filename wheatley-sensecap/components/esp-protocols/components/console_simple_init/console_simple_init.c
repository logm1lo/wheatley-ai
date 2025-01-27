/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include "esp_console.h"
#include "esp_log.h"
#include "console_simple_init.h"


static esp_console_repl_t *repl = NULL;
static const char *TAG = "console_simple_init";

/**
 * @brief Initializes the esp console
 * @return ESP_OK on success
 */
esp_err_t console_cmd_init(void)
{
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    // install console REPL environment
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    return esp_console_new_repl_uart(&hw_config, &repl_config, &repl);

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    return esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl);

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    return esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);

#else
#error Unsupported console type
#endif

}

/**
 * @brief Register a user supplied command
 *
 * @param[in] cmd string that is the user defined command
 * @param[in] do_user_cmd Function pointer for a user-defined command callback function
 *
 * @return ESP_OK on success
 */
esp_err_t console_cmd_user_register(char *cmd, esp_console_cmd_func_t do_user_cmd)
{
    esp_err_t ret;

    const esp_console_cmd_t user_cmd = {
        .command = cmd,
        .help = "User defined command",
        .hint = NULL,
        .func = do_user_cmd,
    };

    ret = esp_console_cmd_register(&user_cmd);
    if (ret) {
        ESP_LOGE(TAG, "Unable to register user cmd");
    }

    return ret;
}


/**
 * @brief Register all the console commands in .console_cmd_desc section
 *
 * @return ESP_OK on success
 */
esp_err_t console_cmd_all_register(void)
{
    esp_err_t ret = ESP_FAIL;
    extern const console_cmd_plugin_desc_t _console_cmd_array_start;
    extern const console_cmd_plugin_desc_t _console_cmd_array_end;

    ESP_LOGI(TAG, "List of Console commands:\n");
    for (const console_cmd_plugin_desc_t *it = &_console_cmd_array_start; it != &_console_cmd_array_end; ++it) {
        ESP_LOGI(TAG, "- Command '%s', function plugin_regd_fn=%p\n", it->name, it->plugin_regd_fn);
        if (it->plugin_regd_fn != NULL) {
            ret = (it->plugin_regd_fn)();
            if (ret != ESP_OK) {
                return ret;
            }
        }
    }

    return ESP_OK;
}


/**
 * @brief Starts the esp console
 * @return ESP_OK on success
 */
esp_err_t console_cmd_start(void)
{
    // start console REPL
    return esp_console_start_repl(repl);
}
