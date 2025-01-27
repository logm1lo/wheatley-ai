#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "cmd";

#define PROMPT_STR "Wheatly"
#define STORAGE_NAMESPACE "Wheatly"
#define LIVEKIT_URL_STORAGE "lk_url_key"
#define LIVEKIT_TOKEN_STORAGE "lk_token_key"
#define LIVEKIT_URL_BUFFER_LEN 64
#define LIVEKIT_TOKEN_BUFFER_LEN 512

char livekit_url_buf[LIVEKIT_URL_BUFFER_LEN] = {
    0,
};

char livekit_token_buf[LIVEKIT_TOKEN_BUFFER_LEN] = {
    0,
};

static int
max(int a, int b)
{
    return (a > b) ? a : b;
}

static esp_err_t storage_write(char *p_key, void *p_data, size_t len)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(my_handle, p_key, p_data, len);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }
    nvs_close(my_handle);
    return ESP_OK;
}

static esp_err_t storage_read(char *p_key, void *p_data, size_t *p_len)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_get_blob(my_handle, p_key, p_data, p_len);
    if (err != ESP_OK)
    {
        nvs_close(my_handle);
        return err;
    }
    nvs_close(my_handle);
    return ESP_OK;
}

/** wifi set command **/
static struct
{
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_end *end;
} wifi_cfg_args;

static int wifi_cfg_set(int argc, char **argv)
{
    bool have_password = false;
    char ssid[32] = {0};
    char password[64] = {0};
    wifi_config_t wifi_config = {0};

    int nerrors = arg_parse(argc, argv, (void **)&wifi_cfg_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, wifi_cfg_args.end, argv[0]);
        return 1;
    }

    if (wifi_cfg_args.ssid->count)
    {
        int len = strlen(wifi_cfg_args.ssid->sval[0]);
        if (len > (sizeof(ssid) - 1))
        {
            ESP_LOGE(TAG, "out of 31 bytes :%s", wifi_cfg_args.ssid->sval[0]);
            return -1;
        }
        strncpy(ssid, wifi_cfg_args.ssid->sval[0], 31);
    }
    else
    {
        ESP_LOGE(TAG, "no ssid");
        return -1;
    }

    if (wifi_cfg_args.password->count)
    {
        int len = strlen(wifi_cfg_args.password->sval[0]);
        if (len > (sizeof(password) - 1))
        {
            ESP_LOGE(TAG, "out of 64 bytes :%s", wifi_cfg_args.password->sval[0]);
            return -1;
        }
        have_password = true;
        strncpy(password, wifi_cfg_args.password->sval[0], 63);
    }

    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));

    if (have_password)
    {
        strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    }
    else
    {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    esp_wifi_stop();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "config wifi, SSID: %s, password: %s", wifi_config.sta.ssid, wifi_config.sta.password);
    return 0;
}

// wifi_cfg -s ssid -p password
static void register_cmd_wifi_sta(void)
{
    wifi_cfg_args.ssid = arg_str0("s", NULL, "<ssid>", "SSID of AP");
    wifi_cfg_args.password = arg_str0("p", NULL, "<password>", "password of AP");
    wifi_cfg_args.end = arg_end(2);

    const esp_console_cmd_t cmd = {.command = "wifi_sta", .help = "WiFi is station mode, join specified soft-AP", .hint = NULL, .func = &wifi_cfg_set, .argtable = &wifi_cfg_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/************* reboot **************/
static int do_reboot(int argc, char **argv)
{
    esp_restart();
    return 0;
}

static void register_cmd_reboot(void)
{
    const esp_console_cmd_t cmd = {.command = "reboot", .help = "reboot the device", .hint = NULL, .func = &do_reboot, .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/** openai api key set command **/
static struct
{
    struct arg_str *url;
    struct arg_str *token;
    struct arg_end *end;
} livekit_cfg_args;

static int livekit_cfg_set(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&livekit_cfg_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, livekit_cfg_args.end, argv[0]);
        return 1;
    }

    size_t len = 0;

    // Local url buffer
    char url_buf[LIVEKIT_URL_BUFFER_LEN] = {
        0,
    };

    // Check length of url
    if (livekit_cfg_args.url->count)
    {
        int len = strlen(livekit_cfg_args.url->sval[0]);
        if (len > sizeof(url_buf))
        {
            ESP_LOGE(TAG, "out of %d bytes :%s", LIVEKIT_URL_BUFFER_LEN, livekit_cfg_args.url->sval[0]);
            return -1;
        }
        strncpy(url_buf, livekit_cfg_args.url->sval[0], LIVEKIT_URL_BUFFER_LEN);

        ESP_LOGI(TAG, "Write Livekit URL:%s", url_buf);
        storage_write(LIVEKIT_URL_STORAGE, (void *)url_buf, sizeof(url_buf));
    }

    // Validate url is saved in storage
    len = sizeof(url_buf);
    memset(url_buf, 0, sizeof(url_buf));
    esp_err_t ret = storage_read(LIVEKIT_URL_STORAGE, (void *)url_buf, &len);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Read Livekit URL:%s", url_buf);
    }
    else
    {
        ESP_LOGE(TAG, "LiveKit URL read fail!");
    }

    // Local token buffer
    char token_buf[LIVEKIT_TOKEN_BUFFER_LEN] = {
        0,
    };

    // Check length of token
    if (livekit_cfg_args.token->count)
    {
        int len = strlen(livekit_cfg_args.token->sval[0]);
        if (len > sizeof(token_buf))
        {
            ESP_LOGE(TAG, "out of %d bytes :%s", LIVEKIT_TOKEN_BUFFER_LEN, livekit_cfg_args.token->sval[0]);
            return -1;
        }
        strncpy(token_buf, livekit_cfg_args.token->sval[0], LIVEKIT_TOKEN_BUFFER_LEN);

        ESP_LOGI(TAG, "Write Livekit Token:%s", token_buf);
        storage_write(LIVEKIT_TOKEN_STORAGE, (void *)token_buf, sizeof(token_buf));
    }

    // Validate url is saved in storage
    len = sizeof(token_buf);
    memset(token_buf, 0, sizeof(token_buf));
    ret = storage_read(LIVEKIT_TOKEN_STORAGE, (void *)token_buf, &len);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Read Livekit Token:%s", token_buf);
    }
    else
    {
        ESP_LOGE(TAG, "LiveKit Token read fail!");
    }

    return 0;
}

// livekit_cfg -u <websocket url> -t <token>
static void register_livekit_cfg(void)
{
    livekit_cfg_args.url = arg_str0("u", NULL, "<url", "set url 64 bytes");
    livekit_cfg_args.token = arg_str0("t", NULL, "<token>", "set token 512 bytes");
    livekit_cfg_args.end = arg_end(1);

    const esp_console_cmd_t cmd = {.command = "livekit_cfg", .help = "To set livekit url and token", .hint = NULL, .func = &livekit_cfg_set, .argtable = &livekit_cfg_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

/************* cmd register **************/
int cmd_init(void)
{
#if CONFIG_ENABLE_FACTORY_FW_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = 1024;

    register_cmd_wifi_sta();
    register_livekit_cfg();
    register_cmd_reboot();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif

    // Since we have SD card access in console cmd, it might trigger the SPI core-conflict issue
    // we can't control the core on which the console runs, so
    // TODO: narrow the SD card access code into another task which runs on Core 1.
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // Check and save livekit url and token to .data
    size_t len = sizeof(livekit_url_buf);
    memset(livekit_url_buf, 0, sizeof(livekit_url_buf));
    esp_err_t url_ret = storage_read(LIVEKIT_URL_STORAGE, (void *)livekit_url_buf, &len);

    len = sizeof(livekit_token_buf);
    memset(livekit_token_buf, 0, sizeof(livekit_token_buf));
    esp_err_t token_ret = storage_read(LIVEKIT_TOKEN_STORAGE, (void *)livekit_token_buf, &len);

    if (url_ret == ESP_OK && token_ret == ESP_OK)
    {
        ESP_LOGI(TAG, "read livekit url and token");
    }
    else
    {
        ESP_LOGE(TAG, "Please set livekit url and token via cmdline, then reboot!");
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    return 0;
}
