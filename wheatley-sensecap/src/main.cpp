#include "main.h"

#include <esp_event.h>
#include <esp_log.h>
#include <peer.h>

#include "nvs_flash.h"

extern "C" char livekit_url_buf[64];

extern "C" char livekit_token_buf[512];

// Board init functions: display init, io init, codec init, rgb init
extern "C" void board_init(void)
{
  bsp_io_expander_init();
  lv_disp_t *lvgl_disp = bsp_lvgl_init();
  assert(lvgl_disp != NULL);
  bsp_rgb_init();
  bsp_codec_init();
  bsp_codec_volume_set(100, NULL);
}

// Shutdown the system after long press
extern "C" void long_press_event_cb(void)
{
  ESP_LOGI("", "long_press_event_cb");
  bsp_system_shutdown();
  bsp_lcd_brightness_set(0);
  bsp_codec_mute_set(true);
  vTaskDelay(pdMS_TO_TICKS(3000));
  esp_restart();
}

extern "C" void app_main(void)
{
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ESP_ERROR_CHECK(esp_event_loop_create_default());

  board_init();

  bsp_set_btn_long_press_cb(long_press_event_cb);

  // UI init [Done]
  ui_init();

  // Core Interaction Functions [Done]
  wheatly_wifi_init();
  cmd_init();
  peer_init();

  // Audio Functions
  wheatly_init_audio_capture();
  wheatly_init_audio_decoder();

  // UI Functions
  wheatly_wifi();

  // Start the websocket connection
  lk_websocket(livekit_url_buf, livekit_token_buf);
}
