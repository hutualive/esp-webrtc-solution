#include <stdio.h>
#include "esp_log.h"
#include "codec_init.h"
#include "codec_board.h"
#include "esp_codec_dev.h"
#include "sdkconfig.h"
#include "settings.h"

static const char *TAG = "Board";

void init_board(void)
{
    ESP_LOGI(TAG, "Init board.");
    set_codec_board_type(TEST_BOARD_NAME);
    // Notes when use playback and record at same time, must set reuse_dev = false
    codec_init_cfg_t cfg = {
#if CONFIG_IDF_TARGET_ESP32S3
        .in_mode = CODEC_I2S_MODE_TDM,
        .in_use_tdm = true,
#endif
        .reuse_dev = false
    };
    init_codec(&cfg);

#ifndef DISABLE_LCD
    // Only initialize LCD if not disabled
    ESP_LOGI(TAG, "Attempting to initialize LCD");

    // Instead of directly calling board_lcd_init, check if LCD is available
    lcd_cfg_t lcd_cfg;
    if (get_lcd_cfg(&lcd_cfg) == 0) {
        ESP_LOGI(TAG, "LCD configuration found, attempting to initialize");
        int lcd_ret = board_lcd_init();
        if (lcd_ret != 0) {
            ESP_LOGW(TAG, "LCD initialization failed with error %d, continuing without LCD", lcd_ret);
        }else {
            ESP_LOGI(TAG, "LCD initialized successfully");
	}
    } else {
        ESP_LOGI(TAG, "No LCD configuration found, skipping LCD initialization");
    }
#else
    ESP_LOGI(TAG, "LCD initialization disabled by compile-time flag");
#endif
}
