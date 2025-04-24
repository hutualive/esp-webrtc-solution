/* OpenAI realtime communication test

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_webrtc.h"
#include "media_lib_adapter.h"
#include "media_lib_os.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "common.h"
#include "esp_capture_defaults.h"
#include "lcd_gui.h"

static int start_chat(int argc, char **argv)
{
    start_webrtc();
    return 0;
}

#define RUN_ASYNC(name, body)           \
    void run_async##name(void *arg)     \
    {                                   \
        body;                           \
        media_lib_thread_destroy(NULL); \
    }                                   \
    media_lib_thread_create_from_scheduler(NULL, #name, run_async##name, NULL);

static int stop_chat(int argc, char **argv)
{
    RUN_ASYNC(stop, { stop_webrtc(); });
    return 0;
}

static int assert_cli(int argc, char **argv)
{
    *(int *)0 = 0;
    return 0;
}

static int sys_cli(int argc, char **argv)
{
    sys_state_show();
    return 0;
}

static int wifi_cli(int argc, char **argv)
{
    if (argc < 1) {
        return -1;
    }
    char *ssid = argv[1];
    char *password = argc > 2 ? argv[2] : NULL;
    return network_connect_wifi(ssid, password);
}

static int measure_cli(int argc, char **argv)
{
    void measure_enable(bool enable);
    void show_measure(void);
    measure_enable(true);
    media_lib_thread_sleep(1500);
    measure_enable(false);
    return 0;
}

static int dump_cli(int argc, char **argv)
{
    bool enable = (argc > 1);
    printf("Enable AEC dump %d\n", enable);
    esp_capture_enable_aec_src_dump(enable);
    return 0;
}

static int rec2play_cli(int argc, char **argv)
{
    test_capture_to_player();
    return 0;
}

static int text_cli(int argc, char **argv)
{
    if (argc > 1) {
        openai_send_text(argv[1]);
    }
    return 0;
}

#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include "esp_timer.h"
#include "esp_cpu.h"

static int tone_cli(int argc, char **argv)
{
    float f = argc > 1 ? atof(argv[1]) : 1000.0f;
    int ms = argc > 2 ? atoi(argv[2]) : 2000;
    ESP_LOGI("TONE", "tone_cli: f=%.1f Hz, d=%d ms", f, ms);
    const int N = 160;
    int16_t buf[N];
    for (int i = 0; i < N; i++) {
        buf[i] = (int16_t)(sinf(2 * M_PI * f * i / 16000) * 3000);
    }
    esp_webrtc_media_provider_t provider;
    media_sys_get_provider(&provider);
    av_render_handle_t player = provider.player;
    av_render_audio_info_t tone_info = { .codec = AV_RENDER_AUDIO_CODEC_PCM, .sample_rate = 16000, .channel = 1, .bits_per_sample = 16 };
    av_render_add_audio_stream(player, &tone_info);
    uint32_t start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000) < start + ms) {
        av_render_audio_data_t d = { .data = (uint8_t*)buf, .size = sizeof(buf), .pts = 0 };
        av_render_add_audio_data(player, &d);
        media_lib_thread_sleep(10);
    }
    av_render_reset(player);
    return 0;
}

static int rec2play_raw_cli(int argc, char **argv)
{
    test_capture_to_player_raw();
    return 0;
}

static int init_console()
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "esp>";
    repl_config.task_stack_size = 10 * 1024;
    repl_config.task_priority = 22;
    repl_config.max_cmdline_length = 1024;
    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif
    esp_console_cmd_t cmds[] = {
        {
            .command = "start",
            .help = "Start OpenAI realtime communication\r\n",
            .func = start_chat,
        },
        {
            .command = "stop",
            .help = "Stop chat\n",
            .func = stop_chat,
        },
        {
            .command = "i",
            .help = "Show system status\r\n",
            .func = sys_cli,
        },
        {
            .command = "assert",
            .help = "Assert system\r\n",
            .func = assert_cli,
        },
        {
            .command = "wifi",
            .help = "wifi ssid psw\r\n",
            .func = wifi_cli,
        },
        {
            .command = "m",
            .help = "measure system loading\r\n",
            .func = measure_cli,
        },
        {
            .command = "dump",
            .help = "Dump AEC data\r\n",
            .func = dump_cli,
        },
        {
            .command = "text",
            .help = "Send text message\r\n",
            .func = text_cli,
        },
        {
            .command = "rec2play",
            .help = "Play recorded voice\r\n",
            .func = rec2play_cli,
        },
        {
            .command = "tone",
            .help = "Play test tone: tone <freq> <ms>\n",
            .func = tone_cli,
        },
        {
            .command = "rec2play_raw",
            .help = "Play recorded voice (raw mic, no AEC)\n",
            .func = rec2play_raw_cli,
        },
    };
    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    return 0;
}

static void thread_scheduler(const char *thread_name, media_lib_thread_cfg_t *thread_cfg)
{
    if (strcmp(thread_name, "pc_task") == 0) {
        thread_cfg->stack_size = 25 * 1024;
        thread_cfg->priority = 18;
        thread_cfg->core_id = 1;
    }
    if (strcmp(thread_name, "start") == 0) {
        thread_cfg->stack_size = 6 * 1024;
    }
    if (strcmp(thread_name, "pc_send") == 0) {
        thread_cfg->stack_size = 4 * 1024;
        thread_cfg->priority = 15;
        thread_cfg->core_id = 1;
    }
    if (strcmp(thread_name, "Adec") == 0) {
        thread_cfg->stack_size = 40 * 1024;
        thread_cfg->priority = 10;
        thread_cfg->core_id = 1;
    }
    if (strcmp(thread_name, "venc") == 0) {
#if CONFIG_IDF_TARGET_ESP32S3
        thread_cfg->stack_size = 20 * 1024;
#endif
        thread_cfg->priority = 10;
    }
#ifdef WEBRTC_SUPPORT_OPUS
    if (strcmp(thread_name, "aenc") == 0) {
        thread_cfg->stack_size = 40 * 1024;
        thread_cfg->priority = 10;
    }
    if (strcmp(thread_name, "SrcRead") == 0) {
        thread_cfg->stack_size = 40 * 1024;
        thread_cfg->priority = 16;
        thread_cfg->core_id = 0;
    }
    if (strcmp(thread_name, "buffer_in") == 0) {
        thread_cfg->stack_size = 6 * 1024;
        thread_cfg->priority = 10;
        thread_cfg->core_id = 0;
    }
#endif
}

static int network_event_handler(bool connected)
{
    static bool splash_shown = false;
    if (connected) {
        char ip[32] = {0};
        esp_netif_ip_info_t ip_info;
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(ip, sizeof(ip), IPSTR, IP2STR(&ip_info.ip));
        } else {
            strncpy(ip, "unknown", sizeof(ip)-1);
        }
        lcd_gui_show_status("WiFi Connected: %s", ip);
        if (!splash_shown) {
            lcd_gui_splash(LCD_GUI_SPLASH_CONNECTED);
            splash_shown = true;
        }
    } else {
        lcd_gui_show_status("WiFi Disconnected");
    }
    return 0;
}

void on_new_chat_message(const char *msg)
{
    lcd_gui_show_chat(msg);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("LCD_GUI", ESP_LOG_ERROR);
    media_lib_add_default_adapter();
    media_lib_thread_set_schedule_cb(thread_scheduler);
    init_board();
    lcd_gui_init();
    lcd_gui_splash(LCD_GUI_SPLASH_POWERON);
    media_sys_buildup();
    init_console();
    network_init(WIFI_SSID, WIFI_PASSWORD, network_event_handler);
    while (1) {
        media_lib_thread_sleep(2000);
        query_webrtc();
    }
}
