// lcd_gui.c - Simple GUI and splash screen for OpenAI WebRTC Demo
#include "lcd_gui.h"
#include "codec_board.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <esp_log.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_st7789.h>
#include <esp_system.h>
#include <sys/param.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdlib.h>

#define TAG "LCD_GUI"

// Prototype for board_get_lcd_handle
extern void *board_get_lcd_handle(void);

// Forward declaration to avoid implicit declaration error
void lcd_gui_draw_bitmap_centered(const uint16_t *bitmap, int bmp_w, int bmp_h);

// Colors (RGB565, but use st7789_fix_color before sending)
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_BLUE    0x001F
#define COLOR_GREEN   0x07E0
#define COLOR_RED     0xF800
#define COLOR_YELLOW  0xFFE0
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F

#define LCD_GUI_TASK_STACK_SIZE 12288
#define LCD_GUI_TASK_PRIORITY   5
#define LCD_GUI_QUEUE_SIZE      8
#define LCD_GUI_MSG_MAXLEN      128

// LCD handle and dimensions must be declared before use in bitmap draw function
static esp_lcd_panel_handle_t lcd_handle = NULL;
static int lcd_width = 240;
static int lcd_height = 240;

// --- Human-Oriented Splash Bitmaps: what you see is what you want ---
// RED square (RGB565)
static const uint16_t splash_bitmap1[32*32] = {
    [0 ... (32*32-1)] = 0xF800 // RED
};
// BLUE square (RGB565)
static const uint16_t splash_bitmap2[32*32] = {
    [0 ... (32*32-1)] = 0x001F // BLUE
};

// Human-oriented color fix for ST7789: swap bytes and invert bits for panel quirk
static uint16_t st7789_fix_color(uint16_t v) {
    uint16_t swapped = (uint16_t)((v << 8) | (v >> 8));
    return ~swapped;
}

static QueueHandle_t lcd_gui_queue = NULL;
static TaskHandle_t lcd_gui_task_handle = NULL;

// Function prototype for internal clear function
static void lcd_gui_draw_clear(uint16_t color);

// Internal: LCD/UI task
static void lcd_gui_task(void *param) {
    lcd_gui_msg_t msg;
    int heartbeat = 0;
    ESP_LOGI(TAG, "lcd_gui_task started");
    while (1) {
        ESP_LOGD(TAG, "Before xQueueReceive");
        // Use timeout instead of portMAX_DELAY
        if (xQueueReceive(lcd_gui_queue, &msg, pdMS_TO_TICKS(2000)) == pdTRUE) {
            ESP_LOGI(TAG, "Received LCD command: %d", msg.cmd);
            switch (msg.cmd) {
                case LCD_GUI_CMD_CLEAR:
                    ESP_LOGI(TAG, "lcd_gui_task: CLEAR");
                    lcd_gui_draw_clear(msg.data.clear.color);
                    ESP_LOGI(TAG, "lcd_gui_task: CLEAR done");
                    break;
                case LCD_GUI_CMD_SPLASH:
                    ESP_LOGI(TAG, "lcd_gui_task: SPLASH type=%d", msg.data.splash.which);
                    lcd_gui_draw_clear(COLOR_WHITE); // Always clear to white
                    if (msg.data.splash.which == LCD_GUI_SPLASH_POWERON) {
                        ESP_LOGI(TAG, "Drawing splash_bitmap1 (POWERON)");
                        lcd_gui_draw_bitmap_centered(splash_bitmap1, 32, 32);
                    } else if (msg.data.splash.which == LCD_GUI_SPLASH_CONNECTED) {
                        ESP_LOGI(TAG, "Drawing splash_bitmap2 (CONNECTED)");
                        lcd_gui_draw_bitmap_centered(splash_bitmap2, 32, 32);
                    } else {
                        ESP_LOGI(TAG, "Drawing splash_bitmap1 (DEFAULT)");
                        lcd_gui_draw_bitmap_centered(splash_bitmap1, 32, 32);
                    }
                    ESP_LOGI(TAG, "Splash displayed, delaying for visibility");
                    vTaskDelay(pdMS_TO_TICKS(1200)); // Show splash for 1.2s
                    break;
                case LCD_GUI_CMD_STATUS:
                    ESP_LOGI(TAG, "lcd_gui_task: STATUS");
                    lcd_gui_draw_clear(COLOR_WHITE); // Use white background for status
                    ESP_LOGI(TAG, "lcd_gui_task: STATUS done");
                    ESP_LOGI(TAG, "LCD STATUS: %s", msg.data.status.msg);
                    break;
                case LCD_GUI_CMD_CHAT:
                    ESP_LOGI(TAG, "lcd_gui_task: CHAT");
                    lcd_gui_draw_clear(COLOR_WHITE);
                    ESP_LOGI(TAG, "lcd_gui_task: CHAT done");
                    ESP_LOGI(TAG, "LCD CHAT: %s", msg.data.chat.msg);
                    break;
            }
            ESP_LOGD(TAG, "After switch, before queue status");
            UBaseType_t q_space = uxQueueSpacesAvailable(lcd_gui_queue);
            ESP_LOGI(TAG, "lcd_gui_queue spaces available: %lu", (unsigned long)q_space);
        } else {
            ESP_LOGW(TAG, "xQueueReceive timeout (no LCD command in 2s)");
        }
        if (++heartbeat % 10 == 0) {
            ESP_LOGI(TAG, "lcd_gui_task heartbeat");
        }
        // Optional: log stack high water mark
        UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGI(TAG, "lcd_gui_task stack high water mark: %lu", (unsigned long)stack_hwm);
    }
}

void lcd_gui_init(void) {
    lcd_handle = (esp_lcd_panel_handle_t)board_get_lcd_handle();
    lcd_cfg_t cfg;
    if (get_lcd_cfg(&cfg) == 0) {
        lcd_width = cfg.width;
        lcd_height = cfg.height;
    }
    // Hardware inversion disabled; software conversion now handles bitwise invert
    // Create queue and task
    if (!lcd_gui_queue) {
        lcd_gui_queue = xQueueCreate(LCD_GUI_QUEUE_SIZE, sizeof(lcd_gui_msg_t));
    }
    if (!lcd_gui_task_handle) {
        xTaskCreate(lcd_gui_task, "lcd_gui_task", LCD_GUI_TASK_STACK_SIZE, NULL, LCD_GUI_TASK_PRIORITY, &lcd_gui_task_handle);
    }
}

// These are the PUBLIC API functions now: they post to the queue
void lcd_gui_clear(uint16_t color) {
    lcd_gui_msg_t msg = { .cmd = LCD_GUI_CMD_CLEAR };
    msg.data.clear.color = color;
    xQueueSend(lcd_gui_queue, &msg, portMAX_DELAY);
}

void lcd_gui_splash(int which) {
    lcd_gui_msg_t msg = { .cmd = LCD_GUI_CMD_SPLASH };
    msg.data.splash.which = which;
    xQueueSend(lcd_gui_queue, &msg, portMAX_DELAY);
}

void lcd_gui_show_status(const char *fmt, ...) {
    lcd_gui_msg_t msg = { .cmd = LCD_GUI_CMD_STATUS };
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg.data.status.msg, LCD_GUI_MSG_MAXLEN, fmt, args);
    va_end(args);
    xQueueSend(lcd_gui_queue, &msg, portMAX_DELAY);
}

void lcd_gui_show_chat(const char *msg_in) {
    lcd_gui_msg_t msg = { .cmd = LCD_GUI_CMD_CHAT };
    strncpy(msg.data.chat.msg, msg_in, LCD_GUI_MSG_MAXLEN-1);
    msg.data.chat.msg[LCD_GUI_MSG_MAXLEN-1] = '\0';
    xQueueSend(lcd_gui_queue, &msg, portMAX_DELAY);
}

// Internal drawing function
// Draw the screen in smaller row blocks to avoid SPI DMA/queue overflow
static void lcd_gui_draw_clear(uint16_t color) {
    if (!lcd_handle) return;
    ESP_LOGD(TAG, "lcd_gui_draw_clear: start");
    const int ROWS_PER_BLOCK = 20; // Tune for your hardware/SPI driver
    uint16_t c = st7789_fix_color(color);
    size_t block_buf_size = lcd_width * ROWS_PER_BLOCK * sizeof(uint16_t);
    uint16_t *buf = malloc(block_buf_size);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate LCD clear buffer");
        return;
    }
    for (int y = 0; y < lcd_height; y += ROWS_PER_BLOCK) {
        int rows = (y + ROWS_PER_BLOCK > lcd_height) ? (lcd_height - y) : ROWS_PER_BLOCK;
        for (int i = 0; i < lcd_width * rows; ++i) {
            buf[i] = c;
        }
        ESP_LOGD(TAG, "lcd_gui_draw_clear: drawing y=%d rows=%d", y, rows);
        esp_err_t err = esp_lcd_panel_draw_bitmap(lcd_handle, 0, y, lcd_width, y + rows, buf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "panel_draw_bitmap failed at y=%d: %s", y, esp_err_to_name(err));
            break;
        }
    }
    ESP_LOGD(TAG, "lcd_gui_draw_clear: done");
    free(buf);
}

void lcd_gui_draw_bitmap_centered(const uint16_t *bitmap, int bmp_w, int bmp_h) {
    if (!lcd_handle) return;
    int x0 = (lcd_width - bmp_w) / 2;
    int y0 = (lcd_height - bmp_h) / 2;
    // Create fixed buffer with proper color endianness
    size_t buf_size = bmp_w * bmp_h * sizeof(uint16_t);
    uint16_t *buf = malloc(buf_size);
    if (!buf) return;
    for (int i = 0; i < bmp_w * bmp_h; ++i) {
        buf[i] = st7789_fix_color(bitmap[i]);
    }
    esp_lcd_panel_draw_bitmap(lcd_handle, x0, y0, x0 + bmp_w, y0 + bmp_h, buf);
    free(buf);
}
