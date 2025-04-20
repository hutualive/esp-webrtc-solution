// lcd_gui.h - Simple GUI and splash screen for OpenAI WebRTC Demo
#pragma once
#include <stdint.h>
#include <stdarg.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#ifdef __cplusplus
extern "C" {
#endif

// LCD GUI command types
typedef enum {
    LCD_GUI_CMD_CLEAR,
    LCD_GUI_CMD_SPLASH,
    LCD_GUI_CMD_STATUS,
    LCD_GUI_CMD_CHAT,
} lcd_gui_cmd_t;

// Splash types
#define LCD_GUI_SPLASH_POWERON 1
#define LCD_GUI_SPLASH_CONNECTED 2

// Max message length for status/chat
#define LCD_GUI_MSG_MAXLEN 128

typedef struct {
    lcd_gui_cmd_t cmd;
    union {
        struct { uint16_t color; } clear;
        struct { int which; } splash;
        struct { char msg[LCD_GUI_MSG_MAXLEN]; } status;
        struct { char msg[LCD_GUI_MSG_MAXLEN]; } chat;
    } data;
    // For status, you may want to allow printf-style formatting
    va_list args;
} lcd_gui_msg_t;

void lcd_gui_init(void);
void lcd_gui_splash(int which);
void lcd_gui_show_status(const char *fmt, ...);
void lcd_gui_show_chat(const char *msg);
void lcd_gui_clear(uint16_t color);

#ifdef __cplusplus
}
#endif
