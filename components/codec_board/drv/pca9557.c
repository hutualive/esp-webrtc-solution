#include <string.h>
#include "esp_log.h"
#include "pca9557.h"
#include "codec_init.h"
#include "audio_codec_ctrl_if.h"
#include "esp_codec_dev_defaults.h"

static char *TAG = "PCA9557";

#define SET_BITS(_m, _s, _v)  ((_v) ? (_m)|((_s)) : (_m)&~((_s)))
#define GET_BITS(_m, _s)      (((_m) & (_s)) ? true : false)

// PCA9557 Register definitions
#define PCA9557_INPUT_PORT      0x00
#define PCA9557_OUTPUT_PORT     0x01
#define PCA9557_POLARITY_PORT   0x02
#define PCA9557_CONFIG_PORT     0x03

typedef struct {
    uint8_t addr;
    char *name;
} pca9557_dev_t;

static pca9557_dev_t dev_list[] = {
    { 0x32, "PCA9557" }, // Based on your Lichuang board code
};

const static audio_codec_ctrl_if_t *i2c_ctrl;

static esp_err_t expander_dev_prob(uint8_t port)
{
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = port,
    };
    for (size_t i = 0; i < sizeof(dev_list) / sizeof(dev_list[0]); i++) {
        i2c_cfg.addr = dev_list[i].addr;
        i2c_cfg.bus_handle = get_i2c_bus_handle(port);
        i2c_ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
        uint8_t data = 0;
        if (i2c_ctrl->read_reg(i2c_ctrl, PCA9557_OUTPUT_PORT, 1, &data, 1) == 0) {
            ESP_LOGI(TAG, "Detected IO expander device at 0x%02X, name is: %s",
                     dev_list[i].addr, dev_list[i].name);
            return ESP_OK;
        }
        audio_codec_delete_ctrl_if(i2c_ctrl);
        i2c_ctrl = NULL;
    }
    ESP_LOGE(TAG, "IO expander device has not detected");
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t pca9557_write_reg(uint8_t reg_addr, uint8_t data)
{
    return i2c_ctrl->write_reg(i2c_ctrl, reg_addr, 1, &data, 1);
}

static char pca9557_read_reg(uint8_t reg_addr)
{
    uint8_t data;
    i2c_ctrl->read_reg(i2c_ctrl, reg_addr, 1, &data, 1);
    return data;
}

esp_err_t pca9557_set_output_state(esp_pca9557_gpio_num_t gpio_num, esp_pca9557_io_level_t level)
{
    char data;
    esp_err_t res = ESP_FAIL;
    if (gpio_num < PCA9557_GPIO_NUM_MAX) {
        data = pca9557_read_reg(PCA9557_OUTPUT_PORT);
        res = pca9557_write_reg(PCA9557_OUTPUT_PORT, SET_BITS(data, gpio_num, level));
    } else {
        ESP_LOGE(TAG, "gpio num is error, current gpio: %d", gpio_num);
    }
    return res;
}

esp_err_t pca9557_set_io_config(esp_pca9557_gpio_num_t gpio_num, esp_pca9557_io_config_t io_config)
{
    char data;
    esp_err_t res = ESP_FAIL;
    if (gpio_num < PCA9557_GPIO_NUM_MAX) {
        data = pca9557_read_reg(PCA9557_CONFIG_PORT);
        res = pca9557_write_reg(PCA9557_CONFIG_PORT, SET_BITS(data, gpio_num, io_config));
    } else {
        ESP_LOGE(TAG, "gpio num is error, current gpio: %d", gpio_num);
    }
    return res;
}

esp_err_t pca9557_init(uint8_t port)
{
    esp_err_t ret = expander_dev_prob(port);
    if (ret == ESP_OK) {
        // Initialize with defaults - outputs low, inputs configured
        pca9557_write_reg(PCA9557_OUTPUT_PORT, 0x03);
        pca9557_write_reg(PCA9557_CONFIG_PORT, 0xf8); // All inputs by default
	ESP_LOGI("PCA9557", "Initialized IO expander: output=0x03, config=0xf8");
    }
    return ret;
}

esp_err_t pca9557_deinit()
{
    if (i2c_ctrl) {
        audio_codec_delete_ctrl_if(i2c_ctrl);
        i2c_ctrl = NULL;
    }
    return ESP_OK;
}
