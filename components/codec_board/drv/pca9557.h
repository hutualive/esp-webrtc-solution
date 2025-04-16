#ifndef _PCA9557_H
#define _PCA9557_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PCA9557_GPIO_NUM_0 = 1 << 0,
    PCA9557_GPIO_NUM_1 = 1 << 1,
    PCA9557_GPIO_NUM_2 = 1 << 2,
    PCA9557_GPIO_NUM_3 = 1 << 3,
    PCA9557_GPIO_NUM_4 = 1 << 4,
    PCA9557_GPIO_NUM_5 = 1 << 5,
    PCA9557_GPIO_NUM_6 = 1 << 6,
    PCA9557_GPIO_NUM_7 = 1 << 7,
    PCA9557_GPIO_NUM_MAX
} esp_pca9557_gpio_num_t;

typedef enum {
    PCA9557_IO_LOW,
    PCA9557_IO_HIGH,
    PCA9557_LEVEL_ERROR
} esp_pca9557_io_level_t;

typedef enum {
    PCA9557_IO_OUTPUT,
    PCA9557_IO_INPUT
} esp_pca9557_io_config_t;

/*
 * @brief Initialize PCA9557 chip
 *
 * @param codec_cfg  configuration of PCA9557
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 */
esp_err_t pca9557_init(uint8_t i2c_port);

/**
 * @brief Deinitialize PCA9557 chip
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t pca9557_deinit(void);

/*
 * @brief Set PCA9557 output state
 *
 * @param gpio_num  GPIO of PCA9557
 *
 * @return
 *      - esp_pca9557_io_level_t
 */
esp_err_t pca9557_set_output_state(esp_pca9557_gpio_num_t gpio_num, esp_pca9557_io_level_t level);

/*
 * @brief Set PCA9557 io config
 *
 * @param gpio_num  GPIO of PCA9557
 *        io_config  io config
 *
 * @return
 *      - ESP_OK
 *      - ESP_FAIL
 */
esp_err_t pca9557_set_io_config(esp_pca9557_gpio_num_t gpio_num, esp_pca9557_io_config_t io_config);

#ifdef __cplusplus
}
#endif

#endif
