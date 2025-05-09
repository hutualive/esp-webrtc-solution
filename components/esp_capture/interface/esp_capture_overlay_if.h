/**
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2025 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#pragma once

#include "esp_capture_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Capture overlay interface
 */
typedef struct esp_capture_overlay_if_t esp_capture_overlay_if_t;

struct esp_capture_overlay_if_t {
    /**
     * @brief  Open the overlay interface.
     */
    int (*open)(esp_capture_overlay_if_t *src);

    /**
     * @brief  Get the overlay region and codec type.
     */
    int (*get_overlay_region)(esp_capture_overlay_if_t *src, esp_capture_codec_type_t *codec, esp_capture_rgn_t *rgn);

    /**
     * @brief  Set the alpha value for the overlay.
     */
    int (*set_alpha)(esp_capture_overlay_if_t *src, uint8_t alpha);

    /**
     * @brief  Get the current alpha value of the overlay.
     */
    int (*get_alpha)(esp_capture_overlay_if_t *src, uint8_t *alpha);

    /**
     * @brief  Acquire a frame for the overlay.
     */
    int (*acquire_frame)(esp_capture_overlay_if_t *src, esp_capture_stream_frame_t *frame);

    /**
     * @brief  Release a previously acquired frame.
     */
    int (*release_frame)(esp_capture_overlay_if_t *src, esp_capture_stream_frame_t *frame);

    /**
     * @brief  Close the overlay interface.
     */
    int (*close)(esp_capture_overlay_if_t *src);
};

#ifdef __cplusplus
}
#endif