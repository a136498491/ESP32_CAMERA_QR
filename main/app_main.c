// Copyright 2015-2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_netif.h"
#include "bitmap.h"
#include "led.h"
#include "qr_recoginize.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>

#include "quirc_internal.h"
#include "qr_recoginize.h"
#include "esp_camera.h"
#include "quirc.h"

#define ENABLE_TEST_PATTERN CONFIG_ENABLE_TEST_PATTERN
#define ENABLE_VERTICAL_FLIP CONFIG_ENABLE_VERTICAL_FLIP
#define ENABLE_HORIZONTAL_MIRROR CONFIG_ENABLE_HORIZONTAL_MIRROR

static const char *TAG = "camera_qr_demo";

static const char *data_type_str(int dt)
{
        switch (dt)
        {
        case QUIRC_DATA_TYPE_NUMERIC:
                return "NUMERIC";
        case QUIRC_DATA_TYPE_ALPHA:
                return "ALPHA";
        case QUIRC_DATA_TYPE_BYTE:
                return "BYTE";
        case QUIRC_DATA_TYPE_KANJI:
                return "KANJI";
        }

        return "unknown";
}

void dump_cells(const struct quirc_code *code)
{
        int u, v;

        printf("    %d cells, corners:", code->size);
        for (u = 0; u < 4; u++)
                printf(" (%d,%d)", code->corners[u].x, code->corners[u].y);
        printf("\n");

        for (v = 0; v < code->size; v++)
        {
                printf("    ");
                for (u = 0; u < code->size; u++)
                {
                        int p = v * code->size + u;

                        if (code->cell_bitmap[p >> 3] & (1 << (p & 7)))
                                printf("[]");
                        else
                                printf("  ");
                }
                printf("\n");
        }
}

void dump_data(const struct quirc_data *data)
{
        printf("    Version: %d\n", data->version);
        printf("    ECC level: %c\n", "MLHQ"[data->ecc_level]);
        printf("    Mask: %d\n", data->mask);
        printf("    Data type: %d (%s)\n", data->data_type,
               data_type_str(data->data_type));
        printf("    Length: %d\n", data->payload_len);
        printf("    Payload: %s\n", data->payload);

        if (data->eci)
                printf("    ECI: %d\n", data->eci);
}

static void dump_info(struct quirc *q)
{
        int count = quirc_count(q);
        int i;

        printf("%d QR-codes found:\n\n", count);
        for (i = 0; i < count; i++)
        {
                struct quirc_code code;
                struct quirc_data data;
                quirc_decode_error_t err;

                quirc_extract(q, i, &code);
                err = quirc_decode(&code, &data);

                dump_cells(&code);
                printf("\n");

                if (err)
                {
                        printf("  Decoding FAILED: %s\n", quirc_strerror(err));
                }
                else
                {
                        printf("  Decoding successful:\n");
                        dump_data(&data);
                }

                printf("\n");
        }
}

void app_main()
{
        esp_log_level_set("wifi", ESP_LOG_WARN);
        esp_log_level_set("gpio", ESP_LOG_WARN);
        esp_err_t err = nvs_flash_init();
        if (err != ESP_OK)
        {
                ESP_ERROR_CHECK(nvs_flash_erase());
                ESP_ERROR_CHECK(nvs_flash_init());
        }

        static camera_config_t camera_config = {
                .pin_pwdn = 32,            // power down is not used
                .pin_reset = CONFIG_RESET, // software reset will be performed
                .pin_xclk = CONFIG_XCLK,
                .pin_sscb_sda = CONFIG_SDA,
                .pin_sscb_scl = CONFIG_SCL,

                .pin_d7 = CONFIG_D7,
                .pin_d6 = CONFIG_D6,
                .pin_d5 = CONFIG_D5,
                .pin_d4 = CONFIG_D4,
                .pin_d3 = CONFIG_D3,
                .pin_d2 = CONFIG_D2,
                .pin_d1 = CONFIG_D1,
                .pin_d0 = CONFIG_D0,
                .pin_vsync = CONFIG_VSYNC,
                .pin_href = CONFIG_HREF,
                .pin_pclk = CONFIG_PCLK,

                //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
                .xclk_freq_hz = CONFIG_XCLK_FREQ,
                .ledc_timer = LEDC_TIMER_0,
                .ledc_channel = LEDC_CHANNEL_0,

#if CONFIG_PIXFORMAT_RGB565
                .pixel_format = PIXFORMAT_RGB565,
#elif CONFIG_PIXFORMAT_YUV422
                .pixel_format = PIXFORMAT_YUV422,
#elif CONFIG_PIXFORMAT_GRAYSCALE
                .pixel_format = PIXFORMAT_GRAYSCALE,
#elif CONFIG_PIXFORMAT_JPEG
                .pixel_format = PIXFORMAT_JPEG,
#elif CONFIG_PIXFORMAT_RGB888
                .pixel_format = PIXFORMAT_RGB888,
#elif CONFIG_PIXFORMAT_RAW
                .pixel_format = PIXFORMAT_RAW,
#elif CONFIG_PIXFORMAT_RGB444
                .pixel_format = PIXFORMAT_RGB444,
#elif CONFIG_PIXFORMAT_RGB555
                .pixel_format = PIXFORMAT_RGB55,
#endif

//QQVGA-QXGA Do not use sizes above QVGA when not JPEG
#if CONFIG_FRAMESIZE_96X96
                .frame_size = FRAMESIZE_96X96,
#elif CONFIG_FRAMESIZE_QQVGA
                .frame_size = FRAMESIZE_QQVGA,
#elif CONFIG_FRAMESIZE_QCIF
                .frame_size = FRAMESIZE_QCIF,
#elif CONFIG_FRAMESIZE_HQVGA
                .frame_size = FRAMESIZE_HQVGA,
#elif CONFIG_FRAMESIZE_240X240
                .frame_size = FRAMESIZE_240X240,
#elif CONFIG_FRAMESIZE_QVGA
                .frame_size = FRAMESIZE_QVGA,
#elif CONFIG_FRAMESIZE_CIF
                .frame_size = FRAMESIZE_CIF,
#elif CONFIG_FRAMESIZE_HVGA
                .frame_size = FRAMESIZE_HVGA,
#elif CONFIG_FRAMESIZE_VGA
                .frame_size = FRAMESIZE_VGA,
#elif CONFIG_FRAMESIZE_SVGA
                .frame_size = FRAMESIZE_SVGA,
#elif CONFIG_FRAMESIZE_XGA
                .frame_size = FRAMESIZE_XGA,
#elif CONFIG_FRAMESIZE_HD
                .frame_size = FRAMESIZE_HD,
#endif

                .jpeg_quality = 12, //0-63 lower number means higher quality
                .fb_count = 1       //if more than one, i2s runs in continuous mode. Use only with JPEG
        };

        err = esp_camera_init(&camera_config);
        if (err != ESP_OK)
        {
                ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
                return;
        }

        ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
        ESP_LOGI(TAG, "Camera demo ready");

        vTaskDelay(3000);

        while (true)
        {
                ESP_LOGI("", "ROUND\n");
                vTaskDelay(100);
                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb)
                {
                        ESP_LOGE(TAG, "Camera Capture Failed");
                        continue;
                }

                sensor_t *sensor = esp_camera_sensor_get();

                if (sensor->status.framesize > FRAMESIZE_VGA)
                {
                        ESP_LOGI(TAG, "Camera Size err");
                        continue;
                }

                if (fb)
                {
                        printf("begin to qr_recoginze\r\n");
                        struct quirc *q;
                        struct quirc_data qd;
                        uint8_t *image;
                        q = quirc_new();

                        if (!q)
                        {
                                printf("can't create quirc object\r\n");
                                continue;
                        }

                        printf("begin to quirc_resize\r\n");

                        //
                        if (quirc_resize(q, fb->width, fb->height) < 0)
                        {
                                printf("quirc_resize err\r\n");
                                quirc_destroy(q);
                                continue;
                        }

                        image = quirc_begin(q, NULL, NULL);
                        memcpy(image, fb->buf, fb->len);

                        quirc_end(q);

                        int id_count = quirc_count(q);
                        if (id_count == 0)
                        {
                                fprintf(stderr, "Error: not a valid qrcode\n");
                                quirc_destroy(q);
                                continue;
                        }

                        struct quirc_code code;
                        quirc_extract(q, 0, &code);
                        quirc_decode(&code, &qd);
                        dump_info(q);
                        quirc_destroy(q);
                }
                else
                {
                        ESP_LOGE(TAG, "Camera Capture Failed");
                }

                printf("finish recoginize\r\n");
        }
}
