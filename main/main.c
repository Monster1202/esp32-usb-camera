/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "tca9554.h"
#include "bsp_lcd.h"
#include "ft5x06.h"
#include "bsp_sdcard.h"
#include "fs_hal.h"
#include "hal/usb_hal.h"
#include "uvc_stream.h"
#include "jpegd2.h"
#include "usb_camera.h"

#include "wifi_sta.h"
#include "http_client.h"

static const char *TAG = "main";

#define BOOT_ANIMATION_MAX_SIZE (65 * 1024)
//esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
_Bool lcd_write_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    //x += 240;
    //y += 120;
    // w = 320;
    // h = 240;
    esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + w, y + h,  (uint16_t *)data);

    return true;
}
// _Bool lcd_write_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
// {
//     x += 240;
//     y += 120;
//     esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + w, y + h,  (uint16_t *)data);

//     return true;
// }
/* *******************************************************************************************
 * This callback function runs once per frame. Use it to perform any
 * quick processing you need, or have it put the frame into your application's
 * input queue. If this function takes too long, you'll start losing frames. */
// static void frame_cb(uvc_frame_t *frame, void *ptr)
// {
//     assert(ptr);
//     uint8_t *lcd_buffer = (uint8_t *)(ptr);
//     ESP_LOGV(TAG, "callback! frame_format = %d, seq = %u, width = %d, height = %d, length = %u, ptr = %d",
//              frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);

//     switch (frame->frame_format) {
//         case UVC_FRAME_FORMAT_MJPEG:
//             mjpegdraw(frame->data, frame->data_bytes, lcd_buffer, lcd_write_bitmap);
//             vTaskDelay(10 / portTICK_PERIOD_MS); /* add delay to free cpu to other tasks */
//             break;

//         default:
//             ESP_LOGW(TAG, "Format not supported");
//             assert(0);
//             break;
//     }
// }

void app_main(void)
{
    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));
 
    /* Initialize LCD */
    ESP_ERROR_CHECK(bsp_lcd_init());
    lcd_clear_fast(lcd_panel, COLOR_BLACK);
 
    /* malloc a buffer for RGB565 data, as 320*240*2 = 153600B,
    here malloc a smaller buffer refresh lcd with steps */
    uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(DEMO_MAX_TRANFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(lcd_buffer != NULL);

    /* Boot animation useful for LCD checking and camera power-on waiting, But consumes much flash */
#if 1
    esp_vfs_spiffs_conf_t spiffs_config = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,
        .max_files              = 5,
        .format_if_mount_failed = false
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&spiffs_config));

    uint8_t *jpeg_buf = heap_caps_malloc(BOOT_ANIMATION_MAX_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(jpeg_buf != NULL);

    // for (size_t i = 10; i <= 80; i += 2)    //start movie
    // {
    //     char file_name[64] = {0};
    //     sprintf(file_name, "/spiffs/r%03d.jpg", i);
    //     FILE *fd = fopen(file_name, "r");
    //     int read_bytes = fread(jpeg_buf, 1, BOOT_ANIMATION_MAX_SIZE, fd);
    //     fclose(fd);
    //     mjpegdraw(jpeg_buf, read_bytes, lcd_buffer, lcd_write_bitmap);
    //     ESP_LOGD(TAG, "file_name: %s, fd: %p, read_bytes: %d, free_heap: %d", file_name, fd, read_bytes, esp_get_free_heap_size());
    //     // if(i == 10 )//|| i == 80)
    //     //     ESP_LOG_BUFFER_HEX(TAG, jpeg_buf, read_bytes);
    // }

    //free(jpeg_buf);
    free(lcd_buffer);
#endif
 
    //  /* malloc frame buffer for a jpeg frame*/
    // uint8_t *frame_buffer = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // assert(frame_buffer != NULL);

    // /* malloc double buffer for usb payload, xfer_buffer_size >= frame_buffer_size*/
    // uint8_t *xfer_buffer_a = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // assert(xfer_buffer_a != NULL);
    // uint8_t *xfer_buffer_b = (uint8_t *)heap_caps_malloc(DEMO_XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // assert(xfer_buffer_b != NULL);

    // /* the quick demo skip the standred get descriptors process,
    // users need to get params from camera descriptors from PC side,
    // eg. run `lsusb -v` in linux, then modify related MACROS */
    // uvc_config_t uvc_config = {
    //     .dev_speed = USB_SPEED_FULL,
    //     .configuration = DESCRIPTOR_CONFIGURATION_INDEX,
    //     .format_index = DESCRIPTOR_FORMAT_MJPEG_INDEX,
    //     .frame_width = DEMO_FRAME_WIDTH,
    //     .frame_height = DEMO_FRAME_HEIGHT,
    //     .frame_index = DEMO_FRAME_INDEX,
    //     .frame_interval = DEMO_FRAME_INTERVAL,
    //     .interface = DESCRIPTOR_STREAM_INTERFACE_INDEX,
    //     .interface_alt = DEMO_ISOC_INTERFACE_ALT,
    //     .isoc_ep_addr = DESCRIPTOR_STREAM_ISOC_ENDPOINT_ADDR,
    //     .isoc_ep_mps = DEMO_ISOC_EP_MPS,
    //     .xfer_buffer_size = DEMO_XFER_BUFFER_SIZE,
    //     .xfer_buffer_a = xfer_buffer_a,
    //     .xfer_buffer_b = xfer_buffer_b,
    //     .frame_buffer_size = DEMO_XFER_BUFFER_SIZE,
    //     .frame_buffer = frame_buffer,
    // };

    // /* pre-config UVC driver with params from known USB Camera Descriptors*/
    // esp_err_t ret = uvc_streaming_config(&uvc_config);

    // /* Start camera IN stream with pre-configs, uvc driver will create multi-tasks internal
    // to handle usb data from different pipes, and user's callback will be called after new frame ready. */
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "uvc streaming config failed");
    // } else {
    //     uvc_streaming_start(frame_cb, (void *)(lcd_buffer));
    // }

    wifi_connect();
    //ESP_LOGI(TAG, "&lcd_buffer:%d",*lcd_buffer);
    xTaskCreate(&http_test_task, "http_test_task", BOOT_ANIMATION_MAX_SIZE, (void *)(jpeg_buf), 5, NULL);
    while (1) {
        /* task monitor code if necessary */
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


