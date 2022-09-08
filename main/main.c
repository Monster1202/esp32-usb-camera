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
//#include "mqtt_app.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_demos.h"
#include "lv_examples.h"

static const char *TAG = "main";
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_IO_1     35
#define GPIO_INPUT_IO_2     36
#define GPIO_INPUT_IO_3     37
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_1)|(1ULL<<GPIO_INPUT_IO_2)|(1ULL<<GPIO_INPUT_IO_3))
// #define GPIO_OUTPUT_IO_1     35
// #define GPIO_OUTPUT_IO_2     36
// #define GPIO_OUTPUT_IO_3     37
// #define GPIO_OUTPUT_PIN_SEL  ( (1ULL<<GPIO_OUTPUT_IO_1)| (1ULL<<GPIO_OUTPUT_IO_2)| (1ULL<<GPIO_OUTPUT_IO_3))

void gpio_init(void);

#define BOOT_ANIMATION_MAX_SIZE (65 * 1024)
//esp_err_t esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle_t panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
_Bool lcd_write_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    x += 80;
    //y += 120;
    esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + w, y + h,  (uint16_t *)data);

    return true;
}

void app_main(void)
{
    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));
 
    /* Initialize LCD */
    ESP_ERROR_CHECK(bsp_lcd_init());
    lcd_clear_fast(lcd_panel, COLOR_BLACK);
 
    /* malloc a buffer for RGB565 data, as 320*240*2 = 153600B,
    here malloc a smaller buffer refresh lcd with steps */
    // uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(DEMO_MAX_TRANFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    // assert(lcd_buffer != NULL);

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
    //free(lcd_buffer);
#endif
 //ESP_LOGI(TAG, "&lcd_buffer:%d",*lcd_buffer);

    gpio_init();
    wifi_connect();
    
    xTaskCreate(&http_test_task, "http_test_task", BOOT_ANIMATION_MAX_SIZE, (void *)(jpeg_buf), 5, NULL);
    //mqtt_init();
    //lvgl_init();
    //uint8_t s_led_state = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
         lv_timer_handler();
        /* task monitor code if necessary */
        // s_led_state = !s_led_state;
        // gpio_set_level(GPIO_OUTPUT_IO_1, s_led_state);
        // gpio_set_level(GPIO_OUTPUT_IO_2, s_led_state);
        // gpio_set_level(GPIO_OUTPUT_IO_3, s_led_state);
        // lcd_clear_fast(lcd_panel, COLOR_BLACK);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // lcd_clear(lcd_panel, COLOR_BLUE);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // lcd_draw_picture_test(lcd_panel);
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}


static xQueueHandle gpio_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    uint8_t buf_state = 0;
    //int cnt = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            buf_state = gpio_get_level(io_num);
            ESP_LOGI(TAG,"GPIO[%d] intr, val: %d", io_num, buf_state);
            //if(buf_state ==1)
            //sw_key_read(io_num,buf_state); //judge button press once twice or long
        }
    }
}
void gpio_init(void)
{
    //GPIO
    //esp_log_level_set("GPIO_CTRL", ESP_LOG_DEBUG);  //ESP_LOG_DEBUG ESP_LOG_INFO ESP_LOG_WARN
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    // //disable interrupt
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // //set as output mode
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //disable pull-down mode
    // io_conf.pull_down_en = 0;
    // //disable pull-up mode
    // io_conf.pull_up_en = 0;
    // //configure GPIO with the given settings
    // gpio_config(&io_conf);
        //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;//GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    //gpio_set_intr_type(GPIO_INPUT_IO_STOP, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 4096, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    //gpio_isr_handler_add(GPIO_INPUT_IO_STOP, gpio_isr_handler, (void*) GPIO_INPUT_IO_STOP);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
    gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);
    // gpio_isr_handler_add(GPIO_INPUT_IO_4, gpio_isr_handler, (void*) GPIO_INPUT_IO_4);
    // gpio_isr_handler_add(GPIO_INPUT_IO_5, gpio_isr_handler, (void*) GPIO_INPUT_IO_5);
    // gpio_isr_handler_add(GPIO_INPUT_IO_6, gpio_isr_handler, (void*) GPIO_INPUT_IO_6);
    // gpio_isr_handler_add(GPIO_INPUT_IO_7, gpio_isr_handler, (void*) GPIO_INPUT_IO_7);
    //remove isr handler for gpio number.
    //gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    //hook isr handler for specific gpio pin again
    //gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    ESP_LOGI(TAG, "Minimum free heap size: %d bytes", esp_get_minimum_free_heap_size());

}

// void lvgl_init(void);
// #define LVGL_TICK_PERIOD_MS    1
// static void example_increase_lvgl_tick(void *arg)
// {
//     /* Tell LVGL how many milliseconds has elapsed */
//     lv_tick_inc(LVGL_TICK_PERIOD_MS);
// }
// void lvgl_init(void)
// {
//     const esp_timer_create_args_t lvgl_tick_timer_args = {
//         .callback = &example_increase_lvgl_tick,
//         .name = "lvgl_tick"
//     };
//     esp_timer_handle_t lvgl_tick_timer = NULL;
//     ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
//     ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));  //

//     /* LVGL init */
//     lv_init();                  //内核初始化
//     lv_port_disp_init();	    //接口初始化
//     lv_port_indev_init();       //输入设备初始化
//     // lv_port_fs_init();       //文件系统初始化

//     /* example lvgl demos */
//     //lv_demo_music();
//     // lv_demo_widgets();
//     // lv_demo_keypad_encoder();    
//      //lv_demo_benchmark();  
//      //lv_demo_stress();  
     
//     lv_example_get_started_1();
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