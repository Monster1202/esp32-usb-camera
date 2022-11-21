
#include "lcd_icon.h"
#include "jpegd2.h"
#include "esp_log.h"
#include "usb_camera.h"
#include "para_list.h"
#include "esp_spiffs.h"

#define icon1_x 663
#define icon1_y 25
#define icon2_x 743
#define icon2_y 135
#define icon3_y 205
#define icon_size1 34
#define icon_size2 18
#define icon_size3 38

#define ICON_SIZE (8 * 1024)
static const char *TAG = "lcd_icon";
size_t total = 0, used = 0;
void jpg_icon_draw(void *jpg_b,void *lcd_b,size_t i,uint32_t x_0,uint32_t y_0,uint32_t width,uint32_t hight)
{
    //size_t i = 100;
    char file_name[64] = {0};
    sprintf(file_name, "/spiffs/r%03d.jpg", i);
    FILE *fd = fopen(file_name, "r");
    int read_bytes = fread(jpg_b, 1, ICON_SIZE, fd);
    fclose(fd);
    mjpegdraw_icon(jpg_b, read_bytes, lcd_b, lcd_write_bitmap,x_0,y_0,width,hight); //x0 y0 hight width
    ESP_LOGD(TAG, "file_name: %s, fd: %p, read_bytes: %d, free_heap: %d", file_name, fd, read_bytes, esp_get_free_heap_size());
}
void lcd_icon_task(void)
{
    // parameter_read_centralizer();
    // parameter_read_rotation();
    // parameter_read_nozzle();
    // parameter_read_water();
    // parameter_read_pressure_alarm();
    PARAMETER_REMOTE remote_buf = {0};
    PARAMETER_REMOTE former_state = {0};
    spiff_init();
    uint8_t *jpg_buffer = heap_caps_malloc(ICON_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(jpg_buffer != NULL);
    uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(ICON_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(lcd_buffer != NULL);

    while(1){
        get_remote_parameter(&remote_buf);
        if(remote_buf.wifi_connection != former_state.wifi_connection){
            if(remote_buf.wifi_connection!=0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,102,icon1_x,icon1_y,icon_size1,icon_size1);  //wifi
            }
            else{
                jpg_icon_draw(jpg_buffer,lcd_buffer,100,icon1_x,icon1_y,icon_size1,icon_size1);  //wifi
            }
            printf("ICON_wifi_connection:%d",remote_buf.wifi_connection);
        }
        if(remote_buf.battery != former_state.battery){
            if(remote_buf.battery>=1700){
                jpg_icon_draw(jpg_buffer,lcd_buffer,108,icon2_x,icon1_y,icon_size1,icon_size1);  //bat 
            }
            else if(remote_buf.battery>=1500){
                jpg_icon_draw(jpg_buffer,lcd_buffer,106,icon2_x,icon1_y,icon_size1,icon_size1);  //bat
            }
            else if(remote_buf.battery<=1500){
                jpg_icon_draw(jpg_buffer,lcd_buffer,104,icon2_x,icon1_y,icon_size1,icon_size1);  //bat
            }
            printf("ICON_battery:%d",remote_buf.battery);
        }
        if(remote_buf.nozzle != former_state.nozzle){
            if(remote_buf.nozzle == 2){
                jpg_icon_draw(jpg_buffer,lcd_buffer,112,icon1_x,icon2_y,34,40);  //nozzle-bubble
                jpg_icon_draw(jpg_buffer,lcd_buffer,114,icon2_x,icon2_y,34,34);  //nozzle-water 0 
            }
            else if(remote_buf.nozzle == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,110,icon1_x,icon2_y,34,34);  //nozzle-bubble 0
                jpg_icon_draw(jpg_buffer,lcd_buffer,116,icon2_x,icon2_y,34,40);  //nozzle-water
            }
            else if(remote_buf.nozzle == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,110,icon1_x,icon2_y,34,34);  //nozzle-bubble 0
                jpg_icon_draw(jpg_buffer,lcd_buffer,114,icon2_x,icon2_y,34,34);  //nozzle-water 0 
            }
            printf("ICON_nozzle:%d",remote_buf.nozzle);
        }
        if(remote_buf.centralizer != former_state.centralizer){
            if(remote_buf.centralizer == 2){
                jpg_icon_draw(jpg_buffer,lcd_buffer,124,icon1_x,icon3_y,38,38); //centralizer 
            }
            else if(remote_buf.centralizer == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,122,icon1_x,icon3_y,38,38); //centralizer
            }
            else if(remote_buf.centralizer == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,120,icon1_x,icon3_y,34,34); //centralizer
            }
            printf("ICON_centralizer:%d",remote_buf.centralizer);
        }
        if(remote_buf.rotation != former_state.rotation){
            if(remote_buf.rotation == 2){
                jpg_icon_draw(jpg_buffer,lcd_buffer,134,icon2_x,icon3_y,38,38); //rotation  
            }
            else if(remote_buf.rotation == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,132,icon2_x,icon3_y,38,38); //rotation  
            }
            else if(remote_buf.rotation == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,130,icon2_x,icon3_y,34,34); //rotation  
            }
            printf("ICON_rotation:%d",remote_buf.rotation);
        }
        if(remote_buf.pressure_alarm != former_state.pressure_alarm){
            if(remote_buf.pressure_alarm == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,144,icon2_x,330,18,18); //senser-pressure
            }
            else if(remote_buf.pressure_alarm == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,142,icon2_x,330,18,18); //senser-pressure 0   
            }
            printf("ICON_pressure_alarm:%d",remote_buf.pressure_alarm);
        }
        if(remote_buf.water != former_state.water){
            if(remote_buf.water == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,144,icon2_x,380,18,18); //senser-water 
            }
            else if(remote_buf.water == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,142,icon2_x,380,18,18); //senser-water   
            }
            printf("ICON_water:%d",remote_buf.water);
        }
        if(remote_buf.sta_brush != former_state.sta_brush){
            if(remote_buf.sta_brush == 2){
                jpg_icon_draw(jpg_buffer,lcd_buffer,142,icon2_x,430,18,18); //status-brush  
            }
            else if(remote_buf.sta_brush == 1){
                jpg_icon_draw(jpg_buffer,lcd_buffer,144,icon2_x,430,18,18); //status-brush     
            }
            else if(remote_buf.sta_brush == 0){
                jpg_icon_draw(jpg_buffer,lcd_buffer,140,icon2_x,430,18,18); //status-brush     
            }
            printf("ICON_sta_brush:%d",remote_buf.sta_brush);
        }
        get_remote_parameter(&former_state);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
    free(lcd_buffer);
    free(jpg_buffer);
}
void spiff_init(void)
{
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    // size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        //ESP_LOGI(TAG, "Partition table: %s", conf.partition_label);
    }
    // All done, unmount partition and disable SPIFFS
    // esp_vfs_spiffs_unregister(conf.partition_label);
    // ESP_LOGI(TAG, "SPIFFS unmounted");
}



// void lcd_icon_task(void)
// {
//     // parameter_read_centralizer();
//     // parameter_read_rotation();
//     // parameter_read_nozzle();
//     // parameter_read_water();
//     // parameter_read_pressure_alarm();
//     PARAMETER_REMOTE remote_buf = {0};
    
//     spiff_init();
//     uint8_t *jpg_buffer = heap_caps_malloc(ICON_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
//     assert(jpg_buffer != NULL);
//     uint8_t *lcd_buffer = (uint8_t *)heap_caps_malloc(ICON_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
//     assert(lcd_buffer != NULL);

//     while(1){
//         get_remote_parameter(&remote_buf);
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,102,663,25,34,34);  //wifi
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,108,743,25,34,34);  //bat
//         if(remote_buf.wifi_connection!=0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,102,663,25,34,34);  //wifi
//         }
//         else{
//             jpg_icon_draw(jpg_buffer,lcd_buffer,100,663,25,34,34);  //wifi
//         }

//         if(remote_buf.battery>=1500){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,108,743,25,34,34);  //bat 
//         }
//         else if(remote_buf.battery>=500){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,106,743,25,34,34);  //bat
//         }
//         else if(remote_buf.battery<=500){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,104,743,25,34,34);  //bat
//         }

//         if(remote_buf.nozzle == 2){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,112,663,85,34,34);  //nozzle-bubble
//             jpg_icon_draw(jpg_buffer,lcd_buffer,114,743,85,34,34);  //nozzle-water 0 
//         }
//         else if(remote_buf.nozzle == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,110,663,85,34,34);  //nozzle-bubble 0
//             jpg_icon_draw(jpg_buffer,lcd_buffer,116,743,85,34,34);  //nozzle-water
//         }
//         else if(remote_buf.nozzle == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,110,663,85,34,34);  //nozzle-bubble 0
//             jpg_icon_draw(jpg_buffer,lcd_buffer,114,743,85,34,34);  //nozzle-water 0 
//         }

//         if(remote_buf.centralizer == 2){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,124,663,155,34,34); //centralizer 
//         }
//         else if(remote_buf.centralizer == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,120,663,155,34,34); //centralizer
//         }
//         else if(remote_buf.centralizer == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,118,663,155,34,34); //centralizer
//         }

//         if(remote_buf.rotation == 2){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,132,743,155,34,34); //rotation  
//         }
//         else if(remote_buf.rotation == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,128,743,155,34,34); //rotation  
//         }
//         else if(remote_buf.rotation == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,126,743,155,34,34); //rotation  
//         }

//         //jpg_icon_draw(jpg_buffer,lcd_buffer,112,663,85,34,34);  //nozzle-bubble
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,116,743,85,34,34);  //nozzle-water
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,120,663,155,34,34); //centralizer
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,128,743,155,34,34); //rotation  
//         if(remote_buf.pressure_alarm == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,144,747,230,18,18); //senser-pressure
//         }
//         else if(remote_buf.rotation == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,230,18,18); //senser-pressure 0   
//         }

//         if(remote_buf.water == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,144,747,280,18,18); //senser-water 
//         }
//         else if(remote_buf.water == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,280,18,18); //senser-water   
//         }
//         // jpg_icon_draw(jpg_buffer,lcd_buffer,144,747,230,18,18); //senser-pressure
//         // jpg_icon_draw(jpg_buffer,lcd_buffer,144,747,280,18,18); //senser-water
//         jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,330,18,18); //senser-chemical
//         jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,380,18,18); //status-camera
//         if(remote_buf.sta_brush == 2){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,430,18,18); //status-brush  
//         }
//         else if(remote_buf.sta_brush == 1){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,144,747,430,18,18); //status-brush     
//         }
//         else if(remote_buf.sta_brush == 0){
//             jpg_icon_draw(jpg_buffer,lcd_buffer,140,747,430,18,18); //status-brush     
//         }
//         //jpg_icon_draw(jpg_buffer,lcd_buffer,142,747,430,18,18); //status-brush  
//         vTaskDelay(20000 / portTICK_RATE_MS);
//     }
//     free(lcd_buffer);
//     free(jpg_buffer);
// }