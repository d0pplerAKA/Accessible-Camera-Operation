#ifndef __REMOTE_MOTION_H__
#define __REMOTE_MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"

#include "driver/gpio.h"
#include "driver/rmt.h"


#include "esp_random.h"

#include "struct_component.h"


//64:e8:33:82:c4:0c
//dc:54:75:74:1b:68 -> RX
#define ESP_NOW_PEER_ADDR           {0xdc, 0x54, 0x75, 0x74, 0x1b, 0x68}

#define ESP_STATUS_LED_PIN          GPIO_NUM_8


extern TaskHandle_t esp_now_task_handle;
extern TaskHandle_t xbox_controller_task_handle;


void esp_now_task(void * pt);
void esp_now_send_cb(const unsigned char *mac_addr, esp_now_send_status_t status);
void esp_now_recv_cb(const unsigned char *mac_addr, const unsigned char *data, int data_len);


#ifdef __cplusplus
} 
#endif

#endif
