#include "include/Remote_Motion.h"


uint8_t esp_now_rx_addr[] = ESP_NOW_PEER_ADDR;

TaskHandle_t esp_now_task_handle = NULL;
TaskHandle_t xbox_controller_task_handle = NULL;


void esp_now_task(void * pt)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK)   vTaskDelete(NULL);


    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());

    //ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_STA, WIFI_PHY_RATE_MCS7_SGI));
    //ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(ESP_IF_WIFI_AP, WIFI_PHY_RATE_MCS7_SGI));


    err = esp_now_init();
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK)   vTaskDelete(NULL);

#ifdef C3_TX
    esp_now_peer_info_t esp_now_peer_cfg = {
        .channel = 0,
        .encrypt = false,
        .ifidx = ESP_IF_WIFI_STA,
        .peer_addr = ESP_NOW_PEER_ADDR,
    };

    err = esp_now_add_peer(&esp_now_peer_cfg);
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK)   vTaskDelete(NULL);

    //err = esp_now_register_send_cb(esp_now_send_cb);
    //ESP_ERROR_CHECK(err);
    //if(err != ESP_OK)   vTaskDelete(NULL);
    
    TickType_t tick = xTaskGetTickCount();
//mpu_master_info
    while(1)
    {
        vTaskDelayUntil(&tick, 50);
        
        esp_now_send(esp_now_rx_addr, (uint8_t *)&stepper_command, sizeof(stepper_command_t));
    }
#endif

#ifdef C3_RX
    err = esp_now_register_recv_cb(esp_now_recv_cb);
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK)   vTaskDelete(NULL);

    vTaskDelete(NULL);
#endif
}

void esp_now_send_cb(const unsigned char *mac_addr, esp_now_send_status_t status)
{
    if(status) printf("sending fail.\n");
    else printf("sent.\n");
}

void esp_now_recv_cb(const unsigned char *mac_addr, const unsigned char *data, int data_len)
{
    memcpy(&stepper_command, data, sizeof(uint8_t) * data_len);
/*
    printf("roll: %.2f pitch: %.2f yaw: %.2f\n", mpu_master_info.mpu_roll_deg,
                                            mpu_master_info.mpu_pitch_deg,
                                            mpu_master_info.mpu_yaw_deg);
*/                      
}



