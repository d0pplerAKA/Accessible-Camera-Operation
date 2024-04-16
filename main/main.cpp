#include "Arduino.h"

#include "Remote_Motion.h"

#include "XBoxController.h"
#include "A4988.h"
#include "SK6812.h"

#include "hal/usb_serial_jtag_ll.h"




extern void task_initI2C(void);
extern void task_mpu(void*);
extern void task_motion(void *pt);
extern void esp_status_task(void *pt);

extern "C" void app_main()
{
    initArduino();
    // Do your own thing

    vTaskDelay(500);
    
    /*** C3 RX ***/
#ifdef C3_RX
    xTaskCreate(sk6812_task, "sk6812", 8192, NULL, 5, NULL);
    xTaskCreate(esp_now_task, "esp now", 10240, NULL, 7, &esp_now_task_handle);
    xTaskCreate(xboxController_task, "xbox task", 10240, NULL, 7, &xbox_controller_task_handle);

    vTaskDelay(500);
    xTaskCreate(esp_status_task, "status task", 8192, NULL, 5, NULL);
    xTaskCreate(a4988_task, "a4988 task", 8192, NULL, 4, NULL);

    //xTaskCreate(xboxController_task, "xbox task", 12288, NULL, 5, NULL);
    //xTaskCreate(esp_now_task, "esp now", 8192, NULL, 7, NULL);

    //xTaskCreate(xboxController_mode_select, "xbox mode", 4096, NULL, 3, NULL);
#endif
    /*** C3 RX ***/

    /*** C3 TX ***/
#ifdef C3_TX
    xTaskCreate(sk6812_task, "sk6812", 8192, NULL, 3, NULL);
    xTaskCreate(&task_motion, "motion_task", 4096, NULL, 4, NULL);
    xTaskCreate(&task_mpu, "mpu_task", 8192, NULL, 5, NULL);
    xTaskCreate(esp_now_task, "esp now", 8192, NULL, 6, NULL);
    //xTaskCreate(esp_wifi_udp_tx_task, "udp tx", 8192, NULL, 6, NULL);
#endif  
    /*** C3 TX ***/
}

void esp_status_task(void *pt)
{
    gpio_config_t led_cfg;
    led_cfg.intr_type = GPIO_INTR_DISABLE;
    led_cfg.mode = GPIO_MODE_OUTPUT;
    led_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    led_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_cfg.pin_bit_mask = BIT(ESP_STATUS_LED_PIN);

    gpio_config(&led_cfg);
    gpio_set_level(ESP_STATUS_LED_PIN, 1);

    uint8_t *usb_serial_rx_buf;
    //int usb_serial_cnt;

    usb_serial_rx_buf = (uint8_t *) malloc(sizeof(uint8_t) * 64);
    int usb_serial_rx_cnt;
    
    memset(usb_serial_rx_buf, 0, sizeof(uint8_t) * 64);

    TickType_t tick = xTaskGetTickCount();
    while(1)
    {
        if(eTaskGetState(xbox_controller_task_handle) == eSuspended)    gpio_set_level(ESP_STATUS_LED_PIN, 1);
        else                                                            gpio_set_level(ESP_STATUS_LED_PIN, 0);

        if(usb_serial_jtag_ll_rxfifo_data_available())
        {
            usb_serial_rx_cnt = usb_serial_jtag_ll_read_rxfifo(usb_serial_rx_buf, 64);

            if(usb_serial_rx_cnt != 1)
            {
                if(strncmp("camera", (const char *) usb_serial_rx_buf, sizeof(uint8_t) * 6) == 0)
                {
                    if(eTaskGetState(xbox_controller_task_handle) != eSuspended)
                    {
                        printf("Susbending xbox task\n");
                        vTaskSuspend(xbox_controller_task_handle);
                    }

                    camera_mode = 1;
                    a4988_stepper_motor0_reset();
                    a4988_stepper_motor1_reset();
                    a4988_stepper_motor2_reset();
                }
                else if(strncmp("esp_now", (const char *) usb_serial_rx_buf, sizeof(uint8_t) * 7) == 0)
                {            
                    if(eTaskGetState(xbox_controller_task_handle) != eSuspended)
                    {
                        printf("Susbending xbox task\n");

                        camera_mode = 0;
                        vTaskSuspend(xbox_controller_task_handle);
                    }
                }
                else
                {
                    if(strncmp("xbox", (const char *) usb_serial_rx_buf, sizeof(uint8_t) * 4) == 0)
                    {
                        if(eTaskGetState(xbox_controller_task_handle) == eSuspended)
                        {
                            printf("Resume xbox controller task\n");

                            camera_mode = 0;
                            vTaskResume(xbox_controller_task_handle);
                        }
                        else printf("xbox controller task already running!\n");
                    }
                }

                memset(usb_serial_rx_buf, 0, sizeof(uint8_t) * 64);
            }
            else
            {
                if(camera_mode)
                {
                    uint8_t temp_order = usb_serial_rx_buf[0];
                    memset(usb_serial_rx_buf, 0, sizeof(uint8_t) * 64);

                    switch(temp_order & 0x0F)
                    {
                        case 0x0C:              // left
                        {
                            a4988_stepper_motor0_update(0, A4988_LEDC_OUTPUT_DUTY);
                            printf("Camera look left\n");

                            break;
                        }

                        case 0x03:              // right
                        {
                            a4988_stepper_motor0_update(1, A4988_LEDC_OUTPUT_DUTY);
                            printf("Camera look right\n");

                            break;
                        }
                    }

                    switch(temp_order & 0xF0)
                    {
                        case 0xC0:              // up
                        {
                            a4988_stepper_motor1_update(0, A4988_LEDC_OUTPUT_DUTY);
                            printf("Camera look up\n");

                            break;
                        }

                        case 0x30:              // down
                        {
                            a4988_stepper_motor1_update(1, A4988_LEDC_OUTPUT_DUTY);
                            printf("Camera look down\n");

                            break;
                        }
                    }

                    vTaskDelayUntil(&tick, 150);

                    a4988_stepper_motor0_update(0, 0);
                    a4988_stepper_motor1_update(0, 0);
                    a4988_stepper_motor2_update(0, 0);

                    vTaskDelayUntil(&tick, 25);
                }
            }
        }

        if(camera_mode) { vTaskDelayUntil(&tick, 5); }
        else { vTaskDelayUntil(&tick, 50); }
    }
}
