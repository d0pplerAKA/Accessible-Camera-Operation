#include "include/SK6812.h"

color_RGB_t sk6812_rgb_color_struct;
color_HSV_t sk6812_hsv_color_struct;

led_strip_t *sk6812_strip = NULL;


void sk6812_init(void)
{
    rmt_config_t sk6812_cfg;

    sk6812_cfg.rmt_mode = RMT_MODE_TX;
    sk6812_cfg.channel = RMT_CHANNEL_0;
    sk6812_cfg.gpio_num = ESP_STATUS_WS2812_PIN;
    sk6812_cfg.clk_div = 2;
    sk6812_cfg.mem_block_num = 1;
    sk6812_cfg.flags = 0;

    sk6812_cfg.tx_config.carrier_duty_percent = 33;

    sk6812_cfg.tx_config.carrier_en = false;
    sk6812_cfg.tx_config.carrier_freq_hz = 38000;       //
    sk6812_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
    
    sk6812_cfg.tx_config.idle_output_en = true;
    sk6812_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    
#if SOC_RMT_SUPPORT_TX_LOOP_COUNT
    sk6812_cfg.tx_config.loop_count = 0;
#endif

    sk6812_cfg.tx_config.loop_en = false;

    rmt_config(&sk6812_cfg);
    rmt_driver_install(sk6812_cfg.channel, 0, 0);

    led_strip_config_t sk6812_config = LED_STRIP_CONFIG(1, RMT_CHANNEL_0);
    sk6812_strip = led_strip_new_rmt_ws2812(&sk6812_config);
    sk6812_strip->clear(sk6812_strip, 100);
    sk6812_strip->set_pixel(sk6812_strip, 0, 0, 0, 0);
    sk6812_strip->refresh(sk6812_strip, 100);
}

void sk6812_task(void *pt)
{
    led_rgb_struct_reset(&sk6812_rgb_color_struct);
    led_hsv_struct_reset(&sk6812_hsv_color_struct);

    sk6812_init();

#ifdef C3_TX
    while(mpu_dmp_status == 0)
    {
        sk6812_set_rgb_value(RGB_Red);

        vTaskDelay(100);
    }

    while(mpu_dmp_calibration == 0)
    {
        sk6812_set_rgb_value(RGB_Yellow);

        vTaskDelay(100);
    }
#endif

    for(uint8_t i = 0; i < 5; i++)
    {
        sk6812_set_rgb_value(RGB_Green);
        vTaskDelay(50);

        sk6812_set_clear();
        vTaskDelay(50);
    }

    uint16_t iteration_i = 0;
    uint8_t iteration_i_switch = 0;

    TickType_t tick = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&tick, 50);

        led_set_hsv(&sk6812_hsv_color_struct, 0.99f, 0.99f, (float) iteration_i);
        led_hsv2rgb(&sk6812_hsv_color_struct, &sk6812_rgb_color_struct);
        
        sk6812_set_rgb((uint8_t) ((float) sk6812_rgb_color_struct.r / 64.0f), 
                        (uint8_t) ((float) sk6812_rgb_color_struct.g / 64.0f), 
                        (uint8_t) ((float) sk6812_rgb_color_struct.b / 64.0f));

        if(iteration_i_switch == 0) iteration_i += 1;
        else iteration_i -= 1;

        if(iteration_i == 360) iteration_i_switch = !iteration_i_switch;
        if(iteration_i == 0) iteration_i_switch = !iteration_i_switch;
    }
}


void sk6812_set_clear(void)
{
    sk6812_strip->clear(sk6812_strip, 100);
}


void sk6812_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    //sk6812_strip->clear(sk6812_strip, 100);
    sk6812_strip->set_pixel(sk6812_strip, 0, r, g, b);
    sk6812_strip->refresh(sk6812_strip, 500);
}


void sk6812_set_rgb_value(uint32_t rgb_val)
{
    uint8_t r = (rgb_val >> 16) & 0xFF;
    uint8_t g = (rgb_val >> 8) & 0xFF;
    uint8_t b = (rgb_val >> 0) & 0xFF;

    sk6812_set_rgb(r, g, b);
}

