#ifndef __SK6812_H__
#define __SK6812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "led_strip.h"
#include "led_rgb_hsv.h"

#include "esp_random.h"

#include "driver/rmt.h"

#include "struct_component.h"


#ifdef C3_TX
    #define ESP_STATUS_WS2812_PIN       GPIO_NUM_4
#endif

#ifdef C3_RX
    #define ESP_STATUS_WS2812_PIN       GPIO_NUM_9
#endif


void sk6812_init(void);
void sk6812_task(void *pt);

void sk6812_set_clear(void);
void sk6812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void sk6812_set_rgb_value(uint32_t rgb_val);


#ifdef __cplusplus
}
#endif

#endif

