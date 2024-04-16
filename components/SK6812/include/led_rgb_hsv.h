#ifndef __LED_RGB_HSV_H__
#define __LED_RGB_HSV_H__


#include "math.h"

#include "led_strip.h"


#define LED_STRIP_CONFIG(number, dev_hdl)   \
    {                                       \
        .max_leds = number,                 \
        .dev = dev_hdl,                     \
    }

#define max(a, b)                       ((a) > (b) ? (a) : (b))
#define min(a, b)                       ((a) < (b) ? (a) : (b))
#define max3(a, b, c)                   (((a) > (b) ? (a) : (b)) > (c) ? ((a) > (b) ? (a) : (b)) : (c))
#define min3(a, b, c)                   (((a) < (b) ? (a) : (b)) < (c) ? ((a) < (b) ? (a) : (b)) : (c))


#define RGB_Red                         0xFF0000
#define RGB_Green                       0x00FF00
#define RGB_Blue                        0x0000FF
#define RGB_White                       0xFFFFFF
#define RGB_Gray                        0x808080

#define RGB_Yellow                      0xFFFF00
#define RGB_Magenta                     0xFF00FF
#define RGB_Cyan                        0x00FFFF

#define RGB_Olive                       0x808000
#define RGB_Purple                      0x800080
#define RGB_Teal                        0x008080



typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    /* data */
} color_RGB_t;

typedef struct
{
    float h;
    float s;
    float v;
    /* data */
} color_HSV_t;


void led_rgb_struct_reset(color_RGB_t *rgb);
void led_hsv_struct_reset(color_HSV_t *hsv);

void led_set_rgb(color_RGB_t *rgb, uint8_t color_r, uint8_t color_g, uint8_t color_b);
void led_set_hsv(color_HSV_t *hsv, float value_h, float value_s, float value_v);

void led_rgb2hsv(color_RGB_t *rgb, color_HSV_t *hsv);
void led_hsv2rgb(color_HSV_t *hsv, color_RGB_t *rgb);


#endif
