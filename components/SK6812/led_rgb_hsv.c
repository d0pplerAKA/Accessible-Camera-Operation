#include "include/led_rgb_hsv.h"


void led_rgb_struct_reset(color_RGB_t *rgb)
{
    rgb->r = 0;
    rgb->g = 0;
    rgb->b = 0;
}

void led_hsv_struct_reset(color_HSV_t *hsv)
{
    hsv->h = 0.0f;
    hsv->s = 0.0f;
    hsv->v = 0.0f;
}

void led_set_rgb(color_RGB_t *rgb, uint8_t color_r, uint8_t color_g, uint8_t color_b)
{
    rgb->r = color_r;
    rgb->g = color_g;
    rgb->b = color_b;
}

void led_set_hsv(color_HSV_t *hsv, float value_h, float value_s, float value_v)
{
    hsv->h = value_h;
    hsv->s = value_s;
    hsv->v = value_v;
}

void led_rgb2hsv(color_RGB_t *rgb, color_HSV_t *hsv)
{
    float max, min, delta = 0;

    float r = (float) ((float) ((int32_t) rgb->r) / 255);
    float g = (float) ((float) ((int32_t) rgb->g) / 255);
    float b = (float) ((float) ((int32_t) rgb->b) / 255);

    max = max3(r, g, b);
    min = min3(r, g, b);
    delta = max - min;

    if(delta != 0)
    {
        if(r == max) hsv->h = ((g - b) / delta) * 60; 
        else if(g == max) hsv->h = 120 + (((b - r) / delta) * 60);
        else if(b == max) hsv->h = 240 + (((r - g) / delta) * 60);

        if(hsv->h < 0) hsv->h += 360;
    }
    else hsv->h = 0;

    if(max == 0) hsv->s = 0;
    else hsv->s = (float)(delta/max);

    hsv->v = max;
}

void led_hsv2rgb(color_HSV_t *hsv, color_RGB_t *rgb)
{
    int32_t i;
    float f, a, b, c;

    float h = hsv->h;
    float s = hsv->s;
    float v = hsv->v;

	if(h >= 360) h = 0;

    if(s != 0)
    {
        h /= 60.0;                                          // sector 0 to 5, h_max=360 360/60=6[0,1,2,3,4,5]
        i = (int32_t) floor(h);                             // floor(h)
        f = h - i;                                          // factorial path of h
        a = v * (1 - s);
        b = v * (1 - s * f);
        c = v * (1 - s * (1 - f));

        switch(i)
        {
            case 0:
                rgb->r = (uint8_t) ((int32_t) (v * 255));   //v*255
                rgb->g = (uint8_t) ((int32_t) (c * 255));   //c*255;
                rgb->b = (uint8_t) ((int32_t) (a * 255));   //a*255;
                break;
            
            case 1:
                rgb->r = (uint8_t) ((int32_t) (b * 255));   //b*255;
                rgb->g = (uint8_t) ((int32_t) (v * 255));   //v*255;
                rgb->b = (uint8_t) ((int32_t) (a * 255));   //a*255;
                break;
            
            case 2:
                rgb->r = (uint8_t) ((int32_t) (a * 255));   //a*255;
                rgb->g = (uint8_t) ((int32_t) (v * 255));   //v*255;
                rgb->b = (uint8_t) ((int32_t) (c * 255));   //c*255;
                break;
            
            case 3:
                rgb->r = (uint8_t) ((int32_t) (a * 255));   //a*255;
                rgb->g = (uint8_t) ((int32_t) (b * 255));   //b*255;
                rgb->b = (uint8_t) ((int32_t) (v * 255));   //v*255;
                break;
            
            case 4:
                rgb->r = (uint8_t) ((int32_t) (c * 255));   //c*255;
                rgb->g = (uint8_t) ((int32_t) (a * 255));   //a*255;
                rgb->b = (uint8_t) ((int32_t) (v * 255));   //v*255;
                break;
            
            default:
                rgb->r = (uint8_t) ((int32_t) (v * 255));       //v*255;
                rgb->g = (uint8_t) ((int32_t) (a * 255));       //a*255;
                rgb->b = (uint8_t) ((int32_t) (b * 255));       //b*255;
                break;
        }
    }
    else
    {
        rgb->r = (uint8_t) ((int32_t) (v * 255));
        rgb->g = (uint8_t) ((int32_t) (v * 255));
        rgb->b = (uint8_t) ((int32_t) (v * 255));
    }
}

