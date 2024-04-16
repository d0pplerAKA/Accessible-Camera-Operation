#ifndef __GY95T_H__
#define __GY95T_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/ledc.h"

#include "Remote_Motion.h"
#include "GY95T_Reg.h"


#define GY95T_INDICATOR_LED             GPIO_NUM_8


extern uint8_t gy95t_operation_status;



void gy95t_task(void *pt);
void gy95t_ledc_status_task(void *pt);

uint8_t gy95t_init(void);
uint8_t gy95t_onLoop(void);
void LPF_Init(lpf_1p_t * lpf);
void LPF_onLoop(lpf_1p_t * lpf, float input);
float constrain_float(float input, float min, float max);

#ifdef __cplusplus
}
#endif


#endif
