#ifndef __A4988_H__
#define __A4988_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "math.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Remote_Motion.h"


#define A4988_Stepper_Motor0_GPIO_RST               GPIO_NUM_0
#define A4988_Stepper_Motor0_GPIO_STP               GPIO_NUM_4
#define A4988_Stepper_Motor0_GPIO_DIR               GPIO_NUM_10

#define A4988_Stepper_Motor1_GPIO_RST               GPIO_NUM_1
#define A4988_Stepper_Motor1_GPIO_STP               GPIO_NUM_2
#define A4988_Stepper_Motor1_GPIO_DIR               GPIO_NUM_3

#define A4988_Stepper_Motor2_GPIO_RST               GPIO_NUM_7
#define A4988_Stepper_Motor2_GPIO_STP               GPIO_NUM_6
#define A4988_Stepper_Motor2_GPIO_DIR               GPIO_NUM_5

#define A4988_LEDC_FREQ_HZ                          35
#define A4988_LDEC_RES_BIT                          LEDC_TIMER_14_BIT
#define A4988_LEDC_OUTPUT_DUTY                      (uint32_t) ((powf(2, A4988_LDEC_RES_BIT)) / 2.0f)


typedef struct
{
    uint32_t gpio_enable;           // low lvl active   default: - 

    uint32_t gpio_reset;            // low lvl active   default: - 

    uint32_t gpio_sleep;            // low lvl active   default: high

    uint32_t gpio_direction;
    uint32_t gpio_step;
} a4988_init_struct_t;


void a4988_stepper_motor_init(void);

void a4988_stepper_motor0_update(uint32_t dir, uint32_t duty);
void a4988_stepper_motor1_update(uint32_t dir, uint32_t duty);
void a4988_stepper_motor2_update(uint32_t dir, uint32_t duty);

void a4988_stepper_motor0_reset(void);
void a4988_stepper_motor1_reset(void);
void a4988_stepper_motor2_reset(void);
void a4988_stepper_motors_reset(void);

void a4988_task(void *pt);

#ifdef __cplusplus
}
#endif

#endif
