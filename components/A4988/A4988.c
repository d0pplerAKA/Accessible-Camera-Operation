#include "include/A4988.h"


//a4988_init_struct_t a4988_stepper_motor[3];

void a4988_stepper_motor_init(void)
{
    //memset(&a4988_stepper_motor, 0, sizeof(a4988_init_struct_t) * 3);

    // three pins should be init, reset, step, dir
    gpio_config_t a4988_pin_reset = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask = BIT(A4988_Stepper_Motor0_GPIO_RST) | 
                        BIT(A4988_Stepper_Motor1_GPIO_RST) | 
                        BIT(A4988_Stepper_Motor2_GPIO_RST)
    };
    gpio_config(&a4988_pin_reset);
    gpio_set_level(A4988_Stepper_Motor0_GPIO_RST, 1);
    gpio_set_level(A4988_Stepper_Motor1_GPIO_RST, 1);
    gpio_set_level(A4988_Stepper_Motor2_GPIO_RST, 1);

    gpio_config_t a4988_pin_dir = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = BIT(A4988_Stepper_Motor0_GPIO_DIR) | 
                        BIT(A4988_Stepper_Motor1_GPIO_DIR) | 
                        BIT(A4988_Stepper_Motor2_GPIO_DIR)
    };
    gpio_config(&a4988_pin_dir);
    gpio_set_level(A4988_Stepper_Motor0_GPIO_DIR, 0);
    gpio_set_level(A4988_Stepper_Motor1_GPIO_DIR, 0);
    gpio_set_level(A4988_Stepper_Motor2_GPIO_DIR, 0);


    ledc_timer_config_t ledc_timer0_cfg = {
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = A4988_LDEC_RES_BIT,
        .freq_hz = A4988_LEDC_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer0_cfg);

    ledc_timer_config_t ledc_timer1_cfg = {
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = A4988_LDEC_RES_BIT,
        .freq_hz = A4988_LEDC_FREQ_HZ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1
    };
    ledc_timer_config(&ledc_timer1_cfg);

    ledc_timer_config_t ledc_timer2_cfg = {
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = A4988_LDEC_RES_BIT,
        .freq_hz = A4988_LEDC_FREQ_HZ * 2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_2
    };
    ledc_timer_config(&ledc_timer2_cfg);

    ledc_channel_config_t ledc_channel_cfg[3] = {
        {
            .channel = LEDC_CHANNEL_2,
            .duty = (uint32_t) powf(2, ledc_timer0_cfg.duty_resolution),
            .flags.output_invert = 1,
            .gpio_num = A4988_Stepper_Motor0_GPIO_STP,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0
        },
        {
            .channel = LEDC_CHANNEL_1,
            .duty = (uint32_t) powf(2, ledc_timer1_cfg.duty_resolution),
            .flags.output_invert = 1,
            .gpio_num = A4988_Stepper_Motor1_GPIO_STP,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_1
        },
        {
            .channel = LEDC_CHANNEL_0,
            .duty = (uint32_t) powf(2, ledc_timer2_cfg.duty_resolution),
            .flags.output_invert = 1,
            .gpio_num = A4988_Stepper_Motor2_GPIO_STP,
            .hpoint = 0,
            .intr_type = LEDC_INTR_DISABLE,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_2
        }
    };

    ledc_channel_config(&ledc_channel_cfg[0]);
    ledc_channel_config(&ledc_channel_cfg[1]); 
    ledc_channel_config(&ledc_channel_cfg[2]); 

    ledc_fade_func_install(0);
}

void a4988_stepper_motor0_update(uint32_t dir, uint32_t duty)
{
    gpio_set_level(A4988_Stepper_Motor0_GPIO_DIR, dir);
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty, 0);
}

void a4988_stepper_motor1_update(uint32_t dir, uint32_t duty)
{
    gpio_set_level(A4988_Stepper_Motor1_GPIO_DIR, dir);
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty, 0);
}

void a4988_stepper_motor2_update(uint32_t dir, uint32_t duty)
{
    gpio_set_level(A4988_Stepper_Motor2_GPIO_DIR, dir);
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 0);
}

void a4988_stepper_motor0_reset(void)
{
    gpio_set_level(A4988_Stepper_Motor0_GPIO_RST, 0);
    vTaskDelay(50 / portTICK_RATE_MS);

    gpio_set_level(A4988_Stepper_Motor0_GPIO_RST, 1);
    vTaskDelay(50 / portTICK_RATE_MS);
}

void a4988_stepper_motor1_reset(void)
{
    gpio_set_level(A4988_Stepper_Motor1_GPIO_RST, 0);
    vTaskDelay(50 / portTICK_RATE_MS);

    gpio_set_level(A4988_Stepper_Motor1_GPIO_RST, 1);
    vTaskDelay(50 / portTICK_RATE_MS);
}

void a4988_stepper_motor2_reset(void)
{
    gpio_set_level(A4988_Stepper_Motor2_GPIO_RST, 0);
    vTaskDelay(50 / portTICK_RATE_MS);

    gpio_set_level(A4988_Stepper_Motor2_GPIO_RST, 1);
    vTaskDelay(50 / portTICK_RATE_MS);
}

void a4988_stepper_motors_reset(void)
{
    gpio_set_level(A4988_Stepper_Motor0_GPIO_RST, 0);
    gpio_set_level(A4988_Stepper_Motor1_GPIO_RST, 0);
    gpio_set_level(A4988_Stepper_Motor2_GPIO_RST, 0);
    vTaskDelay(50 / portTICK_RATE_MS);

    gpio_set_level(A4988_Stepper_Motor0_GPIO_RST, 1);
    gpio_set_level(A4988_Stepper_Motor1_GPIO_RST, 1);
    gpio_set_level(A4988_Stepper_Motor2_GPIO_RST, 1);
    vTaskDelay(50 / portTICK_RATE_MS);
}

void a4988_task(void *pt)
{
    memset(&mpu_master_info, 0, sizeof(mpu_rotation_t ));
    
    uint8_t motor0_rst_flag = 0;
    uint8_t motor1_rst_flag = 0;
    uint8_t motor2_rst_flag = 0;
    float temp_joyLHori = 0.0f;
    float temp_joyLVert = 0.0f;
    float temp_joyRVert = 0.0f;

    a4988_stepper_motor_init();
    a4988_stepper_motors_reset();

    TickType_t tick = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&tick, 100);

        if(!camera_mode)
        {
            if(eTaskGetState(xbox_controller_task_handle) != eSuspended)  // Xbox Contorller Online
            {
                vTaskDelayUntil(&tick, 100);

                // Xbox Control Stepper Motor
                temp_joyLHori = (float) xbox_info.joyLHori / 32768.0f;
                temp_joyLVert = (float) xbox_info.joyLVert / 32768.0f;
                temp_joyRVert = (float) xbox_info.joyRVert / 32768.0f;

                if(!(temp_joyLHori == 0.0f && temp_joyLVert == 0.0f && temp_joyRVert == 0.0f))
                {
                    if(temp_joyLHori > 1.5f)   
                    {
                        a4988_stepper_motor0_update(0, A4988_LEDC_OUTPUT_DUTY);
                        motor0_rst_flag = 1;
                        //printf("temp_joyLHori: %.2f\n", temp_joyLHori);
                    }
                    else if(temp_joyLHori < 0.5f)
                    {
                        a4988_stepper_motor0_update(1, A4988_LEDC_OUTPUT_DUTY);
                        motor0_rst_flag = 1;
                        //printf("temp_joyLHori: %.2f\n", temp_joyLHori);
                    }
                    else
                    {
                        if(motor0_rst_flag)
                        {
                            motor0_rst_flag = 0;
                            a4988_stepper_motor0_reset();
                        }

                        a4988_stepper_motor0_update(0, 0);
                    }

                    if(temp_joyLVert > 1.5f)
                    {
                        a4988_stepper_motor1_update(0, A4988_LEDC_OUTPUT_DUTY);
                        motor1_rst_flag = 1;
                        //printf("temp_joyLVert: %.2f\n", temp_joyLVert);

                    }
                    else if(temp_joyLVert < 0.5f)
                    {
                        a4988_stepper_motor1_update(1, A4988_LEDC_OUTPUT_DUTY);
                        motor1_rst_flag = 1;
                        //printf("temp_joyLVert: %.2f\n", temp_joyLVert);
                    }
                    else
                    {
                        if(motor1_rst_flag)
                        {
                            motor1_rst_flag = 0;
                            a4988_stepper_motor1_reset();
                        }
                        a4988_stepper_motor1_update(0, 0);
                    }

                    if(temp_joyRVert > 1.5f)
                    {
                        a4988_stepper_motor2_update(0, A4988_LEDC_OUTPUT_DUTY);
                        motor2_rst_flag = 1;
                        //printf("temp_joyRVert: %.2f\n", temp_joyRVert);
                    }
                    else if(temp_joyRVert < 0.5f)
                    {
                        a4988_stepper_motor2_update(1, A4988_LEDC_OUTPUT_DUTY);
                        motor2_rst_flag = 1;
                        //printf("temp_joyRVert: %.2f\n", temp_joyRVert);
                    }
                    else
                    {
                        if(motor2_rst_flag)
                        {
                            motor2_rst_flag = 0;
                            a4988_stepper_motor2_reset();
                        }
                        a4988_stepper_motor2_update(0, 0);
                    }
                }
            }
            else                                                        // ESP-NOW Remote MPU Data
            {
                //printf("esp now handler\n");

                switch(stepper_command.motor_pitching)
                {
                    case 1:
                    {
                        a4988_stepper_motor0_update(0, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera look up\n");

                        break;
                    }

                    case -1:
                    {
                        a4988_stepper_motor0_update(1, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera look down\n");

                        break;
                    }

                    default:
                    {
                        a4988_stepper_motor0_update(0, 0);
                    }
                }

                switch(stepper_command.motor_height)
                {
                    case 1:
                    {
                        a4988_stepper_motor1_update(0, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera goes up\n");
                        break;
                    }

                    case -1:
                    {
                        
                        a4988_stepper_motor1_update(1, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera goes down\n");
                        break;
                    }

                    default:
                    {
                        a4988_stepper_motor1_update(0, 0);
                    }
                }

                switch(stepper_command.motor_rotation)
                {
                    case 1:
                    {
                        a4988_stepper_motor2_update(0, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera look left\n");
                        break;
                    }

                    case -1:
                    {
                        a4988_stepper_motor2_update(1, A4988_LEDC_OUTPUT_DUTY);
                        printf("Camera look right\n");
                        break;
                    }

                    default:
                    {
                        a4988_stepper_motor2_update(0, 0);
                    }
                }

                vTaskDelayUntil(&tick, 50);
            }
        }
    }
}

