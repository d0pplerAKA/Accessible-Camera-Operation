#include "include/GY95T.h"


mpu_gy95t_raw_data_t mpu_gy95t_raw;

int16_t mpu_gy95t_roll_fifo[GY95T_FIFO_SIZE];
int16_t mpu_gy95t_pitch_fifo[GY95T_FIFO_SIZE];
int16_t mpu_gy95t_yaw_fifo[GY95T_FIFO_SIZE];

lpf_1p_t lpf_roll_degree;
lpf_1p_t lpf_pitch_degree;
lpf_1p_t lpf_yaw_degree;


uint8_t gy95t_operation_status = 1;


void gy95t_task(void *pt)
{
    memset(&mpu_master_info, 0, sizeof(mpu_rotation_t));

    esp_err_t err;
    err = gy95t_init();
    printf("done\n");

    if(err != 0)
    {
        gy95t_operation_status = 0;
        vTaskDelete(NULL);
    }

    TickType_t tick = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&tick, 100);
        err = gy95t_onLoop();

        if(err != ESP_OK)
        {
            mpu_master_info.mpu_valid_pack = 0;

            gy95t_operation_status = 0;
            uint8_t init_counter = 0;

            while(1)
            {
                init_counter++;

                i2c_driver_delete(I2C_PORT_GY);

                err = gy95t_init();

                if(err == 0) break;

                if(init_counter == 10) esp_restart();
            }
        }
        else 
        {
            mpu_master_info.mpu_valid_pack = 0;

            gy95t_operation_status = 1;
        }
    }
}


uint8_t gy95t_init(void)
{
    memset(mpu_gy95t_roll_fifo, 0, GY95T_FIFO_SIZE * sizeof(int16_t));
    memset(mpu_gy95t_pitch_fifo, 0, GY95T_FIFO_SIZE * sizeof(int16_t));
    memset(mpu_gy95t_yaw_fifo, 0, GY95T_FIFO_SIZE * sizeof(int16_t));

    LPF_Init(&lpf_roll_degree);
    LPF_Init(&lpf_pitch_degree);
    LPF_Init(&lpf_yaw_degree);

    vTaskDelay(100);
/*
    gpio_config_t gpio_int = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = BIT(GPIO_NUM_4)
    };
    gpio_config(&gpio_int);
*/
/*
    gpio_config_t gpio_ps = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = BIT(GPIO_NUM_0)
    };
    gpio_config(&gpio_ps);
    gpio_set_level(GPIO_NUM_0, 0);
*/
    esp_err_t error_type;

    error_type = i2c_peripheral_init(I2C_PORT_GY, I2C_SCL_GPIO, I2C_SDA_GPIO, 400000);
    if(error_type != ESP_OK) return 1;
    delay_ms(200);
    
    while(1)
    {
        uint8_t who;

        i2c_read_byte(I2C_NUM_0, 0x52, 0x00, &who);
        printf("%x\n", who);
        vTaskDelay(100);
    }

    while(1)
    {
        error_type = I2C_WriteByte(I2C_PORT_GY, GY95T_ADDR, GY95T_UPDATE_RATE, GY95T_UPDATE_RATE_SET);
        if(error_type == ESP_OK) break;
        delay_ms(3000);
    }
printf("break\n");
	
    while(1)
    {
        error_type = I2C_WriteByte(I2C_PORT_GY, GY95T_ADDR, GY95T_OUTPUT_MODE, GY95T_OUTPUT_MODE_DEFAULT);
        if(error_type == ESP_OK) break;
        delay_ms(3000);
    }

    

    //error_type = I2C_WriteByte(I2C_PORT_GY, GY95T_ADDR, GY95T_OUTPUT_FORMAT, GY96T_OUTPUT_FORMAT_HEX);
    //if(error_type != ESP_OK) return 4;
    //delay_ms(10);
		
    return 0;
}


uint8_t gy95t_onLoop(void)
{   
    uint8_t raw_data[21];
    memset(raw_data, 0, sizeof(uint8_t) * 21);

    esp_err_t error_type = I2C_ReadBytes(I2C_PORT_GY, GY95T_ADDR, GY95T_ACC_X_L, raw_data, 21);
    if(error_type != ESP_OK)
    {
        return 1;
    }
    else
    {
        memcpy(&mpu_gy95t_raw, raw_data, 21);

        int32_t temp_roll = 0, temp_pitch = 0, temp_yaw = 0;

        for(uint8_t i = GY95T_FIFO_SIZE - 1; i > 0; i--)
        {
            mpu_gy95t_roll_fifo[i] = mpu_gy95t_roll_fifo[i - 1];
            mpu_gy95t_pitch_fifo[i] = mpu_gy95t_pitch_fifo[i - 1];
            mpu_gy95t_yaw_fifo[i] = mpu_gy95t_yaw_fifo[i - 1];

            temp_roll += mpu_gy95t_roll_fifo[i];
            temp_pitch += mpu_gy95t_pitch_fifo[i];
            temp_yaw += mpu_gy95t_yaw_fifo[i];
        }

        mpu_gy95t_roll_fifo[0] = mpu_gy95t_raw.raw_roll;
        mpu_gy95t_pitch_fifo[0] = mpu_gy95t_raw.raw_pitch;
        mpu_gy95t_yaw_fifo[0] = mpu_gy95t_raw.raw_yaw;

        temp_roll += mpu_gy95t_raw.raw_roll;
        temp_pitch += mpu_gy95t_raw.raw_pitch;
        temp_yaw += mpu_gy95t_raw.raw_yaw;

        temp_roll = (temp_roll) / (float) (GY95T_FIFO_SIZE);
        temp_pitch = (temp_pitch) / (float) (GY95T_FIFO_SIZE);
        temp_yaw = (temp_yaw) / (float) (GY95T_FIFO_SIZE);

        LPF_onLoop(&lpf_roll_degree, temp_roll / 100.0f);
        LPF_onLoop(&lpf_pitch_degree, temp_pitch / 100.0f);
        LPF_onLoop(&lpf_yaw_degree, temp_yaw / 100.0f);

        mpu_master_info.mpu_roll_deg = lpf_roll_degree.output;
        mpu_master_info.mpu_pitch_deg = lpf_pitch_degree.output;
        mpu_master_info.mpu_yaw_deg = lpf_yaw_degree.output;

        mpu_master_info.mpu_x_accel = mpu_gy95t_raw.raw_accel_x;
        mpu_master_info.mpu_y_accel = mpu_gy95t_raw.raw_accel_y;
        mpu_master_info.mpu_z_accel = mpu_gy95t_raw.raw_accel_z;

        mpu_master_info.mpu_x_gyro = mpu_gy95t_raw.raw_gyro_x;
        mpu_master_info.mpu_y_gyro = mpu_gy95t_raw.raw_gyro_y;
        mpu_master_info.mpu_z_gyro = mpu_gy95t_raw.raw_gyro_z;

/*
        printf("Roll: %.2f\n", mpu_gy95t_raw.raw_roll);
        printf("Pitch: %.2f\n", mpu_gy95t_raw.raw_pitch);
        printf("Yaw: %.2f\n", mpu_gy95t_raw.raw_yaw);
*/
    }
    
    return 0;
}


void LPF_Init(lpf_1p_t * lpf)
{
    lpf->rc =  (float) ((float) 1.0f / (2.0f * LPF_PI * LPF_CUTOFF_FREQ));

    lpf->alpha = constrain_float(LPF_SAMPLING_RATE_DT / (LPF_SAMPLING_RATE_DT + lpf->rc), 0.0f, 1.0f);

    lpf->output = 0.0f;
}


void LPF_onLoop(lpf_1p_t * lpf, float input)
{
    lpf->output += (input - lpf->output) * lpf->alpha;
}


float constrain_float(float input, float min, float max)
{
    if(input < min) return min;
    else if(input > max) return max;
    else return input;
}



void gy95t_ledc_status_task(void *pt)
{
    ledc_timer_config_t ledc_timer_cfg = {
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .freq_hz = 5000,
    };
    ledc_timer_config(&ledc_timer_cfg);

    ledc_channel_config_t ledc_channel_cfg = {
        .channel = LEDC_CHANNEL_0,
        .duty = 127,
        .flags.output_invert = 0,
        .gpio_num = GY95T_INDICATOR_LED,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
    };

    ledc_channel_config(&ledc_channel_cfg);

    ledc_fade_func_install(0);

    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255, 0);

    while(1)
    {
        if(gy95t_operation_status)
        {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255, 1000);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
            vTaskDelay(1000);

            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 1000);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
            vTaskDelay(1000);
        }
        else
        {
            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255, 400);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
            vTaskDelay(100);

            ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 400);
            ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
            vTaskDelay(100);
        }
    }
}
