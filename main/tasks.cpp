#include <stdio.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "struct_component.h"


#define PIN_SDA 		8
#define PIN_CLK 		7

#define LPF_SAMPLING_RATE               100
#define LPF_SAMPLING_RATE_DT            ((float) (1.0f / (float)LPF_SAMPLING_RATE))
#define LPF_CUTOFF_FREQ                 37
#define LPF_PI                          M_PI
#define LPF_LFP_BUFFER                  4

typedef struct
{
    float rc;
    float alpha;

    float output;

    /* data */
} lpf_1p_t;


lpf_1p_t lpf_gyro_x;
lpf_1p_t lpf_gyro_y;
lpf_1p_t lpf_gyro_z;

lpf_1p_t lpf_accel_x;
lpf_1p_t lpf_accel_y;
lpf_1p_t lpf_accel_z;

lpf_1p_t lpf_degree_x;
lpf_1p_t lpf_degree_y;
lpf_1p_t lpf_degree_z;


void LPF_Init(lpf_1p_t * lpf);
void LPF_onLoop(lpf_1p_t * lpf, float input);
float constrain_float(float input, float min, float max);




void task_initI2C(void)
{
	i2c_config_t conf;

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)PIN_SDA;
	conf.scl_io_num = (gpio_num_t)PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0;

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
	
	//printf("this is task 1\n");
	//vTaskDelay(5000/portTICK_PERIOD_MS);
	//vTaskDelete(NULL);
}

void task_mpu(void*)
{
	task_initI2C();

	LPF_Init(&lpf_accel_x);
	LPF_Init(&lpf_accel_y);
	LPF_Init(&lpf_accel_z);

	LPF_Init(&lpf_gyro_x);
	LPF_Init(&lpf_gyro_y);
	LPF_Init(&lpf_gyro_z);

	LPF_Init(&lpf_degree_x);
	LPF_Init(&lpf_degree_y);
	LPF_Init(&lpf_degree_z);

	Quaternion q;           
	VectorFloat gravity;

	VectorInt16 mpu_raw_accel;
	VectorInt16 mpu_linear_accel;
	VectorInt16 mpu_gyro;

	float ypr[3];
	uint16_t packetSize = 42;    
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

	MPU6050 mpu = MPU6050();
	mpu.initialize();
	mpu.dmpInitialize();
mpu_dmp_status = 1;
	
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
mpu_dmp_calibration = 1;

	mpu.setDMPEnabled(true);

	TickType_t tick = xTaskGetTickCount();

	while(1)
	{
	    mpuIntStatus = mpu.getIntStatus();
		
		fifoCount = mpu.getFIFOCount();

	    if((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
	        mpu.resetFIFO();
			mpu_master_info.mpu_valid_pack = 0;
	    } 
		else if(mpuIntStatus & 0x02) 
		{
	        // wait for correct available data length
	        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

	        // read a packet from FIFO

	        mpu.getFIFOBytes(fifoBuffer, packetSize);
	 		mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&mpu_raw_accel, fifoBuffer);
			mpu.dmpGetGyro(&mpu_gyro, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);

			mpu.dmpGetLinearAccel(&mpu_linear_accel, &mpu_raw_accel, &gravity);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			

			LPF_onLoop(&lpf_degree_x, (float) ypr[2] * 180.0f / M_PI);
			LPF_onLoop(&lpf_degree_y, (float) ypr[1] * 180.0f / M_PI);
			LPF_onLoop(&lpf_degree_z, (float) ypr[0] * 180.0f / M_PI);

			//LPF_onLoop(&lpf_accel_x, (float) mpu_linear_accel.x);
			//LPF_onLoop(&lpf_accel_y, (float) mpu_linear_accel.y);
			//LPF_onLoop(&lpf_accel_z, (float) mpu_linear_accel.z);

			LPF_onLoop(&lpf_gyro_x, (float) mpu_gyro.x);
			LPF_onLoop(&lpf_gyro_y, (float) mpu_gyro.y);
			LPF_onLoop(&lpf_gyro_z, (float) mpu_gyro.z);

			//printf("ROLL=%3.3f,", lpf_degree_x.output);
			//printf("PITCH=%3.3f,", lpf_degree_y.output);
			//printf("YAW=%3.3f,", lpf_degree_z.output);

			//printf("Accel_x=%3.3f,", lpf_accel_x.output);
			//printf("Accel_y=%3.3f,", lpf_accel_y.output);
			//printf("Accel_z=%3.3f,", lpf_accel_z.output);

			//printf("Gyro_x=%3.3f,", lpf_gyro_x.output);
			//printf("Gyro_y=%3.3f,", lpf_gyro_y.output);
			//printf("Gyro_z=%3.3f\r\n", lpf_gyro_z.output);

			mpu_master_info.mpu_valid_pack = 1;
			
			vTaskDelayUntil(&tick, 8 / portTICK_PERIOD_MS);
	    }
	}

	vTaskDelete(NULL);
}

void task_motion(void *pt)
{
	TickType_t tick = xTaskGetTickCount();

	stepper_command.motor_height = 0;
	stepper_command.motor_pitching = 0;
	stepper_command.motor_rotation = 0;

	while(1)
	{
		mpu_master_info.mpu_roll_deg = lpf_degree_x.output;
		mpu_master_info.mpu_pitch_deg = lpf_degree_y.output;
		mpu_master_info.mpu_yaw_deg = lpf_degree_z.output;

		mpu_master_info.mpu_x_gyro = lpf_gyro_x.output;
		mpu_master_info.mpu_y_gyro = lpf_gyro_y.output;
		mpu_master_info.mpu_z_gyro = lpf_gyro_z.output;

		vTaskDelayUntil(&tick, 50);

		// Pitching
		if(((mpu_master_info.mpu_roll_deg - lpf_degree_x.output) > 3.0f && (lpf_gyro_x.output < -30.0f)))
		{
			stepper_command.motor_pitching = 1;
		}
		else if(((mpu_master_info.mpu_roll_deg - lpf_degree_x.output) < -3.0f && (lpf_gyro_x.output > 30.0f)))
		{
			stepper_command.motor_pitching = -1;
		}
		else
		{
			stepper_command.motor_pitching = 0;
		}
		//printf("diff %.5f  gyro %.2f\n", mpu_master_info.mpu_roll_deg - lpf_degree_x.output, lpf_gyro_x.output);

		// Height
		if(((mpu_master_info.mpu_pitch_deg - lpf_degree_y.output) > 3.0f && (lpf_gyro_y.output > 30.0f)))
		{
			stepper_command.motor_height = 1;
		}
		else if(((mpu_master_info.mpu_pitch_deg - lpf_degree_y.output) < -3.0f && (lpf_gyro_y.output < -30.0f)))
		{
			stepper_command.motor_height = -1;
		}
		else
		{
			stepper_command.motor_height = 0;
		}

		// Rotation
		if(((mpu_master_info.mpu_yaw_deg - lpf_degree_z.output) > 3.0f && (lpf_gyro_z.output > 25.0f)))
		{
			stepper_command.motor_rotation = 1;
		}
		else if(((mpu_master_info.mpu_yaw_deg - lpf_degree_z.output) < -3.0f && (lpf_gyro_z.output < -25.0f)))
		{
			stepper_command.motor_rotation = -1;
		}
		else
		{
			stepper_command.motor_rotation = 0;
		}

		vTaskDelayUntil(&tick, 50);
	}
}

void LPF_Init(lpf_1p_t * lpf)
{
    lpf->rc =  (float) ((float) 1.0f / (2.0f * M_PI * LPF_CUTOFF_FREQ));

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
