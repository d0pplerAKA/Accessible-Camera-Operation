#ifndef __GY95T_REG_H__
#define __GY95T_REG_H__


#include "i2c_peripheral.h"
#include "math.h"

#define I2C_SCL_GPIO                    GPIO_NUM_9
#define I2C_SDA_GPIO                    GPIO_NUM_8
#define I2C_IRQ_GPIO                    GPIO_NUM_3
#define I2C_PS_GPIO                     GPIO_NUM_0

#define I2C_PORT_GY                     I2C_NUM_0
#define I2C_WriteByte                   i2c_write_byte
#define I2C_WriteBytes                  i2c_write_bytes
#define I2C_ReadByte                    i2c_read_byte
#define I2C_ReadBytes                   i2c_read_bytes


#define LPF_SAMPLING_RATE               200
#define LPF_SAMPLING_RATE_DT            ((float) (1.0f / (float)LPF_SAMPLING_RATE))
#define LPF_CUTOFF_FREQ                 35
#define LPF_PI                          M_PI
#define LPF_LFP_BUFFER                  4

#define delay_ms                        vTaskDelay


/*** GY95T Register ***/

#define GY95T_ADDR                      0x52
#define GY95T_ACC_X_L                   0x08
#define GY95T_ROLL_L                    0x14
#define GY95T_INFO                      0x2B

#define GY95T_UPDATE_RATE               0x02
#define GY95T_UPDATE_RATE_SET           0x03        //200Hz

#define GY95T_OUTPUT_MODE               0x03
#define GY95T_OUTPUT_MODE_IRQ           0x00        //持续模式输出
#define GY95T_OUTPUT_MODE_DEFAULT       0x01        //查询模式输出

#define GY95T_OUTPUT_FORMAT             0x04
#define GY96T_OUTPUT_FORMAT_HEX         0x00        //16进制输出格式
#define GY96T_OUTPUT_FORMAT_ASCII       0x01        //字符串输出格式

#define GY95T_SAVE_SETTING              0x05        
#define GY95T_CALIBRATION               0x06
#define GY95T_MODE_SET                  0x07


/*** 0x05 ***/
#define GY95T_SETTING_SAVE              0x55        //保存设置
#define GY95T_SETTING_FACT_RESET        0xAA        //恢复出厂设置
#define GY95T_SETTING_AUTO_CALIB        0x57        //自动校准
#define GY95T_SETTING_START_CALIB       0x58        //磁场校准开始
#define GY95T_SETTING_STOP_CALIB        0x59        //磁场校准结束
#define GY95T_SETTING_SAVE_CALIB        0x5A        //保存磁场校准数据

#define GY95T_FIFO_SIZE                 6

/*** GY95T Register End ***/

typedef struct
{
    /* data */
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;

    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    int16_t raw_roll;
    int16_t raw_pitch;
    int16_t raw_yaw;

    uint8_t raw_mfc; //magnetic field calibration

    int16_t raw_temp;

} __attribute__((packed)) mpu_gy95t_raw_data_t;


typedef struct
{
    float rc;
    float alpha;

    float output;

    /* data */
} lpf_1p_t;


#endif
