#ifndef __STRUCT_COMPONENT_H__
#define __STRUCT_COMPONENT_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "stdio.h"
#include "stdint.h"
#include "math.h"



typedef struct
{
    uint8_t mpu_valid_pack;

    float mpu_roll_deg;
    int16_t mpu_x_accel;
    int16_t mpu_x_gyro;

    float mpu_pitch_deg;
    int16_t mpu_y_accel;
    int16_t mpu_y_gyro;

    float mpu_yaw_deg;
    int16_t mpu_z_accel;
    int16_t mpu_z_gyro;
} __attribute__((packed)) mpu_rotation_t;

typedef struct
{   
    uint8_t btnA;
    uint8_t btnB;
    uint8_t btnX;
    uint8_t btnY;
    uint8_t btnShare;                               // 上传/分享
    uint8_t btnStart;                               // 设置菜单
    uint8_t btnSelect;                              // 复制
    uint8_t btnXbox;                                // 主按键
    uint8_t btnLB;                                  // side top button
    uint8_t btnRB;                                  // side top button
    uint8_t btnLS;                                  // button on joy stick
    uint8_t btnRS;                                  // button on joy stick
    uint8_t btnDirUp;
    uint8_t btnDirLeft;
    uint8_t btnDirRight;
    uint8_t btnDirDown;
                                                    // 16 bytes

    uint16_t joyLHori;                              // 2 bytes
    uint16_t joyLVert;                              // 2 bytes
    uint16_t joyRHori;                              // 2 bytes
    uint16_t joyRVert;                              // 2 bytes
    uint16_t trigLT;                                // 2 bytes
    uint16_t trigRT;                                // 2 bytes
    // 12 bytes
} __attribute__((packed)) xbox_info_t;

typedef struct
{
    /* data */
    int8_t motor_pitching;
    int8_t motor_rotation;
    int8_t motor_height;
} __attribute__((packed)) stepper_command_t;



extern mpu_rotation_t mpu_master_info;

extern xbox_info_t xbox_info;

extern stepper_command_t stepper_command;

extern uint8_t xbox_status;

extern uint8_t mpu_dmp_status;
extern uint8_t mpu_dmp_calibration;

extern uint8_t camera_mode;

#ifdef __cplusplus
}
#endif

#endif

