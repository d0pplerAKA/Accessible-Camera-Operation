# README

### GNG2101 G2.4-OpenUV, 2023 Winter, University of Ottawa

##### *This is an esp-32 project running in the esp-idf platform. The programming language is C/C++. It implements the 360-Camera Rotation System for the user who is not able to control the camera themselves. The user can caontrol the system by using motion detector, Xbox Controller(Caregiver Mode), and Computer Vision.*

---

- ##### *Used External Library*
    - *[esp-idf - Apache-2.0](https://github.com/espressif/esp-idf)*
    - *[arduino-esp32 - LGPL-2.1](https://github.com/espressif/arduino-esp32)*
    - *[XboxSeriesXControllerESP32 - MIT](https://github.com/asukiaaa/arduino-XboxSeriesXControllerESP32)*
    - *[libfacedectection - BSD](https://github.com/ShiqiYu/libfacedetection)*
    - *[MPU6050 & I2Cdev](https://github.com/fcayci/mpu6050/tree/master)*

---

### How to Run?

1. Environmental preparation
    1. esp-idf version limit: from `v4.4` up to `v4.4.7`.
    2. Make sure your ESP32 series moudule support `BLE` feature. Possible build target: ESP32 / ESP32-C3 / ESP32-S3
    3. Recommend to use *vscode + esp-idf* , the integrated environment.

2. Configuration - Adjust settings in `menuconfig`.
Set the corresponding ESP32 series moudule.
    1. Open the EDP-IDF Terminal and run `idf.py menuconfig`.
    2. Set the Flash Size if needed. *`{(Top) → Serial flasher config}`*
    3. Set FreeRTOS Hertz == `1000` in component `FreeRTOS`. *`{(Top) → Component config → FreeRTOS → Tick rate (Hz) == 1000}`*
    4. Enable component `Bluetooth`. *`{(Top) → Component config → Bluetooth}`*
    5. Enable external `TLS Key Exchange`  method in component `mbedsTLS`. *`{(Top) → Component config → mbedTLS → TLS Key Exchange Methods → Enable pre-shared-key ciphersuites}`*
    6. Optionally, change the CPU clock rate of the ESP32 series moudule. *`{(Top) → Component config → ESP32-specific → CPU frequency}`*

3. Build the project
    1. Click *ESP-IDF: Build Project* to build the project.
    2. Errors and Warnings may happen during this stage, if the console prints output related to `ninja error` description, clean Click *ESP-IDF: Full Clean* and check the configuration, run `menuconfig` edit the settings, then Try again.
4. Click *ESP-IDF: Flash Device* to flash the code into the ESP32 series moudule.

5. Using Python3 Script for testing CV
    1. Run the script
    2. Currently, CV commands are transfering with USB-Serial, after open the UART with 115200 baudrates, insert 'camera\r\n' and send to the Main Control Board
    3. Click *Send Order*, an indicator window with CV scratch will pop of to shows CV status.



#### This is a University Course Project! Everything is under experimental condition.
- #####     *Acknowledgement - Thanks to the team members for their help*
- `- Xiaoyi Fu - 2nd Electrical Engineering student, as Report Writer & 3D structure designer`
- `- Hasan Jaber - 2nd Civil Engineering student, as Report Writer & Planner`
- #####     *Especially Acknowledgement*
- `- Anjali - Course TA, Firmly support the advancement of the project!`




    
