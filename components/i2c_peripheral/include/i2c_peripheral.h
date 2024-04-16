#ifndef __I2C_PERIPHERAL_H__
#define __I2C_PERIPHERAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c.h"

esp_err_t i2c_peripheral_init(i2c_port_t i2c_port, int32_t scl_gpio, int32_t sda_gpio, uint32_t clk_speed);
esp_err_t i2c_write_byte(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t data);
esp_err_t i2c_write_bytes(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len);
esp_err_t i2c_read_byte(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data);
esp_err_t i2c_read_bytes(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len);

esp_err_t i2c_write_to_device(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t data);
esp_err_t i2c_read_from_device(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len);

#ifdef __cplusplus
}
#endif

#endif

