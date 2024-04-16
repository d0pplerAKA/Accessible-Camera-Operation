#include "i2c_peripheral.h"

esp_err_t i2c_peripheral_init(i2c_port_t i2c_port, int32_t scl_gpio, int32_t sda_gpio, uint32_t clk_speed)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .scl_io_num = scl_gpio,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_io_num = sda_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,
        .clk_flags = 0
    };
    esp_err_t err;
    err = i2c_param_config(i2c_port, &i2c_cfg);
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK) return err;

    err = i2c_driver_install(i2c_port, i2c_cfg.mode, 0, 0, 0);
    ESP_ERROR_CHECK(err);
    if(err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t i2c_write_byte(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t data)
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    err = i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_WRITE, 1);
    err = i2c_master_write_byte(cmd, sl_reg, 1);
    err = i2c_master_write_byte(cmd, data, 1);
    err = i2c_master_stop(cmd);

    err = i2c_master_cmd_begin(i2c_port, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    if(err == ESP_OK) printf("ESP_OK: %s\n", esp_err_to_name(err));
    else printf("ERROR: %s\n", esp_err_to_name(err));

    return err;
}

esp_err_t i2c_write_bytes(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, sl_reg, 1);
    i2c_master_write(cmd, data, data_len, 1);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    //if(err == ESP_OK) printf("ESP_OK: %s\n", esp_err_to_name(err));
    //else printf("ERROR: %s\n", esp_err_to_name(err));

    return err;
}

esp_err_t i2c_read_byte(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data)
{
     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, sl_reg, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, data, 1);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port, cmd, 100);
    i2c_cmd_link_delete(cmd);

    if(err == ESP_OK) printf("ESP_OK: %s\n", esp_err_to_name(err));
    else printf("ERROR: %s\n", esp_err_to_name(err));

    return err;
}

esp_err_t i2c_read_bytes(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, sl_reg, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sl_addr << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_port, cmd, 1000);
    i2c_cmd_link_delete(cmd);

    //if(err == ESP_OK) printf("ESP_OK: %s\n", esp_err_to_name(err));
    //else printf("ERROR: %s\n", esp_err_to_name(err));

    return err;
}

esp_err_t i2c_write_to_device(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t data)
{   
    const uint8_t write_buf[2] = {sl_reg, data};

    esp_err_t err = i2c_master_write_to_device(i2c_port, sl_addr, write_buf, sizeof(write_buf), 1000);

    if(err != ESP_OK)
    {
        printf("Fail Sending %x to addr %x, slave %x\n", data, sl_reg, sl_addr);
    }

    return err;
}

esp_err_t i2c_read_from_device(uint8_t i2c_port, uint8_t sl_addr, uint8_t sl_reg, uint8_t *data, uint8_t data_len)
{
    esp_err_t err = i2c_master_write_read_device(i2c_port, sl_addr, &sl_reg, 1, data, data_len, 1000);

    if(err != ESP_OK)
    {
        printf("Fail reading data from addr %x, slave %x\n", sl_reg, sl_addr);
    }

    return err;
}
