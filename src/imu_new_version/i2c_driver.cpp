#include "i2c_driver.h"

gpio_num_t i2c_sda_port = GPIO_NUM_21;
gpio_num_t i2c_scl_port = GPIO_NUM_22;
uint32_t timeout = 1000 / portTICK_PERIOD_MS;

i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = -1,
    .sda_io_num = i2c_sda_port,
    .scl_io_num = i2c_scl_port,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
};

i2c_master_bus_handle_t i2c_bus_handle = NULL;

void i2c_initialize(const i2c_device_config_t *dev_config, i2c_master_dev_handle_t *ret_handle)
{
    if(i2c_bus_handle == NULL)
    {
        esp_err_t bus_result = i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle);
        log_i("Init I2C bus with code: %s", esp_err_to_name(bus_result));
    }

    esp_err_t device_result = i2c_master_bus_add_device(i2c_bus_handle, dev_config, ret_handle);

    log_i("Add device to I2C bus with code %s", esp_err_to_name(device_result));
}

void i2c_write_register(uint8_t reg, uint8_t data, i2c_master_dev_handle_t *ret_handle)
{
    const uint8_t buff[2] = {reg, data};
    esp_err_t i2c_result = i2c_master_transmit(*ret_handle, buff, sizeof(buff), timeout);
    if(i2c_result)
        log_e("I2C transmit failed with code: %s", esp_err_to_name(i2c_result));
}

void i2c_write_register_with_diff_buff(uint8_t reg, uint8_t data[2], i2c_master_dev_handle_t *ret_handle)
{
    uint8_t buff[3] = {reg, data[0], data[1]};
    esp_err_t i2c_result = i2c_master_transmit(*ret_handle, buff, sizeof(buff), timeout);
    if(i2c_result)
        log_e("I2C transmit failed with code: %s", esp_err_to_name(i2c_result));
}

void i2c_write_register_with_specific_size(uint8_t reg, uint8_t data, i2c_master_dev_handle_t *ret_handle, uint32_t len)
{
    uint8_t buff[2] = {reg, data};
    esp_err_t i2c_result = i2c_master_transmit(*ret_handle, buff, len, timeout);
    if(i2c_result)
        log_e("I2C transmit failed with code: %s", esp_err_to_name(i2c_result));
}

void i2c_read_register(uint8_t reg, uint8_t *data, i2c_master_dev_handle_t *ret_handle)
{
    esp_err_t i2c_result = i2c_master_transmit_receive(*ret_handle, &reg, sizeof(reg), data, sizeof(*data), timeout);
    if(i2c_result)
        log_e("I2C read failed with code: %s", esp_err_to_name(i2c_result));
}

esp_err_t i2c_device_is_connected(uint16_t address)
{
    return i2c_master_probe(i2c_bus_handle, address, timeout);
}