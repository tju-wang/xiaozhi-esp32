#include "i2c_device.h"
#include <cstring>

#include <esp_log.h>

#define TAG "I2cDevice"


I2cDevice::I2cDevice(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_));
    ESP_LOGI(TAG, "I2cDevice addr: %d  i2c_device_ %p", addr, i2c_device_);
    assert(i2c_device_ != NULL);
}

void I2cDevice::WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buffer, 2, 100));
}

uint8_t I2cDevice::ReadReg(uint8_t reg) {
    uint8_t buffer[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, 1, 100));
    return buffer[0];
}

void I2cDevice::ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    // ESP_LOGI(TAG, "[wx] WriteReg i2c_device_ %p reg %d data[0] %d data[1] %d len %d", i2c_device_, reg, buffer[0], buffer[1], length);
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, 100));
}

void I2cDevice::WriteRegs(uint8_t reg, const uint8_t* data, size_t len) {
    uint8_t* buffer = new uint8_t[len + 1];
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);
    // ESP_LOGI(TAG, "[wx] WriteReg i2c_device_ %p reg %d data[0] %d data[1] %d len %d", i2c_device_, reg, buffer[0], buffer[1], len);
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buffer, len + 1, 100));
    delete[] buffer;
}