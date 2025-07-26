#include "ultrasound.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "Ultrasound";

Ultrasound::~Ultrasound() {
}

Ultrasound::Ultrasound(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
    : I2cDevice(i2c_bus, addr) {}  // 显式调用基类构造函数

// 修改所有为基类方法调用
void Ultrasound::setBreathingMode(uint8_t r1, uint8_t g1, uint8_t b1,
                                 uint8_t r2, uint8_t g2, uint8_t b2) {
    WriteReg(RGB_WORK_MODE, RGB_WORK_BREATHING_MODE);
    uint8_t params[6] = {r1, g1, b1, r2, g2, b2};
    WriteRegs(RGB1_R_BREATHING_CYCLE, params, 6);
}

void Ultrasound::setSolidColor(uint8_t r1, uint8_t g1, uint8_t b1, 
                              uint8_t r2, uint8_t g2, uint8_t b2) {
    WriteReg(RGB_WORK_MODE, RGB_WORK_SIMPLE_MODE);

    uint8_t colors[6] = {r1, g1, b1, r2, g2, b2};
    WriteRegs(RGB1_R, colors, 6);
}

uint16_t Ultrasound::getDistance() {
    uint8_t data[2] = {0};

    data[0] = ReadReg(0x00);
    data[1] = ReadReg(0x01);

    // ESP_LOGI(TAG, "[wx] data %d %d", data[0], data[1]);
    return (data[1] << 8) | data[0];
}