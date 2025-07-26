#pragma once
#include "driver/i2c.h"
#include "i2c_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ULTRASOUND_I2C_ADDR 0x77 

//寄存器
#define DISDENCE_L    0//距离低8位，单位mm
#define DISDENCE_H    1

#define RGB_BRIGHTNESS  50//0-255

#define RGB_WORK_MODE 2//RGB灯模式，0：用户自定义模式   1：呼吸灯模式  默认0

#define RGB1_R      3//1号探头的R值，0~255，默认0
#define RGB1_G      4//默认0
#define RGB1_B      5//默认255

#define RGB2_R      6//2号探头的R值，0~255，默认0
#define RGB2_G      7//默认0
#define RGB2_B      8//默认255

#define RGB1_R_BREATHING_CYCLE      9 //呼吸灯模式时，1号探头的R的呼吸周期，单位100ms 默认0，
                                      //如果设置周期3000ms，则此值为30
#define RGB1_G_BREATHING_CYCLE      10
#define RGB1_B_BREATHING_CYCLE      11

#define RGB2_R_BREATHING_CYCLE      12//2号探头
#define RGB2_G_BREATHING_CYCLE      13
#define RGB2_B_BREATHING_CYCLE      14

#define RGB_WORK_SIMPLE_MODE    0
#define RGB_WORK_BREATHING_MODE   1

class Ultrasound : public I2cDevice {
public:
    Ultrasound(i2c_master_bus_handle_t i2c_bus, uint8_t addr = ULTRASOUND_I2C_ADDR);
    
    ~Ultrasound();
    
    bool init();                          // 初始化传感器
    void setBreathingMode(uint8_t r1, uint8_t g1, uint8_t b1, 
                          uint8_t r2, uint8_t g2, uint8_t b2); // 呼吸灯模式
    void setSolidColor(uint8_t r1, uint8_t g1, uint8_t b1, 
                       uint8_t r2, uint8_t g2, uint8_t b2);   // 固定颜色
    uint16_t getDistance();               // 获取距离（毫米）


private:
    uint8_t devAddr; // 该成员可移至基类
};