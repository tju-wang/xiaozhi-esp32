#ifndef CAMERA_H
#define CAMERA_H

#include "esp_camera.h"
#include "esp_http_server.h"
#include "i2c_device.h"
#define BOARD_ESP32S3_WROOM 1

/***********************************************************/
/****************    camera ↓   ****************************/


// ESP32S3 (WROOM) PIN Map
#ifdef BOARD_ESP32S3_WROOM
#define CAM_PIN_PWDN 38
#define CAM_PIN_RESET -1   //software reset will be performed
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D0 11
#define CAM_PIN_D1 9
#define CAM_PIN_D2 8
#define CAM_PIN_D3 10
#define CAM_PIN_D4 12
#define CAM_PIN_D5 18
#define CAM_PIN_D6 17
#define CAM_PIN_D7 16
#endif

// #define CAMERA_PIN_PWDN -1
// #define CAMERA_PIN_RESET -1
// #define CAMERA_PIN_XCLK 5
// #define CAMERA_PIN_SIOD 1
// #define CAMERA_PIN_SIOC 2

// #define CAMERA_PIN_D7 9
// #define CAMERA_PIN_D6 4
// #define CAMERA_PIN_D5 6
// #define CAMERA_PIN_D4 15
// #define CAMERA_PIN_D3 17
// #define CAMERA_PIN_D2 8
// #define CAMERA_PIN_D1 18
// #define CAMERA_PIN_D0 16
// #define CAMERA_PIN_VSYNC 3
// #define CAMERA_PIN_HREF 46
// #define CAMERA_PIN_PCLK 7

#define XCLK_FREQ_HZ 24000000

void bsp_camera_init(void);
// class Pca9557 : public I2cDevice {
// public:
//     Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
//         WriteReg(0x01, 0x03);
//         WriteReg(0x03, 0xf8);
//     };

//     void SetOutputState(uint8_t bit, uint8_t level) {
//         uint8_t data = ReadReg(0x01);
//         data = (data & ~(1 << bit)) | (level << bit);
//         WriteReg(0x01, data);
//     }
// };

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

httpd_handle_t start_webserver(void);
/********************    摄像头 ↑   *************************/
/***********************************************************/

#endif