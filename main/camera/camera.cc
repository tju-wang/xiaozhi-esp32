#include "camera.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "application.h"
// 处理流 
static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    // 发送这个图片http响应
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

esp_err_t jpg_httpd_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    auto& app = Application::GetInstance();
    if(app.camera_flag == false) {
        ESP_LOGE("Camera", "Camera can't use");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    bsp_camera_init();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    fb = esp_camera_fb_get(); // 第一帧的数据不使用
    esp_camera_fb_return(fb); // 处理结束以后把这部分的buf返回
    fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE("Camera", "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI("Camera", "Camera capture success");
    }
    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    if(res == ESP_OK){
            jpg_chunking_t jchunk = {req, 0}; // 输入输出参数
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
    }
    esp_camera_fb_return(fb); // 处理结束以后把这部分的buf返回
    esp_camera_deinit();
    return res;
}

httpd_uri_t uri_handler_jpg = {
    .uri = "/jpg",
    .method = HTTP_GET,
    .handler = jpg_httpd_handler};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    ESP_LOGI("Camera", "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // Set URI handlers
        ESP_LOGI("Camera", "Registering URI handlers");
        httpd_register_uri_handler(server, &uri_handler_jpg);
        return server;
    }

    ESP_LOGI("Camera", "Error starting server!");
    return NULL;
}

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};


void bsp_camera_init(void)
{
    // extern Pca9557* pca9557_;
    // pca9557_->SetOutputState(2, 0);

    // camera init
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE("Camera", "Camera init failed with error 0x%x", err);
        return;
    }

    // sensor_t *s = esp_camera_sensor_get(); // 获取摄像头型号

    // if (s->id.PID == GC0308_PID) {
    //     s->set_hmirror(s, 1);  // 这里控制摄像头镜像 写1镜像 写0不镜像
    // }
    ESP_LOGI("Camera", "Camera init successful");  
}




