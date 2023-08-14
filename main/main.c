#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_netif.h"
#include "i2c.h"
#include "adxl345.h"
#include "lwip/sockets.h"
#include "esp_heap_caps.h"

#define SAMPLE_INTERVAL_SECONDS 1  // Khoảng thời gian giữa hai lần tính tần số lấy mẫu

// Define kích thước gói tin
#define PACKET_SIZE 1200
#define SAMPLE_SIZE 160
#define SAMPLE_BYTES 12


void app_main(void)
{
    //wifi_station_init();
 
    i2c_init(I2C_CONTROLLER_1, 21, 22);
    adxl345_init(I2C_CONTROLLER_1);

    uint32_t samples = 0;
    uint32_t packages = 0;  // Biến lưu số lần lấy mẫu
    //TickType_t last_time = xTaskGetTickCount();  // Thời gian khi lần lấy mẫu cuối cùng được thực hiện

    // Khởi tạo bộ lọc Kalman cho mỗi trục
    KalmanFilter kalman_x;
    kalman_init(&kalman_x, 0, 1, 0.1, 5);

    KalmanFilter kalman_y;
    kalman_init(&kalman_y, 0, 1, 0.1, 5);

    KalmanFilter kalman_z;
    kalman_init(&kalman_z, 0, 1, 0.1, 5);

    // Tạo gói tin dữ liệu
    uint8_t packet[PACKET_SIZE];
    uint8_t* packet_ptr = packet;
    
    while (true) {
        // Lấy giá trị mới từ cảm biến
        int16_t raw_x = adxl345_read_x(I2C_CONTROLLER_1);
        int16_t raw_y = adxl345_read_y(I2C_CONTROLLER_1);
        int16_t raw_z = adxl345_read_z(I2C_CONTROLLER_1);

        // Lọc nhiễu bằng bộ lọc Kalman cho mỗi trục
        float filtered_x = kalman_filtering(&kalman_x, (float)raw_x);
        float filtered_y = kalman_filtering(&kalman_y, (float)raw_y);
        float filtered_z = kalman_filtering(&kalman_z, (float)raw_z);

        samples++;

        // Chuyển đổi giá trị sang byte và thêm vào gói tin dữ liệu
        memcpy(packet_ptr, &filtered_x, sizeof(float));
        packet_ptr += sizeof(float);
        memcpy(packet_ptr, &filtered_y, sizeof(float));
        packet_ptr += sizeof(float);
        memcpy(packet_ptr, &filtered_z, sizeof(float));
        packet_ptr += sizeof(float);

        if (samples >= SAMPLE_SIZE) {
            // Gửi gói tin dữ liệu tới QT Creator thông qua TCP WiFi
            //send_data_to_qt_creator(packet, SAMPLE_SIZE * SAMPLE_BYTES);
            samples = 0;
            packet_ptr = packet;
            packages++;
            printf("Total packages sent: %lu\n", packages);
        }

        // In giá trị đã lọc
        printf("X Axis = %.2f, Y Axis = %.2f, Z Axis = %.2f\r\n", filtered_x, filtered_y, filtered_z);
        printf("X Axis Fake = %2d, Y Axis Fake = %2d, Z Axis Fake = %2d\r\n", raw_x, raw_y, raw_z);
        print_memory_info();
        
        
        vTaskDelay(10);
    }
}