/*
 */

#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "i2c.h"
#include "adxl345.h"


void adxl345_init(uint8_t i2c_master_port)
{
	printf("Manufacturer ID:        0x%02X\r\n",i2c_read_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_DEVID));
	i2c_write_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_POWER_CTL, 0b00001000);	// Enable measure mode

	// Enable threshold interrupt on Z axis
	i2c_write_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_THRESH_ACT, 4);			// Set threshold
	i2c_write_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_ACT_INACT_CTL, ACT_AC_COUPLED | ACT_Z_EN);	// Enable activity in Z axis
	i2c_write_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_INT_MAP, 0b00000000);		// All functions generate INT1
	i2c_write_byte(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_INT_ENABLE, ACTIVITY);		// Enable activity function
}

int16_t adxl345_read_x(uint8_t i2c_master_port)
{
	return (__bswap16(i2c_read_short(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_DATAX)));
}

int16_t adxl345_read_y(uint8_t i2c_master_port)
{
	return (__bswap16(i2c_read_short(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_DATAY)));
}

int16_t adxl345_read_z(uint8_t i2c_master_port)
{
	return (__bswap16(i2c_read_short(i2c_master_port, ADXL345_ALT_SLAVE_ADDRESS, ADXL345_DATAZ)));
}


float kalman_filtering(KalmanFilter *filter, float measurement)
{
    // Dự đoán
    float x_priori = filter->x;
    float P_priori = filter->P + filter->Q;

    // Cập nhật
    float K = P_priori / (P_priori + filter->R);
    float x_posteriori = x_priori + K * (measurement - x_priori);
    float P_posteriori = (1 - K) * P_priori;

    // Cập nhật trạng thái ước lượng và ma trận hiệp phương sai ước lượng
    filter->x = x_posteriori;
    filter->P = P_posteriori;

    return x_posteriori;
}

void kalman_init(KalmanFilter *filter, float initial_state, float initial_covariance, float process_noise, float measurement_noise)
{
    filter->x = initial_state;
    filter->P = initial_covariance;
    filter->Q = process_noise;
    filter->R = measurement_noise;
}

void print_memory_info() {
    printf("Free heap RAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    printf("Free stack RAM: %d bytes\n", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
    printf("Minimum free heap RAM: %lu bytes\n", esp_get_minimum_free_heap_size());
    printf("Free DRAM (D/IRAM) heap RAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    printf("Minimum free DRAM (D/IRAM) heap RAM: %d bytes\n", heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
    printf("Free IRAM heap RAM: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_32BIT));
    printf("Minimum free IRAM heap RAM: %d bytes\n", heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT));
}