
#ifndef MAIN_ADXL345_H_
#define MAIN_ADXL345_H_

#define ADXL345_SLAVE_ADDRESS			0x1D
#define ADXL345_ALT_SLAVE_ADDRESS		0x53

#define ADXL345_DEVID				0x00
#define ADXL345_THRESH_TAP			0x1D
#define ADXL345_OFSX				0x1E
#define ADXL345_OFSY				0x1F
#define ADXL345_OFSZ				0x20
#define ADXL345_DUR				0x21
#define ADXL345_LATENT				0x22
#define ADXL345_WINDOW				0x23
#define ADXL345_THRESH_ACT			0x24
#define ADXL345_TIME_INACT			0x25
#define ADXL345_THRESH_INACT			0x26
#define ADXL345_ACT_INACT_CTL			0x27
#define ADXL345_THRESH_FF			0x28
#define ADXL345_TIME_FF				0x29
#define ADXL345_TAP_AXES			0x2A
#define ADXL345_ACT_TAP_STATUS			0x2B
#define ADXL345_BW_RATE				0x2C
#define ADXL345_POWER_CTL			0x2D
#define ADXL345_INT_ENABLE			0x2E
#define ADXL345_INT_MAP				0x2F
#define ADXL345_INT_SOURCE			0x30
#define ADXL345_DATAFORMAT			0x31
#define ADXL345_DATAX				0x32
#define ADXL345_DATAY				0x34
#define ADXL345_DATAZ				0x36
#define ADXL345_FIFO_CTL			0x38
#define ADXL345_FIFO_STATUS			0x39

// Bit flags for INT_ENABLE, INT_MAP and INT_SOURCE
#define DATA_READY				0x80
#define SINGLE_TAP				0x40
#define DOUBLE_TAP				0x20
#define ACTIVITY				0x10
#define INACTIVITY 				0x08
#define FREE_FALL				0x04
#define WATERMARK				0x02
#define OVERRUN					0x01

// Bit flags for ACT_INACT_CTL
#define ACT_AC_COUPLED				0x80
#define ACT_X_EN				0x40
#define ACT_Y_EN				0x20
#define ACT_Z_EN				0x10
#define INACT_AC_COUPLED			0x08
#define INACT_X_EN				0x04
#define INACT_Y_EN				0x02
#define INACT_Z_EN				0x01

void adxl345_init(uint8_t i2c_master_port);
int16_t adxl345_read_x(uint8_t i2c_master_port);
int16_t adxl345_read_y(uint8_t i2c_master_port);
int16_t adxl345_read_z(uint8_t i2c_master_port);

/*
Declare KalmanFilter function and its params
 */
typedef struct {
    float x;     // Trạng thái ước lượng => mức độ ước lượng
    float P;     // Ma trận hiệp phương sai ước lượng => mức độ ko chắc chắn của ước lượng
    float Q;     // Ma trận hiệp phương sai tiêu chuẩn Q => mức độ ko chắc chắn trong quá trình chuyển đổi
    float R;     // Ma trận hiệp phương sai đo lường R => mức độ ko chắc chắn của dữ liệu đo lường
} KalmanFilter;

void kalman_init(KalmanFilter *filter, float initial_state, float initial_covariance, float process_noise, float measurement_noise);
float kalman_filtering(KalmanFilter *filter, float measurement);
void print_memory_info();
#endif
