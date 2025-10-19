#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// BNO055 I2C addresses
#define BNO055_ADDR_A 0x28
#define BNO055_ADDR_B 0x29

// BNO055 Register addresses
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0x06

// Page id register definition
#define BNO055_PAGE_ID_ADDR 0x07

// Accel data register
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D

// Mag data register
#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0x13

// Gyro data registers
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19

// Euler data registers
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B
#define BNO055_EULER_R_LSB_ADDR 0x1C
#define BNO055_EULER_R_MSB_ADDR 0x1D
#define BNO055_EULER_P_LSB_ADDR 0x1E
#define BNO055_EULER_P_MSB_ADDR 0x1F

// Quaternion data registers
#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27

// Temperature data register
#define BNO055_TEMP_ADDR 0x34

// Status registers
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_SELFTEST_RESULT_ADDR 0x36
#define BNO055_INTR_STAT_ADDR 0x37

// System status registers
#define BNO055_SYS_CLK_STAT_ADDR 0x38
#define BNO055_SYS_STAT_ADDR 0x39
#define BNO055_SYS_ERR_ADDR 0x3A

// Unit selection register
#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_DATA_SELECT_ADDR 0x3C

// Mode registers
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_TEMP_SOURCE_ADDR 0x40

// Axis remap registers
#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42

// SIC registers
#define BNO055_SIC_MATRIX_0_LSB_ADDR 0x43
#define BNO055_SIC_MATRIX_0_MSB_ADDR 0x44
#define BNO055_SIC_MATRIX_1_LSB_ADDR 0x45
#define BNO055_SIC_MATRIX_1_MSB_ADDR 0x46
#define BNO055_SIC_MATRIX_2_LSB_ADDR 0x47
#define BNO055_SIC_MATRIX_2_MSB_ADDR 0x48
#define BNO055_SIC_MATRIX_3_LSB_ADDR 0x49
#define BNO055_SIC_MATRIX_3_MSB_ADDR 0x4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR 0x4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR 0x4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR 0x4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR 0x4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR 0x4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR 0x50
#define BNO055_SIC_MATRIX_7_LSB_ADDR 0x51
#define BNO055_SIC_MATRIX_7_MSB_ADDR 0x52
#define BNO055_SIC_MATRIX_8_LSB_ADDR 0x53
#define BNO055_SIC_MATRIX_8_MSB_ADDR 0x54

// Accelerometer Offset registers
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR 0x55
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR 0x56
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR 0x57
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR 0x58
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR 0x59
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR 0x5A

// Magnetometer Offset registers
#define BNO055_MAG_OFFSET_X_LSB_ADDR 0x5B
#define BNO055_MAG_OFFSET_X_MSB_ADDR 0x5C
#define BNO055_MAG_OFFSET_Y_LSB_ADDR 0x5D
#define BNO055_MAG_OFFSET_Y_MSB_ADDR 0x5E
#define BNO055_MAG_OFFSET_Z_LSB_ADDR 0x5F
#define BNO055_MAG_OFFSET_Z_MSB_ADDR 0x60

// Gyroscope Offset registers
#define BNO055_GYRO_OFFSET_X_LSB_ADDR 0x61
#define BNO055_GYRO_OFFSET_X_MSB_ADDR 0x62
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR 0x63
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR 0x64
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR 0x65
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR 0x66

// Radius registers
#define BNO055_ACCEL_RADIUS_LSB_ADDR 0x67
#define BNO055_ACCEL_RADIUS_MSB_ADDR 0x68
#define BNO055_MAG_RADIUS_LSB_ADDR 0x69
#define BNO055_MAG_RADIUS_MSB_ADDR 0x6A

// BNO055 ID
#define BNO055_ID 0xA0

// Power modes
typedef enum {
    BNO055_POWER_MODE_NORMAL = 0x00,
    BNO055_POWER_MODE_LOWPOWER = 0x01,
    BNO055_POWER_MODE_SUSPEND = 0x02
} bno055_powermode_t;

// Operation modes
typedef enum {
    BNO055_OPERATION_MODE_CONFIG = 0x00,
    BNO055_OPERATION_MODE_ACCONLY = 0x01,
    BNO055_OPERATION_MODE_MAGONLY = 0x02,
    BNO055_OPERATION_MODE_GYRONLY = 0x03,
    BNO055_OPERATION_MODE_ACCMAG = 0x04,
    BNO055_OPERATION_MODE_ACCGYRO = 0x05,
    BNO055_OPERATION_MODE_MAGGYRO = 0x06,
    BNO055_OPERATION_MODE_AMG = 0x07,
    BNO055_OPERATION_MODE_IMUPLUS = 0x08,
    BNO055_OPERATION_MODE_COMPASS = 0x09,
    BNO055_OPERATION_MODE_M4G = 0x0A,
    BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPERATION_MODE_NDOF = 0x0C
} bno055_opmode_t;

// Data structure for BNO055 sample
typedef struct {
    uint32_t t_ms;
    // Raw sensor data
    float ax, ay, az;  // Accelerometer (m/s²)
    float gx, gy, gz;  // Gyroscope (rad/s)
    float mx, my, mz;  // Magnetometer (μT)
    // Fused data
    float roll, pitch, yaw;  // Euler angles (degrees)
    float qw, qx, qy, qz;    // Quaternion
    float temp;              // Temperature (°C)
    // Calibration status
    uint8_t sys_cal, gyro_cal, accel_cal, mag_cal;
} bno055_sample_t;

// Function prototypes
esp_err_t bno055_init(int port, uint8_t addr);
esp_err_t bno055_set_operation_mode(int port, uint8_t addr, bno055_opmode_t mode);
esp_err_t bno055_get_calibration_status(int port, uint8_t addr, uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
esp_err_t bno055_read_sample(int port, uint8_t addr, bno055_sample_t *out);
esp_err_t bno055_reset(int port, uint8_t addr);
esp_err_t bno055_start_calibration(int port, uint8_t addr);
esp_err_t bno055_save_calibration_data(int port, uint8_t addr, uint8_t *cal_data);
esp_err_t bno055_load_calibration_data(int port, uint8_t addr, const uint8_t *cal_data);

#ifdef __cplusplus
}
#endif
