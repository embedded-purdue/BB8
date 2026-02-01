#ifndef OMNIWHEEL_MOTOR_SYSTEM_H
#define OMNIWHEEL_MOTOR_SYSTEM_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Number of motors in the omniwheel system
#define OMNIWHEEL_NUM_MOTORS 4

// Motor positions in diamond configuration for BB8 omniwheel system
// Motor 0: Front-Right (A) - 45 degrees, formula: (y + x + r)
// Motor 1: Front-Left (B) - 135 degrees, formula: (y - x + r) [FIXED: all motors use +r for rotation]
// Motor 2: Back-Left (C) - 225 degrees, formula: (-y - x + r)
// Motor 3: Back-Right (D) - 315 degrees, formula: (-y + x + r)
typedef enum {
    OMNIWHEEL_MOTOR_FRONT_RIGHT = 0,  // 45° - Front-right (A), formula: (y + x + r)
    OMNIWHEEL_MOTOR_FRONT_LEFT  = 1,  // 135° - Front-left (B), formula: (y - x + r)
    OMNIWHEEL_MOTOR_BACK_LEFT   = 2,  // 225° - Back-left (C), formula: (-y - x + r)
    OMNIWHEEL_MOTOR_BACK_RIGHT  = 3   // 315° - Back-right (D), formula: (-y + x + r)
} omniwheel_motor_position_t;

// Motor configuration for each wheel
typedef struct {
    gpio_num_t dir_pin;
    gpio_num_t pwm_pin;
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    ledc_channel_t pwm_channel;
    ledc_timer_t pwm_timer;
    bool enabled;  // Set to false if motor not yet connected
} omniwheel_motor_config_t;

// Omniwheel system configuration
typedef struct {
    omniwheel_motor_config_t motors[OMNIWHEEL_NUM_MOTORS];
    uint32_t pwm_frequency;
    uint32_t max_pwm_duty;
} omniwheel_system_config_t;

// Omniwheel system handle
typedef struct {
    omniwheel_system_config_t config;
    motor_driver_handle_t motor_handles[OMNIWHEEL_NUM_MOTORS];
    bool motors_initialized[OMNIWHEEL_NUM_MOTORS];
} omniwheel_system_handle_t;

/**
 * @brief Initialize omniwheel motor system
 * 
 * @param config System configuration
 * @param handle System handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t omniwheel_system_init(const omniwheel_system_config_t *config, omniwheel_system_handle_t *handle);

/**
 * @brief Set omniwheel movement (X, Y, rotation)
 * 
 * @param handle System handle
 * @param vx Horizontal velocity (-100 to +100, positive = right)
 * @param vy Vertical velocity (-100 to +100, positive = forward)
 * @param vr Rotation velocity (-100 to +100, positive = clockwise)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t omniwheel_system_set_velocity(omniwheel_system_handle_t *handle, int8_t vx, int8_t vy, int8_t vr);

/**
 * @brief Stop all motors
 * 
 * @param handle System handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t omniwheel_system_stop(omniwheel_system_handle_t *handle);

/**
 * @brief Deinitialize omniwheel system
 * 
 * @param handle System handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t omniwheel_system_deinit(omniwheel_system_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // OMNIWHEEL_MOTOR_SYSTEM_H

