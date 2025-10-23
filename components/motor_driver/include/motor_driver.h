#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Motor driver configuration structure
typedef struct {
    gpio_num_t dir_pin;        // Direction control pin
    gpio_num_t pwm_pin;        // PWM control pin
    gpio_num_t encoder_a_pin;  // Encoder channel A pin (optional)
    gpio_num_t encoder_b_pin;  // Encoder channel B pin (optional)
    ledc_channel_t pwm_channel; // PWM channel
    ledc_timer_t pwm_timer;    // PWM timer
    uint32_t pwm_frequency;    // PWM frequency in Hz
    uint32_t max_pwm_duty;     // Maximum PWM duty cycle
} motor_driver_config_t;

// Motor direction enumeration
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_BACKWARD = 1
} motor_direction_t;

// Motor driver handle
typedef struct {
    motor_driver_config_t config;
    int32_t encoder_count;
    bool encoder_enabled;
} motor_driver_handle_t;

/**
 * @brief Initialize motor driver
 * 
 * @param config Motor driver configuration
 * @param handle Pointer to motor driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_init(const motor_driver_config_t *config, motor_driver_handle_t *handle);

/**
 * @brief Set motor speed and direction
 * 
 * @param handle Motor driver handle
 * @param speed Speed value (-100 to 100, negative = reverse)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_set_speed(motor_driver_handle_t *handle, int8_t speed);

/**
 * @brief Set motor direction
 * 
 * @param handle Motor driver handle
 * @param direction Motor direction
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_set_direction(motor_driver_handle_t *handle, motor_direction_t direction);

/**
 * @brief Stop motor
 * 
 * @param handle Motor driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_stop(motor_driver_handle_t *handle);

/**
 * @brief Get encoder count
 * 
 * @param handle Motor driver handle
 * @return int32_t Encoder count
 */
int32_t motor_driver_get_encoder_count(motor_driver_handle_t *handle);

/**
 * @brief Reset encoder count
 * 
 * @param handle Motor driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_reset_encoder(motor_driver_handle_t *handle);

/**
 * @brief Deinitialize motor driver
 * 
 * @param handle Motor driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t motor_driver_deinit(motor_driver_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H



