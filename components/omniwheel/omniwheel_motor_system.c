#include "omniwheel_motor_system.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "OMNIWHEEL";

// Diamond configuration notes for BB8 omniwheel system (angles for physical placement)
// A: 45°, B: 135°, C: 225°, D: 315° — all motors contribute to rotation (+r)

esp_err_t omniwheel_system_init(const omniwheel_system_config_t *config, omniwheel_system_handle_t *handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing omniwheel motor system");

    // Copy configuration
    memcpy(&handle->config, config, sizeof(omniwheel_system_config_t));
    memset(handle->motors_initialized, 0, sizeof(handle->motors_initialized));

    // Initialize each motor that is enabled
    for (int i = 0; i < OMNIWHEEL_NUM_MOTORS; i++) {
        if (!config->motors[i].enabled) {
            ESP_LOGI(TAG, "Motor %d is disabled (not connected)", i);
            handle->motors_initialized[i] = false;
            continue;
        }

        // Configure motor driver
        motor_driver_config_t motor_config = {
            .dir_pin = config->motors[i].dir_pin,
            .pwm_pin = config->motors[i].pwm_pin,
            .encoder_a_pin = config->motors[i].encoder_a_pin,
            .encoder_b_pin = config->motors[i].encoder_b_pin,
            .pwm_channel = config->motors[i].pwm_channel,
            .pwm_timer = config->motors[i].pwm_timer,
            .pwm_frequency = config->pwm_frequency,
            .max_pwm_duty = config->max_pwm_duty
        };

        esp_err_t ret = motor_driver_init(&motor_config, &handle->motor_handles[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize motor %d: %s", i, esp_err_to_name(ret));
            handle->motors_initialized[i] = false;
            continue;
        }

        // Stop motor initially
        motor_driver_stop(&handle->motor_handles[i]);
        handle->motors_initialized[i] = true;

        const char* motor_names[] = {"FRONT-RIGHT (A)", "FRONT-LEFT (B)", "BACK-LEFT (C)", "BACK-RIGHT (D)"};
        ESP_LOGI(TAG, "Motor %d (%s) initialized - dir_pin=%d, pwm_pin=%d, channel=%d", 
                 i, motor_names[i], config->motors[i].dir_pin, config->motors[i].pwm_pin, 
                 config->motors[i].pwm_channel);
    }

    ESP_LOGI(TAG, "Omniwheel system initialized");
    return ESP_OK;
}

esp_err_t omniwheel_system_set_velocity(omniwheel_system_handle_t *handle, int8_t vx, int8_t vy, int8_t vr) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp velocities to valid range
    if (vx > 100) vx = 100;
    if (vx < -100) vx = -100;
    if (vy > 100) vy = 100;
    if (vy < -100) vy = -100;
    if (vr > 100) vr = 100;
    if (vr < -100) vr = -100;

    // Calculate motor speeds using omniwheel kinematics for diamond configuration
    // For BB8 omniwheel system with diamond configuration:
    // All motors contribute to rotation with the same sign for proper rotation
    // Motor 0 (A, Front-Right, 45°):  speed = vy + vx + vr
    // Motor 1 (B, Front-Left, 135°): speed = vy - vx + vr  [FIXED: was -vr, now +vr]
    // Motor 2 (C, Back-Left, 225°):   speed = -vy - vx + vr
    // Motor 3 (D, Back-Right, 315°): speed = -vy + vx + vr
    //
    // This ensures:
    // - Pure forward (vy>0, vr=0): A=vy, B=vy, C=-vy, D=-vy (front forward, back backward)
    // - Pure rotation (vy=0, vr>0): A=vr, B=vr, C=vr, D=vr (all same direction = rotation)
    // - Combined: All motors contribute properly without cancellation
    //
    // For 45° and 135° motors, they contribute to both forward (y) and strafe (x)
    // Using trigonometry: cos(45°) = sin(45°) = 0.707, but for simplicity we use 1:1 ratio
    
    int8_t motor_speeds[OMNIWHEEL_NUM_MOTORS];
    
    // Motor A (Front-Right, 45°): contributes to forward (y), right strafe (x), rotation (r)
    motor_speeds[OMNIWHEEL_MOTOR_FRONT_RIGHT] = vy + vx + vr;
    
    // Motor B (Front-Left, 135°): contributes to forward (y), left strafe (-x), rotation (r)
    // FIXED: Changed from -vr to +vr so all motors contribute to rotation
    motor_speeds[OMNIWHEEL_MOTOR_FRONT_LEFT] = vy - vx + vr;
    
    // Motor C (Back-Left, 225°): contributes to backward (-y), left strafe (-x), rotation (r)
    motor_speeds[OMNIWHEEL_MOTOR_BACK_LEFT] = -vy - vx + vr;
    
    // Motor D (Back-Right, 315°): contributes to backward (-y), right strafe (x), rotation (r)
    motor_speeds[OMNIWHEEL_MOTOR_BACK_RIGHT] = -vy + vx + vr;

    // Clamp each motor speed to valid range
    for (int i = 0; i < OMNIWHEEL_NUM_MOTORS; i++) {
        if (motor_speeds[i] > 100) motor_speeds[i] = 100;
        if (motor_speeds[i] < -100) motor_speeds[i] = -100;
    }

    // Apply speeds to each initialized motor
    for (int i = 0; i < OMNIWHEEL_NUM_MOTORS; i++) {
        if (!handle->motors_initialized[i]) {
            continue;  // Skip disabled motors
        }

        esp_err_t ret;
        if (motor_speeds[i] == 0) {
            ret = motor_driver_stop(&handle->motor_handles[i]);
        } else {
            ret = motor_driver_set_speed(&handle->motor_handles[i], motor_speeds[i]);
        }

        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set motor %d speed: %s", i, esp_err_to_name(ret));
        }
    }

    // Log movement (only when significant change)
    static int8_t last_vx = 0, last_vy = 0, last_vr = 0;
    if (vx != last_vx || vy != last_vy || vr != last_vr) {
        ESP_LOGI(TAG, "Velocity: vx=%d, vy=%d, vr=%d | Motors: [%d, %d, %d, %d]",
                 vx, vy, vr,
                 motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
        last_vx = vx;
        last_vy = vy;
        last_vr = vr;
    }

    return ESP_OK;
}

esp_err_t omniwheel_system_stop(omniwheel_system_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Stop all initialized motors
    for (int i = 0; i < OMNIWHEEL_NUM_MOTORS; i++) {
        if (handle->motors_initialized[i]) {
            motor_driver_stop(&handle->motor_handles[i]);
        }
    }

    ESP_LOGI(TAG, "All motors stopped");
    return ESP_OK;
}

esp_err_t omniwheel_system_deinit(omniwheel_system_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Deinitializing omniwheel system");

    // Stop all motors first
    omniwheel_system_stop(handle);

    // Deinitialize each motor
    for (int i = 0; i < OMNIWHEEL_NUM_MOTORS; i++) {
        if (handle->motors_initialized[i]) {
            motor_driver_deinit(&handle->motor_handles[i]);
            handle->motors_initialized[i] = false;
        }
    }

    ESP_LOGI(TAG, "Omniwheel system deinitialized");
    return ESP_OK;
}

