#include "motor_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

static const char *TAG = "MOTOR_DRIVER";

// Encoder interrupt handler
static void IRAM_ATTR encoder_isr_handler(void *arg) {
    motor_driver_handle_t *handle = (motor_driver_handle_t *)arg;
    
    if (!handle->encoder_enabled) {
        return;
    }
    
    // Read encoder pins
    int a_state = gpio_get_level(handle->config.encoder_a_pin);
    int b_state = gpio_get_level(handle->config.encoder_b_pin);
    
    // Simple quadrature decoding (can be improved for better accuracy)
    static int last_a_state = 0;
    static int last_b_state = 0;
    
    if (a_state != last_a_state) {
        if (a_state == b_state) {
            handle->encoder_count++;
        } else {
            handle->encoder_count--;
        }
        last_a_state = a_state;
    }
}

esp_err_t motor_driver_init(const motor_driver_config_t *config, motor_driver_handle_t *handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Initializing motor driver");
    
    // Copy configuration
    handle->config = *config;
    handle->encoder_count = 0;
    handle->encoder_enabled = (config->encoder_a_pin != GPIO_NUM_NC && config->encoder_b_pin != GPIO_NUM_NC);
    
    // Configure direction pin
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << config->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&dir_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure direction pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set initial direction
    gpio_set_level(config->dir_pin, MOTOR_DIR_FORWARD);
    
    // Configure PWM timer
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = config->pwm_timer,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = config->pwm_frequency,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure PWM channel
    ledc_channel_config_t channel_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = config->pwm_channel,
        .timer_sel = config->pwm_timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = config->pwm_pin,
        .duty = 0,
        .hpoint = 0
    };
    ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure PWM channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure encoder pins if enabled
    if (handle->encoder_enabled) {
        ESP_LOGI(TAG, "Configuring encoder pins: A=%d, B=%d", config->encoder_a_pin, config->encoder_b_pin);
        
        gpio_config_t encoder_gpio_config = {
            .pin_bit_mask = (1ULL << config->encoder_a_pin) | (1ULL << config->encoder_b_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE,
        };
        ret = gpio_config(&encoder_gpio_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure encoder pins: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Install GPIO interrupt service
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
            return ret;
        }
        
        // Add interrupt handlers
        ret = gpio_isr_handler_add(config->encoder_a_pin, encoder_isr_handler, handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add encoder A ISR handler: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = gpio_isr_handler_add(config->encoder_b_pin, encoder_isr_handler, handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add encoder B ISR handler: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Motor driver initialized successfully");
    return ESP_OK;
}

esp_err_t motor_driver_set_speed(motor_driver_handle_t *handle, int8_t speed) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Clamp speed to valid range
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    
    // Set direction based on speed sign
    motor_direction_t direction = (speed >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    esp_err_t ret = motor_driver_set_direction(handle, direction);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Calculate PWM duty cycle
    uint32_t duty = 0;
    if (speed != 0) {
        uint32_t abs_speed = (speed < 0) ? -speed : speed;
        duty = (handle->config.max_pwm_duty * abs_speed) / 100;
    }
    
    // Set PWM duty cycle
    ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM duty: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Motor speed set to %d%% (duty: %lu)", speed, duty);
    return ESP_OK;
}

esp_err_t motor_driver_set_direction(motor_driver_handle_t *handle, motor_direction_t direction) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = gpio_set_level(handle->config.dir_pin, direction);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Motor direction set to %s", (direction == MOTOR_DIR_FORWARD) ? "FORWARD" : "BACKWARD");
    return ESP_OK;
}

esp_err_t motor_driver_stop(motor_driver_handle_t *handle) {
    return motor_driver_set_speed(handle, 0);
}

int32_t motor_driver_get_encoder_count(motor_driver_handle_t *handle) {
    if (!handle || !handle->encoder_enabled) {
        return 0;
    }
    
    return handle->encoder_count;
}

esp_err_t motor_driver_reset_encoder(motor_driver_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    handle->encoder_count = 0;
    ESP_LOGD(TAG, "Encoder count reset");
    return ESP_OK;
}

esp_err_t motor_driver_deinit(motor_driver_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Deinitializing motor driver");
    
    // Stop motor
    motor_driver_stop(handle);
    
    // Remove encoder interrupt handlers if enabled
    if (handle->encoder_enabled) {
        gpio_isr_handler_remove(handle->config.encoder_a_pin);
        gpio_isr_handler_remove(handle->config.encoder_b_pin);
    }
    
    ESP_LOGI(TAG, "Motor driver deinitialized");
    return ESP_OK;
}



