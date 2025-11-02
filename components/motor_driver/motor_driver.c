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
    
    // Reset and configure direction pin
    // IMPORTANT: Reset pin first to clear any previous configuration
    gpio_reset_pin(config->dir_pin);
    
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << config->dir_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // enable pull-up to help drive HIGH
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&dir_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure direction pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Explicitly set direction to output (redundant but ensures it's set)
    gpio_set_direction(config->dir_pin, GPIO_MODE_OUTPUT);

    // Increase drive capability to strongest to overcome external pulls
    gpio_set_drive_capability(config->dir_pin, GPIO_DRIVE_CAP_3);
    
    // Set initial direction to LOW (forward)
    gpio_set_level(config->dir_pin, MOTOR_DIR_FORWARD);
    
    // Initial log without read-back (some boards can't read outputs reliably)
    ESP_LOGI(TAG, "Direction pin %d initialized to LOW", config->dir_pin);
    
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
    
    // Calculate PWM duty cycle
    uint32_t duty = 0;
    if (speed != 0) {
        uint32_t abs_speed = (speed < 0) ? -speed : speed;
        duty = (handle->config.max_pwm_duty * abs_speed) / 100;
        // Ensure minimum duty for motors that need it (some motors won't start at very low PWM)
        // But for now, we'll trust the calculation - add minimum only if motor doesn't respond
    }
    
    // CRITICAL: Set direction BEFORE PWM to ensure proper sequencing
    // Direction change must happen before PWM is applied
    motor_direction_t direction = (speed >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    esp_err_t ret = motor_driver_set_direction(handle, direction);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Now set PWM duty cycle
    if (duty == 0) {
        // Stop motor: set PWM duty to 0
        ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set PWM duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to update PWM duty to 0: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Set PWM duty cycle (only if not zero - already handled above)
    if (duty > 0) {
        // Ensure PWM timer is running first
        ret = ledc_timer_resume(LEDC_LOW_SPEED_MODE, handle->config.pwm_timer);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGD(TAG, "PWM timer resume: %s (expected if already running)", esp_err_to_name(ret));
        }
        
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
        
        // Small delay to allow hardware to update (a few CPU cycles)
        volatile int delay_counter = 20;
        while (delay_counter-- > 0) {
            __asm__ __volatile__("nop");
        }
        
        // Optional read-back (disabled warnings to avoid confusion due to timing)
        (void) ledc_get_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
    }
    
    // Log motor state changes (not just debug level)
    if (speed == 0) {
        ESP_LOGI(TAG, "Motor STOPPED: speed=0%%, duty=0, dir_pin=%d", handle->config.dir_pin);
    } else {
        uint32_t actual_duty = ledc_get_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
        ESP_LOGI(TAG, "Motor MOVING: speed=%d%%, duty=%lu/%lu (actual=%lu), dir_pin=%d=%s, pwm_pin=%d", 
               speed, duty, handle->config.max_pwm_duty, actual_duty, handle->config.dir_pin,
               (direction == MOTOR_DIR_FORWARD) ? "HIGH" : "LOW", handle->config.pwm_pin);
    }
    return ESP_OK;
}

esp_err_t motor_driver_set_direction(motor_driver_handle_t *handle, motor_direction_t direction) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read current state before changing (for debugging)
    int prev_level = -1; // skip unreliable read-back on some boards
    
    // CRITICAL: Do NOT call gpio_set_direction here - it's already configured in init
    // Calling it repeatedly can cause issues. Just set the level directly.
    
    // Invert polarity: FORWARD -> HIGH, BACKWARD -> LOW
    int level = (direction == MOTOR_DIR_FORWARD) ? 1 : 0;
    esp_err_t ret = gpio_set_level(handle->config.dir_pin, level);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set direction level: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Do not verify read-back: some boards can't read output state reliably on strap pins
    ESP_LOGD(TAG, "Motor direction: %s (pin=%d, requested_level=%d)", 
             (direction == MOTOR_DIR_FORWARD) ? "FORWARD" : "BACKWARD",
             handle->config.dir_pin, level);
    return ESP_OK;
}

esp_err_t motor_driver_stop(motor_driver_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Explicitly set PWM duty to 0 and update
    // NOTE: No delays - may be called from interrupt/event context
    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM duty to 0: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update PWM duty to 0: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update again immediately (no delay)
    ledc_update_duty(LEDC_LOW_SPEED_MODE, handle->config.pwm_channel);
    
    ESP_LOGD(TAG, "Motor STOPPED - pwm_pin=%d duty=0", handle->config.pwm_pin);
    return ESP_OK;
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



