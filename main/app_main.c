#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"

#include "motor_driver.h"
#include "uni.h"

// BTstack includes for proper initialization
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include "sdkconfig.h"

static const char *TAG = "APP";

// ============================================================================
// BODY MOTOR CONTROL (Controller-Based)
// ============================================================================
// This branch controls the body motor using Xbox controller input.
// When merged to main, this will work alongside the head motor (IMU-based)
// which uses separate GPIO pins and motor driver instance.
// ============================================================================

// Body motor driver handle (for controller-based movement)
static motor_driver_handle_t body_motor_handle;

// Controller state
static bool controller_connected = false;
static SemaphoreHandle_t body_motor_mux = NULL;

// Bluepad32 platform callbacks
static void bb8_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    ESP_LOGI(TAG, "BB8 Platform initialized");
}

static void bb8_platform_on_init_complete(void) {
    ESP_LOGI(TAG, "BB8 Platform init complete - starting Bluetooth scanning");
    ESP_LOGI(TAG, "ESP32 is now discoverable - put your Xbox controller in pairing mode");
    // Start scanning for controllers
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
    ESP_LOGI(TAG, "Bluetooth scanning started - waiting for controllers to connect...");
}

static uni_error_t bb8_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // Log discovered devices for debugging
    char addr_str[18];
    snprintf(addr_str, sizeof(addr_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    ESP_LOGI(TAG, "Device discovered: %s (name: %s, COD: 0x%04x, RSSI: %d)", 
             addr_str, name ? name : "Unknown", cod, rssi);
    // Accept all gamepads (Xbox controllers will be automatically accepted)
    return UNI_ERROR_SUCCESS;
}

static void bb8_platform_on_device_connected(uni_hid_device_t* d) {
    ARG_UNUSED(d);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CONTROLLER CONNECTED!");
    ESP_LOGI(TAG, "========================================");
    controller_connected = true;
    // Ensure motor is stopped when controller connects
    motor_driver_stop(&body_motor_handle);
}

static void bb8_platform_on_device_disconnected(uni_hid_device_t* d) {
    ARG_UNUSED(d);
    ESP_LOGI(TAG, "Controller disconnected");
    controller_connected = false;
    // Stop body motor when controller disconnects
    motor_driver_stop(&body_motor_handle);
}

static uni_error_t bb8_platform_on_device_ready(uni_hid_device_t* d) {
    ARG_UNUSED(d);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CONTROLLER READY!");
    ESP_LOGI(TAG, "Use LEFT JOYSTICK Y-axis to control body motor:");
    ESP_LOGI(TAG, "  - Push UP    = Forward (increases with deflection)");
    ESP_LOGI(TAG, "  - Push DOWN  = Backward (increases with deflection)");
    ESP_LOGI(TAG, "  - Center     = Stop");
    ESP_LOGI(TAG, "========================================");
    return UNI_ERROR_SUCCESS;
}

static void bb8_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    ARG_UNUSED(d);
    
    if (ctl->klass != UNI_CONTROLLER_CLASS_GAMEPAD) {
        return;
    }
    
    uni_gamepad_t* gp = &ctl->gamepad;
    
    // ========================================================================
    // BODY MOTOR CONTROL: Map left joystick Y-axis to body motor speed
    // ========================================================================
    // Xbox controller axis_y ranges:
    //   -512 (full UP) to +511 (full DOWN), 0 = centered
    // We invert so: UP (raw=-512) → +512 → forward, DOWN (raw=+511) → -511 → backward
    int32_t axis_y_raw = gp->axis_y;
    int32_t axis_y = -axis_y_raw;  // Invert: UP becomes positive, DOWN becomes negative
    
    // Dead zone to ignore joystick drift/calibration near center
    const int32_t DEAD_ZONE = 50;  // Ignore movements within ±50 of center (~10% of range)
    int8_t body_motor_speed = 0;
    
    // Calculate motor speed only if joystick is outside dead zone
    if (axis_y > DEAD_ZONE) {
        // UP pushed: axis_y is positive, motor should go forward (positive speed)
        // Subtract dead zone and scale
        int32_t axis_offset = axis_y - DEAD_ZONE;
        body_motor_speed = (int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (body_motor_speed > 100) body_motor_speed = 100;
        if (body_motor_speed < 1) body_motor_speed = 1;  // Minimum forward speed
    } else if (axis_y < -DEAD_ZONE) {
        // DOWN pushed: axis_y is negative, motor should go backward (negative speed)
        // Add dead zone (make positive) and scale
        int32_t axis_offset = -(axis_y + DEAD_ZONE);  // Make positive
        body_motor_speed = -(int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (body_motor_speed < -100) body_motor_speed = -100;
        if (body_motor_speed > -1) body_motor_speed = -1;  // Minimum backward speed
    } else {
        // Within dead zone: EXPLICITLY set speed to 0
        body_motor_speed = 0;
    }
    
    // Always apply motor speed (even if 0, to ensure motor stops when centered)
    // Use non-blocking mutex take with timeout to avoid blocking BTstack
    if (body_motor_mux != NULL && xSemaphoreTake(body_motor_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        static int8_t last_applied_speed = 127;  // Track last applied speed to avoid redundant calls
        
        // Only update if speed actually changed
        if (body_motor_speed != last_applied_speed) {
            esp_err_t err;
            if (body_motor_speed == 0) {
                err = motor_driver_stop(&body_motor_handle);
            } else {
                err = motor_driver_set_speed(&body_motor_handle, body_motor_speed);
            }
            
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set body motor speed: %s", esp_err_to_name(err));
            } else {
                last_applied_speed = body_motor_speed;
                
                // Log speed changes (reduced frequency - only when speed changes)
                if (body_motor_speed > 0) {
                    ESP_LOGI(TAG, "MOTOR: FORWARD speed=%d%% [joystick raw=%ld]", 
                           body_motor_speed, axis_y_raw);
                } else if (body_motor_speed < 0) {
                    ESP_LOGI(TAG, "MOTOR: BACKWARD speed=%d%% [joystick raw=%ld]", 
                           body_motor_speed, axis_y_raw);
                } else {
                    ESP_LOGI(TAG, "MOTOR: STOPPED [joystick raw=%ld]", axis_y_raw);
                }
            }
        }
        xSemaphoreGive(body_motor_mux);
    }
}

static const uni_property_t* bb8_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void bb8_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    ARG_UNUSED(event);
    ARG_UNUSED(data);
}

// Platform structure
static struct uni_platform bb8_platform = {
    .name = "bb8",
    .init = bb8_platform_init,
    .on_init_complete = bb8_platform_on_init_complete,
    .on_device_discovered = bb8_platform_on_device_discovered,
    .on_device_connected = bb8_platform_on_device_connected,
    .on_device_disconnected = bb8_platform_on_device_disconnected,
    .on_device_ready = bb8_platform_on_device_ready,
    .on_controller_data = bb8_platform_on_controller_data,
    .get_property = bb8_platform_get_property,
    .on_oob_event = bb8_platform_on_oob_event,
};

void app_main(void) {
    ESP_LOGI(TAG, "BB8 Controller-Only FW starting");

    // Initialize BTstack stdio (if using UART console)
    // Don't use BTstack buffered UART if it conflicts with console
#ifdef CONFIG_ESP_CONSOLE_UART
#ifndef CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE
    btstack_stdio_init();
#endif
#endif

    // Initialize BTstack - MUST be called before uni_init()
    btstack_init();
    ESP_LOGI(TAG, "BTstack initialized");

    // Set custom platform for bluepad32 - MUST be before uni_init()
    uni_platform_set_custom(&bb8_platform);
    
    // Initialize Bluepad32 (this will call our platform callbacks)
    uni_init(0, NULL);
    ESP_LOGI(TAG, "Bluepad32 initialized");

    // ========================================================================
    // BODY MOTOR INITIALIZATION (Controller-Based)
    // ========================================================================
    // NOTE: When merged to main, the head motor (IMU-based) will use
    //       different GPIO pins and a separate motor_driver_handle_t instance.
    //       Both motors can coexist and operate independently.
    // ========================================================================

    // Create mutex for body motor control protection
    body_motor_mux = xSemaphoreCreateMutex();
    if (body_motor_mux == NULL) {
        ESP_LOGE(TAG, "Failed to create body motor control mutex");
        return;
    }

    // Configure body motor driver
    motor_driver_config_t body_motor_config = {
        .dir_pin = GPIO_NUM_4,        // Body motor direction control pin
        .pwm_pin = GPIO_NUM_5,        // Body motor PWM control pin
        .encoder_a_pin = GPIO_NUM_18, // Body motor encoder channel A (optional)
        .encoder_b_pin = GPIO_NUM_19, // Body motor encoder channel B (optional)
        .pwm_channel = LEDC_CHANNEL_0, // Body motor PWM channel
        .pwm_timer = LEDC_TIMER_0,     // Body motor PWM timer
        .pwm_frequency = 1000,         // 1kHz PWM frequency
        .max_pwm_duty = 8191          // 13-bit resolution (2^13 - 1)
    };
    
    // Initialize body motor driver
    ESP_ERROR_CHECK(motor_driver_init(&body_motor_config, &body_motor_handle));
    ESP_LOGI(TAG, "Body motor driver initialized");

    // Explicitly stop motor at startup
    motor_driver_stop(&body_motor_handle);
    ESP_LOGI(TAG, "Body motor explicitly stopped - waiting for controller connection...");
    ESP_LOGI(TAG, "When connected, use left joystick Y-axis to control body motor");
    ESP_LOGI(TAG, "  - Push joystick UP = forward (positive speed)");
    ESP_LOGI(TAG, "  - Push joystick DOWN = backward (negative speed)");
    ESP_LOGI(TAG, "  - Center joystick = stop");
    
    // Start BTstack run loop (this blocks forever - controller callbacks run in BTstack context)
    // This must be called after all initialization
    ESP_LOGI(TAG, "Starting BTstack run loop...");
    btstack_run_loop_execute();
    
    // Should never reach here
    ESP_LOGE(TAG, "BTstack run loop exited unexpectedly!");
}
