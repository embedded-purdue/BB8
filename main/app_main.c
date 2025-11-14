#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"

#include "motor_driver.h"
#include "omniwheel_motor_system.h"
#include "uni.h"

// BTstack includes for proper initialization
#include <btstack_port_esp32.h>
#include <btstack_run_loop.h>
#include <btstack_stdio_esp32.h>
#include "sdkconfig.h"

static const char *TAG = "APP";

    // ============================================================================
    // BB8 OMNIWHEEL MOTOR SYSTEM (Controller-Based)
    // ============================================================================
    // Controls 4 omniwheel motors in diamond configuration for BB8 body movement.
    // Currently only 2 motors are connected (front two), but system supports all 4.
    // Diamond configuration for BB8 omniwheel system:
    //   Motor 0 (A): Front-Right - 45 degrees, formula: (y + x + r)
    //   Motor 1 (B): Front-Left - 135 degrees, formula: (y - x + r) [FIXED: all motors use +r]
    //   Motor 2 (C): Back-Left - 225 degrees, formula: (-y - x + r)
    //   Motor 3 (D): Back-Right - 315 degrees, formula: (-y + x + r)
    // ============================================================================

// Omniwheel system handle
static omniwheel_system_handle_t omniwheel_handle;

// Controller state
static bool controller_connected = false;
static SemaphoreHandle_t omniwheel_mux = NULL;

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
    // Ensure all motors are stopped when controller connects
    omniwheel_system_stop(&omniwheel_handle);
}

static void bb8_platform_on_device_disconnected(uni_hid_device_t* d) {
    ARG_UNUSED(d);
    ESP_LOGI(TAG, "Controller disconnected");
    controller_connected = false;
    // Stop all motors when controller disconnects
    omniwheel_system_stop(&omniwheel_handle);
}

static uni_error_t bb8_platform_on_device_ready(uni_hid_device_t* d) {
    ARG_UNUSED(d);
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "CONTROLLER READY!");
    ESP_LOGI(TAG, "BB8 Omniwheel Control:");
    ESP_LOGI(TAG, "  - RIGHT JOYSTICK X = Rotation/Direction (left/right)");
    ESP_LOGI(TAG, "  - LEFT JOYSTICK Y = Forward/Backward speed");
    ESP_LOGI(TAG, "    * Push UP (positive Y) = Forward");
    ESP_LOGI(TAG, "    * Push DOWN (negative Y) = Backward");
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
    // OMNIWHEEL CONTROL: Map joysticks to omniwheel velocities
    // ========================================================================
    // Control scheme for BB8 omniwheel system (based on image formulas):
    //   - RIGHT JOYSTICK X = Rotation component (vr)
    //   - LEFT JOYSTICK Y = Forward/Backward component (vy)
    //   - No strafing for now (vx = 0)
    // 
    // The omniwheel kinematics calculate motor speeds for BB8:
    //   - Motor 0 (A, Front-Right, 45°):  vy + vx + vr = vy + vr (since vx=0)
    //   - Motor 1 (B, Front-Left, 135°): vy - vx + vr = vy + vr (since vx=0) [FIXED: +vr]
    //   - Motor 2 (C, Back-Left, 225°):   -vy - vx + vr = -vy + vr (since vx=0)
    //   - Motor 3 (D, Back-Right, 315°): -vy + vx + vr = -vy + vr (since vx=0)
    //
    // This ensures proper behavior:
    //   - Forward only (vy>0, vr=0): A=vy, B=vy, C=-vy, D=-vy (front forward, back backward)
    //   - Rotation only (vy=0, vr>0): A=vr, B=vr, C=vr, D=vr (all same direction = rotation)
    //   - Combined (vy>0, vr>0): All motors contribute without cancellation
    //
    // With 2 motors at 45° and 135°:
    //   - Forward only: Both motors spin forward (vy + vr)
    //   - Rotation only: Both motors spin same direction (vr) = rotation
    //   - Combined: Both motors contribute to forward + rotation
    // ========================================================================
    // Xbox controller axes range: -512 to +511, 0 = centered
    
    // Dead zone to ignore joystick drift/calibration near center
    const int32_t DEAD_ZONE = 50;  // Ignore movements within ±50 of center (~10% of range)
    
    // No strafing for now (can add later with left joystick Y if needed)
    int8_t vx = 0;
    
    // RIGHT joystick X-axis: Rotation/Direction (vr)
    // Keep same sense as earlier LEFT stick mapping (no inversion):
    //   push RIGHT -> positive vr (CW), push LEFT -> negative vr (CCW)
    int32_t axis_rx_raw = gp->axis_rx;
    int32_t axis_rx = axis_rx_raw;  // No inversion
    int8_t vr = 0;
    if (axis_rx > DEAD_ZONE) {
        int32_t axis_offset = axis_rx - DEAD_ZONE;
        vr = (int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (vr > 100) vr = 100;
        if (vr < 1) vr = 1;
    } else if (axis_rx < -DEAD_ZONE) {
        int32_t axis_offset = -(axis_rx + DEAD_ZONE);
        vr = -(int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (vr < -100) vr = -100;
        if (vr > -1) vr = -1;
    }
    
    // LEFT joystick Y-axis: Forward/Backward speed (vy)
    // UP = forward, DOWN = backward.
    int32_t axis_ly_raw = gp->axis_y;  // Left stick Y
    int32_t axis_ly = -axis_ly_raw;  // Invert: UP becomes positive, DOWN becomes negative
    int8_t vy = 0;
    if (axis_ly > DEAD_ZONE) {
        // Pushed UP: Forward
        int32_t axis_offset = axis_ly - DEAD_ZONE;
        vy = (int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (vy > 100) vy = 100;
        if (vy < 1) vy = 1;
    } else if (axis_ly < -DEAD_ZONE) {
        // Pushed DOWN: Backward
        int32_t axis_offset = -(axis_ly + DEAD_ZONE);
        vy = -(int8_t)((axis_offset * 100) / (512 - DEAD_ZONE));
        if (vy < -100) vy = -100;
        if (vy > -1) vy = -1;
    }
    
    // Apply omniwheel velocities using proper kinematics
    // This will work correctly with 2 motors now, and automatically work with 4 motors later
    // Use non-blocking mutex take with timeout to avoid blocking BTstack
    if (omniwheel_mux != NULL && xSemaphoreTake(omniwheel_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        static int8_t last_vx = 127, last_vy = 127, last_vr = 127;
        
        // Only update if velocities actually changed
        if (vx != last_vx || vy != last_vy || vr != last_vr) {
            // Use the omniwheel system's proper kinematics
            // This calculates the correct speed for each motor based on omniwheel math
            esp_err_t err = omniwheel_system_set_velocity(&omniwheel_handle, vx, vy, vr);
            
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set omniwheel velocity: %s", esp_err_to_name(err));
            } else {
                last_vx = vx;
                last_vy = vy;
                last_vr = vr;
            }
        }
        xSemaphoreGive(omniwheel_mux);
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
    // OMNIWHEEL SYSTEM INITIALIZATION
    // ========================================================================
    // Configure 4-motor omniwheel system in diamond configuration
    // Currently only 2 motors are connected (front two), but system supports all 4
    // ========================================================================

    // Create mutex for omniwheel control protection
    omniwheel_mux = xSemaphoreCreateMutex();
    if (omniwheel_mux == NULL) {
        ESP_LOGE(TAG, "Failed to create omniwheel control mutex");
        return;
    }

    // Configure omniwheel system
    // CURRENT SETUP: 2 motors only (based on image formulas)
    //   - Motor 0 (A): Front-Right at 45° - formula: (y + x + r)
    //   - Motor 1 (B): Front-Left at 135° - formula: (y - x + r)
    // NOTE: Each motor MUST have separate GPIO pins for independent control
    //       If motors share GPIO pins, they will move together (not independent)
    // 
    // Physical positioning: Place motors at 45° and 135° angles (front-right and front-left)
    omniwheel_system_config_t omniwheel_config = {
        .pwm_frequency = 1000,   // 1kHz PWM frequency
        .max_pwm_duty = 8191,    // 13-bit resolution (2^13 - 1)
        .motors = {
            // Motor 0 (A): Front-Right - 45 degrees
            // Formula: (y + x + r) - contributes to forward, right strafe, and rotation
            {
                .dir_pin = GPIO_NUM_4,        // Direction pin - MUST be unique
                .pwm_pin = GPIO_NUM_5,        // PWM pin - MUST be unique
                .encoder_a_pin = GPIO_NUM_18, // Encoder A (optional)
                .encoder_b_pin = GPIO_NUM_19, // Encoder B (optional)
                .pwm_channel = LEDC_CHANNEL_0,
                .pwm_timer = LEDC_TIMER_0,
                .enabled = true  // Connected - Motor A (Front-Right)
            },
            // Motor 1 (B): Front-Left - 135 degrees
            // Formula: (y - x + r) - contributes to forward, left strafe, and rotation
            {
                .dir_pin = GPIO_NUM_12,  // Available GPIO on ESP32-WROOM DevKit C
                .pwm_pin = GPIO_NUM_13,  // Available GPIO on ESP32-WROOM DevKit C
                .encoder_a_pin = GPIO_NUM_NC,
                .encoder_b_pin = GPIO_NUM_NC,
                .pwm_channel = LEDC_CHANNEL_1,
                .pwm_timer = LEDC_TIMER_1,
                .enabled = true  // Connected - Motor B (Front-Left)
            },
            // Motor 2 (C): Back-Left - 225 degrees
            // Formula: (-y - x + r) - contributes to backward, left strafe, and rotation
            {
                .dir_pin = GPIO_NUM_14,  // Available GPIO on ESP32-WROOM DevKit C
                .pwm_pin = GPIO_NUM_15,  // Available GPIO on ESP32-WROOM DevKit C
                .encoder_a_pin = GPIO_NUM_NC,
                .encoder_b_pin = GPIO_NUM_NC,
                .pwm_channel = LEDC_CHANNEL_2,
                .pwm_timer = LEDC_TIMER_2,
                .enabled = false  // Not yet connected - Motor C (Back-Left)
            },
            // Motor 3 (D): Back-Right - 315 degrees
            // Formula: (-y + x + r) - contributes to backward, right strafe, and rotation
            {
                .dir_pin = GPIO_NUM_16,  // Available GPIO on ESP32-WROOM DevKit C
                .pwm_pin = GPIO_NUM_17,  // Available GPIO on ESP32-WROOM DevKit C
                .encoder_a_pin = GPIO_NUM_NC,
                .encoder_b_pin = GPIO_NUM_NC,
                .pwm_channel = LEDC_CHANNEL_3,
                .pwm_timer = LEDC_TIMER_3,
                .enabled = false  // Not yet connected - Motor D (Back-Right)
            }
        }
    };
    
    // Initialize omniwheel system
    ESP_ERROR_CHECK(omniwheel_system_init(&omniwheel_config, &omniwheel_handle));
    ESP_LOGI(TAG, "Omniwheel system initialized");

    // Explicitly stop all motors at startup
    omniwheel_system_stop(&omniwheel_handle);
    ESP_LOGI(TAG, "All motors stopped - waiting for controller connection...");
    ESP_LOGI(TAG, "CURRENT SETUP: 2-Motor Mode (Based on Image Formulas)");
    ESP_LOGI(TAG, "  - Motor 0 (A): Front-Right at 45° - GPIO 4 (dir), GPIO 5 (PWM)");
    ESP_LOGI(TAG, "    Formula: (y + x + r) - Forward + Right + Rotation");
    ESP_LOGI(TAG, "  - Motor 1 (B): Front-Left at 135° - GPIO 12 (dir), GPIO 13 (PWM)");
    ESP_LOGI(TAG, "    Formula: (y - x + r) - Forward - Right + Rotation");
    ESP_LOGI(TAG, "FUTURE SETUP: 4-Motor Mode (when C and D are connected)");
    ESP_LOGI(TAG, "  - Motor 2 (C): Back-Left at 225° - GPIO 14 (dir), GPIO 15 (PWM)");
    ESP_LOGI(TAG, "    Formula: (-y - x + r) - Backward - Right + Rotation");
    ESP_LOGI(TAG, "  - Motor 3 (D): Back-Right at 315° - GPIO 16 (dir), GPIO 17 (PWM)");
    ESP_LOGI(TAG, "    Formula: (-y + x + r) - Backward + Right + Rotation");
    ESP_LOGI(TAG, "When connected:");
    ESP_LOGI(TAG, "  - RIGHT JOYSTICK X = Rotation component (vr)");
    ESP_LOGI(TAG, "  - LEFT JOYSTICK Y = Forward/Backward component (vy)");
    ESP_LOGI(TAG, "    * Push UP = Forward");
    ESP_LOGI(TAG, "    * Push DOWN = Backward");
    ESP_LOGI(TAG, "NOTE: Position motors at 45° (A), 135° (B), 225° (C), 315° (D) angles");
    
    // Start BTstack run loop (this blocks forever - controller callbacks run in BTstack context)
    // This must be called after all initialization
    ESP_LOGI(TAG, "Starting BTstack run loop...");
    btstack_run_loop_execute();
    
    // Should never reach here
    ESP_LOGE(TAG, "BTstack run loop exited unexpectedly!");
}
