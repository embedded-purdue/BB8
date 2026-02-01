
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "bus_i2c.h"
#include "bno055.h"
#include "serial_stream.h"
#include "motor_driver.h"
#include "stabilize.h"

static const char *TAG = "APP";

// Motor driver handle
static motor_driver_handle_t motor_handle;

    static float smoothed_pitch = 0.0f;
    static float smoothed_roll = 0.0f;

    const float PITCH_OFFSET = -1.2; // Use the value from your log
    const float ROLL_OFFSET  = .9;  // Use the value from your log

    // Define your filter strength (a value between 0 and 1)
    // A larger value means more smoothing but more "lag".
    const float LPF_ALPHA = 0.8;

// Motor control task
static void motor_control_task(void *pvParameters) {
    
    ESP_LOGI(TAG, "Motor control task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 10Hz motor control loop
    
    int8_t motor_speed = 0;
    int speed_increment = 10;
    bool increasing = true;
    
    while (1) {
        // Simple motor control demo - gradually increase/decrease speed
        if (increasing) {
            motor_speed += speed_increment;
            if (motor_speed >= 50) {
                increasing = false;
            }
        } else {
            motor_speed -= speed_increment;
            if (motor_speed <= -50) {
                increasing = true;
            }
        }
        
        // Set motor speed
        esp_err_t err = motor_driver_set_speed(&motor_handle, motor_speed);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set motor speed: %s", esp_err_to_name(err));
        }
        
       // Get encoder count
        int32_t encoder_count = motor_driver_get_encoder_count(&motor_handle);
        ESP_LOGI(TAG, "Motor speed: %d%%, Encoder count: %ld", motor_speed, encoder_count);


        // Only log motor speed every 10 loops (1 per second)
        static int motor_log_counter = 0;
        motor_log_counter++;
        if (motor_log_counter >= 10) {
            motor_log_counter = 0;
            ESP_LOGI(TAG, "Motor speed: %d%%, Encoder count: %ld", motor_speed, encoder_count);
        }
        
        vTaskDelayUntil(&last_wake_time, period);
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "BB8 FW starting");

    // Init I2C (SDA=21, SCL=22 @ 100kHz for BNO055 compatibility)
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 100000));

    // Init BNO055 IMU (with improved timeout handling)
    ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, BNO055_ADDR_A));

    // Init serial streaming
    ESP_ERROR_CHECK(serial_stream_init());

    // Configure motor driver
    motor_driver_config_t motor_config = {
        .dir_pin = GPIO_NUM_4,        // Direction control pin
        .pwm_pin = GPIO_NUM_5,        // PWM control pin
        .encoder_a_pin = GPIO_NUM_18, // Encoder channel A (optional)
        .encoder_b_pin = GPIO_NUM_19, // Encoder channel B (optional)
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0,
        .pwm_frequency = 1000,        // 1kHz PWM frequency
        .max_pwm_duty = 8191         // 13-bit resolution (2^13 - 1)
    };
    
    // Init motor driver
    ESP_ERROR_CHECK(motor_driver_init(&motor_config, &motor_handle));

    stabilize_init();

    ESP_LOGI(TAG, "All systems initialized, starting real-time IMU streaming and motor control");

    // Create motor control task
    //xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, NULL);

    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50Hz sampling (reduced for BNO055 stability)

    int print_counter = 0;
    while (1) {
        bno055_sample_t s;
        esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s);

        if (err == ESP_OK) {

            float corrected_pitch = s.pitch - PITCH_OFFSET;
            float corrected_roll = s.roll - ROLL_OFFSET;

            // --- ADD THIS FILTER ---
            // Apply the low-pass filter
            smoothed_pitch = (LPF_ALPHA * smoothed_pitch) + ((1.0 - LPF_ALPHA) * corrected_pitch);
            smoothed_roll  = (LPF_ALPHA * smoothed_roll)  + ((1.0 - LPF_ALPHA) * corrected_roll);

            // Create a *copy* of the sample 's' to pass to the PID
            // (or just modify 's' directly if you don't need the raw value elsewhere)
            bno055_sample_t filtered_s = s;
            filtered_s.pitch = smoothed_pitch;
            filtered_s.roll = smoothed_roll;
            // --- END FILTER ---
            // NOW, use the FILTERED sample for the PID
            double fb_response = calculateFB(filtered_s);
            double lr_response = calculateLR(filtered_s);
            // (10 loops * 100ms = 1000ms = 1 print per second)
            //motor_driver_set_speed(&motor_handle, (int8_t)fb_response);
            
            // Log BNO055 data with calibration status
            ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s | mx=%.1f my=%.1f mz=%.1f μT | r=%.1f p=%.1f y=%.1fgit° | cal=%d%d%d%d",
                        (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, 
                        s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
                        s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal);
            
            // Get motor encoder data
            int32_t encoder_count = motor_driver_get_encoder_count(&motor_handle);
            
            // Create JSON with all 9-axis data, motor data, and send via serial
            char json_data[768];
            snprintf(json_data, sizeof(json_data), 
                "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"temp\":%.1f,\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d},\"motor\":{\"encoder\":%ld}}",
                (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                s.mx, s.my, s.mz, smoothed_roll, smoothed_pitch, s.yaw,
                s.qw, s.qx, s.qy, s.qz, s.temp,
                s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal,
                encoder_count);
            
            serial_stream_send_data(json_data);
            
            

            printf("FB Response: %lf\n", fb_response);
            printf("LR Response: %lf\n\n\n\n\n", lr_response);

            
        // Here we would use the responses
        // motor_driver_set_speed(&motor_handle, (int8_t)fb_response);
        } else {
            ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&t0, pdMS_TO_TICKS(1000));
    }

}
