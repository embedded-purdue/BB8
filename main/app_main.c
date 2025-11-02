
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"
// #include "driver/i2c.h"

// #include "bus_i2c.h"
// #include "bno055.h"
// #include "serial_stream.h"
// #include "motor_driver.h"
// #include "stabilize.h"

// static const char *TAG = "APP";

// // Motor driver handle
// static motor_driver_handle_t motor_handle;

// // Motor control task
// static void motor_control_task(void *pvParameters) {
//     ESP_LOGI(TAG, "Motor control task started");
    
//     TickType_t last_wake_time = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(100); // 10Hz motor control loop
    
//     int8_t motor_speed = 0;
//     int speed_increment = 10;
//     bool increasing = true;
    
//     while (1) {
//         // Simple motor control demo - gradually increase/decrease speed
//         // if (increasing) {
//         //     motor_speed += speed_increment;
//         //     if (motor_speed >= 50) {
//         //         increasing = false;
//         //     }
//         // } else {
//         //     motor_speed -= speed_increment;
//         //     if (motor_speed <= -50) {
//         //         increasing = true;
//         //     }
//         motor_speed = calcFB(s, pidFB, &pidFB.integral)
//         }
        
//         // Set motor speed
//         esp_err_t err = motor_driver_set_speed(&motor_handle, motor_speed);
//         if (err != ESP_OK) {
//             ESP_LOGW(TAG, "Failed to set motor speed: %s", esp_err_to_name(err));
//         }
        
//         // Get encoder count
//         int32_t encoder_count = motor_driver_get_encoder_count(&motor_handle);
//         ESP_LOGI(TAG, "Motor speed: %d%%, Encoder count: %ld", motor_speed, encoder_count);
        
//         vTaskDelayUntil(&last_wake_time, period);
//     }
// }

// void app_main(void) {
//     ESP_LOGI(TAG, "BB8 FW starting");

//     // Init I2C (SDA=21, SCL=22 @ 100kHz for BNO055 compatibility)
//     ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 100000));

//     // Init BNO055 IMU (with improved timeout handling)
//     ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, BNO055_ADDR_A));

//     // Init serial streaming
//     ESP_ERROR_CHECK(serial_stream_init());

//     // Configure motor driver
//     motor_driver_config_t motor_config = {
//         .dir_pin = GPIO_NUM_4,        // Direction control pin
//         .pwm_pin = GPIO_NUM_5,        // PWM control pin
//         .encoder_a_pin = GPIO_NUM_18, // Encoder channel A (optional)
//         .encoder_b_pin = GPIO_NUM_19, // Encoder channel B (optional)
//         .pwm_channel = LEDC_CHANNEL_0,
//         .pwm_timer = LEDC_TIMER_0,
//         .pwm_frequency = 1000,        // 1kHz PWM frequency
//         .max_pwm_duty = 8191         // 13-bit resolution (2^13 - 1)
//     };
    
//     // Init motor driver
//     ESP_ERROR_CHECK(motor_driver_init(&motor_config, &motor_handle));

//     ESP_LOGI(TAG, "All systems initialized, starting real-time IMU streaming and motor control");

//     // Create motor control task
//     xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, NULL);

//     TickType_t t0 = xTaskGetTickCount();
//     const TickType_t period = pdMS_TO_TICKS(20); // 50Hz sampling (reduced for BNO055 stability)

//     PID pidFB;
//     PID pidLR;
//     pidFB.integral = 0;
//     pidLR.integral = 0;
//     while (1) {
//         bno055_sample_t s;
//         esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, &s);
//         if (err == ESP_OK) {
//             // Log BNO055 data with calibration status
//             ESP_LOGI(TAG, "t=%u ax=%.3f ay=%.3f az=%.3f m/s^2 | gx=%.3f gy=%.3f gz=%.3f rad/s | mx=%.1f my=%.1f mz=%.1f μT | r=%.1f p=%.1f y=%.1f° | cal=%d%d%d%d",
//                         (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz, 
//                         s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
//                         s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal);
            
//             // Get motor encoder data
//             int32_t encoder_count = motor_driver_get_encoder_count(&motor_handle);
            
//             // Create JSON with all 9-axis data, motor data, and send via serial
//             char json_data[768];
//             snprintf(json_data, sizeof(json_data), 
//                 "{\"t\":%u,\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"temp\":%.1f,\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d},\"motor\":{\"encoder\":%ld}}",
//                 (unsigned) s.t_ms, s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
//                 s.mx, s.my, s.mz, s.roll, s.pitch, s.yaw,
//                 s.qw, s.qx, s.qy, s.qz, s.temp,
//                 s.sys_cal, s.gyro_cal, s.accel_cal, s.mag_cal,
//                 encoder_count);
            
//             serial_stream_send_data(json_data);
//             printf("%lf\n", calculateFB(s, pidFB, &pidFB.integral));
//             printf("%lf\n\n\n\n\n", calculateLR(s, pidLR, &pidLR.integral));
//         } else {
//             ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
//         }

//         vTaskDelayUntil(&t0, pdMS_TO_TICKS(100));
//     }

// }

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

/* ---------- Task parameter bundle ---------- */
typedef struct {
    PID *pidFB;                   // uses pidFB only
    volatile bno055_sample_t *s;  // shared IMU sample (producer: app_main)
} motor_task_params_t;

/* ---------- Globals ---------- */
static motor_driver_handle_t motor_handle;
static volatile bno055_sample_t g_shared_sample;   // IMU sample storage shared with task

/* ---------- Motor control task (driven by pidFB) ---------- */
static void motor_control_task(void *pvParameters) {
    motor_task_params_t *p = (motor_task_params_t *)pvParameters;
    PID *pidFB = p->pidFB;
    const volatile bno055_sample_t *shared = p->s;

    ESP_LOGI(TAG, "Motor control task started (pidFB-driven)");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(100); // 10 Hz demo; adjust as needed

    while (1) {
        // Snapshot the shared IMU sample (avoid tearing by copying once)
        bno055_sample_t s_local = *shared;

        // PID output -> speed command (assumed to be in “percent-like” units)
        double u = 3 * calculateFB(s_local, *pidFB, &pidFB->integral);

        // Optional: simple clamp to [-100, 100] for motor_driver_set_speed
        if (u > 100.0) u = 100.0;
        if (u < -100.0) u = -100.0;

        int8_t motor_speed = (int8_t)(u);

        esp_err_t err = motor_driver_set_speed(&motor_handle, motor_speed);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set_speed failed: %s", esp_err_to_name(err));
        }

        int32_t enc = motor_driver_get_encoder_count(&motor_handle);
        ESP_LOGI(TAG, "u=%.2f -> speed=%d%% | encoder=%ld", u, motor_speed, enc);

        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---------- Main ---------- */
void app_main(void) {
    ESP_LOGI(TAG, "BB8 FW starting");

    // I2C + BNO055 + Serial
    ESP_ERROR_CHECK(bus_i2c_init(I2C_NUM_0, 21, 22, 100000));
    ESP_ERROR_CHECK(bno055_init(I2C_NUM_0, BNO055_ADDR_A));
    ESP_ERROR_CHECK(serial_stream_init());

    // Motor driver
    motor_driver_config_t motor_config = {
        .dir_pin = GPIO_NUM_4,
        .pwm_pin = GPIO_NUM_5,
        .encoder_a_pin = GPIO_NUM_18,
        .encoder_b_pin = GPIO_NUM_19,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0,
        .pwm_frequency = 1000,
        .max_pwm_duty = 8191
    };
    ESP_ERROR_CHECK(motor_driver_init(&motor_config, &motor_handle));

    // PID (FB only used for motor speed)
    PID pidFB = {0};
    // TODO: set gains if needed, e.g.:
    // pidFB.Kp = ...; pidFB.Ki = ...; pidFB.Kd = ...;
    pidFB.integral = 0;

    ESP_LOGI(TAG, "Init complete: IMU + Serial + Motor + PID(FB)");

    // Create task params
    motor_task_params_t params = {
        .pidFB = &pidFB,
        .s     = &g_shared_sample
    };

    // Start motor control task
    xTaskCreate(motor_control_task, "motor_control", 4096, &params, 5, NULL);

    // Main loop: read IMU @ 50 Hz, stream JSON (and update shared sample for task)
    TickType_t t0 = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz

    while (1) {
        esp_err_t err = bno055_read_sample(I2C_NUM_0, BNO055_ADDR_A, (bno055_sample_t *)&g_shared_sample);
        if (err == ESP_OK) {
            const bno055_sample_t *s = (const bno055_sample_t *)&g_shared_sample;

            ESP_LOGI(TAG,
                     "t=%u ax=%.3f ay=%.3f az=%.3f | gx=%.3f gy=%.3f gz=%.3f | "
                     "mx=%.1f my=%.1f mz=%.1f | r=%.1f p=%.1f y=%.1f | cal=%d%d%d%d",
                     (unsigned)s->t_ms, s->ax, s->ay, s->az, s->gx, s->gy, s->gz,
                     s->mx, s->my, s->mz, s->roll, s->pitch, s->yaw,
                     s->sys_cal, s->gyro_cal, s->accel_cal, s->mag_cal);

            int32_t enc = motor_driver_get_encoder_count(&motor_handle);

            char json[768];
            snprintf(json, sizeof(json),
                     "{\"t\":%u,"
                     "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
                     "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,"
                     "\"mx\":%.1f,\"my\":%.1f,\"mz\":%.1f,"
                     "\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f,"
                     "\"qw\":%.4f,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,"
                     "\"temp\":%.1f,"
                     "\"cal\":{\"sys\":%d,\"gyro\":%d,\"accel\":%d,\"mag\":%d},"
                     "\"motor\":{\"encoder\":%ld}}",
                     (unsigned)s->t_ms, s->ax, s->ay, s->az, s->gx, s->gy, s->gz,
                     s->mx, s->my, s->mz, s->roll, s->pitch, s->yaw,
                     s->qw, s->qx, s->qy, s->qz, s->temp,
                     s->sys_cal, s->gyro_cal, s->accel_cal, s->mag_cal,
                     enc);

            serial_stream_send_data(json);
        } else {
            ESP_LOGW(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
        }

        vTaskDelayUntil(&t0, period);
    }
}

