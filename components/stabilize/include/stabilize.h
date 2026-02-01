#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "bno055.h"     // for bno055_sample_t
#include "QuickPID.h"  

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate head compensation to keep head upright when BB8 body tilts
 * 
 * This is used for head stabilization - when BB8 rolls/tilts, the motor
 * compensates to keep the head pointing upright (90° from horizontal).
 * 
 * @param bb8_tilt_angle BB8 body tilt angle in degrees (positive = forward tilt)
 * @param bb8_tilt_rate BB8 body angular velocity in rad/s (rotation rate)
 * @param integral Pointer to integral accumulator (maintain state between calls)
 * @return float Motor compensation output (-100 to +100)
 * 
 * Example:
 *   BB8 tilts forward +10° → compensation = -10 (motor moves head backward)
 *   BB8 tilts backward -5° → compensation = +5 (motor moves head forward)
 */
float calculate_head_compensation(float bb8_tilt_angle, float bb8_tilt_rate, float *integral);
 * @brief Initializes the PID controllers for stabilization.
 * Call this ONCE at startup.
 */
void stabilize_init(void);

/**
 * @brief Calculates the stabilization response for the Pitch (Front/Back) axis.
 * @param s The BNO055 sensor sample.
 * @return The calculated PID response (e.g., motor adjustment).
 */
double calculateFB(bno055_sample_t s);

/**
 * @brief Calculates the stabilization response for the Roll (Left/Right) axis.
 * @param s The BNO055 sensor sample.
 * @return The calculated PID response (e.g., motor adjustment).
 */
double calculateLR(bno055_sample_t s);

/**
 * @brief Sets new tunings for the Pitch (FB) controller at runtime.
 */
void stabilize_set_fb_tunings(float kp, float ki, float kd);

/**
 * @brief Sets new tunings for the Roll (LR) controller at runtime.
 */
void stabilize_set_lr_tunings(float kp, float ki, float kd);


#ifdef __cplusplus
}
#endif
