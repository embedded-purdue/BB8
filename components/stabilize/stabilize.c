#include "stabilize.h"
#include <stdio.h>
#include <math.h>

// Head stabilization PID gains
// Tuned for keeping head upright (90°) when BB8 tilts
float Kp = 3.0f;       // Proportional gain - response to tilt angle
float Ki = 0.05f;      // Integral gain - steady-state correction (small)
float Kd = 1.5f;       // Derivative gain - damping on angular velocity
float dt = 0.02f;      // Sample period (50Hz = 0.02s)

/**
 * @brief Calculate head compensation to keep head upright when BB8 tilts
 * 
 * @param bb8_tilt_angle BB8 body tilt angle in degrees (pitch or roll)
 * @param bb8_tilt_rate BB8 body angular velocity in rad/s (gy or gx)
 * @param integral Accumulated integral term (maintain between calls)
 * @return float Motor compensation output (-100 to +100)
 * 
 * Logic: When BB8 tilts forward, head needs to move backward to stay upright
 *        Compensation = -tilt_angle (opposite direction)
 */
float calculate_head_compensation(float bb8_tilt_angle, float bb8_tilt_rate, float *integral) {
    // Calculate error: head should be at 0° relative to gravity
    // If BB8 tilts +10° forward, head needs -10° compensation (move backward)
    float error = -bb8_tilt_angle;  // Negative = opposite direction
    
    // Apply small dead zone to prevent jitter
    if (fabs(error) < 0.5f) {
        error = 0.0f;
    }
    
    // Accumulate integral term
    *integral += error * dt;
    
    // Anti-windup: limit integral to prevent saturation
    if (*integral > 30.0f) *integral = 30.0f;
    if (*integral < -30.0f) *integral = -30.0f;
    
    // PID calculation
    // Convert tilt_rate from rad/s to degrees/s for derivative term
    float derivative_error = -bb8_tilt_rate * (180.0f / M_PI);  // Convert rad/s to deg/s
    
    float output = Kp * error + Ki * *integral + Kd * derivative_error;
    
    // Clamp output to reasonable range (will be further clamped to -100/+100 in main)
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    return output;
}

