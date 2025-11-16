#include "stabilize.h"
#include <stdio.h>
#include <math.h>


// PID controller instance for Front/Back (Pitch)
static QuickPID_t pidFB;
// PID controller instance for Left/Right (Roll)
static QuickPID_t pidLR;

float inputFBDerivative; //to store the gyroscope movement for FB
float inputLRDerivative; //to store the gyroscope movement for LR

// Variables to link to the PID controllers
static float inputFB, outputFB, setpointFB;
static float inputLR, outputLR, setpointLR;

static const float radToDeg = 180.0f / (float)M_PI;
//static const float radToDeg = 57.29577951308232f; another way to do that ^^


/**
 * @brief Initializes the PID controllers.
 */
void stabilize_init(void) {
    // Set our desired stabilization targets (0 degrees)
    setpointFB = 0.0f;
    setpointLR = 0.0f;

    // --- Define tuning parameters here ---
    float fb_kp = 1.0;
    float fb_ki = 0.0;
    float fb_kd = 0.0; // changed from 1 to 0

    float lr_kp = 1.0;
    float lr_ki = 0.0;
    float lr_kd = 0.0; //changed from 1 to 0

    /*
     * Initialize the Pitch (FB) PID controller:
     * We link it to our static variables.
     *
     * IMPORTANT LOGIC:
     * pMode = PID_P_ON_ERROR: Proportional term is Kp * error (setpoint - angle).
     * 
     *
     * dMode = PID_D_ON_MEAS: Derivative term is based on the change in measurement (angle).
     * 
     *
     * Action = PID_ACTION_REVERSE:
     * - If angle is positive (e.g., tilted forward), output must be negative
     * (e.g., slow front motors).
     * 
     *
     */
    QuickPID_InitFull(&pidFB, &inputFB, &outputFB, &setpointFB,
                      fb_kp, fb_ki, fb_kd,
                      PID_P_ON_ERROR,
                      PID_D_ON_MEAS,
                      PID_IAW_CONDITION, // Good default anti-windup
                      PID_ACTION_REVERSE);

    // Initialize the Roll (LR) PID controller (same logic)
    QuickPID_InitFull(&pidLR, &inputLR, &outputLR, &setpointLR,
                      lr_kp, lr_ki, lr_kd,
                      PID_P_ON_ERROR,
                      PID_D_ON_MEAS,
                      PID_IAW_CONDITION,
                      PID_ACTION_REVERSE);

    // Set sample time (10000 us = 10ms = 100Hz)
    // 100Hz is a good starting point for stabilization.
    QuickPID_SetSampleTimeUs(&pidFB, 10000);
    QuickPID_SetSampleTimeUs(&pidLR, 10000);

    // Set output limits (e.g., -255 to +255 for motor control)
    // ADJUST THESE to match your actuator (motor/servo) range
    QuickPID_SetOutputLimits(&pidFB, -255.0, 255.0);
    QuickPID_SetOutputLimits(&pidLR, -255.0, 255.0);

    // Set mode to automatic
    QuickPID_SetMode(&pidFB, PID_CONTROL_AUTOMATIC);
    QuickPID_SetMode(&pidLR, PID_CONTROL_AUTOMATIC);
}

/**
 * @brief Sets new tunings for the Pitch (FB) controller.
 */
void stabilize_set_fb_tunings(float kp, float ki, float kd) {
    QuickPID_SetTunings(&pidFB, kp, ki, kd);
}

/**
 * @brief Sets new tunings for the Roll (LR) controller.
 */
void stabilize_set_lr_tunings(float kp, float ki, float kd) {
    QuickPID_SetTunings(&pidLR, kp, ki, kd);
}

/**
 * @brief Calculates the stabilization response for the Pitch (Front/Back) axis.
 */
double calculateFB(bno055_sample_t s) {
    //REAL kd value (since kp is 0 at the top)
    float kd = .4;

    // 1. Update the input variable with the new sensor reading
    //quaternion values to use (more precise than IMU Euler angles)
    float qw = s.qw;
    float qx = s.qx;
    float qy = s.qy;
    float qz = s.qz;

    // float sinp = 2.0f * (qw*qx + qy*qz);
    float sinp = 2.0f * (qw*qy - qz*qx);
    //safety checks in the rare case float precision causes or code to break
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    inputFB = asinf(sinp) * radToDeg; 

    inputFBDerivative = s.gy;

    // 2. Compute the PID response.
    // The library handles timing, integral, and all calculations.
    QuickPID_Compute(&pidFB);

    // 3. Return the calculated output
    // printf("outputFB=%.3f kd=%.3f inputFBDerivative=%.3f", outputFB, kd, inputFBDerivative);
    return (double)(outputFB + kd * inputFBDerivative);
}

/**
 * @brief Calculates the stabilization response for the Roll (Left/Right) axis.
 */
double calculateLR(bno055_sample_t s) {
    //Another REAL kd value (since kp is 0 at the top)
    float kd = .4f;
    
    float qw = s.qw;
    float qx = s.qx;
    float qy = s.qy;
    float qz = s.qz;

    // Correct roll (x-axis) from quaternion
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    inputLR = atan2f(sinr_cosp, cosr_cosp) * radToDeg;

    inputLRDerivative = s.gx;   // assuming gx is the roll-rate axis for your mounting

    QuickPID_Compute(&pidLR);

    return (double)(outputLR + kd * inputLRDerivative);
}