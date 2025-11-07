#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "bno055.h"     // for bno055_sample_t
#include "QuickPID.h"  

#ifdef __cplusplus
extern "C" {
#endif

/**
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
