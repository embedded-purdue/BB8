#include <stdbool.h>
#include <stdint.h>

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

#ifdef __cplusplus
}
#endif


