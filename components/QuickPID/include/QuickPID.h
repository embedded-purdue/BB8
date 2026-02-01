#pragma once

#ifndef QUICKPID_C_H
#define QUICKPID_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Enums ---
// Replaces C++ 'enum class Control'
typedef enum {
    PID_CONTROL_MANUAL,
    PID_CONTROL_AUTOMATIC,
    PID_CONTROL_TIMER,
    PID_CONTROL_TOGGLE
} PID_Control;

// Replaces C++ 'enum class Action'
typedef enum {
    PID_ACTION_DIRECT,
    PID_ACTION_REVERSE
} PID_Action;

// Replaces C++ 'enum class pMode'
typedef enum {
    PID_P_ON_ERROR,
    PID_P_ON_MEAS,
    PID_P_ON_ERROR_MEAS
} PID_pMode;

// Replaces C++ 'enum class dMode'
typedef enum {
    PID_D_ON_ERROR,
    PID_D_ON_MEAS
} PID_dMode;

// Replaces C++ 'enum class iAwMode'
typedef enum {
    PID_IAW_CONDITION,
    PID_IAW_CLAMP,
    PID_IAW_OFF
} PID_iAwMode;


// --- Main PID Structure ---
// Replaces C++ 'class QuickPID'
typedef struct {
    // --- Configuration ---
    float dispKp; // Proportional gain for display/user
    float dispKi; // Integral gain for display/user
    float dispKd; // Derivative gain for display/user

    float kp; // (P)roportional Tuning Parameter (runtime scaled)
    float ki; // (I)ntegral Tuning Parameter (runtime scaled)
    float kd; // (D)erivative Tuning Parameter (runtime scaled)

    float *myInput;   // Pointer to the Input variable
    float *myOutput;  // Pointer to the Output variable
    float *mySetpoint;// Pointer to the Setpoint variable

    PID_Control mode;
    PID_Action action;
    PID_pMode pmode;
    PID_dMode dmode;
    PID_iAwMode iawmode;

    uint32_t sampleTimeUs;
    float outMin, outMax;

    // --- Runtime State ---
    int64_t lastTime;      // Using int64_t for esp_timer_get_time()
    float outputSum;     // The integral accumulator
    float lastInput;
    float lastError;

    // --- Queryable Terms ---
    float pTerm;
    float iTerm;
    float dTerm;

} QuickPID_t;


// --- Function Prototypes (replaces class methods) ---

/**
 * @brief Default constructor equivalent. Initializes with minimal defaults.
 * @param pid Pointer to the QuickPID_t struct
 * @param Input Pointer to the input variable (e.g., sensor reading)
 * @param Output Pointer to the output variable (e.g., motor speed)
 * @param Setpoint Pointer to the setpoint variable (e.g., target temperature)
 */
void QuickPID_InitDefault(QuickPID_t *pid, float *Input, float *Output, float *Setpoint);

/**
 * @brief Constructor equivalent. Links PID, sets tunings, and control action.
 * @param pid Pointer to the QuickPID_t struct
 * @param Input Pointer to the input variable
 * @param Output Pointer to the output variable
 * @param Setpoint Pointer to the setpoint variable
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param Action PID_ACTION_DIRECT or PID_ACTION_REVERSE
 */
void QuickPID_Init(QuickPID_t *pid, float *Input, float *Output, float *Setpoint,
                   float Kp, float Ki, float Kd, PID_Action Action);

/**
 * @brief Full constructor equivalent. Links PID, sets all tunings and modes.
 * @param pid Pointer to the QuickPID_t struct
 * @param Input Pointer to the input variable
 * @param Output Pointer to the output variable
 * @param Setpoint Pointer to the setpoint variable
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param pMode Proportional mode (PID_P_ON_ERROR, etc.)
 * @param dMode Derivative mode (PID_D_ON_ERROR, etc.)
 * @param iAwMode Anti-Windup mode (PID_IAW_CONDITION, etc.)
 * @param Action PID_ACTION_DIRECT or PID_ACTION_REVERSE
 */
void QuickPID_InitFull(QuickPID_t *pid, float *Input, float *Output, float *Setpoint,
                       float Kp, float Ki, float Kd,
                       PID_pMode pMode, PID_dMode dMode, PID_iAwMode iAwMode, PID_Action Action);

/**
 * @brief Sets PID mode to manual (0), automatic (1), timer (2) or toggle (3).
 * @param pid Pointer to the QuickPID_t struct
 * @param Mode The new control mode
 */
void QuickPID_SetMode(QuickPID_t *pid, PID_Control Mode);

/**
 * @brief Performs the PID calculation. Call this once per main loop.
 * Timing is handled internally using sampleTimeUs.
 * @param pid Pointer to the QuickPID_t struct
 * @return true if a new output value was computed, false otherwise.
 */
bool QuickPID_Compute(QuickPID_t *pid);

/**
 * @brief Sets and clamps the output to a specific range (0-255 by default).
 * @param pid Pointer to the QuickPID_t struct
 * @param Min Minimum output value
 * @param Max Maximum output value
 */
void QuickPID_SetOutputLimits(QuickPID_t *pid, float Min, float Max);

/**
 * @brief Sets new tuning parameters (Kp, Ki, Kd) during runtime.
 * @param pid Pointer to the QuickPID_t struct
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 */
void QuickPID_SetTunings(QuickPID_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief Overload for setting tunings and modes during runtime.
 * @param pid Pointer to the QuickPID_t struct
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param pMode Proportional mode
 * @param dMode Derivative mode
 * @param iAwMode Anti-Windup mode
 */
void QuickPID_SetTuningsFull(QuickPID_t *pid, float Kp, float Ki, float Kd,
                             PID_pMode pMode, PID_dMode dMode, PID_iAwMode iAwMode);

/**
 * @brief Sets the controller direction (direct or reverse).
 * @param pid Pointer to the QuickPID_t struct
 * @param Action PID_ACTION_DIRECT or PID_ACTION_REVERSE
 */
void QuickPID_SetControllerDirection(QuickPID_t *pid, PID_Action Action);

/**
 * @brief Sets the sample time in microseconds.
 * @param pid Pointer to the QuickPID_t struct
 * @param NewSampleTimeUs The new sample time in microseconds.
 */
void QuickPID_SetSampleTimeUs(QuickPID_t *pid, uint32_t NewSampleTimeUs);

/**
 * @brief Sets the computation method for the proportional term.
 * @param pid Pointer to the QuickPID_t struct
 * @param pMode The new proportional mode.
 */
void QuickPID_SetProportionalMode(QuickPID_t *pid, PID_pMode pMode);

/**
 * @brief Sets the computation method for the derivative term.
 * @param pid Pointer to the QuickPID_t struct
 * @param dMode The new derivative mode.
 */
void QuickPID_SetDerivativeMode(QuickPID_t *pid, PID_dMode dMode);

/**
 * @brief Sets the integral anti-windup mode.
 * @param pid Pointer to the QuickPID_t struct
 * @param iAwMode The new anti-windup mode.
 */
void QuickPID_SetAntiWindupMode(QuickPID_t *pid, PID_iAwMode iAwMode);

/**
 * @brief Manually sets the integral accumulator value.
 * @param pid Pointer to the QuickPID_t struct
 * @param sum The new value for the integral sum.
 */
void QuickPID_SetOutputSum(QuickPID_t *pid, float sum);

/**
 * @brief Initializes the PID for a bumpless transfer from manual to automatic.
 * Call this before switching to automatic mode.
 * @param pid Pointer to the QuickPID_t struct
 */
void QuickPID_Initialize(QuickPID_t *pid);

/**
 * @brief Clears pTerm, iTerm, dTerm and outputSum values to zero.
 * @param pid Pointer to the QuickPID_t struct
 */
void QuickPID_Reset(QuickPID_t *pid);

// --- PID Query functions ---

float QuickPID_GetKp(QuickPID_t *pid);
float QuickPID_GetKi(QuickPID_t *pid);
float QuickPID_GetKd(QuickPID_t *pid);
float QuickPID_GetPterm(QuickPID_t *pid);
float QuickPID_GetIterm(QuickPID_t *pid);
float QuickPID_GetDterm(QuickPID_t *pid);
float QuickPID_GetOutputSum(QuickPID_t *pid);
uint8_t QuickPID_GetMode(QuickPID_t *pid);
uint8_t QuickPID_GetDirection(QuickPID_t *pid);
uint8_t QuickPID_GetPmode(QuickPID_t *pid);
uint8_t QuickPID_GetDmode(QuickPID_t *pid);
uint8_t QuickPID_GetAwMode(QuickPID_t *pid);


#ifdef __cplusplus
}
#endif

#endif // QUICKPID_C_H
