#include "Quickpid.h"
#include <string.h> // For memset
#include "esp_timer.h" // For esp_timer_get_time()

// --- Helper Macro ---
#ifndef CONSTRAIN
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// --- Private Helper Functions ---
// Internal function to set tunings and handle scaling
static void _QuickPID_SetTunings(QuickPID_t *pid, float Kp, float Ki, float Kd) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    pid->dispKp = Kp;
    pid->dispKi = Ki;
    pid->dispKd = Kd;

    float sampleTimeSec = (float)pid->sampleTimeUs / 1000000.0f;

    pid->kp = Kp;
    pid->ki = Ki * sampleTimeSec;
    pid->kd = Kd / sampleTimeSec;

    if (pid->action == PID_ACTION_REVERSE) {
        pid->kp = (0 - pid->kp);
        pid->ki = (0 - pid->ki);
        pid->kd = (0 - pid->kd);
    }
}

// --- Public Function Definitions ---

void QuickPID_InitFull(QuickPID_t *pid, float *Input, float *Output, float *Setpoint,
                       float Kp, float Ki, float Kd,
                       PID_pMode pMode, PID_dMode dMode, PID_iAwMode iAwMode, PID_Action Action) {
    pid->myInput = Input;
    pid->myOutput = Output;
    pid->mySetpoint = Setpoint;

    pid->mode = PID_CONTROL_MANUAL;
    pid->outMin = 0.0f;
    pid->outMax = 255.0f;
    pid->sampleTimeUs = 100000; // 100ms default

    pid->action = Action;
    pid->pmode = pMode;
    pid->dmode = dMode;
    pid->iawmode = iAwMode;

    _QuickPID_SetTunings(pid, Kp, Ki, Kd);

    pid->lastTime = esp_timer_get_time() - pid->sampleTimeUs;
    pid->outputSum = 0.0f;
    pid->lastInput = 0.0f;
    pid->lastError = 0.0f;
    pid->pTerm = 0.0f;
    pid->iTerm = 0.0f;
    pid->dTerm = 0.0f;
}

void QuickPID_Init(QuickPID_t *pid, float *Input, float *Output, float *Setpoint,
                   float Kp, float Ki, float Kd, PID_Action Action) {
    QuickPID_InitFull(pid, Input, Output, Setpoint, Kp, Ki, Kd,
                      PID_P_ON_ERROR, PID_D_ON_MEAS, PID_IAW_CONDITION, Action);
}

void QuickPID_InitDefault(QuickPID_t *pid, float *Input, float *Output, float *Setpoint) {
    // Initialize with zero gains, manual mode, and direct action.
    QuickPID_InitFull(pid, Input, Output, Setpoint, 0, 0, 0,
                      PID_P_ON_ERROR, PID_D_ON_MEAS, PID_IAW_CONDITION, PID_ACTION_DIRECT);
}

void QuickPID_SetMode(QuickPID_t *pid, PID_Control Mode) {
    if (Mode == PID_CONTROL_TOGGLE) {
        Mode = (pid->mode == PID_CONTROL_MANUAL) ? PID_CONTROL_AUTOMATIC : PID_CONTROL_MANUAL;
    }
    
    // Check for bumpless transfer
    bool newAuto = (Mode != PID_CONTROL_MANUAL) && (pid->mode == PID_CONTROL_MANUAL);
    if (newAuto) {
        QuickPID_Initialize(pid);
    }
    pid->mode = Mode;
}

bool QuickPID_Compute(QuickPID_t *pid) {
    int64_t now = esp_timer_get_time();
    int64_t timeChange = now - pid->lastTime;

    if (pid->mode == PID_CONTROL_TIMER) {
        if (timeChange >= pid->sampleTimeUs) {
            pid->mode = PID_CONTROL_AUTOMATIC; // Timer expired, switch to auto
        } else {
            return false; // Timer still running
        }
    }

    if (pid->mode != PID_CONTROL_AUTOMATIC) {
        return false; // Not in automatic, do not compute
    }

    if (timeChange < pid->sampleTimeUs) {
        return false; // Not time to compute yet
    }

    // --- Read inputs ---
    float input = *pid->myInput;
    float setpoint = *pid->mySetpoint;
    float error = setpoint - input;
    float dInput = input - pid->lastInput;
    float dError = error - pid->lastError;

    float output = 0.0f;
    float pTerm = 0.0f;
    float dTerm = 0.0f;

    // --- Proportional Term ---
    if (pid->pmode == PID_P_ON_ERROR) {
        pTerm = pid->kp * error;
    } else if (pid->pmode == PID_P_ON_MEAS) {
        output -= pid->kp * dInput;
    } else if (pid->pmode == PID_P_ON_ERROR_MEAS) {
        pTerm = pid->kp * 0.5f * error;
        output -= pid->kp * 0.5f * dInput;
    }
    pid->pTerm = pTerm;
    output += pTerm;

    // --- Integral Term ---
    // Check anti-windup
    bool integrate = true;
    if (pid->iawmode == PID_IAW_CONDITION) {
        // Only integrate if output is not saturated OR error is pulling away from saturation
        if ((*pid->myOutput >= pid->outMax && error > 0) || (*pid->myOutput <= pid->outMin && error < 0)) {
            integrate = false;
        }
    }

    if (integrate) {
        pid->outputSum += pid->ki * error;
    }

    // Clamp integral sum if iAwClamp is active
    if (pid->iawmode == PID_IAW_CLAMP) {
        pid->outputSum = CONSTRAIN(pid->outputSum, pid->outMin, pid->outMax);
    }

    output += pid->outputSum; // Add integral
    pid->iTerm = pid->outputSum; // Store for getter

    // --- Derivative Term ---
    if (pid->dmode == PID_D_ON_ERROR) {
        dTerm = pid->kd * dError;
    } else if (pid->dmode == PID_D_ON_MEAS) {
        dTerm = -pid->kd * dInput;
    }
    pid->dTerm = dTerm;
    output += dTerm; // Add derivative

    // --- Final Output Clamping and Anti-Windup ---
    if (pid->iawmode == PID_IAW_CLAMP) {
        // Clamp final output. Integral sum was already clamped.
        output = CONSTRAIN(output, pid->outMin, pid->outMax);
    } else {
        // For iAwCondition and iAwOff, perform "back-calculation" anti-windup
        // Clamp the integral sum based on the other terms to prevent windup
        float nonIntegral = output - pid->outputSum;
        pid->outputSum = CONSTRAIN(pid->outputSum, pid->outMin - nonIntegral, pid->outMax - nonIntegral);
        pid->iTerm = pid->outputSum; // Update for getter after constraint
        
        // Recalculate output with constrained integral
        output = nonIntegral + pid->outputSum;
        // Final output clamp (should be redundant if logic is correct, but safe)
        output = CONSTRAIN(output, pid->outMin, pid->outMax);
    }
    
    // --- Write Output and Save State ---
    *pid->myOutput = output;
    pid->lastInput = input;
    pid->lastError = error;
    pid->lastTime = now;

    return true;
}

void QuickPID_SetOutputLimits(QuickPID_t *pid, float Min, float Max) {
    if (Min >= Max) return;
    pid->outMin = Min;
    pid->outMax = Max;

    // Apply constraints immediately
    if (pid->mode != PID_CONTROL_MANUAL) {
        *pid->myOutput = CONSTRAIN(*pid->myOutput, Min, Max);
    }
    pid->outputSum = CONSTRAIN(pid->outputSum, Min, Max);
}

void QuickPID_SetTunings(QuickPID_t *pid, float Kp, float Ki, float Kd) {
    _QuickPID_SetTunings(pid, Kp, Ki, Kd);
}

void QuickPID_SetTuningsFull(QuickPID_t *pid, float Kp, float Ki, float Kd,
                             PID_pMode pMode, PID_dMode dMode, PID_iAwMode iAwMode) {
    pid->pmode = pMode;
    pid->dmode = dMode;
    pid->iawmode = iAwMode;
    _QuickPID_SetTunings(pid, Kp, Ki, Kd);
}

void QuickPID_SetControllerDirection(QuickPID_t *pid, PID_Action Action) {
    if (pid->action != Action) {
        pid->action = Action;
        // Re-apply tunings to invert gains
        _QuickPID_SetTunings(pid, pid->dispKp, pid->dispKi, pid->dispKd);
    }
}

void QuickPID_SetSampleTimeUs(QuickPID_t *pid, uint32_t NewSampleTimeUs) {
    if (NewSampleTimeUs > 0) {
        float ratio = (float)NewSampleTimeUs / (float)pid->sampleTimeUs;
        pid->ki *= ratio;
        pid->kd /= ratio;
        pid->sampleTimeUs = NewSampleTimeUs;
    }
}

void QuickPID_SetProportionalMode(QuickPID_t *pid, PID_pMode pMode) {
    pid->pmode = pMode;
}

void QuickPID_SetDerivativeMode(QuickPID_t *pid, PID_dMode dMode) {
    pid->dmode = dMode;
}

void QuickPID_SetAntiWindupMode(QuickPID_t *pid, PID_iAwMode iAwMode) {
    pid->iawmode = iAwMode;
}

void QuickPID_SetOutputSum(QuickPID_t *pid, float sum) {
    pid->outputSum = CONSTRAIN(sum, pid->outMin, pid->outMax);
    pid->iTerm = pid->outputSum;
}

void QuickPID_Initialize(QuickPID_t *pid) {
    // Sync integral sum with current output
    pid->outputSum = CONSTRAIN(*pid->myOutput, pid->outMin, pid->outMax);
    pid->iTerm = pid->outputSum;
    
    // Sync lastInput with current input
    pid->lastInput = *pid->myInput;
    pid->lastError = *pid->mySetpoint - *pid->myInput;
    
    // Reset timer to allow immediate compute
    pid->lastTime = esp_timer_get_time() - pid->sampleTimeUs; 
}

void QuickPID_Reset(QuickPID_t *pid) {
    pid->pTerm = 0.0f;
    pid->iTerm = 0.0f;
    pid->dTerm = 0.0f;
    pid->outputSum = 0.0f;
    pid->lastInput = 0.0f;
    pid->lastError = 0.0f;
    // Call initialize to sync with current I/O and reset timer
    QuickPID_Initialize(pid); 
}

// --- PID Query Functions ---

float QuickPID_GetKp(QuickPID_t *pid) { return pid->dispKp; }
float QuickPID_GetKi(QuickPID_t *pid) { return pid->dispKi; }
float QuickPID_GetKd(QuickPID_t *pid) { return pid->dispKd; }
float QuickPID_GetPterm(QuickPID_t *pid) { return pid->pTerm; }
float QuickPID_GetIterm(QuickPID_t *pid) { return pid->iTerm; }
float QuickPID_GetDterm(QuickPID_t *pid) { return pid->dTerm; }
float QuickPID_GetOutputSum(QuickPID_t *pid) { return pid->outputSum; }
uint8_t QuickPID_GetMode(QuickPID_t *pid) { return (uint8_t)pid->mode; }
uint8_t QuickPID_GetDirection(QuickPID_t *pid) { return (uint8_t)pid->action; }
uint8_t QuickPID_GetPmode(QuickPID_t *pid) { return (uint8_t)pid->pmode; }
uint8_t QuickPID_GetDmode(QuickPID_t *pid) { return (uint8_t)pid->dmode; }
uint8_t QuickPID_GetAwMode(QuickPID_t *pid) { return (uint8_t)pid->iawmode; }
