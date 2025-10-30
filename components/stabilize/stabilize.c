#include "stabilize.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

float Kp = 1;         // proportional gain
float Ki = 0;         // integral gain
float Kd = 1;         // derivative gain
float dt = .1;         // sample period (seconds)


double calculateFB(bno055_sample_t s, PID pidFB, float *integral) {

    pidFB.errorAngle = s.pitch; 
    pidFB.derivative = s.gy;
    *integral += s.pitch * dt; 

    double response = Kp * pidFB.errorAngle + Ki * *integral + Kd * pidFB.derivative;

    return response;
}

double calculateLR(bno055_sample_t s, PID pidLR, float *integral) {

    pidLR.errorAngle = s.roll; 
    pidLR.derivative = s.gy;
    *integral += s.pitch * dt; 

    double response = Kp * pidLR.errorAngle + Ki * *integral + Kd * pidLR.derivative;

    return response;
}

