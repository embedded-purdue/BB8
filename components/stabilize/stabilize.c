#include "stabilize.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"



double calculateFB(bno055_sample_t s) {
    PID pidFB; //forward backward

    pidFB.errorAngle = s.pitch; 
    pidFB.derivative = s.gx;
    pidFB.integral += s.pitch * pidFB.dt; 

    pidFB.response = Kp * pidFB.errorAngle + Ki * pidFB.integral + Kd * pidFB.derivative;

    pidFB.time += dt;
    return pidFB.response;
}

double calculateLR(bno055_sample_t s) {
    PID pidLR; // left right  

    pidLR.errorAngle = s.pitch; 
    pidLR.derivative = s.gx;
    pidLR.integral += s.pitch * pidLR.dt; 

    pidLR.response = Kp * pidLR.errorAngle + Ki * pidLR.integral + Kd * pidLR.derivative;

    pidLR.time += dt;
    return pidLR.response;
}

double test(double x) {
    return x * 2;
}
