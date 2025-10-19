#include <stdbool.h>
#include <stdint.h>
#include "bno055.h"  // for bno055_sample_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Kp;         // proportional gain
    float Ki;         // integral gain
    float Kd;         // derivative gain
    float dt;         // sample period (seconds)

    float errorAngle; // current error (e.g., pitch or roll angle)
    float integral;   // integral accumulator
    float derivative; // measured or computed derivative (e.g., gx in rad/s)
    double response;  // last computed control output

    //float time;       // optional elapsed time accumulator
} PID;

double calculateLR(bno055_sample_t s);
double calculateFB(bno055_sample_t s);

double test(double x);


#ifdef __cplusplus
}
#endif


