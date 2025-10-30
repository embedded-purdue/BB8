#include <stdbool.h>
#include <stdint.h>
#include "bno055.h"  // for bno055_sample_t

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

    float errorAngle; // current error (e.g., pitch or roll angle)
    float integral;   // integral accumulator
    float derivative; // measured or computed derivative (e.g., gx in rad/s)

    //float time;       // optional elapsed time accumulator
} PID;

double calculateLR(bno055_sample_t s, PID pidFB, float *);
double calculateFB(bno055_sample_t s, PID pidLR, float *);



#ifdef __cplusplus
}
#endif


