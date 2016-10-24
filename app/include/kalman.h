#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include "common.h"

typedef struct{
    Point mean;
    int32_t std; // Standard deviation
} Guassian2d;

void kalman2d(Guassian2d* prediction, Guassian2d* measurement, float kal_const, Guassian2d* fused);

#endif // KALMAN_H
