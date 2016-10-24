#include "kalman.h"

void kalman2d(Guassian2d* prediction, Guassian2d* measurement, int32_t kal_const, Guassian2d* fused){
    /*  Calculate mean in both x and y
     */

    fused->mean.x =
        prediction->mean.x +
        ((kal_const * prediction->std) /
        (SQUARE(kal_const) * prediction->std + measurement->std)) *
        (measurement->mean.x - kal_const * prediction->mean.x);

    fused->mean.y =
        prediction->mean.y +
        ((kal_const * prediction->std) /
        (SQUARE(kal_const) * prediction->std + measurement->std)) *
        (measurement->mean.y - kal_const * prediction->mean.y);

    /*  Standard deviation of fused data is always a function of just the input
     *  standard deviations, which probably remain constant.
     */

    fused->std =
        prediction->std -
        ((kal_const * prediction->std) /
        (SQUARE(kal_const) * prediction->std + measurement->std)) *
        kal_const * prediction->std;
}
