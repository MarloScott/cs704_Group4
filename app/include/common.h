#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define SQUARE(x) (x*x)

typedef struct {
    int32_t x;
    int32_t y;
} Point;

uint32_t SquareRoot(uint32_t a_nInput);
int32_t euclidean_distance(Point *p1, Point *p2);
void point_partway(Point *p1, Point *p2, float p, Point *midp);

#endif // COMMON_H
