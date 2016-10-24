#include "common.h"

/*  Integer Square root Function taken from from:
 *  http://stackoverflow.com/a/1101217
 *  Based on this algorithm:
 *  https://web.archive.org/web/20120306040058/http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
 */
uint32_t SquareRoot(uint32_t a_nInput)
{
    uint32_t op  = a_nInput;
    uint32_t res = 0;
    uint32_t one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type

    // "one" starts at the highest power of four <= than the argument.
    while (one > op) {
        one >>= 2;
    }

    while (one != 0) {
        if (op >= res + one) {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }
    return res;
}

int32_t euclidean_distance(Point *p1, Point *p2){
    Point diff;

    diff.x = p2->x - p1->x;
    diff.y = p2->y - p1->y;

    return SquareRoot(diff.x*diff.x + diff.y*diff.y);
}

// Calculates point that is proportion p of the distance from p1 to p2
void point_partway(Point *p1, Point *p2, float p, Point *midp){
    Point diff;

    diff.x = p2->x - p1->x;
    diff.y = p2->y - p1->y;

    midp->x = p1->x + (int32_t)(diff.x * p);
    midp->y = p1->y + (int32_t)(diff.y * p);
}
