#ifndef TRILATERATE_H
#define TRILATERATE_H

#include <stdint.h>
#include "math.h"

//PHY Modes
#define BPSK300 -100
#define BPSK600 -100
#define OQPSK400 -98
#define OQPSK1000Sin -98
#define OQPSK1000Cos -97

#define RSSI_BASE_VAL BPSK300

typedef struct {
    int32_t x;
    int32_t y;
} Point;

typedef struct {
    Point I1;
    Point I2;
} Intersects;

//Beacon Data
#define X_S 0
#define Y_S 0
#define N_BEACONS 4
static Point beacon_locations[N_BEACONS] =
{
    {X_S+10600, Y_S+1400      },
    {X_S+0,     Y_S+2500      },
    {X_S+0,     Y_S+2500+12300},
    {X_S+10600, Y_S+11400     }
};

static Point start_location = {0,0};

// Function Prototypes
int32_t calculate_beacon_distance(uint8_t ED);
int32_t euclidean_distance(Point *p1, Point *p2);
void point_partway(Point *p1, Point *p2, float p, Point *midp);
void trilaterate(uint8_t EDs[]);

#endif // TRILATERATE_H
