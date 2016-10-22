#include "trilaterate.h"


int32_t calculate_beacon_distance(uint8_t ED){
    // Calculate Receiver Input Power
    // P(RF)[dBm] = RSSI_BASE_VAL + 1.03 ⋅ED_LEVEL
    //uint32_t P_RF = RSSI_BASE_VAL + 1.03*ED;

    return 7000;
}

int32_t euclidean_distance(Point *p1, Point *p2){
    Point diff;

    diff.x = p2->x - p1->x;
    diff.y = p2->y - p1->y;

    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

void point_partway(Point *p1, Point *p2, float p, Point *midp){
    Point diff;

    diff.x = p2->x - p1->x;
    diff.y = p2->y - p1->y;

    midp->x = p1->x + (uint32_t)(diff.x * p);
    midp->y = p1->y + (uint32_t)(diff.y * p);
}

void calculate_intersects(Point* beacon_locations[2], uint32_t beacon_distances[2], Intersects *out_intersects){
    Point P;
    float p;

    uint32_t beacon_to_beacon_distance =
        euclidean_distance(beacon_locations[0], beacon_locations[1]);
    uint32_t measured_distance_sum = beacon_distances[0] + beacon_distances[1];

    // TODO: Circles contained within each other
    if(beacon_to_beacon_distance < measured_distance_sum){
        // Double intersect

        /*  a is the distance from the first to the second beacon that meets
         *  the axis that both of the intersects sit on.
         *  p is the proportion of this relative to the total distance
         *  between the two beacon locations.
         *  P is the point that 'a' describes.
         *  h is the distance from point P to the intersects of the beacon's
         *  circles.
         */
        int32_t a = ( beacon_distances[0]*beacon_distances[0]
                    - beacon_distances[1]*beacon_distances[1]
                    + beacon_to_beacon_distance*beacon_to_beacon_distance )
                    / (2 * beacon_to_beacon_distance);
        p = (float)a/(float)beacon_to_beacon_distance;
        point_partway(beacon_locations[0], beacon_locations[1], p, &P);
        uint32_t h = sqrt( beacon_distances[0]*beacon_distances[0] - a * a );

        // Kill me
        out_intersects->I1.x = P.x + h*(beacon_locations[1]->y-beacon_locations[0]->y)/beacon_to_beacon_distance;
        out_intersects->I2.x = P.x - h*(beacon_locations[1]->y-beacon_locations[0]->y)/beacon_to_beacon_distance;
        out_intersects->I1.y = P.y + h*(beacon_locations[1]->x-beacon_locations[0]->x)/beacon_to_beacon_distance;
        out_intersects->I2.y = P.y - h*(beacon_locations[1]->x-beacon_locations[0]->x)/beacon_to_beacon_distance;

    } else {
        // Single / no intersect, so make one up!
        /*  p is the proportion of the distance from the first to second
         *  beacon that the imaginary intersect should be placed at
         */
        p = (float)(beacon_distances[0]) /
            (float)(beacon_distances[0]+beacon_distances[1]);
        point_partway(beacon_locations[1], beacon_locations[2], p, &out_intersects->I1);
        out_intersects->I2 = out_intersects->I1;
    }
}

void trilaterate(uint8_t EDs[]){
    int i;
    int32_t beacon_distances[N_BEACONS];
    Intersects beacon_intersects[N_BEACONS];


    // Convert each ED value to a distance estimate
    for(i=0;i<N_BEACONS;i++){
        beacon_distances[i] = calculate_beacon_distance(EDs[i]);
    }

    for(i=0;i<N_BEACONS;i++){
        #define THIS_INDEX (i)
        #define NEXT_INDEX ((i+1)%N_BEACONS)

        Point* two_beacon_locations[2] = {&beacon_locations[THIS_INDEX], &beacon_locations[NEXT_INDEX]};
        uint32_t two_beacon_distances[2] = {beacon_distances[THIS_INDEX], beacon_distances[NEXT_INDEX]};

        calculate_intersects(two_beacon_locations, two_beacon_distances, &beacon_intersects[i]);
    }

    // TODO: Use intersects to localise
}
