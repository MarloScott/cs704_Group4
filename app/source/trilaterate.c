#include "trilaterate.h"

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

    midp->x = p1->x + (diff.x * p);
    midp->y = p1->y + (diff.y * p);
}

int32_t calculate_beacon_distance(uint8_t ED){
    // Calculate Receiver Input Power
    // P(RF)[dBm] = RSSI_BASE_VAL + 1.03 ⋅ED_LEVEL
    uint8_t P_RF = RSSI_BASE_VAL + 1.03*ED;

    return -1;
}

void trilaterate(uint8_t EDs[]){
    int i;
    int32_t beacon_distances[N_BEACONS];
    Intersects beacon_intersects[N_BEACONS];

    float p;
    Point P;

    #define THIS_INDEX (i)
    #define NEXT_INDEX ((i+1)%N_BEACONS)

    #define THIS_BEACON_LOC beacon_locations[THIS_INDEX]
    #define NEXT_BEACON_LOC beacon_locations[NEXT_INDEX]
    #define THIS_BEACON_DIST beacon_distances[THIS_INDEX]
    #define NEXT_BEACON_DIST beacon_distances[NEXT_INDEX]
    #define THIS_BEACON_INT beacon_intersects[THIS_INDEX]

    // Convert each ED value to a distance estimate
    for(i=0;i<N_BEACONS;i++){
        THIS_BEACON_DIST = calculate_beacon_distance(EDs[i]);
    }

    for(i=0;i<N_BEACONS;i++){
        uint32_t beacon_to_beacon_distance =
            euclidean_distance(&THIS_BEACON_LOC, &NEXT_BEACON_LOC);
        uint32_t measured_distance_sum = THIS_BEACON_DIST + NEXT_BEACON_DIST;

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
            int32_t a = ( THIS_BEACON_DIST*THIS_BEACON_DIST
                        - NEXT_BEACON_DIST*NEXT_BEACON_DIST
                        + beacon_to_beacon_distance*beacon_to_beacon_distance )
                        / (2 * beacon_to_beacon_distance);
            p = (float)a/(float)beacon_to_beacon_distance;
            point_partway(&THIS_BEACON_LOC, &NEXT_BEACON_LOC, p, &P);
            uint32_t h = sqrt( THIS_BEACON_DIST*THIS_BEACON_DIST - a * a );

            // Kill me
            THIS_BEACON_INT.I1.x = P.x + h*(NEXT_BEACON_LOC.y-THIS_BEACON_LOC.y)/beacon_to_beacon_distance;
            THIS_BEACON_INT.I2.x = P.x - h*(NEXT_BEACON_LOC.y-THIS_BEACON_LOC.y)/beacon_to_beacon_distance;
            THIS_BEACON_INT.I1.y = P.y + h*(NEXT_BEACON_LOC.x-THIS_BEACON_LOC.x)/beacon_to_beacon_distance;
            THIS_BEACON_INT.I2.y = P.y - h*(NEXT_BEACON_LOC.x-THIS_BEACON_LOC.x)/beacon_to_beacon_distance;
        } else {
            // Single / no intersect, so make one up!

            /*  p is the proportion of the distance from the first to second
             *  beacon that the imaginary intersect should be placed at
             */
            p = (float)(THIS_BEACON_DIST) /
                (float)(THIS_BEACON_DIST+NEXT_BEACON_DIST);
            point_partway(&THIS_BEACON_LOC, &NEXT_BEACON_LOC, p, &THIS_BEACON_INT.I1);
            THIS_BEACON_INT.I2 = THIS_BEACON_INT.I1;
        }
    }

    // TODO: Use intersects to localise
}
