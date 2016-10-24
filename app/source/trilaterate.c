#include "trilaterate.h"

// ED to millimeters
int32_t calculate_beacon_distance(uint8_t ED){
    /*  Calculate Receiver Input Power
     *  Method abandoned in favour of experimental
     */
    // P(RF)[dBm] = RSSI_BASE_VAL + 1.03 ⋅ED_LEVEL
    //uint32_t P_RF = RSSI_BASE_VAL + 1.03*ED;

    // Linear approximation constants based on experimental values
    return (int32_t)(MAX(50-ED,1)*500);
}

/*  Find the intersects of the beacon radii, or a suitable substitute if
 *  no intersects exist.
 */
void calculate_intersects(Point* two_beacon_locations[2], int32_t two_beacon_distances[2], Intersects *out_intersects){
    // Temp variables
    Point P;
    float p;

    int32_t beacon_to_beacon_distance =
        euclidean_distance(two_beacon_locations[0], two_beacon_locations[1]);
    int32_t measured_distance_sum = two_beacon_distances[0] + two_beacon_distances[1];

    /*  First two cases for when the radius of one beacon is contained within
     *  the radius of another.
     */
    if(two_beacon_distances[0] > beacon_to_beacon_distance + two_beacon_distances[1]){
        // Second beacon's radius contained within radius of first one

        p = 1 + (float)(two_beacon_distances[0]+two_beacon_distances[1]) /
                (float)(2 * beacon_to_beacon_distance);
        point_partway(two_beacon_locations[0], two_beacon_locations[1], p, &out_intersects->I1);
        out_intersects->I2 = out_intersects->I1;

    } else if(two_beacon_distances[1] > beacon_to_beacon_distance + two_beacon_distances[0]){
        // First beacon's radius contained within radius of second one

        p = 1 + (float)(two_beacon_distances[0]+two_beacon_distances[1]) /
                (float)(2 * beacon_to_beacon_distance);
        point_partway(two_beacon_locations[1], two_beacon_locations[0], p, &out_intersects->I1);
        out_intersects->I2 = out_intersects->I1;

    } else if(beacon_to_beacon_distance < measured_distance_sum){
        /*  Code to calculate the intersects of two circles
         *  Based on algorithm described at:
         *  http://paulbourke.net/geometry/circlesphere/
         *  Section: Intersection of two circles
         */

        /*  - a is the distance from the first to the second beacon that meets
         *  the axis that both of the intersects sit on.
         *  - p is the proportion of this relative to the total distance
         *  between the two beacon locations.
         *  - P is the point that 'a' describes.
         *  - h is the distance from point P to the intersects of the beacon's
         *  circles.
         */

        int32_t a = ( two_beacon_distances[0]*two_beacon_distances[0]
                    - two_beacon_distances[1]*two_beacon_distances[1]
                    + beacon_to_beacon_distance*beacon_to_beacon_distance )
                    / (2 * beacon_to_beacon_distance);
        p = (float)a/(float)beacon_to_beacon_distance;
        point_partway(two_beacon_locations[0], two_beacon_locations[1], p, &P);
        int32_t h = SquareRoot( two_beacon_distances[0]*two_beacon_distances[0] - a * a );

        // Calculating the two intersects' x and y values
        out_intersects->I1.x = P.x + h*(two_beacon_locations[1]->y-two_beacon_locations[0]->y)/beacon_to_beacon_distance;
        out_intersects->I2.x = P.x - h*(two_beacon_locations[1]->y-two_beacon_locations[0]->y)/beacon_to_beacon_distance;
        out_intersects->I1.y = P.y - h*(two_beacon_locations[1]->x-two_beacon_locations[0]->x)/beacon_to_beacon_distance;
        out_intersects->I2.y = P.y + h*(two_beacon_locations[1]->x-two_beacon_locations[0]->x)/beacon_to_beacon_distance;

    } else {
        /*  Single / no intersect, so pretend there is one between them.
         *  - p is the proportion of the distance from the first to second
         *  beacon that the imaginary intersect should be placed at
         */

        p = (float)(two_beacon_distances[0]) /
            (float)(two_beacon_distances[0]+two_beacon_distances[1]);
        point_partway(two_beacon_locations[0], two_beacon_locations[1], p, &out_intersects->I1);
        out_intersects->I2 = out_intersects->I1;
    }
}

void position_average(uint8_t n_points, Point in_points[], Point *point_average){
    point_average->x = 0;
    point_average->y = 0;

    for(int i=0;i<n_points;i++){
        point_average->x += in_points[i].x / n_points;
        point_average->y += in_points[i].y / n_points;
    }
}

void trilaterate(uint8_t EDs[], Point *position_estimate, Point *position_out){
    int i;
    int32_t two_beacon_distances[N_BEACONS];
    Intersects beacon_intersects[N_BEACONS];
    Point closest_points[N_BEACONS];

    // Convert each ED value to a distance estimate
    for(i=0;i<N_BEACONS;i++){
        two_beacon_distances[i] = calculate_beacon_distance(EDs[i]);
    }

    // Find the intersects of the beacon distances
    for(i=0;i<N_BEACONS;i++){
        #define THIS_INDEX (i)
        #define NEXT_INDEX ((i+1)%N_BEACONS)

        Point* two_beacon_locations[2] = {&beacon_locations[THIS_INDEX], &beacon_locations[NEXT_INDEX]};
        int32_t two_two_beacon_distances[2] = {two_beacon_distances[THIS_INDEX], two_beacon_distances[NEXT_INDEX]};

        calculate_intersects(two_beacon_locations, two_two_beacon_distances, &beacon_intersects[THIS_INDEX]);
    }

//#define TRI_DEBUG_PRINT
#ifdef TRI_DEBUG_PRINT
    char txt_buffer[256];
    for(i=0;i<N_BEACONS;i++) {
        sprintf(txt_buffer, "%d: (%ld,%ld) (%ld,%ld)\n",i,
            beacon_intersects[i].I1.x,beacon_intersects[i].I1.y,beacon_intersects[i].I2.x,beacon_intersects[i].I2.y);
        USB_print(txt_buffer);
        delay_ms(100);
    }
#endif

    /*  Choose between the two intersects based on which are closest to the
     *  position estimate. This is much lower complexity than something like a
     *  nearest neighbors algorithm or similar, but requires an estimate of
     *  the position as priori
     */
    for(i=0;i<N_BEACONS;i++){
        if((beacon_intersects[i].I1.x == beacon_intersects[i].I2.x &&
            beacon_intersects[i].I1.y == beacon_intersects[i].I2.y) ||
            euclidean_distance(&beacon_intersects[i].I1, position_estimate) <
            euclidean_distance(&beacon_intersects[i].I2, position_estimate))
        {
            closest_points[i] = beacon_intersects[i].I1;
        } else {
            closest_points[i] = beacon_intersects[i].I2;
        }
    }

    /*  Estimate the position based on a simple average of the interscetion
     *  points
     */
    position_average(N_BEACONS, closest_points, position_out);
}
