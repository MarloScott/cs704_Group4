#include "../app/include/trilaterate.h"
#include <stdio.h>

int main(void){
    uint8_t ED = 50;
    uint32_t t32 = calculate_beacon_distance(ED);
    printf("ED:%d -> Dist: %d",ED,t32);
}
