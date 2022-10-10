#include <math.h>

#include "position_solver.h"
#include "uwb_definitions.h"


#define ITERATIONS 15

// something is borked with volatile structs so here goes global variables. woooo!!!
//gauss-newton but with for loops instead of matrices (i think)
void solve_for_position(struct distance_measurement distances[], uint8_t index, struct pos_solver_position* current_pos, uint8_t* fix_type) {
    //read metadata
    printf("distances ------ \n");
    uint8_t num_useful_nodes = 0;
    for (uint8_t i = 0; i < index; i++) {
        uint8_t flags = distances[i].flags;
        if (flags & UWB_HAS_3D_POSITION_BITMASK) {
            num_useful_nodes++;
        }
        printf("%d \t %d \t %d \t %d \t %d\n", distances[i].id, distances[i].position_x, distances[i].position_y, distances[i].position_z, distances[i].distance);
    }

    if (num_useful_nodes >= 4) {
        *fix_type = 3;
    } else {
        *fix_type = 0;
        return;
    }

    struct pos_solver_position step = {0.0, 0.0, 0.0};
    // run algorithm
    for (uint8_t i = 0; i < ITERATIONS; i++) {
        step.x = 0.0;
        step.y = 0.0;
        step.z = 0.0;

        for (uint8_t j = 0; j < index; j++) {
            if (!(distances[j].flags & UWB_HAS_3D_POSITION_BITMASK)) {
                continue;
            }
            struct pos_solver_position difference;
            struct pos_solver_position wanted_step;
            difference.x = current_pos->x - distances[j].position_x / 1000.0;
            difference.y = current_pos->y - distances[j].position_y / 1000.0;
            difference.z = current_pos->z - distances[j].position_z / 1000.0;
            // printf("%f %f %f\n", difference.x,difference.y,difference.z);
            if (difference.x == 0 && difference.y == 0 && difference.z == 0) {
                continue;
            }
            float magnitude = sqrtf(difference.x * difference.x + difference.y * difference.y + difference.z * difference.z);
            //not sure if it's supposed to be 2 here. It should come from the Jacobian i think
            wanted_step.x = difference.x / magnitude * (distances[j].distance / 1000.0 - magnitude);
            wanted_step.y = difference.y / magnitude * (distances[j].distance / 1000.0 - magnitude);
            wanted_step.z = difference.z / magnitude * (distances[j].distance / 1000.0 - magnitude);
            step.x += wanted_step.x / num_useful_nodes;
            step.y += wanted_step.y / num_useful_nodes;
            step.z += wanted_step.z / num_useful_nodes;
        }
        current_pos->x += step.x;
        current_pos->y += step.y;
        current_pos->z += step.z;
    }
    
    float error = sqrtf(step.x * step.x + step.y * step.y + step.z * step.z);
    //algorithm is unstable
    if (error > 0.2) {
        *fix_type = 0;
    }
}