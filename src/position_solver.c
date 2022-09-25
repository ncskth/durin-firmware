#include <math.h>

#include "position_solver.h"
#include "uwb_definitions.h"

#define ITERATIONS 15

//gauss-newton but with for loops instead of matrices (i think)
void solve_for_position(struct distance_measurement *distances, uint8_t index, struct pos_solver_position* current_pos, uint8_t* fix_type) {
    //read metadata
    printf("\n\n----------------------------------\n\n\n");
    uint8_t num_2d_nodes = 0;
    uint8_t num_3d_nodes = 0;
    uint8_t num_useful_nodes = 0;
    for (uint8_t i = 0; i < index; i++) {
        uint8_t flags = distances[i].flags;
        if (flags & UWB_HAS_2D_POSITION_BITMASK) {
            num_2d_nodes++;
            distances[i].position_z = 0;
        } else
        if (flags & UWB_HAS_3D_POSITION_BITMASK) {
            num_3d_nodes++;
        }
        printf("node: %d distance: %f\n", distances[i].id, distances[i].distance / 1000.0);
    }

    if (num_3d_nodes >= 4) {
        *fix_type = 3;
        num_useful_nodes = num_3d_nodes;
    } else
    if (num_2d_nodes >= 3) {
        *fix_type = 2;
        num_useful_nodes = num_2d_nodes;
    } else {
        *fix_type = 0;
        return;
    }

    struct pos_solver_position step = {0.0, 0.0, 0.0};
    // run algorithm
    for (uint8_t i = 0; i < ITERATIONS; i++) {
        step.x = 0;
        step.y = 0;
        step.z = 0;

        for (uint8_t j = 0; j < index; j++) {
            if (*fix_type == 2 && !(distances[i].flags & UWB_HAS_2D_POSITION_BITMASK)) {
                continue;
            }
            if (*fix_type == 3 && !(distances[i].flags & UWB_HAS_3D_POSITION_BITMASK)) {
                continue;
            }

            struct pos_solver_position difference;
            struct pos_solver_position wanted_step;
            difference.x = (current_pos->x - distances[j].position_x) / 1000.0;
            difference.y = (current_pos->y - distances[j].position_y) / 1000.0;
            difference.z = (current_pos->z - distances[j].position_z) / 1000.0;
            float magnitude = sqrtf(difference.x * difference.x + difference.y * difference.y + difference.z * difference.z);

            //not sure if it's supposed to be 2 here. It should come from the Jacobian i think
            wanted_step.x = 2 * difference.x / magnitude * (distances[j].distance / 1000.0 - magnitude);
            wanted_step.y = 2 * difference.y / magnitude * (distances[j].distance / 1000.0 - magnitude);
            wanted_step.z = 2 * difference.z / magnitude * (distances[j].distance / 1000.0 - magnitude);

            step.x += wanted_step.x / num_useful_nodes;
            step.y += wanted_step.y / num_useful_nodes;
            step.z += wanted_step.z / num_useful_nodes;
        }
        current_pos->x += step.x * 1000;
        current_pos->y += step.y * 1000;
        current_pos->z += step.z * 1000;
    }
    
    float error = sqrtf(step.x * step.x + step.y * step.y + step.z * step.z);
    //algorithm is unstable
    if (error > 0.2) {
        *fix_type = 0;
    }
}