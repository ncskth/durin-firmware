#include <stdint.h>
#include "durin.h"

struct pos_solver_position {
    float x;
    float y;
    float z;
};

void solve_for_position(struct distance_measurement *distances, uint8_t index, struct pos_solver_position* current_pos, uint8_t* fix_type);