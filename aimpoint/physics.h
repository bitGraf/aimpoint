#include "defines.h"

#include <cstring>

const uint32 num_objects = 32;
struct physics_world_state {
    double x[num_objects];
    double y[num_objects];
    double z[num_objects];

    physics_world_state() {
        memset(x, 0, sizeof(double)*num_objects);
        memset(y, 0, sizeof(double)*num_objects);
        memset(z, 0, sizeof(double)*num_objects);
    }
};

struct physics_world {
    void integrate_states(physics_world_state* state, double t, double dt);
    physics_world_state interpolate_states(physics_world_state previous, physics_world_state current, double f);
};