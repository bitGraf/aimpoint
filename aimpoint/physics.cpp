#include "physics.h"

void physics_world::integrate_states(physics_world_state* state, double t, double dt) {

}

physics_world_state physics_world::interpolate_states(physics_world_state previous, physics_world_state current, double f) {
    // No Interp!
    return current;
}