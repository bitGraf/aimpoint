#include "mass_spring_damper.h"

#include "log.h"

mass_spring_damper::mass_spring_damper() {
    set_mass(1.0);

    spring_constant = 1.0;
    damping_constant = 1.0;
}

laml::Vec3_highp mass_spring_damper::force_func(const rigid_body_state* at_state, double t) {
    return -spring_constant*(at_state->position - neutral_point) - damping_constant*at_state->velocity;
}