#include "t_bar.h"

#include "log.h"

t_bar::t_bar() {
    set_mass(1.0);
    set_inertia(1.0, 1.0, 1.0);
}

laml::Vec3_highp t_bar::moment_func(const rigid_body_state* at_state, double t) {
    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
}