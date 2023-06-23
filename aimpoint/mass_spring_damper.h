#include "physics.h"

struct mass_spring_damper : public simulation_body {
    mass_spring_damper();

    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t) override;

    laml::Vec3_highp neutral_point;
    double spring_constant;
    double damping_constant;
};