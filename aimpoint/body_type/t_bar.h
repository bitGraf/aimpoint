#include "physics.h"

struct t_bar : public simulation_body {
    t_bar();

    virtual laml::Vec3_highp moment_func(const rigid_body_state* at_state, double t) override;
};