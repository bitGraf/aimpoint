#include "physics.h"

struct rocket : public simulation_body {
    rocket();

    virtual void major_step(double t, double dt) override;

    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t) override;
    //virtual laml::Vec3_highp moment_func(const rigid_body_state& at_state, double t) override;

    double tof;
    double pitch;
};