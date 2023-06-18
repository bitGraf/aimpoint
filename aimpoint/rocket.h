#include "physics.h"

struct rocket : public simulation_body {
    rocket();

    virtual void major_step(double dt) override;
    virtual void minor_step(double dt) override;

    virtual laml::Vec3 force_func(const rigid_body_state& at_state, double t) override;
    virtual laml::Vec3 moment_func(const rigid_body_state& at_state, double t) override;

    double tof;
    double pitch;
};