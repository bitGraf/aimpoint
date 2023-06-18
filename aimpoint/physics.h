#include "defines.h"

#include <cstring>

struct rigid_body_state {
    // primary
    laml::Vec3 position;
    laml::Vec3 momentum;

    laml::Quat orientation;
    laml::Vec3 ang_momentum;

    // secondary
    laml::Vec3 velocity;

    laml::Quat spin; // q dot
    laml::Vec3 ang_velocity;

    // constants
    float mass;
    float inv_mass;

    laml::Vec3 inertia; // principal interias in body-frame
    laml::Vec3 inv_inertia;

    void recalculate() {
        velocity = momentum * inv_mass;

        laml::Vec3 _I_(inertia.z-inertia.y, inertia.x-inertia.z, inertia.y-inertia.x);
        ang_velocity = ang_momentum * inv_inertia; // component-wise operator
        orientation = laml::normalize(orientation);
        laml::Quat q(ang_velocity.x, ang_velocity.y, ang_velocity.z, 0.0f);
        spin = 0.5f * laml::mul(q, orientation);
    }
};

struct rigid_body_derivative {
    laml::Vec3 velocity;
    laml::Vec3 force;

    laml::Quat spin;
    laml::Vec3 moment;
};

struct rigid_body_render_state {
    laml::Vec3 position;
    laml::Quat orientation;
};

struct simulation_body {
    rigid_body_state state;
    rigid_body_derivative derivative;

    void set_mass(double mass);
    void set_inv_mass(double inv_mass);

    void set_inertia(double I1, double I2, double I3);
    void set_inv_inertia(double inv_I1, double inv_I2, double inv_I3);

    void set_state(laml::Vec3 position, laml::Vec3 velocity, 
                   laml::Quat orientation, laml::Vec3 ang_velocity);

    virtual void major_step(double dt);
    virtual void minor_step(double dt);

    void apply_force(laml::Vec3 force, laml::Vec3 location);

    rigid_body_derivative calc_new_deriv(double t, double dt, const rigid_body_derivative* const with_deriv);
    void integrate_states(double t, double dt);

    virtual laml::Vec3 force_func(const rigid_body_state& at_state, double t);
    virtual laml::Vec3 moment_func(const rigid_body_state& at_state, double t);

private:
    laml::Vec3 net_force;
    laml::Vec3 net_moment;
};