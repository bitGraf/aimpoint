#include "defines.h"

#include <cstring>

struct rigid_body_state {
    // primary
    laml::Vec3 position;
    laml::Vec3 velocity;
    laml::Quat orientation;
    laml::Vec3 ang_velocity;

    // secondary
    laml::Vec3 momentum;

    laml::Quat spin; // q dot
    laml::Vec3 ang_momentum;

    float linear_KE;
    float rotational_KE;

    // constants
    float mass;
    float inv_mass;

    laml::Vec3 inertia; // principal interias in body-frame
    laml::Vec3 inv_inertia;

    void recalculate() {
        momentum = velocity * mass;
        ang_momentum = ang_velocity * inertia; // component-wise operator

        orientation = laml::normalize(orientation);

        laml::Quat q(ang_velocity.x, ang_velocity.y, ang_velocity.z, 0.0f);
        spin = 0.5f * laml::mul(q, orientation);

        linear_KE     = 0.5 * laml::dot(momentum, velocity);
        rotational_KE = 0.5 * laml::dot(ang_momentum, ang_velocity);
    }
};

struct rigid_body_derivative {
    laml::Vec3 velocity;
    laml::Vec3 acceleration;

    laml::Quat spin;
    laml::Vec3 ang_acceleration;
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