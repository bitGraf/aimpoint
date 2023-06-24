#pragma once
#include "defines.h"

#include <cstring>

struct rigid_body_state {
    laml::Vec3_highp position;
    laml::Vec3_highp velocity;

    laml::Quat_highp orientation;
    laml::Vec3_highp ang_velocity;
};

struct rigid_body_derivative {
    laml::Vec3_highp velocity;
    laml::Vec3_highp acceleration;

    laml::Quat_highp spin;
    laml::Vec3_highp ang_acceleration;
};

struct simulation_body {
    rigid_body_state state;
    rigid_body_derivative derivative;

    void set_mass(double mass);
    void set_inv_mass(double inv_mass);

    void set_inertia(double I1, double I2, double I3);
    void set_inv_inertia(double inv_I1, double inv_I2, double inv_I3);

    void set_state(laml::Vec3_highp position, laml::Vec3_highp velocity, 
                   laml::Quat_highp orientation, laml::Vec3_highp ang_velocity);
    //void calc_secondary_states();

    rigid_body_derivative calc_derivative(double t, const rigid_body_state* state);
    void base_major_step(double t, double dt);
    void base_minor_step(double t, double dt, rigid_body_derivative* minor_derivative, rigid_body_state* minor_state);
    void integrate_states(double t, double dt);

    virtual void major_step(double t, double dt);
    virtual void minor_step(double t, double dt, rigid_body_derivative* minor_derivative, rigid_body_state* minor_state);

    void apply_force(laml::Vec3_highp force, laml::Vec3_highp location);

    virtual laml::Vec3_highp force_func(const rigid_body_state* at_state, double t);
    virtual laml::Vec3_highp moment_func(const rigid_body_state* at_state, double t);

public:
    // secondary states
    //laml::Vec3_highp momentum;
    //laml::Quat_highp spin; // q dot
    //laml::Vec3_highp ang_momentum;

    //double linear_KE;
    //double rotational_KE;

    // constants
    double mass;
    double inv_mass;
    laml::Vec3_highp inertia; // principal interias in body-frame
    laml::Vec3_highp inv_inertia;

private:
    // step accumulation vars
    laml::Vec3_highp net_force;
    laml::Vec3_highp net_moment;
};