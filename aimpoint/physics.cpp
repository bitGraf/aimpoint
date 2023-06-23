#include "physics.h"

#include "log.h"

void simulation_body::set_mass(double new_mass) {

    mass = new_mass;
    if (new_mass == 0.0) {
        inv_mass = 1.0e9;
    } else {
        inv_mass = 1.0 / new_mass;
    }
}

void simulation_body::set_inv_mass(double new_inv_mass) {

    inv_mass = new_inv_mass;
    if (new_inv_mass == 0.0) {
        mass = 1.0e9;
    } else {
        mass = 1.0 / new_inv_mass;
    }
}

//void simulation_body::set_inertia(double I1, double I2, double I3) {
//    inertia.x = I1;
//    inertia.y = I2;
//    inertia.z = I3;
//
//    if (I1 == 0.0) {
//        inv_inertia.x = 1.0e9;
//    } else {
//        inv_inertia.x = 1.0/I1;
//    }
//    if (I2 == 0.0) {
//        inv_inertia.y = 1.0e9;
//    } else {
//        inv_inertia.y = 1.0/I2;
//    }
//    if (I3 == 0.0) {
//        inv_inertia.z = 1.0e9;
//    } else {
//        inv_inertia.z = 1.0/I3;
//    }
//}
//void simulation_body::set_inv_inertia(double inv_I1, double inv_I2, double inv_I3) {
//    inv_inertia.x = inv_I1;
//    inv_inertia.y = inv_I2;
//    inv_inertia.z = inv_I3;
//
//    if (inv_I1 == 0.0) {
//        inertia.x = 1.0e9;
//    } else {
//        inertia.x = 1.0/inv_I1;
//    }
//    if (inv_I2 == 0.0) {
//        inertia.y = 1.0e9;
//    } else {
//        inertia.y = 1.0/inv_I2;
//    }
//    if (inv_I3 == 0.0) {
//        inertia.z = 1.0e9;
//    } else {
//        inertia.z = 1.0/inv_I3;
//    }
//}

void simulation_body::set_state(laml::Vec3_highp position, laml::Vec3_highp velocity, 
                                laml::Quat_highp orientation, laml::Vec3_highp ang_velocity) {
    state.position = position;
    state.velocity = velocity;
    //state.orientation = orientation;
    //state.ang_velocity = ang_velocity;

    //calc_secondary_states();

    derivative.velocity = state.velocity;
    derivative.acceleration = laml::Vec3_highp(0.0, 0.0, 0.0);

    //derivative.spin = spin; // from calc_secondary_states()
    //derivative.ang_acceleration = laml::Vec3_highp(0.0f, 0.0f, 0.0f);
}

//void simulation_body::calc_secondary_states() {
//    momentum = state.velocity * mass;
//    //ang_momentum = state.ang_velocity * inertia; // component-wise operator
//    
//    //state.orientation = laml::normalize(state.orientation);
//    
//    //laml::Quat_highp q(state.ang_velocity.x, state.ang_velocity.y, state.ang_velocity.z, 0.0);
//    //spin = 0.5f * laml::mul(q, state.orientation);
//    
//    linear_KE     = 0.5 * laml::dot(momentum, state.velocity);
//    //rotational_KE = 0.5 * laml::dot(ang_momentum, state.ang_velocity);
//}

rigid_body_derivative simulation_body::calc_derivative(double t_n, const rigid_body_state* state_n) {
    rigid_body_derivative deriv;
    deriv.velocity = state_n->velocity;
    deriv.acceleration = (net_force + force_func(state_n, t_n))*inv_mass;

    //laml::Quat q(state_n.ang_velocity.x, state_n.ang_velocity.y, state_n.ang_velocity.z, 0.0f);
    //laml::Quat new_spin = 0.5f * laml::mul(q, state_n.orientation);
    //derivative.spin = new_spin;
    //derivative.ang_acceleration = (net_moment + moment_func(state_n, t))*inv_inertia;

    return deriv;
}

void simulation_body::major_step(double t, double dt) {
    state.position = state.position + derivative.velocity*dt;
    state.velocity = state.velocity + derivative.acceleration*dt;

    // reset
    net_force = laml::Vec3_highp(0.0);
    //net_moment = laml::Vec3_highp(0.0);

    spdlog::trace("[{0:.5f}] major timestep  x={1:.5f}   v={2:.2f}   a={3:.2f}", t, state.position.x, state.velocity.x, ((net_force + force_func(&state, t))*inv_mass).x);
}

void simulation_body::minor_step(double t, double dt, rigid_body_derivative* minor_derivative, rigid_body_state* minor_state) {
    minor_state->position = minor_state->position + minor_derivative->velocity*dt;
    minor_state->velocity = minor_state->velocity + minor_derivative->acceleration*dt;

    //spdlog::trace("[{0:.5f}] minor timestep  x={1:.5f}   v={2:.2f}   a={3:.2f}", t, minor_state->position.x, minor_state->velocity.x, ((net_force + force_func(minor_state, t))*inv_mass).x);
}

void simulation_body::apply_force(laml::Vec3_highp force, laml::Vec3_highp location) {
    net_force = net_force + force;

    // TODO: calculate moment
    //       is location in body-frame or world-frame?
}

void simulation_body::integrate_states(double t, double dt) {
    const uint8 integration_mode = 5;

    switch(integration_mode) {
        case 5: {
            rigid_body_state minor_state = state;
            // ode3 - Bogacki–Shampine method
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            minor_step(t + dt/2.0, dt/2.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt/2.0, &minor_state); // K2 = f(t_n + 1/2*dt, y_n + 1/2*dt*K1)

            // substep 2
            minor_state = state;
            minor_step(t + 3*dt/4.0, 3.0*dt/4.0, &K2, &minor_state);
            rigid_body_derivative K3 = calc_derivative(t + 3.0*dt/4.0, &minor_state); // K3 = f(t_n + 3/4*dt, y_n + 3/4*dt*K2)

            // substep 3
            derivative.velocity     = 1.0 / 9.0 * (2.0*K1.velocity     + 3.0*K2.velocity     + 4.0*K3.velocity);
            derivative.acceleration = 1.0 / 9.0 * (2.0*K1.acceleration + 3.0*K2.acceleration + 4.0*K3.acceleration);
            major_step(t + dt, dt);
            //calc_secondary_states();
        } break;
    }
}

laml::Vec3_highp simulation_body::force_func(const rigid_body_state* at_state, double t) {
    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
}

//laml::Vec3_highp simulation_body::moment_func(const rigid_body_state& at_state, double t) {
//    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
//}