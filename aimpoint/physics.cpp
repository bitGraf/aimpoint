#include "physics.h"

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

void simulation_body::calc_derivative(double t_n, const rigid_body_state* state_n) {
    derivative.velocity = state_n->velocity;
    derivative.acceleration = (net_force + force_func(state_n, t_n))*inv_mass;

    //laml::Quat q(state_n.ang_velocity.x, state_n.ang_velocity.y, state_n.ang_velocity.z, 0.0f);
    //laml::Quat new_spin = 0.5f * laml::mul(q, state_n.orientation);
    //derivative.spin = new_spin;
    //derivative.ang_acceleration = (net_moment + moment_func(state_n, t))*inv_inertia;
}

void simulation_body::major_step(double dt) {
    // ...

    // reset
    net_force = laml::Vec3_highp(0.0);
    //net_moment = laml::Vec3_highp(0.0);
}

void simulation_body::minor_step(double dt, rigid_body_state* minor_state) {
    state.position = state.position + derivative.velocity*dt;
    state.velocity = state.velocity + derivative.acceleration*dt;
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
            calc_derivative(t, &minor_state); // f(t_n, x_n)

            // substep 1
            minor_step(dt / 2.0, &minor_state);
            calc_derivative(t + dt/2.0, &minor_state); // f(t_n

            // substep 2
            minor_step(dt / 4.0, &minor_state);
            calc_derivative(t + 3.0*dt/4.0, &minor_state); // K3

            // substep 3
            minor_step(dt / 4.0, &minor_state);
            calc_derivative(t + dt, &minor_state); // K4

            // substep 4
            major_step(dt);
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