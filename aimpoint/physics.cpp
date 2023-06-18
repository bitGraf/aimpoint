#include "physics.h"

void simulation_body::set_mass(double mass) {

    state.mass = mass;
    if (mass == 0.0) {
        state.inv_mass = 1.0e9;
    } else {
        state.inv_mass = 1.0 / mass;
    }
}

void simulation_body::set_inv_mass(double inv_mass) {

    state.inv_mass = inv_mass;
    if (inv_mass == 0.0) {
        state.mass = 1.0e9;
    } else {
        state.mass = 1.0 / inv_mass;
    }
}

void simulation_body::set_inertia(double I1, double I2, double I3) {
    state.inertia.x = I1;
    state.inertia.x = I2;
    state.inertia.x = I3;

    if (I1 == 0.0) {
        state.inv_inertia.x = 1.0e9;
    } else {
        state.inv_inertia.x = 1.0/I1;
    }
    if (I2 == 0.0) {
        state.inv_inertia.y = 1.0e9;
    } else {
        state.inv_inertia.y = 1.0/I2;
    }
    if (I3 == 0.0) {
        state.inv_inertia.z = 1.0e9;
    } else {
        state.inv_inertia.z = 1.0/I3;
    }
}
void simulation_body::set_inv_inertia(double inv_I1, double inv_I2, double inv_I3) {
    state.inv_inertia.x = inv_I1;
    state.inv_inertia.x = inv_I2;
    state.inv_inertia.x = inv_I3;

    if (inv_I1 == 0.0) {
        state.inertia.x = 1.0e9;
    } else {
        state.inertia.x = 1.0/inv_I1;
    }
    if (inv_I2 == 0.0) {
        state.inertia.y = 1.0e9;
    } else {
        state.inertia.y = 1.0/inv_I2;
    }
    if (inv_I3 == 0.0) {
        state.inertia.z = 1.0e9;
    } else {
        state.inertia.z = 1.0/inv_I3;
    }
}

void simulation_body::set_state(laml::Vec3 position, laml::Vec3 velocity, 
                                laml::Quat orientation, laml::Vec3 ang_velocity) {
    state.position = position;
    state.momentum = velocity * state.mass;
    state.orientation = orientation;
    state.ang_momentum = ang_velocity * state.inertia; // component-wise multiply

    state.recalculate();

    derivative.velocity = state.velocity;
    derivative.force = laml::Vec3(0.0f, 0.0f, 0.0f);

    derivative.spin = state.spin;
    derivative.moment = laml::Vec3(0.0f, 0.0f, 0.0f);
}

void simulation_body::apply_force(laml::Vec3 force, laml::Vec3 location) {
    net_force = net_force + force;

    // TODO: calculate moment
    //       is location in body-frame or world-frame?
}

rigid_body_derivative simulation_body::calc_new_deriv(double t, double dt, const rigid_body_derivative* const with_deriv) {
    // TODO: why...
    float dt_f = dt;

    if (with_deriv) {
        // linear eom
        rigid_body_state new_state;
        memcpy(&new_state, &state, sizeof(rigid_body_state));
        new_state.position = state.position + with_deriv->velocity*dt_f;
        new_state.momentum = state.momentum + with_deriv->force*dt_f;

        // euler eom
        new_state.orientation  = state.orientation  + with_deriv->spin*dt_f;
        new_state.ang_momentum = state.ang_momentum + with_deriv->moment*dt_f; // TODO: this is wrong!
                                                                               //       this is in a rotating reference frame
                                                                               //       so that needs to be accounted for in the integral

        new_state.recalculate();

        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = new_state.velocity;
        new_deriv.force = this->force_func(new_state, t);

        new_deriv.spin = new_state.spin;
        new_deriv.moment = this->moment_func(new_state, t);

        return new_deriv;
    } else {
        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = state.velocity;
        new_deriv.force = this->force_func(state, t);

        new_deriv.spin = state.spin;
        new_deriv.moment = this->moment_func(state, t);

        return new_deriv;
    }
}

void simulation_body::integrate_states(double t, double dt_d) {
    float dt = dt_d;

    const uint8 integration_mode = 1;

    switch(integration_mode) {
        case 0: {
            // explicit euler
            rigid_body_derivative deriv_euler = calc_new_deriv(t, dt, nullptr);
            state.position = state.position + deriv_euler.velocity*dt;
            state.momentum = state.momentum + deriv_euler.force*dt;
            state.orientation  = state.orientation  + deriv_euler.spin*dt;
            state.ang_momentum = state.ang_momentum + deriv_euler.moment*dt;

            state.recalculate();
            memcpy(&derivative, &deriv_euler, sizeof(rigid_body_derivative));
        } break;
        case 1: {
            // RK4
            rigid_body_derivative A = calc_new_deriv(t, 0.0f, nullptr);
            rigid_body_derivative B = calc_new_deriv(t, dt*0.5f, &A);
            rigid_body_derivative C = calc_new_deriv(t, dt*0.5f, &B);
            rigid_body_derivative D = calc_new_deriv(t, dt, &C);
    
            rigid_body_derivative deriv_rk4;
            deriv_rk4.velocity = (1.0f/6.0f)*(A.velocity + 2.0f*B.velocity + 2.0f*C.velocity + D.velocity);
            deriv_rk4.force    = (1.0f/6.0f)*(A.force    + 2.0f*B.force    + 2.0f*C.force    + D.force);
            deriv_rk4.spin   = (1.0f/6.0f)*(A.spin   + 2.0f*B.spin   + 2.0f*C.spin   + D.spin);
            deriv_rk4.moment = (1.0f/6.0f)*(A.moment + 2.0f*B.moment + 2.0f*C.moment + D.moment);

            state.position = state.position + deriv_rk4.velocity*dt;
            state.momentum = state.momentum + deriv_rk4.force*dt;
            state.orientation  = state.orientation  + deriv_rk4.spin*dt;
            state.ang_momentum = state.ang_momentum + deriv_rk4.moment*dt;

            state.recalculate();
            memcpy(&derivative, &deriv_rk4, sizeof(rigid_body_derivative));

        } break;
    }
}

laml::Vec3 simulation_body::force_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(-state.position.x, 0.0f, 0.0f);
}

laml::Vec3 simulation_body::moment_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(0.0f);
}