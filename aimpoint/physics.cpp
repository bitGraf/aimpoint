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
    state.inertia.y = I2;
    state.inertia.z = I3;

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
    state.inv_inertia.y = inv_I2;
    state.inv_inertia.z = inv_I3;

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
    state.velocity = velocity;
    state.orientation = orientation;
    state.ang_velocity = ang_velocity;

    state.recalculate();

    derivative.velocity = state.velocity;
    derivative.acceleration = laml::Vec3(0.0f, 0.0f, 0.0f);

    derivative.spin = state.spin;
    derivative.ang_acceleration = laml::Vec3(0.0f, 0.0f, 0.0f);
}

void simulation_body::major_step(double dt) {

}

void simulation_body::minor_step(double dt) {

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
        new_state.velocity = state.velocity + with_deriv->acceleration*dt_f;

        // euler eom
        new_state.orientation  = state.orientation  + with_deriv->spin*dt_f;
        laml::Vec3 wd_rot = with_deriv->ang_acceleration - (state.inv_inertia*laml::cross(state.ang_velocity, state.ang_momentum));
        new_state.ang_velocity = state.ang_velocity + wd_rot*dt_f;

        new_state.recalculate();

        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = new_state.velocity;
        new_deriv.acceleration = (net_force + this->force_func(new_state, t))*state.inv_mass;

        new_deriv.spin = new_state.spin;
        new_deriv.ang_acceleration = (net_moment + this->moment_func(new_state, t))*state.inv_inertia; // component-wise

        return new_deriv;
    } else {
        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = state.velocity;
        new_deriv.acceleration = (net_force + this->force_func(state, t))*state.inv_mass;

        new_deriv.spin = state.spin;
        new_deriv.ang_acceleration = (net_moment + this->moment_func(state, t))*state.inv_inertia;

        return new_deriv;
    }
}

void simulation_body::integrate_states(double t, double dt_d) {
    float dt = dt_d;

    const uint8 integration_mode = 0;

    switch(integration_mode) {
        case 0: {
            // explicit euler
            rigid_body_derivative deriv_euler = calc_new_deriv(t, dt, nullptr);
            state.position = state.position + deriv_euler.velocity*dt;
            state.velocity = state.velocity + deriv_euler.acceleration*dt;
            state.orientation  = state.orientation  + deriv_euler.spin*dt;
            laml::Vec3 wd_rot = deriv_euler.ang_acceleration - (state.inv_inertia*laml::cross(state.ang_velocity, state.ang_momentum));
            state.ang_velocity = state.ang_velocity + wd_rot*dt;

            state.recalculate();
            memcpy(&derivative, &deriv_euler, sizeof(rigid_body_derivative));

            net_force = laml::Vec3(0.0f);
            net_moment = laml::Vec3(0.0f);
        } break;
        case 1: {
            // RK4
            rigid_body_derivative A = calc_new_deriv(t, 0.0f, nullptr);
            rigid_body_derivative B = calc_new_deriv(t, dt*0.5f, &A);
            rigid_body_derivative C = calc_new_deriv(t, dt*0.5f, &B);
            rigid_body_derivative D = calc_new_deriv(t, dt, &C);
    
            rigid_body_derivative deriv_rk4;
            deriv_rk4.velocity = (1.0f/6.0f)*(A.velocity + 2.0f*B.velocity + 2.0f*C.velocity + D.velocity);
            deriv_rk4.acceleration    = (1.0f/6.0f)*(A.acceleration    + 2.0f*B.acceleration    + 2.0f*C.acceleration    + D.acceleration);
            deriv_rk4.spin   = (1.0f/6.0f)*(A.spin   + 2.0f*B.spin   + 2.0f*C.spin   + D.spin);
            deriv_rk4.ang_acceleration = (1.0f/6.0f)*(A.ang_acceleration + 2.0f*B.ang_acceleration + 2.0f*C.ang_acceleration + D.ang_acceleration);

            state.position = state.position + deriv_rk4.velocity*dt;
            state.velocity = state.velocity + deriv_rk4.acceleration*dt;
            state.orientation  = state.orientation  + deriv_rk4.spin*dt;
            laml::Vec3 wd_rot = deriv_rk4.ang_acceleration - (state.inv_inertia*laml::cross(state.ang_velocity, state.ang_velocity));
            state.ang_velocity = state.ang_velocity + wd_rot*dt;

            state.recalculate();
            memcpy(&derivative, &deriv_rk4, sizeof(rigid_body_derivative));

            net_force = laml::Vec3(0.0f);
            net_moment = laml::Vec3(0.0f);
        } break;
#if 0
        case 2: {
            // leap frog algorithm
            rigid_body_derivative deriv_frog = calc_new_deriv(t, dt, nullptr);
            state.position = state.position + deriv_frog.velocity*dt;
            state.momentum = state.momentum + deriv_frog.force*dt;
            state.orientation  = state.orientation  + deriv_frog.spin*dt;
            laml::Vec3 M_rot = deriv_frog.moment - laml::cross(state.ang_velocity, state.ang_momentum);
            state.ang_momentum = state.ang_momentum + M_rot*dt;

            state.recalculate();
            memcpy(&derivative, &deriv_frog, sizeof(rigid_body_derivative));

            net_force = laml::Vec3(0.0f);
            net_moment = laml::Vec3(0.0f);
        } break;
#endif
    }
}

laml::Vec3 simulation_body::force_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(-state.position.x, 0.0f, 0.0f);
}

laml::Vec3 simulation_body::moment_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(0.0f);
}