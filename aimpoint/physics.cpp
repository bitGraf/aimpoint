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

rigid_body_derivative simulation_body::RK_stage(double t_n, rigid_body_state state_n, float dt, rigid_body_derivative* prev_stage) {
    rigid_body_derivative deriv;
    rigid_body_state state_2;
    rigid_body_state& stage_state = state_n;
    if (prev_stage) {
        double t_2 = t_n + dt / 2;
        state_2 = state_n;

        state_2.position = state_n.position + prev_stage->velocity*dt;
        state_2.velocity = state_n.velocity + prev_stage->acceleration*dt;
        state_2.orientation = state_n.orientation + prev_stage->spin*dt;
        laml::Vec3 wd_n = prev_stage->ang_acceleration - (state_n.inv_inertia*laml::cross(state_n.ang_velocity, state_n.ang_momentum));
        state_2.ang_velocity = state_n.ang_velocity + wd_n*dt;

        stage_state = state_2;
    }

    deriv.velocity = stage_state.velocity;
    deriv.acceleration = (net_force + force_func(stage_state, t_n))*stage_state.inv_mass;
    deriv.spin = stage_state.spin;
    deriv.ang_acceleration = (net_moment + moment_func(stage_state, t_n))*stage_state.inv_inertia;

    return deriv;
}

void simulation_body::integrate_states(double t, float dt) {
    const uint8 integration_mode = 1;

    switch(integration_mode) {
        case 0: {
            // explicit euler
            double t_n = t;
            rigid_body_state state_n = state;

            rigid_body_derivative deriv_n;
            deriv_n.velocity = state_n.velocity;
            deriv_n.acceleration = (net_force + force_func(state_n, t_n))*state_n.inv_mass;
            deriv_n.spin = state_n.spin;
            deriv_n.ang_acceleration = (net_moment + moment_func(state_n, t_n))*state_n.inv_inertia;

            rigid_body_state state_np1 = state_n;
            state_np1.position = state_n.position + deriv_n.velocity*dt;
            state_np1.velocity = state_n.velocity + deriv_n.acceleration*dt;
            state_np1.orientation = state_n.orientation + deriv_n.spin*dt;
            laml::Vec3 wd_n = deriv_n.ang_acceleration - (state_n.inv_inertia*laml::cross(state_n.ang_velocity, state_n.ang_momentum));
            state_np1.ang_velocity = state_n.ang_velocity + wd_n*dt;

            state_np1.recalculate();
            memcpy(&state, &state_np1, sizeof(rigid_body_state));
            memcpy(&derivative, &deriv_n, sizeof(rigid_body_derivative));

            // reset
            net_force = laml::Vec3(0.0f);
            net_moment = laml::Vec3(0.0f);
        } break;
        case 1: {
            // semi-implicit euler
            double t_n = t;
            rigid_body_state state_n = state;

            rigid_body_derivative deriv_n;
            deriv_n.velocity = state_n.velocity;
            deriv_n.acceleration = (net_force + force_func(state_n, t_n))*state_n.inv_mass;
            deriv_n.spin = state_n.spin;
            deriv_n.ang_acceleration = (net_moment + moment_func(state_n, t_n))*state_n.inv_inertia;

            rigid_body_state state_np1 = state_n;
            state_np1.velocity = state_n.velocity + deriv_n.acceleration*dt; // acceleration at state_n
            state_np1.position = state_n.position + state_np1.velocity*dt;   // velocity at state_n+1

            laml::Vec3 wd_n = deriv_n.ang_acceleration - (state_n.inv_inertia*laml::cross(state_n.ang_velocity, state_n.ang_momentum));
            state_np1.ang_velocity = state_n.ang_velocity + wd_n*dt;
            laml::Quat q(state_np1.ang_velocity.x, state_np1.ang_velocity.y, state_np1.ang_velocity.z, 0.0f);
            laml::Quat spin_np1 = 0.5f * laml::mul(q, state_np1.orientation);
            state_np1.orientation = state_n.orientation + spin_np1*dt;

            state_np1.recalculate();
            memcpy(&state, &state_np1, sizeof(rigid_body_state));
            memcpy(&derivative, &deriv_n, sizeof(rigid_body_derivative));

            // reset
            net_force = laml::Vec3(0.0f);
            net_moment = laml::Vec3(0.0f);
        } break;
        case 4: {
            // RK4
            double t_n = t;
            rigid_body_state state_n = state;

            rigid_body_derivative K1 = RK_stage(t_n, state_n, 0, 0);
            rigid_body_derivative K2 = RK_stage(t_n, state_n, dt/2.0, &K1);
            rigid_body_derivative K3 = RK_stage(t_n, state_n, dt/2.0, &K2);
            rigid_body_derivative K4 = RK_stage(t_n, state_n, dt, &K3);
            rigid_body_derivative deriv_n;
            deriv_n.velocity         = (1.0f / 6.0f) * (K1.velocity         + 2.0f*K2.velocity         + 2.0f*K3.velocity         + K4.velocity);
            deriv_n.acceleration     = (1.0f / 6.0f) * (K1.acceleration     + 2.0f*K2.acceleration     + 2.0f*K3.acceleration     + K4.acceleration);
            deriv_n.spin             = (1.0f / 6.0f) * (K1.spin             + 2.0f*K2.spin             + 2.0f*K3.spin             + K4.spin);
            deriv_n.ang_acceleration = (1.0f / 6.0f) * (K1.ang_acceleration + 2.0f*K2.ang_acceleration + 2.0f*K3.ang_acceleration + K4.ang_acceleration);

            rigid_body_state state_np1 = state_n;
            state_np1.position = state_n.position + deriv_n.velocity*dt;
            state_np1.velocity = state_n.velocity + deriv_n.acceleration*dt;
            state_np1.orientation = state_n.orientation + deriv_n.spin*dt;
            laml::Vec3 wd_n = deriv_n.ang_acceleration - (state_n.inv_inertia*laml::cross(state_n.ang_velocity, state_n.ang_momentum));
            state_np1.ang_velocity = state_n.ang_velocity + wd_n*dt;

            state_np1.recalculate();
            memcpy(&state, &state_np1, sizeof(rigid_body_state));
            memcpy(&derivative, &deriv_n, sizeof(rigid_body_derivative));

            // reset
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