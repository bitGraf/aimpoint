#include "physics.h"

rigid_body_derivative calc_new_deriv(const rigid_body_state* initial_state, double t, double dt, const rigid_body_derivative* const initial_deriv) {
    // TODO: why...
    float dt_f = dt;

    if (initial_deriv) {
        // linear eom
        rigid_body_state new_state;
        memcpy(&new_state, initial_state, sizeof(rigid_body_state));
        new_state.position = initial_state->position + initial_deriv->velocity*dt_f;
        new_state.momentum = initial_state->momentum + initial_deriv->force*dt_f;

        // euler eom
        new_state.orientation = initial_state->orientation + initial_deriv->spin*dt_f;
        new_state.ang_momentum = initial_state->ang_momentum + initial_deriv->moment*dt_f;

        new_state.recalculate();

        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = new_state.velocity;
        new_deriv.force = laml::Vec3(-initial_state->position.x, 0.0f, 0.0f); // TODO: how get this...

        new_deriv.spin = new_state.spin;
        new_deriv.moment = laml::Vec3(0.0f, 0.0f, 0.0f); // TODO: how get this...

        return new_deriv;
    } else {
        // recaulate new derivatives
        rigid_body_derivative new_deriv;
        new_deriv.velocity = initial_state->velocity;
        new_deriv.force = laml::Vec3(-initial_state->position.x, 0.0f, 0.0f); // TODO: how get this...

        new_deriv.spin = initial_state->spin;
        new_deriv.moment = laml::Vec3(0.0f, 0.0f, 0.0f); // TODO: how get this...

        return new_deriv;
    }
}

void physics_world::integrate_states(physics_world_state* world, double t, double dt_d) {
    float dt = dt_d;
    for (uint32 n = 0; n < world->num_objects; n++) {
        rigid_body_state* state = &world->states[n];
        //rigid_body_derivative* deriv = &state->derivatives[n];

        const uint8 integration_mode = 1;

        switch(integration_mode) {
            case 0: {
                // explicit euler
                rigid_body_derivative deriv_euler = calc_new_deriv(state, t, dt, nullptr);
                state->position = state->position + deriv_euler.velocity*dt;
                state->momentum = state->momentum + deriv_euler.force*dt;
                state->orientation  = state->orientation  + deriv_euler.spin*dt;
                state->ang_momentum = state->ang_momentum + deriv_euler.moment*dt;

                state->recalculate();
            } break;
            case 1: {
                // RK4
                rigid_body_derivative A = calc_new_deriv(state, t, 0.0f, nullptr);
                rigid_body_derivative B = calc_new_deriv(state, t, dt*0.5f, &A);
                rigid_body_derivative C = calc_new_deriv(state, t, dt*0.5f, &B);
                rigid_body_derivative D = calc_new_deriv(state, t, dt, &C);
        
                rigid_body_derivative deriv_rk4;
                deriv_rk4.velocity = (1.0f/6.0f)*(A.velocity + 2.0f*B.velocity + 2.0f*C.velocity + D.velocity);
                deriv_rk4.force    = (1.0f/6.0f)*(A.force    + 2.0f*B.force    + 2.0f*C.force    + D.force);
                deriv_rk4.spin   = (1.0f/6.0f)*(A.spin   + 2.0f*B.spin   + 2.0f*C.spin   + D.spin);
                deriv_rk4.moment = (1.0f/6.0f)*(A.moment + 2.0f*B.moment + 2.0f*C.moment + D.moment);

                state->position = state->position + deriv_rk4.velocity*dt;
                state->momentum = state->momentum + deriv_rk4.force*dt;
                state->orientation  = state->orientation  + deriv_rk4.spin*dt;
                state->ang_momentum = state->ang_momentum + deriv_rk4.moment*dt;

                state->recalculate();

            } break;
        }
    }
}

physics_world_render_state physics_world::interpolate_states(physics_world_state previous, physics_world_state current, double f) {
    physics_world_render_state state;

    float f_f = f;

    state.num_objects = current.num_objects;
    for (int n = 0; n < state.num_objects; n++) {
        state.states[n].position    = laml::lerp(previous.states[n].position, current.states[n].position, f_f);
        state.states[n].orientation = laml::slerp(previous.states[n].orientation, current.states[n].orientation, f_f);
    }

    return state;
}