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

    float inertia; // TODO: make these 3 principal interias
    float inverseInertia;

    void recalculate() {
        velocity = momentum * inv_mass;

        ang_velocity = ang_momentum * inverseInertia;
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

#define max_num_objects 32
struct physics_world_state {
    uint32 num_objects;

    rigid_body_state states[max_num_objects];
    rigid_body_derivative derivatives[max_num_objects];

    physics_world_state() {
        num_objects = 0;
        memset(states, 0, sizeof(rigid_body_state)*max_num_objects);
        memset(derivatives, 0, sizeof(rigid_body_derivative)*max_num_objects);
    }
};

struct rigid_body_render_state {
    laml::Vec3 position;
    laml::Quat orientation;
};

struct physics_world_render_state {
    uint32 num_objects;

    rigid_body_render_state states[max_num_objects];

    physics_world_render_state() {
        num_objects = 0;
        memset(states, 0, sizeof(rigid_body_render_state)*max_num_objects);
    }
};

struct physics_world {
    void integrate_states(physics_world_state* state, double t, double dt);
    physics_world_render_state interpolate_states(physics_world_state previous, physics_world_state current, double f);
};