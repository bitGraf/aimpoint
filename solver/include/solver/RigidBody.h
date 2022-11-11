#pragma once

namespace ab_solver {

    class RigidBody {
    public:
        RigidBody();
        ~RigidBody();

        double pos_x, pos_y, pos_z;
        double vel_x, vel_y, vel_z;
        double mass;

        int index;
    };
}