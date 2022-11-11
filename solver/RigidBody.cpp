#include "RigidBody.h"

namespace ab_solver {

    RigidBody::RigidBody() {
        pos_x = pos_y = pos_z = 0.0;
        vel_x = vel_y = vel_z = 0.0;
        mass = 1.0;
        index = -1;
    }

    RigidBody::~RigidBody() {

    }

}