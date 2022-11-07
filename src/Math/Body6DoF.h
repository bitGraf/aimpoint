#pragma once

#include "Core/Core.h"

#include <glm/glm.hpp>

namespace aimpoint {

    struct State6DoF {
        glm::vec3 Position;
        glm::vec3 Velocity;
        glm::vec4 Orientation; //quat
        glm::vec3 BodyRate;
    };
}