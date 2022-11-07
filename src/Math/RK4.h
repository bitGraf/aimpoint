#pragma once

#include <glm/glm.hpp>
#include "Core/Timing/Clock.h"
#include "Body6DoF.h"

namespace aimpoint {
    // derivative function of the form: x_dot = f(t, x)
    typedef float (*scalar_derivative_fcn)(float, float);
    typedef glm::vec2 (*vec2_derivative_fcn)(float, glm::vec2);
    typedef State6DoF (*state6dof_derivative_fcn)(float, State6DoF);

    float Integrate_Euler(scalar_derivative_fcn fn_xdot, float x, const Clock clock);
    glm::vec2 Integrate_Euler(vec2_derivative_fcn fn_xdot, glm::vec2 x, const Clock clock);
    State6DoF Integrate_Euler(state6dof_derivative_fcn fn_xdot, State6DoF x, const Clock clock);

    float Integrate_RK4(scalar_derivative_fcn fn_xdot, float x, const Clock clock);
    glm::vec2 Integrate_RK4(vec2_derivative_fcn fn_xdot, glm::vec2 x, const Clock clock);
    State6DoF Integrate_RK4(state6dof_derivative_fcn fn_xdot, State6DoF x, const Clock clock);
}