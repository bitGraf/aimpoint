#include "RK4.h"

namespace aimpoint {

    float Integrate_Euler(scalar_derivative_fcn fn_xdot, float x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();

        float x_dot = fn_xdot(t, x);
        float x_new = x + x_dot * h;

        return x_new;
    }

    float Integrate_RK4(scalar_derivative_fcn fn_xdot, float x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();

        // K1 = f(t, x)
        float K1 = fn_xdot(t, x);

        // K2 = f(t + h/2, x + h*K1/2)
        float K2 = fn_xdot(t + h/2.0, x + h*K1/2.0);

        // K3 = f(t + h/2.0, x + h*K2/2)
        float K3 = fn_xdot(t + h/2.0, x + h*K2/2.0);

        // K4 = f(t + h, x + h*K3)
        float K4 = fn_xdot(t + h, x + h*K3);
        
        // Sum
        float x_new = x + (K1 + 2*K2 + 2*K3 + K4)*h/6.0;

        return x_new;   
    }

    glm::vec2 Integrate_Euler(vec2_derivative_fcn fn_xdot, glm::vec2 x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();

        glm::vec2 x_dot = fn_xdot(t, x);
        glm::vec2 x_new = x + x_dot * h;

        return x_new;
    }

    glm::vec2 Integrate_RK4(vec2_derivative_fcn fn_xdot, glm::vec2 x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();

        // K1 = f(t, x)
        glm::vec2 K1 = fn_xdot(t, x);

        // K2 = f(t + h/2, x + h*K1/2)
        glm::vec2 K2 = fn_xdot(t + h/2.0, x + h*K1/2.0f);

        // K3 = f(t + h/2.0, x + h*K2/2)
        glm::vec2 K3 = fn_xdot(t + h/2.0, x + h*K2/2.0f);

        // K4 = f(t + h, x + h*K3)
        glm::vec2 K4 = fn_xdot(t + h, x + h*K3);
        
        // Sum
        glm::vec2 x_new = x + (K1 + 2.0f*K2 + 2.0f*K3 + K4)*h/6.0f;
        
        return x_new;   
    }
}