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

    State6DoF Integrate_Euler(state6dof_derivative_fcn fn_xdot, State6DoF x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();

        State6DoF x_dot = fn_xdot(t, x);
        State6DoF x_new;

        // Translation
        x_new.Position = x.Position + x_dot.Position*h;
        x_new.Velocity = x.Velocity + x_dot.Velocity*h;

        // Orientation
        x_new.BodyRate = x.BodyRate + x_dot.BodyRate*h;
        x_new.Orientation = x.Orientation + x_dot.Orientation*h;
        //x_new.Orientation = glm::normalize(x_new.Orientation); // to ensure orientation is unit-value

        return x_new;
    }

    State6DoF Integrate_RK4(state6dof_derivative_fcn fn_xdot, State6DoF x, const Clock clock) {
        float t = clock.GetTime();
        float h = clock.GetTimestep();
        State6DoF x_new;

        // K1 = f(t, x)
        State6DoF K1 = fn_xdot(t, x);

        // K2 = f(t + h/2, x + h*K1/2)
        x_new.Position = x.Position       + h*K1.Position/2.0f;
        x_new.Velocity = x.Velocity       + h*K1.Velocity/2.0f;
        x_new.Orientation = x.Orientation + h*K1.Orientation/2.0f;
        x_new.BodyRate = x.BodyRate       + h*K1.BodyRate/2.0f;
        State6DoF K2 = fn_xdot(t + h/2.0, x_new);

        // K3 = f(t + h/2.0, x + h*K2/2)
        x_new.Position = x.Position       + h*K2.Position/2.0f;
        x_new.Velocity = x.Velocity       + h*K2.Velocity/2.0f;
        x_new.Orientation = x.Orientation + h*K2.Orientation/2.0f;
        x_new.BodyRate = x.BodyRate       + h*K2.BodyRate/2.0f;
        State6DoF K3 = fn_xdot(t + h/2.0, x_new);

        // K4 = f(t + h, x + h*K3)
        x_new.Position = x.Position       + h*K3.Position;
        x_new.Velocity = x.Velocity       + h*K3.Velocity;
        x_new.Orientation = x.Orientation + h*K3.Orientation;
        x_new.BodyRate = x.BodyRate       + h*K3.BodyRate;
        State6DoF K4 = fn_xdot(t + h, x_new);
        
        // Sum
        x_new.Position = x.Position + (K1.Position + 2.0f*K2.Position + 2.0f*K3.Position + K4.Position)*h/6.0f;
        x_new.Velocity = x.Velocity + (K1.Velocity + 2.0f*K2.Velocity + 2.0f*K3.Velocity + K4.Velocity)*h/6.0f;
        x_new.Orientation = x.Orientation + (K1.Orientation + 2.0f*K2.Orientation + 2.0f*K3.Orientation + K4.Orientation)*h/6.0f;
        x_new.BodyRate = x.BodyRate + (K1.BodyRate + 2.0f*K2.BodyRate + 2.0f*K3.BodyRate + K4.BodyRate)*h/6.0f;

        x_new.Orientation = glm::normalize(x_new.Orientation);
        
        return x_new;   
    }
}