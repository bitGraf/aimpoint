#include "physics.h"

#include "log.h"

void simulation_body::set_mass(double new_mass) {

    mass = new_mass;
    if (new_mass == 0.0) {
        inv_mass = 1.0e9;
    } else {
        inv_mass = 1.0 / new_mass;
    }
}

void simulation_body::set_inv_mass(double new_inv_mass) {

    inv_mass = new_inv_mass;
    if (new_inv_mass == 0.0) {
        mass = 1.0e9;
    } else {
        mass = 1.0 / new_inv_mass;
    }
}

void simulation_body::set_inertia(double I1, double I2, double I3) {
    inertia.x = I1;
    inertia.y = I2;
    inertia.z = I3;

    if (I1 == 0.0) {
        inv_inertia.x = 1.0e9;
    } else {
        inv_inertia.x = 1.0/I1;
    }
    if (I2 == 0.0) {
        inv_inertia.y = 1.0e9;
    } else {
        inv_inertia.y = 1.0/I2;
    }
    if (I3 == 0.0) {
        inv_inertia.z = 1.0e9;
    } else {
        inv_inertia.z = 1.0/I3;
    }
}
void simulation_body::set_inv_inertia(double inv_I1, double inv_I2, double inv_I3) {
    inv_inertia.x = inv_I1;
    inv_inertia.y = inv_I2;
    inv_inertia.z = inv_I3;

    if (inv_I1 == 0.0) {
        inertia.x = 1.0e9;
    } else {
        inertia.x = 1.0/inv_I1;
    }
    if (inv_I2 == 0.0) {
        inertia.y = 1.0e9;
    } else {
        inertia.y = 1.0/inv_I2;
    }
    if (inv_I3 == 0.0) {
        inertia.z = 1.0e9;
    } else {
        inertia.z = 1.0/inv_I3;
    }
}

laml::Quat_highp calc_spin(laml::Vec3_highp ang_velocity, laml::Quat_highp orientation) {

    /* from MATLAB Simulink test
    e4 = -0.5 * ( i*w1+j*w2+k*w3 ) + C*s [SCALAR]
    spin.w = -0.5 * dot(orientation.xyz, ang_velocity) + C*orientation.w
    
    e1 = 0.5 * ( s*w1+j*w3-k*w2 ) + C*i
    spin.x = 0.5 * dot(orientation.wy(-z), ang_velocity.xzy) + C*orientation.x
    
    e2 = 0.5 * ( s*w2+k*w1-i*w3 ) + C*j
    spin.y = 0.5 * dot(orientation.wz(-x), ang_velocity.yxz) + C*orientation.y
    
    e3 = 0.5 * ( s*w3+i*w2-j*w1 ) + C*k
    spin.z = 0.5 * dot(orientation.wx(-y), ang_velocity.zyx) + C*orientation.z
    
    C = 1-( pow(u[1],2) + pow(u[2],2) + pow(u[3],2) + pow(u[4],2) )
    1-( s^2 + i^2 + j^2 + k^2 )
    C = k_quat(1 - ||orientation||)   <=  ||x|| is squared magnitude of x vector
    */

    const double K_quat = 100.0;
    double C = K_quat * (1 - laml::length_sq(orientation));

    double e1 =  0.5 * (orientation.w*ang_velocity.x + orientation.y*ang_velocity.z - orientation.z*ang_velocity.y) + C*orientation.x;
    double e2 =  0.5 * (orientation.w*ang_velocity.y + orientation.z*ang_velocity.x - orientation.x*ang_velocity.z) + C*orientation.y;
    double e3 =  0.5 * (orientation.w*ang_velocity.z + orientation.x*ang_velocity.y - orientation.y*ang_velocity.x) + C*orientation.z;
    double e4 = -0.5 * (orientation.x*ang_velocity.x + orientation.y*ang_velocity.y + orientation.z*ang_velocity.z) + C*orientation.w;
    
    return laml::Quat_highp(e1, e2, e3, e4);
}

void simulation_body::set_state(laml::Vec3_highp position, laml::Vec3_highp velocity, 
                                laml::Quat_highp orientation, laml::Vec3_highp ang_velocity) {
    state.position = position;
    state.velocity = velocity;
    state.orientation = laml::normalize(orientation);
    state.ang_velocity = ang_velocity;

    derivative.velocity = state.velocity;
    derivative.acceleration = laml::Vec3_highp(0.0, 0.0, 0.0);

    derivative.spin = calc_spin(ang_velocity, orientation);
    derivative.ang_acceleration = laml::Vec3_highp(0.0, 0.0, 0.0);

    calc_energy();
    truth_total_energy = linear_KE + rotational_KE;
}

void simulation_body::calc_energy() {
    linear_KE     = 0.5 * laml::dot(mass*state.velocity,        state.velocity);
    rotational_KE = 0.5 * laml::dot(inertia*state.ang_velocity, state.ang_velocity);
}

/* Operatores to create weighted sums of Derivatives (for certain integration stages) */
rigid_body_derivative operator+(const rigid_body_derivative& A, const rigid_body_derivative& B) {
    rigid_body_derivative res;

    res.velocity         = A.velocity         + B.velocity;
    res.acceleration     = A.acceleration     + B.acceleration;
    res.spin             = A.spin             + B.spin;
    res.ang_acceleration = A.ang_acceleration + B.ang_acceleration;

    return res;
}

rigid_body_derivative operator-(const rigid_body_derivative& A, const rigid_body_derivative& B) {
    rigid_body_derivative res;

    res.velocity         = A.velocity         - B.velocity;
    res.acceleration     = A.acceleration     - B.acceleration;
    res.spin             = A.spin             - B.spin;
    res.ang_acceleration = A.ang_acceleration - B.ang_acceleration;

    return res;
}

rigid_body_derivative operator*(const rigid_body_derivative& A, double f) {
    rigid_body_derivative res;

    res.velocity         = A.velocity*f;
    res.acceleration     = A.acceleration*f;
    res.spin             = A.spin*f;
    res.ang_acceleration = A.ang_acceleration*f;

    return res;
}
rigid_body_derivative operator*(double f, const rigid_body_derivative& A) {
    rigid_body_derivative res;

    res.velocity         = A.velocity*f;
    res.acceleration     = A.acceleration*f;
    res.spin             = A.spin*f;
    res.ang_acceleration = A.ang_acceleration*f;

    return res;
}

rigid_body_derivative simulation_body::calc_derivative(double t_n, const rigid_body_state* state_n) {
    rigid_body_derivative deriv;
    deriv.velocity = state_n->velocity;
    deriv.acceleration = (net_force + force_func(state_n, t_n))*inv_mass;

    deriv.spin = calc_spin(state_n->ang_velocity, state_n->orientation);
    deriv.ang_acceleration = (net_moment + moment_func(state_n, t_n))*inv_inertia - inv_inertia*(laml::cross(state_n->ang_velocity, inertia*state_n->ang_velocity));

    return deriv;
}

void simulation_body::base_major_step(double t, double dt) {
    //major_step(t, dt);

    t = t + dt;

    state.position = state.position + derivative.velocity*dt;
    state.velocity = state.velocity + derivative.acceleration*dt;

    state.orientation = state.orientation + derivative.spin*dt;
    state.ang_velocity = state.ang_velocity + derivative.ang_acceleration*dt;

    // reset
    net_force  = laml::Vec3_highp(0.0);
    net_moment = laml::Vec3_highp(0.0);

    //spdlog::trace("[{0:.5f}] major timestep  x={1:.5f}   v={2:.2f}   a={3:.2f}", t, state.position.x, state.velocity.x, ((net_force + force_func(&state, t))*inv_mass).x);
    //spdlog::trace("[{0:.5f}] major timestep  x={1:.2f}   y={2:.2f}   z={3:.2f}", t, state.ang_velocity.x, state.ang_velocity.y, state.ang_velocity.z);

    calc_energy();
}

void simulation_body::base_minor_step(double t, double dt, rigid_body_derivative* minor_derivative, rigid_body_state* minor_state) {
    //minor_step(t, dt, minor_derivative, minor_state);

    t = t + dt;

    minor_state->position = minor_state->position + minor_derivative->velocity*dt;
    minor_state->velocity = minor_state->velocity + minor_derivative->acceleration*dt;

    minor_state->orientation  = minor_state->orientation  + minor_derivative->spin*dt;
    minor_state->ang_velocity = minor_state->ang_velocity + minor_derivative->ang_acceleration*dt;

    //spdlog::trace("[{0:.5f}] minor timestep  x={1:.5f}   v={2:.2f}   a={3:.2f}", t, minor_state->position.x, minor_state->velocity.x, ((net_force + force_func(minor_state, t))*inv_mass).x);
    //spdlog::trace("[{0:.5f}] minor timestep  x={1:.2f}   y={2:.2f}   z={3:.2f}", t, state.ang_velocity.x, state.ang_velocity.y, state.ang_velocity.z);
    //spdlog::trace("[{0:.5f}] minor timestep  x={1:.2f}   y={2:.2f}   z={3:.2f}", t, minor_state->ang_velocity.x, minor_state->ang_velocity.y, minor_state->ang_velocity.z);
}

void simulation_body::major_step(double t, double dt) {}
void simulation_body::minor_step(double t, double dt, rigid_body_derivative* minor_derivative, rigid_body_state* minor_state) {}

void simulation_body::apply_force(laml::Vec3_highp force, laml::Vec3_highp location) {
    // TODO: calculate moment and force correctly
    //       is location in body-frame or world-frame?
    //net_force  = net_force  + force;
    //net_moment = net_moment + laml::cross(location, force);

    net_force = net_force + force;
}

void simulation_body::integrate_states(double t, double dt) {
    const int8 integration_mode = 3;
    major_step(t, dt);

    switch(integration_mode) {
        case 1: { // ode1 - Euler Method
            derivative = calc_derivative(t, &state); // K1 = f(t_n, y_n)
            base_major_step(t, dt);
        } break;
        case 2: { // ode2 - Heun Method
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, dt, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt, &minor_state); // K2 = f(t_n + dt, y_n + dt*K1)

            derivative = (1.0/2.0)*K1 + (1.0/2.0)*K2;
            base_major_step(t, dt);
        } break;
        case -2: { // Ralston's Method
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, 2.0*dt/3.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + 2.0*dt/3.0, &minor_state); // K2 = f(t_n + 2/3*dt, y_n + 2/3*dt*K1)

            derivative = (1.0/4.0)*K1 + (3.0/4.0)*K2;
            base_major_step(t, dt);
        } break;
        case 3: { // ode3 - Bogacki–Shampine method
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, dt/2.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt/2.0, &minor_state); // K2 = f(t_n + 1/2*dt, y_n + 1/2*dt*K1)

            // substep 2
            minor_state = state;
            base_minor_step(t, 3.0*dt/4.0, &K2, &minor_state);
            rigid_body_derivative K3 = calc_derivative(t + 3.0*dt/4.0, &minor_state); // K3 = f(t_n + 3/4*dt, y_n + 3/4*dt*K2)

            // substep 3
            derivative = (2.0/9.0)*K1 + (1.0/3.0)*K2 + (4.0/9.0)*K3;
            base_major_step(t, dt);
        } break;
        case 4: { // ode4 - Runge-Kutta
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, dt/2.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt/2.0, &minor_state); // K2 = f(t_n + 1/2*dt, y_n + 1/2*dt*K1)

            // substep 2
            minor_state = state;
            base_minor_step(t, dt/2.0, &K2, &minor_state);
            rigid_body_derivative K3 = calc_derivative(t + dt/2.0, &minor_state); // K3 = f(t_n + 1/2*dt, y_n + 1/2*dt*K2)

            // substep 3
            minor_state = state;
            base_minor_step(t, dt, &K3, &minor_state);
            rigid_body_derivative K4 = calc_derivative(t + dt, &minor_state); // K4 = f(t_n + dt, y_n + dt*K3)

            // substep 4
            derivative = (1.0/6.0)*K1 + (1.0/3.0)*K2 + (1.0/3.0)*K3 + (1.0/6.0)*K4;
            base_major_step(t, dt);
        } break;
        case -4: { // Alt Runge-Kutta - "3/8 rule"
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, dt/3.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt/3.0, &minor_state); // K2 = f(t_n + 1/3*dt, y_n + 1/3*dt*K1)

            // substep 2
            minor_state = state;
            rigid_body_derivative K_12 = (3.0/2.0)*K2 - (1.0/2.0)*K1;
            base_minor_step(t, 2.0*dt/3.0, &K_12, &minor_state);
            rigid_body_derivative K3 = calc_derivative(t + dt/2.0, &minor_state); // K3 = f(t_n + 2/3*dt, y_n + dt(-1/3*K1 + K2))

            // substep 3
            minor_state = state;
            rigid_body_derivative K_123 = K1 - K2 + K3;
            base_minor_step(t, dt, &K_123, &minor_state);
            rigid_body_derivative K4 = calc_derivative(t + dt, &minor_state); // K4 = f(t_n + dt, y_n + dt(K1 - K2 + K3))

            // substep 4
            derivative = (1.0/8.0)*K1 + (3.0/8.0)*K2 + (3.0/8.0)*K3 + (1.0/8.0)*K4;
            base_major_step(t, dt);
        } break;
        case 5: { // ode5 - Dormand-Prince
            rigid_body_state minor_state = state;
            rigid_body_derivative K1 = calc_derivative(t, &minor_state); // K1 = f(t_n, y_n)

            // substep 1
            minor_state = state;
            base_minor_step(t, dt/5.0, &K1, &minor_state);
            rigid_body_derivative K2 = calc_derivative(t + dt/5.0, &minor_state); // K2 = f(t_n + 1/5*dt, y_n + 1/5*dt*K1)

            // substep 2
            minor_state = state;
            rigid_body_derivative K_12 = (1.0/4.0)*K1 + (3.0/4.0)*K2;
            base_minor_step(t, 3.0*dt/10.0, &K_12, &minor_state);
            rigid_body_derivative K3 = calc_derivative(t + 3*dt/10.0, &minor_state); // K3 = f(t_n + 3/10*dt, y_n + dt(3/40*K1 + 9/40*K2))

            // substep 3
            minor_state = state;
            rigid_body_derivative K_123 = (11.0/9.0)*K1 - (14.0/3.0)*K2 + (40.0/9.0)*K3;
            base_minor_step(t, 4.0*dt/5.0, &K_123, &minor_state);
            rigid_body_derivative K4 = calc_derivative(t + 4.0*dt/5.0, &minor_state); // K4 = f(t_n + 4/5*dt, y_n + dt(44/45*K1 - 56/15*K2 + 32/9*K3))

            // substep 4
            minor_state = state;
            rigid_body_derivative K_1234 = (4843.0/1458.0)*K1 - (3170.0/243.0)*K2 + (8056.0/729.0)*K3 - (26.5/81.0)*K4;
            base_minor_step(t, 8.0*dt/9.0, &K_1234, &minor_state);
            rigid_body_derivative K5 = calc_derivative(t + 8.0*dt/9.0, &minor_state); // K5 = f(t_n + 8/9*dt, y_n + dt(...))

            // substep 5
            minor_state = state;
            rigid_body_derivative K_12345 = (9017.0/3168.0)*K1 - (355.0/33.0)*K2 + (46732.0/5247.0)*K3 + (49.0/176.0)*K4 - (5103.0/18656.0)*K5;
            base_minor_step(t, dt, &K_12345, &minor_state);
            rigid_body_derivative K6 = calc_derivative(t + dt, &minor_state); // K5 = f(t_n + dt, y_n + dt(...))

            // substep 6
            derivative = (35.0/384.0)*K1 + (500.0/1113.0)*K3 + (125.0/192.0)*K4 - (2187.0/6784.0)*K5 + (11.0/84.0)*K6;
            base_major_step(t, dt);
        } break;
    }
}

laml::Vec3_highp simulation_body::force_func(const rigid_body_state* at_state, double t) {
    return laml::Vec3_highp(0.0, 0.0, 0.0);
}

laml::Vec3_highp simulation_body::moment_func(const rigid_body_state* at_state, double t) {
    return laml::Vec3_highp(0.0, 0.0, 0.0);
}