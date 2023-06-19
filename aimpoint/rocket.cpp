#include "rocket.h"

#include "log.h"

rocket::rocket() {
    tof = 0.0f;
    pitch = 90.0f;

    this->set_mass(550'000.0);
    this->set_inertia(1.0, 2.0, 3.0);

    state.ang_momentum.x = 0.5f;
    state.ang_momentum.y = 40.0f;
    state.ang_momentum.z = 0.5f;

    state.recalculate();
}

void rocket::major_step(double dt) {
    if (tof < 150.0) {
        double thrust = 8'000'000.0f;
        double isp = 300.0;
        double mdot = thrust / (isp * 9.81);

        double pitch_rate = (90.0 - 30.0) / (150.0 - 0.0);
        pitch -= pitch_rate*dt;

        apply_force(laml::Vec3(thrust*cos(pitch*laml::constants::deg2rad<double>), thrust*sin(pitch*laml::constants::deg2rad<double>), 0.0f), laml::Vec3(0.0f, 0.0f, 0.0f));
        state.mass -= mdot*dt;
    }

    tof += dt;
}

laml::Vec3 rocket::force_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(0.0f, -9.81f*state.mass, 0.0f);
}

laml::Vec3 rocket::moment_func(const rigid_body_state& at_state, double t) {
    return laml::Vec3(0.0f, 0.0f, 0.0f);
}