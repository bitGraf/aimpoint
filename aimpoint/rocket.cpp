#include "rocket.h"

#include "log.h"

rocket::rocket() {
    tof = 0.0;
    pitch = 90.0;

    this->set_mass(550'000.0);

    //calc_secondary_states();
}

void rocket::major_step(double dt) {
    if (tof < 150.0) {
        double thrust = 8'000'000.0;
        double isp = 300.0;
        double mdot = thrust / (isp * 9.81);

        double pitch_rate = (90.0 - 30.0) / (150.0 - 0.0);
        pitch -= pitch_rate*dt;

        apply_force(laml::Vec3_highp(thrust*cos(pitch*laml::constants::deg2rad<double>), thrust*sin(pitch*laml::constants::deg2rad<double>), 0.0), laml::Vec3_highp(0.0, 0.0, 0.0));
        mass -= mdot*dt;
    }

    tof += dt;
}

laml::Vec3_highp rocket::force_func(const rigid_body_state* at_state, double t) {
    //return laml::Vec3(0.0f, -9.81f*state.mass, 0.0f);
    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
}

//laml::Vec3_highp rocket::moment_func(const rigid_body_state& at_state, double t) {
//    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
//}