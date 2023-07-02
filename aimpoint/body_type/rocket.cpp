#include "rocket.h"

#include "log.h"

rocket::rocket() {
    tof = 0.0;
    stage = 0;

    set_mass(549'000);
    set_inertia(1, 1, 1);

    // KSC 39b
    launch_lat = 28.627023;
    launch_lon = -80.620856;
    launch_az = 0;

    //launch_lat = 45.01; // transferred to e^2 = .5 ellipsoid


    //launch_lat = 40;
    //launch_lon = -75;

    //launch_lat = 57.80;
}

void rocket::launch(planet* p) {
    earth = p;

    LCI2ECI = earth->create_local_inertial(launch_lat, launch_lon, launch_az);

    // set position to launch site position (inertial)
    //state.position = earth->fixed_to_inertial(earth->lla_to_fixed(launch_lat, launch_lon, 0.0));
    //state.velocity = laml::transform::transform_point(LCI2ECI, vec3d(0.0, 0.0, -0.1));

    // set position to launch site position (fixed)
    //vec3d pos_ecef = earth->lla_to_fixed(launch_lat, launch_lon, 480000);
    //vec3d vel_ecef(0.0f, 0.0f, 0.0f);
    //earth->fixed_to_inertial(pos_ecef, vel_ecef, &state.position, &state.velocity);

    // set position to orbit state above launch site
    state.position = earth->fixed_to_inertial(earth->lla_to_fixed(launch_lat, launch_lon, 480000));
    vec3d v_dir = laml::normalize(laml::cross(vec3d(0.0, 0.0, 1.0), state.position));
    state.velocity = v_dir * 8500.0;
}

void rocket::major_step(double t, double dt) {
    //apply_force(earth->gravity(state.position), vec3d());
    //if (tof < 150.0) {
    //    double thrust = 8'000'000.0;
    //    double isp = 300.0;
    //    double mdot = thrust / (isp * 9.81);
    //
    //    double pitch_rate = (90.0 - 30.0) / (150.0 - 0.0);
    //    pitch -= pitch_rate*dt;
    //
    //    apply_force(laml::Vec3_highp(thrust*cos(pitch*laml::constants::deg2rad<double>), thrust*sin(pitch*laml::constants::deg2rad<double>), 0.0), laml::Vec3_highp(0.0, 0.0, 0.0));
    //
    //    mass -= mdot*dt;
    //    inv_mass = 1.0 / mass;
    //}

    tof += dt;
}

laml::Vec3_highp rocket::force_func(const rigid_body_state* at_state, double t) {
    //return laml::Vec3(0.0f, -9.81f*state.mass, 0.0f);
    return earth->gravity(at_state->position)*mass;
}

//laml::Vec3_highp rocket::moment_func(const rigid_body_state& at_state, double t) {
//    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
//}