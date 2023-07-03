#include "satellite.h"

#include "log.h"

void satellite_body::set_orbit_circ(planet* p, double lat, double lon, double alt, double inc) {
    set_mass(1.0);
    set_inertia(1.0, 1.0, 1.0);
    grav_body = p;

    if (inc < lat) {
        // not possible!

        inc = lat;
    }

    vec3d r = grav_body->fixed_to_inertial(grav_body->lla_to_fixed(lat, lon, alt));
    double r_mag = laml::length(r);
    double v_circ = sqrt(grav_body->gm / r_mag);
    double h = r_mag*v_circ;

    vec3d v_dir = laml::normalize(laml::cross(vec3d(0.0, 0.0, 1.0), r));

    // set position to orbit state above launch site
    state.position = r;
    state.velocity = v_dir * v_circ;
}

laml::Vec3_highp satellite_body::force_func(const rigid_body_state* at_state, double t) {
    //return grav_body->gravity(at_state->position)*mass;
    return grav_body->gravity_J2(at_state->position)*mass;
}

//laml::Vec3_highp satellite_body::moment_func(const rigid_body_state& at_state, double t) {
//    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
//}