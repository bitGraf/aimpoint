#include "satellite.h"

#include "log.h"

void satellite_body::set_orbit_circ(planet* p, double lat, double lon, double alt, double inc) {
    set_mass(1.0);
    set_inertia(1.0, 1.0, 1.0);
    grav_body = p;

    double geocentric_lat = laml::atand((1 - grav_body->eccentricity_sq) * laml::tand(lat));
    if (inc < geocentric_lat)
        inc = geocentric_lat;

    //TODO: need to figure this algorithm out.
    //      stuck at a point where RAAN and Arg.Periapsis need to be chosen still
    //double theta = laml::asind_safe(geocentric_lat / inc, trig_tol);
    double theta = laml::asind_safe(laml::tand(geocentric_lat) / laml::tand(inc), trig_tol);
    double Omega = lon - theta;
    if (Omega < 0.0)
        Omega += 360.0;
    double arg_periapsis = laml::acosd_safe(laml::cosd(theta)*laml::cosd(geocentric_lat), trig_tol);
    if (arg_periapsis < 0.0)
        arg_periapsis += 360.0;

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