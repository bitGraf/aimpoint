#include "planet.h"

planet::planet() : mat_inertial_to_fixed(1.0f) {
    //rotation_rate *= .01*86400;
    //gm = 1.0;
    //eccentricity_sq = 0;
    //eccentricity_sq = 0.5; // doenst work
    //eccentricity_sq = 0.7071; // does work?? confusion...
}

void planet::load_mesh() {
    double polar_radius = equatorial_radius * sqrt(1 - eccentricity_sq);
    //mesh.load_from_mesh_file("../data/unit_sphere.mesh", equatorial_radius);
    mesh.load_from_mesh_file("../data/unit_sphere.mesh", equatorial_radius, equatorial_radius, polar_radius);

    diffuse.load_texture_file("../data/earth.jpg");
}

void planet::update(double t, double dt) {
    yaw += rotation_rate * dt;

    const double d360 = 360.0 * laml::constants::deg2rad<double>;
    while (yaw > d360)
        yaw -= d360;

    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);

    //inertial_to_fixed = laml::Mat3(cy, -sy, 0, 0, 0, 1, sy, cy, 0);
    //mat_inertial_to_fixed = laml::Mat3(cy, sy, 0, -sy, cy, 0, 0, 0, 1);
    mat_inertial_to_fixed = laml::Mat3(cy, -sy, 0, sy, cy, 0, 0, 0, 1);
    mat_fixed_to_inertial = laml::transpose(mat_inertial_to_fixed);
}

// for inverse: https://github.com/planet36/ecef-geodetic/blob/main/olson_1996/olson_1996.c#L37
vec3d planet::lla_to_fixed(double lat, double lon, double alt) {
    double slat = laml::sind(lat);
    double clat = laml::cosd(lat);

    double slon = laml::sind(lon);
    double clon = laml::cosd(lon);

    double N = equatorial_radius / sqrt(1.0 - eccentricity_sq*slat*slat);

    return vec3d(
        (N + alt)*clat*clon,
        (N + alt)*clat*slon,
        ((1 - eccentricity_sq)*N + alt)*slat);
}

vec3d planet::fixed_to_inertial(vec3d pos_fixed) {
    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);
    return vec3d(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
}
vec3d planet::fixed_to_inertial(vec3d pos_fixed, double t) {
    double cy = laml::cos(rotation_rate * t);
    double sy = laml::sin(rotation_rate * t);
    return vec3d(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
}



void planet::fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, vec3d* pos_inertial, vec3d* vel_inertial) {
    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);

    vec3d pos(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
    vec3d vel(cy*vel_fixed.x - sy*vel_fixed.y, sy*vel_fixed.x + cy*vel_fixed.y, vel_fixed.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = laml::cross(omega, pos)/r + vel;

    *pos_inertial = pos;
    *vel_inertial = vel;
}
void planet::fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, double t, vec3d* pos_inertial, vec3d* vel_inertial) {
    double cy = laml::cos(rotation_rate * t);
    double sy = laml::sin(rotation_rate * t);

    vec3d pos(cy*pos_fixed.x - sy*pos_fixed.y, sy*pos_fixed.x + cy*pos_fixed.y, pos_fixed.z);
    vec3d vel(cy*vel_fixed.x - sy*vel_fixed.y, sy*vel_fixed.x + cy*vel_fixed.y, vel_fixed.z);

    vec3d omega(0.0, 0.0, rotation_rate);
    double r = laml::length(pos);

    vel = laml::cross(pos, omega)/r + vel;

    *pos_inertial = pos;
    *vel_inertial = vel;
}

vec3d planet::gravity(vec3d pos_inertial) {
    vec3d r_mag = laml::length(pos_inertial);
    vec3d r_unit = pos_inertial / r_mag;

    return -gm * r_unit / (r_mag*r_mag);
}

mat3d planet::create_local_inertial(double lat, double lon, double az) {
    double slat = laml::sind(lat);
    double clat = laml::cosd(lat);

    double slon = laml::sind(lon);
    double clon = laml::cosd(lon);

    double saz = laml::sind(az);
    double caz = laml::cosd(az);

    // 2 step rotation: 3-2 rotation (lon-lat)
    mat3d ECI2NED(
        -clon*slat, -slon, -clat*clon,
        -slat*slon,  clon, -clat*slon,
         clat,       0,    -slat);
    mat3d NED2LCF(caz, -saz, 0,
                  saz, caz, 0,
                  0, 0, 1);

    mat3d LCI2ECI = laml::mul(laml::transpose(ECI2NED), laml::transpose(NED2LCF));

    return LCI2ECI;
}