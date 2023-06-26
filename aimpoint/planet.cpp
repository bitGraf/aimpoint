#include "planet.h"

planet::planet() {

}

void planet::load_mesh() {
    //mesh.load_from_mesh_file("../data/unit_sphere.mesh", equatorial_radius);
    mesh.load_from_mesh_file("../data/unit_sphere.mesh", 1.0f);
}

void planet::update(double t, double dt) {
    yaw += rotation_rate * dt * .1 * 86400;

    const double d360 = 360.0 * laml::constants::deg2rad<double>;
    while (yaw > d360)
        yaw -= d360;

    double cy = laml::cos(yaw);
    double sy = laml::sin(yaw);

    //inertial_to_fixed = laml::Mat3(cy, -sy, 0, 0, 0, 1, sy, cy, 0);
    mat_inertial_to_fixed = laml::Mat3(cy, sy, 0, -sy, cy, 0, 0, 0, 1);
}

vec3d planet::lla_to_fixed(double lat, double lon, double alt) {
    double slat = laml::sind(lat);
    double clat = laml::cosd(lat);

    double slon = laml::sind(lon);
    double clon = laml::cosd(lon);

    double N = 1.0f / sqrt(1 - eccentricity_sq*slat*slat);

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