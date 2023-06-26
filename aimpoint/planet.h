#pragma once
#include "defines.h"

#include "render/mesh.h"

struct planet {
    planet();

    void load_mesh();
    void update(double t, double dt);

    vec3d lla_to_fixed(double lat, double lon, double alt); // in deg
    vec3d fixed_to_inertial(vec3d pos_fixed);

    triangle_mesh mesh;

//private:
    // WGS-84 Earth
    double rotation_rate = 72.92115e-6;        // rad/s
    double equatorial_radius = 6378137.0;      // m
    double eccentricity_sq = 6.69437999014e-3; // 
    double yaw = 0.0;                          // rad

    laml::Mat3 mat_inertial_to_fixed;
};