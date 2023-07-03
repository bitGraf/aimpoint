#pragma once
#include "defines.h"

#include "render/mesh.h"
#include "render/texture.h"

struct planet {
    planet();

    void load_mesh();
    void update(double t, double dt);

    vec3d lla_to_fixed(double lat, double lon, double alt); // in deg
    void fixed_to_lla(vec3d pos_fixed, double *lat, double *lon, double *alt); // in deg

    vec3d fixed_to_inertial(vec3d pos_fixed);
    vec3d fixed_to_inertial(vec3d pos_fixed, double t);
    void fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, vec3d* pos_inertial, vec3d* vel_inertial);
    void fixed_to_inertial(vec3d pos_fixed, vec3d vel_fixed, double t, vec3d* pos_inertial, vec3d* vel_inertial);

    vec3d inertial_to_fixed(vec3d pos_inertial);
    vec3d inertial_to_fixed(vec3d pos_inertial, double t);
    void  inertial_to_fixed(vec3d pos_inertial, vec3d vel_inertial, vec3d* pos_fixed, vec3d* vel_fixed);
    void  inertial_to_fixed(vec3d pos_inertial, vec3d vel_inertial, double t, vec3d* pos_fixed, vec3d* vel_fixed);

    vec3d gravity(vec3d pos_inertial);
    vec3d gravity_J2(vec3d pos_inertial);

    mat3d create_local_inertial(double lat, double lon, double alt);

    triangle_mesh mesh;
    texture diffuse;

//private:
    // WGS-84 Earth
    double rotation_rate = 72.92115e-6;        // rad/s
    double equatorial_radius = 6378137.0;      // m
    double eccentricity_sq = 6.69437999014e-3; // 
    double yaw = 0.0;                          // rad
    double gm = 3.986004418e14;                // m^3/s^2
    double J2 = 1.75553e25;                    // 

    laml::Mat3 mat_inertial_to_fixed;
    laml::Mat3 mat_fixed_to_inertial;
};