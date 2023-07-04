#pragma once
#include "defines.h"

#include "planet.h"

struct orbit {
    orbit(const planet& set_body);

    // 6 Keplerian Elements - Define at Epoch and drive the rest
    double eccentricity;
    double semimajor_axis;
    double inclination;
    double right_ascension;
    double argument_of_periapsis;
    double mean_anomaly_at_epoch;

    // other constant parameters
    double periapsis_alt;
    double apoapsis_alt;
    double mean_motion;
    double period;
    double specific_ang_momentum;
    double specific_energy;
    vec3d specific_ang_momentum_unit;
    vec3d ascending_node_unit;
    vec3d apsis_line_unit;
    mat3d perifocal_to_inertial;

    // time varying anomalies
    double true_anomaly;
    double mean_anomaly;
    double eccentric_anomaly;

    void create_from_state_vectors(const vec3d& pos_eci, const vec3d& vel_eci, double T);
    void create_from_kep_elements(double e, double a, double i, double Omega, double omega, double M0, double T);
    void advance(double dt);
    void get_state_vectors(vec3d* pos_eci = nullptr, vec3d* vel_eci = nullptr);

    void calc_path_mesh();

    uint32 path_handle;
    uint32 path_vbo, path_ebo;
private:
    // the orbital body
    const planet& body;

    bool path_buffer_created;
};