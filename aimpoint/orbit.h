#pragma once
#include "defines.h"

#include "planet.h"

struct orbit {
    orbit(const planet& set_body);

    // 6 Keplerian Elements - At Epoch
    double eccentricity;
    double semimajor_axis;
    double inclination;
    double right_ascension;
    double argument_of_periapsis;
    double mean_anomaly_at_epoch;

    // other parameters
    double mean_anomaly;
    double eccentric_anomaly;
    double true_anomaly;
    double mean_motion;
    double period;

    double periapsis_alt;
    double apoapsis_alt;

    void create_from_state_vectors(const vec3d& pos_eci, const vec3d& vel_eci, double T);
    void calc_params(double t);
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