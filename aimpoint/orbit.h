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

    void initialize(const vec3d& pos_eci, const vec3d& vel_eci);
    void calc_params(double t);
    void advance(double dt);
    void get_state_vectors(vec3d* pos_eci = nullptr, vec3d* vel_eci = nullptr);

private:
    // the orbital body
    const planet& body;
};