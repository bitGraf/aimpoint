#include "orbit.h"

orbit::orbit(const planet& set_body) : body(set_body) {}

void orbit::initialize(const vec3d& r_vec, const vec3d& v_vec) {
    // Reference Frame - ECI
    vec3d I_eci = vec3d(1.0, 0.0, 0.0);
    vec3d J_eci = vec3d(0.0, 1.0, 0.0);
    vec3d K_eci = vec3d(0.0, 0.0, 1.0);

    // Magnitude and Unit Vectors
    double r_mag = laml::length(r_vec);
    double v_mag = laml::length(v_vec);
    vec3d r_hat = r_vec / r_mag;
    vec3d v_hat = v_vec / v_mag;

    // angular momentum
    vec3d h_vec = laml::cross(r_vec, v_vec);
    double h_mag = laml::length(h_vec);
    vec3d h_hat = h_vec / h_mag;

    // inclination
    //inclination = laml::acosd(laml::dot(K_eci, h_vec) / h_mag);
    inclination = laml::acosd_safe(h_vec.z / h_mag, trig_tol); // inclination is [0,180] so ambigious acos() is fine.

    // right ascension of the ascending node (RAAN)
    //vec3d k_hat = h_vec / h_mag; // orbit axis
    vec3d n_vec = laml::cross(K_eci, h_vec);
    double n_mag = laml::length(n_vec);
    //right_ascension = laml::acosd(laml::dot(I_eci, n_vec) / n_mag);
    right_ascension = laml::acosd_safe(n_vec.x / n_mag, trig_tol);
    if (n_vec.y < 0.0) // acos is [0,180] but RAAN is [0,360], so need to determine proper quadrant
        right_ascension = 360 - right_ascension;

    // eccentricity and semimajor axis
    // TODO: check eccentricity for class of orbit - some math wont work with e=0!
    vec3d e_vec = laml::cross(v_vec, h_vec) / body.gm - r_hat;
    eccentricity = laml::length(e_vec);
    vec3d e_hat = e_vec / eccentricity;
    double semi_latus = h_mag*h_mag / body.gm;
    semimajor_axis = semi_latus / (1 - eccentricity*eccentricity);

    // arg. of periapsis
    argument_of_periapsis = laml::acosd_safe(laml::dot(n_vec, e_vec)/(n_mag*eccentricity), trig_tol);
    if (e_vec.z < 0.0)
        argument_of_periapsis = 360.0 - argument_of_periapsis;

    // True/Eccentric/Mean anomaly
    true_anomaly = laml::acosd_safe(laml::dot(e_vec, r_vec) / (eccentricity*r_mag), trig_tol);
    if (laml::dot(r_vec, v_vec) < 0.0)
        true_anomaly = 360.0 - true_anomaly;
    eccentric_anomaly = laml::acosd_safe((eccentricity + laml::cosd(true_anomaly)) / (1 + eccentricity*laml::cosd(true_anomaly)), trig_tol);
    if (true_anomaly > 180.0)
        eccentric_anomaly = 360.0 - eccentric_anomaly;
    mean_anomaly_at_epoch = eccentricity - eccentricity*laml::sind(eccentric_anomaly);

    // calculate other params
    calc_params(0.0);
}

void orbit::advance(double dt) {
    mean_anomaly += mean_motion*dt;
    if (mean_anomaly > 360.0)
        mean_anomaly -= 360.0;

    calc_true_anomaly();
    eccentric_anomaly = laml::acosd_safe((eccentricity + laml::cosd(true_anomaly)) / (1 + eccentricity*laml::cosd(true_anomaly)), trig_tol);
    if (true_anomaly > 180.0)
        eccentric_anomaly = 360.0 - eccentric_anomaly;
}

void orbit::calc_params(double t) {
    period = 2*laml::constants::pi<double>*sqrt(semimajor_axis*semimajor_axis*semimajor_axis / body.gm);
    mean_motion = 360.0 / period; // deg/s
    mean_anomaly = mean_anomaly_at_epoch + mean_motion*t;

    calc_true_anomaly();
}

void orbit::calc_true_anomaly() {
    // iterative method to solve
    // M = E - e*sinE for E given e,M
    // M = Mean Anomaly
    // E = True Anomaly (v)

    // Method:
    // rewrite as E_(n+1) = M + e*sin(E_n)
    // start with E_0 = M;
    // in the future: if this is being propogated: E_0 can be the 'old' value of E
    if (true_anomaly > 180.0 && true_anomaly < 185.0)
        bool sto = true;
    const double M = mean_anomaly * laml::constants::deg2rad<double>;
    const double e = eccentricity;
    double E_prev = M;
    double E_new = M + e*laml::sin(E_prev);
    double error = laml::abs(E_new - E_prev);
    E_prev = E_new;
    const double tol = 1e-12;
    while (error > tol) {
        E_new = M + e*laml::sin(E_prev);
        error = laml::abs(E_new - E_prev);
        E_prev = E_new;
    }

    true_anomaly = E_new * laml::constants::rad2deg<double>;
}

void orbit::get_state_vectors(vec3d* pos_eci, vec3d* vel_eci) {

}