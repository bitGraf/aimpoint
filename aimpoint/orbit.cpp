#include "orbit.h"

static const double trig_tol = 1e-9;

double eccentric_from_mean(const double e, const double mean_deg, double eccentric_guess_deg) {
    const double tol = 1e-12;

    // iterative method to solve
    // M = E - e*sinE for E given e,M
    // M = Mean Anomaly
    // E = Eccentric Anomaly

    // Method:
    // rewrite as E_(n+1) = M + e*sin(E_n)
    // start with E_0 = M;
    // in the future: if this is being propogated: E_0 can be the 'old' value of E
    const double M = mean_deg * laml::constants::deg2rad<double>;
    double E_prev = eccentric_guess_deg * laml::constants::deg2rad<double>;
    double E_new = M + e*laml::sin(E_prev);
    double error = laml::abs(E_new - E_prev);
    E_prev = E_new;
    while (error > tol) {
        E_new = M + e*laml::sin(E_prev);
        error = laml::abs(E_new - E_prev);
        E_prev = E_new;
    }

    return E_new * laml::constants::rad2deg<double>;
}

double true_from_eccentric(const double e, const double eccentric_deg) {
    double cE = laml::cosd(eccentric_deg);
    double true_deg = laml::acosd_safe((cE-e)/(1 - e*cE), trig_tol);
    if (eccentric_deg > 180.0)
        true_deg = 360.0 - true_deg;
    return true_deg;
}

orbit::orbit(const planet& set_body) : body(set_body), path_buffer_created(false) {}

void orbit::create_from_state_vectors(const vec3d& r_vec, const vec3d& v_vec, double T) {
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
    mean_anomaly = eccentric_anomaly - eccentricity*laml::sind(eccentric_anomaly)*laml::constants::rad2deg<double>;

    period = 2*laml::constants::pi<double>*sqrt(semimajor_axis*semimajor_axis*semimajor_axis / body.gm);
    mean_motion = 360.0 / period; // deg/s

    mean_anomaly_at_epoch = fmod(mean_anomaly - T*mean_motion, 360.0);

    // calculate other params
    calc_params(T);
}

void orbit::advance(double dt) {
    mean_anomaly += mean_motion*dt;
    if (mean_anomaly > 360.0)
        mean_anomaly -= 360.0;

    eccentric_anomaly = eccentric_from_mean(eccentricity, mean_anomaly, eccentric_anomaly);
    true_anomaly = true_from_eccentric(eccentricity, eccentric_anomaly);
}

void orbit::calc_params(double t) {
    period = 2*laml::constants::pi<double>*sqrt(semimajor_axis*semimajor_axis*semimajor_axis / body.gm);
    mean_motion = 360.0 / period; // deg/s
    periapsis_alt = semimajor_axis*(1 + eccentricity) - body.equatorial_radius;
    apoapsis_alt = semimajor_axis*(1 - eccentricity) - body.equatorial_radius;

    mean_anomaly = mean_anomaly_at_epoch + mean_motion*t;
    eccentric_anomaly = eccentric_from_mean(eccentricity, mean_anomaly, eccentric_anomaly);
    true_anomaly = true_from_eccentric(eccentricity, eccentric_anomaly);
}

void orbit::get_state_vectors(vec3d* pos_eci, vec3d* vel_eci) {
    double semiminor_axis = semimajor_axis*sqrt(1 - eccentricity*eccentricity);
    double h = mean_motion*semimajor_axis*semiminor_axis*laml::constants::deg2rad<double>;

    // first calculate in perifocal frame
    double r_mag = (h*h / body.gm) * 1.0 / (1 + eccentricity*laml::cosd(true_anomaly));
    vec3d r_w(r_mag*laml::cosd(true_anomaly), r_mag*laml::sind(true_anomaly), 0.0);
    double v_mag = (body.gm / h);
    vec3d v_w(-v_mag*laml::sind(true_anomaly), v_mag*(eccentricity + laml::cosd(true_anomaly)), 0.0);
    v_mag = laml::length(v_w);

    laml::Mat3_highp rot;
    laml::transform::create_ZXZ_rotation(rot, right_ascension, inclination, argument_of_periapsis);

    if (pos_eci)
        *pos_eci = laml::transform::transform_point(rot, r_w);
    if (vel_eci)
        *vel_eci = laml::transform::transform_point(rot, v_w);
}

#include <glad/gl.h>
void orbit::calc_path_mesh() {
    laml::Mat3_highp rot;
    laml::transform::create_ZXZ_rotation(rot, right_ascension, inclination, argument_of_periapsis);
    double semiminor_axis = semimajor_axis*sqrt(1 - eccentricity*eccentricity);
    double h = mean_motion*semimajor_axis*semiminor_axis*laml::constants::deg2rad<double>;

    // sample the orbit at N points along the orbit for rendering
    const size_t N = 100;
    const double spacing = 360.0 / N;
    vec3f path[N];
    uint32 indices[N];

    for (int n = 0; n < N; n++) {
        double theta = spacing*n; // treat at True Anomaly

        // first calculate in perifocal frame
        double r_mag = (h*h / body.gm) * 1.0 / (1 + eccentricity*laml::cosd(theta));
        vec3d r_w(r_mag*laml::cosd(theta), r_mag*laml::sind(theta), 0.0);

        path[n] = laml::transform::transform_point(rot, r_w);
        indices[n] = n;
    }

    // load points into GPU
    if (!path_buffer_created) {
        glGenVertexArrays(1, &path_handle);
        glGenBuffers(1, &path_vbo);
        glGenBuffers(1, &path_ebo);

        glBindVertexArray(path_handle);

        glBindBuffer(GL_ARRAY_BUFFER, path_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*N, path[0]._data, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, path_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*3*N, indices, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);

        path_buffer_created = true;
    } else {
        // only need to buffer new data
        glBindBuffer(GL_ARRAY_BUFFER, path_vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*N, path[0]._data, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}