#include "flat_earth_rocket.h"

#include "log.h"

rocket_flat_earth_ltg::rocket_flat_earth_ltg() {
    tof = 0.0;
    stage = 0;

    set_mass(750000.0);
    set_inertia(1, 1, 1);

    major_loop_rate = 1.0; // Hz
    update_step_num = 0;

    thrust = 10000000;
    mdot = -2.2661e03;
}

void rocket_flat_earth_ltg::launch(planet* grav_body, double orbit_height, double Tgo_guess) {
    body = grav_body;

    state.position = vec3d(0.0, 0.0, 0.0);
    state.velocity = vec3d(0.0, 0.0, 0.0);

    yf = orbit_height;
    double rf = yf + body->equatorial_radius;
    vyf = 0.0;
    vxf = sqrt(body->gm / rf);

    T = Tgo_guess;
    A = 1.0;
    B = 0.0;

    //
    A = 2.4304;
    B = -0.0097;
    T = 292.2326;

    A = 1;
    B = -0.01;
    B = 0;
    T = 280;

    Tgo = T;
    alpha = laml::atand(A + B*0);
    t0 = 0; // should be launch time
}

void rocket_flat_earth_ltg::major_step(double t, double dt) {
    uint64 loop_steps = uint64(1.0 / (dt*major_loop_rate));
    if (Tgo > 10.0) {
        if (update_step_num % loop_steps == 0) {
            // major loop update!

            //calc_new_terms(t, mass, state.position.y, state.velocity.x, state.velocity.y, Tgo, A, B);
            //A = A + B*(t-t0);
            const uint32 N = 1000; // number of points from 0 to T for num. integration
            double ds;

            double dvx_p = 0.0;
            double dvy_p = 0.0;
            double dy_p = 0.0;
            //double x0 = state.position.y;
            double y0 = -state.position.z;
            double vx0 = state.velocity.y;
            double vy0 = -state.velocity.z;
            double m0 = mass;
            double g = 9.80655;
            ds = (Tgo-0.0) / double(N-1);
            mat3d C(0.0);
            for (uint32 n = 0; n < N; n++) {
                double s = n*ds;
                double m = m0 + mdot*s;
                double acc = thrust / m;

                double t_alpha = (A + B*s);
                double c_alpha = 1.0 / sqrt(1.0 + t_alpha*t_alpha);
                double s_alpha = t_alpha * c_alpha;

                double dc_dA = -t_alpha* pow(t_alpha*t_alpha + 1.0, -1.50);
                double dc_dB = s*dc_dA;

                double ds_dA = -(A*A - (t_alpha*t_alpha) + B*B*s*s + 2*A*B*s - 1.0) * pow(t_alpha*t_alpha + 1.0, -1.50);
                double ds_dB = s*pow(t_alpha*t_alpha + 1.0, -1.50);

                // integrate state using current steering law
                dvx_p += (acc * c_alpha) * ds;
                dvy_p += (acc * s_alpha - g) * ds;
                dy_p -= s * (acc * s_alpha - g) * ds;

                // Calculate coefficients
                C.c_11 = acc*c_alpha; // silly way: this just saves the last iteration: we WANT acc_T and c_alpha_T
                C.c_12 += (acc*dc_dA)*ds;
                C.c_13 += (acc*dc_dB)*ds;

                C.c_21 = acc*s_alpha - g; // silly way: this just saves the last iteration: we WANT acc_T and c_alpha_T
                C.c_22 += (acc*ds_dA)*ds;
                C.c_23 += (acc*ds_dB)*ds;

                C.c_31 = s*(acc*s_alpha - g); // silly way: this just saves the last iteration: we WANT acc_T and c_alpha_T
                C.c_32 += (s*acc*ds_dA)*ds;
                C.c_33 += (s*acc*ds_dB)*ds;
            }

            vec3d e(vxf - (dvx_p + vx0),
                    vyf - (dvy_p + vy0),
                    -(yf - (dy_p + y0)));

            // solve matrix equation:
            // C*K = e
            // K = inv(C)*e
            // first check if C is invertible:
            double det = laml::det(C);
            if (laml::abs(det) < 1.0e-3) {
                spdlog::warn("det(C) = {0}", det);
            }
            vec3d K = laml::transform::transform_point(laml::inverse(C), e);

            Tgo = Tgo + K.x;
            A = A + K.y;
            B = B + K.z;
            T = t + Tgo;

            spdlog::info("[{0:.3f}] Major Guidance loop: T={4:.3f}sec  Tgo={1:.3f} A={2:.3f} B={3:.5f}", t, Tgo, A, B, T);

            t0 = t;
        }
    } else {
        // Terminal Guidance
        Tgo -= dt;
    }

    if (Tgo > 0.0) {
        if (Tgo > 10.0) {
            // use the existing steering coefficients
            double t_alpha = A + B*(t-t0);
            alpha = laml::atand(t_alpha);
            double s_alpha = (t_alpha) / sqrt(1 + t_alpha*t_alpha);
            double c_alpha = (1) / sqrt(1 + t_alpha*t_alpha);
    
    
            vec3d dir(0.0, c_alpha, -s_alpha);
            apply_force(dir*thrust, vec3d());
            set_mass(mass + mdot*dt);

            mat3d rot;
            laml::transform::create_ZXZ_rotation(rot, 0.0, alpha, 0.0);
            state.orientation = laml::transform::quat_from_mat(laml::transpose(rot));
        } else {
            double m = mass;
            double y = -state.position.z;
            double vy = -state.velocity.z;
            double vx = state.velocity.y;
            double alpha = laml::atan2d(-vy, vxf - vx);
            double delH = yf - y;
            double throttle = (-m*vy*vy / (2*laml::sind(alpha)*delH*thrust) - 0.3) / 0.7;
            //(-m * vy * vy / (2 * (yf - y) * laml::sind(alpha)) / thrust - 0.3) / 0.7
            if (throttle > 1.0)
                throttle = 1.0;
            else if (throttle < 0.001)
                throttle = 0.001;

            vec3d dir(0.0, laml::cosd(alpha), -laml::sind(alpha));
            apply_force(throttle*dir*thrust, vec3d());
            set_mass(mass + mdot*dt);

            mat3d rot;
            laml::transform::create_ZXZ_rotation(rot, 0.0, alpha, 0.0);
            state.orientation = laml::transform::quat_from_mat(laml::transpose(rot));
            if (update_step_num % 100 == 0)
                spdlog::info("[{0:.3f}] Terminal: Tgo={1:.3f} alpha={2:.3f} throttle={3:.5f}", t, Tgo, alpha, throttle);
        }
    }
    update_step_num++;
    tof += dt;
}

laml::Vec3_highp rocket_flat_earth_ltg::force_func(const rigid_body_state* at_state, double t) {
    //return laml::Vec3(0.0f, -9.81f*state.mass, 0.0f);
    //return earth->gravity(at_state->position)*mass;
    if (tof < 292.2326) {
        return vec3d(0.0, 0.0, 9.80655*mass);
    } else {
        return vec3d();
    }
}

//laml::Vec3_highp rocket::moment_func(const rigid_body_state& at_state, double t) {
//    return laml::Vec3_highp(0.0f, 0.0f, 0.0f);
//}