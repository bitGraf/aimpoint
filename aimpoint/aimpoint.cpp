#include "aimpoint.h"

#include "log.h"

#include <thread>
#include <chrono>

#include <stdio.h>

int aimpoint::run() {
    if (init()) {
        // failed on initialization
        return 1;
    }

    sim_time = 0.0;
    const double step_time = 1.0 / simulation_rate;

    wall_time = renderer.get_time();
    double accum_time = 0.0;

    bool done = false;
    while(!done) {
        double new_time = renderer.get_time();
        double frame_time = new_time - wall_time;
        wall_time = new_time;

        accum_time += frame_time;

        renderer.poll_events();
        if (renderer.should_window_close()) {
            done = true;
        }

        if (real_time) {
            while (accum_time >= step_time) {
                step(step_time);
                sim_time += step_time;
                accum_time -= step_time;

                if (sim_time >= 50.0) { 
                    done = true;
                    break; 
                }
            }
        } else {
            double render_time = wall_time + 1.0/65.0;
            while (renderer.get_time() < render_time) {
                step(step_time);
                sim_time += step_time;
            }
        }

        double alpha = accum_time / step_time; // interpolation value [0,1]
        render();
    }

    shutdown();

    return 0;
}

int aimpoint::init() {
    simulation_rate = 200.0; // Hz
    sim_frame = 0;
    real_time = true;

    sim_time = 0.0;
    wall_time = 0.0;

    body.spring_constant = 100.0;
    body.damping_constant = 0.25;
    body.set_mass(0.1);
    body.state.position.x = 1.0;
    body.state.velocity.x = 0.0;

    body2.set_mass(1.0);
    body2.set_inertia(0.2, 0.3, 0.4);
    body2.state.ang_velocity.x = 0.5;
    body2.state.ang_velocity.y = 25.0;
    body2.state.ang_velocity.z = 0.5;

    cam_pos = laml::Vec3(0.0f, 0.0f, 3.5f);
    yaw = 0;
    pitch = 0;

    renderer.init_gl_glfw(this, 640, 480);

    // Load mesh from file
    mesh.load_from_mesh_file("../data/Cylinder.mesh");

    spdlog::info("Application intitialized");

    // TODO: do these really need to be initialized like this?
    //body.major_step(sim_time, 0.0);
    body2.base_major_step(sim_time, 0.0);

    return 0;
}

void aimpoint::step(double dt) {
    //spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    int64 cycles_per_second = (int64)simulation_rate;
    if (sim_frame % (cycles_per_second) == 0) {
        //spdlog::debug("[{0:0.3f}] simulation step", sim_time);
        double rot_KE = 0.5 * laml::dot(body2.state.ang_velocity*body2.inertia, body2.state.ang_velocity);
        spdlog::debug("[{0:0.3f}] Rotational KE: {1:.3f} J", sim_time, rot_KE);
        //spdlog::info("t(s) = {0:4.1f}     alt(km) = {1:7.3f}     speed(m/s) = {2:7.3f}", 
        //             sim_time, laml::length(body.state.position)/1000.0, laml::length(body.state.velocity));
    }

    //body.major_step(sim_time, dt);
    body.integrate_states(sim_time, dt);
    body2.integrate_states(sim_time, dt);
    
    sim_frame++;
}

void aimpoint::render() {
    renderer.start_frame(cam_pos, yaw, pitch);

    //{
    //    laml::Vec3 render_pos(body.state.position);
    //    laml::Quat render_rot(body.state.orientation);
    //    renderer.draw_mesh(mesh, render_pos, render_rot);
    //}
    {
        laml::Vec3 render_pos(body2.state.position);
        laml::Quat render_rot(body2.state.orientation);
        renderer.draw_mesh(mesh, render_pos, render_rot);
    }

    renderer.end_frame();

    //spdlog::trace("[{0:0.3f}] ({1:0.3f}) render step", sim_time, alpha);
    //spdlog::trace("[{0:0.3f}] render step", sim_time);
}

void aimpoint::shutdown() {
    renderer.shutdown();

    spdlog::info("Application shutdown");
}

void aimpoint::key_callback(int key, int scancode, int action, int mods) {
}












// Entry point
int main(int argc, char** argv) {
    aimpoint app;

    set_terminal_log_level(log_level::trace);
    spdlog::info("Creating application...");

    app.run();

    system("pause");
	
	return 0;
}