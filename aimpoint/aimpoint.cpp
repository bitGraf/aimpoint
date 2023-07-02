#include "aimpoint.h"

#include "log.h"

#include <thread>
#include <chrono>

#include <stdio.h>

#include "imgui.h"
#include "implot.h"

// TMP for KEY_CODES!
#include <GLFW/glfw3.h>

int aimpoint::run() {
    if (init()) {
        // failed on initialization
        return 1;
    }

    sim_time = 0.0;
    const double step_time = 1.0 / simulation_rate;

    wall_time = renderer.get_time();
    double accum_time = 0.0;
    frame_time = 0.0;

    bool done = false;
    while(!done) {
        double new_time = renderer.get_time();
        frame_time = new_time - wall_time;
        wall_time = new_time;

        accum_time += frame_time;

        input.xvel = 0.0;
        input.yvel = 0.0;
        renderer.poll_events();
        if (renderer.should_window_close()) {
            done = true;
        }
        input.xvel /= frame_time;
        input.yvel /= frame_time;

        if (real_time) {
            while (accum_time >= step_time) {
                step(step_time);
                accum_time -= step_time;

                //if (sim_time >= 10.0) { 
                //    done = true;
                //    break; 
                //}
            }
        } else {
            double render_time = wall_time + 1.0/65.0;
            while (renderer.get_time() < render_time) {
                step(step_time);
            }
        }

        double alpha = accum_time / step_time; // interpolation value [0,1]
        render();
    }

    shutdown();

    return 0;
}

int aimpoint::init() {
    simulation_rate = 1.0; // Hz
    sim_frame = 0;
    render_frame = 0;
    real_time = true;

    sim_time = 0.0;
    wall_time = 0.0;

    cam_orbit_point = laml::Vec3(0.0f, 0.0f, 0.0f);
    cam_orbit_distance = 2.0f*earth.equatorial_radius;
    yaw = -65;
    pitch = -15;

    renderer.init_gl_glfw(this, 800, 600);

    // Load mesh from file
    //mesh.load_from_mesh_file("../data/t_bar.mesh");
    mesh.load_from_mesh_file("../data/blahaj.mesh", 0.01f);
    dot.load_from_mesh_file("../data/unit_sphere.mesh", 100000.0f);
    grid_tex.load_texture_file("../data/grid.png");

    red_tex.load_texture_file("../data/red.png");
    green_tex.load_texture_file("../data/green.png");
    blue_tex.load_texture_file("../data/blue.png");

    earth.load_mesh();

    body.launch(&earth);
    lci2eci = body.LCI2ECI;
    eci2lci = laml::transpose(lci2eci);

    kep.initialize(body.state.position, body.state.velocity*1.0, 0.0);
    kep.calc_path_mesh();
    kep2.initialize(body.state.position, body.state.velocity*1.0, 0.0);
    kep2.calc_path_mesh();

    spdlog::info("Application intitialized");

    return 0;
}

void aimpoint::step(double dt) {
    //spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    //int64 cycles_per_second = (int64)simulation_rate;
    //if (sim_frame % (cycles_per_second) == 0) {
    //    spdlog::info("[{0:0.3f}] simulation step", sim_time);
    //}

    earth.update(sim_time, dt);
    body.integrate_states(sim_time, dt);
    kep.advance(dt);

    if (sim_frame % frames_per_min == 0) {
        t.add_point(sim_time);
        M.add_point(kep.mean_anomaly);
        E.add_point(kep.eccentric_anomaly);
        v.add_point(kep.true_anomaly);
    }
    
    sim_time += dt;
    sim_frame++;
}

void aimpoint::render() {
    if (input.mouse2) {
        yaw   -= input.xvel * frame_time * 0.75f;
        pitch -= input.yvel * frame_time * 0.50f;
    }

    if (earth.yaw > 90.0*laml::constants::deg2rad<double>) {
        bool stop = true;
    }

    laml::Mat3 eci_to_render(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    laml::Mat3 lci_to_render(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
    laml::Mat3 render_coord_frame(1.0f);
    float zoom_level = 1.0f;
    switch(render_frame_enum) {
        case ECI: {
            render_coord_frame = eci_to_render;
            cam_orbit_point = vec3f(0.0f);
        } break;
        case ECEF: {
            render_coord_frame = laml::mul(eci_to_render, earth.mat_inertial_to_fixed);
            cam_orbit_point = vec3f(0.0f);
        } break;
        case LCI: {
            mat3f _eci2lci(eci2lci.c_11, eci2lci.c_21, eci2lci.c_31,
                           eci2lci.c_12, eci2lci.c_22, eci2lci.c_32,
                           eci2lci.c_13, eci2lci.c_23, eci2lci.c_33);
            render_coord_frame = laml::mul(lci_to_render, _eci2lci);

            vec3f lci_point = laml::transform::transform_point(eci2lci, earth.fixed_to_inertial(earth.lla_to_fixed(body.launch_lat,body.launch_lon,0.0), 0.0));
            cam_orbit_point = laml::transform::transform_point(lci_to_render, lci_point);
            zoom_level = 0.25f;
        } break;
        case LCF: {
            mat3f _eci2lci(eci2lci.c_11, eci2lci.c_21, eci2lci.c_31,
                           eci2lci.c_12, eci2lci.c_22, eci2lci.c_32,
                           eci2lci.c_13, eci2lci.c_23, eci2lci.c_33);
            render_coord_frame = laml::mul(lci_to_render, laml::mul(_eci2lci, earth.mat_inertial_to_fixed));
            vec3f lci_point = laml::transform::transform_point(eci2lci, earth.fixed_to_inertial(earth.lla_to_fixed(body.launch_lat,body.launch_lon,0.0), 0.0));
            cam_orbit_point = laml::transform::transform_point(lci_to_render, lci_point);
            zoom_level = 0.25f;
        } break;
    }

    laml::Vec3 cam_pos = cam_orbit_point + 
                         zoom_level*cam_orbit_distance*laml::Vec3(
                            laml::cosd(pitch)*laml::sind(yaw), 
                           -laml::sind(pitch), 
                            laml::cosd(pitch)*laml::cosd(yaw));

    renderer.start_frame(cam_pos, yaw, pitch, render_coord_frame);

    renderer.bind_texture(earth.diffuse);
    renderer.draw_mesh(earth.mesh, laml::Vec3(0.0f), laml::transform::quat_from_mat(earth.mat_fixed_to_inertial));

    renderer.bind_texture(red_tex);
    {
        vec3f pos_eci(earth.fixed_to_inertial(earth.lla_to_fixed(0, 0, 0)));
        renderer.draw_mesh(dot, pos_eci, laml::transform::quat_from_mat(earth.mat_fixed_to_inertial));

        pos_eci = vec3f((earth.lla_to_fixed(0, 0, 0)));
        renderer.draw_mesh(dot, pos_eci, laml::Quat());

        pos_eci = earth.fixed_to_inertial(earth.lla_to_fixed(body.launch_lat,body.launch_lon,0.0));
        renderer.draw_mesh(dot, pos_eci, laml::Quat());
    }
    renderer.bind_texture(green_tex);
    {
        // Fixed
        vec3f pos_eci(earth.fixed_to_inertial(earth.lla_to_fixed(0, 90, 0)));
        renderer.draw_mesh(dot, pos_eci, laml::transform::quat_from_mat(earth.mat_fixed_to_inertial));

        // Inertial
        pos_eci = vec3f((earth.lla_to_fixed(0, 90, 0)));
        renderer.draw_mesh(dot, pos_eci, laml::Quat());
    }
    renderer.bind_texture(blue_tex);
    {
        //if (render_frame_enum == ECEF) {
            vec3f pos_eci(earth.fixed_to_inertial(earth.lla_to_fixed(90, 0, 0)));
            renderer.draw_mesh(dot, pos_eci, laml::transform::quat_from_mat(earth.mat_fixed_to_inertial));
        //} else {
        //    vec3f pos_eci((earth.lla_to_fixed(90, 0, 0)));
        //    renderer.draw_mesh(dot, pos_eci, laml::Quat(), render_frame);
        //}
    }

    renderer.bind_texture(grid_tex);
    renderer.draw_mesh(dot, body.state.position, body.state.orientation);
    vec3d pos_kep;
    double r_mag_ = laml::length(body.state.position);
    double v_mag_ = laml::length(body.state.velocity);
    double h_mag = laml::length(laml::cross(body.state.position, body.state.velocity));
    kep.get_state_vectors(&pos_kep);
    renderer.bind_texture(red_tex);
    renderer.draw_mesh(dot, pos_kep, body.state.orientation);
    renderer.draw_path(kep.path_handle, 100);

    kep2.initialize(body.state.position, body.state.velocity, sim_time);
    kep2.calc_path_mesh();
    renderer.draw_path(kep2.path_handle, 100);
    //vec3d launch_eci = earth.fixed_to_inertial(earth.lla_to_fixed(28.3922, -80.6077, 0.0), 0.0);
    //for (int n = 0; n < 5; n++) {
    //    vec3d pos_lci(0.0, 0.0, -0.2*n);
    //    vec3d pos_eci = launch_eci + laml::transform::transform_point(lci_2_eci, pos_lci);
    //
    //    renderer.draw_mesh(dot, pos_eci, laml::Quat(), render_frame);
    //}

    // Draw UI
    renderer.start_debug_UI();

    const ImGuiIO& io = ImGui::GetIO();

    //ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin(".", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    switch(render_frame_enum) {
        case ECI:
        case ECEF: {
            ImGui::Text("Earth-Centered");
        } break;
        case LCI:
        case LCF: {
            ImGui::Text("Launch-Centered");
        } break;
    }
    ImGui::SameLine();
    switch(render_frame_enum) {
        case ECI:
        case LCI: {
            ImGui::Text("Inertial");
        } break;
        case ECEF:
        case LCF: {
            ImGui::Text("Fixed");
        } break;
    }
    ImGui::End();

    if (show_info_panel) {
        ImGui::Begin("Simulation Info");

        static uint64 last_frames = 0;
        double steps_per_second = (double)(sim_frame - last_frames);
        last_frames = sim_frame;
        double sim_scale = io.Framerate * steps_per_second * (1.0/simulation_rate);
        sim_scale_history.add_point(sim_scale);
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::Text("Sim Scale: %5.1fX", sim_scale_history.get_avg());
        //ImGui::Text("xpos=%.1f   ypos=%.1f", input.xpos, input.ypos);
        //ImGui::Text("xvel=%.1f   yvel=%.1f", input.xvel, input.yvel);
        ImGui::Separator();

        //ImGui::Text("Mouse Buttons:");
        //ImGui::Text("M1:%s    M2:%s", input.mouse1 ? "Pressed" : "Released", input.mouse2 ? "Pressed" : "Released");
        //ImGui::Separator();

        ImGui::Text("Camera State:");
        ImGui::Text("Yaw:%.2f    Pitch:%.2f", yaw, pitch);
        ImGui::Text("[%.1f, %.1f, %.1f]", cam_pos.x, cam_pos.y, cam_pos.z);
        ImGui::Separator();

        ImGui::Text("Earth State:");
        ImGui::Text("Yaw:%.2f", earth.yaw*laml::constants::rad2deg<double>);
        ImGui::Text("Rendering Frame: %s", render_frame_enum == ECI ? "Inertial" : (render_frame_enum == ECEF ? "Fixed" : "LCI"));
        ImGui::Separator();

        ImGui::Text("Rocket State:");
        ImGui::Text("Pos: [%.2f, %.2f, %.2f] km", body.state.position.x/1000.0, body.state.position.y/1000.0, body.state.position.z/1000.0);
        ImGui::Text("Vel: [%.2f, %.2f, %.2f] m/s", body.state.velocity.x, body.state.velocity.y, body.state.velocity.z);
        ImGui::Text("Acc: [%.2f, %.2f, %.2f] m/s^2", body.derivative.acceleration.x, body.derivative.acceleration.y, body.derivative.acceleration.z);
        //ImGui::Text("Rendering Frame: %s", render_frame_enum == 0 ? "Inertial" : "Fixed");
        ImGui::Separator();

        ImGui::End();
    }

    if (show_keplerian_panel) {
        ImGui::Begin("Keplerian Elements");

        /*
        double eccentricity;
        double semimajor_axis;
        double inclination;
        double right_ascension;
        double argument_of_periapsis;
        double mean_anomaly;

        double mean_motion;
        double period;
         */

        ImGui::Text("Eccentricity: %.5f", kep.eccentricity);
        ImGui::Text("Semimajor Axis: %.1f km", kep.semimajor_axis/1000.0);
        ImGui::Text("Inclination: %.2f deg", kep.inclination);
        ImGui::Text("RAAN: %.2f deg", kep.right_ascension);
        ImGui::Text("Arg. of Periapsis: %.2f deg", kep.argument_of_periapsis);
        ImGui::Text("Mean Anomaly (Epoch): %.2f deg", kep.mean_anomaly_at_epoch);
        ImGui::Separator();
        ImGui::Text("Mean Anomaly: %.5f deg", kep.mean_anomaly);
        ImGui::Text("Ecc  Anomaly: %.5f deg", kep.eccentric_anomaly);
        ImGui::Text("True Anomaly: %.5f deg", kep.true_anomaly);
        ImGui::Text("Mean Motion: %.5f deg/s", kep.mean_motion);
        ImGui::Text("Period: %.3f min", kep.period / 60.0);
        ImGui::Separator();

        ImGui::Text("Eccentricity: %.5f", kep2.eccentricity);
        ImGui::Text("Semimajor Axis: %.1f km", kep2.semimajor_axis/1000.0);
        ImGui::Text("Inclination: %.2f deg", kep2.inclination);
        ImGui::Text("RAAN: %.2f deg", kep2.right_ascension);
        ImGui::Text("Arg. of Periapsis: %.2f deg", kep2.argument_of_periapsis);
        ImGui::Text("Mean Anomaly (Epoch): %.2f deg", kep2.mean_anomaly_at_epoch);
        ImGui::Separator();

        ImGui::End();

        if (show_anomoly_panel) {
            ImGui::Begin("Anomalies");

            double history = t.length;
            static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
            if (ImPlot::BeginPlot("Anomaly")) {
                ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
                ImPlot::SetupAxisLimits(ImAxis_X1,(sim_time - history), sim_time, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1,0,360);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);

                ImPlot::PlotLine("Mean", t.data, M.data, t.length, 0, t.offset, sizeof(double));
                ImPlot::PlotLine("Eccentric", t.data, E.data, t.length, 0, t.offset, sizeof(double));
                ImPlot::PlotLine("True", t.data, v.data, t.length, 0, t.offset, sizeof(double));

                ImPlot::EndPlot();
            }

            ImGui::End();
        }
    }

#if 0
    // Plots
    w_t.add_point(sim_time);
    w_x.add_point(body.state.ang_velocity.x);
    w_y.add_point(body.state.ang_velocity.y);
    w_z.add_point(body.state.ang_velocity.z);

    ImGui::Begin("Body Rates");
    double history = w_t.length * (1.0/60.0);
    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;
    if (ImPlot::BeginPlot("##Scrolling")) {
        ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,sim_time - history, sim_time, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,-10,10);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);

        ImPlot::PlotLine("Body X", w_t.data, w_x.data, w_t.length, 0, w_t.offset, sizeof(double));
        ImPlot::PlotLine("Body Y", w_t.data, w_y.data, w_t.length, 0, w_t.offset, sizeof(double));
        ImPlot::PlotLine("Body Z", w_t.data, w_z.data, w_t.length, 0, w_t.offset, sizeof(double));

        ImPlot::EndPlot();
    }
    ImGui::End();
#endif

    renderer.end_debug_UI();

    renderer.end_frame();

    //spdlog::trace("[{0:0.3f}] ({1:0.3f}) render step", sim_time, alpha);
    //spdlog::trace("[{0:0.3f}] render step", sim_time);

    render_frame++;
}

void aimpoint::shutdown() {
    renderer.shutdown();

    spdlog::info("Application shutdown");
}

void aimpoint::key_callback(int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_I && action == GLFW_PRESS) {
        show_info_panel = !show_info_panel;
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        real_time = !real_time;
    }

    // toggle frame (Earth-centered vs. Launch site)
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        switch(render_frame_enum) {
            case ECI: {
                render_frame_enum = LCI;
            } break;
            case ECEF: {
                render_frame_enum = LCF;
            } break;
            case LCI: {
                render_frame_enum = ECI;
            } break;
            case LCF: {
                render_frame_enum = ECEF;
            } break;
        }
    }

    // toggle frame (Fixed vs. Inertial)
    if (key == GLFW_KEY_F && action == GLFW_RELEASE) {
        switch(render_frame_enum) {
            case ECI: {
                render_frame_enum = ECEF;
            } break;
            case ECEF: {
                render_frame_enum = ECI;
            } break;
            case LCI: {
                render_frame_enum = LCF;
            } break;
            case LCF: {
                render_frame_enum = LCI;
            } break;
        }
    }

    // toggle orbital param displays
    if (key == GLFW_KEY_K && action == GLFW_RELEASE) {
        show_keplerian_panel = !show_keplerian_panel;
    }
    if (key == GLFW_KEY_A && action == GLFW_RELEASE) {
        show_anomoly_panel = !show_anomoly_panel;
    }
}

void aimpoint::mouse_pos_callback(double xpos, double ypos) {
    input.xvel += (xpos - input.xpos);
    input.yvel += (ypos - input.ypos);

    input.xpos = xpos;
    input.ypos = ypos;
}

void aimpoint::mouse_button_callback(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
        input.mouse1 = true;
    }
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_RELEASE) {
        input.mouse1 = false;
    }

    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_PRESS) {
        input.mouse2 = true;
    }
    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_RELEASE) {
        input.mouse2 = false;
    }
}











// Entry point
int main(int argc, char** argv) {
    aimpoint app;

    set_terminal_log_level(log_level::info);
    spdlog::info("Creating application...");

    app.run();

    system("pause");
	
	return 0;
}