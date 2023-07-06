#include "aimpoint.h"

#include "log.h"

#include <thread>
#include <chrono>

#include <stdio.h>

#include "imgui.h"
#include "implot.h"

// TMP for KEY_CODES!
#include <GLFW/glfw3.h>

static char* pretty_time(double time, double* value) {
    double abs_time = laml::abs(time);
    double sign = laml::sign(time);

    if (abs_time < 2.0*60.0) {
        *value = abs_time * sign;
        return "s";
    }

    if (abs_time < 2.0*60.0*60.0) {
        *value = abs_time * sign / 60.0;
        return "min";
    }

    if (abs_time < 2*60.0*60.0*24.0) {
        *value = abs_time * sign / (60.0*60.0);
        return "hours";
    }

    *value = abs_time * sign / (60.0*60.0*24.0);
    return "days";
}

int aimpoint::init() {
    // Load mesh from file
    //mesh.load_from_mesh_file("data/t_bar.mesh");
    mesh.load_from_mesh_file("data/blahaj.mesh", 0.01f);
    dot.load_from_mesh_file("data/unit_sphere.mesh", 100000.0f);
    grid_tex.load_texture_file("data/grid.png");

    red_tex.load_texture_file("data/red.png");
    green_tex.load_texture_file("data/green.png");
    blue_tex.load_texture_file("data/blue.png");

    earth.load_mesh();

    //satellite.set_orbit_circ(&earth, 28.627023, -80.620856, 480000, 40);
    satellite.set_orbit_circ(&earth, 69.099597, 49.092329, 250000, 75);
    constant_orbit.create_from_state_vectors(satellite.state.position, satellite.state.velocity, 0.0);
    //constant_orbit.create_from_kep_elements(0.0, 7000000, 30, 0, 0, 0, 0);
    constant_orbit.calc_path_mesh();
    J2_perturbations.create_from_state_vectors(satellite.state.position, satellite.state.velocity, 0.0);
    J2_perturbations.calc_path_mesh();

    hmm.launch(&earth);
    lci2eci = hmm.LCI2ECI;
    eci2lci = laml::transpose(lci2eci);

    mat4f projection_matrix;
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, renderer.get_AR(), 1000.0f, 50'000'000.0f);
    renderer.set_projection(projection_matrix);
    cam_orbit_distance = 2.0f*earth.equatorial_radius;

    spdlog::info("Aimpoint intitialized");

    return 0;
}

void aimpoint::step(double dt) {
    //spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    //int64 cycles_per_second = (int64)simulation_rate;
    //if (sim_frame % (cycles_per_second) == 0) {
    //    spdlog::info("[{0:0.3f}] simulation step", sim_time);
    //}

    earth.update(sim_time, dt);
    satellite.integrate_states(sim_time, dt);
    constant_orbit.advance(dt);

    if (sim_frame % frames_per_min == 0) {
        t.add_point(sim_time);
        M.add_point(constant_orbit.mean_anomaly);
        E.add_point(constant_orbit.eccentric_anomaly);
        v.add_point(constant_orbit.true_anomaly);
    }
}

void aimpoint::render2D() {
    //renderer.start_2D_render(earth.diffuse);
    vec3d pos_ecef = earth.inertial_to_fixed(satellite.state.position);
    double lat, lon, alt;
    earth.fixed_to_lla(pos_ecef, &lat, &lon, &alt);
    renderer.draw_dot(lat, lon, vec3f(1.0f, 0.0f, 0.0f), 1.0f);

    vec3d pos_kep, vel_kep;
    constant_orbit.get_state_vectors(&pos_kep, &vel_kep);
    pos_ecef = earth.inertial_to_fixed(pos_kep);
    earth.fixed_to_lla(pos_ecef, &lat, &lon, &alt);
    renderer.draw_dot(lat, lon, vec3f(1.0f, 1.0f, 0.0f), 1.0f);

    //renderer.end_2D_render();
}

void aimpoint::render3D() {
    laml::Mat3 eci_to_render(0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
    laml::Mat3 lci_to_render(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
    laml::Mat3 render_coord_frame(1.0f);
    float frame_zoom_level = 1.0f;
    switch(render_frame_enum) {
        case ECI: {
            render_coord_frame = eci_to_render;
            cam_orbit_point = vec3f(0.0f);
        } break;
        case ECEF: {
            render_coord_frame = laml::mul(eci_to_render, earth.mat_inertial_to_fixed);
            cam_orbit_point = vec3f(0.0f);
        } break;
        case ECLIPTIC: {
            mat3f ecliptic_rot;
            laml::transform::create_ZXZ_rotation(ecliptic_rot, 0.0f, 23.4362f, 0.0f);
            render_coord_frame = laml::mul(eci_to_render, ecliptic_rot);
            cam_orbit_point = vec3f(0.0f);
        } break;
        case LCI: {
            mat3f _eci2lci(eci2lci.c_11, eci2lci.c_21, eci2lci.c_31,
                           eci2lci.c_12, eci2lci.c_22, eci2lci.c_32,
                           eci2lci.c_13, eci2lci.c_23, eci2lci.c_33);
            render_coord_frame = laml::mul(lci_to_render, _eci2lci);

            vec3f lci_point = laml::transform::transform_point(eci2lci, earth.fixed_to_inertial(earth.lla_to_fixed(hmm.launch_lat,hmm.launch_lon,0.0), 0.0));
            cam_orbit_point = laml::transform::transform_point(lci_to_render, lci_point);
            frame_zoom_level = 0.25f;
        } break;
        case LCF: {
            mat3f _eci2lci(eci2lci.c_11, eci2lci.c_21, eci2lci.c_31,
                           eci2lci.c_12, eci2lci.c_22, eci2lci.c_32,
                           eci2lci.c_13, eci2lci.c_23, eci2lci.c_33);
            render_coord_frame = laml::mul(lci_to_render, laml::mul(_eci2lci, earth.mat_inertial_to_fixed));
            vec3f lci_point = laml::transform::transform_point(eci2lci, earth.fixed_to_inertial(earth.lla_to_fixed(hmm.launch_lat,hmm.launch_lon,0.0), 0.0));
            cam_orbit_point = laml::transform::transform_point(lci_to_render, lci_point);
            frame_zoom_level = 0.25f;
        } break;
    }

    laml::Vec3 cam_pos = cam_orbit_point + 
                         zoom_level*frame_zoom_level*cam_orbit_distance*laml::Vec3(
                            laml::cosd(pitch)*laml::sind(yaw), 
                           -laml::sind(pitch), 
                            laml::cosd(pitch)*laml::cosd(yaw));

    renderer.setup_frame(cam_pos, yaw, pitch, render_coord_frame);
    
    renderer.bind_texture(earth.diffuse);
    renderer.draw_mesh(earth.mesh, laml::Vec3(0.0f), laml::transform::quat_from_mat(earth.mat_fixed_to_inertial));
    
    // draw ECI frame
    renderer.draw_vector(vec3f(1.0f, 0.0f, 0.0f), 10000000, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 1.0f, 0.0f), 10000000, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 0.0f, 1.0f), 10000000, vec3f(0.1f, 0.1f, 1.0f));
    
    // draw ECEF frame
    renderer.draw_vector(laml::transform::transform_point(earth.mat_fixed_to_inertial, vec3f(1.0f, 0.0f, 0.0f)), 8000000, vec3f(1.0f, 0.3f, 0.3f));
    renderer.draw_vector(laml::transform::transform_point(earth.mat_fixed_to_inertial, vec3f(0.0f, 1.0f, 0.0f)), 8000000, vec3f(0.3f, 1.0f, 0.3f));
    //renderer.draw_vector(laml::transform::transform_point(earth.mat_fixed_to_inertial, vec3f(0.0f, 0.0f, 1.0f)), 8000000, vec3f(0.3f, 0.3f, 1.0f));
    
    // dot at launch site
    renderer.bind_texture(red_tex);
    vec3f launch_site = earth.fixed_to_inertial(earth.lla_to_fixed(hmm.launch_lat,hmm.launch_lon,0.0));
    renderer.draw_mesh(dot, launch_site, laml::Quat());
    
    // Orbit from RK4 integrator
    renderer.bind_texture(grid_tex);
    renderer.draw_mesh(dot, satellite.state.position, satellite.state.orientation);
    J2_perturbations.create_from_state_vectors(satellite.state.position, satellite.state.velocity, sim_time);
    J2_perturbations.calc_path_mesh();
    renderer.draw_path(J2_perturbations.path_handle, 100, vec3f(.3333f, 0.4588f, .5418f));
    
    // Orbit from orbit integrator
    vec3d pos_kep, vel_kep;
    constant_orbit.get_state_vectors(&pos_kep, &vel_kep);
    renderer.bind_texture(red_tex);
    renderer.draw_mesh(dot, pos_kep, satellite.state.orientation);
    renderer.draw_path(constant_orbit.path_handle,  100, vec3f(.3333f, 0.4588f, .5418f));
    
    // draw orbit/equatorial planes
    if (draw_planes) {
        renderer.draw_vector(J2_perturbations.specific_ang_momentum_unit,  8000000, vec3f(1.0f, 0.96f, 0.68f));
        //renderer.draw_vector(kep2.ascending_node_unit,  8000000, vec3f(1.0f, 1.0f, 0.6f));
    
        renderer.draw_vector(constant_orbit.specific_ang_momentum_unit, 10000000, vec3f(0.8f, 0.76f, 0.68f));
        //renderer.draw_vector(kep.ascending_node_unit,  8000000, vec3f(1.0f, 1.0f, 0.6f));
        
        renderer.draw_plane(vec3f(0.0f, 0.0f, 1.0f), 30000000, vec3f(0.8f, 0.70f, 0.80f), 0.3f);
        renderer.draw_plane(J2_perturbations.specific_ang_momentum_unit, 20000000, vec3f(1.0f, 0.96f, 0.68f), 0.7f);
    }
}

void aimpoint::renderUI() {
    // Draw UI
    const ImGuiIO& io = ImGui::GetIO();

    if (draw_ground_tracks) {
        ImGui::Begin("Ground Tracks");
        {
            // Using a Child allow to fill all the space of the window.
            // It also alows customization
            ImGui::BeginChild("GameRender");
            // Get the size of the child (i.e. the whole draw size of the windows).
            ImVec2 wsize = ImGui::GetWindowSize();
            // Because I use the texture from OpenGL, I need to invert the V from the UV.
            uint64 tmp = renderer.get_2D_output();
            ImGui::Image((ImTextureID)tmp, wsize, ImVec2(0, 1), ImVec2(1, 0));
            //ImGui::Image((ImTextureID)tmp, wsize, ImVec2(0, 1), ImVec2(1, 0));
            ImGui::EndChild();
        }
        ImGui::End();
    }

    if (show_keplerian_panel) {
        ImGui::Begin("Keplerian Elements", NULL, ImGuiWindowFlags_NoResize);

        ImGui::Text("Keplerian Orbit Propagation");
        ImGui::Text("Eccentricity: %.5f", constant_orbit.eccentricity);
        ImGui::Text("Semimajor Axis: %.1f km", constant_orbit.semimajor_axis/1000.0);
        ImGui::Text("Inclination: %.2f deg", constant_orbit.inclination);
        ImGui::Text("RAAN: %.2f deg", constant_orbit.right_ascension);
        ImGui::Text("Arg. of Periapsis: %.2f deg", constant_orbit.argument_of_periapsis);
        ImGui::Text("Mean Anomaly (Epoch): %.2f deg", constant_orbit.mean_anomaly_at_epoch);
        ImGui::Text("Period: %.3f min", constant_orbit.period / 60.0);
        ImGui::Separator();

        ImGui::Text("Num. Integration with J2 Perturbation");
        ImGui::Text("Eccentricity: %.5f", J2_perturbations.eccentricity);
        ImGui::Text("Semimajor Axis: %.1f km", J2_perturbations.semimajor_axis/1000.0);
        ImGui::Text("Inclination: %.2f deg", J2_perturbations.inclination);
        ImGui::Text("RAAN: %.2f deg", J2_perturbations.right_ascension);
        ImGui::Text("Arg. of Periapsis: %.2f deg", J2_perturbations.argument_of_periapsis);
        ImGui::Text("Mean Anomaly (Epoch): %.2f deg", J2_perturbations.mean_anomaly_at_epoch);
        ImGui::Text("Period: %.3f min", J2_perturbations.period / 60.0);
        ImGui::Separator();

        ImGui::End();

        if (show_anomoly_panel) {
            ImGui::Begin("Anomalies", NULL, ImGuiWindowFlags_NoResize);

            ImGui::Text("Mean Anomaly: %.5f deg", constant_orbit.mean_anomaly);
            ImGui::Text("Ecc  Anomaly: %.5f deg", constant_orbit.eccentric_anomaly);
            ImGui::Text("True Anomaly: %.5f deg", constant_orbit.true_anomaly);
            ImGui::Separator();

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

    //ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin("Render Frame", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    switch(render_frame_enum) {
        case ECI:
        case ECEF: {
            ImGui::Text("Earth-Centered");
        } break;
        case ECLIPTIC: {
            ImGui::Text("Earth-Ecliptic");
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
}

void aimpoint::shutdown() {
}

void aimpoint::key_callback(int key, int scancode, int action, int mods) {
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
            case ECLIPTIC: {
                render_frame_enum = ECI;
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

    if (key == GLFW_KEY_G && action == GLFW_RELEASE) {
        draw_ground_tracks = !draw_ground_tracks;
    }

    // toggle orbital param displays
    if (key == GLFW_KEY_K && action == GLFW_RELEASE) {
        show_keplerian_panel = !show_keplerian_panel;
    }
    if (key == GLFW_KEY_A && action == GLFW_RELEASE) {
        show_anomoly_panel = !show_anomoly_panel;
    }

    // orbital planes
    if (key == GLFW_KEY_P && action == GLFW_RELEASE) {
        draw_planes = !draw_planes;
    }
}

void aimpoint::mouse_pos_callback(double xpos, double ypos) {
}

void aimpoint::mouse_button_callback(int button, int action, int mods) {
}

void aimpoint::mouse_scroll_callback(double xoffset, double yoffset) {
}











// Entry point
int main(int argc, char** argv) {

    set_terminal_log_level(log_level::info);
    spdlog::info("Creating application...");

    aimpoint app;
    app.run();

    system("pause");

    return 0;
}