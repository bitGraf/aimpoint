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
                sim_time += step_time;
                accum_time -= step_time;

                //if (sim_time >= 250.0) { 
                //    done = true;
                //    break; 
                //}
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

    body.set_mass(1.0);
    body.set_inertia(0.2, 0.3, 0.4);
    body.state.ang_velocity.x = 0.5;
    body.state.ang_velocity.y = 10.0;
    body.state.ang_velocity.z = 0.5;
    body.calc_energy();
    body.truth_total_energy = body.linear_KE + body.rotational_KE;

    cam_orbit_point = laml::Vec3(0.0f, 0.0f, 0.0f);
    cam_orbit_distance = 2.0f;
    yaw = 0;
    pitch = 0;

    renderer.init_gl_glfw(this, 1280, 720);

    // Load mesh from file
    //mesh.load_from_mesh_file("../data/Cylinder.mesh");
    mesh.load_from_mesh_file("../data/blahaj.mesh");

    spdlog::info("Application intitialized");

    return 0;
}

void aimpoint::step(double dt) {
    //spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    int64 cycles_per_second = (int64)simulation_rate;
    if (sim_frame % (cycles_per_second) == 0) {
        spdlog::info("[{0:0.3f}] simulation step", sim_time);
    }

    body.integrate_states(sim_time, dt);
    
    sim_frame++;
}

void aimpoint::render() {
    if (input.mouse2) {
        yaw   -= input.xvel * frame_time * 0.75f;
        pitch -= input.yvel * frame_time * 0.50f;
    }

    laml::Vec3 cam_pos = cam_orbit_point + cam_orbit_distance*laml::Vec3(laml::cosd(pitch)*laml::sind(yaw), -laml::sind(pitch), laml::cosd(pitch)*laml::cosd(yaw));
    renderer.start_frame(cam_pos, yaw, pitch);

    renderer.draw_mesh(mesh);

    {
        laml::Vec3 render_pos(body.state.position);
        laml::Quat render_rot(body.state.orientation);
        renderer.draw_mesh(mesh, render_pos, render_rot);
    }

    // Draw UI
    renderer.start_debug_UI();

    const ImGuiIO& io = ImGui::GetIO();
    if (show_info_panel) {
        ImGui::Begin("Spinning T-Handle");

        ImGui::TextWrapped("Demonstrates the proper integration of Euler's Equations of Motion using body-fixed anguar velocites.");

        ImGui::Separator();
        ImGui::Text("System Energy:");
        ImGui::Text("    Linear KE = %.3f J", body.linear_KE);
        ImGui::Text("Rotational KE = %.3f J", body.rotational_KE);
        ImGui::Separator();
        ImGui::Text("Total KE         = %.3f J", body.linear_KE + body.rotational_KE);
        ImGui::Text("Total KE (truth) = %.3f J", body.truth_total_energy);
        ImGui::Text("Energy Loss      = %.3e J", body.truth_total_energy - (body.linear_KE + body.rotational_KE));
        ImGui::Separator();

        ImGui::Text("Body Rates:");
        ImGui::Text("%6.3f %6.3f %6.3f", body.state.ang_velocity.x, body.state.ang_velocity.y, body.state.ang_velocity.z);
        ImGui::Separator();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::Text("xpos=%.1f   ypos=%.1f", input.xpos, input.ypos);
        ImGui::Text("xvel=%.1f   yvel=%.1f", input.xvel, input.yvel);
        ImGui::Separator();

        ImGui::Text("Mouse Buttons:");
        ImGui::Text("M1:%s    M2:%s", input.mouse1 ? "Pressed" : "Released", input.mouse2 ? "Pressed" : "Released");
        ImGui::Separator();

        ImGui::Text("Camera State:");
        ImGui::Text("Yaw:%.2f    Pitch:%.2f", yaw, pitch);
        ImGui::Separator();

        ImGui::End();
    }

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

    renderer.end_debug_UI();

    renderer.end_frame();

    //spdlog::trace("[{0:0.3f}] ({1:0.3f}) render step", sim_time, alpha);
    //spdlog::trace("[{0:0.3f}] render step", sim_time);
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

    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        yaw = 0.0;
        pitch = 0.0;
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