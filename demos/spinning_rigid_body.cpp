#include "spinning_rigid_body.h"

#include "log.h"

#include "implot.h"
#include <GLFW/glfw3.h>

void spinning_rigid_body_demo::key_callback(int key, int scancode, int action, int mods) {
    if ((key == GLFW_KEY_P) && (action == GLFW_PRESS)) {
        show_plots = !show_plots;
    }

    if ((key == GLFW_KEY_F) && (action == GLFW_PRESS)) {
        inertial_frame = !inertial_frame;
    }
}
void spinning_rigid_body_demo::mouse_pos_callback(double xpos, double ypos) {

}
void spinning_rigid_body_demo::mouse_button_callback(int button, int action, int mods) {

}
void spinning_rigid_body_demo::mouse_scroll_callback(double xoffset, double yoffset) {

}

int spinning_rigid_body_demo::init() {
    body.set_mass(1.0);
    body.set_inertia(0.2, 0.3, 0.4);

    body.set_state(vec3d(0.0, 0.0, 0.0), vec3d(),
                   laml::Quat_highp(),
                   vec3d(0.1, 10.0, 0.1));

    if (!mesh.load_from_mesh_file("data/t_bar.mesh", 0.5f)) 
        return 4;

    tex.load_texture_file("data/blue.png");

    mat4f projection_matrix;
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, renderer.get_AR(), 0.1f, 10.0f);
    renderer.set_projection(projection_matrix);

    cam_orbit_distance = 2.0f;
    //cam_orbit_point.y = 1.0f;

    simulation_rate = 200.0;

    yaw = 45;
    pitch = -15;
    zoom_level = 1.0;

    show_plots = false;
    inertial_frame = true;

    return 0;
}
void spinning_rigid_body_demo::step(double dt) {
    body.integrate_states(sim_time, dt);

    if ((sim_time + dt) > 6.28) done = true;
}
void spinning_rigid_body_demo::render2D() {

}
void spinning_rigid_body_demo::render3D() {
    laml::Vec3 cam_pos = cam_orbit_point + 
                         zoom_level*cam_orbit_distance*laml::Vec3(
                         laml::cosd(pitch)*laml::sind(yaw), 
                        -laml::sind(pitch), 
                         laml::cosd(pitch)*laml::cosd(yaw));
    mat3f rot;
    laml::transform::create_transform_rotation(rot, laml::Quat(body.state.orientation));
    vec3f w = body.state.ang_velocity;
    if (!inertial_frame) {
        renderer.setup_frame(cam_pos, yaw, pitch, laml::inverse(rot));
    } else {
        w = laml::transform::transform_point(laml::inverse(rot), w);
    }

    renderer.bind_texture(tex);
    vec3d off(0.0, 0.0, 0.0);
    renderer.draw_mesh(mesh, body.state.position + off, body.state.orientation);

    // draw inertial frame
    renderer.draw_vector(vec3f(1.0f, 0.0f, 0.0f), 1.0f, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 1.0f, 0.0f), 1.0f, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 0.0f, 1.0f), 1.0f, vec3f(0.1f, 0.1f, 1.0f));

    renderer.draw_vector(w, laml::length(w)/10.0f, vec3f(1.0f, 0.1f, 1.0f));

    renderer.draw_vector(rot._cols[0], 1.0f, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(rot._cols[1], 1.0f, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(rot._cols[2], 1.0f, vec3f(0.1f, 0.1f, 1.0f));

    // draw ground plane
    renderer.draw_plane(vec3f(0.0f, 1.0f, 0.0f), 1.0f, vec3f(0.8f, 0.70f, 0.80f), 0.3f);
}
void spinning_rigid_body_demo::renderUI() {
    if (show_plots) {
        // Plots
        w_t.add_point(sim_time);
        w_x.add_point(body.state.ang_velocity.x);
        w_y.add_point(body.state.ang_velocity.y);
        w_z.add_point(body.state.ang_velocity.z);

        ImGui::Begin("Body Rates");
        double history = 5.0; //w_t.length * (1.0 / 60.0);
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
    }
}
void spinning_rigid_body_demo::shutdown() {

}


int main() {
    set_terminal_log_level(log_level::info);
    spdlog::info("Spinning Rigid Body demo...");

    spinning_rigid_body_demo app;
    app.run(1280, 720);

    return 0;
}