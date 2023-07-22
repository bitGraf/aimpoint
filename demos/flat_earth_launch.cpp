#include "flat_earth_launch.h"

#include "log.h"

#include "implot.h"
#include <GLFW/glfw3.h>

void flat_earth_launch_demo::key_callback(int key, int scancode, int action, int mods) {
    if ((key == GLFW_KEY_P) && (action == GLFW_PRESS)) {
        //show_plots = !show_plots;
    }

    if ((key == GLFW_KEY_F) && (action == GLFW_PRESS)) {
        draw_flat_earth = !draw_flat_earth;
    }
}
void flat_earth_launch_demo::mouse_pos_callback(double xpos, double ypos) {

}
void flat_earth_launch_demo::mouse_button_callback(int button, int action, int mods) {

}
void flat_earth_launch_demo::mouse_scroll_callback(double xoffset, double yoffset) {

}

int flat_earth_launch_demo::init() {
    double map_width  = 4.0075e07;
    double map_height = 2.0038e07;
    if (!earth_plane.create_plane(vec3f(0.0f, 0.0f, -1.0f), map_width/2.0f, map_height/2.0f))
        return 4;

    earth_diffuse.load_texture_file("data/earth.jpg");
    //earth_diffuse.load_texture_file("data/map.png");
    rocket_mesh.load_from_mesh_file("data/rocket.mesh", 100.0f);

    mat4f projection_matrix;
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, renderer.get_AR(), 1000.0f, 30000000.0f);
    renderer.set_projection(projection_matrix);

    cam_orbit_distance = 100000;
    //cam_orbit_point.y = 1.0f;

    simulation_rate = 2000.0;

    yaw = 45;
    pitch = -15;

    double launch_lat = 28.3922;
    double launch_lon = -80.6077;
    plane_offset.x = -(launch_lat/180.0f) * map_height;
    plane_offset.y = -(launch_lon/360.0f) * map_width;

    earth.eccentricity_sq = 0.0;
    earth.load_mesh();
    mat3d rotM;
    laml::transform::create_ZXZ_rotation(rotM, -90.0, -90.0f - launch_lat, 90 - launch_lon);
    earth_rot = laml::transform::quat_from_mat(rotM);

    body.launch(&earth, 180000.0, 300.0);

    f = 7000.0f;
    //a = -90.0f;
    //b = -90.0f - launch_lat;
    //c = 90 - launch_lon;
    return 0;
}
void flat_earth_launch_demo::step(double dt) {
    body.integrate_states(sim_time, dt);
}
void flat_earth_launch_demo::render2D() {

}
void flat_earth_launch_demo::render3D() {
    mat3f NEDKM2Render(0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f);
    cam_orbit_point = laml::transform::transform_point(NEDKM2Render, vec3f(body.state.position));
    laml::Vec3 cam_pos = cam_orbit_point + 
                         zoom_level*cam_orbit_distance*laml::Vec3(
                         laml::cosd(pitch)*laml::sind(yaw), 
                        -laml::sind(pitch), 
                         laml::cosd(pitch)*laml::cosd(yaw));

    float scale_km = 1.0f;
    earth_offset = vec3d(0.0, 0.0, (earth.equatorial_radius - f)*scale_km);
    renderer.setup_frame(cam_pos, yaw, pitch, NEDKM2Render, scale_km);

    if (draw_flat_earth) {
        renderer.bind_texture(earth_diffuse);
        renderer.draw_mesh(earth_plane, plane_offset);
    } else {
        renderer.bind_texture(earth.diffuse);
        renderer.draw_mesh(earth.mesh, earth_offset, earth_rot);
    }

    renderer.bind_texture(blank_tex);
    renderer.draw_mesh(rocket_mesh, body.state.position, body.state.orientation);

    // draw inertial frame
    double axis_size = 500000;
    renderer.draw_vector(vec3f(1.0f, 0.0f, 0.0f), axis_size, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 1.0f, 0.0f), axis_size, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 0.0f, 1.0f), axis_size, vec3f(0.1f, 0.1f, 1.0f));

    // draw ground plane
    //renderer.draw_plane(vec3f(0.0f, 1.0f, 0.0f), 1.0f, vec3f(0.8f, 0.70f, 0.80f), 0.3f);
}
void flat_earth_launch_demo::renderUI() {
    ImGui::Begin("Rocket");
    //ImGui::Text("Pos NED: [%.1f, %.1f, %.1f] km", body.state.position.x/1000.0, body.state.position.y/1000.0, body.state.position.z/1000.0);
    //ImGui::Text("Vel NED: [%.1f, %.1f, %.1f] m/s", body.state.velocity.x, body.state.velocity.y, body.state.velocity.z);
    ImGui::Text("State\t\t\t\t\tTarget");
    ImGui::Text("height:  %6.2f km\t  %6.2f km", -body.state.position.z/1000.0, body.yf/1000.0);
    ImGui::Text("x-vel:  %7.2f m/s\t%7.2f m/s", body.state.velocity.y, body.vxf);
    ImGui::Text("y-vel:  %7.2f m/s\t%7.2f m/s",-body.state.velocity.z, body.vyf);
    ImGui::Separator();

    if (body.tof < body.T) {
        float dt = body.T / 999.0;
        float t0 = sim_time;
        for (int n = 0; n < 1000; n++) {
            t_data[n] = n*dt;

            float A_ = body.A - body.B*t0;
            float B_ = body.B;
            a_data[n] = laml::atand(A_ + B_*t_data[n]);
        }
    }

    if (ImPlot::BeginPlot("Steering Law", ImVec2(-1,0), ImPlotFlags_NoLegend)) {
        ImPlot::SetupAxisLimits(ImAxis_X1, 0, body.T, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 90);
        ImPlot::PlotLine("law", t_data, a_data, 1000);
        ImPlot::PlotScatter("current point", &sim_time, &body.alpha, 1);
        ImPlot::EndPlot();
    }

    ImGui::End();
}
void flat_earth_launch_demo::shutdown() {

}


int main() {
    set_terminal_log_level(log_level::info);
    spdlog::info("Spinning Rigid Body demo...");

    flat_earth_launch_demo app;
    app.run(1280, 720);

    return 0;
}