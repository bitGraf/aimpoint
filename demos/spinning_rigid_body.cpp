#include "spinning_rigid_body.h"

#include "log.h"


void demo_app::key_callback(int key, int scancode, int action, int mods) {

}
void demo_app::mouse_pos_callback(double xpos, double ypos) {

}
void demo_app::mouse_button_callback(int button, int action, int mods) {

}
void demo_app::mouse_scroll_callback(double xoffset, double yoffset) {

}

int demo_app::init() {
    body.set_mass(1.0);
    body.set_inertia(0.2, 0.3, 0.4);

    body.set_state(vec3d(0.0, 0.0, 0.0), vec3d(),
                   laml::Quat_highp(),
                   vec3d(0.1, 5.0, 0.1));

    if (!mesh.load_from_mesh_file("data/blahaj.mesh", 0.01f)) 
        return 4;

    tex.load_texture_file("data/blue.png");

    mat4f projection_matrix;
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, renderer.get_AR(), 0.1f, 10.0f);
    renderer.set_projection(projection_matrix);

    cam_orbit_distance = 2.0f;
    //cam_orbit_point.y = 1.0f;

    simulation_rate = 200.0;

    return 0;
}
void demo_app::step(double dt) {
    body.integrate_states(sim_time, dt);
}
void demo_app::render2D() {

}
void demo_app::render3D() {
    renderer.bind_texture(tex);
    vec3d off(0.0, 0.0, 0.0);
    renderer.draw_mesh(mesh, body.state.position + off, body.state.orientation);

    // draw inertial frame
    renderer.draw_vector(vec3f(1.0f, 0.0f, 0.0f), 1.0f, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 1.0f, 0.0f), 1.0f, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(vec3f(0.0f, 0.0f, 1.0f), 1.0f, vec3f(0.1f, 0.1f, 1.0f));

    mat3d rot;
    laml::transform::create_transform_rotation(rot, body.state.orientation);
    renderer.draw_vector(rot._cols[0], 1.0f, vec3f(1.0f, 0.1f, 0.1f));
    renderer.draw_vector(rot._cols[1], 1.0f, vec3f(0.1f, 1.0f, 0.1f));
    renderer.draw_vector(rot._cols[2], 1.0f, vec3f(0.1f, 0.1f, 1.0f));

    // draw ground plane
    renderer.draw_plane(vec3f(0.0f, 1.0f, 0.0f), 1.0f, vec3f(0.8f, 0.70f, 0.80f), 0.3f);
}
void demo_app::renderUI() {

}
void demo_app::shutdown() {

}


int main() {
    set_terminal_log_level(log_level::info);
    spdlog::info("Spinning Rigid Body demo...");

    demo_app app;
    app.run(1280, 720);

    return 0;
}