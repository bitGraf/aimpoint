#include "base_app.h"

#include "planet.h"
#include "round_earth_rocket_flat_approx.h"

struct round_earth_launch_flat_approx_demo : public base_app {
    round_earth_launch_flat_approx_demo() {}

    // user-override functions
    void key_callback(int key, int scancode, int action, int mods) override;
    void mouse_pos_callback(double xpos, double ypos) override;
    void mouse_button_callback(int button, int action, int mods) override;
    void mouse_scroll_callback(double xoffset, double yoffset) override;

    // user-override functions
    int init() override;
    void step(double dt) override;
    void render2D() override;
    void render3D() override;
    void renderUI() override;
    void shutdown() override;

    triangle_mesh earth_plane;
    triangle_mesh rocket_mesh;
    texture earth_diffuse;
    vec3f plane_offset;

    bool draw_flat_earth = false;

    laml::Quat_highp earth_rot;
    vec3d earth_offset;
    float f;

    rocket_round_earth_flat_ltg body;

    float t_data[1000];
    float a_data[1000];

    planet earth;
};