#include "base_app.h"

struct demo_app : public base_app {
    demo_app() {}

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

    simulation_body body;
    triangle_mesh mesh;
    texture tex;

    bool inertial_frame;
    bool show_plots;
    plot_signal<double, 5000> w_t;
    plot_signal<double, 5000> w_x;
    plot_signal<double, 5000> w_y;
    plot_signal<double, 5000> w_z;
};