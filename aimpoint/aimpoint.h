#pragma once
#include "defines.h"

#include "render/renderer.h"
#include "render/mesh.h"
#include "physics.h"
#include "body_type/rocket.h"

#include "planet.h"

const size_t num_seconds_history = 5;
const size_t buffer_length = num_seconds_history * 60;

template<typename T, size_t num_points>
struct plot_signal {
    plot_signal() : length(num_points) {
        for (size_t n = 0; n < num_points; n++) {
            data[n] = static_cast<T>(0);
        }
    }

    void add_point(T value) {
        data[offset] = value;

        offset++;

        if (offset == num_points)
            offset = 0;
    }

    T get_avg() {
        T sum = 0;
        for (size_t n = 0; n < num_points; n++) {
            sum = sum + data[n];
        }
        return sum / static_cast<T>(num_points);
    }

    T data[num_points];

    size_t offset = 0;
    const size_t length;
};

enum coordinate_frame : int {
    ECI = 0,
    ECEF = 1,
    LCI = 2,
    LCF = 3,
};

struct aimpoint {
public:
    int run();

    void key_callback(int key, int scancode, int action, int mods);
    void mouse_pos_callback(double xpos, double ypos);
    void mouse_button_callback(int button, int action, int mods);

private:
    int init();
    void step(double dt);
    void render();
    void shutdown();

    bool real_time;

    double simulation_rate;
    uint64 sim_frame;

    double sim_time;
    double wall_time;
    double frame_time;

    bool show_info_panel = false;

    opengl_renderer renderer;
    triangle_mesh mesh, dot;
    texture grid_tex, red_tex, green_tex, blue_tex;
    coordinate_frame render_frame_enum = ECI;

    planet earth;
    rocket body;

    float cam_orbit_distance;
    laml::Vec3 cam_orbit_point;
    float yaw, pitch;
    mat3d lci2eci, eci2lci;

    struct {
        double xpos = 0; 
        double ypos = 0;

        double xvel = 0; 
        double yvel = 0;

        bool mouse1 = false; 
        bool mouse2 = false;
    } input;

    // plotting
    plot_signal<double, buffer_length> w_t;
    plot_signal<double, buffer_length> w_x;
    plot_signal<double, buffer_length> w_y;
    plot_signal<double, buffer_length> w_z;

    plot_signal<double, buffer_length> sim_scale_history;
};