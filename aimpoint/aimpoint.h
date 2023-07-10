#pragma once
#include "defines.h"
#include "base_app.h"

#include "body_type/satellite.h"

#include "planet.h"
#include "orbit.h"

const size_t num_seconds_history = 5;
const size_t buffer_length = num_seconds_history * 60;
const size_t orbit_buffer_length = (200) * 60;
const size_t frames_per_min = 60*60;

enum coordinate_frame : int {
    ECI = 0,
    ECEF = 1,
    LCI = 2,
    LCF = 3,
    ECLIPTIC = 4,
};

struct aimpoint : public base_app {
public:
    aimpoint() : constant_orbit(earth), J2_perturbations(earth) {}

    void key_callback(int key, int scancode, int action, int mods) override;
    void mouse_pos_callback(double xpos, double ypos) override;
    void mouse_button_callback(int button, int action, int mods) override;
    void mouse_scroll_callback(double xoffset, double yoffset) override;

private:
    int init() override;
    void step(double dt) override;
    void render2D() override;
    void render3D() override;
    void renderUI() override;
    void shutdown() override;

    bool show_keplerian_panel = false;
    bool show_anomoly_panel = false;
    bool draw_planes = false;
    bool draw_ground_tracks = false;

    triangle_mesh mesh, dot;
    texture grid_tex, red_tex, green_tex, blue_tex;
    coordinate_frame render_frame_enum = ECI;

    double launch_lat, launch_lon, launch_az;

    planet earth;
    orbit constant_orbit, J2_perturbations;
    satellite_body satellite;

    mat3d lci2eci, eci2lci;

    // plotting
    plot_signal<double, orbit_buffer_length> t;
    plot_signal<double, orbit_buffer_length> M;
    plot_signal<double, orbit_buffer_length> E;
    plot_signal<double, orbit_buffer_length> v;

    plot_signal<double, buffer_length> sim_scale_history;
};