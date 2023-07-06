#pragma once
#include "defines.h"

#include "render/renderer.h"
#include "physics.h"

struct base_app {
    int run(int32 window_width = 1280, int32 window_heigt = 720);

    // base functions
    void base_key_callback(int key, int scancode, int action, int mods);
    void base_mouse_pos_callback(double xpos, double ypos);
    void base_mouse_button_callback(int button, int action, int mods);
    void base_mouse_scroll_callback(double xoffset, double yoffset);

    // user-override functions
    virtual void key_callback(int key, int scancode, int action, int mods) {};
    virtual void mouse_pos_callback(double xpos, double ypos) {};
    virtual void mouse_button_callback(int button, int action, int mods) {};
    virtual void mouse_scroll_callback(double xoffset, double yoffset) {};

protected:
    // base functions
    int  base_init(int32 window_width, int32 window_heigt);
    void base_step(double dt);
    void base_render();
    void base_shutdown();

    // user-override functions
    virtual int init() { return 0; };
    virtual void step(double dt) {};
    virtual void render2D() {};
    virtual void render3D() {};
    virtual void renderUI() {};
    virtual void shutdown() {};

    bool real_time;
    bool done;

    double simulation_rate;
    uint64 sim_frame;
    uint64 render_frame;

    double sim_time;
    double wall_time;
    double frame_time;

    float zoom_level = 1.0f;
    int32 log_zoom_level;

    bool show_info_panel = false;

    opengl_renderer renderer;
    texture blank_tex;

    float cam_orbit_distance;
    laml::Vec3 cam_orbit_point;
    float yaw;   // [  0, 360]
    float pitch; // [-89,  89]

    struct {
        double xpos = 0; 
        double ypos = 0;

        double xvel = 0; 
        double yvel = 0;

        bool mouse1 = false; 
        bool mouse2 = false;
    } input;
};