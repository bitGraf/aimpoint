#pragma once
#include "defines.h"

#include "render/renderer.h"
#include "render/mesh.h"

#include "body_type/mass_spring_damper.h"
#include "body_type/t_bar.h"

struct aimpoint {
public:
    int run();

    void key_callback(int key, int scancode, int action, int mods);

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

    opengl_renderer renderer;
    triangle_mesh mesh;

    // openGL handles
    uint32 shader;

    mass_spring_damper body;
    t_bar body2;

    laml::Vec3 cam_pos;
    float yaw, pitch;
};