#include "defines.h"

#include "physics.h"

struct aimpoint {
public:
    int run();

    void key_callback(int key, int scancode, int action, int mods);

private:
    int init();
    void step(double dt);
    void render(physics_world_render_state state);
    void shutdown();

    double simulation_rate;

    double sim_time;
    double wall_time;

    int window_width, window_height;

    physics_world world;
    physics_world_state current_state;
};