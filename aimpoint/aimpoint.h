#include "defines.h"

#include "physics.h"

struct aimpoint {
public:
    int run();

    void key_callback(int key, int scancode, int action, int mods);

private:
    int init();
    void step(double dt);
    void render();
    void shutdown();

    double simulation_rate;

    double sim_time;
    double wall_time;

    int window_width, window_height;

    simulation_body body;
};