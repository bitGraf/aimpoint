#include "defines.h"

struct aimpoint {
    void run();

private:
    void init();
    void step(float dt);
    void render();
    void shutdown();

    float simulation_rate;
    float render_rate;

    float sim_time;
};