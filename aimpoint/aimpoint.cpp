#include "aimpoint.h"

#include "log.h"

void aimpoint::run() {
    init();

    const float step_time = 1.0f / simulation_rate;
    const float render_time = 1.0f / render_rate;

    float time_to_next_step = 0.0f;
    float time_to_next_render = 0.0f;

    int iteration = 0;

    bool done = false;
    while(!done) {
        while (time_to_next_step <= 0.0f) {
            step(step_time);
            time_to_next_step += step_time;
        }

        if (time_to_next_render <= 0.0f) {
            render();
            time_to_next_render += render_time;
            iteration++;
        }

        float wait_time = time_to_next_step < time_to_next_render ? time_to_next_step : time_to_next_render;
        //spdlog::trace("[{0:.3f}] Waiting {1:.3f} seconds", sim_time, wait_time);
        time_to_next_step -= wait_time;
        time_to_next_render -= wait_time;

        if (iteration >= 6) {
            done = true;
        }
    }

    shutdown();
}

void aimpoint::init() {
    simulation_rate = 120.0f;
    render_rate = 30.0f;

    sim_time = 0.0f;

    spdlog::info("Application intitialized");
}

void aimpoint::step(float dt) {
    sim_time += dt;
    spdlog::trace("[{0:0.3f}] simulation step", sim_time);
}

void aimpoint::render() {
    spdlog::trace("[{0:0.3f}] render step", sim_time);
}

void aimpoint::shutdown() {
    spdlog::info("Application shutdown");
}














// Entry point
#include <cstdlib>
int main(int argc, char** argv) {
    aimpoint app;

    set_terminal_log_level(log_level::trace);
    spdlog::info("Starting up application...");

    app.run();

    system("pause");
	
	return 0;
}