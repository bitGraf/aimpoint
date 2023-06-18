#include "aimpoint.h"

#include "log.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <thread>
#include <chrono>

// global vars for GLFW
static GLFWwindow* window = nullptr;
void glfw_error_callback(int error, const char* description);
void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

int aimpoint::run() {
    if (init()) {
        // failed on initialization
        return 1;
    }

    sim_time = 0.0;
    const double step_time = 1.0 / simulation_rate;

    wall_time = glfwGetTime();
    double accum_time = 0.0;

    bool done = false;
    while(!done) {
        double new_time = glfwGetTime();
        double frame_time = new_time - wall_time;
        wall_time = new_time;

        accum_time += frame_time;

        glfwPollEvents();
        if (glfwWindowShouldClose(window) || sim_time >= 150.0) {
            done = true;
        }

        if (real_time) {
            while (accum_time >= step_time) {
                step(step_time);
                sim_time += step_time;
                accum_time -= step_time;
            }
        } else {
            double render_time = wall_time + 1.0/65.0;
            while (glfwGetTime() < render_time) {
                step(step_time);
                sim_time += step_time;
            }
        }

        double alpha = accum_time / step_time; // interpolation value [0,1]
        render();
        glfwSwapBuffers(window);
    }

    shutdown();

    return 0;
}

int aimpoint::init() {
    simulation_rate = 1000.0; // Hz
    sim_frame = 0;
    real_time = false;

    sim_time = 0.0;
    wall_time = 0.0;

    window_width = 640;
    window_height = 480;

    // setup glfw
    if (!glfwInit()) {
        // init failed
        spdlog::critical("Failed to initialize GLFW");
        return 1;
    }
    glfwSetErrorCallback(glfw_error_callback);

    // create window and OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    window = glfwCreateWindow(window_width, window_height, "Aimpoint", NULL, NULL);
    if (!window) {
        // Window or OpenGL context creation failed
        spdlog::critical("Failed to create GLFW Window");
        return 2;
    }

    glfwSetKeyCallback(window, glfw_key_callback);

    glfwSetWindowUserPointer(window, this);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    glfwSwapInterval(2);

    spdlog::info("Application intitialized");
    return 0;
}

void aimpoint::step(double dt) {
    spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    int64 cycles_per_second = (int64)simulation_rate;
    if (sim_frame % (cycles_per_second) == 0) {
        spdlog::info("t(s) = {0:4.1f}     alt(km) = {1:7.3f}     speed(m/s) = {2:7.3f}", 
                     sim_time, laml::length(body.state.position)/1000.0, laml::length(body.state.velocity));
    }
    body.major_step(dt);
    body.integrate_states(sim_time, dt);
    sim_frame++;
}

void aimpoint::render() {
    glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //spdlog::trace("[{0:0.3f}] ({1:0.3f}) render step", sim_time, alpha);
    spdlog::trace("[{0:0.3f}] render step", sim_time);
}

void aimpoint::shutdown() {
    glfwDestroyWindow(window);

    glfwTerminate();

    spdlog::info("Application shutdown");
}

void aimpoint::key_callback(int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}



// GLFW Callbacks
void glfw_error_callback(int error, const char* description) {
    spdlog::error("[GLFW] Error {0}: {1}", error, description);
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->key_callback(key, scancode, action, mods);
}












// Entry point
int main(int argc, char** argv) {
    aimpoint app;

    set_terminal_log_level(log_level::debug);
    spdlog::info("Creating application...");

    app.run();

    system("pause");
	
	return 0;
}