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

    const double step_time = 1.0 / simulation_rate;
    const double render_time = 1.0 / render_rate;

    double time_to_next_step = 0.0;
    double time_to_next_render = 0.0;

    int iteration = 0;

    wall_time = glfwGetTime();

    bool done = false;
    while(!glfwWindowShouldClose(window)) {
        glfwSwapBuffers(window);
        glfwPollEvents();

        double time = glfwGetTime();

        while (time_to_next_step <= 0.0) {
            step(step_time);
            time_to_next_step += step_time;
        }

        if (time_to_next_render <= 0.0) {
            render();
            time_to_next_render += render_time;
            iteration++;
        }

        double wait_time = time_to_next_step < time_to_next_render ? time_to_next_step : time_to_next_render;
        int64 wait_time_nano = static_cast<int64>(wait_time*1000000000);
        //std::this_thread::sleep_for(std::chrono::nanoseconds(wait_time_nano));
        //spdlog::trace("[{0:.3f}] Waiting {1:.3f} seconds", sim_time, wait_time);
        time_to_next_step -= wait_time;
        time_to_next_render -= wait_time;

        if (iteration >= 3) {
            done = true;
        }
    }

    shutdown();

    return 0;
}

int aimpoint::init() {
    simulation_rate = 100.0;
    render_rate = 60.0;

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

    spdlog::info("Application intitialized");
    return 0;
}

void aimpoint::step(float dt) {
    sim_time += dt;
    spdlog::trace("[{0:0.3f}] simulation step", sim_time);
}

void aimpoint::render() {
    glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    set_terminal_log_level(log_level::trace);
    spdlog::info("Creating application...");

    app.run();
	
	return 0;
}