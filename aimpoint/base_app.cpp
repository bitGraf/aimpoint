#include "base_app.h"

#include "log.h"

#include <thread>
#include <chrono>

#include <stdio.h>

#include "imgui.h"
#include "implot.h"

// TMP for KEY_CODES!
#include <GLFW/glfw3.h>

static char* pretty_time(double time, double* value) {
    double abs_time = laml::abs(time);
    double sign = laml::sign(time);

    if (abs_time < 2.0*60.0) {
        *value = abs_time * sign;
        return "s";
    }

    if (abs_time < 2.0*60.0*60.0) {
        *value = abs_time * sign / 60.0;
        return "min";
    }

    if (abs_time < 2*60.0*60.0*24.0) {
        *value = abs_time * sign / (60.0*60.0);
        return "hours";
    }

    *value = abs_time * sign / (60.0*60.0*24.0);
    return "days";
}

int base_app::run(int32 window_width, int32 window_heigt) {
    if (base_init(window_width, window_heigt)) {
        // failed on initialization
        return 1;
    }

    sim_time = 0.0;
    const double step_time = 1.0 / simulation_rate;

    wall_time = renderer.get_time();
    double accum_time = 0.0;
    frame_time = 0.0;

    bool done = false;
    while(!done) {
        double new_time = renderer.get_time();
        frame_time = new_time - wall_time;
        wall_time = new_time;

        accum_time += frame_time;

        input.xvel = 0.0;
        input.yvel = 0.0;
        renderer.poll_events();
        if (renderer.should_window_close()) {
            done = true;
        }
        input.xvel /= frame_time;
        input.yvel /= frame_time;

        if (real_time) {
            while (accum_time >= step_time) {
                base_step(step_time);
                accum_time -= step_time;

                //if (sim_time >= 10.0) { 
                //    done = true;
                //    break; 
                //}
            }
        } else {
            double render_time = wall_time + 1.0/65.0;
            while (renderer.get_time() < render_time) {
                base_step(step_time);
            }
        }

        base_render();
    }

    base_shutdown();

    return 0;
}

int base_app::base_init(int32 window_width, int32 window_heigt) {
    simulation_rate = 10.0; // Hz
    sim_frame = 0;
    render_frame = 0;
    real_time = true;
    log_zoom_level = 50;
    zoom_level = log(((float)log_zoom_level)/10.0f);

    sim_time = 0.0;
    wall_time = 0.0;

    cam_orbit_point = laml::Vec3(0.0f, 0.0f, 0.0f);
    cam_orbit_distance = 1.0f;
    yaw = 0.0;
    pitch = 0.0;

    renderer.init_gl_glfw(this, window_width, window_heigt);

    blank_tex.load_texture_file("data/blank.png");

    spdlog::info("Application intitialized");

    return init();
}

void base_app::base_step(double dt) {
    //spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    step(dt);
    
    sim_time += dt;
    sim_frame++;
}

void base_app::base_render() {
    if (input.mouse2) {
        yaw   -= input.xvel * frame_time * 0.75f;
        pitch -= input.yvel * frame_time * 0.50f;
    }

    if (pitch >  89.0f) pitch =  89.0f;
    if (pitch < -89.0f) pitch = -89.0f;
    if (yaw > 360.0f) yaw -= 360.0f;
    if (yaw <   0.0f) yaw += 360.0f;

    renderer.start_2D_render(blank_tex);
    render2D();
    renderer.end_2D_render();

    laml::Vec3 cam_pos = cam_orbit_point + 
                         zoom_level*cam_orbit_distance*laml::Vec3(
                         laml::cosd(pitch)*laml::sind(yaw), 
                        -laml::sind(pitch), 
                         laml::cosd(pitch)*laml::cosd(yaw));

    renderer.clear_screen();
    renderer.setup_frame(cam_pos, yaw, pitch, mat3f(1.0f));
    renderer.bind_texture(blank_tex);

    render3D();
    
    // draw ECI frame
    //renderer.draw_vector(vec3f(1.0f, 0.0f, 0.0f), 10000000, vec3f(1.0f, 0.1f, 0.1f));
    //renderer.draw_vector(vec3f(0.0f, 1.0f, 0.0f), 10000000, vec3f(0.1f, 1.0f, 0.1f));
    //renderer.draw_vector(vec3f(0.0f, 0.0f, 1.0f), 10000000, vec3f(0.1f, 0.1f, 1.0f));
    
    // draw orbit/equatorial planes
    //renderer.draw_plane(vec3f(0.0f, 0.0f, 1.0f), 30000000, vec3f(0.8f, 0.70f, 0.80f), 0.3f);

    // Draw UI
    renderer.start_debug_UI();
    const ImGuiIO& io = ImGui::GetIO();
    renderUI();

    if (show_info_panel) {
        ImGui::Begin("App Info", NULL, ImGuiWindowFlags_NoResize);

        ImGui::Text("Avg. %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
        ImGui::Text("Timestep: %.3f s (%.1f Hz)", (1.0/simulation_rate), simulation_rate);
        ImGui::Separator();

        ImGui::Text("Camera State:");
        ImGui::Text("Yaw:%.2f    Pitch:%.2f", yaw, pitch);
        //ImGui::Text("[%.1f, %.1f, %.1f]", cam_pos.x, cam_pos.y, cam_pos.z);

        ImGui::End();
    }

    ImGui::Begin("Zoom", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    //ImGui::Begin("Zoom", NULL, ImGuiWindowFlags_NoTitleBar);
    ImGui::Text("Zoom: %.1f", zoom_level);
    //ImGui::Text("LogZoom: %d", log_zoom_level);
    ImGui::End();

    ImGui::Begin("Clock", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
    //ImGui::Begin("Clock", NULL, ImGuiWindowFlags_NoTitleBar);
    double value = 0.0f;
    char* unit = pretty_time(sim_time, &value);
    ImGui::Text("T: %.1f %s", value, unit);
    ImGui::End();

    renderer.end_debug_UI();

    renderer.end_frame();

    render_frame++;
}

void base_app::base_shutdown() {
    shutdown();

    renderer.shutdown();

    spdlog::info("Application shutdown");
}




void base_app::base_key_callback(int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_I && action == GLFW_PRESS) {
        show_info_panel = !show_info_panel;
    }

    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        real_time = !real_time;
    }

    key_callback(key, scancode, action, mods);
}

void base_app::base_mouse_pos_callback(double xpos, double ypos) {
    input.xvel += (xpos - input.xpos);
    input.yvel += (ypos - input.ypos);

    input.xpos = xpos;
    input.ypos = ypos;

    mouse_pos_callback(xpos, ypos);
}

void base_app::base_mouse_button_callback(int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
        input.mouse1 = true;
    }
    if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_RELEASE) {
        input.mouse1 = false;
    }

    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_PRESS) {
        input.mouse2 = true;
    }
    if (button == GLFW_MOUSE_BUTTON_2 && action == GLFW_RELEASE) {
        input.mouse2 = false;
    }

    mouse_button_callback(button, action, mods);
}

void base_app::base_mouse_scroll_callback(double xoffset, double yoffset) {
    int32 yoff = (yoffset > 0.0) ? -1 : ((yoffset < 0.0) ? 1 : 0);
    int32 rate = 1;

    if ((log_zoom_level == 30 && yoff > 0) || log_zoom_level >= 33) rate = 3;
    if ((log_zoom_level == 45 && yoff > 0) || log_zoom_level >= 50) rate = 5;
    
    log_zoom_level += yoff*rate; // negative to swap dir

    if (log_zoom_level > 100) log_zoom_level = 100;
    if (log_zoom_level <  17) log_zoom_level =  17;

    zoom_level = log(((float)log_zoom_level)/10.0f);

    mouse_scroll_callback(xoffset, yoffset);
}