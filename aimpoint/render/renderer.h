#pragma once
#include "defines.h"

#include "mesh.h"
#include "texture.h"

// for recording
#if USE_DTV
#include "dtv.h"
#endif

struct aimpoint;
struct opengl_renderer {
    opengl_renderer();

    int32 init_gl_glfw(aimpoint* app_ptr, int32 width, int32 height);
    void shutdown();

    void poll_events();
    bool should_window_close();
    double get_time();

    void start_frame(const laml::Vec3& cam_pos, float cam_yaw, float cam_pitch,
                     const laml::Mat3& render_frame = laml::Mat3());
    
    void bind_texture(const texture& tex);
    void draw_mesh(const triangle_mesh& mesh,
                   const laml::Vec3& position = laml::Vec3(),
                   const laml::Quat& orientation = laml::Quat());

    void start_debug_UI();
    void end_debug_UI();

    void end_frame();

private:
    int32 window_width, window_height;

    void* raw_glfw_window;

    uint32 shader;

    mat3f render_frame;

    // video recording
#if USE_DTV
    atg_dtv::Encoder encoder;

    uint8 *tmp_buffer = nullptr;
#endif
    bool init_recording();
    bool stop_recording();
};