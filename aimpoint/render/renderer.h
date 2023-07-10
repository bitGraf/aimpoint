#pragma once
#include "defines.h"

#include "mesh.h"
#include "texture.h"
#include "shader_program.h"

// for recording
#if USE_DTV
#include "dtv.h"
#endif

struct base_app;
struct opengl_renderer {
    opengl_renderer();

    int32 init_gl_glfw(base_app* app_ptr, int32 width, int32 height);
    void shutdown();

    void poll_events();
    bool should_window_close();
    double get_time();
    float get_AR() const { return ((float)window_width / (float)window_height); }

    void start_2D_render(const texture& bg);
    void draw_dot(float lat, float lon, const vec3f& color = vec3f(1.0f, 1.0f, 1.0f), float alpha = 1.0f);
    void end_2D_render();
    uint32 get_2D_output() const { return handle_2D_render_output; }

    void clear_screen();
    void set_projection(const laml::Mat4& mat);
    void setup_frame(const laml::Vec3& cam_pos, float cam_yaw, float cam_pitch,
                     const laml::Mat3& render_frame = laml::Mat3(),
                     float render_scale = 1.0f);
    
    void bind_texture(const texture& tex);
    void draw_mesh(const triangle_mesh& mesh,
                   const laml::Vec3& position = laml::Vec3(),
                   const laml::Quat& orientation = laml::Quat());

    void draw_path(uint32 handle, uint32 N,     vec3f color = vec3f(1.0f, 1.0f, 1.0f), float alpha = 1.0f);
    void draw_plane(vec3f normal, float scale,  vec3f color = vec3f(1.0f, 1.0f, 1.0f), float alpha = 1.0f);
    void draw_vector(vec3f vector, float scale, vec3f color = vec3f(1.0f, 1.0f, 1.0f), float alpha = 1.0f);

    void start_debug_UI();
    void end_debug_UI();

    void end_frame();

private:
    int32 window_width, window_height;

    void* raw_glfw_window;

    shader_program basic_2D_shader;
    shader_program basic_shader;
    shader_program line_shader;

    uint32 box_2D_handle;
    uint32 handle_2D_render_output;
    uint32 handle_2D_framebuffer;

    uint32 plane_handle;
    triangle_mesh vector_mesh;
    uint32 dot_handle;
    uint32 num_dot_inds;
    texture blank_tex;

    mat3f render_frame;
    float render_scale;
    mat4f projection_matrix;

    // video recording
#if USE_DTV
    atg_dtv::Encoder encoder;

    uint8 *tmp_buffer = nullptr;
#endif
    bool init_recording();
    bool stop_recording();
};