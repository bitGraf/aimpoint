#pragma once
#include "defines.h"

struct shader_program {

    bool create_shader_from_source(const char* vertex_src, const char* fragment_src);

    void bind() const;

    void set_uniform(const char* name, uint32 value);
    void set_uniform(const char* name, float value);
    void set_uniform(const char* name, const vec3f& value);
    void set_uniform(const char* name, const mat4f& value);

private:
    uint32 handle = 0;

    enum class stage {
        vertex = 1,
        geometry,
        fragment
    };
    static uint32 create_shader_stage(const char* src, shader_program::stage shader_stage);

    friend struct opengl_renderer;
};