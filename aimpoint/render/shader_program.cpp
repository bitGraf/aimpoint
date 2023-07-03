#include "shader_program.h"

#include "log.h"

#include "glad/gl.h"
#include <stdio.h>

static int success;
static char infoLog[512];

bool shader_program::create_shader_from_source(const char* vertex_src, const char* fragment_src) {
    // create vertex shader
    uint32 vertexShader = create_shader_stage(vertex_src, stage::vertex);

    // create fragment shader
    uint32 fragmentShader = create_shader_stage(fragment_src, stage::fragment);

    if (vertexShader*fragmentShader == 0) {
        spdlog::critical("Failed in creating individual shader stage.");
        return false;
    }

    // create and link final shader program
    uint32 shaderProgram;
    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        spdlog::critical("Shader progam linking failed!:\n {0}", infoLog);
        return false;
    }

    handle = shaderProgram; //save handle

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

uint32 shader_program::create_shader_stage(const char* src, shader_program::stage shader_stage) {
    GLenum gl_shader_type;
    const char* gl_shader_string;
    switch (shader_stage) {
        case stage::vertex: {
            gl_shader_type = GL_VERTEX_SHADER;
            gl_shader_string = "Vertex";
        } break;
        case stage::fragment: {
            gl_shader_type = GL_FRAGMENT_SHADER;
            gl_shader_string = "Fragment";
        } break;
        case stage::geometry: {
            spdlog::error("Geometry stages not yet implemented!");
            return 0;
        } break;
    }
    // create shader stage
    uint32 shader_handle;
    shader_handle = glCreateShader(gl_shader_type);

    glShaderSource(shader_handle, 1, &src, NULL);
    glCompileShader(shader_handle);

    glGetShaderiv(shader_handle, GL_COMPILE_STATUS, &success);

    if (!success) {
        glGetShaderInfoLog(shader_handle, 512, NULL, infoLog);
        spdlog::critical("{1} shader compilation failed!:\n {0}", infoLog, gl_shader_string);
        return 0;
    }

    return shader_handle;
}

void shader_program::bind() const {
    glUseProgram(handle);
}

void shader_program::set_uniform(const char* name, uint32 value){
    int location = glGetUniformLocation(handle, name);
    glUniform1i(location, value);
}
void shader_program::set_uniform(const char* name, float value){
    int location = glGetUniformLocation(handle, name);
    glUniform1f(location, value);
}
void shader_program::set_uniform(const char* name, const vec3f& value){
    int location = glGetUniformLocation(handle, name);
    glUniform3fv(location, 1, value._data);
}
void shader_program::set_uniform(const char* name, const mat4f& value){
    int location = glGetUniformLocation(handle, name);
    glUniformMatrix4fv(location, 1, GL_FALSE, value._data);
}