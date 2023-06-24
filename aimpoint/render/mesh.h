#pragma once
#include "defines.h"

struct triangle_mesh {
    struct vertex {
        laml::Vec3 position;
        laml::Vec3 normal;
    };

    bool load_from_mesh_file(const char* filename);

private:
    uint32 handle;

    uint32 num_verts;
    uint32 num_inds;

    uint32 flag;

    friend struct opengl_renderer;
};