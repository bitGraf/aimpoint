#pragma once
#include "defines.h"

struct triangle_mesh {
    ~triangle_mesh();

    struct vertex {
        laml::Vec3 position;
        laml::Vec3 normal;
    };

    bool load_from_mesh_file(const char* filename, float scale_factor = 1.0f);
    bool load_from_mesh_file(const char* filename, float x_scale_factor, float y_scale_factor, float z_scale_factor);

private:
    uint16 num_prims = 0;
    uint32* handles = nullptr;
    uint32* num_verts = nullptr;
    uint32* num_inds = nullptr;
    uint32* mat_idxs = nullptr;

    uint32 flag = 0;

    friend struct opengl_renderer;
};