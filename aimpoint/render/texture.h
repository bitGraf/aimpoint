#pragma once
#include "defines.h"

struct texture {

    bool load_texture_file(const char* filename);

private:
    uint32 width = 0;
    uint32 height = 0;
    uint32 num_components = 0;

    uint32 handle = 0;

    friend struct opengl_renderer;
};