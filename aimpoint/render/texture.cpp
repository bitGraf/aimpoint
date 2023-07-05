#include "texture.h"

#include "log.h"

#include "stb/stb_image.h"
#include "glad/gl.h"
#include <stdio.h>

bool texture::load_texture_file(const char* filename) {
    // load texture
    stbi_set_flip_vertically_on_load(true);
    int width_, height_, num_comp_;
    uint8* data = stbi_load(filename, &width_, &height_, &num_comp_, 0);

    if (data == nullptr) {
        // try one more directory up
        char new_filename[256];
        snprintf(new_filename, 256, "../%s", filename);

        data = stbi_load(new_filename, &width_, &height_, &num_comp_, 0);
        if (data == nullptr) {
            char new_new_filename[256];
            snprintf(new_new_filename, 256, "../%s", new_filename);

            data = stbi_load(new_new_filename, &width_, &height_, &num_comp_, 0);

            if (data == nullptr) {
                spdlog::critical("Could not open image file!\n");
                return false;
            }
        }
    }

    width = width_;
    height = height_;
    num_components = num_comp_;
    spdlog::info("Loaded image! {0}x{1}x{2}", width, height, num_components);

    glGenTextures(1, &handle);
    glBindTexture(GL_TEXTURE_2D, handle);

    GLenum InternalFormat = 0;
    GLenum Format = 0;
    switch (num_components) {
        case 1: {
            InternalFormat = GL_R8;
            Format = GL_RED;
        } break;
        case 2: {
            InternalFormat = GL_RG8;
            Format = GL_RG;
        } break;
        case 3: {
            InternalFormat = GL_RGB8;
            Format = GL_RGB;
        } break;
        case 4: {
            InternalFormat = GL_RGBA8;
            Format = GL_RGBA;
        } break;
    }

    glTexImage2D(GL_TEXTURE_2D, 0, InternalFormat, width, height, 0, Format, GL_UNSIGNED_BYTE, data);
    //glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, ResolutionX, ResolutionY, 0, GL_RED, GL_UNSIGNED_BYTE, Bitmap);
    glGenerateMipmap(GL_TEXTURE_2D);

    // This for font texture
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // This for everything else
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);

    return true;
}