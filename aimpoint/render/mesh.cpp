#include "mesh.h"

#include "log.h"

#include "glad/gl.h"
#include <stdio.h>

triangle_mesh::~triangle_mesh() {
    free(handles);
    free(num_verts);
    free(num_inds);
    free(mat_idxs);
}

bool triangle_mesh::load_from_mesh_file(const char* filename, float scale_factor) {
    return load_from_mesh_file(filename, scale_factor, scale_factor, scale_factor);
}
bool triangle_mesh::load_from_mesh_file(const char* filename, float x_scale_factor, float y_scale_factor, float z_scale_factor) {
    FILE* fid = fopen(filename, "rb");
    if (fid == nullptr) {
        // try one more directory up
        char new_filename[256];
        snprintf(new_filename, 256, "../%s", filename);
        fid = fopen(new_filename, "rb");
    
        if (fid == nullptr) {
            // try one more directory up
            char new_new_filename[256];
            snprintf(new_new_filename, 256, "../%s", new_filename);
            fid = fopen(new_new_filename, "rb");
    
            if (fid == nullptr) {
                spdlog::critical("Could not open mesh file!\n");
                return false;
            }
        }
    }
    fseek(fid, 4, SEEK_SET); // "MESH"
    uint32 filesize, mesh_version;
    uint64 timestamp;

    fread(&filesize,     sizeof(uint32), 1, fid);
    fread(&mesh_version, sizeof(uint32), 1, fid);
    fread(&timestamp,    sizeof(uint64), 1, fid);
    fread(&flag,         sizeof(uint32), 1, fid);
    fread(&num_prims,    sizeof(uint16), 1, fid);

    spdlog::info("'{0}' loaded", filename);
    spdlog::debug("FileSize: {0}", filesize);
    spdlog::debug("{0} primitives", num_prims);

    handles = (uint32*)malloc(sizeof(uint32)*num_prims);
    num_verts = (uint32*)malloc(sizeof(uint32)*num_prims);
    num_inds = (uint32*)malloc(sizeof(uint32)*num_prims);
    mat_idxs = (uint32*)malloc(sizeof(uint32)*num_prims);

    // skip materials
    for (int prim_idx = 0; prim_idx < num_prims; prim_idx++) {
        fseek(fid, 4, SEEK_CUR);

        uint32 mat_flag;
        fread(&mat_flag, sizeof(uint32), 1, fid);

        bool mat_double_sided          = mat_flag & 0x01;
        bool mat_diffuse_has_texture   = mat_flag & 0x02;
        bool mat_normal_has_texture    = mat_flag & 0x04;
        bool mat_amr_has_texture       = mat_flag & 0x08;
        bool mat_emissive_has_texture  = mat_flag & 0x10;

        // skip the rest of the data
        fseek(fid, 40, SEEK_CUR);

        // mat name
        uint8 len;
        fread(&len, sizeof(uint8), 1, fid);
        fseek(fid, len, SEEK_CUR);

        if (mat_diffuse_has_texture) {
            uint8 tex_len;
            fread(&tex_len, sizeof(uint8), 1, fid);
            fseek(fid, tex_len, SEEK_CUR);
        }
        if (mat_normal_has_texture) {
            uint8 tex_len;
            fread(&tex_len, sizeof(uint8), 1, fid);
            fseek(fid, tex_len, SEEK_CUR);
        }
        if (mat_amr_has_texture) {
            uint8 tex_len;
            fread(&tex_len, sizeof(uint8), 1, fid);
            fseek(fid, tex_len, SEEK_CUR);
        }
        if (mat_emissive_has_texture) {
            uint8 tex_len;
            fread(&tex_len, sizeof(uint8), 1, fid);
            fseek(fid, tex_len, SEEK_CUR);
        }
    }

    // read primitives
    struct vertex_file {
        laml::Vec3 position;
        laml::Vec3 normal;
        laml::Vec3 tangent;
        laml::Vec3 bitangent;
        laml::Vec2 texcoord;
    };


    for (int prim_idx = 0; prim_idx < num_prims; prim_idx++) {
        fseek(fid, 4, SEEK_CUR); // "PRIM"

        //uint32 num_verts, num_inds;
        fread(&num_verts[prim_idx], sizeof(uint32), 1, fid);
        fread(&num_inds[prim_idx],  sizeof(uint32), 1, fid);
        fread(&mat_idxs[prim_idx],  sizeof(uint32), 1, fid);

        spdlog::debug("  Primitive #{0}", prim_idx);
        spdlog::debug("    Verts: {0}", num_verts[prim_idx]);
        spdlog::debug("    Inds: {0}", num_inds[prim_idx]);
        spdlog::debug("    Mat_idx: {0}", mat_idxs[prim_idx]);

        uint32* indices = (uint32*)malloc(num_inds[prim_idx]*sizeof(uint32));
        fread(indices, sizeof(uint32), num_inds[prim_idx], fid);

        vertex_file* vertices_file = (vertex_file*)malloc(num_verts[prim_idx]*sizeof(vertex_file));
        fread(vertices_file, sizeof(vertex_file), num_verts[prim_idx], fid);

        const size_t num_attr = 8;
        float* vertices = (float*)malloc(num_verts[prim_idx]*num_attr*sizeof(float));
        for (int n = 0; n < num_verts[prim_idx]; n++) {
            vertices[n*num_attr + 0] = x_scale_factor*vertices_file[n].position.x;
            vertices[n*num_attr + 1] = y_scale_factor*vertices_file[n].position.y;
            vertices[n*num_attr + 2] = z_scale_factor*vertices_file[n].position.z;

            vertices[n*num_attr + 3] = vertices_file[n].normal.x;
            vertices[n*num_attr + 4] = vertices_file[n].normal.y;
            vertices[n*num_attr + 5] = vertices_file[n].normal.z;

            vertices[n*num_attr + 6] = vertices_file[n].texcoord.x;
            vertices[n*num_attr + 7] = vertices_file[n].texcoord.y;
        }
        free(vertices_file);

        unsigned int VBO, VAO, EBO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*num_attr*num_verts[prim_idx], vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*num_inds[prim_idx], indices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)(6*sizeof(float)));
        glEnableVertexAttribArray(2);

        glBindBuffer(GL_ARRAY_BUFFER, 0); 
        glBindVertexArray(0); 

        free(indices);
        free(vertices);

        handles[prim_idx] = VAO;
        spdlog::debug("    handle: {0}", handles[prim_idx]);
    }

    char tag[] = "----";
    fread(tag, sizeof(uint8), 4, fid);
    spdlog::trace("End tag = '{0}'", tag);

    fclose(fid);

    return true;
}

bool triangle_mesh::create_plane(vec3f normal, float sizeX, float sizeY) {
    num_prims = 1;

    handles   = (uint32*)malloc(sizeof(uint32)*num_prims);
    num_verts = (uint32*)malloc(sizeof(uint32)*num_prims);
    num_inds  = (uint32*)malloc(sizeof(uint32)*num_prims);
    mat_idxs  = (uint32*)malloc(sizeof(uint32)*num_prims);

    num_verts[0] = 4;
    num_inds[0] = 6;
    mat_idxs[0] = 0;
    int32 num_attr = 8;

    // determine frame
    vec3f Z_vec(0.0f, 0.0f, 1.0f);

    normal = laml::normalize(normal);
    vec3f tangent = laml::cross(Z_vec, normal);
    float m = laml::length(tangent);
    if (laml::abs(m) < 1e-9) {
        vec3f Y_vec(0.0f, 1.0f, 0.0f);
        tangent = laml::normalize(laml::cross(Y_vec, normal));
    } else {
        tangent = tangent/m;
    }
    vec3f bitangent = laml::cross(normal, tangent);

    // extents
    vec3f V0 = sizeX*(-bitangent) + sizeY*( tangent);
    vec3f V1 = sizeX*( bitangent) + sizeY*( tangent);
    vec3f V2 = sizeX*( bitangent) + sizeY*(-tangent);
    vec3f V3 = sizeX*(-bitangent) + sizeY*(-tangent);

    // create simple plane mesh
    float verts[] = {V0.x, V0.y, V0.z, normal.x, normal.y, normal.z, 0.0f, 0.0f,
                     V1.x, V1.y, V1.z, normal.x, normal.y, normal.z, 1.0f, 0.0f,
                     V2.x, V2.y, V2.z, normal.x, normal.y, normal.z, 1.0f, 1.0f,
                     V3.x, V3.y, V3.z, normal.x, normal.y, normal.z, 0.0f, 1.0f,
    };
    uint32 inds[] = {
                    0, 1, 2,
                    0, 2, 3
    };
    // load points into GPU
    unsigned int VBO, EBO;
    glGenVertexArrays(1, &handles[0]);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(handles[0]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*num_verts[0]*num_attr, verts, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*num_inds[0], inds, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, num_attr * sizeof(float), (void*)(6*sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}