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
    FILE* fid = fopen(filename, "rb");

    //// Load mesh from file
    //FILE* fid = fopen("../data/Cylinder.mesh", "rb");
    if (fid == nullptr) {
        // try one more directory up
        //fid = fopen("../../data/Cylinder.mesh", "rb");
    
        //if (fid == nullptr) {
            spdlog::critical("Could not open mesh file!\n");
            return false;
        //}
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

        float* vertices = (float*)malloc(num_verts[prim_idx]*6*sizeof(float));
        for (int n = 0; n < num_verts[prim_idx]; n++) {
            vertices[n*6 + 0] = scale_factor*vertices_file[n].position.x;
            vertices[n*6 + 1] = scale_factor*vertices_file[n].position.y;
            vertices[n*6 + 2] = scale_factor*vertices_file[n].position.z;

            vertices[n*6 + 3] = vertices_file[n].normal.x;
            vertices[n*6 + 4] = vertices_file[n].normal.y;
            vertices[n*6 + 5] = vertices_file[n].normal.z;
        }
        free(vertices_file);

        unsigned int VBO, VAO, EBO;
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);
        // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*6*num_verts[prim_idx], vertices, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*num_inds[prim_idx], indices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

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