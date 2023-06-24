#include "mesh.h"

#include "log.h"

#include "glad/gl.h"
#include <stdio.h>

bool triangle_mesh::load_from_mesh_file(const char* filename) {
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
    uint16 num_prims;
    uint32 filesize, mesh_version;
    uint64 timestamp;

    fread(&filesize,     sizeof(uint32), 1, fid);
    fread(&mesh_version, sizeof(uint32), 1, fid);
    fread(&timestamp,    sizeof(uint64), 1, fid);
    fread(&flag,         sizeof(uint32), 1, fid);
    fread(&num_prims,    sizeof(uint16), 1, fid);

    // skip material
    fseek(fid, 48, SEEK_CUR);
    uint8 len;
    fread(&len, sizeof(uint8), 1, fid);
    fseek(fid, len, SEEK_CUR);

    // read first primitive
    fseek(fid, 4, SEEK_CUR); // "PRIM"
    //uint32 num_verts, num_inds;
    fread(&num_verts, sizeof(uint32), 1, fid);
    fread(&num_inds,  sizeof(uint32), 1, fid);
    fseek(fid, 4, SEEK_CUR); // mat_idx

    uint32* indices = (uint32*)malloc(num_inds*sizeof(uint32));
    fread(indices, sizeof(uint32), num_inds, fid);

    struct vertex_file {
        laml::Vec3 position;
        laml::Vec3 normal;
        laml::Vec3 tangent;
        laml::Vec3 bitangent;
        laml::Vec2 texcoord;
    };
    vertex_file* vertices_file = (vertex_file*)malloc(num_verts*sizeof(vertex_file));
    fread(vertices_file, sizeof(vertex_file), num_verts, fid);

    fclose(fid);

    float* vertices = (float*)malloc(num_verts*6*sizeof(float));
    for (int n = 0; n < num_verts; n++) {
        vertices[n*6 + 0] = vertices_file[n].position.x;
        vertices[n*6 + 1] = vertices_file[n].position.y;
        vertices[n*6 + 2] = vertices_file[n].position.z;

        vertices[n*6 + 3] = vertices_file[n].normal.x;
        vertices[n*6 + 4] = vertices_file[n].normal.y;
        vertices[n*6 + 5] = vertices_file[n].normal.z;
    }
    free(vertices_file);

    spdlog::info("'{0}' loaded", filename);
    spdlog::info("FileSize: {0}", filesize);
    spdlog::info("Verts: {0}", num_verts);
    spdlog::info("Inds: {0}", num_inds);

    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*6*num_verts, vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*num_inds, indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    glBindBuffer(GL_ARRAY_BUFFER, 0); 

    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    glBindVertexArray(0); 

    free(indices);
    free(vertices);

    handle = VAO;

    return true;
}