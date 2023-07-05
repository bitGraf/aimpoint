#include "renderer.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

// Dear ImGui
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <implot.h>

#include "log.h"

#include "aimpoint.h"

// silly function :/
const char* find_imgui_ini_file() {
    FILE* fid = fopen("data/imgui.ini", "r");
    if (fid) {
        fclose(fid);
        return "data/imgui.ini";
    }

    fid = fopen("../data/imgui.ini", "r");
    if (fid) {
        fclose(fid);
        return "../data/imgui.ini";
    }

    fid = fopen("../../data/imgui.ini", "r");
    if (fid) {
        fclose(fid);
        return "../../data/imgui.ini";
    }

    return "imgui.ini";
}

static void glfw_error_callback(int error, const char* description);
static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
static void glfw_cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
static void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset);


void APIENTRY openGL_debug_msg_callback(GLenum source,
                                        GLenum type,
                                        GLuint id,
                                        GLenum severity,
                                        GLsizei length,
                                        const GLchar *message,
                                        const void *userParam);

opengl_renderer::opengl_renderer() : raw_glfw_window(nullptr), 
                                     window_width(64), 
                                     window_height(48) {}

int32 opengl_renderer::init_gl_glfw(aimpoint* app_ptr, int32 width, int32 height) {
    window_width = width;
    window_height = height;

    // setup glfw
    if (!glfwInit()) {
        // init failed
        spdlog::critical("Failed to initialize GLFW");
        return 1;
    }
    glfwSetErrorCallback(glfw_error_callback);

    // create window and OpenGL context
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Aimpoint", NULL, NULL);
    raw_glfw_window = window;
    if (!window) {
        // Window or OpenGL context creation failed
        spdlog::critical("Failed to create GLFW Window");
        return 2;
    }

    glfwSetKeyCallback(window, glfw_key_callback);
    glfwSetCursorPosCallback(window, glfw_cursor_pos_callback);
    glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
    glfwSetScrollCallback(window, glfw_scroll_callback);

    glfwSetWindowUserPointer(window, app_ptr);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    
    glDebugMessageCallback(openGL_debug_msg_callback, nullptr);

    // get the actual framebuffer size
    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    glfwSwapInterval(2);

    glEnable(GL_DEPTH_TEST);

    // create basic shader
    const char *vertexShaderSource = "#version 430 core\n"
                                     "layout (location = 0) in vec3 a_Position;\n"
                                     "layout (location = 1) in vec3 a_Normal;\n"
                                     "layout (location = 2) in vec2 a_TexCoord;\n"
                                     "layout (location = 1) uniform mat4 r_View;\n"
                                     "layout (location = 2) uniform mat4 r_Projection;\n"
                                     "layout (location = 3) uniform mat4 r_Transform;\n"
                                     "out vec3 out_normal;\n"
                                     "out vec2 out_texcoord;\n"
                                     "void main() {\n"
                                     "    gl_Position = r_Projection * r_View * r_Transform * vec4(a_Position, 1.0);\n"
                                     "    out_normal = vec3(r_Transform * vec4(a_Normal, 0.0f));\n"
                                     "    out_texcoord = a_TexCoord;\n"
                                     "}\n";

    const char *fragmentShaderSource = "#version 430 core\n"
                                       "out vec4 FragColor;\n"
                                       "in vec3 out_normal;\n"
                                       "in vec2 out_texcoord;\n"
                                       "uniform sampler2D diffuse_tex;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   vec3 light_dir = normalize(vec3(1.0f, -5.0f, 3.0f));\n"
                                       "   vec3 color = vec3(.3333f, 0.4588f, .5418f);\n"
                                       "   vec3 tex_color = texture(diffuse_tex, out_texcoord).rgb;\n"
                                       "   vec3 ambient = vec3(.15f, 0.15f, .15f);\n"
                                       "   FragColor = vec4(ambient + dot(light_dir, out_normal) * tex_color, 1.0f);\n"
                                       "   FragColor = vec4(tex_color, 1.0f);\n"
                                       "}\0";
    if (!basic_shader.create_shader_from_source(vertexShaderSource, fragmentShaderSource)) {
        return 3;
    }

    // create Line shader
    const char *lineVertexShaderSource = "#version 430 core\n"
                                         "layout (location = 0) in vec3 a_Position;\n"
                                         "layout (location = 1) uniform mat4 r_View;\n"
                                         "layout (location = 2) uniform mat4 r_Projection;\n"
                                         "layout (location = 3) uniform mat4 r_Transform;\n"
                                         "void main() {\n"
                                         "    gl_Position = r_Projection * r_View * r_Transform * vec4(a_Position, 1.0);\n"
                                         "}\n";

    const char *lineFragmentShaderSource = "#version 430 core\n"
                                           "out vec4 FragColor;\n"
                                           "layout (location = 4) uniform vec3 r_color;\n"
                                           "layout (location = 5) uniform float r_alpha;\n"
                                           "void main()\n"
                                           "{\n"
                                           "   //vec3 color = vec3(.3333f, 0.4588f, .5418f);\n"
                                           "   FragColor = vec4(r_color, r_alpha);\n"
                                           "}\0";

    if (!line_shader.create_shader_from_source(lineVertexShaderSource, lineFragmentShaderSource)) {
        return 3;
    }

    // create 2D shader
    const char *twoDVertexShaderSource = "#version 430 core\n"
                                         "layout (location = 0) in vec2 a_Position;\n"
                                         "layout (location = 1) in vec2 a_TexCoord;\n"
                                         "out vec2 out_texcoord;\n"
                                         "layout (location = 1) uniform mat4 r_Projection;\n"
                                         "layout (location = 2) uniform vec2 r_position;\n"
                                         "void main() {\n"
                                         "    gl_Position = r_Projection * vec4(a_Position.x+r_position.x, a_Position.y+r_position.y, 0.0, 1.0);\n"
                                         "    out_texcoord = a_TexCoord;\n"
                                         "}\n";

    const char *twoDFragmentShaderSource = "#version 430 core\n"
                                           "out vec4 FragColor;\n"
                                           "in vec2 out_texcoord;\n"
                                           "layout (location = 3) uniform vec3 r_color;\n"
                                           "layout (location = 4) uniform float r_alpha;\n"
                                           "uniform sampler2D diffuse_tex;\n"
                                           "void main()\n"
                                           "{\n"
                                           "   vec4 tex_raw = texture(diffuse_tex, out_texcoord);\n"
                                           "   vec3 tex_color = tex_raw.rgb;\n"
                                           "   float tex_alpha = tex_raw.a;\n"
                                           "   FragColor = vec4(r_color*tex_color, tex_alpha*r_alpha);\n"
                                           "}\0";

    if (!basic_2D_shader.create_shader_from_source(twoDVertexShaderSource, twoDFragmentShaderSource)) {
        return 3;
    }
    basic_2D_shader.set_uniform("diffuse_tex", uint32(0));

    // OpenGL settings
    glLineWidth(4.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // load primitives
    vector_mesh.load_from_mesh_file("data/vector.mesh", 1.0f, 0.7f, 0.7f);
    blank_tex.load_texture_file("data/circle.png");

    // create simple plane mesh
    {
        float verts[] = {
            0.0f, -1.0f, -1.0f,
            0.0f,  1.0f, -1.0f,
            0.0f,  1.0f,  1.0f,
            0.0f, -1.0f,  1.0f
        };
        uint32 inds[] = {
            0, 1, 2,
            0, 2, 3
        };
        // load points into GPU
        unsigned int VBO, EBO;
        glGenVertexArrays(1, &plane_handle);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(plane_handle);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*12, verts, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*6, inds, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    // create 2D assets
    {
        float verts[] = {
                            -180.0f, -90.0f, 0.0f,   0.0f, 0.0f,
                             180.0f, -90.0f, 0.0f,   1.0f, 0.0f,
                             180.0f,  90.0f, 0.0f,   1.0f, 1.0f,
                            -180.0f,  90.0f, 0.0f,   0.0f, 1.0f,
        };
        uint32 inds[] = {
                        0, 1, 2,
                        0, 2, 3
        };
        // load points into GPU
        unsigned int VBO, EBO;
        glGenVertexArrays(1, &box_2D_handle);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(box_2D_handle);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*20, verts, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*6, inds, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    {
        float verts[] = {
                        -2.5f, -2.5f, 0.0f,   0.0f, 0.0f,
                         2.5f, -2.5f, 0.0f,   1.0f, 0.0f,
                         2.5f,  2.5f, 0.0f,   1.0f, 1.0f,
                        -2.5f,  2.5f, 0.0f,   0.0f, 1.0f,
        };
        uint32 inds[] = {
                        0, 1, 2,
                        0, 2, 3
        };
        num_dot_inds = 6;
        // load points into GPU
        unsigned int VBO, EBO;
        glGenVertexArrays(1, &dot_handle);
        glGenBuffers(1, &VBO);
        glGenBuffers(1, &EBO);

        glBindVertexArray(dot_handle);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float)*5*4, verts, GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32)*num_dot_inds, inds, GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3*sizeof(float)));
        glEnableVertexAttribArray(1);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

    // setup framebuffer / render texture
    {
        glGenFramebuffers(1, &handle_2D_framebuffer);
        glBindFramebuffer(GL_FRAMEBUFFER, handle_2D_framebuffer);
        // create a color attachment texture
        glGenTextures(1, &handle_2D_render_output);
        glBindTexture(GL_TEXTURE_2D, handle_2D_render_output);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, window_width, window_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, handle_2D_render_output, 0);
        // create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
        unsigned int rbo;
        glGenRenderbuffers(1, &rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, window_width, window_height); // use a single renderbuffer object for both a depth AND stencil buffer.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); // now actually attach it
        // now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            return 4;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.IniFilename = find_imgui_ini_file();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking
    //io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;       // Enable Multi-Viewport / Platform Windows
    io.ConfigDockingWithShift = true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    const char* glsl_version = "#version 130";
    ImGui_ImplOpenGL3_Init(glsl_version);

    init_recording();

    return 0;
}

void opengl_renderer::shutdown() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    stop_recording();

    glfwDestroyWindow((GLFWwindow*)raw_glfw_window);

    glfwTerminate();
}

// GLFW Callbacks
void glfw_error_callback(int error, const char* description) {
    spdlog::error("[GLFW] Error {0}: {1}", error, description);
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);

    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->key_callback(key, scancode, action, mods);
}

void glfw_cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->mouse_pos_callback(xpos, ypos);
}

void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->mouse_button_callback(button, action, mods);
}

void glfw_scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->mouse_scroll_callback(xoffset, yoffset);
}

// main functions
void opengl_renderer::poll_events() {
    glfwPollEvents();
}
bool opengl_renderer::should_window_close() {
    return glfwWindowShouldClose((GLFWwindow*)raw_glfw_window);
}
double opengl_renderer::get_time() {
    return glfwGetTime();
}

void opengl_renderer::start_frame(const laml::Vec3& cam_pos, float cam_yaw, float cam_pitch,
                                  const laml::Mat3& new_render_frame){
    render_frame = new_render_frame;

    //glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    {
        line_shader.bind();

        laml::Mat4 cam_transform, view_matrix;
        laml::transform::create_transform(cam_transform, cam_yaw, cam_pitch, 0.0f, cam_pos);
        laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
        line_shader.set_uniform("r_View", view_matrix);

        laml::Mat4 projection_matrix;
        float AR = ((float)window_width / (float)window_height);
        laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 1000.0f, 50'000'000.0f);
        //laml::transform::create_projection_orthographic(projection_matrix, -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
        line_shader.set_uniform("r_Projection", projection_matrix);

        //laml::Mat4 transform_matrix(1.0f);
        //int transformLocation = glGetUniformLocation(line_shader, "r_Transform");
        //glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);

        vec3f color(1.0f, 1.0f, 1.0f);
        line_shader.set_uniform("r_color", color);

        float alpha = 1.0f;
        line_shader.set_uniform("r_alpha", alpha);
    }

    {
        basic_shader.bind();

        laml::Mat4 cam_transform, view_matrix;
        laml::transform::create_transform(cam_transform, cam_yaw, cam_pitch, 0.0f, cam_pos);
        laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
        line_shader.set_uniform("r_View", view_matrix);

        laml::Mat4 projection_matrix;
        float AR = ((float)window_width / (float)window_height);
        laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 1000.0f, 50'000'000.0f);
        //laml::transform::create_projection_orthographic(projection_matrix, -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
        line_shader.set_uniform("r_Projection", projection_matrix);
    }
}

void opengl_renderer::bind_texture(const texture& tex) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex.handle);
}

void opengl_renderer::draw_mesh(const triangle_mesh& mesh, 
                                const laml::Vec3& position, 
                                const laml::Quat& orientation) {
    basic_shader.bind();

    laml::Mat4 transform_matrix;
    //laml::transform::create_transform_translate(transform_matrix, position);
    //transform_matrix = laml::mul(laml::Mat4(render_frame), transform_matrix);
    laml::Quat r = laml::transform::quat_from_mat(render_frame);

    laml::transform::create_transform(transform_matrix, laml::mul(r, orientation), laml::transform::transform_point(render_frame, position));
    basic_shader.set_uniform("r_Transform", transform_matrix);

    for (int n = 0; n < mesh.num_prims; n++) {
        glBindVertexArray(mesh.handles[n]);
        glDrawElements(GL_TRIANGLES, mesh.num_inds[n], GL_UNSIGNED_INT, 0);
    }
}

void opengl_renderer::draw_path(uint32 handle, uint32 N, vec3f color, float alpha) {
    line_shader.bind();

    laml::Mat4 transform_matrix;
    laml::Quat r = laml::transform::quat_from_mat(render_frame);

    laml::transform::create_transform_rotation(transform_matrix, r);
    line_shader.set_uniform("r_Transform", transform_matrix);

    line_shader.set_uniform("r_color", color);
    line_shader.set_uniform("r_alpha", alpha);

    glBindVertexArray(handle);
    glDrawElements(GL_LINE_LOOP, N, GL_UNSIGNED_INT, 0);
}

void opengl_renderer::draw_plane(vec3f normal, float scale, vec3f color, float alpha) {
    line_shader.bind();

    vec3f Z_vec(0.0f, 0.0f, 1.0f);

    vec3f tangent = laml::cross(Z_vec, normal);
    float m = laml::length(tangent);
    if (laml::abs(m) < 1e-9) {
        vec3f Y_vec(0.0f, 1.0f, 0.0f);
        tangent = laml::normalize(laml::cross(Y_vec, normal));
    } else {
        tangent = tangent/m;
    }
    vec3f bitangent = laml::cross(normal, tangent);

    mat3f rot(normal, tangent, bitangent);
    laml::Mat4 transform_matrix(laml::mul(render_frame, rot*scale));

    line_shader.set_uniform("r_Transform", transform_matrix);
    line_shader.set_uniform("r_color", color);
    line_shader.set_uniform("r_alpha", alpha);

    glBindVertexArray(plane_handle);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void opengl_renderer::draw_vector(vec3f vector, float scale, vec3f color, float alpha) {
    line_shader.bind();

    vec3f Z_vec(0.0f, 0.0f, 1.0f);

    vec3f tangent = laml::cross(Z_vec, vector);
    float m = laml::length(tangent);
    if (laml::abs(m) < 1e-9) {
        vec3f Y_vec(0.0f, 1.0f, 0.0f);
        tangent = laml::normalize(laml::cross(Y_vec, vector));
    } else {
        tangent = tangent/m;
    }
    vec3f bitangent = laml::cross(vector, tangent);

    mat3f rot(vector, tangent, bitangent);
    laml::Mat4 transform_matrix(laml::mul(render_frame, rot*scale));

    line_shader.set_uniform("r_Transform", transform_matrix);

    line_shader.set_uniform("r_color", color);
    line_shader.set_uniform("r_alpha", alpha);

    glBindVertexArray(vector_mesh.handles[0]);
    glDrawElements(GL_TRIANGLES, vector_mesh.num_inds[0], GL_UNSIGNED_INT, 0);
}

// 2D pass
void opengl_renderer::start_2D_render(const texture& bg) {
    glBindFramebuffer(GL_FRAMEBUFFER, handle_2D_framebuffer);

    glDisable(GL_DEPTH_TEST);

    //glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    basic_2D_shader.bind();

    laml::Mat4 projection_matrix;
    laml::transform::create_projection_orthographic(projection_matrix, -180.0f, 180.0f, -90.0f, 90.0f, -1.0f, 1.0f);
    basic_2D_shader.set_uniform("r_Projection", projection_matrix);

    basic_2D_shader.set_uniform("r_color", vec3f(1.0f, 1.0f, 1.0f));
    basic_2D_shader.set_uniform("r_alpha", 1.0f);
    basic_2D_shader.set_uniform("r_position", laml::Vec2(0.0f, 0.0f));

    bind_texture(bg);

    glBindVertexArray(box_2D_handle);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void opengl_renderer::draw_dot(float lat, float lon, const vec3f& color, float alpha) {
    basic_2D_shader.bind();

    basic_2D_shader.set_uniform("r_color", color);
    basic_2D_shader.set_uniform("r_alpha", alpha);
    basic_2D_shader.set_uniform("r_position", laml::Vec2(lon, lat));

    bind_texture(blank_tex);

    glBindVertexArray(dot_handle);
    glDrawElements(GL_TRIANGLES, num_dot_inds, GL_UNSIGNED_INT, 0);
}
void opengl_renderer::end_2D_render() {
    glEnable(GL_DEPTH_TEST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void opengl_renderer::start_debug_UI() {    
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::DockSpaceOverViewport(NULL, ImGuiDockNodeFlags_PassthruCentralNode);
}

void opengl_renderer::end_debug_UI() {
    // Rendering
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void opengl_renderer::end_frame(){
    glfwSwapBuffers((GLFWwindow*)raw_glfw_window);

    // RECORDING
#if USE_DTV
    atg_dtv::Frame *frame = encoder.newFrame(false);
    if (frame) {
        if (encoder.getError() != atg_dtv::Encoder::Error::None) {
            spdlog::error("RTV Error!");
        }

        // NOTE: This is very SLOW!! possibly need to look into pixel buffer objects?
        //       not an issue rn tho...
        //glReadPixels(0, 0, window_width, window_height, GL_RGB, GL_UNSIGNED_BYTE, frame->m_rgb);

        if (tmp_buffer == nullptr) {
            tmp_buffer = new uint8[(size_t)frame->m_maxHeight * frame->m_lineWidth];
        }
        glReadPixels(0, 0, window_width, window_height, GL_RGB, GL_UNSIGNED_BYTE, tmp_buffer);
        const int lineWidth = frame->m_lineWidth;
        for (int y = 0; y < frame->m_maxHeight; ++y) {
            uint8_t *row_src = &tmp_buffer[(frame->m_maxHeight - y - 1) * lineWidth];
            uint8_t *row_dst = &frame->m_rgb[y * lineWidth];

            memcpy(row_dst, row_src, frame->m_lineWidth);
        }

        encoder.submitFrame();
    } else {
        spdlog::info("RTV Done!");
    }
#endif
}

bool opengl_renderer::init_recording() {
#if USE_DTV
    atg_dtv::Encoder::VideoSettings settings{};

    // Output filename
    settings.fname = "output.mp4";

    // Input dimensions
    settings.inputWidth = window_width;
    settings.inputHeight = window_height;

    // Output dimensions
    settings.width = window_width;
    settings.height = window_height;

    // Encoder settings
    settings.hardwareEncoding = true;
    settings.bitRate = 16000000; // 16 Mbits?

    const int VideoLengthSeconds = 10;
    const int FrameCount = VideoLengthSeconds * settings.frameRate;

    encoder.run(settings, 2);
#endif
    return true;
}

bool opengl_renderer::stop_recording() {
#if USE_DTV
    encoder.commit();
    encoder.stop();

    if (tmp_buffer) {
        delete[] tmp_buffer;
    }
    tmp_buffer = nullptr;
#endif
    return true;
}




void APIENTRY openGL_debug_msg_callback(GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar *message,
    const void *userParam) {
    spdlog::trace("OpenGL Message: {0}", message);
}