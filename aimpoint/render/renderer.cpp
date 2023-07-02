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

static void glfw_error_callback(int error, const char* description);
static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
static void glfw_cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
static void glfw_mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
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

    glfwSetWindowUserPointer(window, app_ptr);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    
    glDebugMessageCallback(openGL_debug_msg_callback, nullptr);

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

    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n{0}", infoLog);
        return 3;
    }

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

    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);

    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n{0}", infoLog);
        return 3;
    }

    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::LINKING_FAILED\n{0}", infoLog);
        return 3;
    }

    basic_shader = shaderProgram;
    glUseProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // create basic shader
    const char *lineVertexShaderSource = "#version 430 core\n"
                                         "layout (location = 0) in vec3 a_Position;\n"
                                         "layout (location = 1) uniform mat4 r_View;\n"
                                         "layout (location = 2) uniform mat4 r_Projection;\n"
                                         "layout (location = 3) uniform mat4 r_Transform;\n"
                                         "void main() {\n"
                                         "    gl_Position = r_Projection * r_View * r_Transform * vec4(a_Position, 1.0);\n"
                                         "}\n";

    vertexShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertexShader, 1, &lineVertexShaderSource, NULL);
    glCompileShader(vertexShader);

    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

    if(!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n{0}", infoLog);
        return 3;
    }

    const char *lineFragmentShaderSource = "#version 430 core\n"
                                       "out vec4 FragColor;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   vec3 color = vec3(.3333f, 0.4588f, .5418f);\n"
                                       "   FragColor = vec4(color, 1.0f);\n"
                                       "}\0";

    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &lineFragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);

    if(!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n{0}", infoLog);
        return 3;
    }

    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::LINKING_FAILED\n{0}", infoLog);
        return 3;
    }

    line_shader = shaderProgram;
    glUseProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glLineWidth(4.0f);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.IniFilename = "../data/imgui.ini";
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

    glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    {
        glUseProgram(line_shader);

        laml::Mat4 cam_transform, view_matrix;
        laml::transform::create_transform(cam_transform, cam_yaw, cam_pitch, 0.0f, cam_pos);
        laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
        int viewLocation = glGetUniformLocation(line_shader, "r_View");
        glUniformMatrix4fv(viewLocation, 1, GL_FALSE, view_matrix._data);

        laml::Mat4 projection_matrix;
        float AR = ((float)window_width / (float)window_height);
        laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 1000.0f, 20'000'000.0f);
        //laml::transform::create_projection_orthographic(projection_matrix, -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
        int projLocation = glGetUniformLocation(line_shader, "r_Projection");
        glUniformMatrix4fv(projLocation, 1, GL_FALSE, projection_matrix._data);

        //laml::Mat4 transform_matrix(1.0f);
        //int transformLocation = glGetUniformLocation(line_shader, "r_Transform");
        //glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);
    }

    {
        glUseProgram(basic_shader);

        laml::Mat4 cam_transform, view_matrix;
        laml::transform::create_transform(cam_transform, cam_yaw, cam_pitch, 0.0f, cam_pos);
        laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
        int viewLocation = glGetUniformLocation(basic_shader, "r_View");
        glUniformMatrix4fv(viewLocation, 1, GL_FALSE, view_matrix._data);

        laml::Mat4 projection_matrix;
        float AR = ((float)window_width / (float)window_height);
        laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 1000.0f, 20'000'000.0f);
        //laml::transform::create_projection_orthographic(projection_matrix, -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
        int projLocation = glGetUniformLocation(basic_shader, "r_Projection");
        glUniformMatrix4fv(projLocation, 1, GL_FALSE, projection_matrix._data);
    }
}

void opengl_renderer::bind_texture(const texture& tex) {
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex.handle);
}

void opengl_renderer::draw_mesh(const triangle_mesh& mesh, 
                                const laml::Vec3& position, 
                                const laml::Quat& orientation) {
    glUseProgram(basic_shader);

    laml::Mat4 transform_matrix;
    //laml::transform::create_transform_translate(transform_matrix, position);
    //transform_matrix = laml::mul(laml::Mat4(render_frame), transform_matrix);
    laml::Quat r = laml::transform::quat_from_mat(render_frame);

    laml::transform::create_transform(transform_matrix, laml::mul(r, orientation), laml::transform::transform_point(render_frame, position));
    int transformLocation = glGetUniformLocation(basic_shader, "r_Transform");
    glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);

    for (int n = 0; n < mesh.num_prims; n++) {
        glBindVertexArray(mesh.handles[n]);
        glDrawElements(GL_TRIANGLES, mesh.num_inds[n], GL_UNSIGNED_INT, 0);
    }
}

void opengl_renderer::draw_path(uint32 handle, uint32 N) {
    glUseProgram(line_shader);

    laml::Mat4 transform_matrix;
    laml::Quat r = laml::transform::quat_from_mat(render_frame);

    laml::transform::create_transform_rotation(transform_matrix, r);
    int transformLocation = glGetUniformLocation(line_shader, "r_Transform");
    glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);

    glBindVertexArray(handle);
    glDrawElements(GL_LINE_LOOP, N, GL_UNSIGNED_INT, 0);
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