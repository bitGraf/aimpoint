#include "renderer.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include "log.h"

#include "aimpoint.h"

static void glfw_error_callback(int error, const char* description);
static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

opengl_renderer::opengl_renderer() : raw_glfw_window(nullptr), 
                                     window_width(64), 
                                     window_height(48) {}

int32 opengl_renderer::init_gl_glfw(aimpoint* app_ptr, int32 width, int32 height) {
    window_width  = width;
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

    glfwSetWindowUserPointer(window, app_ptr);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    glfwSwapInterval(2);

    glEnable(GL_DEPTH_TEST);

    // create basic shader
    const char *vertexShaderSource = "#version 430 core\n"
                                     "layout (location = 0) in vec3 a_Position;\n"
                                     "layout (location = 1) in vec3 a_Normal;\n"
                                     "layout (location = 1) uniform mat4 r_View;\n"
                                     "layout (location = 2) uniform mat4 r_Projection;\n"
                                     "layout (location = 3) uniform mat4 r_Transform;\n"
                                     "out vec3 out_normal;\n"
                                     "void main() {\n"
                                     "    gl_Position = r_Projection * r_View * r_Transform * vec4(a_Position, 1.0);\n"
                                     "    out_normal = vec3(r_Transform * vec4(a_Normal, 0.0f));\n"
                                     "}\n";

    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);

    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    int  success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);

    if(!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::VERTEX::COMPILATION_FAILED\n{0}", infoLog);
        return 3;
    }

    const char *fragmentShaderSource = "#version 430 core\n"
                                       "out vec4 FragColor;\n"
                                       "in vec3 out_normal;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   vec3 light_dir = normalize(vec3(1.0f, -5.0f, 3.0f));\n"
                                       "   vec3 color = vec3(.4f, 0.1f, .2f);\n"
                                       "   vec3 ambient = vec3(.2f, 0.2f, .2f);\n"
                                       "   FragColor = vec4(ambient + dot(light_dir, out_normal) * color, 1.0f);\n"
                                       "}\0";

    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);

    if(!success) {
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
    if(!success) {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        spdlog::critical("ERROR::SHADER::LINKING_FAILED\n{0}", infoLog);
        return 3;
    }

    shader = shaderProgram;
    glUseProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return 0;
}

void opengl_renderer::shutdown() {
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

void opengl_renderer::poll_events() {
    glfwPollEvents();
}
bool opengl_renderer::should_window_close() {
    return glfwWindowShouldClose((GLFWwindow*)raw_glfw_window);
}
double opengl_renderer::get_time() {
    return glfwGetTime();
}

void opengl_renderer::start_frame(const laml::Vec3& cam_pos, float cam_yaw, float cam_pitch){
    glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shader);

    laml::Mat4 cam_transform, view_matrix;
    laml::transform::create_transform_translate(cam_transform, cam_pos);
    laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
    int viewLocation = glGetUniformLocation(shader, "r_View");
    glUniformMatrix4fv(viewLocation, 1, GL_FALSE, view_matrix._data);

    laml::Mat4 projection_matrix;
    float AR = ((float)window_width / (float)window_height);
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 0.1f, 100.0f);
    //laml::transform::create_projection_orthographic(projection_matrix, -10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
    int projLocation = glGetUniformLocation(shader, "r_Projection");
    glUniformMatrix4fv(projLocation, 1, GL_FALSE, projection_matrix._data);
}

void opengl_renderer::draw_mesh(const triangle_mesh& mesh, const laml::Vec3& position, const laml::Quat& orientation) {
    laml::Mat4 transform_matrix;
    //laml::transform::create_transform_translate(transform_matrix, position);
    laml::transform::create_transform(transform_matrix, orientation, position);
    int transformLocation = glGetUniformLocation(shader, "r_Transform");
    glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);

    glBindVertexArray(mesh.handle);
    glDrawElements(GL_TRIANGLES, mesh.num_inds, GL_UNSIGNED_INT, 0);
}

void opengl_renderer::end_frame(){
    glfwSwapBuffers((GLFWwindow*)raw_glfw_window);
}