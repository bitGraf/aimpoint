#include "aimpoint.h"

#include "log.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <thread>
#include <chrono>

// global vars for GLFW
static GLFWwindow* window = nullptr;
void glfw_error_callback(int error, const char* description);
void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

int aimpoint::run() {
    if (init()) {
        // failed on initialization
        return 1;
    }

    sim_time = 0.0;
    const double step_time = 1.0 / simulation_rate;

    wall_time = glfwGetTime();
    double accum_time = 0.0;

    bool done = false;
    while(!done) {
        double new_time = glfwGetTime();
        double frame_time = new_time - wall_time;
        wall_time = new_time;

        accum_time += frame_time;

        glfwPollEvents();
        if (glfwWindowShouldClose(window) || sim_time >= 150.0) {
            done = true;
        }

        if (real_time) {
            while (accum_time >= step_time) {
                step(step_time);
                sim_time += step_time;
                accum_time -= step_time;
            }
        } else {
            double render_time = wall_time + 1.0/65.0;
            while (glfwGetTime() < render_time) {
                step(step_time);
                sim_time += step_time;
            }
        }

        double alpha = accum_time / step_time; // interpolation value [0,1]
        render();
        glfwSwapBuffers(window);
    }

    shutdown();

    return 0;
}

int aimpoint::init() {
    simulation_rate = 1000.0; // Hz
    sim_frame = 0;
    real_time = true;

    sim_time = 0.0;
    wall_time = 0.0;

    window_width = 640;
    window_height = 480;

    cam_pos = laml::Vec3(0.0f, 0.0f, 2.0f);
    yaw = 0;
    pitch = 0;

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
    window = glfwCreateWindow(window_width, window_height, "Aimpoint", NULL, NULL);
    if (!window) {
        // Window or OpenGL context creation failed
        spdlog::critical("Failed to create GLFW Window");
        return 2;
    }

    glfwSetKeyCallback(window, glfw_key_callback);

    glfwSetWindowUserPointer(window, this);

    glfwMakeContextCurrent(window);
    gladLoadGL(glfwGetProcAddress);

    glfwGetFramebufferSize(window, &window_width, &window_height);
    glViewport(0, 0, window_width, window_height);

    glfwSwapInterval(2);

    glEnable(GL_DEPTH_TEST);

    // create gpu objects
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



    float vertices[] = {
        -0.5f, -0.5f, -0.5f,   0.0f, 0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,   0.0f, 0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,   0.0f, 0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,   0.0f, 0.0f, -1.0f,

         0.5f, -0.5f,  0.5f,   0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, 0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,   0.0f, 0.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,   0.0f, 0.0f, 1.0f,

        -0.5f,  0.5f,  0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  -1.0f, 0.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  -1.0f, 0.0f, 0.0f,

         0.5f,  0.5f,  0.5f,   1.0f, 0.0f, 0.0f,
         0.5f,  0.5f, -0.5f,   1.0f, 0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,   1.0f, 0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,   1.0f, 0.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,   0.0f, -1.0f, 0.0f,
         0.5f, -0.5f, -0.5f,   0.0f, -1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,   0.0f, -1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,   0.0f, -1.0f, 0.0f,

        -0.5f,  0.5f, -0.5f,   0.0f, 1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,   0.0f, 1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,   0.0f, 1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,   0.0f, 1.0f, 0.0f,
    };
    unsigned int indices[] = {  // note that we start from 0!
        0,  1,  2,  0,  2,  3,
        4,  5,  6,  6,  5,  7,
        9,  8, 10, 10,  8, 11,
       12, 13, 14, 12, 14, 15,
       17, 16, 18, 18, 16, 19,
       20, 21, 22, 20, 22, 23,
    };  

    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

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

    vao = VAO;

    spdlog::info("Application intitialized");
    return 0;
}

void aimpoint::step(double dt) {
    spdlog::trace("[{0:0.3f}] simulation step", sim_time);

    //int64 cycles_per_second = (int64)simulation_rate;
    //if (sim_frame % (cycles_per_second) == 0) {
    //    spdlog::info("t(s) = {0:4.1f}     alt(km) = {1:7.3f}     speed(m/s) = {2:7.3f}", 
    //                 sim_time, laml::length(body.state.position)/1000.0, laml::length(body.state.velocity));
    //}
    body.major_step(dt);
    body.integrate_states(sim_time, dt);
    
    sim_frame++;
}

void aimpoint::render() {
    glClearColor(0.2f, 0.4f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shader);

    float eye_4x4[16] = {1.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 1.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f,
                         0.0f, 0.0f, 0.0f, 1.0f};

    //laml::transform::create_projection_perspective(

    laml::Mat4 cam_transform, view_matrix;
    laml::transform::create_transform_translate(cam_transform, cam_pos);
    laml::transform::create_view_matrix_from_transform(view_matrix, cam_transform);
    int viewLocation = glGetUniformLocation(shader, "r_View");
    glUseProgram(shader);
    glUniformMatrix4fv(viewLocation, 1, GL_FALSE, view_matrix._data);

    laml::Mat4 projection_matrix;
    float AR = ((float)window_width / (float)window_height);
    laml::transform::create_projection_perspective(projection_matrix, 75.0f, AR, 0.1f, 100.0f);
    int projLocation = glGetUniformLocation(shader, "r_Projection");
    glUseProgram(shader);
    glUniformMatrix4fv(projLocation, 1, GL_FALSE, projection_matrix._data);

    laml::Mat4 transform_matrix;
    laml::transform::create_transform_rotation(transform_matrix, body.state.orientation);
    int transformLocation = glGetUniformLocation(shader, "r_Transform");
    glUseProgram(shader);
    glUniformMatrix4fv(transformLocation, 1, GL_FALSE, transform_matrix._data);

    glBindVertexArray(vao);
    //glDrawArrays(GL_TRIANGLES, 0, 6);
     glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

    //spdlog::trace("[{0:0.3f}] ({1:0.3f}) render step", sim_time, alpha);
    spdlog::trace("[{0:0.3f}] render step", sim_time);
}

void aimpoint::shutdown() {
    glfwDestroyWindow(window);

    glfwTerminate();

    spdlog::info("Application shutdown");
}

void aimpoint::key_callback(int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}



// GLFW Callbacks
void glfw_error_callback(int error, const char* description) {
    spdlog::error("[GLFW] Error {0}: {1}", error, description);
}

void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    aimpoint* app = (aimpoint*)glfwGetWindowUserPointer(window);
    app->key_callback(key, scancode, action, mods);
}












// Entry point
int main(int argc, char** argv) {
    aimpoint app;

    set_terminal_log_level(log_level::debug);
    spdlog::info("Creating application...");

    app.run();

    system("pause");
	
	return 0;
}