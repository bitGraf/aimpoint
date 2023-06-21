#include "defines.h"

#include "rocket.h"

// for recording
#define USE_DTV 0
#if USE_DTV
#include "dtv.h"
#endif

struct aimpoint {
public:
    int run();

    void key_callback(int key, int scancode, int action, int mods);

private:
    int init();
    void step(double dt);
    void render();
    void shutdown();

    bool real_time;

    double simulation_rate;
    uint64 sim_frame;

    double sim_time;
    double wall_time;

    int window_width, window_height;

    // openGL handles
    uint32 shader, vao;

    simulation_body body;

    laml::Vec3 cam_pos;
    float yaw, pitch;

    // video recording
#if USE_DTV
    atg_dtv::Encoder encoder;
#endif
    bool init_recording();
    bool stop_recording();
};