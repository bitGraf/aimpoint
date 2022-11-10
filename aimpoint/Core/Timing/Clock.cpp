#include "Clock.h"

namespace aimpoint {

    Clock::Clock() : Ts(-1), cur_time(0) {
    }

    Clock::~Clock() {
    }

    void Clock::Create(double dt, double start_time) {
        LOG_INFO("Clock created with a timestep of {0}", dt);
        LOG_INFO("Clock starting at time={0}", start_time);

        Ts = dt;
        cur_time = start_time;
    }

    void Clock::Advance() {
        cur_time += Ts;
    }
}