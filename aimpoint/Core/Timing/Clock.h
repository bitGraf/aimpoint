#pragma once

#include "Core/Core.h"

namespace aimpoint {

    class Clock {
    public:
        Clock();
        ~Clock();

        void Create(double dt, double start_time);
        void Advance();

        inline double GetTime() const { return cur_time; }
        inline double GetTimestep() const { return Ts; }

    private:
        double cur_time;
        double Ts;
    };
}