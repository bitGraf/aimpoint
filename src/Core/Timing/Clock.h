#pragma once

#include "Core/Core.h"

namespace aimpoint {

    class Clock {
    public:
        Clock();
        ~Clock();

        void Create(double dt, double start_time);
        void Advance();
        double GetTime() const;

    private:
        double cur_time;
        double Ts;
    };
}