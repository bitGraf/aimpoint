#pragma once

#include "SystemState.h"

namespace ab_solver {

    class OdeSolver {
    public:
        OdeSolver();
        ~OdeSolver();

        void Start(SystemState* initial, double dt);
        bool Step(SystemState* state);
        void Solve(SystemState* state);
        void End();

    private:
        double m_dt;
    };
}