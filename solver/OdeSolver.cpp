#include "OdeSolver.h"

namespace ab_solver {

    OdeSolver::OdeSolver() {
        m_dt = 0.0;
    }

    OdeSolver::~OdeSolver() {

    }

    void OdeSolver::Start(SystemState* initial, double dt) {
        m_dt = dt;
    }

    bool OdeSolver::Step(SystemState* state) {
        state->dt = m_dt;
        return true;
    }

    void OdeSolver::Solve(SystemState* state) {
        state->dt = m_dt;

        for (int i = 0; i < state->num_bodies; ++i) {
            int offset = i*state->NUM_STATE_VARS;

            state->translational[offset+0] += state->translational[offset+3] * m_dt;
            state->translational[offset+1] += state->translational[offset+4] * m_dt;
            state->translational[offset+2] += state->translational[offset+5] * m_dt;

            state->translational[offset+3] += state->translational[offset+6] * m_dt;
            state->translational[offset+4] += state->translational[offset+7] * m_dt;
            state->translational[offset+5] += state->translational[offset+8] * m_dt;
        }
    }

    void OdeSolver::End() {

    }

}