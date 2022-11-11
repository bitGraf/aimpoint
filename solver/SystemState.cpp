#include "SystemState.h"

#include <assert.h>
#include <cstring>
#include <stdio.h>

namespace ab_solver {

    SystemState::SystemState() {
        translational = nullptr;

        num_bodies = 0;
        dt = 0;
    }

    SystemState::~SystemState() {
        assert(num_bodies == 0);
    }

    void SystemState::Copy(const SystemState* state) {
        printf("Copying SystemState data [%d->%d]\n", state->num_bodies, num_bodies);
        Resize(state->num_bodies);

        if (state->num_bodies == 0) {
            return;
        }

        std::memcpy((void*)translational, (void*)state->translational, sizeof(double)*num_bodies*NUM_STATE_VARS);
    }

    void SystemState::Resize(int n_bodies) {
        if (num_bodies >= n_bodies) {
            return; // already big enough
        }

        printf("Resizing SystemState to %d bodies\n", n_bodies);

        Destroy();

        num_bodies = n_bodies;

        translational = new double[num_bodies*NUM_STATE_VARS];
    }

    void SystemState::Destroy() {
        printf("Destroying SystemState\n");
        if (num_bodies > 0) {
            delete[] translational;
            translational = nullptr;
        }

        num_bodies = 0;
    }

}