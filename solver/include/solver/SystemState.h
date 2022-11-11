#pragma once

namespace ab_solver {

    // pos x,y,z
    // vel x,y,z
    // acc x,y,z
    // mass

    class SystemState {
    public:
        static const unsigned int NUM_STATE_VARS = (10);
    public:
        SystemState();
        ~SystemState();

        void Copy(const SystemState* state);
        void Resize(int n_bodies);
        void Destroy();

        double* translational;

        int num_bodies;
        double dt;
    };
}